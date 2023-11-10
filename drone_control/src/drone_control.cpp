/**
 * Drone control high level state machine
 * 
*/
// Standard libs
#include <stdint.h>
#include <chrono>
#include <iostream>
#include <cstdio>
#include <math.h>
#include <vector>
#include <thread>
#include <mutex>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
// ROS2 libs
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/path.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// PX4 libs
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_ros_com/frame_transforms.h>

using namespace std::chrono;
using namespace std::chrono_literals;

// State machine states
enum class State {
    PREFLIGHT,
    IDLE,
    OFFBOARD,
    MISSION,
    LAND,
    RTL,
    FAIL,
    LIMBO,
    ERROR
};

class DroneControl : public rclcpp::Node
{
public:
    DroneControl() : Node("drone_control")
    {
        RCLCPP_INFO_STREAM(get_logger(), "Init Node");
        RCLCPP_INFO_STREAM(get_logger(), "State transitioned to PREFLIGHT");

        // QoS settings
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        // Parameter description
        auto frequency_des = rcl_interfaces::msg::ParameterDescriptor{};
        frequency_des.description = "Timer callback frequency [Hz]";
        // Declare default parameters values
        declare_parameter("frequency", 100, frequency_des); // Hz for timer_callback
        // Get params - Read params from yaml file that is passed in the launch file
        int frequency = get_parameter("frequency").get_parameter_value().get<int>();

        // Create publishers
        offboard_control_publisher_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ = create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10);
        home_position_publisher_ = create_publisher<geometry_msgs::msg::Point>(
            "/home_position", 10);

        // Create subscribers
        vehicle_odometry_subscriber_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", qos, std::bind(&DroneControl::vehicle_odometry_callback,
            this, std::placeholders::_1)
        );
        vehicle_control_mode_subscriber_ = create_subscription<px4_msgs::msg::VehicleControlMode>(
            "/fmu/out/vehicle_control_mode", qos,
            std::bind(&DroneControl::vehicle_control_mode_callback,
            this, std::placeholders::_1)
        );
        path_subscriber_ = create_subscription<nav_msgs::msg::Path>(
            "/f2c_path", qos, std::bind(&DroneControl::path_callback,
            this, std::placeholders::_1)
        );
        vehicle_pose_subscriber_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/vehicle_pose", qos, std::bind(&DroneControl::pose_callback,
            this, std::placeholders::_1)
        );

        // Create timer
        timer_ = create_wall_timer(milliseconds(1000 / frequency),
                                   std::bind(&DroneControl::timer_callback, this));
    }

private:
    // Variables
    size_t offboard_setpoint_counter_ = 0;   // Counter for the number of setpoints sent
    float test_counter_ = 0;                 // TODO delete
    State current_state_ = State::PREFLIGHT;      // Current state machine state
    // Vehicle position from fmu/out/vehicle_odometry -> Init to all zeros
    geometry_msgs::msg::Point vehicle_position_ = geometry_msgs::msg::Point{};
    std::vector<geometry_msgs::msg::Point> waypoints_;
    nav_msgs::msg::Path f2c_path_ros_; // Fields2Cover path in ROS coordinates frame
    px4_msgs::msg::VehicleControlMode vehicle_status_; // Drone status flags
    size_t global_i_ = 0; // global counter for f2c_path_ros_
    double position_tolerance_ = 0.5; // [m]
    geometry_msgs::msg::Point take_off_waypoint = geometry_msgs::msg::Point{}; // In ROS coordinates
    geometry_msgs::msg::Point home_position_ros_ = geometry_msgs::msg::Point{}; // In ROS coordinates
    double drone_yaw_ros_;
    double take_off_heading_;
    float take_off_height = 3.0; // Take-off height [m]
    geometry_msgs::msg::Point vehicle_position_ros_ = geometry_msgs::msg::Point{};
    geometry_msgs::msg::Point path_moved_to_drone_local_coordinates_ = geometry_msgs::msg::Point{};
    geometry_msgs::msg::Point velocity_setpoint_ = geometry_msgs::msg::Point{};

    // Flags
    mutable std::mutex mutex_; // Used in the Non-block wait thread timer
    bool flag_timer_done_ = false;
    bool has_executed_ = false;  // Flag to check if the code has executed before
    bool flag_next_waypoint_ = true; // Send the next waypoint in path
    bool flag_mission_ = false; // Flag that turns true if a mission is ready
    bool flag_vehicle_odometry_ = false; // Wait until vehicle odometry is available
    bool flag_take_off_position = false; // Set take-off position once

    // Create objects
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr home_position_publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr vehicle_control_mode_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vehicle_pose_subscriber_;

    /**
     * @brief Vehicle odometry subscriber callback
     *
    */
    void vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::UniquePtr msg)
    {
        // TODO do not use this anymore just use pose_callback() topic:/vehicle_pose
        // Convert from PX4 to ROS coordinates
        // Position
        Eigen::Vector3d px4_ned;
        px4_ned << static_cast<double>(msg->position[0]),
                   static_cast<double>(msg->position[1]),
                   static_cast<double>(msg->position[2]);
        Eigen::Vector3d ros_enu;
        ros_enu = px4_ros_com::frame_transforms::ned_to_enu_local_frame(px4_ned);
        // Set vehicle position
        vehicle_position_.x = ros_enu(0);
        vehicle_position_.y = ros_enu(1);
        vehicle_position_.z = ros_enu(2);
    }

    /**
     * @brief Vehicle control mode subscriber callback
     *
    */
    void vehicle_control_mode_callback(const px4_msgs::msg::VehicleControlMode::UniquePtr msg)
    {
        // Save vehicle status
        vehicle_status_ = *msg;
    }

    /**
     * @brief Fields2Cover path from path_planning node
     *
    */
    void path_callback(const nav_msgs::msg::Path msg)
    {
        // Save Fields2Cover path
        f2c_path_ros_ = msg;
        flag_mission_ = true;
    }

    /**
     * @brief Vehicle pose subscriber callback
     *
    */
    void pose_callback(const geometry_msgs::msg::PoseStamped::UniquePtr msg)
    {
        // Save vehicle position
        vehicle_position_ros_ = msg->pose.position;

        // Drone take-off heading from quaternion
        tf2::Quaternion tf_q;
        tf2::fromMsg(msg->pose.orientation, tf_q);
        tf2::Matrix3x3 m(tf_q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        drone_yaw_ros_ = yaw;
        flag_vehicle_odometry_ = true;
    }

    /**
     * \brief Publish the off-board control mode.
     *        Only position and altitude controls are active.
    */
    void publish_offboard_control_mode()
    {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.position = true;
        msg.velocity = true;
        msg.acceleration = true;
        msg.attitude = false;
        msg.body_rate = false;
        msg.timestamp = get_clock()->now().nanoseconds() / 1000;
        offboard_control_publisher_->publish(msg);
    }

    /**
     * @brief Publish a trajectory set point
     *        For this example, it sends a trajectory setpoint to make the
     *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
     * 
     * @param position - IN ROS COORDINATE SYSTEM - (x: North, y: East, z: Down)
     * @param velocity - Velocity vector at the set point position
     * @param yaw      - Desired yaw now
     * 
     * Reference:
     *    Coordinate frame: https://docs.px4.io/main/en/ros/ros2_comm.html#ros-2-px4-frame-conventions
     */
    void publish_trajectory_setpoint(std::array<float, 3UL> position,
                                     std::array<float, 3UL> velocity,
                                     float yaw)
    {

        // Convert from ROS to PX4 coordinates before publishing
        // Position
        Eigen::Vector3d ros_enu;
        ros_enu << position.at(0),
                   position.at(1),
                   position.at(2);
        Eigen::Vector3d px4_ned;
        px4_ned = px4_ros_com::frame_transforms::enu_to_ned_local_frame(ros_enu);
        position.at(0) = px4_ned(0);
        position.at(1) = px4_ned(1);
        position.at(2) = px4_ned(2);
        // Orientation
        Eigen::Quaterniond q_ros;
        q_ros = px4_ros_com::frame_transforms::utils::quaternion::quaternion_from_euler(0, 0, yaw);
        Eigen::Quaterniond q_px4;
        q_px4 = px4_ros_com::frame_transforms::enu_to_ned_orientation(q_ros);
        yaw = px4_ros_com::frame_transforms::utils::quaternion::quaternion_get_yaw(q_px4);

        // Create trajectory setpoint message and publish
        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.position = position; // (x: North, y: East, z: Down) - (Go up -z)
        msg.velocity = velocity;
        msg.acceleration = {0.0, 0.0, 0.0};
        msg.yaw = yaw; // [-PI:PI]
        msg.yawspeed = 0.174533; // [rad/s] -> 10 [Deg/s]
        msg.timestamp = get_clock()->now().nanoseconds() / 1000;
        trajectory_setpoint_publisher_->publish(msg);

        //     px4_msgs::msg::TrajectorySetpoint msg{};
        //     msg.timestamp = get_clock()->now().nanoseconds() / 1000;
        //     msg.x = 0.0;
        //     msg.y = 0.0;
        //     msg.z = 2.0;
        //     msg.yaw = 0.0;
        //     msg.yawspeed = 0.0;
        //     msg.vx = 0.0;
        //     msg.vy = 0.0;
        //     msg.vz = 0.0;
        //     msg.acceleration = 0.0;
        //     msg.acceleration_valid = false;
        //     msg.velocity_valid = false;
        //     msg.type = 0;
        //     msg.position_valid = true;
        //     msg.velocity_frame = 0;
    }

    /**
     * \brief Send a command to Arm the drone
    */
    void arm()
    {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
                                1.0);

        RCLCPP_INFO(get_logger(), "Arm command send");
    }

    /**
     * @brief Send a command to Disarm the vehicle
     */
    void disarm()
    {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

        RCLCPP_INFO(get_logger(), "Disarm command send");
    }

    /**
     * @brief Send take-off command
     * 
     * Takeoff from ground / hand
     * Command -> VEHICLE_CMD_NAV_TAKEOFF = 12
     * Parameters:
     * | Minimum pitch (if airspeed sensor present), desired pitch without sensor
     * | Empty
     * | Empty
     * | Yaw angle (if magnetometer present), ignored without magnetometer
     * | Latitude [Deg * 1e7]
     * | Longitude [Deg * 1e7]
     * | Altitude [mm above sea level] https://www.freemaptools.com/elevation-finder.htm
     * Reference for how Lat, Lon and Alt is calculated
     * https://docs.px4.io/v1.12/en/advanced_config/parameter_reference.html#data-link-loss
     * The latitude becomes approximately: 473,977,222
     * The longitude becomes approximately: 85,456,111
     * 
     * NOTE TODO NOT WORKING YET HAS PROBLEMS
    */
    void takeoff()
    {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, 0,0,0,
                               4.21, 473977222, 85456111, 570200); // 4.2, 473977445, 85455941, 488200);
        RCLCPP_INFO(get_logger(), "Takeoff command send");
    }

    /**
     * @brief Send a command to Land the vehicle
     */
    void land()
    {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
        RCLCPP_INFO(get_logger(), "Land command send");
    }

    /**
     * @brief Send Return to launch command
     * 
     * Return to launch location
     * Command -> VEHICLE_CMD_NAV_RETURN_TO_LAUNCH = 20
     * Parameters:
     * |Empty
     * |Empty
     * |Empty
     * |Empty
     * |Empty
     * |Empty
     * |Empty
    */
    void RTL()
    {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH);
        RCLCPP_INFO(get_logger(), "RTL command send");
    }

    /**
     * @brief Publish vehicle commands (https://github.com/PX4/PX4-Autopilot/blob/main/msg/VehicleCommand.msg)
     * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
     * @param param1    Parameter 1, as defined by MAVLink uint16 VEHICLE_CMD enum.
     * @param param2    Parameter 2, as defined by MAVLink uint16 VEHICLE_CMD enum.
     * @param param3    Parameter 3, as defined by MAVLink uint16 VEHICLE_CMD enum.
     * @param param4    Parameter 4, as defined by MAVLink uint16 VEHICLE_CMD enum.
     * @param param5    Parameter 5, as defined by MAVLink uint16 VEHICLE_CMD enum.
     * @param param6    Parameter 6, as defined by MAVLink uint16 VEHICLE_CMD enum.
     * @param param7    Parameter 7, as defined by MAVLink uint16 VEHICLE_CMD enum.
     */
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0,
                                 float param3 = 0.0, float param4 = 0.0, float param5 = 0.0,
                                 float param6 = 0.0, float param7 = 0.0)
    {
        px4_msgs::msg::VehicleCommand msg{};
        msg.param1 = param1;
        msg.param2 = param2;
        msg.param3 = param3;
        msg.param4 = param4;
        msg.param5 = param5;
        msg.param6 = param6;
        msg.param7 = param7;
        msg.command = command;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = get_clock()->now().nanoseconds() / 1000;
        vehicle_command_publisher_->publish(msg);
    }

    /**
     * @brief Non-blocking timer counter thread. This will change the flag: Flag_timer_done = True
     * @param duration Time to sleep in milliseconds
    */
    void nonBlockingWait(std::chrono::milliseconds duration)
    {
        std::thread([this, duration]() {
            std::this_thread::sleep_for(duration);

            // Shared data variable mutex is locked when the non_blocking_wait_thread is accessing
            // it and unlocked when the non_blocking_wait_thread is done accessing it. The {} help
            // with data syncronization and to ensure two thread do not change a shared variable
            // at the same time.
            {
                std::lock_guard<std::mutex> lock(mutex_);
                flag_timer_done_ = true;
            }

            // RCLCPP_INFO(rclcpp::get_logger("non_blocking_wait_thread"), "Waited for %ld seconds.", duration.count());
        }).detach();
    }

    ///
    ///
    /// LIBRARY functions later!!
    ///
    ///
    /**
     * @brief Calculate 3D Euclidean distance
     * @param v1 First 3D point
     * @param v2 Second 3D point
     * @return Euclidean distance
    */
    double euclidean_distance(const geometry_msgs::msg::Point v1,
                              const geometry_msgs::msg::Point v2)
    {
    return std::sqrt((v2.x - v1.x) * (v2.x - v1.x) +
                     (v2.y - v1.y) * (v2.y - v1.y) +
                     (v2.z - v1.z) * (v2.z - v1.z));
    }

    /**
     * @brief Checks if drone is at specified setpoint and within an Euclidean tolerance
     * @param v1 First 3D point
     * @param v2 Second 3D point
     * @param tolerance Acceptable Euclidean tolerance in meters
     * @return (bool) True if inside the Euclidean tolerance
    */
    bool reached_setpoint(const geometry_msgs::msg::Point v1,
                          const geometry_msgs::msg::Point v2,
                          double tolerance = 1.0)
    {
        // std::cout << "Euclidean distance: " << std::abs(euclidean_distance(v1, v2)) << std::endl;
        return std::abs(euclidean_distance(v1, v2)) <= tolerance;
    }

    /**
     * @brief Convert degrees to radians
     * @param degrees Angle in [Deg]
     * @return Angle in radians
    */
    double degreesToRadians(double degrees)
    {
        return degrees * (M_PI / 180.0);
    }
    ///
    ///
    /// LIBRARY functions later!!
    ///
    ///

    void check_pilot_state_switch()
    {
        // Check if drone mode has been switched by pilot then go to limbo state
        if (vehicle_status_.flag_control_offboard_enabled == false &&
            vehicle_status_.flag_control_auto_enabled == true)
            {
                // Change state to LIMBO
                current_state_ = State::LIMBO;
                RCLCPP_INFO(get_logger(), "Pilot switched state! State transitioned to LIMBO");
            }
    }

    /**
     * @brief Set take-off waypoint
     * 
     * Set take-off waypoint to current drone position
    */
    void set_take_off_waypoint()
    {
        // Only do this once
        if (!flag_take_off_position){
            // Set take-off waypoint in ROS coordinates
            take_off_waypoint.x = vehicle_position_.x;
            take_off_waypoint.y = vehicle_position_.y;
            take_off_waypoint.z = vehicle_position_.z + take_off_height;
            take_off_heading_ = drone_yaw_ros_;

            // Landing pad home position in ROS coordinates
            home_position_ros_.x = vehicle_position_.x;
            home_position_ros_.y = vehicle_position_.y;
            home_position_ros_.z = vehicle_position_.z;
        }
        flag_take_off_position = true;
    }

    /**
     * @brief Check if drone is at the startup position (Within a threshold)
     * 
     * @return (bool) True if drone is at startup position
    */
    bool check_drone_startup_position()
    {
        // Check if drone is at startup position
        float tolerance_xy = 5.0;
        float tolerance_z = 1.5;
        if (vehicle_position_ros_.x < tolerance_xy && vehicle_position_ros_.x > -tolerance_xy)
        {
            if (vehicle_position_ros_.y < tolerance_xy && vehicle_position_ros_.y > -tolerance_xy)
            {
                if (vehicle_position_ros_.z < tolerance_z && vehicle_position_ros_.z > -tolerance_z)
                {
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * @brief Calculate velocity setpoint
     * 
     * Calculate the velocity setpoint from the current position and the next waypoint
     * 
     * @return geometry_msgs::msg::Point Velocity setpoint
    */
    geometry_msgs::msg::Point calculate_velocity_setpoint()
    {
        // Calculate velocity setpoint
        geometry_msgs::msg::Point velocity_setpoint;
        // Check if the current path and the next n_steps is a swath by comparing the x values
        // If the x values are the same then the path is a swath
        // TODO Make this more general
        int n_steps = 10;
        // This gives a ground speed of 1 [m/s]
        double x_vel = 0.5; // NB PX4 x velocity is ROS2 path y anv vice versa
        if (global_i_ + n_steps < f2c_path_ros_.poses.size() && global_i_ >= 1)
        {
            if (f2c_path_ros_.poses.at(global_i_-1).pose.position.x ==
                f2c_path_ros_.poses.at(global_i_ + n_steps).pose.position.x)
            {
                // Determine if y velocity setpoint should be +/-
                if (f2c_path_ros_.poses.at(global_i_ - 1).pose.position.y <
                    f2c_path_ros_.poses.at(global_i_ + n_steps).pose.position.y)
                {
                    velocity_setpoint.x = x_vel;
                } else
                {
                    velocity_setpoint.x = -x_vel;
                }
                velocity_setpoint.y = 0.0; // Along swath no y-velocities
            }
        } else // Default velocity setpoint for all paths that are not swaths
        {
            velocity_setpoint.x = 0.1;
            velocity_setpoint.y = 0.1;
        }
        // z velocities are always the same in a mission.
        velocity_setpoint.z = 0.0;

        return velocity_setpoint;
    }

    /**
     * @brief Update the path to the take-off position
     * 
     * Moves the mission setpoint to the drone take-off position to ensure it is in
     * the correct frame as the drone does not always start at [0,0,0] - [x,y,z]
     * local coordinates, but the path/mission assume it does.
     * 
     * @param x Path x position
     * @param y Path y position
     * @param z Path z position
     * @return geometry_msgs::msg::Point Updated path to zero drone take-off position
    */
    geometry_msgs::msg::Point path_update_to_takeoff_position(double x, double y, double z)
    {
        // Update path
        geometry_msgs::msg::Point point;
        point.x = x + home_position_ros_.x;
        point.y = y + home_position_ros_.y;
        point.z = z + home_position_ros_.z;
        return point;
    }

    /// \brief Main control timer loop
    void timer_callback()
    {
        // Publish home_position
        if (flag_take_off_position)
        {
            // Minus hover/take-off height and save to home position variable
            home_position_publisher_->publish(home_position_ros_);
        }

        // State machine
        switch (current_state_)
        {
            // PREFLIGHT state -> Do preflight checklist
            case State::PREFLIGHT:
                // Wait for vehicle odometry
                if (flag_vehicle_odometry_)
                {
                    // Check if drone is at startup position
                    if (check_drone_startup_position())
                    {
                        // Set take-off waypoint
                        set_take_off_waypoint();

                        // Change state to IDLE
                        current_state_ = State::IDLE;
                    }
                    else
                    {
                        // ROS stream error message with drone x, y, z position
                        RCLCPP_ERROR_STREAM(get_logger(), "Drone is not at startup position! x: " <<
                                            vehicle_position_ros_.x << " y: " << vehicle_position_ros_.y <<
                                            " z: " << vehicle_position_ros_.z);
                    }
                }
                break;
            // IDLE state -> ARM and Set OFFBOARD mode
            case State::IDLE:
                // Off-board control mode
                publish_offboard_control_mode();
                // Take-off and hover at 5[m] at current drone position
                publish_trajectory_setpoint({static_cast<float>(take_off_waypoint.x),
                                             static_cast<float>(take_off_waypoint.y),
                                             static_cast<float>(take_off_waypoint.z)},
                                             {0.1, 0.1, 0.1}, static_cast<float>(take_off_heading_));

                if (offboard_setpoint_counter_ >= 200 && flag_mission_) {
                    // Change to off-board mode after 200 setpoints
                    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

                    RCLCPP_INFO_STREAM(get_logger(), "Sent Vehicle Command");

                    // Arm the vehicle
                    arm();

                    // Check if drone/px4 state transitioned to offboard and that the drone is armed
                    if (vehicle_status_.flag_armed == true &&
                        vehicle_status_.flag_control_offboard_enabled == true)
                    {
                        // Change state to OFFBOARD
                        current_state_ = State::OFFBOARD;
                        RCLCPP_INFO(get_logger(), "State transitioned to OFFBOARD");
                    }
                }

                // Increment setpoint counter
                offboard_setpoint_counter_++;
                break;
            // OFFBOARD state -> Take-off and hover at 5m
            case State::OFFBOARD:
                // Check if drone mode has been switched by pilot then go to limbo state
                check_pilot_state_switch();

                // Off-board control mode
                publish_offboard_control_mode();
                // Take-off and hover at 5[m] at current drone position
                publish_trajectory_setpoint({static_cast<float>(take_off_waypoint.x), 
                                             static_cast<float>(take_off_waypoint.y),
                                             static_cast<float>(take_off_waypoint.z)},
                                             {0.0, 0.0, 0.0}, static_cast<float>(take_off_heading_));

                // Check if the setpoint has been reached in a specified tolerance
                if (reached_setpoint(take_off_waypoint, vehicle_position_, position_tolerance_))
                {
                    // nonBlockingWait timer
                    if (!has_executed_)
                    {
                        nonBlockingWait(seconds(5)); 
                        has_executed_ = true;
                    }

                    // Wait until nonBlockingWait is done
                    if (flag_timer_done_)
                    {
                        // Change state to MISSION
                        current_state_ = State::MISSION;
                        // current_state_ = State::LAND; // TODO Change back to MISSION
                        RCLCPP_INFO(get_logger(), "State transitioned to MISSION with Fields2Cover path");

                        // Reset flags
                        flag_timer_done_ = false;
                        has_executed_ = false;
                    }
                }
                break;
            // Mission path with Fields2Cover
            case State::MISSION:
                // Check if drone mode has been switched by pilot then go to limbo state
                check_pilot_state_switch();

                // Off-board control mode
                publish_offboard_control_mode();
                // Loop through path and command drone to each point
                if (flag_next_waypoint_)
                {
                    // Publish path waypoint
                    tf2::Quaternion tf_q;
                    tf2::fromMsg(f2c_path_ros_.poses.at(global_i_).pose.orientation, tf_q);
                    tf2::Matrix3x3 m(tf_q);
                    double roll, pitch, yaw_setpoint;
                    m.getRPY(roll, pitch, yaw_setpoint);
                    // Moves the mission setpoint to the drone take-off position to ensure it is in
                    // the correct frame as the drone does not always start at [0,0,0] - [x,y,z]
                    // local coordinates, but the path/mission assume it does.
                    path_moved_to_drone_local_coordinates_ = path_update_to_takeoff_position(
                            f2c_path_ros_.poses.at(global_i_).pose.position.x,
                            f2c_path_ros_.poses.at(global_i_).pose.position.y,
                            f2c_path_ros_.poses.at(global_i_).pose.position.z);
                    // Calculate velocity setpoint
                    velocity_setpoint_ = calculate_velocity_setpoint();
                    // velocity_setpoint_.x = 0.1;
                    // velocity_setpoint_.y = 0.1;
                    // velocity_setpoint_.z = 0.0;
                    // Publish set_point
                    publish_trajectory_setpoint({static_cast<float>(path_moved_to_drone_local_coordinates_.x),
                                                 static_cast<float>(path_moved_to_drone_local_coordinates_.y),
                                                 static_cast<float>(path_moved_to_drone_local_coordinates_.z)},
                                                {static_cast<float>(velocity_setpoint_.x),
                                                 static_cast<float>(velocity_setpoint_.y),
                                                 static_cast<float>(velocity_setpoint_.z)},
                                                yaw_setpoint);
                    flag_next_waypoint_ = false;
                } else
                {
                    // Check if the setpoint has been reached in a specified tolerance
                    // if (reached_setpoint(f2c_path_ros_.poses.at(global_i_).pose.position, vehicle_position_,
                    //                     position_tolerance_))
                    if (reached_setpoint(path_moved_to_drone_local_coordinates_, vehicle_position_,
                                         position_tolerance_))
                    {
                        // nonBlockingWait timer
                        if (!has_executed_)
                        {
                            nonBlockingWait(milliseconds(1));
                            has_executed_ = true;
                        }

                        // Wait until nonBlockingWait is done
                        if (flag_timer_done_)
                        {
                            // Reset flags
                            flag_timer_done_ = false;
                            has_executed_ = false;
                            flag_next_waypoint_ = true;
                            global_i_++; // Increment waypoint number

                            // Land when path is done
                            if (global_i_ >= f2c_path_ros_.poses.size())
                            {
                                current_state_ = State::RTL; // NOTE Was LAND
                                RCLCPP_INFO(get_logger(), "State transitioned to LAND");
                            }
                        }
                    }
                }
                break;
            // LAND at current position state
            case State::LAND:
                land();
                // Check if drone mode has switched to Land mode
                if (vehicle_status_.flag_control_offboard_enabled == false &&
                    vehicle_status_.flag_control_auto_enabled == true)
                    {
                        // Change state to LIMBO
                        current_state_ = State::LIMBO;
                        RCLCPP_INFO(get_logger(), "State transitioned to LIMBO");
                    }
                break;
            // Return to launch
            case State::RTL:
                RTL();
                // Check if drone mode has switched to RTL mode
                if (vehicle_status_.flag_control_offboard_enabled == false &&
                    vehicle_status_.flag_control_auto_enabled == true)
                    {
                        // Change state to LIMBO
                        current_state_ = State::LIMBO;
                        RCLCPP_INFO(get_logger(), "State transitioned to LIMBO");
                    }
                break;
            // Emergency error state
            case State::FAIL:
                current_state_ = State::LAND;
                break;
            // Limbo state
            case State::LIMBO:
                // Stuck in LIMBO -> Do nothing
                break;
            // Default state
            default:
                RCLCPP_INFO(get_logger(), "------ Default state ------");
                break;
        }
    }
};

int main(int argc, char *argv[])
{
    // Sets the standard output to unbuffered mode, which means any data written to stdout will be
    // written immediately without waiting for the buffer to fill up. Less efficient if a lot is
    // printed, but get faster prints/output.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneControl>());

    rclcpp::shutdown();
    return 0;
}



// void StartWork()
// {
//     if (current_state_ == State::Idle) {
//         current_state_ = State::Working;
//         RCLCPP_INFO(this->get_logger(), "State transitioned to Working");
//     }
// }

// void Fail()
// {
//     if (current_state_ == State::Working) {
//         current_state_ = State::Error;
//         RCLCPP_INFO(this->get_logger(), "State transitioned to Error");
//     }
// }

// void Reset()
// {
//     if (current_state_ == State::Error) {
//         current_state_ = State::Idle;
//         RCLCPP_INFO(this->get_logger(), "State transitioned to Idle");
//     }
// }