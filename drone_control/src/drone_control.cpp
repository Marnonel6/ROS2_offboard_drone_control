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

using namespace std::chrono;
using namespace std::chrono_literals;

// State machine states
enum class State {
    IDLE,
    OFFBOARD,
    MISSION,
    LAND,
    RTL,
    FAIL
};

class DroneControl : public rclcpp::Node
{
public:
    DroneControl() : Node("drone_control")
    {
        RCLCPP_INFO_STREAM(get_logger(), "Init Node");

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

        //
        // TODO waypoint should be read in from a .json
        //
        // Hover at 5[m]
        waypoint_0_.z = -5;
        // Add all waypoint to the vector
        waypoints_.push_back(waypoint_0_);

        // Initialize variables

        // Create publishers
        offboard_control_publisher_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ = create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10);

        // Create subscribers
        vehicle_odometry_subscriber_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", qos, std::bind(&DroneControl::vehicle_odometry_callback,
            this, std::placeholders::_1)
        );
        path_subscriber_ = create_subscription<nav_msgs::msg::Path>(
            "/f2c_path", qos, std::bind(&DroneControl::path_callback,
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
    State current_state_ = State::IDLE;      // Current state machine state
    // Vehicle position from fmu/out/vehicle_odometry -> Init to all zeros
    geometry_msgs::msg::Point vehicle_position_ = geometry_msgs::msg::Point{};
    std::vector<geometry_msgs::msg::Point> waypoints_;
    nav_msgs::msg::Path f2c_path_; // Fields2Cover path
    size_t global_i_ = 0; // global counter for f2c_path_
    double position_tolerance_ = 1.0; // [m]

    // Flags
    mutable std::mutex mutex_; // Used in the Non-block wait thread timer
    bool flag_timer_done_ = false;
    bool has_executed_ = false;  // Flag to check if the code has executed before
    bool flag_next_waypoint_ = true; // Send the next waypoint in path
    bool flag_mission_ = false; // Flag that turns true if a mission is ready

    //
    // TODO waypoint should be read in from a .json
    //
    geometry_msgs::msg::Point waypoint_0_ = geometry_msgs::msg::Point{};

    // Create objects
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber_;

    /**
     * @brief Vehicle odometry subscriber callback
     *
    */
    void vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::UniquePtr msg)
    {
        // Set vehicle position
        vehicle_position_.x = static_cast<double>(msg->position[0]);
        vehicle_position_.y = static_cast<double>(msg->position[1]);
        vehicle_position_.z = static_cast<double>(msg->position[2]);
    }

    /**
     * @brief Fields2Cover path from path_planning node
     *
    */
    void path_callback(const nav_msgs::msg::Path msg)
    {
        // Save Fields2Cover path
        f2c_path_ = msg;
        flag_mission_ = true;
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
     * @brief Publish a trajectory setpoint
     *        For this example, it sends a trajectory setpoint to make the
     *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
     * 
     * @param position - (x: North, y: East, z: Down)
     * @param velocity
     * @param yaw
     * 
     * Reference:
     *    Coordinate frame: https://docs.px4.io/main/en/ros/ros2_comm.html#ros-2-px4-frame-conventions
     */
    void publish_trajectory_setpoint(std::array<float, 3UL> position,
                                     std::array<float, 3UL> velocity,
                                     float yaw)
    {
        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.position = position; // (x: North, y: East, z: Down) - (Go up -z)
        msg.velocity = velocity;
        msg.acceleration = {0.1, 0.1, 0.1};
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
    double degreesToRadians(double degrees) {
        return degrees * (M_PI / 180.0);
    }
    ///
    ///
    /// LIBRARY functions later!!
    ///
    ///

    /// \brief Main control timer loop
    void timer_callback()
    {
        switch (current_state_)
        {
            // IDLE state -> ARM and Set OFFBOARD mode
            case State::IDLE:
                publish_offboard_control_mode();
                // Take-off and hover at 5[m]
                // TODO change to take-off 5m above CURRENT position
                publish_trajectory_setpoint({static_cast<float>(waypoints_.at(0).x), 
                                            static_cast<float>(waypoints_.at(0).y),
                                            static_cast<float>(waypoints_.at(0).z)},
                                            {0.1, 0.1, 0.1}, M_PI_2);

                if (offboard_setpoint_counter_ >= 200 && flag_mission_) {
                    // Change to off-board mode after 200 setpoints
                    // TODO WHY DO I HAVE TO DO THIS AND DO I NEED TO ADD THIS TO OTHERS
                    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

                    RCLCPP_INFO_STREAM(get_logger(), "Sent Vehicle Command");

                    // Arm the vehicle
                    arm();

                    // TODO Check if state transitioned to offboard and armed an only then change state
                    //      otherwise arm again

                    // Change state to OFFBOARD
                    current_state_ = State::OFFBOARD;
                    RCLCPP_INFO(get_logger(), "State transitioned to OFFBOARD");
                }

                // Increment setpoint counter
                offboard_setpoint_counter_++;
                break;
            // OFFBOARD state -> Take-off and hover at 5m
            case State::OFFBOARD:
                // Off-board control mode
                publish_offboard_control_mode();
                // Take-off and hover at 5[m]
                publish_trajectory_setpoint({static_cast<float>(waypoints_.at(0).x), 
                                            static_cast<float>(waypoints_.at(0).y),
                                            static_cast<float>(waypoints_.at(0).z)},
                                            {0.1, 0.1, 0.1}, M_PI_2);

                // Check if the setpoint has been reached in a specified tolerance
                if (reached_setpoint(waypoints_.at(0), vehicle_position_, 2.0))
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
                        RCLCPP_INFO(get_logger(), "State transitioned to MISSION with Fields2Cover path");

                        // Reset flags
                        flag_timer_done_ = false;
                        has_executed_ = false;
                    }
                }
                break;
            // Mission path with Fields2Cover
            case State::MISSION:
                // Off-board control mode
                publish_offboard_control_mode();
                // Loop through path and command drone to each point
                if (flag_next_waypoint_)
                {
                    // Publish path waypoint
                    tf2::Quaternion tf_q;
                    tf2::fromMsg(f2c_path_.poses.at(global_i_).pose.orientation, tf_q);
                    tf2::Matrix3x3 m(tf_q);
                    double roll, pitch, yaw_setpoint;
                    m.getRPY(roll, pitch, yaw_setpoint);
                    publish_trajectory_setpoint({static_cast<float>(f2c_path_.poses.at(global_i_).pose.position.x),
                                                static_cast<float>(f2c_path_.poses.at(global_i_).pose.position.y),
                                                static_cast<float>(f2c_path_.poses.at(global_i_).pose.position.z)},
                                                {0.1, 0.1, 0.1}, yaw_setpoint);
                    flag_next_waypoint_ = false;
                } else
                {
                    // Check if the setpoint has been reached in a specified tolerance
                    if (reached_setpoint(f2c_path_.poses.at(global_i_).pose.position, vehicle_position_,
                                        position_tolerance_))
                    {
                        // nonBlockingWait timer
                        if (!has_executed_)
                        {
                            nonBlockingWait(milliseconds(10));
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
                            if (global_i_ >= f2c_path_.poses.size())
                            {
                                current_state_ = State::RTL;
                                RCLCPP_INFO(get_logger(), "State transitioned to RTL");
                            }
                        }
                    }
                }
                break;
            // LAND at current position state
            case State::LAND:
                land();
                break;
            // Return to launch
            case State::RTL:
                RTL();
                break;
            // Emergency error state
            case State::FAIL:
                RTL();
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