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
    MISSION1,
    MISSION2,
    MISSION3,
    MISSION4,
    LAND
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
        // Up
        waypoint_0_.z = -5;
        // Forward
        waypoint_1_.y = 5;
        waypoint_1_.z = -5;
        // Left
        waypoint_2_.x = 5;
        waypoint_2_.y = 5;
        waypoint_2_.z = -5;
        // Back
        waypoint_3_.x = 5;
        waypoint_3_.z = -5;
        // Home
        waypoint_4_.z = -5;
        // Add all waypoint to the vector
        waypoints_.push_back(waypoint_0_);
        waypoints_.push_back(waypoint_1_);
        waypoints_.push_back(waypoint_2_);
        waypoints_.push_back(waypoint_3_);
        waypoints_.push_back(waypoint_4_);

        // Initialize variables
        current_state_ = State::IDLE;
        RCLCPP_INFO(get_logger(), "State transitioned to IDLE");

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
    State current_state_;                    // Current state machine state
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

    //
    // TODO waypoint should be read in from a .json
    //
    geometry_msgs::msg::Point waypoint_0_ = geometry_msgs::msg::Point{};
    geometry_msgs::msg::Point waypoint_1_ = geometry_msgs::msg::Point{};
    geometry_msgs::msg::Point waypoint_2_ = geometry_msgs::msg::Point{};
    geometry_msgs::msg::Point waypoint_3_ = geometry_msgs::msg::Point{};
    geometry_msgs::msg::Point waypoint_4_ = geometry_msgs::msg::Point{};

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
        // std::cout << "\n\n\n";
        // std::cout << "RECEIVED VEHICLE ODOMETRY DATA"   << std::endl;
        // std::cout << "=============================="   << std::endl;
        // std::cout << "timestamp: " << msg->timestamp    << std::endl;
        // std::cout << "position[0] North: " << msg->position[0]  << std::endl;
        // std::cout << "position[1] East: " << msg->position[1]  << std::endl;
        // std::cout << "position[2] Down: " << msg->position[2]  << std::endl;

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
    }

    ///
    /// \brief Publish the off-board control mode.
    ///        Only position and altitude controls are active.
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

    ///
    /// \brief Send a command to Arm the drone
    ///
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
     * @brief Send a command to Land the vehicle
     */
    void land()
    {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
        RCLCPP_INFO(get_logger(), "Land command send");
    }

    /**
     * @brief Publish vehicle commands
     * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
     * @param param1    Command parameter 1
     * @param param2    Command parameter 2
     */
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0)
    {
        px4_msgs::msg::VehicleCommand msg{};
        msg.param1 = param1;
        msg.param2 = param2;
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
        // IDLE state
        if (current_state_ == State::IDLE)
        {
            publish_offboard_control_mode();
            // Take-off and hover at 5[m]
            publish_trajectory_setpoint({static_cast<float>(waypoints_.at(0).x), 
                                         static_cast<float>(waypoints_.at(0).y),
                                         static_cast<float>(waypoints_.at(0).z)},
                                         {0.1, 0.1, 0.1}, M_PI_2);

            if (offboard_setpoint_counter_ >= 200) {
                // Change to off-board mode after 200 setpoints
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
        }
        // OFFBOARD state
        else if (current_state_ == State::OFFBOARD)
        {
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
                    // Change state to MISSION1
                    // current_state_ = State::MISSION1;
                    // RCLCPP_INFO(get_logger(), "State transitioned to MISSION1");
                    current_state_ = State::MISSION;
                    RCLCPP_INFO(get_logger(), "State transitioned to MISSION with Fields2Cover path");

                    // Reset flags
                    flag_timer_done_ = false;
                    has_executed_ = false;
                }
            }
        }
        // Mission path with Fields2Cover
        else if (current_state_ == State::MISSION)
        {
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
                // RCLCPP_INFO(get_logger(), "x: %f     y: %f     z: %f", static_cast<float>(f2c_path_.poses.at(global_i_).pose.position.x),
                //                                                        static_cast<float>(f2c_path_.poses.at(global_i_).pose.position.y),
                //                                                        static_cast<float>(f2c_path_.poses.at(global_i_).pose.position.z));
                // Do not send next waypoint until the waypoint have been reached
                flag_next_waypoint_ = false;
            } else
            {
                // Check if the setpoint has been reached in a specified tolerance
                // TODO CHECK IF YAW POSITION IS CLOSE ENOUGH AS WELL
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
                        // RCLCPP_INFO(get_logger(), static_cast<char>(global_i_));

                        // Reset flags
                        flag_timer_done_ = false;
                        has_executed_ = false;
                        flag_next_waypoint_ = true;
                        global_i_++; // Increment waypoint number

                        // Land when path is done
                        if (global_i_ >= f2c_path_.poses.size())
                        {
                            current_state_ = State::LAND;
                            RCLCPP_INFO(get_logger(), "State transitioned to LAND");
                        }
                    }
                }
            }
        }


        // MISSION1 state
        else if (current_state_ == State::MISSION1)
        {
            // Off-board control mode
            publish_offboard_control_mode();
            // Move 5[m] forward
            publish_trajectory_setpoint({static_cast<float>(waypoints_.at(1).x),
                                         static_cast<float>(waypoints_.at(1).y),
                                         static_cast<float>(waypoints_.at(1).z)},
                                         {0.1, 0.1, 0.1}, M_PI_2);

                                        //  static_cast<float>(test_counter_/100.0)

            // Increment counter
            test_counter_++;

            // Check if the setpoint has been reached in a specified tolerance
            if (reached_setpoint(waypoints_.at(1), vehicle_position_, 2.0))
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
                    // Change state to MISSION2
                    current_state_ = State::MISSION2;
                    RCLCPP_INFO(get_logger(), "State transitioned to MISSION2");

                    // Reset flags
                    flag_timer_done_ = false;
                    has_executed_ = false;
                }
            }
        }
        // MISSION2 state
        else if (current_state_ == State::MISSION2)
        {
            // Off-board control mode
            publish_offboard_control_mode();
            // Move 5[m] left
            publish_trajectory_setpoint({static_cast<float>(waypoints_.at(2).x), 
                                         static_cast<float>(waypoints_.at(2).y),
                                         static_cast<float>(waypoints_.at(2).z)},
                                         {0.1, 0.1, 0.1}, M_PI_2);

            // Check if the setpoint has been reached in a specified tolerance
            if (reached_setpoint(waypoints_.at(2), vehicle_position_, 2.0))
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
                    // Change state to MISSION2
                    current_state_ = State::MISSION3;
                    RCLCPP_INFO(get_logger(), "State transitioned to MISSION3");

                    // Reset flags
                    flag_timer_done_ = false;
                    has_executed_ = false;
                }
            }
        }
        // MISSION3 state
        else if (current_state_ == State::MISSION3)
        {
            // Off-board control mode
            publish_offboard_control_mode();
            // Move 5[m] back
            publish_trajectory_setpoint({static_cast<float>(waypoints_.at(3).x), 
                                         static_cast<float>(waypoints_.at(3).y),
                                         static_cast<float>(waypoints_.at(3).z)},
                                         {0.1, 0.1, 0.1}, M_PI_2);

            // Check if the setpoint has been reached in a specified tolerance
            if (reached_setpoint(waypoints_.at(3), vehicle_position_, 2.0))
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
                    // Change state to MISSION4
                    current_state_ = State::MISSION4;
                    RCLCPP_INFO(get_logger(), "State transitioned to MISSION4");

                    // Reset flags
                    flag_timer_done_ = false;
                    has_executed_ = false;
                }
            }
        }
        // MISSION4 state
        else if (current_state_ == State::MISSION4)
        {
            // Off-board control mode
            publish_offboard_control_mode();
            // Move 5[m] right (Back home)
            publish_trajectory_setpoint({static_cast<float>(waypoints_.at(4).x), 
                                         static_cast<float>(waypoints_.at(4).y),
                                         static_cast<float>(waypoints_.at(4).z)},
                                         {0.1, 0.1, 0.1}, M_PI_2);

            // Check if the setpoint has been reached in a specified tolerance
            if (reached_setpoint(waypoints_.at(4), vehicle_position_, 2.0))
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
                    // Change state to LAND
                    current_state_ = State::LAND;
                    RCLCPP_INFO(get_logger(), "State transitioned to LAND");

                    // Reset flags
                    flag_timer_done_ = false;
                    has_executed_ = false;
                }
            }
        }
        // LAND state
        else if (current_state_ == State::LAND)
        {
            // TODO actually add code for landing noo it will just timeout as I'm not sending
            // off-board control anymore.
            RCLCPP_INFO(get_logger(), "LANDING ...");
            land();
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