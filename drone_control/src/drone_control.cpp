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
// ROS2 libs
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/point.hpp"
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
    MISSION1,
    MISSION2
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
        offboard_setpoint_counter_ = 0;
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

        // Create timer
        timer_ = create_wall_timer(milliseconds(1000 / frequency),
                                   std::bind(&DroneControl::timer_callback, this));
    }

private:
    // Variables
    size_t offboard_setpoint_counter_;   // Counter for the number of setpoints sent
    State current_state_;                // Current state machine state
    // Vehicle position from fmu/out/vehicle_odometry -> Init to all zeros
    geometry_msgs::msg::Point vehicle_position_ = geometry_msgs::msg::Point{};
    std::vector<geometry_msgs::msg::Point> waypoints_;

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

    ///
    /// \brief Publish the off-board control mode.
    ///        Only position and altitude controls are active.
    void publish_offboard_control_mode()
    {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
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
        // msg.acceleration = {0.0, 0.0, 0.0};
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
        std::cout << "Euclidean distance: " << std::abs(euclidean_distance(v1, v2)) << std::endl;
        return std::abs(euclidean_distance(v1, v2)) <= tolerance;
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
            publish_trajectory_setpoint({0.0, 0.0, -5.0}, {1.0, 1.0, 1.0}, M_PI_2);

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
            publish_trajectory_setpoint({0.0, 0.0, -5.0}, {1.0, 1.0, 1.0}, M_PI_2);

            // Check if the setpoint has been reached in a specified tolerance
            if (reached_setpoint(waypoints_.at(0), vehicle_position_, 2.0))
            {
                // Change state to MISSION1
                current_state_ = State::MISSION1;
                RCLCPP_INFO(get_logger(), "State transitioned to MISSION1");
                // TODO NOTE It moves very fast when it reaches the waypoint add a delay or waypoint stay timer
            }
        }
        // MISSION1 state
        else if (current_state_ == State::MISSION1)
        {
            // Off-board control mode
            publish_offboard_control_mode();
            // Take-off and hover at 5[m]
            publish_trajectory_setpoint({0.0, 5.0, -5.0}, {1.0, 1.0, 1.0}, M_PI_2);

            // Check if the setpoint has been reached in a specified tolerance
            if (reached_setpoint(waypoints_.at(1), vehicle_position_, 2.0))
            {
                // Change state to MISSION1
                current_state_ = State::MISSION2;
                RCLCPP_INFO(get_logger(), "State transitioned to MISSION2");
            }

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