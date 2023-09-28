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
// ROS2 libs
#include <rclcpp/rclcpp.hpp>
// PX4 libs
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

using namespace std::chrono;
using namespace std::chrono_literals;

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

        // Initialize variables
        offboard_setpoint_counter_ = 0;

    }

private:
    // Variables
    size_t offboard_setpoint_counter_;   // Counter for the number of setpoints sent

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
        std::cout << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n";
        std::cout << "RECEIVED VEHICLE ODOMETRY DATA"   << std::endl;
        std::cout << "=============================="   << std::endl;
        std::cout << "timestamp: " << msg->timestamp    << std::endl;
        std::cout << "position[0]: " << msg->position[0]  << std::endl;
        std::cout << "position[1]: " << msg->position[1]  << std::endl;
        std::cout << "position[2]: " << msg->position[2]  << std::endl;
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
     * @param position 
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
        msg.position = position; // (Go up -z)
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

    /// \brief Main control timer loop
    void timer_callback()
    {
        publish_offboard_control_mode();
        // Take-off and hover at 5[m]
        publish_trajectory_setpoint({0.0, 0.0, -5.0}, {1.0, 1.0, 1.0}, M_PI_2);

        if (offboard_setpoint_counter_ == 200) {
            // Change to off-board mode after 200 setpoints
            publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

            RCLCPP_INFO_STREAM(get_logger(), "Sent Vehicle Command");

            // Arm the vehicle
            arm();
        }

        // If at 5m then move forward


        // Increment setpoint counter
        offboard_setpoint_counter_++;
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