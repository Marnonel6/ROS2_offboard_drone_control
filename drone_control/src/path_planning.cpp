/**
 * ROS2 path planning node utilizing the packages Fields2Cover
 * 
 * Reference:
 *     https://github.com/Fields2Cover/Fields2Cover
 * 
*/
// Standard libs
#include <stdint.h>
#include <iostream>
#include <cstdio>
#include <math.h>
#include <vector>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
// ROS2 libs
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// Import lib for motion planning package
#include "fields2cover.h"
// PX4 libs
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_ros_com/frame_transforms.h>

// State machine states
enum class State {
    IDLE,
    PATH_PLANNING,
    PUB_PATH,
    RESET,
    FAIL
};

class PathPlanning : public rclcpp::Node
{
public:
    PathPlanning() : Node("path_planning")
    {
        RCLCPP_INFO_STREAM(get_logger(), "Init Node");

        // QoS settings
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        // // Parameter description
        // auto frequency_des = rcl_interfaces::msg::ParameterDescriptor{};
        // frequency_des.description = "Timer callback frequency [Hz]";
        // // Declare default parameters values
        // declare_parameter("frequency", 100, frequency_des); // Hz for timer_callback
        // // Get params - Read params from yaml file that is passed in the launch file
        // int frequency = get_parameter("frequency").get_parameter_value().get<int>();
        // TODO NOTE Set field, drone parameters from a .json file

        // Initialize variables

        // Create publishers
        path_publisher_ = create_publisher<nav_msgs::msg::Path>("/f2c_path", 10);
        vehicle_pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>("/vehicle_pose",
                                                                                    10);

        // Create subscribers
        vehicle_odometry_subscriber_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", qos, std::bind(&PathPlanning::vehicle_odometry_callback,
            this, std::placeholders::_1)
        );

        // Create main timer
        timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&PathPlanning::timer_callback, this));
    }

private:
    // Variables
    State current_state_ = State::IDLE;      // Current state machine state
    F2CPath f2c_path_;
    nav_msgs::msg::Path path_;
    // Vehicle pose from fmu/out/vehicle_odometry
    geometry_msgs::msg::PoseStamped vehicle_pose_px4_;
    geometry_msgs::msg::PoseStamped vehicle_pose_ros_;
    geometry_msgs::msg::Pose home_pose_ = geometry_msgs::msg::Pose{};
    geometry_msgs::msg::Pose hover_home_pose_ = geometry_msgs::msg::Pose{}; // 5[m] above home pose

    // Flags
    bool flag_vehicle_odometry_ = false;

    // Create objects
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vehicle_pose_publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_subscriber_;

    /**
     * @brief Plan full path
     *
    */
    F2CPath path_planning()
    {
        // Use Fields2Cover
        // Define field and robot
        F2CRobot robot (2.0, 4.0);
        // NOTE: The z-height that is specified here gets halved in the path for some reason
        F2CCells field(F2CCell(F2CLinearRing({F2CPoint(5,5,10), F2CPoint(5,15,10), F2CPoint(15,15,10),
                                              F2CPoint(15,5,10), F2CPoint(5,5,10)})));
        // Swath generation
        f2c::sg::BruteForce bf;
        f2c::obj::NSwath n_swath_obj;
        F2CSwaths swaths = bf.generateSwaths(M_PI, robot.op_width, field.getGeometry(0));
        f2c::rp::BoustrophedonOrder boustrophedon_sorter;
        auto boustrophedon_swaths = boustrophedon_sorter.genSortedSwaths(swaths, 1);
        // Path planner
        f2c::pp::PathPlanning path_planner;
        robot.setMinRadius(2.0);  // m
        // Create swath connections using Dubins curves with Continuous curvature
        f2c::pp::DubinsCurvesCC dubins_cc;
        F2CPath path_dubins_cc = path_planner.searchBestPath(robot, boustrophedon_swaths, dubins_cc);
        // Discretize the turns -> Specify significant number precision
        path_dubins_cc.serializePath(3);
        // Discretize swath lines in path object -> Specify the step size for the swath section
        double discretize_step_size = 0.5; // Step size for discretization in [m]
        F2CPath new_path = path_dubins_cc.discretize_swath(discretize_step_size);
        // Save to file
        // new_path.saveToFile("discretized_swath_path.csv", 3); // Specify precision to the significant number
        // Visualize
        f2c::Visualizer::figure();
        f2c::Visualizer::plot(field);
        f2c::Visualizer::plot(new_path);
        f2c::Visualizer::plot(boustrophedon_swaths);
        f2c::Visualizer::show();

        return new_path;
    }

    /**
     * @brief Convert F2CPath to nav_msgs::msg::path
     *        (https://fields2cover.github.io/api/structf2c_1_1types_1_1PathState.html#_CPPv4N3f2c5types9PathStateE)
     * @param f2c_path Fields2Cover path to convert
     * @param path nav_msgs::msg::Path variable to save to
     * @return void
    */
    void f2cpath_to_navpath(nav_msgs::msg::Path& path, F2CPath f2c_path)
    {
        // Loop through all points in the path
        for (size_t i = 0; i < f2c_path.size(); i++)
        {
            // F2CPoint -> geometry_msgs::msg::Point
            geometry_msgs::msg::Point point;
            point.x = static_cast<float>(f2c_path.states.at(i).point.getX());
            point.y = static_cast<float>(f2c_path.states.at(i).point.getY());
            point.z = static_cast<float>(f2c_path.states.at(i).point.getZ());

            // Yaw (Radians) -> tf2::Quaternion
            tf2::Quaternion tf2_q;
            tf2_q.setRPY(0, 0, f2c_path.states.at(i).angle);
            // Convert tf2::Quaternion to geometry_msgs::msg::Quaternion
            geometry_msgs::msg::Quaternion quaternion = tf2::toMsg(tf2_q);

            // Create pose stamped
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "/map";
            pose.header.stamp = rclcpp::Node::now();
            pose.pose.position = point;
            pose.pose.orientation = quaternion;

            // Add point to path
            path.poses.push_back(pose);
        }
    }

    /**
     * @brief Callback function for vehicle_odometry subscriber
     * @param msg px4_msgs::msg::VehicleOdometry message
     * @return void
    */
    void vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::UniquePtr msg)
    {
        // Get vehicle position in PX4 coordinates
        vehicle_pose_px4_.header.frame_id = "/map";
        vehicle_pose_px4_.header.stamp = rclcpp::Node::now();
        vehicle_pose_px4_.pose.position.x = static_cast<double>(msg->position[0]);
        vehicle_pose_px4_.pose.position.y = static_cast<double>(msg->position[1]);
        vehicle_pose_px4_.pose.position.z = static_cast<double>(msg->position[2]);
        vehicle_pose_px4_.pose.orientation.x = static_cast<double>(msg->q[1]);
        vehicle_pose_px4_.pose.orientation.y = static_cast<double>(msg->q[2]);
        vehicle_pose_px4_.pose.orientation.z = static_cast<double>(msg->q[3]);
        vehicle_pose_px4_.pose.orientation.w = static_cast<double>(msg->q[0]);

        // Convert from PX4 to ROS coordinates
        // To display in rviz the vehicle pose with -> vehicle_pose_ros_
        vehicle_pose_ros_.header.frame_id = "/map";
        vehicle_pose_ros_.header.stamp = rclcpp::Node::now();
        // Convert Position
        Eigen::Vector3d px4_ned;
        px4_ned << vehicle_pose_px4_.pose.position.x, vehicle_pose_px4_.pose.position.y,
                   vehicle_pose_px4_.pose.position.z;
        Eigen::Vector3d ros_enu;
        ros_enu = px4_ros_com::frame_transforms::ned_to_enu_local_frame(px4_ned);
        vehicle_pose_ros_.pose.position.x = ros_enu(0);
        vehicle_pose_ros_.pose.position.y = ros_enu(1);
        vehicle_pose_ros_.pose.position.z = ros_enu(2);
        // Convert Orientation
        Eigen::Quaterniond px4_q;
        px4_q.x() = vehicle_pose_px4_.pose.orientation.x;
        px4_q.y() = vehicle_pose_px4_.pose.orientation.y;
        px4_q.z() = vehicle_pose_px4_.pose.orientation.z;
        px4_q.w() = vehicle_pose_px4_.pose.orientation.w;
        Eigen::Quaterniond ros_q;
        ros_q = px4_ros_com::frame_transforms::ned_to_enu_orientation(px4_q);
        vehicle_pose_ros_.pose.orientation.x = ros_q.x();
        vehicle_pose_ros_.pose.orientation.y = ros_q.y();
        vehicle_pose_ros_.pose.orientation.z = ros_q.z();
        vehicle_pose_ros_.pose.orientation.w = ros_q.w();

        flag_vehicle_odometry_ = true; // Received vehicle odometry flag
    }

    /**
     * @brief Plan straight path with discretization steps
     * @param path The nav_msgs::msg::Path variable to append a path to
     * @param start_pose Start pose to plan from
     * @param end_pose End pose to plan to
     * @param step_size Step size for discretization
     * @param frame_id Reference frame_id for poses

     * @return The planned path with type nav_msgs::msg::Path
    */
    void plan_straight_path(nav_msgs::msg::Path& path, geometry_msgs::msg::Pose start_pose,
                            geometry_msgs::msg::Pose end_pose,
                            double step_size = 0.1, std::string frame_id = "/map")
    {
        // Calculate the distance between the start and end point
        double distance = euclidean_distance(start_pose.position, end_pose.position);
        // Calculate the number of steps using the input step size
        double number_of_steps = fabs(distance / step_size);
        // Round the number of steps to the nearest integer
        int rounded_number_of_steps = std::round(number_of_steps);

        // If rounded number of steps is equal to zero, then the provided step_size is greater
        // than the distance. In this case, set number of steps to 1, thus do not discretize the path
        if (rounded_number_of_steps == 0) {
            rounded_number_of_steps = 1;
        }

        // Yaw should move from start to end yaw in a tenth of the time that it takes the position
        // to move from start to end position
        // TODO probably not the best way to do this
        int yaw_number_of_steps = ceil(static_cast<float>(rounded_number_of_steps/10));
        if (yaw_number_of_steps <= 0){ yaw_number_of_steps = 1;}

        // Calculate yaw to point drone to end_pose
        double yaw_to_point = angleBetweenPoints(start_pose.position.x, start_pose.position.y,
                                                 end_pose.position.x, end_pose.position.y);

        // geometry_msgs::msg::Quaternion -> tf2::Quaternion -> Yaw (Radians)
        tf2::Quaternion tf_q_start;
        tf2::Quaternion tf_q_end;
        tf2::fromMsg(start_pose.orientation, tf_q_start);
        tf2::fromMsg(end_pose.orientation, tf_q_end);
        tf2::Matrix3x3 q_mat_start(tf_q_start);
        tf2::Matrix3x3 q_mat_end(tf_q_end);
        double roll_start, pitch_start, yaw_start, roll_end, pitch_end, yaw_end;
        q_mat_start.getRPY(roll_start, pitch_start, yaw_start);
        q_mat_end.getRPY(roll_end, pitch_end, yaw_end);

        // Populate path
        path.header.frame_id = frame_id;
        path.header.stamp = rclcpp::Node::now();

        // Iterate over each pair of coordinates and add them as states into our new Path object
        for (int j = 0; j <= rounded_number_of_steps; j++) {
            // Update point with incremental step values
            geometry_msgs::msg::Point point;
            point.x = start_pose.position.x +
                      (j * (end_pose.position.x - start_pose.position.x) / rounded_number_of_steps);
            point.y = start_pose.position.y +
                      (j * (end_pose.position.y - start_pose.position.y) / rounded_number_of_steps);
            point.z = start_pose.position.z +
                      (j * (end_pose.position.z - start_pose.position.z) / rounded_number_of_steps);

            // Update yaw-angle with incremental step values to point to the end_pose
            double yaw;
            if (j<=yaw_number_of_steps)
            {
                yaw = yaw_start + (j * (yaw_to_point - yaw_start) / rounded_number_of_steps);
            } else
            {
                yaw = yaw_to_point;
            }
            // Yaw (Radians) -> tf2::Quaternion
            tf2::Quaternion tf2_q;
            tf2_q.setRPY(0, 0, yaw);
            // tf2::Quaternion -> geometry_msgs::msg::Quaternion
            geometry_msgs::msg::Quaternion quaternion = tf2::toMsg(tf2_q);

            // Create pose stamped
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = frame_id;
            pose.header.stamp = rclcpp::Node::now();
            pose.pose.position = point;
            pose.pose.orientation = quaternion;

            // Add point to path
            path.poses.push_back(pose);
        }
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
     * @brief Convert degrees to radians
     * @param degrees Angle in [Deg]
     * @return Angle in radians
    */
    double degreesToRadians(double degrees) {
        return degrees * (M_PI / 180.0);
    }

    /**
     * @brief Change Fields2Cover PathState to geometry_msgs::msg::Pose
     * @param pathState Fields2Clover PathState to convert
     * @return geometry_msgs::msg::Pose
    */
    geometry_msgs::msg::Pose PathState_to_Pose(f2c::types::PathState pathState)
    {
        // Create pose
        geometry_msgs::msg::Pose pose;

        // F2CPoint -> geometry_msgs::msg::Pose
        pose.position.x = pathState.point.getX();
        pose.position.y = pathState.point.getY();
        pose.position.z = pathState.point.getZ();
        // Yaw (Radians) -> tf2::Quaternion
        tf2::Quaternion tf2_q;
        tf2_q.setRPY(0, 0, pathState.angle);
        // tf2::Quaternion -> geometry_msgs::msg::Quaternion
        geometry_msgs::msg::Quaternion quaternion = tf2::toMsg(tf2_q);
        pose.orientation = quaternion;

        return pose;
    }

    /**
     * @brief Calculate angle between two points
     * @param x1 First point x-coordinate
     * @param y1 First point y-coordinate
     * @param x2 Second point x-coordinate
     * @param y2 Second point y-coordinate
     * @return Angle between points in radians
    */
    double angleBetweenPoints(double x1, double y1, double x2, double y2) {
        double dy = y2 - y1;
        double dx = x2 - x1;
        
        double angle = atan2(dy, dx);

        // Adjust angle to [0, 2*Pi] range
        if (angle < 0) {
            angle += 2 * M_PI;
        }

        return angle;
    }

    ///
    ///
    /// LIBRARY functions later!!
    ///
    ///

    /**
     * @brief Main Timer callback
     * @return void
    */
    void timer_callback()
    {
        switch (current_state_)
        {
            // IDLE state -> Wait for drone odometry
            case State::IDLE:
                if (flag_vehicle_odometry_)
                {
                    // Save the drones HOME position and Hover home position
                    home_pose_ = vehicle_pose_ros_.pose;
                    hover_home_pose_ = home_pose_;
                    hover_home_pose_.position.z = 5; // 5[m] above home pose
                    // Change state to path planning
                    current_state_ = State::PATH_PLANNING;
                    RCLCPP_INFO_STREAM(get_logger(), "State: PATH_PLANNING");
                }
                break;
            case State::PATH_PLANNING:
                // Mission Path planning with Fields2Cover
                f2c_path_ = path_planning();

                // TODO Create take-off path to the height of the Fields2Cover path
                // For now just take-off is a setpoint at 5[m]

                // Create path from hover to start of Fields2Cover path
                plan_straight_path(path_, hover_home_pose_,
                                   PathState_to_Pose(f2c_path_.states.at(0)),
                                   0.1, "/map");

                // Convert Fields2Cover path to a nav_msg::msg::path format
                f2cpath_to_navpath(path_, f2c_path_);

                // Create path from end of fields 2 cover to home position
                plan_straight_path(path_, PathState_to_Pose(f2c_path_.states.back()),
                                   hover_home_pose_,
                                   0.1, "/map");

                // Change state to path planned
                current_state_ = State::PUB_PATH;
                RCLCPP_INFO_STREAM(get_logger(), "State: PUB_PATH");
                break;
            case State::PUB_PATH:
                // Publish path
                path_publisher_->publish(path_);
                // Publish drone pose
                vehicle_pose_publisher_->publish(vehicle_pose_ros_);
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
    rclcpp::spin(std::make_shared<PathPlanning>());

    rclcpp::shutdown();
    return 0;
}
