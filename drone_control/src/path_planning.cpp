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
// ROS2 libs
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/point.hpp"
// Import lib for motion planning package
#include "fields2cover.h"

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

        //
        // TODO waypoint should be read in from a .json
        // NOTE this is a rectangle
        //
        // Up
        waypoint_0_.x = 0;
        waypoint_0_.y = 0;
        waypoint_0_.z = -5;
        // Forward
        waypoint_1_.x = 0;
        waypoint_1_.y = 40;
        waypoint_1_.z = -5;
        // Left
        waypoint_2_.x = 20;
        waypoint_2_.y = 40;
        waypoint_2_.z = -5;
        // Back
        waypoint_3_.x = 20;
        waypoint_3_.y = 0;
        waypoint_3_.z = -5;
        // Home
        waypoint_4_.x = 0;
        waypoint_4_.y = 0;
        waypoint_4_.z = -5;
        // Add all waypoint to the vector
        waypoints_.push_back(waypoint_0_);
        waypoints_.push_back(waypoint_1_);
        waypoints_.push_back(waypoint_2_);
        waypoints_.push_back(waypoint_3_);
        waypoints_.push_back(waypoint_4_);

        // Path planning with Fields2Cover
        path_planning();

        // Initialize variables

        // Create publishers

        // Create subscribers

        // Create timer
    }

private:
    // Variables
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

    /**
     * @brief Path planning
     *s
    */
    void path_planning()
    {
        // Use Fields2Cover
        // Define field and robot
        F2CRobot robot (2.0, 5.0);
        F2CCells field(F2CCell(F2CLinearRing({F2CPoint(0,0), F2CPoint(0,40), F2CPoint(20,40),
                                              F2CPoint(20,0), F2CPoint(0,0)})));
        // Swath generation
        f2c::sg::BruteForce bf;
        f2c::obj::NSwath n_swath_obj;
        F2CSwaths swaths = bf.generateSwaths(M_PI, robot.op_width, field.getGeometry(0));
        f2c::rp::BoustrophedonOrder boustrophedon_sorter;
        auto boustrophedon_swaths = boustrophedon_sorter.genSortedSwaths(swaths, 1);
        // Path planner
        f2c::pp::PathPlanning path_planner;
        robot.setMinRadius(2.5);  // m
        // Create swath connections using Dubins curves with Continuous curvature
        f2c::pp::DubinsCurvesCC dubins_cc;
        F2CPath path_dubins_cc = path_planner.searchBestPath(robot, boustrophedon_swaths, dubins_cc);
        // Discretize swath lines in path object
        // Specify the step size and the robot velocity for the swath section
        // double discretize_step_size = 30; // Step size for discretization in [m]
        // F2CPath new_path = path_dubins_cc.discretize_swath(discretize_step_size);
        // Save to file
        path_dubins_cc.saveToFile("discretized_swath_path.csv", 4); // Specify precision to the significant number
        // Visualize
        f2c::Visualizer::figure();
        f2c::Visualizer::plot(field);
        // f2c::Visualizer::plot(no_hl);
        f2c::Visualizer::plot(path_dubins_cc);
        f2c::Visualizer::plot(boustrophedon_swaths);
        f2c::Visualizer::show();
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
