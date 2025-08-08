// include/mpc_local_planner/mpc_planner.hpp

#pragma once
#include <iostream>
#include <angles/angles.h>
#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <pluginlib/class_list_macros.h>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Path.h>


#include <casadi/casadi.hpp>

namespace mpc_local_planner {

    class MPCPlanner : public nav_core::BaseLocalPlanner {
    public:
    MPCPlanner();
    ~MPCPlanner();
    
    enum class MPCState
    {
        TRACK,
        ROTATE_GOAL,
        STOP
    };


    void initialize(std::string name, tf2_ros::Buffer* tf,
                    costmap_2d::Costmap2DROS* costmap_ros) override;

    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;
    bool isGoalReached() override;
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override;
    bool rotateGoal(geometry_msgs::Twist& cmd_vel);
    bool mpcCalculator(geometry_msgs::Twist& cmd_vel);
    void updateObstaclePoints();

    private:
    MPCState state_;
    std::vector<std::pair<double, double>> obstacle_points_;

    bool initialized_;
    double pos_weight = 1.0, obs_weight;
    tf2_ros::Buffer* tf_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    costmap_2d::Costmap2D* costmap_;
    std::string global_frame_;
    std::string robot_base_frame_;

    ros::Publisher cmd_pub_;
    std::vector<geometry_msgs::PoseStamped> global_plan_;

    // MPC setup
    casadi::Opti opti_;
    casadi::MX X_, U_;
    casadi::MX X0_;      // Başlangıç durumu parametresi
    casadi::MX Ref_;     // Referans yol parametresi
    casadi::MX Obs_;


    int N_;        // horizon length
    double dt_;    // time step
    casadi::Function dynamicsFunc_;

    int max_obstacles_ = 50;


    void setupOptimizer();
    casadi::MX dynamics(const casadi::MX& x, const casadi::MX& u);
    
    // ros
    ros::NodeHandle nh;
    ros::Publisher mpc_path_pub_;
    };

} // namespace mpc_local_planner
