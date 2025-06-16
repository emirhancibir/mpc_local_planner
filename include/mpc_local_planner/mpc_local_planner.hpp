// include/mpc_local_planner/mpc_planner.hpp

#pragma once

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <pluginlib/class_list_macros.h>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#include <casadi/casadi.hpp>

namespace mpc_local_planner {

    class MPCPlanner : public nav_core::BaseLocalPlanner {
    public:
    MPCPlanner();
    ~MPCPlanner();

    void initialize(std::string name, tf2_ros::Buffer* tf,
                    costmap_2d::Costmap2DROS* costmap_ros) override;

    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;
    bool isGoalReached() override;
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override;

    private:
    bool initialized_;
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

    int N_;        // horizon length
    double dt_;    // time step
    casadi::Function dynamicsFunc_;


    void setupOptimizer();
    casadi::MX dynamics(const casadi::MX& x, const casadi::MX& u);
    };

} // namespace mpc_local_planner
