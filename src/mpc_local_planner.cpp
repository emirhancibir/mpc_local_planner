#include "mpc_local_planner/mpc_local_planner.hpp"
#include <pluginlib/class_list_macros.h>

using namespace casadi;


PLUGINLIB_EXPORT_CLASS(mpc_local_planner::MPCPlanner, nav_core::BaseLocalPlanner)
namespace mpc_local_planner {

  MPCPlanner::MPCPlanner() : initialized_(false), N_(150), dt_(0.1), opti_(), state_(MPCState::TRACK) {}

  MPCPlanner::~MPCPlanner() {}

  void MPCPlanner::initialize(std::string name, tf2_ros::Buffer* tf,
                              costmap_2d::Costmap2DROS* costmap_ros) {
    if (initialized_) return;

    tf_ = tf;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    global_frame_ = costmap_ros_->getGlobalFrameID();
    robot_base_frame_ = costmap_ros_->getBaseFrameID();

    ros::NodeHandle private_nh("~/" + name);

    setupOptimizer();

    mpc_path_pub_ = nh.advertise<nav_msgs::Path>("/mpc_local_planner/mpc_path", 1);

    initialized_ = true;
  }

  bool MPCPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) {
    global_plan_ = plan;
    state_ = MPCState::TRACK; 
    return true;
  }

  bool MPCPlanner::isGoalReached()
  {
    if (global_plan_.empty()) return false;
    if (state_ == MPCState::STOP){
      return true;
      ROS_INFO("Goal Reached!");
    }

    geometry_msgs::PoseStamped pose;
    if (!costmap_ros_->getRobotPose(pose)) {
      ROS_WARN("Robot pozisyonu alinamadi");
      return false;
    }

    geometry_msgs::PoseStamped goal = global_plan_.back();

    double dx = goal.pose.position.x - pose.pose.position.x;
    double dy = goal.pose.position.y - pose.pose.position.y;
    double dist_to_goal = std::hypot(dx, dy);

    double goal_yaw = tf2::getYaw(goal.pose.orientation);
    double robot_yaw = tf2::getYaw(pose.pose.orientation);
    double yaw_diff = angles::shortest_angular_distance(robot_yaw, goal_yaw);

    const double position_tolerance = 0.1;  // metre
    const double yaw_tolerance = 0.1;       // rad

    if (dist_to_goal < position_tolerance && std::abs(yaw_diff) < yaw_tolerance) 
    {
      ROS_INFO("Goal Reached!");
      return true;
    }

    return false;
  }


  bool MPCPlanner::rotateGoal(geometry_msgs::Twist& cmd_vel)
  {
    geometry_msgs::PoseStamped pose;
    if (!costmap_ros_->getRobotPose(pose)) {
      ROS_WARN("Robot pozisyonu alinamadi");
      return false;
    }

    geometry_msgs::PoseStamped goal = global_plan_.back();

    double goal_yaw = tf2::getYaw(goal.pose.orientation);
    double robot_yaw = tf2::getYaw(pose.pose.orientation);

    double yaw_diff = angles::shortest_angular_distance(robot_yaw, goal_yaw);

    const double yaw_tolerance = 0.05;
    const double max_rot_speed = 0.3;
    const double k_rot = 1.0; // P kontrol kazancı

    if (std::abs(yaw_diff) < yaw_tolerance) {
      cmd_vel.angular.z = 0.0;
      return true;
    }

    cmd_vel.linear.x = 0.0;
    double clamped_rot = std::max(-max_rot_speed, std::min(k_rot * yaw_diff, max_rot_speed));
    cmd_vel.angular.z = clamped_rot;
    return true;
  }


  void MPCPlanner::setupOptimizer() {
    opti_ = casadi::Opti();

    X_ = opti_.variable(3, N_ + 1); // [x, y, theta]
    U_ = opti_.variable(2, N_);     // [v, w]

    X0_ = opti_.parameter(3);       // Robotu start
    Ref_ = opti_.parameter(2, N_);
    Obs_ = opti_.parameter(2, max_obstacles_);  // max_obstacles_: sabit sayı (örneğin 50)


    opti_.subject_to(X_(Slice(), 0) == X0_);

    for (int k = 0; k < N_; ++k) {
      MX x_next = dynamics(X_(Slice(), k), U_(Slice(), k));
      opti_.subject_to(X_(Slice(), k + 1) == x_next);
    }
    MX cost = 0;
    for (int k = 0; k < N_; ++k) {
      MX pos_err = X_(Slice(0, 2), k) - Ref_(Slice(), k);

      cost += MX::dot(pos_err, pos_err) * pos_weight + 0.1 * MX::dot(U_(Slice(), k), U_(Slice(), k));
      // std::cout << "Engel oncesi costt : " << cost << std::endl;
      for (int j = 0; j < max_obstacles_; ++j) {
        MX dx = X_(0, k) - Obs_(0, j);
        MX dy = X_(1, k) - Obs_(1, j);
        MX dist_sq = dx*dx + dy*dy;
        cost += log(1 + 1.0 / (dist_sq + 0.1));  // daha yumuşak alternatif
        // std::cout << "Engel sonrasi costt : " << cost << std::endl;

      }
    }
    
    

    opti_.minimize(cost);

    opti_.subject_to(opti_.bounded(-0.9, U_(0, Slice()), 0.9));   // v ileri
    opti_.subject_to(opti_.bounded(-0.4, U_(1, Slice()), 0.4));  // w bound

    Dict opts_dict;
    opts_dict["ipopt.print_level"] = 0;
    opts_dict["ipopt.sb"] = "yes";
    opts_dict["print_time"] = 0;

    opti_.solver("ipopt", opts_dict);

  }

  MX MPCPlanner::dynamics(const MX& x, const MX& u) {
    MX x_next = MX::zeros(3);
    x_next(0) = x(0) + dt_ * u(0) * cos(x(2)); // x + v*dt*cos(theta)
    x_next(1) = x(1) + dt_ * u(0) * sin(x(2)); // y + v*dt*sin(theta)
    x_next(2) = x(2) + dt_ * u(1);             // theta + w*dt
    return x_next;
  }

  void MPCPlanner::updateObstaclePoints()
  {
    // std::cout << "update obsss "<< std::endl;
    // costmap_ros_->updateMap();

    obstacle_points_.clear();

    unsigned int size_x = costmap_->getSizeInCellsX();
    unsigned int size_y = costmap_->getSizeInCellsY();
    double resolution = costmap_->getResolution();
    double origin_x = costmap_->getOriginX();
    double origin_y = costmap_->getOriginY();

    for (unsigned int mx = 0; mx < size_x; ++mx) {
      for (unsigned int my = 0; my < size_y; ++my) {
        unsigned char cost = costmap_->getCost(mx, my);
        // if (cost > 0)
        //   std::cout << "cost: " << static_cast<int>(cost) << std::endl;
        if (cost > 0) {
          double wx = origin_x + mx * resolution;
          double wy = origin_y + my * resolution;
          // std::cout << " wx " << wx << "wy "<< wy << std::endl;
          obstacle_points_.emplace_back(wx, wy);
        }
      }
    }
  }


  bool MPCPlanner::mpcCalculator(geometry_msgs::Twist& cmd_vel)
  {
    //  if (global_plan_.size() < N_) {
    //   ROS_WARN("Global plan kisa: %lu < %d", global_plan_.size(), N_);
    //   N_ = global_plan_.size();
    //   // return false;
    // }

    geometry_msgs::PoseStamped pose;
    if (!costmap_ros_->getRobotPose(pose)) {
      ROS_WARN("Robot pose alinamadi");
      return false;
    }

    double x = pose.pose.position.x;
    double y = pose.pose.position.y;
    double theta = tf2::getYaw(pose.pose.orientation);

    double min_dist_to_obstacle = 1e9;
    for (const auto& obs : obstacle_points_) {
        double dx = obs.first - x;
        double dy = obs.second - y;
        double dist = std::hypot(dx, dy);
        if (dist < min_dist_to_obstacle)
            min_dist_to_obstacle = dist;
    }

    pos_weight = (min_dist_to_obstacle < 0.6) ? 0.05 : 1;


    opti_.set_value(X0_, casadi::DM({x, y, theta}));
    updateObstaclePoints();  

    int n_obs = std::min((int)obstacle_points_.size(), max_obstacles_);
    casadi::DM obs_matrix = casadi::DM::zeros(2, max_obstacles_);

    for (int i = 0; i < n_obs; ++i) {
      obs_matrix(0, i) = obstacle_points_[i].first;
      obs_matrix(1, i) = obstacle_points_[i].second;
    }

    for (int i = n_obs; i < max_obstacles_; ++i) {
      obs_matrix(0, i) = 9999;
      obs_matrix(1, i) = 9999;
    }

    opti_.set_value(Obs_, obs_matrix);


    int start_idx = 0;
    double min_dist = 1e9;
    for (size_t i = 0; i < global_plan_.size(); ++i) {
      double dx = global_plan_[i].pose.position.x - x;
      double dy = global_plan_[i].pose.position.y - y;
      double dist = std::hypot(dx, dy);
      if (dist < min_dist) {
        min_dist = dist;
        start_idx = i;
      }
    }


    casadi::DM ref_points = casadi::DM::zeros(2, N_);
    for (int i = 0; i < N_; ++i) {
      int idx = std::min(start_idx + i, (int)global_plan_.size() - 1);
      ref_points(0, i) = global_plan_[idx].pose.position.x;
      ref_points(1, i) = global_plan_[idx].pose.position.y;
    }


    opti_.set_value(Ref_, ref_points);

    try {
      casadi::OptiSol sol = opti_.solve();
      double v = static_cast<double>(sol.value(U_(0, 0)));
      double w = static_cast<double>(sol.value(U_(1, 0)));

      cmd_vel.linear.x = v;
      cmd_vel.angular.z = w;

      // Path Publishinggg /////////////////////////////////////////////////////////////////////////////
      nav_msgs::Path path_msg;
      path_msg.header.frame_id = global_frame_;
      path_msg.header.stamp = ros::Time::now();

      casadi::DM sol_X = sol.value(X_);

      for (int k = 0; k <= N_; ++k) {
        geometry_msgs::PoseStamped pose;
        pose.header = path_msg.header;
        pose.pose.position.x = static_cast<double>(sol_X(0, k));
        pose.pose.position.y = static_cast<double>(sol_X(1, k));
        pose.pose.position.z = 0.0;

        double theta = static_cast<double>(sol_X(2, k));
        tf2::Quaternion q;
        q.setRPY(0, 0, theta);
        pose.pose.orientation = tf2::toMsg(q);

        path_msg.poses.push_back(pose);
      }

      mpc_path_pub_.publish(path_msg);
      /////////////////////////////////////////////////////////////////////



      return true;
    } catch (std::exception& e) {
      ROS_WARN("MPC cannot solve: %s", e.what());
      return false;
    }
  }
  
  bool MPCPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
  {
    if (global_plan_.empty()) {
      ROS_WARN_THROTTLE(1.0, "Global plan bos.");
      return false;
    }

    geometry_msgs::PoseStamped pose;
    if (!costmap_ros_->getRobotPose(pose)) {
      ROS_WARN("Robot pozisyonu alinamadi");
      return false;
    }

    geometry_msgs::PoseStamped goal = global_plan_.back();
    double dx = goal.pose.position.x - pose.pose.position.x;
    double dy = goal.pose.position.y - pose.pose.position.y;
    double dist_to_goal = std::hypot(dx, dy);
    double goal_yaw = tf2::getYaw(goal.pose.orientation);
    double robot_yaw = tf2::getYaw(pose.pose.orientation);
    double yaw_diff = angles::shortest_angular_distance(robot_yaw, goal_yaw);

    const double position_tolerance = 0.1;
    const double yaw_tolerance = 0.1;

    // FSM LOGIC
    switch (state_) {
      case MPCState::TRACK:

        if (dist_to_goal < position_tolerance) {
          ROS_INFO("Goal reached rotating....");
          state_ = MPCState::ROTATE_GOAL;
          return rotateGoal(cmd_vel); 
        }
        ROS_INFO("TRACKING MPC");
        return mpcCalculator(cmd_vel);

      case MPCState::ROTATE_GOAL:
        if (std::abs(yaw_diff) < yaw_tolerance) {
          ROS_INFO("stopped");
          state_ = MPCState::STOP;
          cmd_vel.linear.x = 0.0;
          cmd_vel.angular.z = 0.0;
          return true;
        } else {
          return rotateGoal(cmd_vel);
        }

      case MPCState::STOP:
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        return true;
    }

    return false;
  }




}


