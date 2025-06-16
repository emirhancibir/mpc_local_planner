#include "mpc_local_planner/mpc_local_planner.hpp"
#include <pluginlib/class_list_macros.h>

using namespace casadi;


PLUGINLIB_EXPORT_CLASS(mpc_local_planner::MPCPlanner, nav_core::BaseLocalPlanner)
namespace mpc_local_planner {

  MPCPlanner::MPCPlanner() : initialized_(false), N_(20), dt_(0.1), opti_() {}

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

    initialized_ = true;
  }

  bool MPCPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) {
    global_plan_ = plan;
    return true;
  }

  bool MPCPlanner::isGoalReached() {
    return global_plan_.empty();
  }

  void MPCPlanner::setupOptimizer() {
    opti_ = casadi::Opti();

    X_ = opti_.variable(3, N_ + 1); // [x, y, theta]
    U_ = opti_.variable(2, N_);     // [v, w]

    X0_ = opti_.parameter(3);       // Robotu start
    Ref_ = opti_.parameter(2, N_);

    opti_.subject_to(X_(Slice(), 0) == X0_);

    for (int k = 0; k < N_; ++k) {
      MX x_next = dynamics(X_(Slice(), k), U_(Slice(), k));
      opti_.subject_to(X_(Slice(), k + 1) == x_next);
    }

    MX cost = 0;
    for (int k = 0; k < N_; ++k) {
      MX pos_err = X_(Slice(0, 2), k) - Ref_(Slice(), k);
      MX angle_to_goal = atan2(Ref_(1, k) - X_(1, k), Ref_(0, k) - X_(0, k));
      MX heading_error = angle_to_goal - X_(2, k);
      // normalize
      heading_error = heading_error - 2*M_PI * floor((heading_error + M_PI)/(2*M_PI));

      cost += MX::dot(pos_err, pos_err)
              + 0.1 * MX::dot(U_(Slice(), k), U_(Slice(), k))
              + 0.05 * heading_error * heading_error;
    }


    opti_.minimize(cost);

    opti_.subject_to(opti_.bounded(0.0, U_(0, Slice()), 0.5));   // v ileri
    opti_.subject_to(opti_.bounded(-0.4, U_(1, Slice()), 0.4));  // w bound

    opti_.solver("ipopt");
  }

  MX MPCPlanner::dynamics(const MX& x, const MX& u) {
    MX x_next = MX::zeros(3);
    x_next(0) = x(0) + dt_ * u(0) * cos(x(2)); // x + v*dt*cos(theta)
    x_next(1) = x(1) + dt_ * u(0) * sin(x(2)); // y + v*dt*sin(theta)
    x_next(2) = x(2) + dt_ * u(1);             // theta + w*dt
    return x_next;
  }

  bool MPCPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
    if (global_plan_.size() < N_) {
      ROS_WARN("Global plan kisa: %lu < %d", global_plan_.size(), N_);
      return false;
    }

    geometry_msgs::PoseStamped pose;
    if (!costmap_ros_->getRobotPose(pose)) {
      ROS_WARN("Robot pose alinamadi");
      return false;
    }

    double x = pose.pose.position.x;
    double y = pose.pose.position.y;
    double theta = tf2::getYaw(pose.pose.orientation);

    opti_.set_value(X0_, casadi::DM({x, y, theta}));

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

      return true;
    } catch (std::exception& e) {
      ROS_WARN("MPC çözümü başarısız: %s", e.what());
      return false;
    }
  }



}


