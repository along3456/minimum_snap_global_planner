#include "minimum_snap_planner/minimum_snap_planner.h"
#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>

PLUGINLIB_EXPORT_CLASS(minimum_snap_planner::MinimumSnapPlanner, nav_core::BaseGlobalPlanner)

namespace minimum_snap_planner
{

MinimumSnapPlanner::MinimumSnapPlanner()
    : costmap_ros_(nullptr)
    , costmap_(nullptr)
    , global_planner_(nullptr)
    , minimum_snap_ptr_(nullptr)
    , pending_waypoints_available_(false)
    , has_last_goal_(false)
    , goal_change_tolerance_(0.05)
    , initialized_(false)
{
}

MinimumSnapPlanner::MinimumSnapPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_ros_(nullptr)
    , costmap_(nullptr)
    , global_planner_(nullptr)
    , minimum_snap_ptr_(nullptr)
    , pending_waypoints_available_(false)
    , has_last_goal_(false)
    , goal_change_tolerance_(0.05)
    , initialized_(false)
{
    initialize(name, costmap_ros);
}

MinimumSnapPlanner::~MinimumSnapPlanner()
{
}

void MinimumSnapPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    if (!initialized_)
    {
        costmap_ros_ = costmap_ros;
        costmap_ = costmap_ros_->getCostmap();
        global_frame_ = costmap_ros_->getGlobalFrameID();

        ros::NodeHandle private_nh("~/" + name);
        
        // Load parameters
        private_nh.param("downsample_resolution", downsample_resolution_, 0.5);
        private_nh.param("max_vel", max_vel_, 1.0);
        private_nh.param("max_accel", max_accel_, 1.0);
        int poly_order_int = 3;
        private_nh.param("polynomial_order", poly_order_int, 3);
        polynomial_order_ = static_cast<unsigned int>(poly_order_int);
        private_nh.param("trajectory_time_step", trajectory_time_step_, 0.01);
        private_nh.param("waypoint_topic", waypoint_topic_, std::string("/waypoints"));
        private_nh.param("goal_change_tolerance", goal_change_tolerance_, 0.05);

        ROS_INFO("[WaypointSnapPlanner] Parameters loaded:");
        ROS_INFO("  - downsample_resolution: %.2f", downsample_resolution_);
        ROS_INFO("  - max_vel: %.2f", max_vel_);
        ROS_INFO("  - max_accel: %.2f", max_accel_);
        ROS_INFO("  - polynomial_order: %u", polynomial_order_);
        ROS_INFO("  - waypoint_topic: %s", waypoint_topic_.c_str());
        ROS_INFO("  - goal_change_tolerance: %.3f", goal_change_tolerance_);

        // Initialize helper global planner (cost-aware A*)
        global_planner_ = std::make_shared<global_planner::GlobalPlanner>();
        global_planner_->initialize(name + "_costaware_segments", costmap_ros_);

        // Initialize Minimum Snap trajectory generator
        minimum_snap_ptr_ = std::make_shared<MinimumSnap>(
            polynomial_order_, max_vel_, max_accel_);

        // Subscribe to waypoints
        ros::NodeHandle nh;
        waypoint_sub_ = nh.subscribe(waypoint_topic_, 1, 
                                     &MinimumSnapPlanner::waypointCallback, this);
        
        plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);

        initialized_ = true;
        ROS_INFO("[WaypointSnapPlanner] Initialized successfully!");
    }
    else
    {
        ROS_WARN("[WaypointSnapPlanner] Already initialized!");
    }
}

void MinimumSnapPlanner::waypointCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(waypoint_mutex_);
    pending_waypoints_.assign(msg->poses.begin(), msg->poses.end());
    pending_waypoints_available_ = true;

    ROS_INFO("[WaypointSnapPlanner] Received %lu pending waypoints", pending_waypoints_.size());
}

bool MinimumSnapPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                   const geometry_msgs::PoseStamped& goal,
                                   std::vector<geometry_msgs::PoseStamped>& plan)
{
    if (!initialized_)
    {
        ROS_ERROR("[WaypointSnapPlanner] Planner not initialized!");
        return false;
    }

    plan.clear();

    ROS_INFO("[WaypointSnapPlanner] Planning from (%.2f, %.2f) to (%.2f, %.2f)",
             start.pose.position.x, start.pose.position.y,
             goal.pose.position.x, goal.pose.position.y);

    // Load the waypoint set that should be applied for this navigation request
    std::vector<geometry_msgs::Pose> active_waypoints = loadWaypointsForGoal(goal);
    ROS_INFO("[WaypointSnapPlanner] Using %lu waypoint(s) for this plan", active_waypoints.size());

    std::vector<geometry_msgs::PoseStamped> waypoint_stamped;
    waypoint_stamped.reserve(active_waypoints.size());
    for (const auto& pose : active_waypoints)
    {
        geometry_msgs::PoseStamped wp;
        wp.header.frame_id = global_frame_;
        wp.header.stamp = ros::Time::now();
        wp.pose = pose;
        waypoint_stamped.push_back(wp);
    }

    // Multi-segment cost-aware planning
    std::vector<Eigen::Vector2d> all_path_points;
    if (!planMultiSegmentPath(start, waypoint_stamped, goal, all_path_points))
    {
        ROS_ERROR("[WaypointSnapPlanner] Multi-segment cost-aware planning failed!");
        return false;
    }

    // Waypoints are one-shot; clear active buffer until the user publishes again
    clearActiveWaypoints();

    ROS_INFO("[WaypointSnapPlanner] Total A* path points: %lu", all_path_points.size());

    if (all_path_points.size() < 2)
    {
        ROS_ERROR("[WaypointSnapPlanner] Not enough path points!");
        return false;
    }

    // Downsample path
    std::vector<Eigen::Vector2d> downsampled_path = downsamplePath(all_path_points, downsample_resolution_);
    
    ROS_INFO("[WaypointSnapPlanner] Downsampled to %lu points", downsampled_path.size());

    if (downsampled_path.size() < 2)
    {
        ROS_ERROR("[WaypointSnapPlanner] Not enough points after downsampling!");
        return false;
    }

    // Generate smooth trajectory using Minimum Snap
    bool success = generateMinimumSnapTrajectory(downsampled_path, plan);
    
    if (!success)
    {
        ROS_ERROR("[WaypointSnapPlanner] Failed to generate Minimum Snap trajectory!");
        return false;
    }

    ROS_INFO("[WaypointSnapPlanner] Generated plan with %lu points", plan.size());

    // Publish plan for visualization
    if (plan_pub_.getNumSubscribers() > 0)
    {
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = global_frame_;
        path_msg.header.stamp = ros::Time::now();
        path_msg.poses = plan;
        plan_pub_.publish(path_msg);
    }

    return true;
}

std::vector<Eigen::Vector2d> MinimumSnapPlanner::downsamplePath(
    const std::vector<Eigen::Vector2d>& path,
    double resolution)
{
    if (path.size() <= 2)
    {
        return path;
    }

    std::vector<Eigen::Vector2d> downsampled;
    downsampled.push_back(path.front());  // Always keep start
    
    double accumulated_dist = 0.0;
    
    for (size_t i = 1; i < path.size() - 1; ++i)
    {
        double dist = (path[i] - path[i-1]).norm();
        accumulated_dist += dist;
        
        if (accumulated_dist >= resolution)
        {
            downsampled.push_back(path[i]);
            accumulated_dist = 0.0;
        }
    }
    
    downsampled.push_back(path.back());  // Always keep goal
    
    return downsampled;
}

bool MinimumSnapPlanner::generateMinimumSnapTrajectory(
    const std::vector<Eigen::Vector2d>& waypoints,
    std::vector<geometry_msgs::PoseStamped>& plan)
{
    if (waypoints.size() < 2)
    {
        return false;
    }

    // Convert waypoints to matrix format
    unsigned int num_waypoints = waypoints.size();
    MatXd waypoint_matrix = MatXd::Zero(num_waypoints, 2);
    
    for (unsigned int i = 0; i < num_waypoints; ++i)
    {
        waypoint_matrix(i, 0) = waypoints[i].x();
        waypoint_matrix(i, 1) = waypoints[i].y();
    }

    // Allocate time for each segment
    VecXd segment_times = minimum_snap_ptr_->AllocateTime(waypoint_matrix);
    
    // Initialize velocity and acceleration (zero at start and end)
    VecXd waypoint_vel = VecXd::Zero(num_waypoints);
    VecXd waypoint_accel = VecXd::Zero(num_waypoints);

    // Solve for polynomial coefficients in x and y directions
    MatXd poly_coeff_x = minimum_snap_ptr_->SolveQPClosedForm(
        waypoint_matrix.col(0), waypoint_vel, waypoint_accel, segment_times);
    
    MatXd poly_coeff_y = minimum_snap_ptr_->SolveQPClosedForm(
        waypoint_matrix.col(1), waypoint_vel, waypoint_accel, segment_times);

    unsigned int num_segments = segment_times.size();
    unsigned int poly_coeff_num = minimum_snap_ptr_->GetPolyCoeffNum();

    // Generate trajectory points
    for (unsigned int seg = 0; seg < num_segments; ++seg)
    {
        double segment_time = segment_times(seg);
        
        for (double t = 0.0; t < segment_time; t += trajectory_time_step_)
        {
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = global_frame_;
            pose.header.stamp = ros::Time::now();
            
            // Evaluate polynomial at time t
            VecXd coeff_x = poly_coeff_x.row(seg);
            VecXd coeff_y = poly_coeff_y.row(seg);
            
            pose.pose.position.x = evaluatePolynomial(coeff_x, t);
            pose.pose.position.y = evaluatePolynomial(coeff_y, t);
            pose.pose.position.z = 0.0;
            
            // Calculate orientation from trajectory direction
            if (!plan.empty())
            {
                double dx = pose.pose.position.x - plan.back().pose.position.x;
                double dy = pose.pose.position.y - plan.back().pose.position.y;
                double yaw = atan2(dy, dx);
                
                tf2::Quaternion q;
                q.setRPY(0, 0, yaw);
                pose.pose.orientation.x = q.x();
                pose.pose.orientation.y = q.y();
                pose.pose.orientation.z = q.z();
                pose.pose.orientation.w = q.w();
            }
            else
            {
                pose.pose.orientation.w = 1.0;
            }
            
            plan.push_back(pose);
        }
    }
    
    // Add final point
    if (!plan.empty() && num_waypoints > 0)
    {
        geometry_msgs::PoseStamped final_pose;
        final_pose.header.frame_id = global_frame_;
        final_pose.header.stamp = ros::Time::now();
        final_pose.pose.position.x = waypoints.back().x();
        final_pose.pose.position.y = waypoints.back().y();
        final_pose.pose.position.z = 0.0;
        
        if (plan.size() > 1)
        {
            final_pose.pose.orientation = plan.back().pose.orientation;
        }
        else
        {
            final_pose.pose.orientation.w = 1.0;
        }
        
        plan.push_back(final_pose);
    }

    return true;
}

double MinimumSnapPlanner::evaluatePolynomial(const VecXd& coeffs, double t)
{
    double result = 0.0;
    unsigned int n = coeffs.size();
    
    // Coefficients are stored in reverse order (highest degree first)
    for (unsigned int i = 0; i < n; ++i)
    {
        double power = (i == 0) ? 1.0 : std::pow(t, i);
        result += coeffs(n - 1 - i) * power;
    }
    
    return result;
}

bool MinimumSnapPlanner::planMultiSegmentPath(
    const geometry_msgs::PoseStamped& start,
    const std::vector<geometry_msgs::PoseStamped>& waypoints,
    const geometry_msgs::PoseStamped& goal,
    std::vector<Eigen::Vector2d>& out_points)
{
    out_points.clear();

    if (!global_planner_)
    {
        ROS_ERROR("[WaypointSnapPlanner] Global planner helper is not initialized.");
        return false;
    }

    std::vector<geometry_msgs::PoseStamped> sequence;
    sequence.reserve(waypoints.size() + 2);
    sequence.push_back(start);
    sequence.insert(sequence.end(), waypoints.begin(), waypoints.end());
    sequence.push_back(goal);

    if (sequence.size() < 2)
    {
        ROS_WARN("[WaypointSnapPlanner] Planning sequence must contain at least start and goal.");
        return false;
    }

    bool include_first_point = true;
    std::vector<geometry_msgs::PoseStamped> segment_plan;

    for (size_t i = 0; i < sequence.size() - 1; ++i)
    {
        segment_plan.clear();

        geometry_msgs::PoseStamped segment_start = sequence[i];
        geometry_msgs::PoseStamped segment_goal = sequence[i + 1];
        segment_start.header.stamp = ros::Time::now();
        segment_goal.header.stamp = ros::Time::now();

        ROS_INFO("[WaypointSnapPlanner] Cost-aware segment %lu: (%.2f, %.2f) -> (%.2f, %.2f)",
                 i,
                 segment_start.pose.position.x, segment_start.pose.position.y,
                 segment_goal.pose.position.x, segment_goal.pose.position.y);

        if (!global_planner_->makePlan(segment_start, segment_goal, segment_plan) || segment_plan.size() < 2)
        {
            ROS_ERROR("[WaypointSnapPlanner] Global planner failed for segment %lu.", i);
            return false;
        }

        size_t start_idx = include_first_point ? 0 : 1;
        for (size_t j = start_idx; j < segment_plan.size(); ++j)
        {
            out_points.emplace_back(segment_plan[j].pose.position.x,
                                    segment_plan[j].pose.position.y);
        }

        include_first_point = false;
    }

    return !out_points.empty();
}

std::vector<geometry_msgs::Pose> MinimumSnapPlanner::loadWaypointsForGoal(const geometry_msgs::PoseStamped& goal)
{
    std::lock_guard<std::mutex> lock(waypoint_mutex_);
    bool goal_changed = isNewGoal(goal);

    if (pending_waypoints_available_)
    {
        active_waypoints_ = pending_waypoints_;
        pending_waypoints_.clear();
        pending_waypoints_available_ = false;
    }
    else if (goal_changed)
    {
        active_waypoints_.clear();
    }

    last_goal_ = goal;
    has_last_goal_ = true;

    return active_waypoints_;
}

bool MinimumSnapPlanner::isNewGoal(const geometry_msgs::PoseStamped& goal) const
{
    if (!has_last_goal_)
    {
        return true;
    }

    double dx = goal.pose.position.x - last_goal_.pose.position.x;
    double dy = goal.pose.position.y - last_goal_.pose.position.y;
    double dz = goal.pose.position.z - last_goal_.pose.position.z;

    double distance_sq = dx * dx + dy * dy + dz * dz;
    return distance_sq > (goal_change_tolerance_ * goal_change_tolerance_);
}

void MinimumSnapPlanner::clearActiveWaypoints()
{
    std::lock_guard<std::mutex> lock(waypoint_mutex_);
    if (!active_waypoints_.empty())
    {
        ROS_INFO("[WaypointSnapPlanner] Clearing %lu active waypoint(s) after planning.",
                 active_waypoints_.size());
        active_waypoints_.clear();
    }
}

} // namespace minimum_snap_planner

