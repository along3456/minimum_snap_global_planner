#ifndef MINIMUM_SNAP_PLANNER_H
#define MINIMUM_SNAP_PLANNER_H

#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/buffer.h>

#include <global_planner/planner_core.h>
#include "minimum_snap_planner/trajectory/minimum_snap/minimum_snap.h"

#include <vector>
#include <memory>
#include <mutex>

namespace minimum_snap_planner
{

class MinimumSnapPlanner : public nav_core::BaseGlobalPlanner
{
public:
    MinimumSnapPlanner();
    MinimumSnapPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    ~MinimumSnapPlanner();

    /**
     * @brief Initialize the planner
     * @param name The name of the planner
     * @param costmap_ros A pointer to the ROS wrapper of the costmap
     */
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;

    /**
     * @brief Make a plan from start to goal
     * @param start The start pose
     * @param goal The goal pose
     * @param plan The plan (output)
     * @return True if a valid plan was found
     */
    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan) override;

private:
    /**
     * @brief Callback for waypoint updates
     * @param msg The waypoint array message
     */
    void waypointCallback(const geometry_msgs::PoseArray::ConstPtr& msg);

    /**
     * @brief Plan multi-segment path using Navigation stack A* implementation
     * @param start Start pose
     * @param waypoints Intermediate waypoint poses
     * @param goal Goal pose
     * @param out_points Output sequence of 2D coordinates
     */
    bool planMultiSegmentPath(
        const geometry_msgs::PoseStamped& start,
        const std::vector<geometry_msgs::PoseStamped>& waypoints,
        const geometry_msgs::PoseStamped& goal,
        std::vector<Eigen::Vector2d>& out_points);

    /**
     * @brief Downsample path points
     * @param path Original path
     * @param resolution Downsampling resolution
     * @return Downsampled path
     */
    std::vector<Eigen::Vector2d> downsamplePath(
        const std::vector<Eigen::Vector2d>& path,
        double resolution);

    /**
     * @brief Generate smooth trajectory using Minimum Snap
     * @param waypoints Waypoints for trajectory
     * @param plan Output plan
     * @return True if successful
     */
    bool generateMinimumSnapTrajectory(
        const std::vector<Eigen::Vector2d>& waypoints,
        std::vector<geometry_msgs::PoseStamped>& plan);

    /**
     * @brief Evaluate polynomial at time t
     * @param coeffs Polynomial coefficients
     * @param t Time
     * @return Position value
     */
    double evaluatePolynomial(const VecXd& coeffs, double t);

    /**
     * @brief Retrieve the waypoint list that should be applied for the current planning request
     */
    std::vector<geometry_msgs::Pose> loadWaypointsForGoal(const geometry_msgs::PoseStamped& goal);

    void robotPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void robotPoseCovCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void handleRobotPose(const geometry_msgs::Pose& pose);
    void publishActiveWaypointsLocked();

    // Member variables
    costmap_2d::Costmap2DROS* costmap_ros_;
    costmap_2d::Costmap2D* costmap_;
    std::shared_ptr<global_planner::GlobalPlanner> global_planner_;
    std::shared_ptr<MinimumSnap> minimum_snap_ptr_;

    std::vector<geometry_msgs::Pose> active_waypoints_;
    std::vector<geometry_msgs::Pose> pending_waypoints_;
    bool pending_waypoints_available_;
    mutable std::mutex waypoint_mutex_;
    ros::Subscriber waypoint_sub_;
    ros::Publisher plan_pub_;
    ros::Subscriber robot_pose_sub_;
    ros::Subscriber robot_pose_cov_sub_;
    ros::Publisher waypoint_feedback_pub_;
    
    bool initialized_;
    std::string global_frame_;
    
    // Parameters
    double downsample_resolution_;
    double max_vel_;
    double max_accel_;
    unsigned int polynomial_order_;
    double trajectory_time_step_;
    std::string waypoint_topic_;
    std::string waypoint_feedback_topic_;
    std::string robot_pose_topic_;
    std::string robot_pose_type_;
    double waypoint_reached_tolerance_;
    bool use_covariance_pose_;
};

} // namespace minimum_snap_planner

#endif // MINIMUM_SNAP_PLANNER_H

