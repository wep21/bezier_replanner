#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Dense>
#include <autoware_msgs/Lane.h>
#include <autoware_msgs/LaneArray.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <dynamic_reconfigure/server.h>
#include <bezier_replanner/bezierReplannerConfig.h>
class BezierReplannerNode {
public:
  BezierReplannerNode();
  ~BezierReplannerNode();
private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher lane_pub_;
  ros::Subscriber lane_sub_;
  bool replanning_mode_;
  autoware_msgs::LaneArray lane_array_;
  std::string src_lane_;
  std::string dst_lane_;
  dynamic_reconfigure::Server<bezier_replanner::bezierReplannerConfig> server;
  int step_;
  int resampling_num_;
  void replan(autoware_msgs::LaneArray &lane_array);
  void publishLaneArray();
  void setupDynamicReconfigure();
  void configCallback(bezier_replanner::bezierReplannerConfig &config, uint32_t level);
  void laneCallback(const autoware_msgs::LaneArray::ConstPtr &lane_array);
  Eigen::MatrixXd generateControlPoints(const Eigen::MatrixXd &points);
  autoware_msgs::Lane bezierReplan(const autoware_msgs::Lane &lane, const int step,
                                   const int resampling_num);
};

