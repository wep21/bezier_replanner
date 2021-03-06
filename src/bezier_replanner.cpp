#include "bezier_replanner.h"
#include "Bezier.h"

BezierReplannerNode::BezierReplannerNode()
    : private_nh_("~"), replanning_mode_(false) {
  private_nh_.param<std::string>("src_lane", src_lane_, "/based/lane_waypoints_raw");
  private_nh_.param<std::string>("dst_lane", dst_lane_,
                            "/based/lane_waypoints_array");
  private_nh_.param<int>("step", step_, 5);
  private_nh_.param<int>("resampling_num", resampling_num_, 10);
  lane_pub_ = nh_.advertise<autoware_msgs::LaneArray>(dst_lane_, 10, true);
  lane_sub_ =
      nh_.subscribe(src_lane_, 1, &BezierReplannerNode::laneCallback, this);
  // set dynamic reconfigure
  dynamic_reconfigure::Server<bezier_replanner::bezierReplannerConfig>::CallbackType cb;
  cb = boost::bind(&BezierReplannerNode::configCallback, this, _1, _2);
  server.setCallback(cb);
}
BezierReplannerNode::~BezierReplannerNode() {}

void BezierReplannerNode::replan(autoware_msgs::LaneArray &lane_array) {
  for (auto &el : lane_array.lanes) {
      el = bezierReplan(el, step_, resampling_num_);
  }
}

void BezierReplannerNode::configCallback(bezier_replanner::bezierReplannerConfig &config, uint32_t level){
    src_lane_ = config.src_lane;
    dst_lane_ = config.dst_lane;
    step_ = config.step;
    resampling_num_ = config.resampling_num;
    replanning_mode_ = config.replanning_mode;
    lane_pub_ = nh_.advertise<autoware_msgs::LaneArray>(dst_lane_, 10, true);
    lane_sub_ = nh_.subscribe(src_lane_, 1, &BezierReplannerNode::laneCallback, this);
    publishLaneArray();
}


void BezierReplannerNode::publishLaneArray() {
  autoware_msgs::LaneArray array(lane_array_);
  if (replanning_mode_) {
    replan(array);
  }
  lane_pub_.publish(array);
}

void BezierReplannerNode::laneCallback(
    const autoware_msgs::LaneArray::ConstPtr &lane_array) {
  lane_array_ = *lane_array;
  publishLaneArray();
}

Eigen::MatrixXd BezierReplannerNode::generateControlPoints(const Eigen::MatrixXd &points) {
  int p_size = points.rows();
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(2 * (p_size - 1), 2 * (p_size - 1));
  Eigen::MatrixXd b = Eigen::MatrixXd::Zero(2 * (p_size - 1), 3);
  A(0, 0) = -2;
  A(0, 1) = 1;
  A(2 * p_size - 3, 2 * p_size - 4) = 1;
  A(2 * p_size - 3, 2 * p_size - 3) = -2;
  b(0, 0) = -points(0, 0);
  b(0, 1) = -points(0, 1);
  b(0, 2) = -points(0, 2);
  b(2 * p_size - 3, 0) = -points(p_size - 1, 0);
  b(2 * p_size - 3, 1) = -points(p_size - 1, 1);
  b(2 * p_size - 3, 2) = -points(p_size - 1, 2);
  for (int i = 1; i < p_size - 1; i++) {
    A(i * 2 - 1, i * 2 - 1) = 1;
    A(i * 2 - 1, i * 2) = 1;
    A(i * 2, i * 2 - 2) = 1;
    A(i * 2, i * 2 - 1) = -2;
    A(i * 2, i * 2) = 2;
    A(i * 2, i * 2 + 1) = -1;
    b(i * 2 - 1, 0) = 2 * points(i, 0);
    b(i * 2, 0) = 0;
    b(i * 2 - 1, 1) = 2 * points(i, 1);
    b(i * 2, 1) = 0;
    b(i * 2 - 1, 2) = 2 * points(i, 2);
    b(i * 2, 2) = 0;
  }
  Eigen::FullPivLU<Eigen::MatrixXd> lu(A);
  Eigen::MatrixXd x = lu.solve(b);
  return x;
}

autoware_msgs::Lane BezierReplannerNode::bezierReplan(const autoware_msgs::Lane &lane, const int step,
                      const int resampling_num) {
  autoware_msgs::Lane out_lane;
  int wp_size = lane.waypoints.size();
  int last_step = (wp_size - 1) % step;
  Eigen::MatrixXd step_point;
  Eigen::VectorXi step_point_idx;
  Eigen::MatrixXd waypoint = Eigen::MatrixXd::Zero(wp_size, 5);
  if (last_step == 0) {
    step_point = Eigen::MatrixXd::Zero((wp_size - 1) / step + 1, 3);
    step_point_idx = Eigen::VectorXi::Zero((wp_size - 1) / step + 1);
    Eigen::VectorXd velocity = Eigen::VectorXd::Zero((wp_size - 1) / step + 1);
    Eigen::VectorXd change_flag =
    Eigen::VectorXd::Zero((wp_size - 1) / step + 1);
  } else {
    step_point = Eigen::MatrixXd::Zero((wp_size - 1) / step + 2, 3);
    step_point_idx = Eigen::VectorXi::Zero((wp_size - 1) / step + 2);
    Eigen::VectorXd velocity = Eigen::VectorXd::Zero((wp_size - 1) / step + 2);
    Eigen::VectorXd change_flag =
    Eigen::VectorXd::Zero((wp_size - 1) / step + 2);
  }
  int idx_count = 0;
  int p_count = 0;
  for (auto i = lane.waypoints.begin(); i != lane.waypoints.end(); i++) {
    waypoint(idx_count, 0) = i->pose.pose.position.x;
    waypoint(idx_count, 1) = i->pose.pose.position.y;
    waypoint(idx_count, 2) = i->pose.pose.position.z;
    waypoint(idx_count, 3) = i->twist.twist.linear.x;
    waypoint(idx_count, 4) = i->change_flag;
    if (idx_count % step == 0 || i == lane.waypoints.end() - 1) {
      step_point(p_count, 0) = i->pose.pose.position.x;
      step_point(p_count, 1) = i->pose.pose.position.y;
      step_point(p_count, 2) = i->pose.pose.position.z;
      step_point_idx(p_count) = idx_count;
      // velocity(p_count) = i->velocity;
      // change_flag(p_count) = i->change_flag;
      p_count++;
    }
    idx_count++;
  }
  Eigen::MatrixXd control_point = generateControlPoints(step_point);
  std::vector<autoware_msgs::Waypoint> wps;
  for (int i = 0; i < p_count-1; i++) {
    Eigen::Vector3d p1(step_point(i, 0), step_point(i, 1), step_point(i, 2));
    Eigen::Vector3d p2(control_point(2*i, 0), control_point(2*i, 1), control_point(2*i, 2));
    Eigen::Vector3d p3(control_point(2*i+1, 0), control_point(2*i+1, 1), control_point(2*i+1, 2));
    Eigen::Vector3d p4(step_point(i+1, 0), step_point(i+1, 1), step_point(i+1, 2));
    CubicBezier bezier(p1, p2, p3, p4, resampling_num);
    Eigen::MatrixXd path = bezier.getPath();
    for (int j = 0; j < path.rows() ; j++){
        autoware_msgs::Waypoint wp;
        wp.pose.pose.position.x = path(j, 0);
        wp.pose.pose.position.y = path(j, 1);
        wp.pose.pose.position.z = path(j, 2);
        wp.pose.pose.orientation = tf::createQuaternionMsgFromYaw(path(j, 3));
        Eigen::RowVectorXd point(3);
        point << path(j, 0), path(j, 1), path(j, 2);
        Eigen::MatrixXd::Index index;
        (waypoint.block(step_point_idx(i), 0, step_point_idx(i+1) - step_point_idx(i) + 1, 3).rowwise() - point).rowwise().squaredNorm().minCoeff(&index);
        wp.twist.twist.linear.x = waypoint.block(step_point_idx(i), 0, step_point_idx(i+1) - step_point_idx(i) + 1, 5)(index, 3);
        wp.change_flag = int(waypoint.block(step_point_idx(i), 0, step_point_idx(i+1) - step_point_idx(i) + 1, 5)(index, 4));
        wps.emplace_back(wp);

    }
    
  }
  out_lane.header.frame_id = "map";
  out_lane.header.stamp = ros::Time(0);
  out_lane.waypoints = wps;
  return out_lane;
}
