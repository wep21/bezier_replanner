#include <ros/ros.h>

#include "bezier_replanner.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "bezier_replanner");
  BezierReplannerNode brn;
  ros::spin();

  return 0;
}