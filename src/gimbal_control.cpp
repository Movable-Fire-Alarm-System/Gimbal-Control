#include <ros/ros.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include "std_msgs/Int64MultiArray.h"
#include "std_msgs/String.h"
#include <cmath>

#define PI 3.14159265358979323846

using namespace std;

ros::Publisher gimbal_pub;
std_msgs::Int64MultiArray gimbal_msg;
// std_msgs::String gimbal_msg;

void boundingBoxCallback(const darknet_ros_msgs::BoundingBoxesConstPtr &msg);

int width = 1280;
int height = 720;

// fov: radian
double h_fov;
double v_fov;

double h_dist;
double v_dist;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gimbal_control");
  ros::NodeHandle nodeHandle;
  ros::Subscriber sub = nodeHandle.subscribe("/darknet_ros/bounding_boxes", 1000, boundingBoxCallback);
  gimbal_pub = nodeHandle.advertise<std_msgs::Int64MultiArray>("/gimbal", 1);
  // gimbal_pub = nodeHandle.advertise<std_msgs::String>("/gimbal", 1);

  h_fov = 69.4 * (PI / 180.0) / 2;
  v_fov = 42.5 * (PI / 180.0) / 2;

  h_dist = (width / 2) / tan(h_fov);
  v_dist = (height / 2) / tan(v_fov);

  cout << "Start gimbal control" << endl;
  cout << "h_dist: " << h_dist << "v_dist: " << v_dist << endl;

  ros::spin();
  return 0;
}

void boundingBoxCallback(const darknet_ros_msgs::BoundingBoxesConstPtr &msg)
{
  auto boxes = msg->bounding_boxes;
  darknet_ros_msgs::BoundingBox max_prob_box;

  for (auto box : boxes)
  {
    if (box.probability > max_prob_box.probability)
    {
      max_prob_box = box;
    }
  }
  if (max_prob_box.probability > 0.95)
  {
    double x_center = (max_prob_box.xmin + max_prob_box.xmax) / 2;
    double y_center = (max_prob_box.ymin + max_prob_box.ymax) / 2;
    double x_diff = x_center - width / 2;
    double y_diff = -(y_center - height / 2);

    double h_angle = atan2(x_diff, h_dist) * 180.0 / PI;
    double v_angle = atan2(y_diff, v_dist) * 180.0 / PI;

    gimbal_msg.data.resize(2);
    // gimbal_msg.data[0] = static_cast<long>(h_angle);
    // gimbal_msg.data[1] = static_cast<long>(v_angle);
    // gimbal_msg.data.push_back((int)h_angle);
    // gimbal_msg.data.push_back((int)v_angle);
    gimbal_msg.data[0] = (int)h_angle;
    gimbal_msg.data[1] = (int)v_angle;
    // cout << (int)h_angle << " " << (int)v_angle << endl;
    cout << gimbal_msg.data.at(0) << " " << gimbal_msg.data.at(1) << endl;
    // gimbal_msg.data = "1";

    gimbal_pub.publish(gimbal_msg);

    // cout << max_prob_box.probability << endl;
    // cout << max_prob_box.probability << " " << h_angle << " " << v_angle << endl;
  }
}
