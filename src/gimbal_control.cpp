#include <ros/ros.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

using namespace std;

void boundingBoxCallback(const darknet_ros_msgs::BoundingBoxesConstPtr &msg);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gimbal_control");
  ros::NodeHandle nodeHandle;
  ros::Subscriber sub = nodeHandle.subscribe("/darknet_ros/bounding_boxes", 1000, boundingBoxCallback);

  cout << "Start gimbal control" << endl;

  ros::spin();
  return 0;
}

void boundingBoxCallback(const darknet_ros_msgs::BoundingBoxesConstPtr &msg)
{
  auto boxes = msg->bounding_boxes;
  darknet_ros_msgs::BoundingBox max_prob_box;
  for (auto box : boxes) {
    if (box.probability > max_prob_box.probability) {
      max_prob_box = box;
    }
  }
  if (max_prob_box.probability > 0) {
    float x_center = (max_prob_box.xmin + max_prob_box.xmax) / 2;
    float y_center = (max_prob_box.ymin + max_prob_box.ymax) / 2;
    cout << max_prob_box.probability << " " << (max_prob_box.xmin + max_prob_box.xmax) / 2 << " " << (max_prob_box.ymin + max_prob_box.ymax) / 2 << endl;
  }
}
