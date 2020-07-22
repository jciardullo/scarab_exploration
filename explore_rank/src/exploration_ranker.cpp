#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/Int32.h"

float prev_x;
float prev_y;
double distance;
int total_cells = 0;
std_msgs::Int8MultiArray prev_map;
ros::Publisher cell_pub;

void posCallback(const nav_msgs::OdometryConstPtr& odom_msg)
{
  float curr_x = odom_msg->pose.pose.position.x;
  float curr_y = odom_msg->pose.pose.position.y;
  distance += sqrt((curr_x - prev_x) * (curr_x - prev_x) +
              (curr_y - prev_y) * (curr_y - prev_y));
  prev_x = curr_x;
  prev_y = curr_y;
}

void mapCallback(const nav_msgs::OccupancyGridConstPtr& map_msg)
{
  if (prev_map.data.empty()) {
    ROS_INFO("Exploration has begun");;
    prev_map.data = map_msg->data;
    for (int i = 0; i < boost::size(prev_map.data); i++) {
      prev_map.data[i] = -1;
    }
    std_msgs::Int32 cell_msg;
    cell_msg.data = 0;
    cell_pub.publish(cell_msg);
    ROS_INFO("Waiting for map generation to be finished...");
  }
    std_msgs::Int8MultiArray map;
    map.data = map_msg->data;
    for (int i = 0; i < boost::size(map.data); i++) {
      if (map.data[i] != -1 && prev_map.data[i] == -1) {
        total_cells++;
      }
    }
    prev_map = map;
    std_msgs::Int32 cell_msg;
    cell_msg.data = total_cells;
    cell_pub.publish(cell_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ROS_INFO("Starting exploration ranker");

  ros::Subscriber pos_sub = n.subscribe("husky/odometry/filtered", 2, posCallback);
  ros::Subscriber map_sub = n.subscribe("husky/map", 2, mapCallback);

  cell_pub = n.advertise<std_msgs::Int32>("grid_total", 100);

  ros::AsyncSpinner spinner(2);
  spinner.start();

  ROS_INFO("Waiting for exploration to start");

  boost::shared_ptr<const std_msgs::String> completion_time = ros::topic::waitForMessage<std_msgs::String>("husky/explore/chatter");
  printf("-------------------------------------------------------------------------------\n");
  ROS_INFO("Completion Time: %s seconds", completion_time->data.c_str());

  spinner.stop();
  pos_sub.shutdown();
  map_sub.shutdown();
  ROS_INFO("Distance Traveled: %f m", distance);

  boost::shared_ptr<const nav_msgs::OccupancyGrid> map_msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("husky/map");
  std_msgs::Int8MultiArray map;
  map.data = map_msg->data;

  boost::shared_ptr<const nav_msgs::OccupancyGrid> true_map_msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("true_map");
  std_msgs::Int8MultiArray true_map;
  true_map.data = true_map_msg->data;

  int count = 0;
  int total = 0;
  for (int i = 0; i < boost::size(true_map.data); i++) {
    //We're ignoring common unknown space in the calculations here
    if (map.data[i] != -1 || true_map.data[i] != -1) {
      if (map.data[i] == true_map.data[i]) {
        count++;
      }
      total++;
    }
  }
  double accuracy = (count * 100.0) / total;
  ROS_INFO("Accuracy of exploration-generated map with true map: %f%%", accuracy);
  ROS_INFO("Amount of grid cells discovered: %d", total_cells);

  ros::spin();

  return 0;
}
