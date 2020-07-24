#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/Int32.h"
#include "ros/callback_queue.h"

double distance;
std::vector<std::tuple<float, float>> position (1);
int total_cells = 0;
std_msgs::Int8MultiArray prev_map;
ros::Publisher cell_pub;

void posCallback(const nav_msgs::OdometryConstPtr& odom_msg)
{
  float curr_x = odom_msg->pose.pose.position.x;
  float curr_y = odom_msg->pose.pose.position.y;
  float prev_x = std::get<0>(position.back());
  float prev_y = std::get<1>(position.back());
  distance += sqrt((curr_x - prev_x) * (curr_x - prev_x) +
              (curr_y - prev_y) * (curr_y - prev_y));
  position.push_back(std::make_tuple(curr_x, curr_y));
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

bool inRadius(std::tuple<float, float> pos1, std::tuple<float, float> pos2, float range) {
  float x1 = std::get<0>(pos1);
  float x2 = std::get<0>(pos2);
  float y1 = std::get<1>(pos1);
  float y2 = std::get<1>(pos2);
  return ((x1 >= x2 - range && x1 <= x2 + range) && (y1 >= y2 - range && y1 <= y2 + range));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;
  ros::CallbackQueue queue;
  n.setCallbackQueue(&queue);

  ROS_INFO("Starting exploration ranker");

  ros::Subscriber pos_sub = n.subscribe("husky/odometry/filtered", 0, posCallback);
  ros::Subscriber map_sub = n.subscribe("husky/map", 0, mapCallback);

  cell_pub = n.advertise<std_msgs::Int32>("grid_total", 0);

  ros::AsyncSpinner spinner(0, &queue);
  spinner.start();

  ROS_INFO("Waiting for exploration to start");

  boost::shared_ptr<const std_msgs::String> completion_time = ros::topic::waitForMessage<std_msgs::String>("husky/explore/chatter");
  printf("-------------------------------------------------------------------------------\n");
  ROS_INFO("Completion Time: %s seconds", completion_time->data.c_str());

  queue.disable();
  pos_sub.shutdown();
  map_sub.shutdown();

  while (!queue.isEmpty()) {
      queue.callAvailable();
  }

  spinner.stop();

  ROS_INFO("Distance Traveled: %f m", distance);

  bool retracing = false;
  double retraced_distance;
  std::vector<std::tuple<float, float>>::reverse_iterator bound = position.rend();
  //Loop that accounts for and logs previously traversed areas
  for (int i = 1; i < position.size(); i++) {
    std::tuple<float, float> pos2 = position[i];
    //Find statement that checks backwards from the current position in the list to find the most recent occurence
    // of a position not within the robot radius (e.g. a past location)
    bound =
        std::find_if(position.rbegin() + (position.size() - i), bound, [&pos2](std::tuple<float, float> pos1){ return !inRadius(pos1, pos2, 0.5); });
    if (bound != position.rend()) {
    //Find statement that checks within the range of the origin and the first location outside the robot radius
    //to find an area currently within range of the robot radius (e.g. a past location the robot is currently near)
      std::vector<std::tuple<float, float>>::reverse_iterator result =
          std::find_if(bound + 1, position.rend(), [&pos2](std::tuple<float, float> pos1){ return inRadius(pos1, pos2, 0.5); });
      if (result != position.rend()) {
        //We don't add the retraced distance until we have a second occurence of retracing (e.g. when the bool 'retracing' is true, it means our last iteration was a retrace)
        if (retracing) {
          float curr_x = std::get<0>(position[i]);
          float curr_y = std::get<1>(position[i]);
          float prev_x = std::get<0>(position[i - 1]);
          float prev_y = std::get<1>(position[i - 1]);
          retraced_distance += sqrt((curr_x - prev_x) * (curr_x - prev_x) +
                      (curr_y - prev_y) * (curr_y - prev_y));
        } else {
          retracing = true;
        }
      } else {
        retracing = false;
      }
    }
  }

  ROS_INFO("Retraced: %f m", retraced_distance);

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
