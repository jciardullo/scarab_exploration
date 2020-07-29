#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "ros/package.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/String.h"
#include "boost/foreach.hpp"
#include "boost/algorithm/string.hpp"
#include "matplotlibcpp.h"
#define foreach BOOST_FOREACH

namespace plt = matplotlibcpp;

double distance;
std::vector<std::tuple<float, float>> position (1);
int total_cells = 0;
std_msgs::Int8MultiArray prev_map;
std::vector<double> ros_time;
std::vector<double> cells;

void positionTrack(const nav_msgs::OdometryConstPtr& odom_msg)
{
  float curr_x = odom_msg->pose.pose.position.x;
  float curr_y = odom_msg->pose.pose.position.y;
  float prev_x = std::get<0>(position.back());
  float prev_y = std::get<1>(position.back());
  distance += sqrt((curr_x - prev_x) * (curr_x - prev_x) +
  (curr_y - prev_y) * (curr_y - prev_y));
  position.push_back(std::make_tuple(curr_x, curr_y));
}

void mapTrack(const nav_msgs::OccupancyGridConstPtr& map_msg)
{
  if (prev_map.data.empty()) {
    prev_map.data.resize(map_msg->data.size());
    std::fill(prev_map.data.begin(), prev_map.data.end(), -1);
    ros_time.push_back(0);
    cells.push_back(0);
  }
  std_msgs::Int8MultiArray map;
  map.data = map_msg->data;
  for (int i = 0; i < boost::size(map.data); i++) {
    if (map.data[i] != -1 && prev_map.data[i] == -1) {
      total_cells++;
    }
  }
  prev_map = map;
  cells.push_back(total_cells);
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

  ros::init(argc, argv, "test");
  ros::NodeHandle nh("~");

  std::string bag_file_names;
  std::string bag_directory;
  nh.getParam("bag_file_names", bag_file_names);
  nh.getParam("bag_directory", bag_directory);
  std::vector<std::string> bag_files;

  boost::split(bag_files, bag_file_names, boost::is_any_of(" "));

  plt::figure_size(1500, 900);

  foreach (std::string filename, bag_files) {

    printf("\n");
    printf("-------------------------------------------------------------------------------\n");
    ROS_INFO("Starting exploration ranker for %s", filename.c_str());

    rosbag::Bag bag(bag_directory + filename, rosbag::bagmode::Read);

    rosbag::View view(bag);

    bool finish = false;

    ROS_INFO("Calculating metrics...");

    foreach (rosbag::MessageInstance const m, view)
    {
      std::string topic_name = m.getTopic().c_str();
      if (topic_name.compare("/husky/odometry/filtered") == 0) {
        const nav_msgs::OdometryConstPtr& odom_msg = m.instantiate<nav_msgs::Odometry>();
        positionTrack(odom_msg);
      }
      if (topic_name.compare("/husky/map") == 0) {
        const nav_msgs::OccupancyGridConstPtr& map_msg = m.instantiate<nav_msgs::OccupancyGrid>();
        mapTrack(map_msg);
        ros_time.push_back(m.getTime().toSec());
      }
      if (topic_name.compare("/husky/explore/chatter") == 0) {
        const std_msgs::StringConstPtr& completion_time = m.instantiate<std_msgs::String>();
        ROS_INFO("Completion Time: %s seconds", completion_time->data.c_str());
        break;
      }
    }
    bag.close();

    ROS_INFO("Distance Traveled: %f m", distance);

    bool retracing = false;
    double retraced_distance = 0;
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

    boost::shared_ptr<const nav_msgs::OccupancyGrid> true_map_msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("true_map");
    std_msgs::Int8MultiArray true_map;
    true_map.data = true_map_msg->data;

    int count = 0;
    int total = 0;
    for (int i = 0; i < boost::size(true_map.data); i++) {
      //We're ignoring common unknown space in the calculations here
      if (prev_map.data[i] != -1 || true_map.data[i] != -1) {
        if (prev_map.data[i] == true_map.data[i]) {
          count++;
        }
        total++;
      }
    }

    double accuracy = (count * 100.0) / total;
    ROS_INFO("Accuracy of exploration-generated map with true map: %f%%", accuracy);

    plt::named_plot(filename.c_str(), ros_time, cells);
    ROS_INFO("Amount of total grid cells discovered: %d", total_cells);

    ROS_INFO("Ranking of %s completed", filename.c_str());

    position.clear();
    distance = 0;
    total_cells = 0;
    prev_map.data.clear();
    ros_time.clear();
    cells.clear();
  }

  plt::legend();
  plt::title("Total map generation during simulation");
  plt::xlabel("Time (seconds)");
  plt::ylabel("Grid cells");
  double* x_limits = plt::xlim();
  double* y_limits = plt::ylim();
  plt::xlim(0.0, *(x_limits + 1));
  plt::ylim(0.0, *(y_limits + 1));
  plt::grid(true);

  plt::save(bag_directory + "bag_map_generation.png");
  plt::show();

  printf("\n");
  ros::shutdown();
}
