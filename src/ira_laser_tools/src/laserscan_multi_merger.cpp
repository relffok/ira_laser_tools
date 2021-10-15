#include "ira_laser_tools/laserscan_multi_merger.hpp"

#include <algorithm>
#include <string>
#include <limits>
#include <memory>
#include <map>
#include <vector>
#include <utility>

namespace laserscan_multi_merger
{
LaserscanMerger::LaserscanMerger()
: Node("laser_multi_merger")
{
  destination_frame_ = this->declare_parameter<std::string>("destination_frame", "dest_link");
  cloud_destination_topic_ = this->declare_parameter<std::string>(
    "cloud_destination_topic",
    "/merged_cloud");
  scan_destination_topic_ = this->declare_parameter<std::string>(
    "scan_destination_topic",
    "/merged_scan");
  laserscan_topics_ = this->declare_parameter<std::string>("laserscan_topics", "");
  min_height_ = this->declare_parameter("min_height", std::numeric_limits<double>::min());
  max_height_ = this->declare_parameter("max_height", std::numeric_limits<double>::max());
  angle_min_ = this->declare_parameter("angle_min", -M_PI);
  angle_max_ = this->declare_parameter("angle_max", M_PI);
  angle_increment_ = this->declare_parameter("angle_increment", M_PI / 180.0);
  time_increment_ = this->declare_parameter("time_increment", 0);
  scan_time_ = this->declare_parameter("scan_time", 1.0 / 30.0);
  range_min_ = this->declare_parameter("range_min", 0.0);
  range_max_ = this->declare_parameter("range_max", std::numeric_limits<double>::max());
  use_inf_ = this->declare_parameter("use_inf", true);
  inf_epsilon_ = this->declare_parameter("inf_epsilon", 1.0);
  best_effort_enabled_ = this->declare_parameter<bool>("best_effort", true);
  max_completion_time_ = this->declare_parameter<double>("max_completion_time", 0.05);
  allow_scan_delay_ = this->declare_parameter("allow_scan_delay", false);
  max_delay_time_sec_ = this->declare_parameter("max_delay_scan_time", 1.0);
  max_merge_time_diff_sec_ = this->declare_parameter("max_merge_time_diff", 0.05);

  callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&LaserscanMerger::parametersCallback, this, std::placeholders::_1));

  topic_parser_timer = this->create_wall_timer(
    std::chrono::seconds(5), std::bind(&LaserscanMerger::laserscan_topic_parser, this));

  this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  this->tfListener_ = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_);

  point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    this->cloud_destination_topic_.c_str(), rclcpp::SystemDefaultsQoS());
  laser_scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
    this->scan_destination_topic_.c_str(), rclcpp::SystemDefaultsQoS());
}

rcl_interfaces::msg::SetParametersResult LaserscanMerger::parametersCallback(
  const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  result.reason = "";

  for (const auto &param : parameters)
  {
    if (param.get_name() == "best_effort" &&
        param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
      {
        best_effort_enabled_ = param.as_bool();
        result.successful = true;
      } else if (param.get_name() == "alow_scan_delay" &&
                 param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL){
        allow_scan_delay_ = param.as_bool();
        result.successful = true;
      } else if (param.get_name() == "max_delay_scan_time" &&
                 param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE){
        max_delay_time_sec_ = param.as_double();
        result.successful = true;
      } else if (param.get_name() == "max_merge_time_diff" &&
                 param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE){
        max_merge_time_diff_sec_ = param.as_double();
        result.successful = true;
      } else if (param.get_name() == "max_completion_time" &&
                 param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE){
        max_completion_time_ = param.as_double();
        result.successful = true;
      } else if (param.get_name() == "destination_frame" &&
                 param.get_type() == rclcpp::ParameterType::PARAMETER_STRING){
        destination_frame_ = param.as_string();
        result.successful = true;
      } else if (param.get_name() == "cloud_destination_topic" &&
                 param.get_type() == rclcpp::ParameterType::PARAMETER_STRING){
        cloud_destination_topic_ = param.as_string();
        result.successful = true;
      } else if (param.get_name() == "scan_destination_topic" &&
                 param.get_type() == rclcpp::ParameterType::PARAMETER_STRING){
        scan_destination_topic_ = param.as_string();
        result.successful = true;
      } else if (param.get_name() == "laserscan_topics" &&
                 param.get_type() == rclcpp::ParameterType::PARAMETER_STRING){
        laserscan_topics_ = param.as_string();
        result.successful = true;
      } else if (param.get_name() == "angle_min" &&
                 param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE){
        angle_min_ = param.as_double();
        result.successful = true;
      } else if (param.get_name() == "angle_max" &&
                 param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE){
        angle_max_ = param.as_double();
        result.successful = true;
      } else if (param.get_name() == "angle_increment" &&
                 param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE){
        angle_increment_ = param.as_double();
        result.successful = true;
      } else if (param.get_name() == "time_increment" &&
                 param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE){
        time_increment_ = param.as_double();
        result.successful = true;
      } else if (param.get_name() == "scan_time" &&
                 param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE){
        scan_time_ = param.as_double();
        result.successful = true;
      } else if (param.get_name() == "range_min" &&
                 param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE){
        range_min_ = param.as_double();
        result.successful = true;
      } else if (param.get_name() == "range_max" &&
                 param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE){
        range_max_ = param.as_double();
        result.successful = true;
      } else if (param.get_name() == "inf_epsilon" &&
                 param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE){
        inf_epsilon_ = param.as_double();
        result.successful = true;
      } else if (param.get_name() == "min_height" &&
                 param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE){
        min_height_ = param.as_double();
        result.successful = true;
      } else if (param.get_name() == "max_height_" &&
                 param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE){
        max_height_ = param.as_double();
        result.successful = true;
      } else if (param.get_name() == "use_inf" &&
                 param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL){
        use_inf_ = param.as_bool();
        result.successful = true;
      }
  }

  return result;
}

void LaserscanMerger::laserscan_topic_parser()
{
  RCLCPP_DEBUG( this->get_logger(), "Parsing topics..");
  // Necessity for sleep workaround to get topics
  // https://github.com/ros2/ros2/issues/1057
  std::map<std::string, std::vector<std::string>> topics;
  topics = this->get_topic_names_and_types();  // first:name, second:type

  std::vector<std::string> published_scan_topics;
  for (auto const & topic : topics) {
    if (topic.second[0].compare("sensor_msgs/msg/LaserScan") == 0) {
      std::string topic_name = topic.first;
      if (topic_name.at(0) == '/') {
        topic_name = topic_name.substr(1);
      }
      published_scan_topics.push_back(topic_name);
    }
  }

  if (published_scan_topics.size() == 0)
    return;

  std::istringstream iss(this->laserscan_topics_);
  std::vector<std::string> tokens((std::istream_iterator<std::string>(
    iss)), std::istream_iterator<std::string>());
  std::vector<std::string> tmp_input_topics;

  // make sure missing topics are published LaserScan topics
  for (auto const & token : tokens) {
    if (std::find(
        subscribed_topics.begin(), subscribed_topics.end(),
        token) == subscribed_topics.end())
    {
      if (std::find(
          published_scan_topics.begin(), published_scan_topics.end(),
          token) != published_scan_topics.end())
      {
        tmp_input_topics.push_back(token);
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "Topic %s [sensor_msg/LaserScan] does not seem to be published yet. Could not subscribe.",
          token.c_str());
      }
    }
  }

  // clean up duplicate topics
  std::sort(tmp_input_topics.begin(), tmp_input_topics.end());
  std::vector<std::string>::iterator last =
    std::unique(tmp_input_topics.begin(), tmp_input_topics.end());
  tmp_input_topics.erase(last, tmp_input_topics.end());
  input_topics = tmp_input_topics;

  // Create subscriptions
  if (input_topics.size() > 0) {
    scan_subscribers.resize(input_topics.size());
    std::stringstream output_info;
    std::copy(
      input_topics.begin(), input_topics.end(),
      std::ostream_iterator<std::string>(output_info, " "));
    RCLCPP_INFO(
      this->get_logger(),
      "Subscribing to %ld topics.", scan_subscribers.size());
    for (uint i = 0; i < input_topics.size(); ++i) {
      // workaround for std::bind https://github.com/ros2/rclcpp/issues/583
      std::function<void(const sensor_msgs::msg::LaserScan::SharedPtr)> callback =
        std::bind(
        &LaserscanMerger::scanCallback,
        this, std::placeholders::_1, input_topics[i]);
      scan_subscribers[i] = this->create_subscription<
        sensor_msgs::msg::LaserScan>(input_topics[i].c_str(),
        rclcpp::SensorDataQoS(), callback);
      RCLCPP_INFO(
        this->get_logger(),
        "Subscribed to %s.", input_topics[i].c_str());
      subscribed_topics.push_back(input_topics[i]);
    }
  }
}

void LaserscanMerger::scanCallback(
  const sensor_msgs::msg::LaserScan::SharedPtr scan,
  std::string topic)
{
  RCLCPP_DEBUG(
  this->get_logger(),
  "Scan callback of %s, merging.. ", topic.c_str());

  if (!this->allow_scan_delay_ && is_scan_too_old(scan->header.stamp)){
    RCLCPP_DEBUG(
      this->get_logger(),
      "Scan data of %s too old, dicarding scan.. ", topic.c_str());
    return;
  } else {
    laser_scan_to_cloud_deque(scan, topic);
    update_cloud_queue();
    // TODO: clean up old data
  }
}

bool LaserscanMerger::is_scan_too_old(const builtin_interfaces::msg::Time stamp_time){
  double time_diff = abs((this->get_clock()->now() - stamp_time).seconds());
  if(time_diff > this->max_delay_time_sec_)
    return true;
  else
    return false;
}

void LaserscanMerger::laser_scan_to_cloud_deque(
  const sensor_msgs::msg::LaserScan::SharedPtr scan,
  std::string topic)
{
  pcl::PCLPointCloud2 pcl_cloud;
  auto singleScanCloud = laser_scan_to_pointcloud(scan);
  pcl_conversions::toPCL(*singleScanCloud, pcl_cloud);
  int topic_index = get_topic_index(topic);

  int pile_index = get_matching_pile(topic_index, scan->header.stamp);

  if (pile_index >= 0){
    cloud_deque[pile_index].add(topic_index, pcl_cloud);
  } else {
    this->cloud_deque.push_back(
      CloudPile(pcl_cloud, scan->header.stamp,
                this->get_clock()->now(),
                topic_index, subscribed_topics.size()));
  }
}

int LaserscanMerger::get_matching_pile(int topic_index, builtin_interfaces::msg::Time time_stamp)
{
for (uint i=0; i<cloud_deque.size(); i++)
  {
    if(cloud_deque[i].is_index_filled(topic_index))
    {
      continue;
    } else {
      double time_diff = abs((cloud_deque[i].get_stamp_time() - time_stamp).seconds());
      if (time_diff < this->max_merge_time_diff_sec_)
        return i;
    }
  }
  return -1;   // no matching index found
}

void LaserscanMerger::update_cloud_queue(){
  if (cloud_deque.front().is_complete()){
    publish_latest_cloud_and_scan();
  } else if(abs((this->get_clock()->now() - cloud_deque.front().get_creation_time()).seconds()) >
          max_completion_time_){
    if(this->best_effort_enabled_)
      publish_latest_cloud_and_scan();
    else
      cloud_deque.pop_front();
  }
}


int LaserscanMerger::get_topic_index(std::string topic){
  for (uint i = 0; i < subscribed_topics.size(); ++i) {
    if (topic.compare(subscribed_topics[i]) == 0) {
      return i;
    }
  }
  return -1;
}

void LaserscanMerger::publish_latest_cloud_and_scan(){
  auto front_pile = cloud_deque.front();
  pcl::PCLPointCloud2 merged_cloud_pcl = front_pile.merge_to_one_cloud();

  auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl_conversions::moveFromPCL(merged_cloud_pcl, *cloud_msg);

  cloud_deque.pop_front();

  this->point_cloud_publisher_->publish(*cloud_msg);
  auto merged_scan_msg = pointcloud_to_laserscan(cloud_msg);
    if (merged_scan_msg){
      this->laser_scan_publisher_->publish(std::move(merged_scan_msg));
    }
}

/* Basic conversions */

sensor_msgs::msg::PointCloud2::SharedPtr LaserscanMerger::laser_scan_to_pointcloud(
  const sensor_msgs::msg::LaserScan::SharedPtr scan){
  RCLCPP_DEBUG(this->get_logger(), "Transforming laserscan to pointcloud..");

  auto singleScanCloud = std::make_shared<sensor_msgs::msg::PointCloud2>();

  // transform scan if necessary
  // if (scan->header.frame_id != destination_frame_){}
  tf2::TimePoint time_point = tf2_ros::fromMsg(scan->header.stamp);
  bool canTransform = tf_buffer_->canTransform(
    destination_frame_.c_str(), scan->header.frame_id.c_str(), time_point,
    tf2::durationFromSec(1.0));
  if (!canTransform) {
    RCLCPP_WARN(
      this->get_logger(),
      "Could not look up transform from %s to %s at time [%d.%d]",
      scan->header.frame_id.c_str(), destination_frame_.c_str(),
      scan->header.stamp.sec, scan->header.stamp.nanosec);
  } else {
    RCLCPP_DEBUG(
      this->get_logger(),
      "Transform available");
  }
  projector_.transformLaserScanToPointCloud(
    destination_frame_.c_str(), *scan, *singleScanCloud, *tf_buffer_);

  return singleScanCloud;
}

sensor_msgs::msg::LaserScan::UniquePtr LaserscanMerger::pointcloud_to_laserscan(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
{
  RCLCPP_DEBUG(this->get_logger(),
    "Transforming from pointcloud to laserscan..");

  // https://github.com/ros-perception/pointcloud_to_laserscan/blob/554173b73f7b5fd2e4d9218b1096cb4c7dcbae1c/src/pointcloud_to_laserscan_node.cpp#L137
  // build laserscan output
  auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
  // check if cloud_msg is available
  if (!cloud_msg)
    return scan_msg;

  scan_msg->header = cloud_msg->header;
  scan_msg->angle_min = this->angle_min_;
  scan_msg->angle_max = this->angle_max_;
  scan_msg->angle_increment = this->angle_increment_;
  scan_msg->time_increment = this->time_increment_;
  scan_msg->scan_time = this->scan_time_;
  scan_msg->range_min = this->range_min_;
  scan_msg->range_max = this->range_max_;

  // determine amount of rays to create
  uint32_t ranges_size = std::ceil(
    (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);

  if (this->use_inf_) {
    scan_msg->ranges.assign(
      ranges_size,
      std::numeric_limits<double>::infinity());
  } else {
    scan_msg->ranges.assign(ranges_size, scan_msg->range_max + inf_epsilon_);
  }

  // Iterate through pointcloud
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x"),
    iter_y(*cloud_msg, "y"), iter_z(*cloud_msg, "z");
    iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for nan in point(%f, %f, %f)\n",
        *iter_x, *iter_y, *iter_z);
      continue;
    }

    if (*iter_z > max_height_ || *iter_z < min_height_) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for height %f not in range (%f, %f)\n",
        *iter_z, min_height_, max_height_);
      continue;
    }

    double range = hypot(*iter_x, *iter_y);
    if (range < range_min_) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for range %f below minimum value %f. Point: (%f, %f, %f)",
        range, range_min_, *iter_x, *iter_y, *iter_z);
      continue;
    }
    if (range > range_max_) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for range %f above maximum value %f. Point: (%f, %f, %f)",
        range, range_max_, *iter_x, *iter_y, *iter_z);
      continue;
    }

    double angle = atan2(*iter_y, *iter_x);
    if (angle < scan_msg->angle_min || angle > scan_msg->angle_max) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for angle %f not in range (%f, %f)\n",
        angle, scan_msg->angle_min, scan_msg->angle_max);
      continue;
    }

    // overwrite range at laserscan ray if new range is smaller
    int index = (angle - scan_msg->angle_min) / scan_msg->angle_increment;
    if (range < scan_msg->ranges[index]) {
      scan_msg->ranges[index] = range;
    }
  }

  return scan_msg;

  RCLCPP_INFO(
    this->get_logger(), "Published LaserScan at time");
}

}  // namespace laserscan_multi_merger


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<laserscan_multi_merger::LaserscanMerger>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
