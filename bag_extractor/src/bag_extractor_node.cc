#include <cv_bridge/cv_bridge.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <image_transport/image_transport.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_listener.h>
#include <boost/filesystem.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

std::string point_cloud_topic;
std::string rgb_topic;
std::string depth_topic;
std::string global_path;
bool use_binary;

ros::Subscriber point_cloud_sub;
ros::Subscriber rgb_sub;
ros::Subscriber depth_sub;
size_t global_counter_pc = 0u;
size_t global_counter_rgb = 0u;
size_t global_counter_depth = 0u;

void convertPointCloud2MsgToPCL(
    const boost::shared_ptr<const sensor_msgs::PointCloud2>& sensor_msg,
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcl_cloud) {
  CHECK(pcl_cloud);

  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*sensor_msg, pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2, *pcl_cloud);
}

void pointCloudCallback(
    const boost::shared_ptr<const sensor_msgs::PointCloud2>& sensor_msg) {
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_cloud(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
  convertPointCloud2MsgToPCL(sensor_msg, input_cloud);
  std::string file_name =
      global_path + "pointcloud/" + std::to_string(global_counter_pc) + ".ply";
  if (use_binary) {
    pcl::io::savePLYFile(file_name, *input_cloud);
  } else {
    pcl::io::savePLYFileASCII(file_name, *input_cloud);
  }
  LOG(INFO) << "Saved " + file_name + " pointcloud";
  ++global_counter_pc;
}

void rgbCallback(const sensor_msgs::ImageConstPtr& sensor_msg) {
  cv::Mat image =
      cv::Mat(cv::Size(sensor_msg->width, sensor_msg->height), CV_8UC3);
  try {
    image = cv_bridge::toCvCopy(sensor_msg, sensor_msg->encoding)->image;
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  std::string file_name =
      global_path + "rgb/" + std::to_string(global_counter_rgb) + ".png";
  if (cv::imwrite(file_name, image)) {
    ROS_INFO("Saved rgb image %d", global_counter_rgb);
  } else {
    ROS_INFO("Could not save rgb image %d", global_counter_rgb);
  }
  ++global_counter_rgb;
}

void depthCallback(const sensor_msgs::ImageConstPtr& sensor_msg) {
  cv::Mat image =
      cv::Mat(cv::Size(sensor_msg->width, sensor_msg->height), CV_16UC1);
  try {
    image = cv_bridge::toCvShare(sensor_msg, sensor_msg->encoding)->image;
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  std::string file_name =
      global_path + "depth/" + std::to_string(global_counter_depth) + ".png";
  if (cv::imwrite(file_name, image)) {
    ROS_INFO("Saved depth image %d", global_counter_depth);
  } else {
    ROS_INFO("Could not save depth image %d", global_counter_depth);
  }
  ++global_counter_depth;
}

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  FLAGS_alsologtostderr = true;

  ros::init(argc, argv, "block_detection_node");
  ros::NodeHandle nodeHandle("~");

  // PARAMS
  nodeHandle.param<std::string>("point_cloud_topic", point_cloud_topic,
                                "/camera/depth_registered/points");
  nodeHandle.param<std::string>("rgb_topic", rgb_topic,
                                "/camera/rgb/image_raw");
  nodeHandle.param<std::string>("depth_topic", depth_topic,
                                "/camera/depth/image_raw");
  nodeHandle.param<std::string>("global_path", global_path,
                                "/Users/ntonci/Desktop/dataset/unpack/");
  nodeHandle.param<bool>("use_binary", use_binary, false);

  // SUBCRIBERS
  point_cloud_sub =
      nodeHandle.subscribe(point_cloud_topic, 1000, &pointCloudCallback);
  rgb_sub = nodeHandle.subscribe(rgb_topic, 1000, &rgbCallback);
  depth_sub = nodeHandle.subscribe(depth_topic, 1000, &depthCallback);

  ros::Rate loop_rate(10);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
