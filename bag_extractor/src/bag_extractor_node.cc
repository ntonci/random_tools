#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <boost/filesystem.hpp>

std::string point_cloud_topic;
std::string global_path;
bool use_binary;

ros::Subscriber point_cloud_sub;
size_t global_counter = 0u;

void convertPointCloud2MsgToPCL(
    const boost::shared_ptr<const sensor_msgs::PointCloud2>& sensor_msg,
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcl_cloud) {
  CHECK(pcl_cloud);

  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*sensor_msg, pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2, *pcl_cloud);
}

void pointCloudCallback(
    const boost::shared_ptr<const sensor_msgs::PointCloud2>& input) {
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_cloud(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
  convertPointCloud2MsgToPCL(input, input_cloud);
  std::string file_name = global_path + std::to_string(global_counter) + ".ply";
  if (use_binary) {
    pcl::io::savePLYFile(file_name, *input_cloud);
  } else {
    pcl::io::savePLYFileASCII(file_name, *input_cloud);
  }
  LOG(INFO) << "Saved " + file_name + " pointcloud";
  ++global_counter;
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
  nodeHandle.param<std::string>(
      "global_path", global_path,
      "/Users/ntonci/Desktop/kaiser_rs50n_bags/bags/clutter/");
  nodeHandle.param<std::bool>("use_binary", use_binary, false);

  // POINTCLOUD SUBCRIBER
  point_cloud_sub =
      nodeHandle.subscribe(point_cloud_topic, 1000, &pointCloudCallback);

  ros::Rate loop_rate(10);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
