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

bool use_binary;
bool convert_to_uint;
bool flip_bgr_to_rgb;
std::string global_path;

std::string registered_point_cloud_topic;
std::string point_cloud_topic;
ros::Subscriber registered_point_cloud_sub;
ros::Subscriber point_cloud_sub;
size_t global_counter_rpc = 0u;
size_t global_counter_pc = 0u;

std::string rgb_topic;
std::string rgb_rect_topic;
ros::Subscriber rgb_sub;
ros::Subscriber rgb_rect_sub;
size_t global_counter_rgb = 0u;
size_t global_counter_rgb_rect = 0u;

std::string depth_image_topic;
std::string depth_image_raw_topic;
std::string depth_image_rect_topic;
std::string depth_image_rect_raw_topic;
std::string depth_reg_hw_image_rect_topic;
std::string depth_reg_hw_image_rect_raw_topic;
std::string depth_reg_sw_image_rect_topic;
std::string depth_reg_sw_image_rect_raw_topic;
std::string depth_reg_image_topic;
std::string depth_reg_image_raw_topic;
ros::Subscriber depth_image_sub;
ros::Subscriber depth_image_raw_sub;
ros::Subscriber depth_image_rect_sub;
ros::Subscriber depth_image_rect_raw_sub;
ros::Subscriber depth_reg_hw_image_rect_sub;
ros::Subscriber depth_reg_hw_image_rect_raw_sub;
ros::Subscriber depth_reg_sw_image_rect_sub;
ros::Subscriber depth_reg_sw_image_rect_raw_sub;
ros::Subscriber depth_reg_image_sub;
ros::Subscriber depth_reg_image_raw_sub;
size_t global_counter_depth = 0u;
size_t global_counter_depth_raw = 0u;
size_t global_counter_depth_rect = 0u;
size_t global_counter_depth_rect_raw = 0u;
size_t global_counter_depth_hw_rect = 0u;
size_t global_counter_depth_hw_rect_raw = 0u;
size_t global_counter_depth_sw_rect = 0u;
size_t global_counter_depth_sw_rect_raw = 0u;
size_t global_counter_depth_reg = 0u;
size_t global_counter_depth_reg_raw = 0u;

void convertPointCloud2MsgToPCL(
    const boost::shared_ptr<const sensor_msgs::PointCloud2>& sensor_msg,
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcl_cloud) {
  CHECK(pcl_cloud);

  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*sensor_msg, pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2, *pcl_cloud);
}

void saveImage(const sensor_msgs::ImageConstPtr& sensor_msg, const int encoding,
               const size_t counter, const std::string folder,
               const std::string extension) {
  cv::Mat image =
      cv::Mat(cv::Size(sensor_msg->width, sensor_msg->height), encoding);
  try {
    image = cv_bridge::toCvCopy(sensor_msg, sensor_msg->encoding)->image;
    if (sensor_msg->encoding == "rgb8" && flip_bgr_to_rgb) {
      cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
    }
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  if (image.type() == CV_32FC1 && convert_to_uint) {
    image.convertTo(image, CV_16UC1, 255.0);
  }
  std::string file_name =
      global_path + folder + "/" + std::to_string(counter) + extension;
  if (cv::imwrite(file_name, image)) {
    ROS_INFO("Saved image %d", counter);
  } else {
    ROS_INFO("Could not save image %d", counter);
  }
}

void saveCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_cloud,
               const size_t counter, const std::string folder) {
  std::string file_name =
      global_path + folder + "/" + std::to_string(counter) + ".ply";
  if (use_binary) {
    pcl::io::savePLYFile(file_name, *input_cloud);
  } else {
    pcl::io::savePLYFileASCII(file_name, *input_cloud);
  }
}

void registeredPointCloudCallback(
    const boost::shared_ptr<const sensor_msgs::PointCloud2>& sensor_msg) {
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_cloud(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
  convertPointCloud2MsgToPCL(sensor_msg, input_cloud);
  saveCloud(input_cloud, global_counter_rpc, "pc_reg");

  LOG(INFO) << "Saved " + std::to_string(global_counter_rpc) +
                   " reg_pointcloud";
  ++global_counter_rpc;
}
void pointCloudCallback(
    const boost::shared_ptr<const sensor_msgs::PointCloud2>& sensor_msg) {
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_cloud(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
  convertPointCloud2MsgToPCL(sensor_msg, input_cloud);
  saveCloud(input_cloud, global_counter_pc, "pc");

  LOG(INFO) << "Saved " + std::to_string(global_counter_pc) + " pointcloud";
  ++global_counter_pc;
}

void rgbCallback(const sensor_msgs::ImageConstPtr& sensor_msg) {
  saveImage(sensor_msg, CV_8UC3, global_counter_rgb, "rgb", ".png");
  LOG(INFO) << "Saved " + std::to_string(global_counter_rgb) + " rgb image";
  ++global_counter_rgb;
}
void rgbRectCallback(const sensor_msgs::ImageConstPtr& sensor_msg) {
  saveImage(sensor_msg, CV_8UC3, global_counter_rgb_rect, "rgb_rect", ".png");
  LOG(INFO) << "Saved " + std::to_string(global_counter_rgb_rect) +
                   " rect_rgb image";
  ++global_counter_rgb_rect;
}

void depthImageCallback(const sensor_msgs::ImageConstPtr& sensor_msg) {
  saveImage(sensor_msg, CV_32FC1, global_counter_depth, "depth_image", ".exr");
  LOG(INFO) << "Saved " + std::to_string(global_counter_depth) + " depth image";
  ++global_counter_depth;
}
void depthImageRawCallback(const sensor_msgs::ImageConstPtr& sensor_msg) {
  saveImage(sensor_msg, CV_16UC1, global_counter_depth_raw, "depth_image_raw",
            ".png");
  LOG(INFO) << "Saved " + std::to_string(global_counter_depth_raw) +
                   " depth_raw image";
  ++global_counter_depth_raw;
}
void depthImageRectCallback(const sensor_msgs::ImageConstPtr& sensor_msg) {
  saveImage(sensor_msg, CV_32FC1, global_counter_depth_rect, "depth_image_rect",
            ".exr");
  LOG(INFO) << "Saved " + std::to_string(global_counter_depth_rect) +
                   " depth_rect image";
  ++global_counter_depth_rect;
}
void depthImageRectRawCallback(const sensor_msgs::ImageConstPtr& sensor_msg) {
  saveImage(sensor_msg, CV_16UC1, global_counter_depth_rect_raw,
            "depth_image_rect_raw", ".png");
  LOG(INFO) << "Saved " + std::to_string(global_counter_depth_rect_raw) +
                   " depth_rect_raw image";
  ++global_counter_depth_rect_raw;
}
void depthImageHwRectCallback(const sensor_msgs::ImageConstPtr& sensor_msg) {
  saveImage(sensor_msg, CV_16UC1, global_counter_depth_hw_rect,
            "depth_image_hw_rect", ".png");
  LOG(INFO) << "Saved " + std::to_string(global_counter_depth_hw_rect) +
                   " depth_hw_rect image";
  ++global_counter_depth_hw_rect;
}
void depthImageHwRectRawCallback(const sensor_msgs::ImageConstPtr& sensor_msg) {
  saveImage(sensor_msg, CV_32FC1, global_counter_depth_hw_rect_raw,
            "depth_image_hw_rect_raw", ".exr");
  LOG(INFO) << "Saved " + std::to_string(global_counter_depth_hw_rect_raw) +
                   " depth_hw_rect_raw image";
  ++global_counter_depth_hw_rect_raw;
}
void depthImageSwRectCallback(const sensor_msgs::ImageConstPtr& sensor_msg) {
  saveImage(sensor_msg, CV_32FC1, global_counter_depth_sw_rect,
            "depth_image_sw_rect", ".exr");
  LOG(INFO) << "Saved " + std::to_string(global_counter_depth_sw_rect) +
                   " depth_sw_rect image";
  ++global_counter_depth_sw_rect;
}
void depthImageSwRectRawCallback(const sensor_msgs::ImageConstPtr& sensor_msg) {
  saveImage(sensor_msg, CV_16UC1, global_counter_depth_sw_rect_raw,
            "depth_image_sw_rect_raw", ".png");
  LOG(INFO) << "Saved " + std::to_string(global_counter_depth_sw_rect_raw) +
                   " depth_sw_rect_raw image";
  ++global_counter_depth_sw_rect_raw;
}
void depthImageRegCallback(const sensor_msgs::ImageConstPtr& sensor_msg) {
  saveImage(sensor_msg, CV_16UC1, global_counter_depth_reg, "depth_image_reg",
            ".png");
  LOG(INFO) << "Saved " + std::to_string(global_counter_depth_reg) +
                   " depth_reg image";
  ++global_counter_depth_reg;
}
void depthImageRegRawCallback(const sensor_msgs::ImageConstPtr& sensor_msg) {
  saveImage(sensor_msg, CV_32FC1, global_counter_depth_reg_raw,
            "depth_image_reg_raw", ".exr");
  LOG(INFO) << "Saved " + std::to_string(global_counter_depth_reg_raw) +
                   " depth_reg_raw image";
  ++global_counter_depth_reg_raw;
}

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  FLAGS_alsologtostderr = true;

  ros::init(argc, argv, "bag_extraction_node");
  ros::NodeHandle nodeHandle("~");

  // PARAMS
  nodeHandle.param<std::string>("registered_point_cloud_topic",
                                registered_point_cloud_topic,
                                "/camera/depth_registered/points");
  nodeHandle.param<std::string>("point_cloud_topic", point_cloud_topic,
                                "/camera/depth/points");

  nodeHandle.param<std::string>("rgb_topic", rgb_topic,
                                "/camera/rgb/image_raw");
  nodeHandle.param<std::string>("rgb_rect_topic", rgb_rect_topic,
                                "/camera/rgb/image_rect_color");

  nodeHandle.param<std::string>("depth_topic", depth_image_topic,
                                "/camera/depth/image");
  nodeHandle.param<std::string>("depth_topic", depth_image_raw_topic,
                                "/camera/depth/image_raw");
  nodeHandle.param<std::string>("depth_topic", depth_image_rect_topic,
                                "/camera/depth/image_rect");
  nodeHandle.param<std::string>("depth_topic", depth_image_rect_raw_topic,
                                "/camera/depth/image_rect_raw");
  nodeHandle.param<std::string>(
      "depth_topic", depth_reg_hw_image_rect_topic,
      "/camera/depth_registered/hw_registered/image_rect");
  nodeHandle.param<std::string>(
      "depth_topic", depth_reg_hw_image_rect_raw_topic,
      "/camera/depth_registered/hw_registered/image_rect_raw");
  nodeHandle.param<std::string>(
      "depth_topic", depth_reg_sw_image_rect_topic,
      "/camera/depth_registered/sw_registered/image_rect");
  nodeHandle.param<std::string>(
      "depth_topic", depth_reg_sw_image_rect_raw_topic,
      "/camera/depth_registered/sw_registered/image_rect_raw");
  nodeHandle.param<std::string>("depth_topic", depth_reg_image_topic,
                                "/camera/depth_registered/image");
  nodeHandle.param<std::string>("depth_topic", depth_reg_image_raw_topic,
                                "/camera/depth_registered/image_raw");

  nodeHandle.param<bool>("use_binary", use_binary, true);
  nodeHandle.param<bool>("convert_to_unit", convert_to_uint, false);
  nodeHandle.param<bool>("flip_bgr_to_rgb", flip_bgr_to_rgb, true);
  nodeHandle.param<std::string>("global_path", global_path,
                                "/Users/ntonci/Datasets/unpack/");

  // SUBCRIBERS
  registered_point_cloud_sub = nodeHandle.subscribe(
      registered_point_cloud_topic, 1000, &registeredPointCloudCallback);
  point_cloud_sub =
      nodeHandle.subscribe(point_cloud_topic, 1000, &pointCloudCallback);

  rgb_sub = nodeHandle.subscribe(rgb_topic, 1000, &rgbCallback);
  rgb_rect_sub = nodeHandle.subscribe(rgb_rect_topic, 1000, &rgbRectCallback);

  depth_image_sub =
      nodeHandle.subscribe(depth_image_topic, 1000, &depthImageCallback);
  depth_image_raw_sub =
      nodeHandle.subscribe(depth_image_raw_topic, 1000, &depthImageRawCallback);
  depth_image_rect_sub = nodeHandle.subscribe(depth_image_rect_topic, 1000,
                                              &depthImageRectCallback);
  depth_image_rect_raw_sub = nodeHandle.subscribe(
      depth_image_rect_raw_topic, 1000, &depthImageRectRawCallback);
  depth_reg_hw_image_rect_sub = nodeHandle.subscribe(
      depth_reg_hw_image_rect_topic, 1000, &depthImageHwRectCallback);
  depth_reg_hw_image_rect_raw_sub = nodeHandle.subscribe(
      depth_reg_hw_image_rect_raw_topic, 1000, &depthImageHwRectRawCallback);
  depth_reg_sw_image_rect_sub = nodeHandle.subscribe(
      depth_reg_sw_image_rect_topic, 1000, &depthImageSwRectCallback);
  depth_reg_sw_image_rect_raw_sub = nodeHandle.subscribe(
      depth_reg_sw_image_rect_raw_topic, 1000, &depthImageSwRectRawCallback);
  depth_reg_image_sub =
      nodeHandle.subscribe(depth_reg_image_topic, 1000, &depthImageRegCallback);
  depth_reg_image_raw_sub = nodeHandle.subscribe(
      depth_reg_image_raw_topic, 1000, &depthImageRegRawCallback);

  ros::Rate loop_rate(10);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
