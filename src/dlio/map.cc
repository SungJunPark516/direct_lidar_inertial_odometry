/***********************************************************
 *                                                         *
 * Copyright (c)                                           *
 *                                                         *
 * The Verifiable & Control-Theoretic Robotics (VECTR) Lab *
 * University of California, Los Angeles                   *
 *                                                         *
 * Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez   *
 * Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu         *
 *                                                         *
 ***********************************************************/

#include "dlio/map.h"
#include "dlio/utils.h"

dlio::MapNode::MapNode() : Node("dlio_map_node")
{

  this->getParams();

  this->keyframe_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto keyframe_sub_opt = rclcpp::SubscriptionOptions();
  keyframe_sub_opt.callback_group = this->keyframe_cb_group;
  this->keyframe_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("keyframes", 10,
                                                                                std::bind(&dlio::MapNode::callbackKeyframe, this, std::placeholders::_1), keyframe_sub_opt);

  this->map_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("map", 100);

  this->save_pcd_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  this->save_pcd_srv = this->create_service<direct_lidar_inertial_odometry::srv::SavePCD>("save_pcd",
                                                                                          std::bind(&dlio::MapNode::savePCD, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, this->save_pcd_cb_group);

  this->dlio_map = std::make_shared<pcl::PointCloud<PointType>>();

  // local map
  this->local_map_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Localmap", 100);
  this->global_map_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("map", 10,
                                                                                  std::bind(&dlio::MapNode::callbackGlobalMap, this, std::placeholders::_1));
  this->latest_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("latest_odom", 10,
                                                                             std::bind(&dlio::MapNode::callbackLatestOdom, this, std::placeholders::_1));
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
}

dlio::MapNode::~MapNode() {}

void dlio::MapNode::getParams()
{

  this->declare_parameter<std::string>("odom/odom_frame", "odom");
  this->declare_parameter<double>("map/sparse/leafSize", 0.5);

  this->get_parameter("odom/odom_frame", this->odom_frame);
  this->get_parameter("map/sparse/leafSize", this->leaf_size_);
}

void dlio::MapNode::start()
{
  // timer that publishes the global map every 200ms
  this->map_pub_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&dlio::MapNode::publishMaps, this));
}

void dlio::MapNode::publishMaps()
{
  // 1. global map publish (dlio_map -> map_pub)
  if (this->dlio_map && !this->dlio_map->empty())
  {
    sensor_msgs::msg::PointCloud2 global_map_msg;
    pcl::toROSMsg(*this->dlio_map, global_map_msg);
    global_map_msg.header.stamp = this->get_clock()->now();
    global_map_msg.header.frame_id = this->odom_frame;
    this->map_pub->publish(global_map_msg);
  }

  // 2. local map은 이미 global_map_sub callback에서 처리 중
}

void dlio::MapNode::callbackKeyframe(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &keyframe)
{

  // convert scan to pcl format
  pcl::PointCloud<PointType>::Ptr keyframe_pcl = std::make_shared<pcl::PointCloud<PointType>>();
  pcl::fromROSMsg(*keyframe, *keyframe_pcl);

  // voxel filter
  this->voxelgrid.setLeafSize(this->leaf_size_, this->leaf_size_, this->leaf_size_);
  this->voxelgrid.setInputCloud(keyframe_pcl);
  this->voxelgrid.filter(*keyframe_pcl);

  // save filtered keyframe to map for rviz
  *this->dlio_map += *keyframe_pcl;

  // publish full map
  if (this->dlio_map->points.size() == this->dlio_map->width * this->dlio_map->height)
  {
    sensor_msgs::msg::PointCloud2 map_ros;
    pcl::toROSMsg(*this->dlio_map, map_ros);
    map_ros.header.stamp = this->now();
    map_ros.header.frame_id = this->odom_frame;
    this->map_pub->publish(map_ros);
  }
}

void dlio::MapNode::callbackLatestOdom(const nav_msgs::msg::Odometry::ConstSharedPtr &odom)
{
  this->latest_odom_ = odom;
}

void dlio::MapNode::callbackGlobalMap(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_global_filtered_in(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_local_filtered_in(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_local_filtered_out(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromROSMsg(*msg, *cloud_global_filtered_in);

  // 위치와 orientation 가져오기 (latest_odom_는 subscriber를 통해 유지)
  if (!latest_odom_)
    return;

  double roll, pitch, yaw;
  tf2::Quaternion q(
      latest_odom_->pose.pose.orientation.x,
      latest_odom_->pose.pose.orientation.y,
      latest_odom_->pose.pose.orientation.z,
      latest_odom_->pose.pose.orientation.w);
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  Eigen::Matrix3f matrix_trans = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
  Eigen::Matrix4f pose_rot = Eigen::Matrix4f::Identity();
  pose_rot.block<3, 3>(0, 0) = matrix_trans;
  pose_rot(0, 3) = latest_odom_->pose.pose.position.x;
  pose_rot(1, 3) = latest_odom_->pose.pose.position.y;
  pose_rot(2, 3) = latest_odom_->pose.pose.position.z;

  Eigen::Matrix4f global_trans_inv = pose_rot.inverse();
  pcl::transformPointCloud(*cloud_global_filtered_in, *cloud_local_filtered_in, global_trans_inv);

  // 거리 기반 필터링 (반경 20m 이내)
  for (const auto &pt : cloud_local_filtered_in->points)
  {
    double dist = std::sqrt(pt.x * pt.x + pt.y * pt.y);
    if (dist <= 20.0)
    {
      cloud_local_filtered_out->points.push_back(pt);
    }
  }

  sensor_msgs::msg::PointCloud2 local_map_msg;
  pcl::toROSMsg(*cloud_local_filtered_out, local_map_msg);
  local_map_msg.header.frame_id = "base_link";
  local_map_msg.header.stamp = this->get_clock()->now();
  local_map_pub_->publish(local_map_msg);
}

void dlio::MapNode::savePCD(std::shared_ptr<direct_lidar_inertial_odometry::srv::SavePCD::Request> req,
                            std::shared_ptr<direct_lidar_inertial_odometry::srv::SavePCD::Response> res)
{

  pcl::PointCloud<PointType>::Ptr m = std::make_shared<pcl::PointCloud<PointType>>(*this->dlio_map);

  float leaf_size = req->leaf_size;
  std::string p = req->save_path;

  std::cout << std::setprecision(2) << "Saving map to " << p + "/dlio_map.pcd"
            << " with leaf size " << to_string_with_precision(leaf_size, 2) << "... ";
  std::cout.flush();

  // voxelize map
  pcl::VoxelGrid<PointType> vg;
  vg.setLeafSize(leaf_size, leaf_size, leaf_size);
  vg.setInputCloud(m);
  vg.filter(*m);

  // save map
  int ret = pcl::io::savePCDFileBinary(p + "/dlio_map.pcd", *m);
  res->success = ret == 0;

  if (res->success)
  {
    std::cout << "done" << std::endl;
  }
  else
  {
    std::cout << "failed" << std::endl;
  }
}
