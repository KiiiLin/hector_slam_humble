//=================================================================================================
// Copyright (c) 2011, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include "HectorMappingRos.h"

#include "map/GridMap.h"

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "HectorDrawings.h"
#include "HectorDebugInfoProvider.h"
#include "HectorMapMutex.h"

#include <tf2/utils.h>
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
#include <string>
#include <memory>

HectorMappingRos::HectorMappingRos()
  : rclcpp::Node("hector_mapping")
  , debugInfoProvider(0)
  , hectorDrawings(0)
  , lastGetMapUpdateIndex(-100)
  , map__publish_thread_(nullptr)
  , initial_pose_set_(false)
  , pause_scan_processing_(false)
{
  std::string mapTopic_ = "map";

  // Declare and get parameters
  this->declare_parameter<bool>("pub_drawings", false);
  this->declare_parameter<bool>("pub_debug_output", false);
  this->declare_parameter<bool>("pub_map_odom_transform", true);
  this->declare_parameter<bool>("pub_odometry", false);
  this->declare_parameter<bool>("advertise_map_service", true);
  this->declare_parameter<int>("scan_subscriber_queue_size", 5);

  this->declare_parameter<double>("map_resolution", 0.025);
  this->declare_parameter<int>("map_size", 1024);
  this->declare_parameter<double>("map_start_x", 0.5);
  this->declare_parameter<double>("map_start_y", 0.5);
  this->declare_parameter<int>("map_multi_res_levels", 3);

  this->declare_parameter<double>("update_factor_free", 0.4);
  this->declare_parameter<double>("update_factor_occupied", 0.9);

  this->declare_parameter<double>("map_update_distance_thresh", 0.4);
  this->declare_parameter<double>("map_update_angle_thresh", 0.9);

  this->declare_parameter<std::string>("scan_topic", "scan");
  this->declare_parameter<std::string>("sys_msg_topic", "syscommand");
  this->declare_parameter<std::string>("pose_update_topic", "poseupdate");

  this->declare_parameter<bool>("use_tf_scan_transformation", true);
  this->declare_parameter<bool>("use_tf_pose_start_estimate", false);
  this->declare_parameter<bool>("map_with_known_poses", false);

  this->declare_parameter<std::string>("base_frame", "base_link");
  this->declare_parameter<std::string>("map_frame", "map");
  this->declare_parameter<std::string>("odom_frame", "odom");

  this->declare_parameter<bool>("pub_map_scanmatch_transform", true);
  this->declare_parameter<std::string>("tf_map_scanmatch_transform_frame_name", "scanmatcher_frame");

  this->declare_parameter<bool>("output_timing", false);

  this->declare_parameter<double>("map_pub_period", 2.0);

  this->declare_parameter<double>("laser_min_dist", 0.4);
  this->declare_parameter<double>("laser_max_dist", 30.0);
  this->declare_parameter<double>("laser_z_min_value", -1.0);
  this->declare_parameter<double>("laser_z_max_value", 1.0);

  this->get_parameter("pub_drawings", p_pub_drawings);
  this->get_parameter("pub_debug_output", p_pub_debug_output_);
  this->get_parameter("pub_map_odom_transform", p_pub_map_odom_transform_);
  this->get_parameter("pub_odometry", p_pub_odometry_);
  this->get_parameter("advertise_map_service", p_advertise_map_service_);
  this->get_parameter("scan_subscriber_queue_size", p_scan_subscriber_queue_size_);

  this->get_parameter("map_resolution", p_map_resolution_);
  this->get_parameter("map_size", p_map_size_);
  this->get_parameter("map_start_x", p_map_start_x_);
  this->get_parameter("map_start_y", p_map_start_y_);
  this->get_parameter("map_multi_res_levels", p_map_multi_res_levels_);

  this->get_parameter("update_factor_free", p_update_factor_free_);
  this->get_parameter("update_factor_occupied", p_update_factor_occupied_);

  this->get_parameter("map_update_distance_thresh", p_map_update_distance_threshold_);
  this->get_parameter("map_update_angle_thresh", p_map_update_angle_threshold_);

  this->get_parameter("scan_topic", p_scan_topic_);
  this->get_parameter("sys_msg_topic", p_sys_msg_topic_);
  this->get_parameter("pose_update_topic", p_pose_update_topic_);

  this->get_parameter("use_tf_scan_transformation", p_use_tf_scan_transformation_);
  this->get_parameter("use_tf_pose_start_estimate", p_use_tf_pose_start_estimate_);
  this->get_parameter("map_with_known_poses", p_map_with_known_poses_);

  this->get_parameter("base_frame", p_base_frame_);
  this->get_parameter("map_frame", p_map_frame_);
  this->get_parameter("odom_frame", p_odom_frame_);

  this->get_parameter("pub_map_scanmatch_transform", p_pub_map_scanmatch_transform_);
  this->get_parameter("tf_map_scanmatch_transform_frame_name", p_tf_map_scanmatch_transform_frame_name_);

  this->get_parameter("output_timing", p_timing_output_);

  this->get_parameter("map_pub_period", p_map_pub_period_);

  double tmp = 0.0;
  this->get_parameter("laser_min_dist", tmp);
  p_sqr_laser_min_dist_ = static_cast<float>(tmp*tmp);

  this->get_parameter("laser_max_dist", tmp);
  p_sqr_laser_max_dist_ = static_cast<float>(tmp*tmp);

  this->get_parameter("laser_z_min_value", tmp);
  p_laser_z_min_value_ = static_cast<float>(tmp);

  this->get_parameter("laser_z_max_value", tmp);
  p_laser_z_max_value_ = static_cast<float>(tmp);

  if (p_pub_drawings)
  {
    RCLCPP_INFO(this->get_logger(), "HectorSM publishing debug drawings");
    hectorDrawings = new HectorDrawings(shared_from_this());
  }

  if(p_pub_debug_output_)
  {
    RCLCPP_INFO(this->get_logger(), "HectorSM publishing debug info");
    debugInfoProvider = new HectorDebugInfoProvider(shared_from_this());
  }

  if(p_pub_odometry_)
  {
    odometryPublisher_ = this->create_publisher<nav_msgs::msg::Odometry>("scanmatch_odom", 50);
  }

  slamProcessor = new hectorslam::HectorSlamProcessor(static_cast<float>(p_map_resolution_), p_map_size_, p_map_size_, Eigen::Vector2f(p_map_start_x_, p_map_start_y_), p_map_multi_res_levels_, 
    static_cast<DrawInterface*>(hectorDrawings), static_cast<HectorDebugInfoInterface*>(debugInfoProvider));
  slamProcessor->setUpdateFactorFree(p_update_factor_free_);
  slamProcessor->setUpdateFactorOccupied(p_update_factor_occupied_);
  slamProcessor->setMapUpdateMinDistDiff(p_map_update_distance_threshold_);
  slamProcessor->setMapUpdateMinAngleDiff(p_map_update_angle_threshold_);

  int mapLevels = slamProcessor->getMapLevels();
  mapLevels = 1;

  for (int i = 0; i < mapLevels; ++i)
  {
    mapPubContainer.push_back(MapPublisherContainer());
    slamProcessor->addMapMutex(i, new HectorMapMutex());

    std::string mapTopicStr(mapTopic_);

    if (i != 0)
    {
      mapTopicStr.append("_" + std::to_string(i));
    }

    std::string mapMetaTopicStr(mapTopicStr);
    mapMetaTopicStr.append("_metadata");

    MapPublisherContainer& tmp = mapPubContainer[i];
    tmp.mapPublisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(mapTopicStr, rclcpp::QoS(1).transient_local());
    tmp.mapMetadataPublisher_ = this->create_publisher<nav_msgs::msg::MapMetaData>(mapMetaTopicStr, rclcpp::QoS(1).transient_local());

    if ( (i == 0) && p_advertise_map_service_)
    {
      tmp.dynamicMapServiceServer_ = this->create_service<nav_msgs::srv::GetMap>("dynamic_map", 
        std::bind(&HectorMappingRos::mapCallback, this, std::placeholders::_1, std::placeholders::_2));
    }

    setServiceGetMapData(tmp.map_, slamProcessor->getGridMap(i));

    if ( i== 0){
      tmp.mapMetadataPublisher_->publish(tmp.map_.map.info);
    }
  }

  // Initialize services
  reset_map_service_ = this->create_service<std_srvs::srv::Trigger>("reset_map", 
    std::bind(&HectorMappingRos::resetMapCallback, this, std::placeholders::_1, std::placeholders::_2));
  restart_hector_service_ = this->create_service<hector_mapping::srv::ResetMapping>("restart_mapping_with_new_pose", 
    std::bind(&HectorMappingRos::restartHectorCallback, this, std::placeholders::_1, std::placeholders::_2));
  toggle_scan_processing_service_ = this->create_service<std_srvs::srv::SetBool>("pause_mapping", 
    std::bind(&HectorMappingRos::pauseMapCallback, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "HectorSM p_base_frame_: %s", p_base_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "HectorSM p_map_frame_: %s", p_map_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "HectorSM p_odom_frame_: %s", p_odom_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "HectorSM p_scan_topic_: %s", p_scan_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "HectorSM p_use_tf_scan_transformation_: %s", p_use_tf_scan_transformation_ ? ("true") : ("false"));
  RCLCPP_INFO(this->get_logger(), "HectorSM p_pub_map_odom_transform_: %s", p_pub_map_odom_transform_ ? ("true") : ("false"));
  RCLCPP_INFO(this->get_logger(), "HectorSM p_scan_subscriber_queue_size_: %d", p_scan_subscriber_queue_size_);
  RCLCPP_INFO(this->get_logger(), "HectorSM p_map_pub_period_: %f", p_map_pub_period_);
  RCLCPP_INFO(this->get_logger(), "HectorSM p_update_factor_free_: %f", p_update_factor_free_);
  RCLCPP_INFO(this->get_logger(), "HectorSM p_update_factor_occupied_: %f", p_update_factor_occupied_);
  RCLCPP_INFO(this->get_logger(), "HectorSM p_map_update_distance_threshold_: %f ", p_map_update_distance_threshold_);
  RCLCPP_INFO(this->get_logger(), "HectorSM p_map_update_angle_threshold_: %f", p_map_update_angle_threshold_);
  RCLCPP_INFO(this->get_logger(), "HectorSM p_laser_z_min_value_: %f", p_laser_z_min_value_);
  RCLCPP_INFO(this->get_logger(), "HectorSM p_laser_z_max_value_: %f", p_laser_z_max_value_);

  // Sensor-data (best-effort) QoS: most lidar drivers publish best-effort,
  // which a reliable subscription does not match, so no scans would arrive.
  // A best-effort subscription also matches reliable publishers.
  scanSubscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(p_scan_topic_,
    rclcpp::SensorDataQoS().keep_last(p_scan_subscriber_queue_size_),
    std::bind(&HectorMappingRos::scanCallback, this, std::placeholders::_1));
  sysMsgSubscriber_ = this->create_subscription<std_msgs::msg::String>(p_sys_msg_topic_, 2, 
    std::bind(&HectorMappingRos::sysMsgCallback, this, std::placeholders::_1));

  poseUpdatePublisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(p_pose_update_topic_, 1);
  posePublisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("slam_out_pose", 1);

  scan_point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("slam_cloud", 1);

  // Initialize TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tfB_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Initialize initial pose subscriber (simplified, no message filter for now)
  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 2, 
    std::bind(&HectorMappingRos::initialPoseCallback, this, std::placeholders::_1));

  // Initialize map_to_odom transform
  map_to_odom_.header.frame_id = p_map_frame_;
  map_to_odom_.child_frame_id = p_odom_frame_;
  map_to_odom_.transform.translation.x = 0.0;
  map_to_odom_.transform.translation.y = 0.0;
  map_to_odom_.transform.translation.z = 0.0;
  map_to_odom_.transform.rotation.x = 0.0;
  map_to_odom_.transform.rotation.y = 0.0;
  map_to_odom_.transform.rotation.z = 0.0;
  map_to_odom_.transform.rotation.w = 1.0;

  map__publish_thread_ = new std::thread(&HectorMappingRos::publishMapLoop, this, p_map_pub_period_);

  lastMapPublishTime = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

HectorMappingRos::~HectorMappingRos()
{
  delete slamProcessor;

  if (hectorDrawings)
    delete hectorDrawings;

  if (debugInfoProvider)
    delete debugInfoProvider;

  if(map__publish_thread_)
  {
    map__publish_thread_->join();
    delete map__publish_thread_;
  }
}

void HectorMappingRos::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  if (pause_scan_processing_)
  {
    return;
  }

  if (hectorDrawings)
  {
    // TODO: Migrate setTime to ROS2
    // hectorDrawings->setTime(scan->header.stamp);
  }

  auto start_time = std::chrono::steady_clock::now();
  if (!p_use_tf_scan_transformation_)
  {
    // If we are not using the tf tree to find the transform between the base frame and laser frame,
    // then just convert the laser scan to our data container and process the update based on our last
    // pose estimate
    this->rosLaserScanToDataContainer(*scan, laserScanContainer, slamProcessor->getScaleToMap());
    slamProcessor->update(laserScanContainer, slamProcessor->getLastScanMatchPose());
  }
  else
  {
    // If we are using the tf tree to find the transform between the base frame and laser frame,
    // let's get that transform
    geometry_msgs::msg::TransformStamped laser_transform;
    try
    {
      laser_transform = tf_buffer_->lookupTransform(p_base_frame_, scan->header.frame_id, scan->header.stamp, rclcpp::Duration::from_seconds(0.5));
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_INFO(this->get_logger(), "lookupTransform %s to %s timed out. Could not transform laser scan into base_frame: %s", 
                  p_base_frame_.c_str(), scan->header.frame_id.c_str(), ex.what());
      return;
    }

    // Project the scan into a point cloud in the laser frame (what
    // laser_geometry's projectLaser() did in the ROS 1 version)
    laser_point_cloud_.header = scan->header;
    laser_point_cloud_.points.clear();
    laser_point_cloud_.points.reserve(scan->ranges.size());
    float angle = scan->angle_min;
    for (float range : scan->ranges)
    {
      if (std::isfinite(range) && range > scan->range_min && range < scan->range_max)
      {
        geometry_msgs::msg::Point32 p;
        p.x = std::cos(angle) * range;
        p.y = std::sin(angle) * range;
        laser_point_cloud_.points.push_back(p);
      }
      angle += scan->angle_increment;
    }

    // Publish the point cloud if there are any subscribers (disabled for now)
    // if (scan_point_cloud_publisher_->get_subscription_count() > 0)
    // {
    //   scan_point_cloud_publisher_->publish(laser_point_cloud_);
    // }

    // Convert the point cloud to our data container, transforming it into the base frame
    this->rosPointCloudToDataContainer(laser_point_cloud_, laser_transform, laserScanContainer, slamProcessor->getScaleToMap());

    // Now let's choose the initial pose estimate for our slam process update
    Eigen::Vector3f start_estimate(Eigen::Vector3f::Zero());
    if (initial_pose_set_)
    {
      // User has requested a pose reset
      initial_pose_set_ = false;
      start_estimate = initial_pose_;
    }
    else if (p_use_tf_pose_start_estimate_)
    {
      // Initial pose estimate comes from the tf tree
      try
      {
        geometry_msgs::msg::TransformStamped stamped_pose = tf_buffer_->lookupTransform(p_map_frame_, p_base_frame_, scan->header.stamp, rclcpp::Duration::from_seconds(0.5));

        tf2::Quaternion q;
        tf2::fromMsg(stamped_pose.transform.rotation, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        start_estimate = Eigen::Vector3f(stamped_pose.transform.translation.x, stamped_pose.transform.translation.y, yaw);
      }
      catch(tf2::TransformException &e)
      {
        RCLCPP_ERROR(this->get_logger(), "Transform from %s to %s failed: %s", p_map_frame_.c_str(), p_base_frame_.c_str(), e.what());
        start_estimate = slamProcessor->getLastScanMatchPose();
      }
    }
    else
    {
      // If none of the above, the initial pose is simply the last estimated pose
      start_estimate = slamProcessor->getLastScanMatchPose();
    }

    // If "p_map_with_known_poses_" is enabled, we assume that start_estimate is precise and doesn't need to be refined
    if (p_map_with_known_poses_)
    {
      slamProcessor->update(laserScanContainer, start_estimate, true);
    }
    else
    {
      slamProcessor->update(laserScanContainer, start_estimate);
    }
  }

  // If the debug flag "p_timing_output_" is enabled, print how long this last iteration took
  if (p_timing_output_)
  {
    auto end_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    RCLCPP_INFO(this->get_logger(), "HectorSLAM Iter took: %ld milliseconds", duration.count());
  }

  // If we're just building a map with known poses, we're finished now. Code below this point publishes the localization results.
  if (p_map_with_known_poses_)
  {
    return;
  }

  poseInfoContainer_.update(slamProcessor->getLastScanMatchPose(), slamProcessor->getLastScanMatchCovariance(), scan->header.stamp, p_map_frame_);

  // Publish pose with and without covariances
  poseUpdatePublisher_->publish(poseInfoContainer_.getPoseWithCovarianceStamped());
  posePublisher_->publish(poseInfoContainer_.getPoseStamped());

  // Publish odometry if enabled
  if(p_pub_odometry_)
  {
    nav_msgs::msg::Odometry tmp;
    tmp.pose = poseInfoContainer_.getPoseWithCovarianceStamped().pose;

    tmp.header = poseInfoContainer_.getPoseWithCovarianceStamped().header;
    tmp.child_frame_id = p_base_frame_;
    odometryPublisher_->publish(tmp);
  }

  // Publish the map->odom transform if enabled
  if (p_pub_map_odom_transform_)
  {
    geometry_msgs::msg::TransformStamped odom_to_base;
    try
    {
      odom_to_base = tf_buffer_->lookupTransform(p_odom_frame_, p_base_frame_, scan->header.stamp, rclcpp::Duration::from_seconds(0.5));
    }
    catch(tf2::TransformException &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Transform failed during publishing of map_odom transform: %s", e.what());
      odom_to_base.transform.translation.x = 0.0;
      odom_to_base.transform.translation.y = 0.0;
      odom_to_base.transform.translation.z = 0.0;
      odom_to_base.transform.rotation.x = 0.0;
      odom_to_base.transform.rotation.y = 0.0;
      odom_to_base.transform.rotation.z = 0.0;
      odom_to_base.transform.rotation.w = 1.0;
    }
    
    // Calculate map_to_odom = pose * odom_to_base^-1
    tf2::Transform odom_to_base_tf, map_to_odom_tf;
    tf2::fromMsg(odom_to_base.transform, odom_to_base_tf);
    map_to_odom_tf = poseInfoContainer_.getTfTransform() * odom_to_base_tf.inverse();
    
    map_to_odom_.header.stamp = scan->header.stamp;
    map_to_odom_.header.frame_id = p_map_frame_;
    map_to_odom_.child_frame_id = p_odom_frame_;
    map_to_odom_.transform = tf2::toMsg(map_to_odom_tf);
    tfB_->sendTransform(map_to_odom_);
  }

  // Publish the transform from map to estimated pose (if enabled)
  if (p_pub_map_scanmatch_transform_)
  {
    geometry_msgs::msg::TransformStamped scanmatch_tf;
    scanmatch_tf.header.stamp = scan->header.stamp;
    scanmatch_tf.header.frame_id = p_map_frame_;
    scanmatch_tf.child_frame_id = p_tf_map_scanmatch_transform_frame_name_;
    scanmatch_tf.transform = tf2::toMsg(poseInfoContainer_.getTfTransform());
    tfB_->sendTransform(scanmatch_tf);
  }
}

void HectorMappingRos::sysMsgCallback(const std_msgs::msg::String::SharedPtr string)
{
  RCLCPP_INFO(this->get_logger(), "HectorSM sysMsgCallback, msg contents: %s", string->data.c_str());

  if (string->data == "reset")
  {
    RCLCPP_INFO(this->get_logger(), "HectorSM reset");
    slamProcessor->reset();
  }
}

void HectorMappingRos::mapCallback(const std::shared_ptr<nav_msgs::srv::GetMap::Request> /* req */,
                                   std::shared_ptr<nav_msgs::srv::GetMap::Response> res)
{
  RCLCPP_INFO(this->get_logger(), "HectorSM Map service called");
  res->map = mapPubContainer[0].map_.map;
}

void HectorMappingRos::resetMapCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /* req */,
                                       std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  RCLCPP_INFO(this->get_logger(), "HectorSM Reset map service called");
  slamProcessor->reset();
  res->success = true;
}

void HectorMappingRos::restartHectorCallback(const std::shared_ptr<hector_mapping::srv::ResetMapping::Request> req,
                                             std::shared_ptr<hector_mapping::srv::ResetMapping::Response> /* res */)
{
  // Reset map
  RCLCPP_INFO(this->get_logger(), "HectorSM Reset map");
  slamProcessor->reset();

  // Reset pose
  this->resetPose(req->initial_pose);

  // Unpause node (in case it is paused)
  this->toggleMappingPause(false);

  // Return success (ResetMapping service has no response fields in ROS2)
}

void HectorMappingRos::pauseMapCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                                        std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
  this->toggleMappingPause(req->data);
  res->success = true;
}

void HectorMappingRos::publishMap(MapPublisherContainer& mapPublisher, const hectorslam::GridMap& gridMap, rclcpp::Time timestamp, MapLockerInterface* mapMutex)
{
  nav_msgs::srv::GetMap::Response& map_ (mapPublisher.map_);

  //only update map if it changed
  if (lastGetMapUpdateIndex != gridMap.getUpdateIndex())
  {

    int sizeX = gridMap.getSizeX();
    int sizeY = gridMap.getSizeY();

    int size = sizeX * sizeY;

    std::vector<int8_t>& data = map_.map.data;

    //std::vector contents are guaranteed to be contiguous, use memset to set all to unknown to save time in loop
    memset(&data[0], -1, sizeof(int8_t) * size);

    if (mapMutex)
    {
      mapMutex->lockMap();
    }

    for(int i=0; i < size; ++i)
    {
      if(gridMap.isFree(i))
      {
        data[i] = 0;
      }
      else if (gridMap.isOccupied(i))
      {
        data[i] = 100;
      }
    }

    lastGetMapUpdateIndex = gridMap.getUpdateIndex();

    if (mapMutex)
    {
      mapMutex->unlockMap();
    }
  }

  map_.map.header.stamp = timestamp;

  mapPublisher.mapPublisher_->publish(map_.map);
}

void HectorMappingRos::rosLaserScanToDataContainer(const sensor_msgs::msg::LaserScan& scan, hectorslam::DataContainer& dataContainer, float scaleToMap)
{
  size_t size = scan.ranges.size();

  float angle = scan.angle_min;

  dataContainer.clear();

  dataContainer.setOrigo(Eigen::Vector2f::Zero());

  float maxRangeForContainer = scan.range_max - 0.1f;

  for (size_t i = 0; i < size; ++i)
  {
    float dist = scan.ranges[i];

    if ( (dist > scan.range_min) && (dist < maxRangeForContainer))
    {
      dist *= scaleToMap;
      dataContainer.add(Eigen::Vector2f(cos(angle) * dist, sin(angle) * dist));
    }

    angle += scan.angle_increment;
  }
}

void HectorMappingRos::rosPointCloudToDataContainer(const sensor_msgs::msg::PointCloud& pointCloud, const geometry_msgs::msg::TransformStamped& laserTransform, hectorslam::DataContainer& dataContainer, float scaleToMap)
{
  size_t size = pointCloud.points.size();

  dataContainer.clear();

  tf2::Transform transform;
  tf2::fromMsg(laserTransform.transform, transform);
  tf2::Vector3 laserPos = transform.getOrigin();
  dataContainer.setOrigo(Eigen::Vector2f(laserPos.x(), laserPos.y())*scaleToMap);

  for (size_t i = 0; i < size; ++i)
  {

    const geometry_msgs::msg::Point32& currPoint(pointCloud.points[i]);

    float dist_sqr = currPoint.x*currPoint.x + currPoint.y* currPoint.y;

    if ( (dist_sqr > p_sqr_laser_min_dist_) && (dist_sqr < p_sqr_laser_max_dist_) ){

      if ( (currPoint.x < 0.0f) && (dist_sqr < 0.50f)){
        continue;
      }

      tf2::Vector3 pointPosLaserFrame(currPoint.x, currPoint.y, currPoint.z);
      tf2::Vector3 pointPosBaseFrame = transform * pointPosLaserFrame;

      float pointPosLaserFrameZ = pointPosBaseFrame.z() - laserPos.z();

      if (pointPosLaserFrameZ > p_laser_z_min_value_ && pointPosLaserFrameZ < p_laser_z_max_value_)
      {
        dataContainer.add(Eigen::Vector2f(pointPosBaseFrame.x(),pointPosBaseFrame.y())*scaleToMap);
      }
    }
  }
}

void HectorMappingRos::setServiceGetMapData(nav_msgs::srv::GetMap::Response& map_, const hectorslam::GridMap& gridMap)
{
  Eigen::Vector2f mapOrigin (gridMap.getWorldCoords(Eigen::Vector2f::Zero()));
  mapOrigin.array() -= gridMap.getCellLength()/1000*0.5f;

  map_.map.info.origin.position.x = mapOrigin.x();
  map_.map.info.origin.position.y = mapOrigin.y();
  map_.map.info.origin.orientation.w = 1.0;

  map_.map.info.resolution = gridMap.getCellLength();

  map_.map.info.width = gridMap.getSizeX();
  map_.map.info.height = gridMap.getSizeY();

  map_.map.header.frame_id = p_map_frame_;
  map_.map.data.resize(map_.map.info.width * map_.map.info.height);
}

/*
void HectorMappingRos::setStaticMapData(const nav_msgs::OccupancyGrid& map)
{
  float cell_length = map.info.resolution;
  Eigen::Vector2f mapOrigin (map.info.origin.position.x + cell_length*0.5f,
                             map.info.origin.position.y + cell_length*0.5f);

  int map_size_x = map.info.width;
  int map_size_y = map.info.height;

  slamProcessor = new hectorslam::HectorSlamProcessor(cell_length, map_size_x, map_size_y, Eigen::Vector2f(0.0f, 0.0f), 1, hectorDrawings, debugInfoProvider);
}
*/


void HectorMappingRos::publishMapLoop(double map_pub_period)
{
  rclcpp::Rate r(1.0 / map_pub_period);
  while(rclcpp::ok())
  {
    rclcpp::Time mapTime = this->now();
    publishMap(mapPubContainer[0],slamProcessor->getGridMap(0), mapTime, slamProcessor->getMapMutex(0));

    r.sleep();
  }
}

void HectorMappingRos::staticMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr /* map */)
{

}

void HectorMappingRos::initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  this->resetPose(msg->pose.pose);
}

void HectorMappingRos::toggleMappingPause(bool pause)
{
  // Pause/unpause
  if (pause && !pause_scan_processing_)
  {
    RCLCPP_INFO(this->get_logger(), "[HectorSM]: Mapping paused");
  }
  else if (!pause && pause_scan_processing_)
  {
    RCLCPP_INFO(this->get_logger(), "[HectorSM]: Mapping no longer paused");
  }
  pause_scan_processing_ = pause;
}

void HectorMappingRos::resetPose(const geometry_msgs::msg::Pose &pose)
{
  initial_pose_set_ = true;
  initial_pose_ = Eigen::Vector3f(pose.position.x, pose.position.y, util::getYawFromQuat(pose.orientation));
  RCLCPP_INFO(this->get_logger(), "[HectorSM]: Setting initial pose with world coords x: %f y: %f yaw: %f",
           initial_pose_[0], initial_pose_[1], initial_pose_[2]);
}
