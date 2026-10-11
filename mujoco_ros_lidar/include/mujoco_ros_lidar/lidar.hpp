/*********************************************************************
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2026, JSK Lab, The University of Tokyo
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#pragma once

#include <mujoco_ros/ros_version.hpp>

#if MJR_ROS_VERSION == ROS_1
#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <mujoco_ros/ros_one/plugin_utils.hpp>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#else
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <mujoco_ros/ros_two/plugin_utils.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#endif

#include <mujoco_ros/mujoco_env.hpp>
#include <mujoco_ros_sensors/mujoco_sensor_handler_plugin.hpp>

#include <cstdint>
#include <mutex>
#include <random>
#include <string>
#include <vector>

namespace mujoco_ros::sensors::lidar {

#if MJR_ROS_VERSION == ROS_1
using PointCloud2 = sensor_msgs::PointCloud2;
using Imu         = sensor_msgs::Imu;
using RosTime     = ros::Time;
#else
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using Imu         = sensor_msgs::msg::Imu;
using RosTime     = rclcpp::Time;
#endif

// Defaults correspond to the Livox Mid-360 datasheet
static constexpr double DEFAULT_UPDATE_RATE          = 10.;
static constexpr int DEFAULT_POINTS_PER_FRAME        = 20000; // 200k pts/s @ 10 Hz
static constexpr double DEFAULT_MIN_RANGE            = 0.1;
static constexpr double DEFAULT_MAX_RANGE            = 40.;
static constexpr double DEFAULT_RANGE_STD            = 0.02; // 2 cm (1 sigma) @ 10 m
static constexpr double DEFAULT_MIN_ELEVATION        = -7. * M_PI / 180.;
static constexpr double DEFAULT_MAX_ELEVATION        = 52. * M_PI / 180.;
static constexpr int DEFAULT_NUM_LINES               = 4;
static constexpr bool DEFAULT_VISUALIZE              = false;
static constexpr int DEFAULT_VISUALIZE_MAX_POINTS    = 2000;
static constexpr double DEFAULT_VISUALIZE_POINT_SIZE = 0.02;
static constexpr double DEFAULT_IMU_RATE             = 200.;
static constexpr double DEFAULT_IMU_GYRO_STD         = 0.001; // rad/s
static constexpr double DEFAULT_IMU_ACCEL_STD        = 0.002; // g

/**
 * @brief Single point layout of the PointCloud2 message. The first fields match livox_ros_driver2 (xfer_format 0) so
 * that downstream nodes written for the real sensor can consume the simulated cloud unchanged. `time` (offset from the
 * header stamp in seconds) and `ring` (= line) are added so that velodyne-style consumers can use the cloud as well
 * (e.g. FAST-LIO with lidar_type 2 and timestamp_unit 0).
 */
#pragma pack(push, 1)
struct LivoxPoint
{
	float x;
	float y;
	float z;
	float intensity;
	uint8_t tag;
	uint8_t line;
	double timestamp;
	float time;
	uint16_t ring;
};
#pragma pack(pop)

enum class IntensitySource
{
	COLOR, // luminance of the hit geom's (material) rgba, scaled to [0, 255]
	GEOM_USER, // first entry of the hit geom's user data (<geom user="..."/>), i.e. Livox reflectivity [0, 255]
};

enum class ScanPattern
{
	MID360, // built-in non-repetitive pattern approximating the Mid-360 coverage
	CSV, // scan pattern loaded from a csv file (livox_laser_simulation format)
};

struct LidarConfig : public mujoco_ros::sensors::SensorConfig
{
public:
	LidarConfig(std::string frame_id, std::string name, int site_attached, int body_attached);

	std::string name;
	int site_attached;
	int body_attached;

	double update_rate          = DEFAULT_UPDATE_RATE;
	uint points_per_frame       = DEFAULT_POINTS_PER_FRAME;
	mjtNum min_range            = DEFAULT_MIN_RANGE;
	mjtNum max_range            = DEFAULT_MAX_RANGE;
	double range_std            = DEFAULT_RANGE_STD;
	double min_elevation        = DEFAULT_MIN_ELEVATION;
	double max_elevation        = DEFAULT_MAX_ELEVATION;
	uint num_lines              = DEFAULT_NUM_LINES;
	bool visualize              = DEFAULT_VISUALIZE;
	uint visualize_max_points   = DEFAULT_VISUALIZE_MAX_POINTS;
	double visualize_point_size = DEFAULT_VISUALIZE_POINT_SIZE;
	std::vector<int> ignore_geom_groups;

	IntensitySource intensity_source = IntensitySource::COLOR;
	ScanPattern pattern              = ScanPattern::MID360;
	std::string pattern_file;

	// Scan pattern state
	std::vector<mjtNum> pattern_dirs; // unit directions in sensor frame (CSV pattern only)
	size_t pattern_index = 0;
	double r2_u          = 0.5;
	double r2_v          = 0.5;
	uint64_t emitted     = 0;

	// Per-frame buffers. Rays of a frame are cast progressively over the frame duration (ray i at
	// frame start + i / points_per_frame / update_rate), so motion during a scan distorts the cloud like on the real
	// sensor and the per-point time offsets are consistent with the sensor pose at emission.
	uint ncast = 0; // number of rays of the current frame cast so far
	std::vector<mjtNum> dirs_local;
	std::vector<mjtNum> dirs_world;
	std::vector<mjtNum> hit_world;
	std::vector<mjtNum> dists;
	std::vector<int> geomids;
	std::vector<float> point_times;
	std::vector<uint8_t> point_lines;

	// Hit points drawn in the viewer, kept until the next frame of this sensor (guarded by render_mutex_)
	std::vector<mjvGeom> vis_geoms;

	RosTime last_update_time; // start of the current frame
	bool last_update_time_initialized = false;

	// Built-in IMU (Mid-360 has an ICM40609 publishing on livox/imu). Disabled if imu_site < 0.
	int imu_site = -1;
	std::string imu_frame_id;
	double imu_rate      = DEFAULT_IMU_RATE;
	double imu_gyro_std  = DEFAULT_IMU_GYRO_STD;
	double imu_accel_std = DEFAULT_IMU_ACCEL_STD;
#if MJR_ROS_VERSION == ROS_1
	ros::Publisher imu_pub;
#else
	rclcpp::Publisher<Imu>::SharedPtr imu_pub;
#endif
	RosTime imu_last_update_time;
	bool imu_last_update_time_initialized = false;
};

class LidarPlugin : public mujoco_ros::MujocoPlugin
{
public:
	~LidarPlugin() override = default;

	bool Load(const mjModel *m, mjData *d) override;
	void Reset() override;

	void RenderCallback(const mjModel *model, mjData *data, mjvScene *scene) override;
	void LastStageCallback(const mjModel *model, mjData *data) override;

	const std::vector<LidarConfig> &GetLidarConfigs() const { return lidar_configs_; }

protected:
	std::vector<LidarConfig> lidar_configs_;

#if MJR_ROS_VERSION == ROS_1
	ros::NodeHandle lidar_nh_;
	bool InitSensor(const mjModel *model, const XmlRpc::XmlRpcValue &config);
#else
	rclcpp_lifecycle::LifecycleNode::SharedPtr lidar_nh_;
	bool InitSensor(const mjModel *model, const std::string &sensor_name);
#endif

	bool InitPattern(LidarConfig &config);
	void GenerateDirections(LidarConfig &config);
	void PublishImu(const mjModel *model, LidarConfig &config, const RosTime &now, const mjtNum *acc6,
	                const mjtNum *vel6);
	void StartFrame(LidarConfig &config, const RosTime &start);
	void CastRays(const mjModel *model, mjData *data, LidarConfig &config, int start, int count);
	void FillCloud(const mjModel *model, const mjData *data, LidarConfig &config, PointCloud2 &cloud,
	               std::vector<mjvGeom> &vis_geoms);

	std::mutex render_mutex_;

	std::mt19937 rand_generator_ = std::mt19937(std::random_device{}());
	std::normal_distribution<double> noise_dist_{ 0.0, 1.0 };
};
} // namespace mujoco_ros::sensors::lidar
