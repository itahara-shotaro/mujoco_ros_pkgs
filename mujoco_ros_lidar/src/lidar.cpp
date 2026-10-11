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

#include <mujoco_ros_lidar/lidar.hpp>

#if MJR_ROS_VERSION == ROS_1
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/PointField.h>
#else
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#endif

#include <mujoco_ros/logging.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>

using namespace mujoco_ros;

namespace mujoco_ros::sensors::lidar {

namespace {
static constexpr std::size_t LIDAR_PUBLISHER_QUEUE_SIZE = 1;
// Number of rays handed to a single threadpool task
static constexpr int RAYS_PER_TASK = 1024;
// Plastic constant, generator of the R2 low-discrepancy sequence
static constexpr double R2_G  = 1.32471795724474602596;
static constexpr double R2_A1 = 1.0 / R2_G;
static constexpr double R2_A2 = 1.0 / (R2_G * R2_G);

#if MJR_ROS_VERSION == ROS_1
using PointField = sensor_msgs::PointField;

ros::Publisher CreatePublisher(ros::NodeHandle &node, const std::string &topic)
{
	return node.advertise<PointCloud2>(topic, LIDAR_PUBLISHER_QUEUE_SIZE);
}

void Publish(const mujoco_ros::sensors::SensorConfig &config, const PointCloud2 &message)
{
	config.value_pub.publish(message);
}

ros::Publisher CreateImuPublisher(ros::NodeHandle &node, const std::string &topic)
{
	return node.advertise<Imu>(topic, 10);
}

RosTime Now(ros::NodeHandle & /*node*/)
{
	return ros::Time::now();
}

double SecondsSince(const RosTime &now, const RosTime &then)
{
	return (now - then).toSec();
}

RosTime AddSeconds(const RosTime &time, double seconds)
{
	return time + ros::Duration(seconds);
}

uint64_t Nanoseconds(const RosTime &time)
{
	return time.toNSec();
}

bool ReadBool(const XmlRpc::XmlRpcValue &config, const std::string &name, bool default_value)
{
	if (config.hasMember(name) && config[name].getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
		return static_cast<bool>(config[name]);
	}
	return default_value;
}

std::string ReadString(const XmlRpc::XmlRpcValue &config, const std::string &name, const std::string &default_value)
{
	if (config.hasMember(name) && config[name].getType() == XmlRpc::XmlRpcValue::TypeString) {
		return static_cast<std::string>(config[name]);
	}
	return default_value;
}

double ReadDouble(const XmlRpc::XmlRpcValue &config, const std::string &name, double default_value)
{
	if (!config.hasMember(name)) {
		return default_value;
	}
	if (config[name].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
		return static_cast<double>(config[name]);
	}
	if (config[name].getType() == XmlRpc::XmlRpcValue::TypeInt) {
		return static_cast<int>(config[name]);
	}
	return default_value;
}

int ReadInt(const XmlRpc::XmlRpcValue &config, const std::string &name, int default_value)
{
	if (config.hasMember(name) && config[name].getType() == XmlRpc::XmlRpcValue::TypeInt) {
		return static_cast<int>(config[name]);
	}
	return default_value;
}

std::vector<int> ReadIntArray(const XmlRpc::XmlRpcValue &config, const std::string &name)
{
	std::vector<int> values;
	if (!config.hasMember(name) || config[name].getType() != XmlRpc::XmlRpcValue::TypeArray) {
		return values;
	}
	for (int i = 0; i < config[name].size(); ++i) {
		if (config[name][i].getType() == XmlRpc::XmlRpcValue::TypeInt) {
			values.push_back(static_cast<int>(config[name][i]));
		}
	}
	return values;
}
#else
using PointField = sensor_msgs::msg::PointField;

rclcpp::Publisher<PointCloud2>::SharedPtr CreatePublisher(rclcpp_lifecycle::LifecycleNode::SharedPtr &node,
                                                          const std::string &topic)
{
	return node->create_publisher<PointCloud2>(topic, rclcpp::SensorDataQoS().keep_last(LIDAR_PUBLISHER_QUEUE_SIZE));
}

void Publish(const mujoco_ros::sensors::SensorConfig &config, const PointCloud2 &message)
{
	std::any_cast<rclcpp::Publisher<PointCloud2>::SharedPtr>(config.value_pub)->publish(message);
}

rclcpp::Publisher<Imu>::SharedPtr CreateImuPublisher(rclcpp_lifecycle::LifecycleNode::SharedPtr &node,
                                                     const std::string &topic)
{
	return node->create_publisher<Imu>(topic, rclcpp::SensorDataQoS());
}

RosTime Now(rclcpp_lifecycle::LifecycleNode::SharedPtr &node)
{
	return node->now();
}

double SecondsSince(const RosTime &now, const RosTime &then)
{
	return (now - then).seconds();
}

RosTime AddSeconds(const RosTime &time, double seconds)
{
	return time + rclcpp::Duration::from_seconds(seconds);
}

uint64_t Nanoseconds(const RosTime &time)
{
	return static_cast<uint64_t>(time.nanoseconds());
}

std::string SensorParamName(const std::string &plugin_name, const std::string &sensor_name,
                            const std::string &param_name)
{
	return "MujocoPlugins." + plugin_name + "." + sensor_name + "." + param_name;
}

bool GetBoolParam(const rclcpp::Node *node, const std::string &name, bool default_value)
{
	if (!node->has_parameter(name)) {
		return default_value;
	}
	return node->get_parameter(name).as_bool();
}

std::string GetStringParam(const rclcpp::Node *node, const std::string &name, const std::string &default_value)
{
	if (!node->has_parameter(name)) {
		return default_value;
	}
	return node->get_parameter(name).as_string();
}

double GetDoubleParam(const rclcpp::Node *node, const std::string &name, double default_value)
{
	if (!node->has_parameter(name)) {
		return default_value;
	}
	const auto parameter = node->get_parameter(name);
	if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
		return static_cast<double>(parameter.as_int());
	}
	return parameter.as_double();
}

int GetIntParam(const rclcpp::Node *node, const std::string &name, int default_value)
{
	if (!node->has_parameter(name)) {
		return default_value;
	}
	return static_cast<int>(node->get_parameter(name).as_int());
}

std::vector<int> GetIntArrayParam(const rclcpp::Node *node, const std::string &name)
{
	std::vector<int> values;
	if (!node->has_parameter(name)) {
		return values;
	}
	for (const auto value : node->get_parameter(name).as_integer_array()) {
		values.push_back(static_cast<int>(value));
	}
	return values;
}
#endif

void AddField(PointCloud2 &cloud, const std::string &name, uint32_t offset, uint8_t datatype)
{
	PointField field;
	field.name     = name;
	field.offset   = offset;
	field.datatype = datatype;
	field.count    = 1;
	cloud.fields.push_back(field);
}

void SphericalToDir(double azimuth, double elevation, mjtNum *dir)
{
	dir[0] = std::cos(elevation) * std::cos(azimuth);
	dir[1] = std::cos(elevation) * std::sin(azimuth);
	dir[2] = std::sin(elevation);
}

struct RayTaskArgs
{
	const mjModel *model;
	mjData *data;
	const mjtNum *pnt;
	const mjtNum *vec;
	const mjtByte *geomgroup;
	int bodyexclude;
	int *geomid;
	mjtNum *dist;
	int nray;
	mjtNum cutoff;
};

void *RayTask(void *args)
{
	auto *a = static_cast<RayTaskArgs *>(args);
	mj_multiRay(a->model, a->data, a->pnt, a->vec, a->geomgroup, 1, a->bodyexclude, a->geomid, a->dist, a->nray,
	            a->cutoff);
	return nullptr;
}
} // namespace

LidarConfig::LidarConfig(std::string frame_id, std::string name, int site_attached, int body_attached)
    : SensorConfig(std::move(frame_id))
    , name(std::move(name))
    , site_attached(site_attached)
    , body_attached(body_attached)
{
}

bool LidarPlugin::Load(const mjModel *model, mjData * /*data*/)
{
	MJR_INFO_NAMED("lidar", "Loading lidar plugin ...");

#if MJR_ROS_VERSION == ROS_1
	std::string lidar_namespace;
	if (rosparam_config_.hasMember("namespace")) {
		lidar_namespace = static_cast<std::string>(rosparam_config_["namespace"]);
	}
	lidar_nh_ = ros::NodeHandle(lidar_namespace);

	if (!rosparam_config_.hasMember("sensors") ||
	    rosparam_config_["sensors"].getType() != XmlRpc::XmlRpcValue::TypeArray) {
		MJR_ERROR_NAMED("lidar", "Lidar plugin needs a 'sensors' array with at least one sensor!");
		return false;
	}

	for (int i = 0; i < rosparam_config_["sensors"].size(); i++) {
		if (!InitSensor(model, rosparam_config_["sensors"][i])) {
			return false;
		}
	}
#else
	lidar_nh_ = get_node();

	const std::string sensors_param = "MujocoPlugins." + get_name() + ".sensors";
	if (!env_ptr_->has_parameter(sensors_param)) {
		MJR_ERROR_STREAM_NAMED("lidar", "Lidar plugin needs sensor names in parameter '" << sensors_param << "'");
		return false;
	}

	for (const auto &sensor_name : env_ptr_->get_parameter(sensors_param).as_string_array()) {
		if (!InitSensor(model, sensor_name)) {
			return false;
		}
	}
#endif
	return true;
}

#if MJR_ROS_VERSION == ROS_1
bool LidarPlugin::InitSensor(const mjModel *model, const XmlRpc::XmlRpcValue &config)
{
	if (!config.hasMember("site_attached")) {
		MJR_ERROR_NAMED("lidar", "Lidar sensor needs a 'site_attached' parameter!");
		return false;
	}
	const std::string site_attached = static_cast<std::string>(config["site_attached"]);
	const std::string topic_name    = ReadString(config, "name", site_attached + "_points");
#else
bool LidarPlugin::InitSensor(const mjModel *model, const std::string &sensor_name)
{
	const auto param = [&](const std::string &name) { return SensorParamName(get_name(), sensor_name, name); };

	if (!env_ptr_->has_parameter(param("site_attached"))) {
		MJR_ERROR_STREAM_NAMED("lidar", "Lidar sensor '" << sensor_name << "' needs a 'site_attached' parameter!");
		return false;
	}
	const std::string site_attached = env_ptr_->get_parameter(param("site_attached")).as_string();
	const std::string topic_name    = GetStringParam(env_ptr_, param("name"), sensor_name);
#endif

	const int site_id = mj_name2id(model, mjOBJ_SITE, site_attached.c_str());
	if (site_id == -1) {
		MJR_ERROR_STREAM_NAMED("lidar", "Site with name '" << site_attached << "' could not be found in the model!");
		return false;
	}
	const int body_id            = model->site_bodyid[site_id];
	const std::string body_frame = mj_id2name(model, mjOBJ_BODY, body_id);

#if MJR_ROS_VERSION == ROS_1
	LidarConfig cfg(ReadString(config, "frame_id", body_frame), topic_name, site_id, body_id);
	cfg.update_rate             = ReadDouble(config, "update_rate", DEFAULT_UPDATE_RATE);
	cfg.points_per_frame        = ReadInt(config, "points_per_frame", DEFAULT_POINTS_PER_FRAME);
	cfg.min_range               = ReadDouble(config, "min_range", DEFAULT_MIN_RANGE);
	cfg.max_range               = ReadDouble(config, "max_range", DEFAULT_MAX_RANGE);
	cfg.range_std               = ReadDouble(config, "range_std", DEFAULT_RANGE_STD);
	cfg.min_elevation           = ReadDouble(config, "min_elevation", DEFAULT_MIN_ELEVATION);
	cfg.max_elevation           = ReadDouble(config, "max_elevation", DEFAULT_MAX_ELEVATION);
	cfg.num_lines               = ReadInt(config, "num_lines", DEFAULT_NUM_LINES);
	cfg.visualize               = ReadBool(config, "visualize", DEFAULT_VISUALIZE);
	cfg.visualize_max_points    = ReadInt(config, "visualize_max_points", DEFAULT_VISUALIZE_MAX_POINTS);
	cfg.visualize_point_size    = ReadDouble(config, "visualize_point_size", DEFAULT_VISUALIZE_POINT_SIZE);
	cfg.ignore_geom_groups      = ReadIntArray(config, "ignore_geom_groups");
	const std::string pattern   = ReadString(config, "scan_pattern", "mid360");
	const std::string intensity = ReadString(config, "intensity_source", "color");
	cfg.pattern_file            = ReadString(config, "scan_pattern_file", "");
	const std::string imu_site  = ReadString(config, "imu_site", "");
	const std::string imu_name  = ReadString(config, "imu_name", "livox/imu");
	cfg.imu_frame_id            = ReadString(config, "imu_frame_id", cfg.frame_id);
	cfg.imu_rate                = ReadDouble(config, "imu_rate", DEFAULT_IMU_RATE);
	cfg.imu_gyro_std            = ReadDouble(config, "imu_gyro_std", DEFAULT_IMU_GYRO_STD);
	cfg.imu_accel_std           = ReadDouble(config, "imu_accel_std", DEFAULT_IMU_ACCEL_STD);
#else
	LidarConfig cfg(GetStringParam(env_ptr_, param("frame_id"), body_frame), topic_name, site_id, body_id);
	cfg.update_rate             = GetDoubleParam(env_ptr_, param("update_rate"), DEFAULT_UPDATE_RATE);
	cfg.points_per_frame        = GetIntParam(env_ptr_, param("points_per_frame"), DEFAULT_POINTS_PER_FRAME);
	cfg.min_range               = GetDoubleParam(env_ptr_, param("min_range"), DEFAULT_MIN_RANGE);
	cfg.max_range               = GetDoubleParam(env_ptr_, param("max_range"), DEFAULT_MAX_RANGE);
	cfg.range_std               = GetDoubleParam(env_ptr_, param("range_std"), DEFAULT_RANGE_STD);
	cfg.min_elevation           = GetDoubleParam(env_ptr_, param("min_elevation"), DEFAULT_MIN_ELEVATION);
	cfg.max_elevation           = GetDoubleParam(env_ptr_, param("max_elevation"), DEFAULT_MAX_ELEVATION);
	cfg.num_lines               = GetIntParam(env_ptr_, param("num_lines"), DEFAULT_NUM_LINES);
	cfg.visualize               = GetBoolParam(env_ptr_, param("visualize"), DEFAULT_VISUALIZE);
	cfg.visualize_max_points    = GetIntParam(env_ptr_, param("visualize_max_points"), DEFAULT_VISUALIZE_MAX_POINTS);
	cfg.visualize_point_size    = GetDoubleParam(env_ptr_, param("visualize_point_size"), DEFAULT_VISUALIZE_POINT_SIZE);
	cfg.ignore_geom_groups      = GetIntArrayParam(env_ptr_, param("ignore_geom_groups"));
	const std::string pattern   = GetStringParam(env_ptr_, param("scan_pattern"), "mid360");
	const std::string intensity = GetStringParam(env_ptr_, param("intensity_source"), "color");
	cfg.pattern_file            = GetStringParam(env_ptr_, param("scan_pattern_file"), "");
	const std::string imu_site  = GetStringParam(env_ptr_, param("imu_site"), "");
	const std::string imu_name  = GetStringParam(env_ptr_, param("imu_name"), "livox/imu");
	cfg.imu_frame_id            = GetStringParam(env_ptr_, param("imu_frame_id"), cfg.frame_id);
	cfg.imu_rate                = GetDoubleParam(env_ptr_, param("imu_rate"), DEFAULT_IMU_RATE);
	cfg.imu_gyro_std            = GetDoubleParam(env_ptr_, param("imu_gyro_std"), DEFAULT_IMU_GYRO_STD);
	cfg.imu_accel_std           = GetDoubleParam(env_ptr_, param("imu_accel_std"), DEFAULT_IMU_ACCEL_STD);
#endif

	if (cfg.update_rate <= 0. || cfg.points_per_frame == 0 || cfg.num_lines == 0) {
		MJR_ERROR_STREAM_NAMED("lidar", "Lidar '" << topic_name
		                                          << "': update_rate, points_per_frame and num_lines must be positive!");
		return false;
	}

	if (pattern == "mid360") {
		cfg.pattern = ScanPattern::MID360;
	} else if (pattern == "csv") {
		cfg.pattern = ScanPattern::CSV;
	} else {
		MJR_ERROR_STREAM_NAMED("lidar", "Unknown scan_pattern '" << pattern << "'. Use 'mid360' or 'csv'.");
		return false;
	}

	if (intensity == "color") {
		cfg.intensity_source = IntensitySource::COLOR;
	} else if (intensity == "geom_user") {
		if (model->nuser_geom < 1) {
			MJR_ERROR_NAMED("lidar", "intensity_source 'geom_user' requires <size nuser_geom=\"1\"/> in the model!");
			return false;
		}
		cfg.intensity_source = IntensitySource::GEOM_USER;
	} else {
		MJR_ERROR_STREAM_NAMED("lidar", "Unknown intensity_source '" << intensity << "'. Use 'color' or 'geom_user'.");
		return false;
	}

	if (!InitPattern(cfg)) {
		return false;
	}

	const uint n = cfg.points_per_frame;
	cfg.dirs_local.resize(3 * n);
	cfg.dirs_world.resize(3 * n);
	cfg.hit_world.resize(3 * n);
	cfg.dists.resize(n);
	cfg.geomids.resize(n);
	cfg.point_times.resize(n);
	cfg.point_lines.resize(n);

	if (!imu_site.empty()) {
		cfg.imu_site = mj_name2id(model, mjOBJ_SITE, imu_site.c_str());
		if (cfg.imu_site == -1) {
			MJR_ERROR_STREAM_NAMED("lidar", "IMU site '" << imu_site << "' could not be found in the model!");
			return false;
		}
		if (cfg.imu_rate <= 0.) {
			MJR_ERROR_STREAM_NAMED("lidar", "Lidar '" << topic_name << "': imu_rate must be positive!");
			return false;
		}
		cfg.imu_pub = CreateImuPublisher(lidar_nh_, imu_name);
		MJR_INFO_STREAM_NAMED("lidar",
		                      "Added IMU '" << imu_name << "' at site '" << imu_site << "' (" << cfg.imu_rate << " Hz)");
	}

	cfg.RegisterPub(CreatePublisher(lidar_nh_, topic_name));
	lidar_configs_.push_back(std::move(cfg));

	MJR_INFO_STREAM_NAMED("lidar", "Added lidar '" << topic_name << "' at site '" << site_attached << "' (" << n
	                                               << " points @ " << lidar_configs_.back().update_rate << " Hz)");
	return true;
}

bool LidarPlugin::InitPattern(LidarConfig &config)
{
	if (config.pattern != ScanPattern::CSV) {
		return true;
	}

	// Expected format (livox_laser_simulation): "time[s], azimuth[deg], zenith[deg]" per line, optional header.
	// The zenith column is the angle measured from the sensor's +z axis, i.e. elevation = 90deg - zenith.
	std::ifstream file(config.pattern_file);
	if (!file.is_open()) {
		MJR_ERROR_STREAM_NAMED("lidar", "Could not open scan pattern file '" << config.pattern_file << "'");
		return false;
	}

	std::string line;
	while (std::getline(file, line)) {
		std::replace(line.begin(), line.end(), ',', ' ');
		std::istringstream ss(line);
		double time, azimuth, zenith;
		if (!(ss >> time >> azimuth >> zenith)) {
			continue; // header or malformed line
		}
		mjtNum dir[3];
		SphericalToDir(azimuth * M_PI / 180., (90. - zenith) * M_PI / 180., dir);
		config.pattern_dirs.insert(config.pattern_dirs.end(), dir, dir + 3);
	}

	if (config.pattern_dirs.empty()) {
		MJR_ERROR_STREAM_NAMED("lidar", "Scan pattern file '" << config.pattern_file << "' contains no valid rows");
		return false;
	}
	MJR_INFO_STREAM_NAMED("lidar",
	                      "Loaded " << config.pattern_dirs.size() / 3 << " rays from '" << config.pattern_file << "'");
	return true;
}

void LidarPlugin::GenerateDirections(LidarConfig &config)
{
	const uint n           = config.points_per_frame;
	const double frame_dur = 1.0 / config.update_rate;

	if (config.pattern == ScanPattern::CSV) {
		const size_t pattern_size = config.pattern_dirs.size() / 3;
		for (uint i = 0; i < n; ++i) {
			mju_copy3(config.dirs_local.data() + 3 * i, config.pattern_dirs.data() + 3 * config.pattern_index);
			config.pattern_index = (config.pattern_index + 1) % pattern_size;
		}
	} else {
		// Built-in Mid-360 approximation: The real sensor uses a non-repetitive scan, so coverage of the field of view
		// grows with integration time. We mimic this with the R2 low-discrepancy sequence, which is continued across
		// frames. Elevations are sampled uniformly in sin(elevation) to get an (approximately) equal-area distribution.
		const double s_min = std::sin(config.min_elevation);
		const double s_max = std::sin(config.max_elevation);
		for (uint i = 0; i < n; ++i) {
			config.r2_u += R2_A1;
			config.r2_v += R2_A2;
			config.r2_u -= std::floor(config.r2_u);
			config.r2_v -= std::floor(config.r2_v);

			const double azimuth   = 2. * M_PI * config.r2_u;
			const double elevation = std::asin(s_min + config.r2_v * (s_max - s_min));
			SphericalToDir(azimuth, elevation, config.dirs_local.data() + 3 * i);
		}
	}

	for (uint i = 0; i < n; ++i) {
		config.point_times[i] = static_cast<float>(frame_dur * i / n);
		config.point_lines[i] = static_cast<uint8_t>((config.emitted + i) % config.num_lines);
	}
	config.emitted += n;
}

void LidarPlugin::StartFrame(LidarConfig &config, const RosTime &start)
{
	config.last_update_time             = start;
	config.last_update_time_initialized = true;
	config.ncast                        = 0;
	GenerateDirections(config);
}

void LidarPlugin::CastRays(const mjModel *model, mjData *data, LidarConfig &config, int start, int count)
{
	const mjtNum *rot = data->site_xmat + 9 * config.site_attached;
	for (int i = start; i < start + count; ++i) {
		mju_mulMatVec3(config.dirs_world.data() + 3 * i, rot, config.dirs_local.data() + 3 * i);
	}

	mjtByte geomgroup[mjNGROUP];
	std::fill(geomgroup, geomgroup + mjNGROUP, 1);
	for (const int group : config.ignore_geom_groups) {
		if (group >= 0 && group < mjNGROUP) {
			geomgroup[group] = 0;
		}
	}

	const mjtNum *pnt = data->site_xpos + 3 * config.site_attached;

	if (!data->threadpool || count <= RAYS_PER_TASK) {
		mj_multiRay(model, data, pnt, config.dirs_world.data() + 3 * start, geomgroup, 1, config.body_attached,
		            config.geomids.data() + start, config.dists.data() + start, count, config.max_range);
	} else {
		// Split rays into chunks and evaluate them in MuJoCo's threadpool
		const int ntasks = (count + RAYS_PER_TASK - 1) / RAYS_PER_TASK;
		std::vector<RayTaskArgs> args(ntasks);
		std::vector<mjTask> tasks(ntasks);
		for (int t = 0; t < ntasks; ++t) {
			const int first = start + t * RAYS_PER_TASK;
			args[t]         = { model,
				                 data,
				                 pnt,
				                 config.dirs_world.data() + 3 * first,
				                 geomgroup,
				                 config.body_attached,
				                 config.geomids.data() + first,
				                 config.dists.data() + first,
				                 std::min(RAYS_PER_TASK, start + count - first),
				                 config.max_range };
			mju_defaultTask(&tasks[t]);
			tasks[t].func = RayTask;
			tasks[t].args = &args[t];
			mju_threadPoolEnqueue(reinterpret_cast<mjThreadPool *>(data->threadpool), &tasks[t]);
		}
		for (auto &task : tasks) {
			mju_taskJoin(&task);
		}
	}

	// Remember hit positions in world frame (at emission) for visualization
	if (config.visualize) {
		for (int i = start; i < start + count; ++i) {
			mju_addScl3(config.hit_world.data() + 3 * i, pnt, config.dirs_world.data() + 3 * i,
			            std::max<mjtNum>(config.dists[i], 0.));
		}
	}
}

void LidarPlugin::FillCloud(const mjModel *model, const mjData * /*data*/, LidarConfig &config, PointCloud2 &cloud,
                            std::vector<mjvGeom> &vis_geoms)
{
	const uint n = config.points_per_frame;

	AddField(cloud, "x", offsetof(LivoxPoint, x), PointField::FLOAT32);
	AddField(cloud, "y", offsetof(LivoxPoint, y), PointField::FLOAT32);
	AddField(cloud, "z", offsetof(LivoxPoint, z), PointField::FLOAT32);
	AddField(cloud, "intensity", offsetof(LivoxPoint, intensity), PointField::FLOAT32);
	AddField(cloud, "tag", offsetof(LivoxPoint, tag), PointField::UINT8);
	AddField(cloud, "line", offsetof(LivoxPoint, line), PointField::UINT8);
	AddField(cloud, "timestamp", offsetof(LivoxPoint, timestamp), PointField::FLOAT64);
	AddField(cloud, "time", offsetof(LivoxPoint, time), PointField::FLOAT32);
	AddField(cloud, "ring", offsetof(LivoxPoint, ring), PointField::UINT16);
	cloud.is_bigendian = false;
	cloud.point_step   = sizeof(LivoxPoint);
	cloud.height       = 1;
	cloud.is_dense     = true;
	cloud.data.resize(static_cast<size_t>(n) * sizeof(LivoxPoint));

	const double stamp_ns    = static_cast<double>(Nanoseconds(RosTime(cloud.header.stamp)));
	const uint vis_stride    = config.visualize ? std::max(1u, n / std::max(1u, config.visualize_max_points)) : 0;
	const mjtNum vis_size[3] = { config.visualize_point_size, 0, 0 };
	const bool add_noise     = config.range_std > 0.;

	uint npoints = 0;
	for (uint i = 0; i < n; ++i) {
		mjtNum dist = config.dists[i];
		if (dist < 0.) { // no hit within max_range
			continue;
		}
		if (add_noise) {
			dist += config.range_std * noise_dist_(rand_generator_);
		}
		if (dist < config.min_range || dist > config.max_range) {
			continue;
		}

		const int geom_id = config.geomids[i];
		float intensity   = 0.f;
		if (geom_id >= 0) {
			if (config.intensity_source == IntensitySource::GEOM_USER) {
				intensity = static_cast<float>(model->geom_user[geom_id * model->nuser_geom]);
			} else {
				const int mat_id  = model->geom_matid[geom_id];
				const float *rgba = mat_id >= 0 ? model->mat_rgba + 4 * mat_id : model->geom_rgba + 4 * geom_id;
				intensity         = 255.f * (0.299f * rgba[0] + 0.587f * rgba[1] + 0.114f * rgba[2]);
			}
		}

		LivoxPoint p;
		const mjtNum *dir = config.dirs_local.data() + 3 * i;
		p.x               = static_cast<float>(dist * dir[0]);
		p.y               = static_cast<float>(dist * dir[1]);
		p.z               = static_cast<float>(dist * dir[2]);
		p.intensity       = intensity;
		p.tag             = 0;
		p.line            = config.point_lines[i];
		p.timestamp       = stamp_ns + 1e9 * config.point_times[i];
		p.time            = config.point_times[i];
		p.ring            = config.point_lines[i];
		std::memcpy(cloud.data.data() + static_cast<size_t>(npoints) * sizeof(LivoxPoint), &p, sizeof(LivoxPoint));
		++npoints;

		if (vis_stride > 0 && i % vis_stride == 0) {
			const mjtNum *pos = config.hit_world.data() + 3 * i;
			// color by range: red (near) -> blue (far)
			const float t        = static_cast<float>(dist / config.max_range);
			const float color[4] = { 1.f - t, 0.2f, t, 1.f };
			mjvGeom geom;
			mjv_initGeom(&geom, mjGEOM_SPHERE, vis_size, pos, nullptr, color);
			geom.category = mjCAT_DECOR;
			vis_geoms.push_back(geom);
		}
	}

	cloud.width    = npoints;
	cloud.row_step = npoints * cloud.point_step;
	cloud.data.resize(static_cast<size_t>(cloud.row_step));
}

void LidarPlugin::PublishImu(const mjModel *model, LidarConfig &config, const RosTime &now, const mjtNum *acc6,
                             const mjtNum *vel6)
{
	// Like the real driver, linear acceleration is reported in units of g (1 g = |gravity| of the model)
	const mjtNum g_norm = mju_norm3(model->opt.gravity);
	const mjtNum g      = g_norm > 0. ? g_norm : 9.80665;

	Imu msg;
	msg.header.stamp    = now;
	msg.header.frame_id = config.imu_frame_id;
	// No orientation estimate, as for the real sensor
	msg.orientation_covariance[0] = -1.;

	msg.angular_velocity.x    = vel6[0] + config.imu_gyro_std * noise_dist_(rand_generator_);
	msg.angular_velocity.y    = vel6[1] + config.imu_gyro_std * noise_dist_(rand_generator_);
	msg.angular_velocity.z    = vel6[2] + config.imu_gyro_std * noise_dist_(rand_generator_);
	msg.linear_acceleration.x = acc6[3] / g + config.imu_accel_std * noise_dist_(rand_generator_);
	msg.linear_acceleration.y = acc6[4] / g + config.imu_accel_std * noise_dist_(rand_generator_);
	msg.linear_acceleration.z = acc6[5] / g + config.imu_accel_std * noise_dist_(rand_generator_);

	const double gyro_var  = config.imu_gyro_std * config.imu_gyro_std;
	const double accel_var = config.imu_accel_std * config.imu_accel_std;
	for (int i = 0; i < 3; ++i) {
		msg.angular_velocity_covariance[4 * i]    = gyro_var;
		msg.linear_acceleration_covariance[4 * i] = accel_var;
	}

#if MJR_ROS_VERSION == ROS_1
	config.imu_pub.publish(msg);
#else
	config.imu_pub->publish(msg);
#endif
}

void LidarPlugin::LastStageCallback(const mjModel *model, mjData *data)
{
	bool updated = false;

	// IMU: Called after mj_step, i.e. kinematics and qacc still describe the state before integration (same as
	// MuJoCo's own sensordata). cacc is only computed by MuJoCo if the model has acceleration-based sensors, so
	// compute it here once if any IMU is due.
	bool rne_done = false;
	for (auto &config : lidar_configs_) {
		if (config.imu_site < 0) {
			continue;
		}
		const auto now = Now(lidar_nh_);
		if (config.imu_last_update_time_initialized &&
		    SecondsSince(now, config.imu_last_update_time) < 1.0 / config.imu_rate) {
			continue;
		}
		config.imu_last_update_time             = now;
		config.imu_last_update_time_initialized = true;
		updated                                 = true;

		if (!rne_done) {
			mj_rnePostConstraint(model, data);
			rne_done = true;
		}
		// Same computation as MuJoCo's accelerometer / gyro sensors (gravity is included via cacc of the world body)
		mjtNum acc6[6], vel6[6];
		mj_objectAcceleration(model, data, mjOBJ_SITE, config.imu_site, acc6, 1);
		mj_objectVelocity(model, data, mjOBJ_SITE, config.imu_site, vel6, 1);
		PublishImu(model, config, now, acc6, vel6);
	}

	for (auto &config : lidar_configs_) {
		const auto now         = Now(lidar_nh_);
		const uint n           = config.points_per_frame;
		const double frame_dur = 1.0 / config.update_rate;

		if (!config.last_update_time_initialized) {
			StartFrame(config, now);
		} else if (config.ncast == n) {
			// previous frame is complete, wait for the next one. Keep the frame phase unless we fell behind (e.g. pause)
			const RosTime next = AddSeconds(config.last_update_time, frame_dur);
			if (now < next) {
				continue;
			}
			StartFrame(config, SecondsSince(now, next) < frame_dur ? next : now);
		}

		// cast all rays whose emission time (frame start + i * frame_dur / n) has been reached
		const double elapsed = SecondsSince(now, config.last_update_time);
		const uint target    = std::min<uint>(n, static_cast<uint>(std::floor(elapsed / frame_dur * n)) + 1);
		if (target > config.ncast) {
			CastRays(model, data, config, config.ncast, target - config.ncast);
			config.ncast = target;
			updated      = true;
		}
		if (config.ncast < n) {
			continue;
		}

		PointCloud2 cloud;
		cloud.header.stamp    = config.last_update_time;
		cloud.header.frame_id = config.frame_id;
		std::vector<mjvGeom> vis_geoms;
		FillCloud(model, data, config, cloud, vis_geoms);
		Publish(config, cloud);

		if (config.visualize) {
			std::lock_guard<std::mutex> lock(render_mutex_);
			config.vis_geoms = std::move(vis_geoms);
		}
	}

	if (!updated) {
		skip_ema_ = true;
	}
}

void LidarPlugin::RenderCallback(const mjModel * /*model*/, mjData * /*data*/, mjvScene *scene)
{
	std::lock_guard<std::mutex> lock(render_mutex_);
	for (const auto &config : lidar_configs_) {
		for (const auto &geom : config.vis_geoms) {
			if (scene->ngeom >= scene->maxgeom) {
				return;
			}
			scene->geoms[scene->ngeom++] = geom;
		}
	}
}

void LidarPlugin::Reset()
{
	for (auto &config : lidar_configs_) {
		config.last_update_time_initialized     = false;
		config.imu_last_update_time_initialized = false;
		config.ncast                            = 0;
	}
	std::lock_guard<std::mutex> lock(render_mutex_);
	for (auto &config : lidar_configs_) {
		config.vis_geoms.clear();
	}
}

} // namespace mujoco_ros::sensors::lidar

PLUGINLIB_EXPORT_CLASS(mujoco_ros::sensors::lidar::LidarPlugin, mujoco_ros::MujocoPlugin)
