# MuJoCo ROS LiDAR Plugin

CPU-based 3D LiDAR plugin publishing `sensor_msgs/PointCloud2`. Defaults mimic a **Livox Mid-360**
(360° x [-7°, 52°], 200k pts/s, 0.1–40 m). Rays are cast with `mj_multiRay` and split into chunks
on MuJoCo's threadpool if more than one thread is configured.

## Output

Points are expressed in the sensor `frame_id` and use the same layout as `livox_ros_driver2` (`xfer_format: 0`):

| field | type | description |
|---|---|---|
| x, y, z | float32 | point in sensor frame [m] |
| intensity | float32 | see `intensity_source` |
| tag | uint8 | always 0 |
| line | uint8 | laser line id (`0 .. num_lines-1`) |
| timestamp | float64 | absolute time [ns] (header stamp + in-frame offset) |
| time | float32 | offset from the header stamp [s] (velodyne-style, e.g. FAST-LIO `lidar_type: 2`, `timestamp_unit: 0`) |
| ring | uint16 | same as `line` (velodyne-style) |

Rays without a hit within `max_range` are dropped (no "max range" points). Like the real sensor, the rays of a
frame are cast progressively over the frame duration (ray i at header stamp + `time`), so motion during a scan
distorts the cloud and can be compensated with the per-point time.

### FAST-LIO

Use the PointCloud2 (velodyne-style) input instead of the livox `CustomMsg`:

```yaml
preprocess:
  lidar_type: 2      # PointCloud2 with per-point time / ring
  timestamp_unit: 0  # 'time' is in seconds
  scan_line: 4       # = num_lines
```

### IMU

If `imu_site` is set, a `sensor_msgs/Imu` is published like `livox_ros_driver2`: angular velocity in rad/s,
**linear acceleration in units of g** (divided by |gravity| of the model, so it reads ~(0, 0, 1) at rest),
no orientation (`orientation_covariance[0] = -1`). Values are computed like MuJoCo's `gyro` / `accelerometer`
sensors at the given site, so no `<sensor>` in the model and no `mujoco_ros_sensors` plugin is needed.
Noise is white gaussian only (no bias / random walk).

Default rates follow `livox_ros_driver2` with Mid-360: point cloud 10 Hz (`publish_freq`), IMU 200 Hz.

## Parameters (per sensor)

| name | default | description |
|---|---|---|
| site_attached | (required) | site the sensor is attached to (x forward, z up) |
| name | `<site>_points` | topic name |
| frame_id | parent body name | frame of the published cloud |
| scan_pattern | `mid360` | `mid360`: built-in non-repetitive pattern (R2 low-discrepancy sequence, continued across frames). `csv`: load from `scan_pattern_file` |
| scan_pattern_file | "" | csv with rows `time[s], azimuth[deg], zenith[deg]` (livox_laser_simulation format, elevation = 90° - zenith). Rows are consumed cyclically, `points_per_frame` per frame |
| update_rate | 10.0 | Hz |
| points_per_frame | 20000 | rays per frame |
| min_range / max_range | 0.1 / 40.0 | m |
| range_std | 0.02 | additive gaussian range noise [m] |
| min_elevation / max_elevation | -0.1222 / 0.9076 | rad (mid360 pattern only) |
| num_lines | 4 | number of line ids assigned round-robin |
| intensity_source | `color` | `color`: luminance of hit geom / material rgba (0–255). `geom_user`: first value of `<geom user="..."/>` (requires `<size nuser_geom="1"/>`) |
| visualize | false | draw hit points in the MuJoCo viewer. RViz is usually sufficient |
| visualize_max_points | 2000 | max. drawn points per frame |
| visualize_point_size | 0.02 | sphere radius [m] |
| ignore_geom_groups | [] | geom groups invisible to the sensor |
| imu_site | "" | site of the built-in IMU. IMU is disabled if empty |
| imu_name | `livox/imu` | IMU topic name (`sensor_msgs/Imu`) |
| imu_frame_id | `frame_id` | frame of the IMU message (the real driver uses the lidar frame for both) |
| imu_rate | 200.0 | Hz |
| imu_gyro_std | 0.001 | additive gaussian white noise [rad/s] |
| imu_accel_std | 0.002 | additive gaussian white noise [g] |

Note: like `mj_ray`, geoms with `rgba` alpha 0 and the body the site is attached to are ignored.

## Example

```bash
roslaunch mujoco_ros_lidar mid360_example.launch rviz:=true
```

See `config/mid360_example_config.yaml` (ROS 1) and `config/mid360_example_config.ros2.yaml` (ROS 2).

## License

This repository is licensed under the [BSD License](../mujoco_ros/LICENSE).
