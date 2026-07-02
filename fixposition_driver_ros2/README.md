# Fixposition ROS2 Driver

[ROS](https://www.ros.org/) Driver for [Fixposition Vision-RTK 2](https://www.fixposition.com/product).

See the [main README.md](../README.md) for the documentation.

## Nav2 planar odometry

The ROS 2 driver can publish a local planar odometry stream for Nav2 using both
Fixposition pose products:

- `FP_A-ODOMSH` provides the continuous smooth pose used for
  `odom -> base_frame`.
- `FP_A-ODOMENU` provides the global `FP_ENU0 -> FP_POI` pose used to update
  `map -> odom` and absorb GNSS re-fixes.

```yaml
nav2_mode: true
odom_frame: odom
planar_mode:
  enabled: true
  latch_origin: true
  planar_topic: /fixposition/odometry_planar
  planar_odom_frame: odom_nav
  base_frame: base_footprint
  sensor_to_base_x: 0.0
  sensor_to_base_y: 0.0
  sensor_to_base_z: -0.180998
  sensor_to_base_yaw: 0.0
  zero_initial_yaw: false
  publish_tf: true
```

`nav2_mode` is the switch for Nav2-friendly odometry and TF output. Planar mode
is a projection mode within Nav2 mode: when it is enabled, the driver latches a
local planar origin and publishes the Nav2 odometry relative to that origin.

With `latch_origin: true`, the first base-frame ENU position becomes the planar
origin. With `zero_initial_yaw: false`, the planar frame remains aligned with
ENU and the robot keeps its initial ENU heading.

If the Fixposition POI is configured to coincide with `base_link` while Nav2
uses `base_footprint`, keep `sensor_to_base_x`, `sensor_to_base_y`, and
`sensor_to_base_yaw` at zero and set `sensor_to_base_z` to the inverse of the
URDF `base_footprint -> base_link` height. For the Scout Mini URDF this is
`-0.180998`. If the POI is configured directly at `base_footprint`, use zero for
all `sensor_to_base_*` values.

When planar mode publishes TF, the Nav2 chain is:

```text
FP_ENU0 -> map -> planar_odom_frame -> base_frame
```

The `planar_odom_frame -> base_frame` transform and
`/fixposition/odometry_planar` are derived from the smooth pose, so local Nav2
odometry remains continuous. The `map -> planar_odom_frame` transform is derived
from the global pose and may move when the Fixposition global solution changes.
The z offset is applied only to this global correction; the local planar odom
message keeps z, roll, and pitch projected out.

The raw Fixposition topics and `FP_*` TF tree remain available. Physical sensor
extrinsics, such as cameras, should be defined relative to the raw Fixposition
sensor frame `FP_VRTK`.
