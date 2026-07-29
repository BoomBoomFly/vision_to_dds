# vision_to_dds

Fail-closed T265 visual-odometry bridge for PX4.  Its production DDS writer
is disabled by default.

## Parameters

`enable_vision_dds` is a bool and defaults to `false`.  In that state the
node creates no `/fmu/in/vehicle_visual_odometry` publisher and emits no
`px4_msgs/msg/VehicleOdometry`.  To create exactly one writer, use:

```bash
ros2 launch vision_to_dds vision_to_dds.launch.py enable_vision_dds:=true
```

Creation of the writer is not permission to publish: source epoch, measured
quality, original synchronized timestamps, TF frames, single-writer topology,
two-sample warm-up and freeze detection must all pass.  Any fault latches and
stops output.

The default frame contract is `odom_frame -> base_link`.  The real T265 input
is `odom_frame -> t265_pose_frame`; Workstation 1 must provide the measured
`t265_pose_frame -> base_link` extrinsic.  No mounting values are shipped.
See `T265_TF_CONTRACT.md` and `VISION_DDS_H0.md`.

Production launch additionally requires explicit `production:=true`,
`enable_vision_dds:=true`, and a measured extrinsics file.  Until that
extrinsic and replay validation are complete, do not use this package as PX4
EKF input or for propeller-on flight.

The production launch also starts `t265_health_adapter_node` against the
observed aircraft topic `/t265/pose/sample`.  It converts the RealSense
wrapper's covariance-encoded tracker confidence into `/vision/quality` and
publishes `/vision/source_epoch`; neither value is a fixed or mock health
signal.  Before the first real T265 odometry sample it publishes no quality
sample, so ordinary process startup remains fail-closed warm-up instead of
fabricating quality zero and latching a false outage.  Once a source has been
observed, a freeze or disconnect does publish quality zero and recovery
advances the source epoch.  The launch forcibly retains `odom_frame`,
`base_link`, and
`/fmu/in/vehicle_visual_odometry` even when a custom parameter file is used.
