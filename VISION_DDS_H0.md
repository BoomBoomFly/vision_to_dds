# Vision DDS H0 production interlock

`vision_to_dds` is **disabled by default** for PX4 external-vision input.
The `enable_vision_dds` node parameter defaults to `false`.  In that state
the node may subscribe to health information and publish diagnostics, but it
must neither create nor publish to `/fmu/in/vehicle_visual_odometry`.

## Production enablement

Production use requires all of the following, in this order:

1. A launch invocation explicitly sets `enable_vision_dds:=true`.
2. Station 1 supplies a measured `t265_pose_frame -> base_link` extrinsic;
   this repository intentionally contains no guessed transform.
3. The TF lookup is exactly `odom_frame -> base_link`, with a current,
   monotonic timestamp from the synchronized ROS/XRCE time base.
4. A current source epoch and acceptable measured quality are available.
5. Exactly one publisher exists on `/fmu/in/vehicle_visual_odometry`.
6. Two fresh TF samples complete the contract warm-up and no contract fault is
   latched.

Any low quality, source epoch change, missing/stale/frozen TF sample,
timestamp regression, frame mismatch, duplicate publisher, or latched fault
stops VehicleOdometry output.  Recovery requires operator fault reset and a
fresh warm-up; it is never automatic.

## Verification

With the workspace sourced, use the launch and ROS graph checks below:

```bash
ros2 launch vision_to_dds vision_to_dds.launch.py
ros2 topic info /fmu/in/vehicle_visual_odometry --verbose

ros2 launch vision_to_dds vision_to_dds.launch.py enable_vision_dds:=true
ros2 topic info /fmu/in/vehicle_visual_odometry --verbose
```

The first invocation must report no publisher.  The second may report exactly
one, but it must still emit no valid VehicleOdometry until all contract inputs
are healthy.  There is no approval in this document to fly or to change PX4
EKF parameters.

## Rollback

Remove the explicit `enable_vision_dds:=true` launch override (or set it to
`false`) and restart the node.  The DDS writer will not be constructed.
