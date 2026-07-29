# T265 TF and PX4 coordinate contract

## Required physical TF chain

The recorded T265 chain is `odom_frame -> t265_pose_frame`.  `vision_to_dds`
therefore consumes exactly this composed transform:

```text
odom_frame -> t265_pose_frame -> base_link
              ^ T265 data       ^ static, measured by Workstation 1
```

The second edge is not a software default.  It is a rigid physical extrinsic
and must be measured on the installed aircraft by Workstation 1.  The
repository intentionally contains only
`config/t265_to_base_link.extrinsics.yaml.template`, with no numeric mounting
values.  Copy it to a controlled local file, enter the measured translation
and unit quaternion, review it, then pass it explicitly to the production
launch.  The launch rejects absent, non-finite, non-unit, or differently named
extrinsics.

The normal launch never starts a static TF publisher.  It defaults to
`world_frame_id=odom_frame`, `body_frame_id=base_link`, and
`enable_vision_dds=false`.

## Axis and attitude conversion

T265 ROS TF is interpreted as ENU world and FLU body:

```text
ENU: +X east, +Y north, +Z up
FLU: +X forward, +Y left, +Z up
```

PX4 `VehicleOdometry` is emitted only after the contract is healthy and uses:

```text
NED: +X north, +Y east, +Z down
FRD: +X forward, +Y right, +Z down
```

For a position/vector `[x, y, z]` in ENU, the required NED vector is
`[y, x, -z]`.  Attitude is transformed as:

```text
R_NED_FRD = R_NED_ENU * R_ENU_FLU * R_FLU_FRD
q_NED_ENU = (sqrt(1/2), sqrt(1/2), 0, 0)  # xyzw
q_FLU_FRD = (1, 0, 0, 0)                  # xyzw
```

The existing `VisionContract::enuFluToNedFrd()` uses precisely that product,
then normalizes it.  It is mathematically correct; the test vectors in
`test_t265_tf_contract.cpp` protect the mapping.

## Timestamp, source and writer constraints

The composed TF must carry its original T265 timestamp on the synchronized
ROS/XRCE time base.  A zero, old, future, non-monotonic, or excessively jumped
timestamp latches a fault.  A changed source epoch, low/stale quality, missing
TF, and duplicate publisher of `/fmu/in/vehicle_visual_odometry` also stop
production output.  Publisher cardinality is a property of the target topic's
publisher endpoint information, not the count of nodes sharing a name.

`enable_vision_dds=false` is a separate production interlock: diagnostics may
remain alive, but no visual-odometry DDS publisher may be created.  The
production launch requires all of the following explicit arguments:

```bash
ros2 launch vision_to_dds vision_to_dds_production.launch.py \
  production:=true enable_vision_dds:=true \
  t265_to_base_link_extrinsics_file:=/absolute/path/measured.yaml
```

Before Workstation 1 provides and validates the measured extrinsics, this
bridge must not be used as PX4 EKF production input or for propeller-on flight.

## Verification

```bash
colcon test --packages-select vision_to_dds --ctest-args -R t265_tf_contract
ros2 launch vision_to_dds vision_to_dds.launch.py
ros2 topic info -v /fmu/in/vehicle_visual_odometry
```

The default graph must show zero publishers on that topic.  With the explicit
production command above, it must show exactly one; health, source epoch,
quality, timestamps, TF frames, and freeze checking remain required before any
message is published.
