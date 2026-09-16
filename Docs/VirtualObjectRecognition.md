# Virtual-object recognition

Open **New Session → select an agent → Recognition settings**. Save the settings,
then load the agent and map. Changes apply when the next session starts.

The former `LincolnMKZ-Simple` bounding-box agent is split into two presets:

| Agent | ROS clock | Localization bridge |
|---|---|---|
| `LincolnMKZ-CARLA-Boxes` | Simulation `/clock` | `launch_mr_pkg_dev.sh` |
| `LincolnMKZ-Physical-Boxes` | System time | `launch_mr_pkg_dev.sh --physical` |

Both publish direct boxes on `/virtual_obstacles` at 30 Hz within 1,000 m.
Import the corresponding JSON from `Agents/` into Unity's agent configuration
folder and select it for a new session. The CARLA preset also uses the CARLA
top-LiDAR topic and mounting pose; the physical preset retains the original
Lincoln sensor settings. In box mode these sensors only forward scans unchanged.
Box publication does not depend on receiving scans: Autoware can consume the
original sensor clouds directly. Enable the Autoware perception profile for both
presets. For CARLA boxes use the regular CARLA launcher, not the LiDAR-test launcher.

| Mode | LiDAR output | Bounding boxes |
|---|---|---|
| LiDAR modification (default) | GPU SDF modification on each configured `<topic>_modified` | No box publisher |
| Direct bounding boxes | Original scans forwarded unchanged on `<topic>_modified` | `DetectedObjects` on `/virtual_obstacles` |

Direct boxes include active virtual objects whose collider centers are within the
configured radius, including occluded objects. The default radius is **1,000 m**
and the default publication rate is **30 Hz** (limited by Unity's frame rate and
available ROS clock ticks). Box mode does not allocate virtual-object SDF textures
or run SDF raymarching. It does not require a LiDAR sensor in the agent configuration.
Camera behavior is unchanged.

The publisher uses the assigned BoxCollider's center, orientation, and scaled
size. Set each VirtualObject's **Classification** in the Inspector; existing
vehicle assets default to Car. Missing, inactive, or invalid colliders are skipped
with a warning. Ego descendants are excluded. Autoware estimates velocities,
assigns track IDs, and predicts motion. Empty arrays indicate no current detections;
removed objects disappear after Autoware's normal tracker lifetime, not immediately.

## ROS time and coordinates

Choose **System time** for physical deployments and **Simulation /clock** for
CARLA or ROS bag playback. The checked-in CARLA agent uses `/clock`. With simulation
time selected, publication waits for positive `/clock` time, pauses when the clock
pauses, and resumes after forward or backward jumps. Unity does not publish `/clock`.
Autoware must use the same time domain (`use_sim_time:=true` for simulation).

Boxes are expressed in **base_link**, sampled after the Unity ego pose update.
Dynamic agents wait for their first `/ub_mr/localization` message. Autoware requires
valid `map` ↔ `base_link` TF at the sample time, and Unity's map placement/localization
must describe that same frame. A clock setting alone cannot correct a map offset.

## Enable the Autoware profile

The profile supports **Autoware Universe 0.43.0**, including the repository's
`ubcavas/autoware-lincoln:20251015.0` image. It adds the `ub_mr_virtual_objects`
channel to the installed multi-object tracker, alongside existing real detections.
An idle virtual channel is supported, so the profile works with either Unity mode.
Do not also relay `/virtual_obstacles` onto the real detector's topic.

From the **UB-DigitalTwin repository root**, enable it with either CARLA launcher:

```bash
UB_MR_PERCEPTION_PROFILE=1 ./launch/launch_autoware_carla.sh
# Or the passive CARLA/SUMO workflow:
UB_MR_PERCEPTION_PROFILE=1 ./scripts/launch_autoware_carla_passive.sh
```

The launchers restore any prior UB-MR profile before other CARLA patches, then apply
the selected profile immediately before launching Autoware. Enabling the profile
turns off the ego-only empty-object shortcut and CARLA ground-truth replacement
of real detections. Launching with `UB_MR_PERCEPTION_PROFILE=0` restores the profile's
original files. This setting does not switch Unity's selected recognition mode.

For a custom box topic, set the same topic in Unity and the launcher:

```bash
UB_MR_PERCEPTION_PROFILE=1 UB_MR_BOUNDING_BOX_TOPIC=/my/virtual_objects \
  ./launch/launch_autoware_carla.sh
```

For native Autoware or manual Docker launches, stop Autoware first and run the
helper from `Autoware/ub-lincoln-docker/docker/resources/`:

```bash
python3 configure_ub_mr_perception.py apply --autoware-root /autoware --dry-run
python3 configure_ub_mr_perception.py apply --autoware-root /autoware
# Explicit rollback (while Autoware is stopped):
python3 configure_ub_mr_perception.py restore --autoware-root /autoware
```

The Compose service mounts the helper at `/resources/configure_ub_mr_perception.py`;
new images also bundle it. From the Autoware docker directory:

```bash
docker compose exec autoware python3 /resources/configure_ub_mr_perception.py apply
```

A new image can opt in at build time using `--build-arg UB_MR_PERCEPTION_PROFILE=1`.
By default, building the image leaves the profile disabled. Unsupported package
versions or launch layouts are rejected before modification. Backups are stored in
`/autoware/.ub_mr_perception_profile.json`; restore before independently editing
those files. The helper refuses to overwrite unrelated edits during restoration.

Existing LiDAR routing can keep consuming `<topic>_modified` in either mode. If
Autoware currently consumes only the original scan topic, configuring this profile
does not change that routing: route perception to the modified output to test the
LiDAR-modification mode. Localization/sensor routing otherwise remains deployment-specific.

## Configuration file

Existing agent JSON needs no migration. Missing recognition settings mean LiDAR
modification with the defaults above. Example box-mode settings:

```json
"recognition": {
  "mode": "BoundingBoxInjection",
  "boundingBoxTopic": "/virtual_obstacles",
  "publishRateHz": 30.0,
  "detectionRadiusMeters": 1000.0,
  "useSimTime": true
}
```

The topic must be absolute, and rate/radius must be finite positive numbers.
There is one recognition mode per active agent/session; live switching and
sensor visibility simulation are not implemented.

## Verification

```bash
ros2 topic info /virtual_obstacles --verbose
ros2 topic echo /virtual_obstacles --once
ros2 topic hz /virtual_obstacles
ros2 topic echo /perception/object_recognition/tracking/objects --once
ros2 run tf2_ros tf2_echo map base_link
```

The virtual-box publisher uses reliable, volatile QoS with depth 1. An idle tracker
subscription may keep `/virtual_obstacles` listed in LiDAR mode; check **Publisher
count**, which should be zero. In box mode, verify stamps follow your ROS clock,
frame_id is `base_link`, and dimensions remain correct as objects rotate.

Automated checks (Unity 6000.0.36f1, project root passed to `-projectPath`):

- `CAVAS.UB_MR.Tests.RecognitionChecks.Run`: JSON compatibility, validation, timing,
  oriented/scaled/sheared boxes, messages, registry and SDF disabling.
- `CAVAS.UB_MR.Tests.RecognitionChecks.RunGpu`: also exercises the actual SDF
  compute shader with hit/miss rays; requires graphics.
- `CAVAS.UB_MR.Tests.RecognitionRosChecks.Run`: real DDS scan passthrough, exact
  metadata/data preservation, `/clock` and repeated cleanup. Run with
  `ROS_DOMAIN_ID=181`, source ROS Humble, and expose the Ros2ForUnity native libraries.
- `CAVAS.UB_MR.Tests.RecognitionMenuChecks.Run`: opens the actual agent editor and
  renders `/tmp/ub-mr-recognition-menu.png`; requires graphics (omit `-nographics`).

For example, from this UB-MR checkout:

```bash
source /opt/ros/humble/setup.bash
export LD_LIBRARY_PATH="$PWD/Assets/Ros2ForUnity/Plugins/Linux/x86_64:${LD_LIBRARY_PATH:-}"
ROS_DOMAIN_ID=181 /path/to/Editor/Unity -batchmode -nographics -projectPath "$PWD" \
  -executeMethod CAVAS.UB_MR.Tests.RecognitionRosChecks.Run -logFile /tmp/ub-mr-ros-checks.log
```

The Autoware submodule includes `tests/test_ub_mr_perception.py` (Python unittest)
and `tests/recognition_tracker_smoke.py`. Run the latter in a sourced Autoware
container with `ROS_DOMAIN_ID=182`. It checks idle virtual input, simultaneous real
and moving virtual tracks, estimated motion, object expiry and Unity disconnection.

Before using a rebuilt player in a scenario, verify the full deployment in RViz:

1. In LiDAR mode, virtual objects affect the modified cloud and no box publisher exists.
2. In box mode, forwarded scans match the source while virtual tracks appear alongside real tracks.
3. Confirm predicted objects reach planning and influence the planned trajectory.
4. Despawn traffic and restart the scene; no old detections or duplicate publishers remain.

The tracker smoke test does not start the complete planning stack or validate a
vehicle's map calibration. Those checks require the target deployment.
