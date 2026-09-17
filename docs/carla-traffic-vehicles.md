# CARLA traffic vehicles

UB-MR resolves the `blueprint` in each traffic snapshot through
`Assets/UB_MR/Prefabs/Traffic/TrafficVehicleCatalog.asset`. The initial entries are
`vehicle.audi.a2` (Audi A2) and `vehicle.ford.mustang` (Mustang 2012). All other
vehicles use the scene's existing Jeep fallback and log one warning per blueprint.
The Renegade is not registered as a Wrangler. Imported source meshes alone do not
register a vehicle.

The existing Redis/UDP payload remains unchanged. Actor IDs identify instances;
blueprint IDs identify models. A changed blueprint replaces that actor's instance,
a changed color updates its paint, and absence from a full snapshot despawns it.
Disabling the renderer clears its instances and resources. Re-enabling it restores
the receiver's current `KnownVehicles`. The original `vehiclePrefab` Inspector
field is the fallback; `vehicleCatalog` is the new catalog reference.

## Register another imported model

1. Put source meshes and textures under `Assets/UB_MR_Assets/Actors/Vehicles/`.
   Keep the CARLA actor pivot and meter scale. Create a traffic prefab under
   `Assets/UB_MR/Prefabs/Traffic/`, with an active identity root, +Z forward and +Y up.
   Keep corrections on a child named `Alignment`; put the visible model and its
   proxy beneath that same transform. Do not independently ground the proxy.
2. Add `TrafficVehicleAppearance` to the root. Bind only body-paint renderer slots
   and their shader color property (blank defaults to `_BaseColor`). Validate the
   shader before binding; glass, tires, trim, lamps and markings must not be tinted.
   `UB-MR/Traffic Paint` supports a linear grayscale `_PaintMask`: white substitutes
   the requested RGB, black preserves the original texture. Use a mask where a
   body texture includes markings. The supplied Mustang mask separates its orange
   paint from neutral stripes; it is specific to that source texture, not a general
   mask generator for other cars. Audi body material slot 4 is entirely paint.
3. Add one root `VirtualObject`, select the vehicle classification, and assign an
   enabled trigger `BoxCollider` sized to the model. Keep rigidbodies absent or
   kinematic. A traffic prefab must not independently simulate vehicle movement.
4. Keep a closed SDF proxy active with its renderer disabled. Assign its `MeshToSDF`
   and `SDFTexture` references. Enable Read/Write on the proxy model importer: the
   current baker recreates GPU buffers, which fails on unreadable meshes in player
   builds even when it works in the Editor. Set the volume center and dimensions to enclose the
   aligned proxy with at least one voxel of padding on all sides. Match size and
   resolution in `VirtualObject` and `SDFTexture`. Resolution is X voxel count;
   Y/Z are truncated from the aspect ratio. The initial models retain X=8. The
   validator checks effective quantized dimensions, not just the configured box.
   Runtime SDF textures remain owned and released by `VirtualObject`; bounding-box
   recognition mode disables the baker.
5. Add the exact server blueprint ID and prefab reference to the catalog. IDs are
   case-sensitive. Do not substitute similarly named vehicle models. Run
   **UB-MR > Traffic > Validate catalog**, then compare with the server inventory.
6. Compare the model at 0/90/180/270 degrees in CARLA and UB-MR: actor-origin
   alignment, ground contact, dimensions, paint and markings. Test both perception
   modes and a standalone build. Include the prefab, catalog update and all new
   `.meta` files in the change.

The initial alignment retains the existing Audi +90-degree child yaw and Mustang
+0.15 m child height. Both visual and proxy now share these adjustments. These
should be checked against the target live CARLA build before claiming exact pivot
parity. Paint follows RGB values; URP lighting is not intended to reproduce Unreal
shaders exactly. Wheel animation, lamps, pitch/roll and interpolation are not added.

## Inventory and validation

From the UB-DigitalTwin repository root, with the matching CARLA Python API:

```bash
python3 CARLA/UB-API/redis-networking/export_vehicle_inventory.py \
  --host 127.0.0.1 --port 2000 --output /tmp/carla-vehicles.json
```

This reads server version and `vehicle.*` blueprint IDs without spawning actors,
ticking, or changing world settings. The JSON fields are `server_version` and
`blueprints` (a sorted string array). Use **UB-MR > Traffic > Compare server
inventory** to select the export. Missing fleet mappings are coverage warnings;
invalid registered entries are errors. An invalid catalog fails closed to fallback
rendering; absent fallback actors are skipped with a diagnostic.

For batch validation, use your Unity executable with the project closed (or an
isolated project copy):

```bash
Unity -batchmode -nographics -projectPath /absolute/path/to/UB-MR \
  -executeMethod UB_MR.Redis_Networking.Editor.TrafficVehicleCatalogValidator.RunBatch \
  -trafficInventory /tmp/carla-vehicles.json -logFile /tmp/traffic-validation.log
```

Omit `-trafficInventory` for prefab-only validation. The runner exits 0 on success
and 1 on validation failure. Runtime code does not load the inventory file.

**UB-MR > Traffic > Rebuild initial Audi and Mustang prefabs** regenerates those two
prefabs and their paint assets in place, preserving asset GUIDs and other catalog
entries. It overwrites edits to the two initial prefabs; normal subsequent vehicle
registration does not require this command. `TrafficVehicleAssetSetup.BuildBatch`
also wires the catalog into the UB-Service-Center-Loop scene.

## Verification

The batch entry points below follow the project's existing recognition checks.
They exit the Unity process; use a separate project copy if the Editor is open.
The ROS message checks require the existing Ros2ForUnity installation and native
library environment. For a ROS Humble overlay installation:

```bash
source /opt/ros/humble/setup.bash
export LD_LIBRARY_PATH="/absolute/path/to/UB-MR/Assets/Ros2ForUnity/Plugins/Linux/x86_64:${LD_LIBRARY_PATH:-}"
```

Pass these methods to Unity's `-executeMethod`:

| Method | Checks |
| --- | --- |
| `CAVAS.UB_MR.Tests.TrafficVehicleChecks.Run` | Catalog validation, case-sensitive lookup, RGB parsing, material sharing and cleanup |
| `CAVAS.UB_MR.Tests.TrafficVehicleChecks.RunPlayMode` | Real loopback UDP snapshots, selection/fallbacks, replacement, color isolation, map yaw, re-enable, bounding-box perception, three 30-actor cleanup cycles |
| `CAVAS.UB_MR.Tests.TrafficVehicleChecks.RunGpu` | Same replay with LiDAR SDF generation, texture cleanup, per-model GPU hit/miss rays and a preview at `/tmp/ubmr-traffic-preview.png` |
| `CAVAS.UB_MR.Tests.RecognitionChecks.Run` | Existing recognition regression checks |
| `CAVAS.UB_MR.Tests.RecognitionChecks.RunGpu` | Existing GPU raymarch regression checks |
| `CAVAS.UB_MR.Tests.TrafficVehicleChecks.BuildPlayer` | Builds the same GPU replay as `/tmp/ubmr-traffic-player/TrafficValidation` |

Use `-batchmode -nographics` for CPU checks. GPU checks require graphics; on Linux,
`-batchmode -force-glcore` can be used with a working display/driver. The player
runs its checks automatically and exits with their result. For the Linux player:

```bash
LD_LIBRARY_PATH="/tmp/ubmr-traffic-player/TrafficValidation_Data/Plugins:${LD_LIBRARY_PATH:-}" \
  /tmp/ubmr-traffic-player/TrafficValidation -batchmode -force-glcore \
  -logFile /tmp/traffic-player.log
```

BuildPlayer replaces its own `/tmp/ubmr-traffic-player` output directory and creates a
temporary validation scene in the project: run it in the isolated copy, not as part
of the normal production scene list. Expected fallback diagnostics are deliberately
exercised by the replay and are not test failures. Read the final PASS/exception
and process exit code.

The GPU probe uses slightly oblique rays: the existing raymarch shader's AABB
intersection uses reciprocal directions, whose exact-zero case differs between
graphics backends. That shader is unchanged by this feature.

The inventory exporter's read-only contract can be tested without CARLA:

```bash
python3 -m unittest discover -s CARLA/UB-API/redis-networking/tests -v
```
