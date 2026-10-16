# Runtime resource HUD

Every simulation module creates a **Resources** panel in the upper-right corner. Click its header to collapse or expand it. Scroll within the panel to see additional GPUs or sensors. Collection continues while collapsed; returning to the main menu stops that module's GPU worker and removes its sensor histories.

The panel refreshes four times per second. Rates use elapsed monotonic time between one-second samples. Timing averages include samples completed in the last ten seconds. An inactive timing retains its last value, marked **inactive** after two seconds; an empty averaging window displays **—**. Missing measurements display **Unavailable** or **Waiting for samples**, rather than zero.

| Measurement | What is counted |
| --- | --- |
| GPU VRAM | Memory attributed to the Unity player PID on each GPU, plus occupied device memory minus that allocation as **Other/system**. Cyan is the application, amber is other/system, and gray is remaining capacity. **Used incl. reserved** adds NVIDIA's separately reported reserved memory to its used-memory counter, so driver reservations are not shown as free. In the Editor, the application is explicitly labeled **Unity Editor**. |
| LiDAR processing | Local elapsed time from selecting a scan through validation, staging, GPU dispatch, blocking readback, and message rewriting. Failed/invalid work contributes no sample. |
| LiDAR receive → publish | Time from subscription callback entry through the successful return of the local publish call, including queue delay. It does not measure downstream DDS delivery. Each queued scan carries its own receipt timestamp. Discarded scans contribute no timing sample. |
| Bypass | Explicit LiDAR passthrough or a scan with no active SDFs. These scans contribute receive-to-publish timing but never processing timing. |
| ROS sensor payload | Data-array bytes delivered to LiDAR/camera callbacks (including subsequently discarded messages), and data-array bytes submitted by successful publish calls. Includes camera RGB/depth, modified images, and sandbox image reception. |
| Redis traffic | Actual RESP bytes read/written through both subscriber and publisher connections, including framing, authentication, and heartbeats. Retries preserve the session's byte totals. |
| Redis server RTT | Elapsed time between PING and its matching PONG on the publisher connection. Probes run once per second even with continuous ego updates. Histories reset when either connection drops; replies from an older connection generation are ignored. |

ROS payload rates exclude message metadata, serialization, DDS discovery, retransmissions, transport headers, odometry, bounding boxes, clock messages, and service calls. Multiple subscriptions count each callback delivery. They are **not machine-wide or total ROS network bandwidth**. Redis latency includes server response time and does not require synchronized clocks. Scene-only UDP traffic has no Redis RTT measurement.

## GPU runtime requirements

GPU metrics initially support Linux with NVIDIA drivers and `nvidia-smi`. Queries run off the Unity thread, never overlap, and have a two-second timeout. Unavailable/stale GPU data does not interrupt simulation. MIG process attribution is currently unavailable; device-level values can still be displayed.

`run_ub_mr.sh` mounts host `/proc` read-only at `/host/proc`. The collector reads `/host/proc/self/status` for the host PID used by legacy drivers; NVIDIA R555 and newer report PIDs in the caller's namespace, so those queries use Unity's local PID. An unknown driver version with different PID namespaces reports process memory as unavailable instead of guessing. Existing containers must be recreated with the updated launcher. Custom Docker launches should include `-v /proc:/host/proc:ro` for legacy-driver support. Native Linux launches use `/proc/self/status`.

The HUD reports the Unity process, not a sum of every ROS helper process in the UB-MR container. A device on which the identified process has no allocation reports zero; failed attribution never becomes zero.

## Validation

- `ResourceTelemetryChecks.Run()` checks rolling windows, inactive/unavailable states, concurrent byte accounting, reconnect generations, GPU XML fixtures, and subprocess timeout/cancellation without a GPU.
- `ResourceNetworkChecks.Run()` uses an isolated RESP fixture on an ephemeral loopback port. It checks byte-exact UTF-8/framing accounting, fragmented responses, continued RTT sampling under publishing load, reconnect, and worker cleanup. It does not access the simulation Redis server.
- With the project closed in other Editor instances, run Unity with `-batchmode -nographics -projectPath <UB-MR> -executeMethod CAVAS.UB_MR.Tests.ResourceHudChecks.Run`. This runs the managed and network checks, actual LiDAR queue/publication checks, and HUD lifecycle checks. Include `Assets/Ros2ForUnity/Plugins/Linux/x86_64` and the ROS installation's `lib` directory in `LD_LIBRARY_PATH` for the generated message bindings.
- To render the HUD fixtures at both resolutions, use `-batchmode -executeMethod CAVAS.UB_MR.Tests.ResourceHudChecks.RunVisual` with a graphics device. Screenshots are written to `/tmp/ubmr-resource-hud-1280x720.png` and `/tmp/ubmr-resource-hud-1920x1080.png`.
- Run the existing `RecognitionChecks.RunGpu`, `RecognitionRosChecks.Run`, and isolated `ServerConnectionChecks.Run` for full runtime integration.
- On an NVIDIA runtime, compare each GPU's player/Editor PID allocation and used/total memory against `nvidia-smi -q -x`, both native and in Docker. Confirm increasing another application's GPU allocation affects Other/system.
- Visually check the panel at 1280×720 and 1920×1080: collapse/reopen, multiple sensors/GPUs, scrollbar, existing map controls, scene transitions, missing Redis, and missing GPU driver.
- For performance comparison, profile the same scan replay with telemetry enabled and with the Resource HUD disabled and instrumentation calls omitted in a local profiling build. Compare median/tail frame and scan-processing times after warm-up. Do not infer live GPU overhead from the deterministic checks.

References: [NVIDIA process and memory reporting](https://docs.nvidia.com/deploy/nvidia-smi/), [NVIDIA R555 namespace-reporting discussion](https://forums.developer.nvidia.com/t/nvmldevicegetcomputerunningprocesses-v3-inside-container-reporting-invalid-memory-usage-number/294682), [Linux PID namespace and procfs behavior](https://www.man7.org/linux/man-pages/man7/pid_namespaces.7.html).
