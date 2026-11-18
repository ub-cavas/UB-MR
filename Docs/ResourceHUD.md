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

## Experiment logs and graphs

The recorder's CSV schema and plotting package are also used by CARLA keyboard,
wheel/joystick, drive-sim and Mustang clients. See the parent repository's
`docs/experiment-logging.md` for generic Python/C# adapters, configuration and
measurement scopes. `Scripts/plot_experiment_metrics.py` remains a compatible
entry point to the shared `ub_experiment` plotter.

New player builds record metrics automatically for the entire player session, including map changes. **Rebuild the Unity player with this code**; downloaded older players do not contain the recorder. Rebuild the Docker image once to install the graphing dependency:

```bash
cd UB-MR
docker build -f Docker/Dockerfile -t ub-mr .
./run_ub_mr.sh <new-build-folder>
# Inside the container:
./ub-mr.sh
```

Close the player normally and allow the launcher to finish generating graphs. The launcher forwards Ctrl+C to Unity and waits for its recording to flush before plotting. Each launch saves a separate folder on the **host**, under `UB-MR/Docker/Logs/experiments/run-<yyyyMMdd-HHmmss>Z-<hostname>-<unique suffix>/`, for example `run-20261118-143611Z-awsim-Dell-G16-7630-oaxJzc/`. The timestamp is the run's start time in UTC; `Z` marks UTC. The Docker launcher passes the host machine's name into the container through `UB_MR_HOST_NAME`, so the folder identifies the deployment host. Editor/native sessions use the same naming format and their machine's name. Hostnames are sanitized for folder names and limited to 63 characters. The unique suffix prevents runs started in the same second from colliding. Existing folders retain their names. The bind mount maps container `/app/Logs` to host `UB-MR/Docker/Logs`, so the files survive container removal. The launcher passes the host UID/GID and assigns the run folder and completed files to that user for review. It contains:

- `metrics.csv`: every successful LiDAR receive-to-publish measurement (including bypass), every accepted Redis RTT probe, connection-state changes, and approximately one-second RX/TX throughput samples with cumulative byte counters.
- `receive_to_publish.png`, `network_rtt.png`, `network_throughput.png`: time-series graphs. Timing graphs show each second's mean and min/max range; the CSV retains individual measurements. Sensor instances and Redis connection generations have separate timing series.
- `summary.json`: timing counts/means/min/max, last sampled traffic totals per source, and recording status.
- `recording-status.txt`: clean-shutdown status, dropped-row count, and any writer error. Check this when reviewing an experiment. Absence indicates interrupted shutdown.

The same measurement definitions in the table above apply to the files. Throughput is measured at the application scopes shown in the graphs; ROS payload and Redis RESP are separate series with different units. `rx_Bps`/`tx_Bps` in the CSV are **bytes per second**. Blank rates with `available=0` mean unavailable; zero with `available=1` means an idle registered source. No RTT samples are invented during a disconnect. Cumulative traffic counters start with each source instance, survive retries within that instance, and are recorded at rate-sampling boundaries.

Durations and rates use a monotonic clock. CSV `elapsed_s` is relative to recording start; `utc` is derived from the start's UTC timestamp, so wall-clock adjustments do not distort timing. Background callbacks enqueue into a bounded queue; a worker flushes files roughly every second. If disk writing falls behind, the recorder counts dropped rows rather than blocking simulation callbacks. A forced kill can lose queued/unflushed rows and skip automatic graph creation. Preserve the CSV and regenerate graphs afterward:

```bash
# Ubuntu host dependency (already installed in the updated Docker image):
sudo apt-get install python3-matplotlib
python3 UB-MR/Scripts/plot_experiment_metrics.py UB-MR/Docker/Logs/experiments/<run-folder>
# Passing the experiments parent regenerates every run below it.
```

If an older Docker image reports `No module named 'matplotlib'`, rebuild it with the command above. Updating the mounted scripts does not install packages inside an existing image. Exit the old container shell and relaunch `run_ub_mr.sh` after rebuilding; an already-running container continues using its original image. The launcher checks `/usr/bin/python3` for Matplotlib before starting an experiment, so this dependency error is reported before recording begins. Graph generation uses that system interpreter and the headless Agg backend inside the container; host Python packages are not required for the deployed workflow.

For Docker, set `UB_MR_LOG_DIR=/absolute/host/path` before `run_ub_mr.sh` to choose another host output directory. The run folders will be in that directory's `experiments/` subfolder. Native player/Editor Play sessions write to `Application.persistentDataPath/Experiments/` by default (Linux: `~/.config/unity3d/UB-CAVAS/UB-MR/Experiments/`); set `UB_MR_METRICS_DIR` before launching Unity to override it, and run the plotting script afterward. The exact directory is printed in the Unity log. Set `UB_MR_METRICS_ENABLED=0` to disable recording. Keep the default enabled for experiments.

**Editor logging is optional and defaults to off.** Before pressing Play, toggle **UB-MR → Telemetry → Record Editor experiments**. A checkmark means logging is enabled. The choice persists across Editor restarts for your user and this project. Stop Play before changing it; the toggle is disabled during Play and Play transitions. Enabling it records the next Play session, and stopping Play flushes the CSV. Generate Editor graphs with the plotting command above. `UB_MR_METRICS_ENABLED=0` still disables recording even when the menu is checked. The Resources HUD continues to display live metrics with Editor logging off; the Editor preference does not change player/Docker recording.

## GPU runtime requirements

GPU metrics initially support Linux with NVIDIA drivers and `nvidia-smi`. Queries run off the Unity thread, never overlap, and have a two-second timeout. Unavailable/stale GPU data does not interrupt simulation. MIG process attribution is currently unavailable; device-level values can still be displayed.

`run_ub_mr.sh` mounts host `/proc` read-only at `/host/proc`. The collector reads `/host/proc/self/status` for the host PID used by legacy drivers; NVIDIA R555 and newer report PIDs in the caller's namespace, so those queries use Unity's local PID. An unknown driver version with different PID namespaces reports process memory as unavailable instead of guessing. Existing containers must be recreated with the updated launcher. Custom Docker launches should include `-v /proc:/host/proc:ro` for legacy-driver support. Native Linux launches use `/proc/self/status`.

The HUD reports the Unity process, not a sum of every ROS helper process in the UB-MR container. A device on which the identified process has no allocation reports zero; failed attribution never becomes zero.

## Validation

- `ResourceTelemetryChecks.Run()` checks rolling windows, inactive/unavailable states, concurrent byte accounting, reconnect generations, GPU XML fixtures, subprocess timeout/cancellation, and experiment CSV recording (concurrent callbacks, CSV escaping, unavailable/idle rates, stale RTT rejection, shutdown flush, and overwrite protection) without a GPU.
- `ResourceNetworkChecks.Run()` uses an isolated RESP fixture on an ephemeral loopback port. It checks byte-exact UTF-8/framing accounting, fragmented responses, continued RTT sampling under publishing load, reconnect, and worker cleanup. It does not access the simulation Redis server.
- With the project closed in other Editor instances, run Unity with `-batchmode -nographics -projectPath <UB-MR> -executeMethod CAVAS.UB_MR.Tests.ResourceHudChecks.Run`. This runs the managed and network checks, actual LiDAR queue/publication checks, and HUD lifecycle checks. Include `Assets/Ros2ForUnity/Plugins/Linux/x86_64` and the ROS installation's `lib` directory in `LD_LIBRARY_PATH` for the generated message bindings.
- To render the HUD fixtures at both resolutions, use `-batchmode -executeMethod CAVAS.UB_MR.Tests.ResourceHudChecks.RunVisual` with a graphics device. Screenshots are written to `/tmp/ubmr-resource-hud-1280x720.png` and `/tmp/ubmr-resource-hud-1920x1080.png`.
- Run the existing `RecognitionChecks.RunGpu`, `RecognitionRosChecks.Run`, and isolated `ServerConnectionChecks.Run` for full runtime integration.
- On an NVIDIA runtime, compare each GPU's player/Editor PID allocation and used/total memory against `nvidia-smi -q -x`, both native and in Docker. Confirm increasing another application's GPU allocation affects Other/system.
- Visually check the panel at 1280×720 and 1920×1080: collapse/reopen, multiple sensors/GPUs, scrollbar, existing map controls, scene transitions, missing Redis, and missing GPU driver.
- For performance comparison, profile the same scan replay with telemetry enabled and with the Resource HUD disabled and instrumentation calls omitted in a local profiling build. Compare median/tail frame and scan-processing times after warm-up. Do not infer live GPU overhead from the deterministic checks.

References: [NVIDIA process and memory reporting](https://docs.nvidia.com/deploy/nvidia-smi/), [NVIDIA R555 namespace-reporting discussion](https://forums.developer.nvidia.com/t/nvmldevicegetcomputerunningprocesses-v3-inside-container-reporting-invalid-memory-usage-number/294682), [Linux PID namespace and procfs behavior](https://www.man7.org/linux/man-pages/man7/pid_namespaces.7.html).
