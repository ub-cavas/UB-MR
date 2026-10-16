# Connecting UB-MR to a traffic server

In the Unity Editor or player, open **Main Menu → Server connection**.

1. Enter the server's IP address or hostname. Use `127.0.0.1` when the server runs on
   this machine, or its Tailscale IPv4 address when it runs on another machine.
2. Set **Port** to `6390`, **Channel** to `carla:telemetry`, and enter the server's
   Redis password. The repository's development default is `password`.
3. Select **Connect**. The status shows whether the connection is established and
   how many traffic vehicles are arriving.
4. Select **Back**, then start a session with an agent and map as usual.

The connection survives Main Menu → simulation scene transitions. Both traffic
reception and ego publication use this connection. No local Python/Docker UDP
bridge is needed. The address, port and channel are remembered on this device;
passwords are kept only in memory for the current run. Use **Disconnect** to stop
both directions. An interrupted connection retries automatically, and traffic
replicas are cleared after two seconds without fresh telemetry.

The scene's `EgoPublisher` still obtains the dynamic agent's pose and uses the
shared `CarlaMapFrame` transform. The server receives type-3 ego messages and
sends type-2 traffic snapshots on the same Redis channel. A connection status
confirms Redis connectivity; it does not by itself prove the server's traffic
publisher or CARLA ego renderer is running.

## Server setup

In the parent UB-DigitalTwin repository, run:

```bash
UB_REDIS_HOST=127.0.0.1 bash scripts/launch_carla_redis_server.sh
```

For a remote client, bind Redis to the server's Tailscale address instead. The
server launcher starts CARLA, Redis, the map loader, the traffic publisher, and
`ego-renderer`. The renderer creates one CARLA replica per ego ID and removes it
after two seconds without updates. Ego replicas are excluded from outgoing
traffic snapshots.

When migrating from the earlier local UDP bridge setup, stop the bridge that was
mirroring ego into CARLA before starting `ego-renderer`, to avoid duplicate ego
replicas. Redis authentication and any tailnet/firewall rules must permit the
client's connection to TCP port 6390.

## Compatibility

Launching a simulation scene directly without visiting Main Menu continues to
use its existing UDP receiver and ego publisher settings. Once Main Menu has
created the server connection, simulations use direct Redis even while
disconnected; they do not silently switch back to the scene's UDP destination.
Existing launch scripts that explicitly start a local UDP bridge should be used
with scene-only clients, or have that bridge disabled when using the menu.

## Validation

The integration checks use an isolated Redis instance, never the simulation
server. From a shell:

```bash
docker run -d --rm --name ubmr-connection-test-redis \
  -p 127.0.0.1:6392:6379 redis:7-alpine \
  redis-server --requirepass ubmr-test --save '' --appendonly no
```

In the Editor, select **UB-MR → Tests → Server connection checks**. The checks
cover validation, password exclusion from saved settings, Unicode messages,
traffic filtering, ego publication, idle heartbeats, forced reconnection,
authentication failure, unreachable endpoints, and worker shutdown. They close
all Pub/Sub clients on the test Redis instance to simulate connection loss.

```bash
docker stop ubmr-connection-test-redis
```
