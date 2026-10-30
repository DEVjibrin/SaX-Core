# sax-agent

Edge daemon for autonomous mobile robots. It voxelizes 3D LiDAR into a 20 cm occupancy grid, encodes it as SX1
vector strings and streams them to [sax-bridge](../sax-bridge) over a WebSocket, instead of 20-50 Mbps of raw
point clouds. It also receives E-stop and recovery-trajectory commands from the bridge.

In the simulated warehouse used by the CLI a frame has about 150 voxels and 1.3 kB, which is roughly 100 kbps
at 10 Hz, or about 32 kbps once WebSocket compression is applied. A raw stream of 30 000 points per frame at
16 bytes per point would be 38 Mbps. The agent never exceeds `max_bandwidth_kbps` (900 by default).

```
include/sax_agent/  sx1_encoder (voxelizer + SX1), command_parser, agent_core, websocket_client
src/                the same, plus agent_node.cpp (the ROS 2 node)
scripts/            run_agent_cli.py  ROS-free simulator for testing the bridge and dashboard
test/               unit tests (no framework needed) and an in-process fake of ixwebsocket
config/, launch/    parameters and a launch file with respawn
```

## Build (ROS 2 Humble or Jazzy)

```bash
sudo apt install libixwebsocket-dev        # or build https://github.com/machinezone/IXWebSocket with -DUSE_TLS=1 -DUSE_ZLIB=1
cd ~/ros2_ws/src && git clone <this repo> sax_agent
cd ~/ros2_ws && colcon build --packages-select sax_agent && source install/setup.bash
ros2 launch sax_agent sax_agent.launch.py robot_id:=robot_01 bridge_url:=ws://bridge.example.com:8000 bridge_token:=SECRET
```

Without ROS, build only the core library and the tests (this needs just a C++20 compiler):

```bash
cmake -S . -B build -DSAX_BUILD_NODE=OFF && cmake --build build && ctest --test-dir build --output-on-failure
python3 test/test_cli.py
```

## Interfaces

| Direction | Name | Type | Purpose |
| --- | --- | --- | --- |
| subscribes | `/point_cloud` | `sensor_msgs/PointCloud2` | float32 x, y, z in any frame; moved into the odometry frame with tf2 |
| subscribes | `/odom` | `nav_msgs/Odometry` | pose in the world frame; this frame is the SX1 coordinate frame |
| subscribes | `/sax/incident` | `std_msgs/UInt8` | the navigation stack reports 1 path blocked, 2 wheel slip, 3 localization lost; 0 clears |
| publishes | `/sax/estop` | `std_msgs/Bool` | latched; true while an E-stop is engaged |
| publishes | `/sax/recovery_path` | `nav_msgs/Path` | operator trajectory, in the odometry frame |
| publishes | `estop_twist_topic` | `geometry_msgs/Twist` | optional zero velocity at 20 Hz while stopped (off by default) |
| service | `~/release_estop` | `std_srvs/Trigger` | releases a latched E-stop locally |

All parameters are in `config/sax_agent.yaml`.

## Safety behaviour

- **E-stop runs on the WebSocket thread, immediately.** The command latches the stop, publishes `/sax/estop`,
  persists the latch, and sends an `H` frame right away, without waiting for the next 10 Hz tick.
- **It is fail-safe.** Any line starting with `SXE` stops the robot unless it names a different robot, even if the
  operator field is missing or malformed. A bad operator is recorded as `unknown`.
- **It latches, and survives restarts.** A latch file (`/var/tmp/sax_agent_<robot_id>.estop`) exists while stopped, so
  a respawn or crash does not silently clear an operator's stop. Release it with `ros2 service call
  /sax_agent/release_estop std_srvs/srv/Trigger` or an `SXR|<robot>|<operator>` command.
- **Trajectories are rejected while stopped**, and malformed commands are counted and logged, never executed.
- Wire the stop into your velocity mux. With `twist_mux`:

```yaml
twist_mux:
  ros__parameters:
    locks:
      sax_estop: {topic: /sax/estop, timeout: 0.0, priority: 255}
```

## Streaming behaviour

- State is `H` while stopped, `S<reason>` while `/sax/incident` is non-zero, `S3` when odometry is older than
  `odom_timeout_s`, otherwise `A`. Nothing is sent before the first odometry message.
- A point cloud older than `cloud_timeout_s` is reported as an empty voxel list, which the bridge flags as a LiDAR
  blockage after three frames.
- Voxels are ordered nearest-first and cut at `max_voxels` or at the byte budget, so the closest obstacles always get through.
- Telemetry is never queued. When the link is down or its send buffer is full the frame is dropped. The client
  reconnects with exponential backoff and jitter (500 ms to 30 s), and restarts a connection whose send buffer stays
  full for 8 s. A connection that opens and then dies within 5 s still counts as a failure, so a server that accepts
  and immediately closes is not hammered.

## Wire format

```
SX1|<robot_id>|<seq>|<state>|<x_cm>,<y_cm>,<z_cm>,<qx>,<qy>,<qz>,<qw>|<i>.<j>.<k>;<i>.<j>.<k>;...
```

Voxel `(i, j, k)` covers `[i*0.2, (i+1)*0.2)` metres on each axis. Inbound: `SXE|<robot>|<operator>` (E-stop),
`SXP|<robot>|<x_cm>,<y_cm>;...` (trajectory), and the optional `SXR|<robot>|<operator>` (release). The bridge
does not send `SXR` yet.

## CLI fallback

```bash
python3 scripts/run_agent_cli.py --robot-id robot_01 --bridge-url ws://localhost:8000 --frequency 10
python3 scripts/run_agent_cli.py --simulate-blockage 15     # blind the LiDAR at 15 s for 6 s, then every 30 s
python3 scripts/run_agent_cli.py --stuck-after 20           # raise an incident until a recovery path arrives
```

It needs only `pip install websockets`. It uses the same voxelizer, encoder and command rules as the daemon; the two
implementations were checked byte for byte on thousands of random frames.
