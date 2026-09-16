# HAL safety repair and verification

Status: software repair verified with mock outputs and in-memory transports.
**Powered testing is not released.** ROS/DDS integration and physical stopping
remain unverified. No hardware ports were opened, tools powered, or robot driven.

## Repaired paths

- `hal/safety_gate.py` owns the final output decision. Startup is disarmed.
  Heartbeat, safety, odometry feedback, motion command and tool command each expire
  independently after 500 ms. A 20 ms steady-clock ROS timer checks expiration
  without requiring another command. Late incoming messages cannot mask an
  already-expired input. Deadlines use local monotonic time, not `/clock`.
- STOP/EMERGENCY, `/emergency_stop=true`, invalid commands, driver faults,
  measured motion loss during tool operation, and explicit disarm immediately
  attempt emergency stop, neutralize every tool, and zero all four wheel channels.
  Stop attempts continue if another stop operation raises. Stop errors appear in
  `/hal/diag`; `/hal/ready` means armed, not merely initialized.
- Trips latch the HAL disarmed. Releasing emergency stop or receiving NORMAL
  never restarts it. `/hal/arm` requires fresh heartbeat, safety, feedback, neutral
  motion/tool commands, measured standstill, a healthy capable driver, and a
  successful neutral-output attempt. Commands issued before a trip cannot arm it.
- Front trimmer, belly modes 0/1/2 and agitator are explicit driver outputs.
  Any active tool requires forward commanded and measured motion, no commanded
  turn, measured yaw rate within 0.05 rad/s, and speed above 0.01 m/s. Stop,
  reverse, turn, stale feedback or loss of measured forward motion trips all
  outputs. These provisional mock thresholds require physical calibration.
- The safety monitor starts STOP, expires missing IMU/contact inputs, detects
  static gravity tilt as well as roll/pitch angular rate, includes rear contact,
  and latches emergency conditions. `/safety/reset` requires fresh NORMAL inputs;
  it does not arm the HAL. IMU gravity must be +Z upright. Dynamic acceleration
  can bias the tilt estimate; this is not a validated overturn detector.
- Mock shutdown clears every output. UART/CAN transport failures now propagate
  and latch unhealthy state. Shutdown attempts the existing estop frame before
  closing transport even on failure. Active tools are explicitly unsupported on
  those backends. Neutral tool calls there cannot establish physical tool stop.
- Physical HAL selection fails before opening a port. There is no fallback to a
  mock on hardware failure. The legacy `hal_node` uses the same protected HAL.
- Repaired unresolved behavior merge markers and its empty safety callback.
  Behavior starts STOP, expires inputs and defaults tools off. Its legacy
  stationary cutting phase cannot activate tools under the new motion interlock.
  Core launch defaults behavior off; Gazebo launch no longer starts behavior or
  coverage planning. This does not redesign autonomous behavior.
- Package discovery includes `hal` and `drivers`; launch assets and missing ROS
  dependencies are installed. Encoder-derived velocity now uses deltas before
  updating the previous encoder reference. Invalid sensor packets are rejected.
- Gazebo raw IMU/odom are remapped to `/sim/raw/imu` and `/sim/raw/odom` to avoid
  reading back reconstructed robot-state outputs. The sensor bridge requires fresh
  IMU, progressing odom and both modeled contact streams; no cached/default healthy
  packets are republished indefinitely. Missing contact heartbeats inhibit packets.
  Side/rear contacts and currents are still unmodeled in that bridge.

## Evidence recorded on 2026-09-16

```powershell
.venv-geometry\Scripts\python.exe -m unittest discover -s tests -q
.venv-geometry\Scripts\python.exe -m compileall -q deweeder_tertill/src/weedbot_core
```

119 discovered tests: **118 passed, 1 skipped**. The skipped test is the actual
ROS/DDS integration test, because this environment lacks ROS and built message
bindings. Syntax compilation passed. This is not a live ROS/Gazebo acceptance run.

New executable tests cover final wheel/tool outputs under each independent
source timeout, all tool channels, emergency latch/rearm, measured standstill and
turning, invalid values/clocks, partial driver failure, shutdown, tilt and stale
sensors, source-cache expiry, packet interval handling, UART short writes, and
CAN/serial stop-before-close behavior. All transport objects in these tests are
in-memory fakes. No physical stopping latency is measured.

## Next acceptance step: ROS, mock only

In an isolated ROS workspace/domain with no robot or hardware connected:

1. Build the existing `weedbot_msgs` and `weedbot_core` packages with colcon and
   source the workspace. Use the project's compatible ROS distribution.
2. From this repository root run:
   `python3 -m unittest discover -s tests -p test_hal_ros_integration.py -v`.
   Require execution and a pass, not a skip. This checks real DDS callbacks,
   manual arming, emergency release without restart, and steady-timer command loss
   against final mock outputs. It has not been executed here.
3. Run `robot_core.launch.py` with its mock/behavior-off defaults. Verify missing
   sensors hold STOP; inject valid sensor packets and neutral commands at a rate
   faster than the deadline; inspect `/hal/diag` and `/hal/ready`.
4. Exercise stop conditions during active mock tools, separately stop each source,
   pause ROS simulation time, reset safety while unsafe, and terminate the command
   publisher. Record topic traces and final mock outputs. Check discovery/QoS,
   scheduling delays and shutdown behavior on the target runtime.
5. Check the Gazebo raw topic mapping and contact publication cadence. Its
   `/cmd_vel` bridge is a separate simulator drive path, not an actuator driven by
   this mock HAL. Do not use robot movement in that world as proof of HAL stopping.

## Remaining powered-test blockers

An executor/process hang or host power loss cannot be stopped by a Python timer.
The 500 ms deadline plus nominal 20 ms timer period is not a guaranteed wall-clock
stop bound. Independent MCU watchdog and hardwired emergency power interruption,
physical tool stop, restart inhibition, channel polarity/mapping, feedback freshness
and measured stopping/coast time must be implemented and tested separately before
lifting the hardware inhibit. Serial/CAN `is_healthy` describes software transport
state; it does not verify firmware watchdog operation or actuator state.

The current messages lack source sequence/session IDs and some source timestamps.
Local receive freshness cannot reject every delayed/replayed source message, and
`MicroSensorPacket.tick_time` cannot disambiguate dropped packets. The simulator
now provides the source odom interval between emitted snapshots, with recovery
snapshots invalidating velocity, but the physical protocol needs a defined timing
and acknowledgement contract. Encoder estimates also cannot prove ground motion
under wheel slip. The PID node remains diagnostic; HAL wheel output uses the
existing open-loop scaling with explicit mock FL/FR/RL/RR mapping.

After mock ROS acceptance, continue the unpowered guard/clearance bench described
in `unpowered_guard_bench.md`. Keep powered tests blocked until the independent
hardware/firmware evidence above is recorded; geometry exposure results alone
cannot authorize them.
