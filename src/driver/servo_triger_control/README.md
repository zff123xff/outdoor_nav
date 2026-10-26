# servo_triger_control

ROS 2 Humble Python package for a mechanical switch servo, using a Linux hardware
PWM channel. The ROS node is `servo_triger_node`; the PWM backend is the independent
`ServoPWM` class in `servo_triger_control/servo_pwm.py`.

## Build and run on the NX

From the outdoor_nav workspace:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select servo_triger_control
source install/local_setup.bash
ros2 launch servo_triger_control servo_switch.launch.py
```

If using a separate NX install base, build/source that install base consistently.
This package is standalone; no changes to the navigation launch are required.

Edit `config/servo.yaml` before building, or pass a separate configuration:

```bash
ros2 launch servo_triger_control servo_switch.launch.py params_file:=/absolute/path/servo.yaml
```

Wait for the `PWM ready` log before testing. In another terminal, source the same
ROS/workspace environment and use the same ROS domain and DDS settings:

```bash
ros2 service call /triger std_srvs/srv/Trigger "{}"
# Alias with the same state machine and busy protection:
ros2 service call /press_switch std_srvs/srv/Trigger "{}"
```

The original request returns `success=true` only after
`IDLE -> PRESS -> HOLD -> RELEASE -> IDLE`. Both services share one busy flag;
a concurrent call immediately returns `success=false`, `message="Servo is busy"`.
The executor continues running while the first service coroutine awaits completion.

`PRESS` sends press_angle and waits move_time; `HOLD` waits hold_time;
`RELEASE` sends release_angle and waits move_time. Defaults give an action duration
of approximately 0.9 seconds plus timer scheduling. Monotonic wall time is used,
independent of simulated ROS time. There is no time.sleep() in the node.

## Parameters

| Parameter | Default | Meaning |
| --- | --- | --- |
| pwmchip_path | /sys/class/pwm/pwmchip0 | Hardware PWM controller sysfs directory |
| pwm_channel | 0 | Channel number; pwm0 is checked/exported for channel 0 |
| period_ns | 20000000 | PWM period in nanoseconds (equivalent to 50 Hz) |
| release_angle / press_angle | 20.0 / 70.0 | Servo angles in degrees |
| min_angle / max_angle | 0.0 / 180.0 | Accepted angle limits |
| min_pulse_ns / max_pulse_ns | 1000000 / 2000000 | Endpoint pulse widths |
| hold_time | 0.3 | Seconds held after the press travel allowance |
| move_time | 0.3 | Seconds allowed for each press/release movement |
| timer_period | 0.01 | State-machine timer interval in seconds |
| export_timeout | 2.0 | Maximum seconds waiting for exported channel directory |

Parameters are startup-only; restart with the new config to change them. Angles
outside limits, nonpositive periods and invalid pulse limits are rejected.
`period_ns` is the only PWM rate setting; no separate frequency parameter is needed.
Pulse widths are rounded to the nearest integer nanosecond:

```text
pulse = min_pulse + (angle-min_angle)/(max_angle-min_angle)*(max_pulse-min_pulse)
```

Initialization checks/exports the channel, then disables PWM if enabled, clears an old duty
to zero (so an old duty cannot prevent shortening the period), sets the period,
sets release_angle and enables PWM. Export readiness is polled by the timer.
For a fresh export with period=0 and enable=0, it first sets a valid period while
remaining disabled: Linux can reject even disable/duty writes with a zero period.
At graceful exit, release_angle is written, pulses are kept active for move_time
outside the executor, then PWM is disabled. The channel remains exported.
Forced termination/power loss cannot run this cleanup.

## Hardware and permissions

Configure the NX hardware PWM controller and the appropriate header pinmux before
starting. The PWM signal pin must correspond to the selected chip/channel. This
package does not configure Jetson pinmux or GPIO modes. Connect a suitable servo
power supply and common ground; do not power the servo motor from the PWM signal.

Do not start ROS with sudo. The process needs write permission on:

```text
/sys/class/pwm/pwmchip0/export          # only if the channel is not exported
/sys/class/pwm/pwmchip0/pwm0/enable
/sys/class/pwm/pwmchip0/pwm0/period
/sys/class/pwm/pwmchip0/pwm0/duty_cycle
```

The enable, period and duty_cycle attributes also require read permission to
inspect the initial state and skip redundant disable operations.
Adjust these paths for your configuration. Have the administrator provision
permissions (for example through a dedicated group and device initialization
policy), including newly created attributes after export. The node never invokes
sudo or changes permissions; errors identify the exact file and required access.
Use this PWM channel exclusively; the node cannot arbitrate another PWM writer.

Missing files, export errors/timeouts, permission errors and write failures are
logged and reject service requests. On an action failure the original caller gets
`success=false`; the node attempts to disable PWM and latches the fault until restart.

Ordinary PWM servos provide no position feedback. Success means PWM commands and
timed movement/hold allowances completed, not verified switch actuation. Tune
angles and move_time for the actual mechanism.

## Automated verification (no real PWM hardware)

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
python3 -m pytest src/driver/servo_triger_control/test -q
```

Tests use temporary files and a fake PWM writer; the integration test runs real
ROS service requests to verify delayed completion, busy responses across aliases,
hold duration, repeat actions and duty-write fault handling.
