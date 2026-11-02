# mavros_odom_relay

A small QoS-bridging relay that re-publishes `/mavros/local_position/odom`
(BEST_EFFORT, from mavros) as `/odom` (RELIABLE) so that Nav2
`controller_server`'s default RELIABLE subscription accepts the data.

## Why this is needed

`ros2 topic info /mavros/local_position/odom` shows:

| Endpoint | QoS |
|---|---|
| Publisher (`mavros/local_position`) | **BEST_EFFORT** |
| Subscriber (`nav2_input_bridge`) | BEST_EFFORT ✅ compatible |
| Subscriber (`controller_server`) | **RELIABLE** ❌ incompatible |

DDS delivers nothing between incompatible endpoints. Result: Nav2 sees
no odom, the controller never runs, `/cmd_vel` stays at zero, the dog
does not move.

This node bridges the two QoS regimes by re-publishing every incoming
odom with RELIABLE.

## Data flow

```
mavros ──(BEST_EFFORT)──> /mavros/local_position/odom ──┬──> nav2_input_bridge (BEST_EFFORT, unchanged)
                                                         │
                                                         └──> mavros_odom_relay ──(RELIABLE)──> /odom ──> controller_server
```

`nav2_input_bridge` continues to subscribe to the original topic for TF
broadcast. Only `controller_server` needs to be repointed at `/odom`.

## Build

```bash
cd ~/ww/outdoor_nav
colcon build --packages-select mavros_odom_relay
source install/setup.bash
```

## Run

```bash
ros2 launch mavros_odom_relay mavros_odom_relay.launch.py
```

Defaults:
- input: `/mavros/local_position/odom` (BEST_EFFORT)
- output: `/odom` (RELIABLE)

## Repoint Nav2

In `src/robot_launch/config/go2w_nav2.yaml`:

```yaml
controller_server:
  ros__parameters:
    odom_topic: /odom   # was: /mavros/local_position/odom
```

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `input_topic` | `/mavros/local_position/odom` | Source `Odometry` topic |
| `output_topic` | `/odom` | Destination `Odometry` topic |
| `frame_id_rewrite` | `""` (no rewrite) | If non-empty, overwrite incoming `header.frame_id` |

## License

Apache-2.0
