#!/usr/bin/env python3
import json
import random
import string
import numpy as np
import pymap3d as pm
import rclpy
from execute_task_server.srv import ExecuteTask
from msg_set_msgs.msg import AreaSample, MultiTask, SingleTask
from multi_cover import CircleRegion, CoverParams, PolygonRegion, plan_coverage
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
#0任务集结，1协同搜索，2平面任务，4返航, 6中继 ，7 线性任务, 8,抗干扰任务 ,9立体 10环绕 11，巡航, 12,起飞 
TASK_TYPE = {
    "LINE_TASK": 7,
    "PATROL_TASK": 11,
    "COVERAGE_TASK": 2,
    "RETURN_TASK": 4,
    "TAKEOFF_TASK": 12,
}

AREA_SAMPLE_TYPE = {
    "POINT": 0,
    "CIRCLE": 1,
    "POLYGON": 2,
    "RING": 3,
    "TARGET_POSE": 4,
}

TEST_FLAG = True

# 随机字符串
def generate_random_string(length: int = 10) -> str:
    return ''.join(random.choices(string.ascii_letters + string.digits, k=length))

class MultiTaskBridge(Node):
    """Subscribe to MultiTask and forward parsed missions to ExecuteTask."""

    def __init__(self):
        super().__init__("multi_task_bridge")

        self.declare_parameter("drone_id", 1)
        self.declare_parameter("flight_alt_m", 5.0)
        self.declare_parameter("swath_width_m", 10.0)
        self.declare_parameter("drone_name", "uav1")

        self.drone_id = self.get_parameter("drone_id").value
        self.flight_alt_m = self.get_parameter("flight_alt_m").value
        self.swath_width_m = self.get_parameter("swath_width_m").value
        self.drone_name = self.get_parameter("drone_name").value

        execute_task_service = f"/agent/{self.drone_name}/execute_task"
        multitask_topic = f"/{self.drone_name}/swarms_backend/global/multitask"

        

        self._client = self.create_client(ExecuteTask, execute_task_service)
        while not self._client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f"waiting for service {execute_task_service} ...")
        self.get_logger().info(f"connected to {execute_task_service}")

        self._subscription = self.create_subscription(
            MultiTask,
            multitask_topic,
            self._multi_task_callback,
            10,
        )

        self._gps_sub = self.create_subscription(
            NavSatFix,
            # "/gps/data",
            "/mavros/global_position/global",
            self._gps_callback,
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            ),
        )

        self._state_pub = self.create_publisher(
            State,
            self.drone_name + "/mavros/state",
            10,
        )

        self._mavros_global_position_pub = self.create_publisher(
            NavSatFix,
            self.drone_name + "/mavros/global_position/global",
            10,
        )

        # 创建一个1HZ定时器
        self.timer = self.create_timer(1.0, self._timer_callback)

        self.get_logger().info(
            f"listening on {multitask_topic} for drone_id={self.drone_id}"
        )
    def _timer_callback(self) -> None:
        # 模拟一个State
        state = State()
        state.header.stamp = self.get_clock().now().to_msg()
        state.connected = True
        state.armed = True
        state.guided = True
        state.manual_input = True
        state.mode = "OFFBOARD"
        state.system_status = 0
        self._state_pub.publish(state)





    def _gps_callback(self, msg: NavSatFix) -> None:
        # self.get_logger().info(f"received GPS: {msg}")
        self._mavros_global_position_pub.publish(msg)

    def _multi_task_callback(self, msg: MultiTask) -> None:
        try:
            self.get_logger().info("received MultiTask, parsing and forwarding")
            request_json = self._parse_multi_task(msg)
            if request_json is None:
                return
            self._send_service_request(request_json)
        except Exception as exc:
            self.get_logger().error(f"failed to handle MultiTask: {exc}")

    def _send_service_request(self, request_json: dict) -> None:
        request = ExecuteTask.Request()
        request.request_json = json.dumps(request_json)
        self.get_logger().info(f"sending ExecuteTask: {request.request_json}")
        future = self._client.call_async(request)
        future.add_done_callback(
            lambda done_future: self._service_response_callback(done_future, request_json)
        )

    def _service_response_callback(self, future, request_json: dict) -> None:
        if future.result() is not None:
            self.get_logger().info(f"ExecuteTask response: {future.result()}")
            return
        self.get_logger().error(f"ExecuteTask call failed: {future.exception()}")
        self.get_logger().error(f"request payload: {request_json}")

    def _parse_multi_task(self, msg: MultiTask):
        if self.drone_id not in msg.drones_id:
            self.get_logger().error(
                f"task not assigned to drone {self.drone_id}: {list(msg.drones_id)}"
            )
            return None

        if len(msg.task_stream) != 1:
            self.get_logger().error(
                f"unsupported task_stream length: {len(msg.task_stream)}"
            )
            return None
        single_task :SingleTask = msg.task_stream[0]
        if TEST_FLAG:
            self.get_logger().info(f"msg: {msg}")
            single_task.drones_id = msg.drones_id
        return self._parse_single_task(single_task)

    def _parse_single_task(self, single_task: SingleTask):
        request_json = {
            "mission_id": single_task.task_id if single_task.task_id != '' else generate_random_string(),
            "agent_id": self.drone_name,
        }
        flight_alt_m = single_task.low_height if single_task.low_height > 0.0 else self.flight_alt_m
        if single_task.task_type == TASK_TYPE["TAKEOFF_TASK"]:
            request_json["capability_id"] = "takeoff"
            request_json["params"] = {
                "auto_takeoff": True,
                "takeoff_alt_m": flight_alt_m,
            }
            return request_json

        elif single_task.task_type == TASK_TYPE["RETURN_TASK"]:
            waypoint = self._assign_region(single_task.task_area, list(single_task.drones_id), flight_alt_m)
            if waypoint is None:
                return None
            request_json["capability_id"] = "return"
            request_json["params"] = {
                "auto_takeoff": True,
                "takeoff_alt_m": flight_alt_m,
                "waypoint_frame": "gps",
                "waypoints": [waypoint],
            }
            return request_json

        elif single_task.task_type == TASK_TYPE["LINE_TASK"]:
            waypoints = [
                {
                    "lat": waypoint.x_lat,
                    "lon": waypoint.y_long,
                    "alt": flight_alt_m,
                }
                for waypoint in single_task.arrive_path.waypoints
            ]
            if not waypoints:
                self.get_logger().error("LINE_TASK requires at least one GPS waypoint")
                return None
            request_json["capability_id"] = "goto_waypoints"
            request_json["params"] = {
                "auto_takeoff": True,
                "takeoff_alt_m": flight_alt_m,
                "waypoint_frame": "gps",
                "waypoints": waypoints,
            }
            return request_json

        elif single_task.task_type == TASK_TYPE["PATROL_TASK"]:
            waypoints = list(single_task.arrive_path.waypoints)
            if len(waypoints) != 2:
                self.get_logger().error(
                    f"PATROL_TASK requires exactly two arrive_path waypoints, got {len(waypoints)}"
                )
                return None
            request_json["capability_id"] = "patrol_line"
            request_json["primitive_id"] = single_task.task_id or "patrol_001"
            request_json["params"] = {
                "auto_takeoff": True,
                "takeoff_alt_m": flight_alt_m,
                "waypoint_frame": "gps",
                "waypoints": [
                    {"lat": point.x_lat, "lon": point.y_long, "alt": flight_alt_m}
                    for point in waypoints
                ],
                "max_laps": 1,
                "hold_time_at_endpoint_s": 0.0,
            }
            return request_json

        elif single_task.task_type == TASK_TYPE["COVERAGE_TASK"]:
            waypoints = self._generate_uav_paths(
                single_task.task_area,
                list(single_task.drones_id),
                flight_alt_m,
            )
            if waypoints is None:
                return None
            request_json["capability_id"] = "goto_waypoints"
            request_json["params"] = {
                "auto_takeoff": True,
                "takeoff_alt_m": flight_alt_m,
                "waypoint_frame": "gps",
                "waypoints": waypoints,
            }
            self.get_logger().info(f"generated {len(waypoints)} coverage waypoints")
            return request_json

        self.get_logger().error(f"unsupported task_type: {single_task.task_type}")
        return None

    def _generate_uav_paths(
        self,
        area_sample: AreaSample,
        drones_id: list[int],
        flight_alt_m: float,
    ):
        num_drones = len(drones_id)
        params = CoverParams(num_drones=num_drones, swath_width=self.swath_width_m)
        ref_lat = 0.0
        ref_lon = 0.0
        ref_alt = 0.0
        result = None

        if area_sample.type == AREA_SAMPLE_TYPE["CIRCLE"]:
            ref_lat = area_sample.data[0]
            ref_lon = area_sample.data[1]
            task_region = CircleRegion(center=(0.0, 0.0), radius=area_sample.data[2])
            result = plan_coverage(task_region, params)
        elif area_sample.type == AREA_SAMPLE_TYPE["POLYGON"]:
            ref_lat = area_sample.data[0]
            ref_lon = area_sample.data[1]
            vertices = []
            for index in range(0, len(area_sample.data), 2):
                east, north, _ = pm.geodetic2enu(
                    area_sample.data[index],
                    area_sample.data[index + 1],
                    ref_alt,
                    ref_lat,
                    ref_lon,
                    ref_alt,
                )
                vertices.append((east, north))
            self.get_logger().debug(f"coverage polygon vertices: {vertices}")
            result = plan_coverage(PolygonRegion(vertices=vertices), params)
        else:
            self.get_logger().error(f"unsupported area sample type: {area_sample.type}")
            return None

        if result.status == "degraded_single_pass":
            self.get_logger().warn("region too small: only one drone received a path")

        drone_index = drones_id.index(self.drone_id)
        uav_paths = []
        for waypoint in result.paths[drone_index]:
            lat, lon, _ = pm.enu2geodetic(
                waypoint.x,
                waypoint.y,
                ref_alt,
                ref_lat,
                ref_lon,
                ref_alt,
            )
            uav_paths.append({"lat": lat, "lon": lon, "alt": flight_alt_m})
        return uav_paths
    def _assign_region(self, area_sample: AreaSample,drones_id: list[int],flight_alt_m: float):
        num_drones = len(drones_id)
        index_id = drones_id.index(self.drone_id)
        radius = 0.0
        ref_alt = 0.0
        if area_sample.type == AREA_SAMPLE_TYPE["POINT"]:
            radius = 4.0
        elif area_sample.type == AREA_SAMPLE_TYPE["CIRCLE"]:
            radius=area_sample.data[2]
        else:
            self.get_logger().error(f"unsupported area sample type: {area_sample.type}")
            return None
        if num_drones == 1:
            return {"lat": area_sample.data[0], "lon": area_sample.data[1], "alt": flight_alt_m}
        else:
            x = radius * np.cos(2 * np.pi * index_id / num_drones)
            y = radius * np.sin(2 * np.pi * index_id / num_drones)
            lat,lon, _ = pm.enu2geodetic(x, y, ref_alt, area_sample.data[0], area_sample.data[1], ref_alt)
            return {"lat": lat, "lon": lon, "alt": flight_alt_m}


def main(args=None):
    rclpy.init(args=args)
    node = MultiTaskBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("shutting down on keyboard interrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
