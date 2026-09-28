#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Scene Spawner Node:
1. Khoi tao va cap nhat vat can va vat the trong MoveIt 2 PlanningScene (CollisionObjects).
2. Phat Marker Visualization 3D tren RViz (ban, 3 khay phoi, 3 khoi hop, 3 khay zone A, B, C theo MSSV).
3. Phat TF Frame cho tung vat the va tung Zone.
"""

import os
import json
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped, Point, Pose
from visualization_msgs.msg import Marker, MarkerArray
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from .student_utils import parse_student_info


class SceneSpawner(Node):
    """Node quan ly khong gian lam viec 3D cho UR3."""

    def __init__(self):
        super().__init__("scene_spawner_node", automatically_declare_parameters_from_overrides=True)
        self.get_logger().info("Khoi tao Scene Spawner Node...")

        try:
            from ament_index_python.packages import get_package_share_directory
            config_dir = os.path.join(get_package_share_directory("ur3_llm_control"), "config")
        except Exception:
            config_dir = os.path.join(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config"
            )
        self.scene_config = self._load_yaml(os.path.join(config_dir, "scene.yaml"))
        self.student_config = self._load_yaml(os.path.join(config_dir, "student_config.yaml"))

        # Doc thong tin sinh vien va tu dong tinh P = XX mod 6
        self.student_name = self.student_config.get("student_name", "Lê Anh Tuấn Bằng")
        self.student_id = str(self.student_config.get("student_id", "23020723"))
        self.xx, self.p_value, self.zone_mapping = parse_student_info(self.student_id)

        # Toa do dong cua cac khoi hop
        self.cube_positions = {}
        for name, data in self.scene_config.get("objects", {}).items():
            self.cube_positions[name] = list(data.get("initial_position", [0.24, 0.0, 0.02]))

        # Publishers & Broadcasters
        self.marker_pub = self.create_publisher(MarkerArray, "/scene_markers", 10)
        self.planning_scene_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.dynamic_tf_broadcaster = TransformBroadcaster(self)

        # Subscriber nhan cap nhat trang thai vat the dong tu RobotSkills
        self.cube_state_sub = self.create_subscription(
            String, "/scene/cube_states", self._cube_states_cb, 10
        )

        # 1. Phat Static TF cho Zones
        self._publish_static_tf()
        # 2. Phat Dynamic TF cho Cubes
        self._publish_dynamic_tf()

        # 3. Timer dinh ky refresh markers tren RViz (1s/lan)
        self.timer = self.create_timer(1.0, self._publish_markers)

        # 4. Phat collision objects sau 2 giay va 5 giay de chac chan MoveIt da san sang
        self.create_timer(2.0, self._publish_collision_objects_once)
        self.create_timer(5.0, self._publish_collision_objects_once)
        self.collision_published = False

        self.get_logger().info(f"Scene Spawner: Sinh vien {self.student_name} - MSSV {self.student_id} (P={self.p_value})")

    def _cube_states_cb(self, msg: String):
        """Cap nhat vi tri vat the khi robot thao tac pick/place."""
        try:
            data = json.loads(msg.data)
            for name, pos in data.items():
                if name in self.cube_positions:
                    self.cube_positions[name] = [float(pos[0]), float(pos[1]), float(pos[2])]
            self._publish_markers()
            self._publish_dynamic_tf()
        except Exception:
            pass

    def _load_yaml(self, filepath: str) -> dict:
        if os.path.exists(filepath):
            with open(filepath, "r", encoding="utf-8") as f:
                return yaml.safe_load(f) or {}
        return {}

    def publish_all(self):
        """Phat toan bo Markers, TF va PlanningScene CollisionObjects."""
        self._publish_static_tf()
        self._publish_dynamic_tf()
        self._publish_markers()
        self._publish_collision_objects_once()

    def _publish_static_tf(self):
        """Phat Static TF cho cac vung Zone (khong doi vi tri)."""
        transforms = []
        zero_stamp = rclpy.time.Time().to_msg()

        for name, data in self.scene_config.get("zones", {}).items():
            pos = data.get("position", [0.35, 0.0, 0.001])
            t = TransformStamped()
            t.header.stamp = zero_stamp
            t.header.frame_id = "base_link"
            t.child_frame_id = name
            t.transform.translation.x = float(pos[0])
            t.transform.translation.y = float(pos[1])
            t.transform.translation.z = float(pos[2])
            t.transform.rotation.w = 1.0
            transforms.append(t)

        if transforms:
            self.static_tf_broadcaster.sendTransform(transforms)

    def _publish_dynamic_tf(self):
        """Phat Dynamic TF cho cac vat the Cube theo toa do thuc te."""
        transforms = []
        now = self.get_clock().now().to_msg()

        for name, pos in self.cube_positions.items():
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = "base_link"
            t.child_frame_id = name
            t.transform.translation.x = float(pos[0])
            t.transform.translation.y = float(pos[1])
            t.transform.translation.z = float(pos[2])
            t.transform.rotation.w = 1.0
            transforms.append(t)

        if transforms:
            self.dynamic_tf_broadcaster.sendTransform(transforms)

    def _publish_markers(self):
        """Phat Visualization Markers len RViz day du, dep va noi bat."""
        msg = MarkerArray()
        now = self.get_clock().now().to_msg()
        marker_id = 0

        # 1. Ban thao tac (Work Table)
        table = self.scene_config.get("table", {})
        t_pos = table.get("position", [0.26, 0.0, -0.21])
        t_size = table.get("size", [0.70, 0.90, 0.42])

        m_table = Marker()
        m_table.header.frame_id = "base_link"
        m_table.header.stamp = now
        m_table.ns = "environment"
        m_table.id = marker_id
        marker_id += 1
        m_table.type = Marker.CUBE
        m_table.action = Marker.ADD
        m_table.pose.position.x = float(t_pos[0])
        m_table.pose.position.y = float(t_pos[1])
        m_table.pose.position.z = float(t_pos[2])
        m_table.pose.orientation.w = 1.0
        m_table.scale.x = float(t_size[0])
        m_table.scale.y = float(t_size[1])
        m_table.scale.z = float(t_size[2])
        m_table.color.r = 0.28
        m_table.color.g = 0.30
        m_table.color.b = 0.35
        m_table.color.a = 0.95
        msg.markers.append(m_table)

        # 2. 3 Khay chua phoi ban dau (Source Trays)
        source_trays = {
            "tray_red": {"pos": [0.24, -0.11, 0.001], "color": [0.95, 0.2, 0.2, 0.35], "label": "TRAY RED"},
            "tray_yellow": {"pos": [0.24, 0.00, 0.001], "color": [0.95, 0.85, 0.1, 0.35], "label": "TRAY YELLOW"},
            "tray_blue": {"pos": [0.24, 0.11, 0.001], "color": [0.2, 0.5, 0.95, 0.35], "label": "TRAY BLUE"},
        }
        for t_name, t_data in source_trays.items():
            m_tr = Marker()
            m_tr.header.frame_id = "base_link"
            m_tr.header.stamp = now
            m_tr.ns = "source_trays"
            m_tr.id = marker_id
            marker_id += 1
            m_tr.type = Marker.CUBE
            m_tr.action = Marker.ADD
            m_tr.pose.position.x = t_data["pos"][0]
            m_tr.pose.position.y = t_data["pos"][1]
            m_tr.pose.position.z = t_data["pos"][2]
            m_tr.pose.orientation.w = 1.0
            m_tr.scale.x = 0.07
            m_tr.scale.y = 0.07
            m_tr.scale.z = 0.002
            m_tr.color.r = t_data["color"][0]
            m_tr.color.g = t_data["color"][1]
            m_tr.color.b = t_data["color"][2]
            m_tr.color.a = t_data["color"][3]
            msg.markers.append(m_tr)

        # 3. 3 Khay Zone dich den (Target Zones) theo dung MSSV
        color_lut = {
            "red_cube": [0.95, 0.2, 0.2, 0.7],
            "yellow_cube": [0.98, 0.85, 0.1, 0.7],
            "blue_cube": [0.15, 0.45, 0.95, 0.7],
        }

        for name, data in self.scene_config.get("zones", {}).items():
            pos = data.get("position", [0.35, 0.0, 0.001])
            size = data.get("size", [0.085, 0.085, 0.003])
            
            # Neu la zone_a, b, c thi lay mau theo zone_mapping tinh tu MSSV
            target_cube = self.zone_mapping.get(name, "")
            color = color_lut.get(target_cube, data.get("color", [0.5, 0.5, 0.5, 0.6]))

            m_zone = Marker()
            m_zone.header.frame_id = "base_link"
            m_zone.header.stamp = now
            m_zone.ns = "zones"
            m_zone.id = marker_id
            marker_id += 1
            m_zone.type = Marker.CUBE
            m_zone.action = Marker.ADD
            m_zone.pose.position.x = float(pos[0])
            m_zone.pose.position.y = float(pos[1])
            m_zone.pose.position.z = float(pos[2])
            m_zone.pose.orientation.w = 1.0
            m_zone.scale.x = float(size[0])
            m_zone.scale.y = float(size[1])
            m_zone.scale.z = float(size[2])
            m_zone.color.r = float(color[0])
            m_zone.color.g = float(color[1])
            m_zone.color.b = float(color[2])
            m_zone.color.a = float(color[3])
            msg.markers.append(m_zone)

            # Text Label cho Zone
            m_text = Marker()
            m_text.header.frame_id = "base_link"
            m_text.header.stamp = now
            m_text.ns = "zone_labels"
            m_text.id = marker_id
            marker_id += 1
            m_text.type = Marker.TEXT_VIEW_FACING
            m_text.action = Marker.ADD
            m_text.pose.position.x = float(pos[0])
            m_text.pose.position.y = float(pos[1])
            m_text.pose.position.z = float(pos[2]) + 0.035
            m_text.scale.z = 0.022
            m_text.color.r = 1.0
            m_text.color.g = 1.0
            m_text.color.b = 1.0
            m_text.color.a = 1.0
            if target_cube:
                m_text.text = f"{name.upper()}\n({target_cube.replace('_cube', '').upper()})"
            else:
                m_text.text = name.upper()
            msg.markers.append(m_text)

        # 4. 3 Khoi hop (Cubes)
        for name, data in self.scene_config.get("objects", {}).items():
            pos = self.cube_positions.get(name, data.get("initial_position", [0.24, 0.0, 0.02]))
            size = data.get("size", [0.04, 0.04, 0.04])
            color = data.get("color", [0.9, 0.1, 0.1, 1.0])

            m_cube = Marker()
            m_cube.header.frame_id = "base_link"
            m_cube.header.stamp = now
            m_cube.ns = "cubes"
            m_cube.id = marker_id
            marker_id += 1
            m_cube.type = Marker.CUBE
            m_cube.action = Marker.ADD
            m_cube.pose.position.x = float(pos[0])
            m_cube.pose.position.y = float(pos[1])
            m_cube.pose.position.z = float(pos[2])
            m_cube.pose.orientation.w = 1.0
            m_cube.scale.x = float(size[0])
            m_cube.scale.y = float(size[1])
            m_cube.scale.z = float(size[2])
            m_cube.color.r = float(color[0])
            m_cube.color.g = float(color[1])
            m_cube.color.b = float(color[2])
            m_cube.color.a = float(color[3])
            msg.markers.append(m_cube)

        # 5. Banner chu thich tren khong: Thong tin sinh vien & MSSV
        m_banner = Marker()
        m_banner.header.frame_id = "base_link"
        m_banner.header.stamp = now
        m_banner.ns = "student_banner"
        m_banner.id = marker_id
        marker_id += 1
        m_banner.type = Marker.TEXT_VIEW_FACING
        m_banner.action = Marker.ADD
        m_banner.pose.position.x = 0.30
        m_banner.pose.position.y = 0.0
        m_banner.pose.position.z = 0.38
        m_banner.scale.z = 0.026
        m_banner.color.r = 0.2
        m_banner.color.g = 1.0
        m_banner.color.b = 0.4
        m_banner.color.a = 0.95
        m_banner.text = f"SV: {self.student_name} | MSSV: {self.student_id} (P={self.p_value})"
        msg.markers.append(m_banner)

        self.marker_pub.publish(msg)

    def _publish_collision_objects_once(self):
        """Phat vat can va vat the vao PlanningScene cua MoveIt."""
        ps = PlanningScene()
        ps.is_diff = True

        table = self.scene_config.get("table", {})
        t_pos = table.get("position", [0.26, 0.0, -0.21])
        t_size = table.get("size", [0.70, 0.90, 0.42])

        co_table = CollisionObject()
        co_table.header.frame_id = "base_link"
        co_table.id = "work_table"
        co_table.operation = CollisionObject.ADD

        sp_table = SolidPrimitive()
        sp_table.type = SolidPrimitive.BOX
        sp_table.dimensions = [float(t_size[0]), float(t_size[1]), float(t_size[2])]

        co_table.primitives.append(sp_table)
        p = Pose()
        p.position.x = float(t_pos[0])
        p.position.y = float(t_pos[1])
        p.position.z = float(t_pos[2])
        p.orientation.w = 1.0
        co_table.primitive_poses.append(p)

        ps.world.collision_objects.append(co_table)
        self.planning_scene_pub.publish(ps)


def main(args=None):
    rclpy.init(args=args)
    node = SceneSpawner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
