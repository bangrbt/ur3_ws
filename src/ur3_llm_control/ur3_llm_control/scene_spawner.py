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

        # Helper tao Marker co ban
        def make_marker(m_id, ns, m_type, pos, scale, color):
            m = Marker()
            m.header.frame_id = "base_link"
            m.header.stamp = now
            m.ns = ns
            m.id = m_id
            m.type = m_type
            m.action = Marker.ADD
            m.pose.position.x = float(pos[0])
            m.pose.position.y = float(pos[1])
            m.pose.position.z = float(pos[2])
            m.pose.orientation.w = 1.0
            m.scale.x = float(scale[0])
            m.scale.y = float(scale[1])
            m.scale.z = float(scale[2])
            m.color.r = float(color[0])
            m.color.g = float(color[1])
            m.color.b = float(color[2])
            m.color.a = float(color[3]) if len(color) > 3 else 1.0
            return m

        # 1. Ban thao tac co khi thuc te (Mặt bàn, Khung viền, 4 Chân bàn, Giằng chân & Bệ đỡ Robot)
        # 1.1 Mat ban cong nghiep (Tabletop)
        msg.markers.append(make_marker(marker_id, "table", Marker.CUBE, [0.26, 0.0, -0.02], [0.70, 0.90, 0.04], [0.38, 0.40, 0.46, 1.0]))
        marker_id += 1
        # 1.2 Khung nep vien kim loai bao quanh mat ban
        msg.markers.append(make_marker(marker_id, "table", Marker.CUBE, [0.26, 0.0, -0.038], [0.72, 0.92, 0.01], [0.22, 0.24, 0.28, 1.0]))
        marker_id += 1
        # 1.3 Be kim loai tron do chan de Robot UR3
        msg.markers.append(make_marker(marker_id, "table", Marker.CYLINDER, [0.0, 0.0, -0.01], [0.18, 0.18, 0.02], [0.58, 0.60, 0.65, 1.0]))
        marker_id += 1

        # 1.4 4 Chan ban thep chiu luc tai 4 goc
        leg_positions = [
            [0.56, 0.40, -0.23],   # Truoc - Trai
            [0.56, -0.40, -0.23],  # Truoc - Phai
            [-0.04, 0.40, -0.23],  # Sau - Trai
            [-0.04, -0.40, -0.23], # Sau - Phai
        ]
        for leg_pos in leg_positions:
            # Than chan ban
            msg.markers.append(make_marker(marker_id, "table_legs", Marker.CUBE, leg_pos, [0.05, 0.05, 0.38], [0.16, 0.17, 0.20, 1.0]))
            marker_id += 1
            # De cao su chan ban chong truot
            msg.markers.append(make_marker(marker_id, "table_feet", Marker.CUBE, [leg_pos[0], leg_pos[1], -0.4125], [0.07, 0.07, 0.015], [0.08, 0.08, 0.08, 1.0]))
            marker_id += 1

        # 1.5 Thanh giang ngang khung chan ban
        msg.markers.append(make_marker(marker_id, "table_frame", Marker.CUBE, [0.26, 0.40, -0.32], [0.60, 0.03, 0.03], [0.16, 0.17, 0.20, 1.0]))
        marker_id += 1
        msg.markers.append(make_marker(marker_id, "table_frame", Marker.CUBE, [0.26, -0.40, -0.32], [0.60, 0.03, 0.03], [0.16, 0.17, 0.20, 1.0]))
        marker_id += 1
        msg.markers.append(make_marker(marker_id, "table_frame", Marker.CUBE, [-0.04, 0.0, -0.32], [0.03, 0.80, 0.03], [0.16, 0.17, 0.20, 1.0]))
        marker_id += 1

        # Helper tao khay chua 3D (Day khay + 4 Thanh vien nho cao tao long khay lom)
        def add_3d_tray(cx, cy, base_color, rim_color, ns_name, label_text=None):
            nonlocal marker_id
            # 1. Day khay (Floor)
            msg.markers.append(make_marker(marker_id, ns_name, Marker.CUBE, [cx, cy, 0.0015], [0.086, 0.086, 0.003], base_color))
            marker_id += 1
            # 2. Thanh vien truoc (Front rim)
            msg.markers.append(make_marker(marker_id, ns_name, Marker.CUBE, [cx, cy - 0.0405, 0.006], [0.086, 0.005, 0.012], rim_color))
            marker_id += 1
            # 3. Thanh vien sau (Back rim)
            msg.markers.append(make_marker(marker_id, ns_name, Marker.CUBE, [cx, cy + 0.0405, 0.006], [0.086, 0.005, 0.012], rim_color))
            marker_id += 1
            # 4. Thanh vien trai (Left rim)
            msg.markers.append(make_marker(marker_id, ns_name, Marker.CUBE, [cx + 0.0405, cy, 0.006], [0.005, 0.076, 0.012], rim_color))
            marker_id += 1
            # 5. Thanh vien phai (Right rim)
            msg.markers.append(make_marker(marker_id, ns_name, Marker.CUBE, [cx - 0.0405, cy, 0.006], [0.005, 0.076, 0.012], rim_color))
            marker_id += 1

            # 6. Nhan chu 3D
            if label_text:
                m_txt = Marker()
                m_txt.header.frame_id = "base_link"
                m_txt.header.stamp = now
                m_txt.ns = f"{ns_name}_labels"
                m_txt.id = marker_id
                marker_id += 1
                m_txt.type = Marker.TEXT_VIEW_FACING
                m_txt.action = Marker.ADD
                m_txt.pose.position.x = float(cx)
                m_txt.pose.position.y = float(cy)
                m_txt.pose.position.z = 0.038
                m_txt.scale.z = 0.020
                m_txt.color.r = 1.0
                m_txt.color.g = 1.0
                m_txt.color.b = 1.0
                m_txt.color.a = 1.0
                m_txt.text = label_text
                msg.markers.append(m_txt)

        # 2. 3 Khay chua cho ban dau (Initial Waiting Trays tai X = 0.24m)
        # Dung chung 1 mau thep anh kim sang (Titanium Silver), hoan toan khong trung voi Do, Vang, Xanh cua cac khay phan loai
        WAIT_BASE_COLOR = [0.72, 0.75, 0.80, 0.65]
        WAIT_RIM_COLOR = [0.85, 0.88, 0.92, 0.95]
        source_trays = {
            "tray_1": {"pos": [0.24, -0.11], "base": WAIT_BASE_COLOR, "rim": WAIT_RIM_COLOR, "lbl": "WAIT 1"},
            "tray_2": {"pos": [0.24, 0.00], "base": WAIT_BASE_COLOR, "rim": WAIT_RIM_COLOR, "lbl": "WAIT 2"},
            "tray_3": {"pos": [0.24, 0.11], "base": WAIT_BASE_COLOR, "rim": WAIT_RIM_COLOR, "lbl": "WAIT 3"},
            "tray_4": {"pos": [0.24, -0.22], "base": WAIT_BASE_COLOR, "rim": WAIT_RIM_COLOR, "lbl": "WAIT GREEN"},
            "tray_5": {"pos": [0.24, 0.22], "base": WAIT_BASE_COLOR, "rim": WAIT_RIM_COLOR, "lbl": "WAIT PURPLE"},
        }
        for t_name, t_info in source_trays.items():
            add_3d_tray(t_info["pos"][0], t_info["pos"][1], t_info["base"], t_info["rim"], "source_trays", t_info["lbl"])

        # 3. 3 Khay Zone dich den (Target Zone Trays tai X = 0.35m) theo dung MSSV
        color_lut = {
            "red_cube": {"base": [0.95, 0.15, 0.15, 0.65], "rim": [0.98, 0.25, 0.25, 0.95]},
            "yellow_cube": {"base": [0.98, 0.85, 0.1, 0.65], "rim": [1.0, 0.92, 0.2, 0.95]},
            "blue_cube": {"base": [0.15, 0.45, 0.95, 0.65], "rim": [0.25, 0.55, 1.0, 0.95]},
        }

        for name, data in self.scene_config.get("zones", {}).items():
            pos = data.get("position", [0.35, 0.0, 0.001])
            target_cube = self.zone_mapping.get(name, "")
            c_info = color_lut.get(target_cube, {"base": [0.55, 0.55, 0.55, 0.5], "rim": [0.7, 0.7, 0.7, 0.9]})

            lbl = f"{name.upper()}\n({target_cube.replace('_cube', '').upper()})" if target_cube else name.upper()
            add_3d_tray(pos[0], pos[1], c_info["base"], c_info["rim"], "zone_trays", lbl)

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
