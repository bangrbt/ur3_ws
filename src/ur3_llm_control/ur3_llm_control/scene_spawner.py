#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Scene Spawner Node:
1. Khoi tao va cap nhat vat can va vat the trong MoveIt 2 PlanningScene (CollisionObjects).
2. Phat Marker Visualization 3D tren RViz (ban, 3 khoi hop mau, 3 vung mau kem chu thich Zone A, B, C theo MSSV).
3. Phat TF Frame cho tung vat the va tung Zone.
"""

import os
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Point, Pose
from visualization_msgs.msg import Marker, MarkerArray
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from tf2_ros import StaticTransformBroadcaster


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

        # Publishers & Broadcasters
        self.marker_pub = self.create_publisher(MarkerArray, "/scene_markers", 10)
        self.planning_scene_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        # 1. Phat Static TF dung 1 lan duy nhat voi timestamp 0 (hop le cho moi thoi diem)
        self._publish_tf()

        # 2. Timer dinh ky chi de refresh markers tren RViz (2s/lan)
        self.timer = self.create_timer(2.0, self._publish_markers)

        # 3. Phat collision objects sau 2 giay va 5 giay de chac chan MoveIt da san sang
        self.create_timer(2.0, self._publish_collision_objects_once)
        self.create_timer(5.0, self._publish_collision_objects_once)
        self.collision_published = False

        self.get_logger().info("Scene Spawner da san sang (Markers, Collision Objects, TF)!")

    def _load_yaml(self, filepath: str) -> dict:
        if os.path.exists(filepath):
            with open(filepath, "r", encoding="utf-8") as f:
                return yaml.safe_load(f) or {}
        return {}

    def publish_all(self):
        """Phat toan bo Markers, TF va PlanningScene CollisionObjects."""
        self._publish_tf()
        self._publish_markers()
        self._publish_collision_objects_once()

    def _publish_tf(self):
        """Phat TF frames cho cac vat va vung tren ban voi timestamp 0 (luon hop le)."""
        transforms = []
        zero_stamp = rclpy.time.Time().to_msg()

        # TF cho Cubes
        for name, data in self.scene_config.get("objects", {}).items():
            pos = data.get("initial_position", [0.3, 0.0, 0.02])
            t = TransformStamped()
            t.header.stamp = zero_stamp
            t.header.frame_id = "base_link"
            t.child_frame_id = name
            t.transform.translation.x = float(pos[0])
            t.transform.translation.y = float(pos[1])
            t.transform.translation.z = float(pos[2])
            t.transform.rotation.w = 1.0
            transforms.append(t)

        # TF cho Zones
        for name, data in self.scene_config.get("zones", {}).items():
            pos = data.get("position", [0.4, 0.0, 0.001])
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
            self.tf_broadcaster.sendTransform(transforms)

    def _publish_markers(self):
        """Phat Visualization Markers len RViz."""
        msg = MarkerArray()
        now = self.get_clock().now().to_msg()
        marker_id = 0

        # 1. Ban thao tac (Work Table)
        table = self.scene_config.get("table", {})
        t_pos = table.get("position", [0.35, 0.0, -0.2])
        t_size = table.get("size", [0.65, 0.85, 0.4])

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
        m_table.color.r = 0.35
        m_table.color.g = 0.35
        m_table.color.b = 0.38
        m_table.color.a = 0.95
        msg.markers.append(m_table)

        # 2. Cac Vung (Zones) va Nhan text
        for name, data in self.scene_config.get("zones", {}).items():
            pos = data.get("position", [0.4, 0.0, 0.001])
            size = data.get("size", [0.08, 0.08, 0.002])
            color = data.get("color", [0.5, 0.5, 0.5, 0.6])

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
            m_text.pose.position.z = float(pos[2]) + 0.03
            m_text.scale.z = 0.022
            m_text.color.r = 1.0
            m_text.color.g = 1.0
            m_text.color.b = 1.0
            m_text.color.a = 1.0
            target_obj = data.get("target_object", "")
            m_text.text = f"{name.upper()}\n({target_obj.replace('_cube', '')})"
            msg.markers.append(m_text)

        # 3. 3 Khoi hop (Cubes)
        for name, data in self.scene_config.get("objects", {}).items():
            pos = data.get("initial_position", [0.3, 0.0, 0.02])
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

        self.marker_pub.publish(msg)

    def _publish_collision_objects_once(self):
        """Phat vat can va vat the vao PlanningScene cua MoveIt."""
        ps = PlanningScene()
        ps.is_diff = True

        # Ban lam viec la vat can (Collision Object) de robot khong dam xuyen ban
        table = self.scene_config.get("table", {})
        t_pos = table.get("position", [0.35, 0.0, -0.2])
        t_size = table.get("size", [0.65, 0.85, 0.4])

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
