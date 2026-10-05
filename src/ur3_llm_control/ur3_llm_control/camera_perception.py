#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Camera Perception Node (Bai 03):
- Subscribe /camera/image_raw (sensor_msgs/Image)
- Xu ly OpenCV HSV color segmentation phat hien 5 khoi hop mau
- Xac dinh vi tri the gioi (X, Y) cua tung khoi thong qua camera intrinsics
- Xac dinh zone nao co vat, zone nao trong
- Expose service /perceive_scene -> tra ve JSON trang thai scene
- Publish /scene/camera_state topic (String JSON)
"""

import math
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from std_srvs.srv import Trigger

try:
    from cv_bridge import CvBridge
    import cv2
    import numpy as np
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False


# =========================================================================
# --- THONG SO CAU HINH ---
# =========================================================================

# Chieu cao ban lam viec so voi world frame (m) - de project pixel -> world
TABLE_Z = 0.02  # mat tren cua khoi hop

# Camera pose trong world frame (mat camera nhin xuong ngang)
# pose: x=0.30, y=0.0, z=0.95, rpy=0, 1.5707963 (pi/2), 0
# => mat camera nhin thang xuong (-Z world)
CAMERA_X = 0.30
CAMERA_Y = 0.00
CAMERA_Z = 0.95

# Khoang cach camera -> mat ban = CAMERA_Z - TABLE_Z_TABLE_SURFACE (0.0m)
# Mat tren ban tai Z=0 trong world frame
CAMERA_HEIGHT_ABOVE_TABLE = CAMERA_Z  # ~0.95m

# Cac vung zone va khay nguon (toa do world frame) de phan loai
ZONE_DEFINITIONS = {
    "zone_a":      {"center": [0.35, -0.11], "radius": 0.055},
    "zone_b":      {"center": [0.35,  0.00], "radius": 0.055},
    "zone_c":      {"center": [0.35,  0.11], "radius": 0.055},
    "zone_temp":   {"center": [0.18, -0.22], "radius": 0.055},
    "zone_temp_1": {"center": [0.18, -0.22], "radius": 0.055},
    "zone_temp_2": {"center": [0.18,  0.22], "radius": 0.055},
    "zone_temp_3": {"center": [0.18,  0.00], "radius": 0.055},
    "source_red":    {"center": [0.24, -0.11], "radius": 0.055},
    "source_yellow": {"center": [0.24,  0.00], "radius": 0.055},
    "source_blue":   {"center": [0.24,  0.11], "radius": 0.055},
    "source_green":  {"center": [0.24, -0.22], "radius": 0.055},
    "source_purple": {"center": [0.24,  0.22], "radius": 0.055},
}

# HSV color ranges cho tung khoi hop (opencv HSV: H=[0,179], S=[0,255], V=[0,255])
COLOR_RANGES = {
    "red_cube": [
        # Red nam o 2 vung trong HSV
        ([0, 120, 80], [10, 255, 255]),
        ([165, 120, 80], [179, 255, 255]),
    ],
    "yellow_cube": [
        ([20, 120, 100], [35, 255, 255]),
    ],
    "blue_cube": [
        ([100, 100, 80], [130, 255, 255]),
    ],
    "green_cube": [
        ([40, 80, 60], [85, 255, 255]),
    ],
    "purple_cube": [
        ([130, 60, 60], [165, 255, 255]),
    ],
}

MIN_CONTOUR_AREA = 300  # pixel^2 - bo qua nhieu nho


class CameraPerceptionNode(Node):
    """Node nhan anh tu camera gazebo, xu ly va phat trang thai scene."""

    def __init__(self):
        super().__init__("camera_perception_node",
                         automatically_declare_parameters_from_overrides=True)

        if not CV2_AVAILABLE:
            self.get_logger().error(
                "cv2 / cv_bridge khong tim thay! "
                "Camera perception se hoat dong o che do fallback (khong xu ly anh)."
            )

        self.bridge = CvBridge() if CV2_AVAILABLE else None
        self.latest_image = None
        self.camera_info = None

        # Ket qua phan tich moi nhat (thread-safe qua GIL Python)
        self.scene_state = {
            "detected_objects": {},  # {object_name: {x, y, location}}
            "zone_occupants": {z: None for z in ["zone_a", "zone_b", "zone_c",
                                                  "zone_temp", "zone_temp_1",
                                                  "zone_temp_2", "zone_temp_3"]},
            "camera_active": CV2_AVAILABLE,
        }

        # Subscriber camera image
        self.image_sub = self.create_subscription(
            Image, "/camera/image_raw", self._image_callback, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, "/camera/camera_info", self._camera_info_callback, 5
        )

        # Publisher scene state
        self.state_pub = self.create_publisher(String, "/scene/camera_state", 10)

        # Service perceive_scene -> tra ve JSON scene state
        self.perceive_srv = self.create_service(
            Trigger, "/perceive_scene", self._perceive_service_callback
        )

        # Timer xu ly anh moi 0.5 giay
        self.process_timer = self.create_timer(0.5, self._process_and_publish)

        self.get_logger().info(
            f"Camera Perception Node khoi dong. OpenCV: {'OK' if CV2_AVAILABLE else 'FALLBACK MODE'}"
        )

    def _camera_info_callback(self, msg: CameraInfo):
        """Luu thong so noi tang camera (focal length, principal point)."""
        self.camera_info = msg

    def _image_callback(self, msg: Image):
        """Nhan frame anh moi nhat tu camera."""
        self.latest_image = msg

    def _process_and_publish(self):
        """Xu ly anh hien tai, cap nhat scene state va publish."""
        if not CV2_AVAILABLE or self.latest_image is None:
            # Neu khong co OpenCV hoac chua co anh, publish trang thai hien tai
            self._publish_state()
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge: {e}")
            return

        detected = {}
        for obj_name, ranges in COLOR_RANGES.items():
            pos = self._detect_object_by_color(cv_image, obj_name, ranges)
            if pos is not None:
                world_xy = self._pixel_to_world(pos[0], pos[1],
                                                cv_image.shape[1],
                                                cv_image.shape[0])
                if world_xy:
                    location = self._classify_location(world_xy[0], world_xy[1])
                    detected[obj_name] = {
                        "x": round(world_xy[0], 4),
                        "y": round(world_xy[1], 4),
                        "location": location,
                        "pixel_u": pos[0],
                        "pixel_v": pos[1],
                    }

        # Cap nhat zone_occupants tu du lieu detect duoc
        zone_occupants = {z: None for z in ["zone_a", "zone_b", "zone_c",
                                             "zone_temp", "zone_temp_1",
                                             "zone_temp_2", "zone_temp_3"]}
        for obj_name, info in detected.items():
            loc = info.get("location", "unknown")
            if loc in zone_occupants:
                zone_occupants[loc] = obj_name

        self.scene_state = {
            "detected_objects": detected,
            "zone_occupants": zone_occupants,
            "camera_active": True,
        }
        self._publish_state()

    def _detect_object_by_color(self, bgr_image, obj_name: str,
                                 color_ranges: list):
        """
        Tim khoi hop theo mau HSV trong anh.
        Tra ve (u, v) pixel cua centroid lon nhat, hoac None.
        """
        hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        mask = None
        for (lo, hi) in color_ranges:
            lo_arr = np.array(lo, dtype=np.uint8)
            hi_arr = np.array(hi, dtype=np.uint8)
            sub_mask = cv2.inRange(hsv, lo_arr, hi_arr)
            mask = sub_mask if mask is None else cv2.bitwise_or(mask, sub_mask)

        if mask is None:
            return None

        # Loc nhieu bang morphology
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        # Lay contour lon nhat
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < MIN_CONTOUR_AREA:
            return None

        M = cv2.moments(largest)
        if M["m00"] == 0:
            return None

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        return (cx, cy)

    def _pixel_to_world(self, u: int, v: int, img_w: int, img_h: int):
        """
        Chuyen doi toa do pixel (u, v) -> toa do the gioi (X, Y) (met).
        Su dung mo hinh pinhole don gian voi camera nhin thang xuong.
        Camera tai vi tri (CAMERA_X, CAMERA_Y, CAMERA_Z) nhin theo -Z.
        """
        if self.camera_info is not None:
            # Su dung thong so thuc tu CameraInfo
            fx = self.camera_info.k[0]  # K[0,0]
            fy = self.camera_info.k[4]  # K[1,1]
            cx = self.camera_info.k[2]  # K[0,2]
            cy = self.camera_info.k[5]  # K[1,2]
        else:
            # Uoc luong fallback tu FOV va kich thuoc anh
            # horizontal_fov = 1.15 rad, width = 640
            fov_h = 1.15
            fx = (img_w / 2.0) / math.tan(fov_h / 2.0)
            fy = fx  # square pixels
            cx = img_w / 2.0
            cy = img_h / 2.0

        # Camera tai (CAMERA_X, CAMERA_Y, CAMERA_Z) nhin thang xuong
        # -> mat ban nam tai Z_world = 0.0
        # Khoang cach tu camera den mat ban
        depth = CAMERA_Z  # mat ban tai Z_world = 0

        # Pixel -> camera-frame normalized coordinates
        x_cam = (u - cx) / fx * depth
        y_cam = (v - cy) / fy * depth

        # Camera orientation: quay 90 do quanh Y (nhin xuong)
        # camera X -> world -Y, camera Y -> world X (voi orientation rpy=0,pi/2,0)
        world_x = CAMERA_X + (-y_cam)
        world_y = CAMERA_Y + (-x_cam)

        # Clamp vao vung ban hop le
        if not (0.10 <= world_x <= 0.55 and -0.40 <= world_y <= 0.40):
            return None

        return (world_x, world_y)

    def _classify_location(self, x: float, y: float) -> str:
        """
        Phan loai vi tri (x, y) vao zone, source_tray, hoac 'unknown'.
        Tim zone gan nhat trong bat ky radius nao.
        """
        best_zone = "unknown"
        best_dist = float("inf")

        for zone_name, zone_info in ZONE_DEFINITIONS.items():
            cx, cy = zone_info["center"]
            r = zone_info["radius"]
            dist = math.sqrt((x - cx) ** 2 + (y - cy) ** 2)
            if dist <= r and dist < best_dist:
                best_dist = dist
                best_zone = zone_name

        # Chuyen source_xxx -> source_tray cho nhan gon
        if best_zone.startswith("source_"):
            return "source_tray"

        return best_zone

    def _publish_state(self):
        """Publish trang thai scene duoi dang JSON."""
        msg = String()
        msg.data = json.dumps(self.scene_state)
        self.state_pub.publish(msg)

    def _perceive_service_callback(self, request, response):
        """Service /perceive_scene -> kich hoat phan tich ngay lap tuc va tra ket qua."""
        # Kich hoat xu ly ngay (neu co anh)
        self._process_and_publish()

        response.success = True
        response.message = json.dumps(self.scene_state)
        return response

    def get_scene_state(self) -> dict:
        """Tra ve ban sao trang thai scene hien tai."""
        return dict(self.scene_state)


def main(args=None):
    rclpy.init(args=args)
    node = CameraPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
