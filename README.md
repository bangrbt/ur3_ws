# Workspace `ur3_ws` - Universal Robots UR3 ROS 2 Humble Project

> **Kho lưu trữ tổng hợp các bài tập môn học Robot & Thị giác máy tính**  
> **Sinh viên thực hiện:** Lê Anh Tuấn Bằng  
> **Mã số sinh viên (MSSV):** `23020723`  
> **Hệ điều hành:** Ubuntu 22.04 LTS (Jammy Jellyfish)  
> **Nền tảng:** ROS 2 Humble Hawksbill, MoveIt 2, Ignition Gazebo (Fortress)

---

## 📌 Mục Lục
1. [Giới Thiệu Workspace](#1-giới-thiệu-workspace)
2. [Cấu Trúc Thư Mục & Quản Lý Nhánh Git](#2-cấu-trúc-thư-mục--quản-lý-nhánh-git)
3. [Danh Mục Các Bài Tập Trong Workspace](#3-danh-mục-các-bài-tập-trong-workspace)
4. [Lệnh Dọn Dẹp Tiến Trình Treo (Bắt buộc trước mỗi lần chạy)](#4-lệnh-dọn-dẹp-tiến-trình-treo-bắt-buộc-trước-mỗi-lần-chạy)
5. [Hướng Dẫn Cài Đặt & Biên Dịch Toàn Bộ Workspace](#5-hướng-dẫn-cài-đặt--biên-dịch-toàn-bộ-workspace)
6. [Hướng Dẫn Chuyển Nhánh & Chạy Từng Bài Tập](#6-hướng-dẫn-chuyển-nhánh--chạy-từng-bài-tập)

---

## 1. Giới Thiệu Workspace

Workspace `ur3_ws` là môi trường phát triển và mô phỏng hoàn chỉnh cho cánh tay robot công nghiệp **Universal Robots UR3 / UR3e** (6 bậc tự do), tích hợp:
* **Điều khiển chuyển động & Tránh va chạm:** MoveIt 2 (Descartes Cartesian Path Planning & Joint Space Planning).
* **Mô phỏng vật lý 3D:** Ignition Gazebo (Gazebo Fortress) kết hợp bộ điều khiển ROS 2 Control (`joint_trajectory_controller`).
* **Trực quan hóa thời gian thực:** RViz 2 (Hiển thị quỹ đạo, trạng thái va chạm, Marker 3D và bàn thao tác công nghiệp).
* **Trí tuệ nhân tạo (AI & LLM):** Kết nối 9Router API Gateway với các mô hình ngôn ngữ lớn tiên tiến (Google Gemini, OpenAI GPT) để tự động dịch câu lệnh tự nhiên sang kế hoạch hành động theo kỹ năng (Skill-based Planning).

---

## 2. Cấu Trúc Thư Mục & Quản Lý Nhánh Git

### 2.1. Cấu trúc cây thư mục Workspace
```text
ur3_ws/
├── .git/                            # Git repository của toàn bộ workspace
├── .gitignore                       # Loại trừ build/, install/, log/, báo cáo LaTeX/PDF
├── .gitmodules                      # Khai báo Git Submodules cho Driver & Simulation UR
├── README.md                        # [Tài liệu này] Tổng quan toàn bộ workspace
│
└── src/
    ├── my_ur3_draw/                 # [BÀI TẬP 1] UR3 vẽ chữ trên mặt phẳng bảng đứng
    │   ├── launch/                  # File launch khởi động vẽ chữ & mô phỏng
    │   ├── src/                     # C++ Source code thuật toán xử lý ảnh & MoveIt
    │   ├── rviz/                    # Cấu hình RViz hiển thị nét vẽ thời gian thực
    │   └── README.md                # 📖 Hướng dẫn chi tiết riêng cho Bài tập 1
    │
    ├── ur3_llm_control/             # [BÀI TẬP 2] UR3 điều khiển bằng LLM & Robot Skills
    │   ├── config/                  # Cấu hình LLM 9Router, scene 3D, MSSV 23020723
    │   ├── launch/                  # Launch file trọn gói 1 lệnh (Gazebo + MoveIt + LLM)
    │   ├── ur3_llm_control/         # Module Python: Planner, Executor, Validator, Skills
    │   ├── worlds/                  # SDF World Gazebo: Bàn thao tác, 3 khay chờ, 4 khay Zone
    │   ├── urdf/                    # URDF/Xacro UR3 tích hợp tay kẹp Gripper 2 ngón
    │   └── README.md                # 📖 Hướng dẫn chi tiết sâu cho Bài tập 2
    │
    ├── Universal_Robots_ROS2_Driver/        # Driver chính hãng Universal Robots (Submodule)
    └── Universal_Robots_ROS2_GZ_Simulation/ # Mô phỏng Ignition Gazebo UR (Submodule)
```

### 2.2. Phân nhánh Git theo từng Bài tập (Git Branches)
Kho lưu trữ được tổ chức theo từng nhánh riêng biệt tương ứng với từng bài tập môn học:

| Tên Nhánh Git | Bài Tập Tương Ứng | Nội Dung Chính |
| :--- | :--- | :--- |
| **`main`** | Nhánh gốc chung | Khung workspace chuẩn mực ban đầu. |
| **`bai-tap-tuan-1`** (hoặc `tuan-1`) | **Bài tập 1** | Package `my_ur3_draw`: Vẽ chữ cái bất kỳ hoặc vẽ theo ảnh `letter.png` trên bảng đứng. |
| **`assignments_2`** | **Bài tập 2** | Package `ur3_llm_control`: Điều khiển gắp đặt phân loại khối hộp theo MSSV bằng 100% Online LLM qua 9Router Gateway. |

---

## 3. Danh Mục Các Bài Tập Trong Workspace

### 📘 Bài tập 1: Robot UR3 Vẽ Chữ Trên Bảng Đứng (`my_ur3_draw`)
* **Mục tiêu:** Cánh tay robot UR3 đọc ảnh chữ cái (hoặc nhận chữ cái bất kỳ do người dùng gõ từ bàn phím), trích xuất contour viền bằng OpenCV và dùng MoveIt 2 điều khiển bút vẽ trên mặt phẳng bảng đứng.
* **Đặc điểm nổi bật:** Vẽ phẳng tuyệt đối, tự động phân tách nét trong/ngoài, tự động nâng bút khi chuyển nét và hiển thị nét mực đỏ thời gian thực trong RViz.
* **Chi tiết & Hướng dẫn chạy:** Xem tại [src/my_ur3_draw/README.md](src/my_ur3_draw/README.md).

### 🤖 Bài tập 2: Điều Khiển UR3 Bằng LLM & Skill-based Planning (`ur3_llm_control`)
* **Mục tiêu:** Nhận câu lệnh tự nhiên tiếng Việt hoặc tiếng Anh, kết nối qua **9Router API Gateway** tới mô hình LLM (`gemini/gemini-3.5-flash-lite`), chuyển đổi thành kế hoạch chuỗi kỹ năng Robot Skills có cấu trúc (`pick`, `place`, `swap`, `stack`, `home`), kiểm tra an toàn bằng Task Validator và thực thi mượt mà trên UR3 có tay kẹp.
* **Đặc điểm nổi bật:**
  - **Nói gì làm nấy:** Ưu tiên tuyệt đối đích đến cụ thể do người dùng yêu cầu, không tự ý áp đặt theo màu khối.
  - **Cá nhân hóa theo MSSV `23020723`:** Tự động tính $P = 23 \pmod 6 = 5$ (Vùng A $\to$ Blue, Vùng B $\to$ Yellow, Vùng C $\to$ Red).
  - **Tối ưu 2 lớp (Dual-layer Optimization):** Tự động bỏ qua (`SKIPPED`) các khối đã ở sẵn vị trí mục tiêu, không gắp lên thả lại thừa thãi.
  - **Khay chờ đồng nhất:** 3 khay chờ ban đầu mang màu Titanium Silver đồng bộ trên cả Gazebo và RViz.
* **Chi tiết & Hướng dẫn chạy:** Xem tại [src/ur3_llm_control/README.md](src/ur3_llm_control/README.md).

---

## 4. Lệnh Dọn Dẹp Tiến Trình Treo (Bắt buộc trước mỗi lần chạy)

Khi tắt mô phỏng Gazebo hoặc RViz bằng `Ctrl + C`, các tiến trình nền (`ign gazebo`, `ruby`, `gzserver`) có thể vẫn chạy ngầm chiếm cổng và tài nguyên phần cứng. **Luôn chạy lệnh dọn dẹp sau trước khi khởi động bất kỳ bài tập nào:**

```bash
killall -9 ruby ign gzserver gzclient rviz2 2>/dev/null || pkill -9 -f "ign gazebo"
```

---

## 5. Hướng Dẫn Cài Đặt & Biên Dịch Toàn Bộ Workspace

### Bước 1: Clone kho lưu trữ kèm Submodule
```bash
git clone --recurse-submodules https://github.com/bangrbt/ur3_ws.git ~/ur3_ws
cd ~/ur3_ws
```
*(Nếu đã clone thông thường không có cờ trên, chạy: `git submodule update --init --recursive`)*.

### Bước 2: Cài đặt các gói phụ thuộc (Dependencies)
```bash
cd ~/ur3_ws
sudo apt update
rosdep update
rosdep install --ignore-src --from-paths src -y -r
```

### Bước 3: Biên dịch toàn bộ Workspace
```bash
cd ~/ur3_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 6. Hướng Dẫn Chuyển Nhánh & Chạy Từng Bài Tập

### 6.1. Chạy Bài Tập 1 (Nhánh `bai-tap-tuan-1`)
```bash
# 1. Chuyển sang nhánh Bài tập 1:
git checkout bai-tap-tuan-1

# 2. Dọn dẹp tiến trình cũ:
killall -9 ruby ign gzserver gzclient rviz2 2>/dev/null || pkill -9 -f "ign gazebo"

# 3. Build package my_ur3_draw:
source /opt/ros/humble/setup.bash
colcon build --packages-select my_ur3_draw --symlink-install
source install/setup.bash

# 4. Khởi chạy mô phỏng vẽ chữ (ví dụ vẽ chữ B):
ros2 launch my_ur3_draw ur3_draw_sim.launch.py letter:=B
```
*(Tham khảo thêm các tùy chọn vẽ ảnh, vẽ từ bất kỳ tại [src/my_ur3_draw/README.md](src/my_ur3_draw/README.md))*

---

### 6.2. Chạy Bài Tập 2 (Nhánh `assignments_2`)
```bash
# 1. Chuyển sang nhánh Bài tập 2:
git checkout assignments_2

# 2. Dọn dẹp tiến trình cũ:
killall -9 ruby ign gzserver gzclient rviz2 2>/dev/null || pkill -9 -f "ign gazebo"

# 3. Khởi chạy 9Router Gateway ở một terminal riêng:
npx 9router

# 4. Build package ur3_llm_control:
source /opt/ros/humble/setup.bash
colcon build --packages-select ur3_llm_control --symlink-install
source install/setup.bash

# 5. Khởi chạy trọn gói hệ thống Bài 2 (Gazebo + MoveIt 2 + RViz + LLM):
ros2 launch ur3_llm_control llm_robot.launch.py
```
*(Hệ thống sẽ mở giao diện dòng lệnh console, bạn có thể nhập lệnh: `"Đưa khối đỏ vào ô A"`, `"Chuyển khối vàng vào ô C"`, hoặc `"Hãy sắp xếp các khối theo mã sinh viên"`)*.  
*(Xem tài liệu chi tiết đầy đủ tại [src/ur3_llm_control/README.md](src/ur3_llm_control/README.md))*.

---

## 👨‍💻 Thông Tin Sinh Viên
* **Họ và tên:** Lê Anh Tuấn Bằng
* **MSSV:** 23020723
* **Repository GitHub:** [https://github.com/bangrbt/ur3_ws.git](https://github.com/bangrbt/ur3_ws.git)
