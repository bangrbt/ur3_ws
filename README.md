# Workspace `ur3_ws` - Universal Robots UR3 Vẽ Chữ (ROS 2 Humble)

Workspace ROS 2 điều khiển cánh tay robot công nghiệp **Universal Robots UR3** thực hiện tác vụ **vẽ chữ trên mặt phẳng bảng đứng** sử dụng **MoveIt 2**, **Ignition Gazebo (Gazebo Fortress)** và hiển thị nét vẽ theo thời gian thực trên **RViz 2**.

---

## 📌 Mục Lục
1. [Bố Trí Thư Mục & Cấu Trúc Nhánh Git](#1-bố-trí-thư-mục--cấu-trúc-nhánh-git)
2. [Lệnh Dọn Dẹp Tiến Trình Treo (Zombie Cleanup)](#2-lệnh-dọn-dẹp-tiến-trình-treo-zombie-cleanup)
3. [Hướng Dẫn Cài Đặt Từ A Đến Z (Dành Cho Người Mới)](#3-hướng-dẫn-cài-đặt-từ-a-đến-z-dành-cho-người-mới)
4. [Chi Tiết Chức Năng 1: Vẽ Theo Ảnh (`letter.png`)](#4-chi-tiết-chức-năng-1-vẽ-theo-ảnh-letterpng)
5. [Chi Tiết Chức Năng 2: Nhập Chữ Cái Bất Kỳ Để Vẽ](#5-chi-tiết-chức-năng-2-nhập-chữ-cái-bất-kỳ-để-vẽ)
6. [Các Điểm Tối Ưu Kỹ Thuật Đáng Chú Ý](#6-các-điểm-tối-ưu-kỹ-thuật-đáng-chú-ý)
7. [Bảng Tham Số Điều Khiển (Parameters & Launch Arguments)](#7-bảng-tham-số-điều-khiển-parameters--launch-arguments)
8. [Xử Lý Sự Cố Thường Gặp (Troubleshooting)](#8-xử-lý-sự-cố-thường-gặp-troubleshooting)

---

## 1. Bố Trí Thư Mục & Cấu Trúc Nhánh Git

### 1.1. Cấu trúc thư mục Workspace
```text
ur3_ws/
├── .git/                            # Git repository của workspace
├── .gitignore                       # Loại trừ build/, install/, log/, .vscode/
├── .gitmodules                      # Khai báo submodule cho driver & simulation UR
├── README.md                        # Tài liệu hướng dẫn chung toàn workspace
└── src/
    ├── my_ur3_draw/                 # [BÀI TẬP TUẦN 1] Package robot UR3 vẽ chữ
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── letter.png               # Ảnh mẫu chữ B mặc định
    │   ├── launch/
    │   │   ├── draw_letter.launch.py   # Launch node vẽ kèm nạp kinematics/RViz
    │   │   └── ur3_draw_sim.launch.py  # Launch trọn gói cả Gazebo + MoveIt + Vẽ
    │   ├── rviz/
    │   │   └── draw_letter.rviz        # Cấu hình RViz có sẵn Marker nét vẽ
    │   ├── src/
    │   │   └── draw_letter_node.cpp    # Mã nguồn C++ thuật toán xử lý ảnh & MoveIt
    │   └── README.md
    │
    ├── my_ur3_tuan_2/               # [BÀI TẬP TUẦN 2] (Các tuần sau tạo tại đây, cùng hàng)
    ├── my_ur3_tuan_3/               # [BÀI TẬP TUẦN 3]
    │
    ├── Universal_Robots_ROS2_Driver/        # Driver điều khiển UR chính hãng (Submodule)
    └── Universal_Robots_ROS2_GZ_Simulation/ # Mô phỏng UR Ignition Gazebo (Submodule)
```

> **Quy tắc mở rộng các tuần sau:**
> - Các bài tập của tuần sau sẽ được tạo thành các package ROS 2 mới **nằm cùng hàng (cùng cấp thư mục)** bên trong thư mục `src/` (ví dụ: `src/my_ur3_tuan_2`, `src/my_ur3_tuan_3`,...).
> - Hai thư mục driver và simulation được quản lý bằng **Git Submodule**, người khác khi clone về chỉ cần dùng cờ `--recurse-submodules` là có đầy đủ toàn bộ môi trường.

---

### 1.2. Cấu trúc các nhánh Git (Git Branches)

Kho chứa được phân nhánh khoa học theo từng tuần bài tập:
- **`main`**: Nhánh chính chứa toàn bộ khung workspace chuẩn.
- **`bai-tap-tuan-1`** (hoặc **`tuan-1`**): Nhánh riêng cho Bài tập tuần 1 (Package `my_ur3_draw`).
- **`bai-tap-tuan-2`**: Nhánh cho Bài tập tuần 2.
- **`bai-tap-tuan-3`**: Nhánh cho Bài tập tuần 3...

#### Lệnh thao tác nhánh Git:
```bash
# Xem danh sách các nhánh:
git branch -a

# Chuyển sang nhánh bài tập tuần 1:
git checkout bai-tap-tuan-1

# Khi làm bài tập tuần 2, tạo nhánh mới từ nhánh main:
git checkout main
git checkout -b bai-tap-tuan-2
```

---

## 2. Lệnh Dọn Dẹp Tiến Trình Treo (Zombie Cleanup)

> ⚠️ **LƯU Ý CỰC KỲ QUAN TRỌNG:**
> Khi tắt cửa sổ mô phỏng Gazebo hoặc RViz bằng tổ hợp phím `Ctrl + C` hoặc bấm nút tắt `X`, các tiến trình nền của Ignition Gazebo (`ign gazebo`, `ruby`, `gzserver`) và `rviz2` **vẫn có thể chạy ngầm trong hệ thống**. 
> Điều này sẽ gây ra hàng loạt lỗi khi bạn khởi động lại:
> - `[spawner-2] Controller already loaded, skipping load_controller`
> - `[spawner-3] Failed to configure controller`
> - `[ign gazebo-5] Unknown message type [9]`

Trước khi chạy bất kỳ lệnh mô phỏng mới nào, **hãy luôn chạy lệnh dọn dẹp một chạm sau**:

```bash
killall -9 ruby ign gzserver gzclient rviz2 2>/dev/null || pkill -9 -f "ign gazebo"
```

Lệnh này sẽ quét sạch toàn bộ tiến trình mô phỏng còn sót lại, đưa hệ thống về trạng thái sạch 100%.

---

## 3. Hướng Dẫn Cài Đặt Từ A Đến Z (Dành Cho Người Mới)

### Bước 1: Chuẩn bị môi trường
- Hệ điều hành: **Ubuntu 22.04 LTS (Jammy Jellyfish)**
- Phiên bản ROS: **ROS 2 Humble Hawksbill**
- Đã cài đặt MoveIt 2 và Ignition Gazebo (Fortress).

### Bước 2: Clone repository kèm Submodule
Sử dụng cờ `--recurse-submodules` để tự động kéo cả package bài tập lẫn hai repository driver và mô phỏng của Universal Robots:

```bash
git clone --recurse-submodules https://github.com/bangrbt/ur3_ws.git ~/ur3_ws
cd ~/ur3_ws
```

*(Nếu đã lỡ clone thông thường không có cờ trên, bạn chỉ cần chạy thêm: `git submodule update --init --recursive`)*.

### Bước 3: Cài đặt các gói phụ thuộc (Dependencies)
```bash
cd ~/ur3_ws
sudo apt update
rosdep update
rosdep install --ignore-src --from-paths src -y -r
```

### Bước 4: Biên dịch Workspace
```bash
cd ~/ur3_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select my_ur3_draw
source install/setup.bash
```

---

## 4. Chi Tiết Chức Năng 1: Vẽ Theo Ảnh (`letter.png`)

> **Mô tả:** Robot chuyển về tư thế viết bảng (`ready_pose`), đọc file ảnh `letter.png` (ảnh chữ B đen trên nền trắng), sử dụng OpenCV quét tất cả đường viền khép kín (cả viền ngoài và các lỗ rỗng bên trong), nhấc bút lùi 8cm trên không (`HOVER_OFFSET`) rồi hạ bút vẽ phẳng trên bảng đứng, tự động vẽ nét mực đỏ trong RViz theo thời gian thực và quay về trạng thái `home_pose`.

### Cách 1.1: Khởi động trọn gói 1 lệnh (Gazebo + MoveIt + RViz + Vẽ) — *Khuyên dùng*
Chỉ với 1 câu lệnh, toàn bộ hệ thống sẽ tự khởi động:

```bash
# Luôn dọn dẹp trước khi chạy:
killall -9 ruby ign gzserver gzclient rviz2 2>/dev/null || pkill -9 -f "ign gazebo"

# Chạy mô phỏng và vẽ ảnh mặc định:
source /opt/ros/humble/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 launch my_ur3_draw ur3_draw_sim.launch.py
```

### Cách 1.2: Chạy riêng biệt (Mô phỏng ở Terminal 1, Gửi lệnh vẽ ở Terminal 2)
Phù hợp khi bạn muốn giữ mô phỏng Gazebo chạy cố định để thử nghiệm vẽ nhiều lần mà không cần khởi động lại từ đầu:

- **Terminal 1 (Khởi động Gazebo & MoveIt):**
  ```bash
  killall -9 ruby ign gzserver gzclient rviz2 2>/dev/null || pkill -9 -f "ign gazebo"
  source /opt/ros/humble/setup.bash
  source ~/ur3_ws/install/setup.bash
  ros2 launch ur_simulation_gz ur_sim_moveit.launch.py ur_type:=ur3
  ```

- **Terminal 2 (Gửi lệnh vẽ):**
  ```bash
  source /opt/ros/humble/setup.bash
  source ~/ur3_ws/install/setup.bash
  ros2 launch my_ur3_draw draw_letter.launch.py
  ```

### Cách 1.3: Vẽ từ một file ảnh tùy chỉnh khác
Bạn có thể truyền đường dẫn tới bất kỳ file ảnh nhị phân đen trắng nào trên máy tính qua tham số `image_path`:
```bash
ros2 launch my_ur3_draw draw_letter.launch.py image_path:=/duong_dan_toi_file/anh_cua_ban.png
```

---

## 5. Chi Tiết Chức Năng 2: Nhập Chữ Cái Bất Kỳ Để Vẽ

> **Mô tả:** Người dùng nhập chữ cái bất kỳ (ví dụ: `A`, `B`, `C`, `X`,...) hoặc cả một từ (ví dụ: `UR3`, `ROBOT`). Hệ thống OpenCV sẽ tự động render chữ cái căn giữa với độ đậm nét tối ưu, trích xuất danh sách các contour khép kín và điều khiển cánh tay UR3 vẽ chính xác từng nét.

### Cách 2.1: Khởi động trọn gói 1 lệnh kèm chữ cái cần vẽ — *Nhanh nhất*

- **Vẽ chữ `A`:**
  ```bash
  killall -9 ruby ign gzserver gzclient rviz2 2>/dev/null || pkill -9 -f "ign gazebo"
  source /opt/ros/humble/setup.bash
  source ~/ur3_ws/install/setup.bash
  ros2 launch my_ur3_draw ur3_draw_sim.launch.py letter:=A
  ```

- **Vẽ chữ `B` (tự động vẽ viền ngoài trước, sau đó nhấc bút vẽ 2 lỗ rỗng bên trong):**
  ```bash
  killall -9 ruby ign gzserver gzclient rviz2 2>/dev/null || pkill -9 -f "ign gazebo"
  source /opt/ros/humble/setup.bash
  source ~/ur3_ws/install/setup.bash
  ros2 launch my_ur3_draw ur3_draw_sim.launch.py letter:=B
  ```

- **Vẽ chữ `C`:**
  ```bash
  killall -9 ruby ign gzserver gzclient rviz2 2>/dev/null || pkill -9 -f "ign gazebo"
  source /opt/ros/humble/setup.bash
  source ~/ur3_ws/install/setup.bash
  ros2 launch my_ur3_draw ur3_draw_sim.launch.py letter:=C
  ```

- **Vẽ từ bất kỳ (ví dụ `UR3`):**
  ```bash
  killall -9 ruby ign gzserver gzclient rviz2 2>/dev/null || pkill -9 -f "ign gazebo"
  source /opt/ros/humble/setup.bash
  source ~/ur3_ws/install/setup.bash
  ros2 launch my_ur3_draw ur3_draw_sim.launch.py letter:=UR3
  ```

---

### Cách 2.2: Truyền chữ cái qua Launch File (khi mô phỏng đang mở sẵn)
Tại Terminal 2, chỉ cần truyền thêm đối số `letter`:
```bash
ros2 launch my_ur3_draw draw_letter.launch.py letter:=B
```

---

### Cách 2.3: Nhập tương tác trực tiếp từ bàn phím Console
Node hỗ trợ hiển thị lời mời nhập trực tiếp trên màn hình terminal:

```bash
ros2 run my_ur3_draw draw_letter_node
```
Màn hình terminal sẽ xuất hiện:
```text
======================================================
  UR3 DRAW LETTER - DIEU KHIEN VE TAY MAY ROBOT
======================================================
Nhap chu cai bat ky muon ve (vi du: A, B, C, UR3, ...)
Hoac nhan ENTER de giu nguyen ve tu file letter.png: 
```
- **Nếu nhập ký tự** (ví dụ gõ `A` rồi ấn Enter): Robot sẽ vẽ chữ `A`.
- **Nếu nhấn Enter ngay**: Robot tự động vẽ file ảnh `letter.png` mặc định.

---

### Cách 2.4: Tùy chỉnh kích thước chữ vẽ
Độ rộng mặc định là `0.15m` (15 cm). Bạn có thể điều chỉnh qua tham số `target_width` (đơn vị: mét):
```bash
# Vẽ chữ B rộng 12cm (tránh vượt tầm với workspace):
ros2 launch my_ur3_draw ur3_draw_sim.launch.py letter:=B target_width:=0.12

# Vẽ chữ A lớn 20cm:
ros2 launch my_ur3_draw ur3_draw_sim.launch.py letter:=A target_width:=0.20
```

---

## 6. Các Điểm Tối Ưu Kỹ Thuật Đáng Chú Ý

1. **Hiển thị nét vẽ RViz tự động 100%**:
   - File cấu hình [`draw_letter.rviz`](src/my_ur3_draw/rviz/draw_letter.rviz) đã tích hợp sẵn display **`DrawingMarker`** lắng nghe topic `/visualization_marker` với namespace `realtime_path`.
   - **Người dùng không cần thao tác thêm thủ công bất kỳ Marker hay Topic nào**. Khi chạy file launch, nét vẽ màu đỏ dày 8mm sẽ tự động vẽ theo cánh tay robot trong RViz.

2. **Bỏ hoàn toàn chữ cái mẫu đồ theo (`planned_guide`)**:
   - Loại bỏ các đường viền cyan đứt đoạn xem trước theo đúng yêu cầu, chỉ hiển thị duy nhất vệt mực thực tế do đầu bút robot vẽ ra.

3. **Triệt tiêu hoàn toàn nét thừa khi nhấc bút vẽ nét mới**:
   - **Tách từng contour thành chu trình độc lập**: Robot di chuyển theo chu trình:
     $$\text{Hover (lùi 8cm trên không)} \longrightarrow \text{Hạ bút vuông góc} \longrightarrow \text{Vẽ viền trên bảng} \longrightarrow \text{Nhấc bút vuông góc lùi ra 8cm} \longrightarrow \text{Dừng hẳn vận tốc = 0}$$
   - **Cờ khóa hoàn thành nét vẽ (`stroke_completed_for_contour`)**: Ngay khi robot nhấc bút lùi quá 3mm so với mặt bảng, nét vẽ lập tức được chốt và đóng lại. Thread vẽ bị khóa hoàn toàn, không ghi nhận bất kỳ điểm nào trong lúc cánh tay lùi lại và bay trên không sang nét tiếp theo.
   - **Mặt phẳng nét vẽ phẳng tuyệt đối**: Tọa độ $X$ của nét vẽ luôn được cố định trên mặt phẳng bảng, loại bỏ hiện tượng răng cưa hay lệch trục 3D.

4. **Sắp xếp thứ tự vẽ tự nhiên**:
   - Sử dụng cây phân cấp `cv::RETR_TREE` kết hợp bounding box: chữ viết được vẽ tuần tự từ trái sang phải, nét bao ngoài vẽ trước, các lỗ rỗng bên trong vẽ sau.

---

## 7. Bảng Tham Số Điều Khiển (Parameters & Launch Arguments)

| Tham Số | Kiểu Dữ Liệu | Mặc Định | Ý Nghĩa / Tác Dụng |
| :--- | :---: | :---: | :--- |
| `letter` | `string` | `""` | Ký tự hoặc từ ngữ muốn vẽ (ví dụ: `'A'`, `'B'`, `'UR3'`). Để trống sẽ vẽ theo file ảnh. |
| `image_path` | `string` | `""` | Đường dẫn file ảnh. Để trống sẽ tự động trỏ tới `letter.png` trong package. |
| `target_width` | `double` | `0.15` | Chiều rộng vùng vẽ của robot trên mặt phẳng đứng (đơn vị: mét, mặc định 15cm). |
| `interactive` | `bool` | `false` | Bật/tắt chế độ hỏi người dùng nhập ký tự qua bàn phím terminal console. |
| `start_sim` | `bool` | `false` | Tự động khởi động đồng thời cả Gazebo UR3 và MoveIt trong file `draw_letter.launch.py`. |
| `ur_type` | `string` | `ur3` | Dòng tay máy UR muốn sử dụng (`ur3`, `ur3e`, `ur5`, `ur5e`, `ur10`,...). |
| `start_rviz` | `bool` | `true` | Tự động mở giao diện RViz đã cấu hình sẵn nét vẽ. |
| `start_delay` | `double` | `20.0` | Thời gian chờ (giây) để Gazebo + MoveIt nạp controller ổn định trước khi robot bắt đầu vẽ. |

---

## 8. Xử Lý Sự Cố Thường Gặp (Troubleshooting)

### 1. Lỗi Controller already loaded hoặc Failed to configure controller
- **Nguyên nhân:** Có tiến trình Gazebo/RViz cũ bị treo ngầm chiếm tài nguyên.
- **Khắc phục:** Chạy lệnh dọn dẹp:
  ```bash
  killall -9 ruby ign gzserver gzclient rviz2 2>/dev/null || pkill -9 -f "ign gazebo"
  ```

### 2. Lỗi MoveIt `Fraction < 0.85` (Không thể lập kế hoạch quỹ đạo)
- **Nguyên nhân:** Kích thước chữ vẽ (`target_width`) quá lớn vượt khỏi vùng làm việc (workspace reach) của UR3.
- **Khắc phục:** Giảm `target_width` xuống mức an toàn, ví dụ `target_width:=0.12`.

### 3. Khi chạy trọn gói (`ur3_draw_sim.launch.py`) mà node báo chưa sẵn sàng controller
- **Nguyên nhân:** Máy tính cấu hình yếu hoặc lần đầu mở Gazebo mất hơn 20 giây để nạp xong mô hình 3D.
- **Khắc phục:** Tăng thời gian chờ khởi động thông qua đối số `start_delay`:
  ```bash
  ros2 launch my_ur3_draw ur3_draw_sim.launch.py letter:=A start_delay:=30.0
  ```

---

## 👨‍💻 Tác Giả & Bản Quyền
- **Workspace:** `ur3_ws` - Universal Robots UR3 ROS 2 Humble Project
- **GitHub:** [https://github.com/bangrbt](https://github.com/bangrbt)
