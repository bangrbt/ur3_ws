# Hướng Dẫn Sử Dụng Package `my_ur3_draw` (Robot UR3 Vẽ Chữ)

Package `my_ur3_draw` cung cấp giải pháp điều khiển cánh tay robot công nghiệp Universal Robots **UR3** (và các dòng UR khác trong ROS 2 & MoveIt) thực hiện tác vụ **vẽ chữ trên mặt phẳng bảng đứng**.

Package hỗ trợ đầy đủ 2 chức năng chính:
1. **Chức năng 1 (Chức năng gốc):** Vẽ theo biên dạng ảnh (mặc định là `letter.png` hoặc ảnh nhị phân tùy chọn).
2. **Chức năng 2 (Chức năng mới):** Nhập chữ cái / từ ngữ bất kỳ (A-Z, chữ số, chuỗi ký tự) để tay máy tự động tính toán và vẽ ra chữ tương ứng.

---

## Mục Lục
- [1. Yêu Cầu Hệ Thống & Biên Dịch](#1-yêu-cầu-hệ-thống--biên-dịch)
- [2. Chi Tiết Chức Năng 1: Vẽ Theo Ảnh (letter.png)](#2-chi-tiết-chức-năng-1-vẽ-theo-ảnh-letterpng)
  - [2.1. Cách 1: Khởi động trọn gói 1 lệnh (Gazebo + MoveIt + RViz + Vẽ)](#21-cách-1-khởi-động-trọn-gói-1-lệnh-gazebo--moveit--rviz--vẽ)
  - [2.2. Cách 2: Chạy riêng biệt (Mô phỏng ở Terminal 1, Vẽ ở Terminal 2)](#22-cách-2-chạy-riêng-biệt-mô-phỏng-ở-terminal-1-vẽ-ở-terminal-2)
  - [2.3. Cách 3: Vẽ từ file ảnh tùy chỉnh khác](#23-cách-3-vẽ-từ-file-ảnh-tùy-chỉnh-khác)
- [3. Chi Tiết Chức Năng 2: Nhập Chữ Cái Bất Kỳ Để Vẽ](#3-chi-tiết-chức-năng-2-nhập-chữ-cái-bất-kỳ-để-vẽ)
  - [3.1. Cách 1: Khởi động trọn gói 1 lệnh kèm chữ cái cần vẽ](#31-cách-1-khởi-động-trọn-gói-1-lệnh-kèm-chữ-cái-cần-vẽ)
  - [3.2. Cách 2: Truyền chữ cái qua Launch File (khi mô phỏng đã mở)](#32-cách-2-truyền-chữ-cái-qua-launch-file-khi-mô-phỏng-đã-mở)
  - [3.3. Cách 3: Nhập tương tác trực tiếp từ bàn phím Console](#33-cách-3-nhập-tương-tác-trực-tiếp-từ-bàn-phím-console)
  - [3.4. Tùy chỉnh kích thước chữ vẽ](#34-tùy-chỉnh-kích-thước-chữ-vẽ)
- [4. Bảng Tham Số (Parameters & Launch Arguments)](#4-bảng-tham-số-parameters--launch-arguments)
- [5. Hướng Dẫn Hiển Thị Nét Vẽ Trong RViz](#5-hướng-dẫn-hiển-thị-nét-vẽ-trong-rviz)
- [6. Xử Lý Sự Cố Thường Gặp (Troubleshooting)](#6-xử-lý-sự-cố-thường-gặp-troubleshooting)

---

## 1. Yêu Cầu Hệ Thống & Biên Dịch

### 1.1. Lệnh Dọn Dẹp Tiến Trình Treo (Khuyên dùng chạy trước mỗi lần mô phỏng):
```bash
killall -9 ruby ign gzserver gzclient rviz2 2>/dev/null || pkill -9 -f "ign gazebo"
```

### 1.2. Biên Dịch Workspace:
Mở Terminal và điều hướng về workspace `ur3_ws`:
```bash
cd ~/ur3_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select my_ur3_draw
source install/setup.bash
```

---

## 2. Chi Tiết Chức Năng 1: Vẽ Theo Ảnh (`letter.png`)

> **Mô tả:** Robot chuyển về tư thế viết bảng (`ready_pose`), đọc file ảnh `letter.png` (ảnh chữ B đen trên nền trắng), sử dụng OpenCV để quét tất cả đường viền khép kín (cả viền ngoài lẫn các lỗ khuyết bên trong), tự động nhấc bút (`hover_pose`) và hạ bút (`draw_pose`), di chuyển theo quỹ đạo Cartesian và quay về trạng thái `home_pose`.

### 2.1. Cách 1: Khởi động trọn gói 1 lệnh (Gazebo + MoveIt + RViz + Vẽ)
Sử dụng file launch `ur3_draw_sim.launch.py`. Lệnh này sẽ tự động bật toàn bộ hệ thống mô phỏng, chờ các bộ điều khiển sẵn sàng và điều khiển robot vẽ ảnh mặc định:
```bash
killall -9 ruby ign gzserver gzclient rviz2 2>/dev/null || pkill -9 -f "ign gazebo"
source /opt/ros/humble/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 launch my_ur3_draw ur3_draw_sim.launch.py
```

### 2.2. Cách 2: Chạy riêng biệt (Mô phỏng ở Terminal 1, Vẽ ở Terminal 2)
Nếu bạn muốn giữ mô phỏng chạy liên tục để thử nhiều lần:

- **Terminal 1 (Bật mô phỏng & MoveIt):**
  ```bash
  source ~/ur3_ws/install/setup.bash
  ros2 launch ur_simulation_gz ur_sim_moveit.launch.py ur_type:=ur3
  ```

- **Terminal 2 (Gửi lệnh vẽ ảnh mặc định):**
  ```bash
  source ~/ur3_ws/install/setup.bash
  ros2 launch my_ur3_draw draw_letter.launch.py
  ```

### 2.3. Cách 3: Vẽ từ file ảnh tùy chỉnh khác
Bạn có thể chỉ định đường dẫn tới một file ảnh bất kỳ trên máy tính qua tham số `image_path`:
```bash
ros2 launch my_ur3_draw draw_letter.launch.py image_path:=/path/to/your_image.png
```

---

## 3. Chi Tiết Chức Năng 2: Nhập Chữ Cái Bất Kỳ Để Vẽ

> **Mô tả:** Bạn có thể nhập bất kỳ chữ cái nào (A, B, C, ..., Z), chữ số hoặc một từ bất kỳ (ví dụ: `UR3`, `ROBOT`). Hệ thống sẽ tự động render ký tự với tỷ lệ và nét đậm tối ưu, căn giữa khung vẽ, trích xuất biên dạng waypoint Cartesian và điều khiển tay máy UR3 vẽ chính xác chữ cái đó.

### 3.1. Cách 1: Khởi động trọn gói 1 lệnh kèm chữ cái cần vẽ
Chỉ với **1 câu lệnh duy nhất**, toàn bộ mô phỏng Gazebo, RViz và MoveIt sẽ khởi động, sau đó UR3 sẽ tự động vẽ chữ cái bạn chỉ định:

- Vẽ chữ **A**:
  ```bash
  source ~/ur3_ws/install/setup.bash
  ros2 launch my_ur3_draw ur3_draw_sim.launch.py letter:=A
  ```

- Vẽ chữ **B**:
  ```bash
  ros2 launch my_ur3_draw ur3_draw_sim.launch.py letter:=B
  ```

- Vẽ chữ **C**:
  ```bash
  ros2 launch my_ur3_draw ur3_draw_sim.launch.py letter:=C
  ```

- Vẽ từ bất kỳ (ví dụ: **UR3**):
  ```bash
  ros2 launch my_ur3_draw ur3_draw_sim.launch.py letter:=UR3
  ```

### 3.2. Cách 2: Truyền chữ cái qua Launch File (khi mô phỏng đã mở)
Khi mô phỏng Gazebo/MoveIt đang chạy sẵn ở Terminal 1, tại Terminal 2 bạn chỉ cần gọi:
```bash
source ~/ur3_ws/install/setup.bash
ros2 launch my_ur3_draw draw_letter.launch.py letter:=A
```
*(Thay `A` bằng bất kỳ ký tự nào bạn muốn: `H`, `M`, `N`, `O`, `X`, ...)*.

### 3.3. Cách 3: Nhập tương tác trực tiếp từ bàn phím Console
Node hỗ trợ chế độ hỏi tương tác người dùng qua bàn phím khi chạy trực tiếp qua `ros2 run`:

```bash
source ~/ur3_ws/install/setup.bash
ros2 run my_ur3_draw draw_letter_node
```

Giao diện console sẽ hiển thị lời mời nhập:
```text
======================================================
  UR3 DRAW LETTER - DIEU KHIEN VE TAY MAY ROBOT
======================================================
Nhap chu cai bat ky muon ve (vi du: A, B, C, UR3, ...)
Hoac nhan ENTER de giu nguyen ve tu file letter.png: 
```

- **Nếu bạn nhập chữ cái** (ví dụ gõ `A` rồi ấn Enter): Robot sẽ tính toán và vẽ chữ `A`.
- **Nếu bạn chỉ bấm Enter** (không gõ gì): Robot sẽ tự động chuyển về vẽ ảnh `letter.png` mặc định.

### 3.4. Tùy chỉnh kích thước chữ vẽ
Độ rộng mặc định của chữ là `0.15m` (15 cm). Bạn có thể tăng giảm kích thước vẽ tùy ý thông qua tham số `target_width` (đơn vị: mét):
```bash
# Vẽ chữ A với độ rộng 20cm
ros2 launch my_ur3_draw draw_letter.launch.py letter:=A target_width:=0.20

# Hoặc trong chế độ sim trọn gói:
ros2 launch my_ur3_draw ur3_draw_sim.launch.py letter:=A target_width:=0.12
```

---

## 4. Bảng Tham Số (Parameters & Launch Arguments)

| Tham Số | Kiểu Dữ Liệu | Mặc Định | Ý Nghĩa / Mô Tả |
| :--- | :---: | :---: | :--- |
| `letter` | `string` | `""` | Ký tự hoặc từ ngữ muốn vẽ (ví dụ: `'A'`, `'B'`, `'UR3'`). Để trống sẽ vẽ theo file ảnh. |
| `image_path` | `string` | `""` | Đường dẫn file ảnh. Để trống sẽ tự động trỏ tới `letter.png` trong package. |
| `target_width` | `double` | `0.15` | Chiều rộng vùng vẽ của robot trên mặt phẳng (đơn vị: mét). |
| `interactive` | `bool` | `false` | Bật/tắt chế độ hỏi người dùng nhập ký tự qua bàn phím console. |
| `start_sim` | `bool` | `false` | Khởi động đồng thời cả mô phỏng Gazebo UR3 và MoveIt (`draw_letter.launch.py`). |
| `ur_type` | `string` | `ur3` | Loại tay máy robot UR (`ur3`, `ur3e`, `ur5`, `ur5e`, `ur10`, ...). |
| `start_rviz` | `bool` | `true` | Mở giao diện RViz khi `start_sim=true`. |
| `start_delay` | `double` | `20.0` | Thời gian chờ (giây) để Gazebo + MoveIt nạp controller xong trước khi robot bắt đầu vẽ. |

---

## 5. Hiển Thị Nét Vẽ Tự Động Trong RViz

Giao diện RViz đi kèm launch file (`draw_letter.rviz`) **đã được cấu hình sẵn hoàn toàn**, bao gồm:
- **`RobotModel` & `MotionPlanning`:** Hiển thị cánh tay robot UR3 và trạng thái chuyển động.
- **`DrawingMarker`:** Tự động lắng nghe topic `/visualization_marker` với namespace `realtime_path`.

👉 **Bạn KHÔNG cần phải thêm thủ công Marker hay topic nào cả!** Ngay khi file launch chạy lên, RViz sẽ tự động bắt đầu và hiển thị trực tiếp các nét vẽ màu đỏ theo thời gian thực khi tay máy chuyển động.

---

## 6. Xử Lý Sự Cố Thường Gặp (Troubleshooting)

1. **Lỗi `Could not find a package configuration file provided by "ament_cmake"` khi build:**
   - *Khắc phục:* Nhớ `source /opt/ros/humble/setup.bash` trước khi chạy `colcon build`.

2. **Lỗi MoveIt `Fraction < 0.9` (Khong the lap ke hoach):**
   - *Nguyên nhân:* Kích thước chữ vẽ (`target_width`) quá lớn vượt khỏi tầm với (workspace) của robot UR3, hoặc tư thế bị điểm kỳ dị (singularity).
   - *Khắc phục:* Giảm kích thước vẽ xuống, ví dụ `target_width:=0.12` hoặc `target_width:=0.15`.

3. **Khi chạy trọn gói (`ur3_draw_sim.launch.py`) mà node báo lỗi không kết nối được `move_group`:**
   - *Nguyên nhân:* Máy tính khởi động Gazebo và MoveIt chậm hơn 20 giây mặc định.
   - *Khắc phục:* Tăng thời gian chờ qua tham số `start_delay`:
     ```bash
     ros2 launch my_ur3_draw ur3_draw_sim.launch.py letter:=A start_delay:=30.0
     ```

