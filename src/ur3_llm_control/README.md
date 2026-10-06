# Package `ur3_llm_control` - LLM Skill Planning với Gripper và Camera

> **Báo cáo Thực hành Bài 3 - Môn học: Xử lý ảnh và thị giác Robot**  
> **Sinh viên thực hiện:** Lê Anh Tuấn Bằng  
> **Mã số sinh viên (MSSV):** `23020723`  
> **Nhánh Git bài tập:** `assignments_3`  

---

Báo cáo kiểm thử và giải thích thiết kế: [REPORT_LAB03.md](REPORT_LAB03.md).

## 1. Tổng quan & Kiến trúc Hệ thống

Bài thực hành 03 nâng cấp từ Bài 02 với các bổ sung cốt lõi:
1. **Gripper vật lý cử động thực tế:** Tay kẹp 2 ngón dùng khớp trượt và `JointPositionController`. Khi hai ngón bao quanh cube, hệ thống tạo `DetachableJoint` trong Gazebo để giữ vật. Khi đặt, robot chờ vật ổn định trên bàn, tháo joint rồi mở ngón từ từ để tránh đẩy văng cube. PlanningScene của MoveIt được cập nhật riêng; các skill gắp/đặt không dùng `set_pose`.
2. **Camera RGB & Node Thị giác máy tính (`camera_perception`):** Camera góc nhìn từ trên xuống (overhead camera), dùng OpenCV HSV Color Segmentation và mô hình Pinhole Camera ngược để phát hiện tọa độ thế giới của toàn bộ 5 khối hộp và trạng thái chiếm giữ của các vùng trong thời gian thực.
3. **Môi trường 5 Khối & 3 Vùng đích + Vùng đệm tạm:**
   - 5 khối: `red_cube`, `yellow_cube`, `blue_cube`, `green_cube`, `purple_cube`.
   - 3 vùng đích: `zone_a`, `zone_b`, `zone_c`.
   - Các vùng đệm tạm: `zone_temp_1`, `zone_temp_2`, `zone_temp_3`.
4. **Giải quyết xung đột vùng đích (Conflict Resolution):** Khi người dùng yêu cầu đưa vật vào vùng đã có vật khác chiếm giữ, hệ thống tự động kiểm tra trạng thái môi trường từ camera, di chuyển vật cản sang vị trí tạm thích hợp trước khi thực hiện hành động chính.

Kiến trúc phân tầng chuẩn:
```
Natural Language Command (Người dùng nhập câu lệnh)
        ↓
    LLM Planner (9Router / Gemini 3.5 Flash-lite / OpenAI)
        ↓
 Structured Plan (Kế hoạch JSON với chuỗi Robot Skills)
        ↓
   Plan Validator (Kiểm tra Whitelist Skills, 5 Objects, Zones)
        ↓
   Skill Executor (Điều phối thực thi & in log chuẩn hóa)
        ↓
   Robot Skills (Pick, Place, Check Zone, Detect Objects, ...)
        ↓
      MoveIt 2 (Lập kế hoạch quỹ đạo & Tránh va chạm)
        ↓
 UR3/UR3e + Gripper (Mô phỏng vật lý chân thực trong Ignition Gazebo)
        ↑
  Camera Perception (OpenCV nhận diện 5 màu & vị trí thời gian thực)
```

---

## 2. Cá nhân hóa theo Mã số Sinh viên (MSSV: 23020723)

Theo quy ước bài tập:
$$XX = 23 \implies P = 23 \pmod 6 = 5$$

Ánh xạ vùng đích tương ứng với $P = 5$:
* **Vùng A (`zone_a`):** Khối màu xanh lam (**`blue_cube`**)
* **Vùng B (`zone_b`):** Khối màu vàng (**`yellow_cube`**)
* **Vùng C (`zone_c`):** Khối màu đỏ (**`red_cube`**)

---

## 3. Hướng dẫn cấu hình kết nối 9Router (LLM Gateway)

Trước khi launch, cung cấp khóa bằng biến môi trường trong terminal chạy ROS:
```bash
export NINE_ROUTER_API_KEY="<khoa-cua-ban>"
```
Không ghi khóa vào YAML hoặc commit lên Git. Khóa cũ đã từng nằm trong repository cần được thay mới.

### 9Router là gì?
**9Router** là cổng API trung gian (API Gateway) tương thích chuẩn OpenAI REST API v1, cho phép kết nối trực tiếp từ local tới các mô hình ngôn ngữ lớn (như Google Gemini, OpenAI GPT) qua một endpoint duy nhất.

### Cấu hình Online 100% (Khuyến nghị):
1. Khởi chạy 9Router trên terminal máy:
   ```bash
   npx 9router
   ```
   Gateway sẽ lắng nghe tại `http://localhost:20128/v1`.
2. File cấu hình `config/llm_config.yaml` đã được thiết lập sẵn sàng:
   ```yaml
   base_url: "http://localhost:20128/v1"
   api_key: ""  # cung cap bang NINE_ROUTER_API_KEY
   model: "gemini/gemini-3.5-flash-lite"
   fallback_to_smart_planner: false  # Chạy 100% Online LLM
   ```
3. Mô hình **`gemini/gemini-3.5-flash-lite`** được cấu hình để phân tích câu lệnh tiếng Anh và tiếng Việt, hỗ trợ cả JSON và SSE streaming chunks.
4. **Cảnh báo mất kết nối:** Nếu 9Router bị tắt hoặc mất mạng, hệ thống sẽ hiện thông báo cảnh báo rõ ràng trên console để bạn dễ dàng kiểm tra.

---

## 4. Hướng dẫn Khởi chạy Hệ thống

### Bước 1: Build Workspace (nếu vừa clone về)
```bash
cd ~/ur3_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ur3_llm_control
source install/setup.bash
```

### Bước 2: Khởi chạy Trọn gói 1 Lệnh (Gazebo + MoveIt 2 + RViz + LLM)
```bash
source ~/ur3_ws/install/setup.bash
ros2 launch ur3_llm_control llm_robot.launch.py
```

*Hệ thống sẽ tự động:*
1. Mở Gazebo với bàn thao tác, 5 block, 3 zone đích và 3 vùng đệm riêng biệt.
2. Mở cánh tay robot UR3 đã gắn tay kẹp cơ khí 2 ngón (`Gripper`).
3. Khởi động MoveIt 2 và mở RViz với các 3D Marker trực quan.
4. Mở cửa sổ giao diện dòng lệnh tương tác trên Terminal để bạn nhập lệnh tự nhiên!

---

## 5. Các Kịch bản Kiểm thử Thực nghiệm

### Kịch bản 1: Mức Cơ bản (Đơn vật thể)
Bạn có thể nhập một trong các câu lệnh sau tại terminal:
* Tiếng Anh:
  ```text
  Please put the red cube in zone B.
  ```
  hoặc:
  ```text
  Move the blue cube to zone A.
  ```
* Tiếng Việt:
  ```text
  Hãy lấy khối màu vàng và đặt nó vào ô C.
  ```
  hoặc:
  ```text
  Đưa khối màu đỏ vào vùng B.
  ```

**Kết quả hiển thị trên Terminal chuẩn form:**
```text
USER COMMAND:
  Put the red cube in zone B.
-----------------------------------------------------------------
LLM PLAN:
    pick(red_cube)
    place(red_cube, zone_b)
    home()
-----------------------------------------------------------------
EXECUTION:
pick(red_cube) ............. SUCCESS
place(red_cube, zone_b) .... SUCCESS
home() ..................... SUCCESS
-----------------------------------------------------------------
TASK SUCCESS
```

---

### Kịch bản 2: Mức Nâng cao (Cá nhân hóa theo MSSV 23020723)
Nhập câu lệnh yêu cầu sắp xếp toàn bộ theo mã số sinh viên:
* Tiếng Anh:
  ```text
  Arrange all objects according to my student ID.
  ```
* Tiếng Việt:
  ```text
  Hãy sắp xếp các khối theo mã sinh viên của tôi.
  ```

**Hệ thống tự động suy luận:**
* MSSV: `23020723` $\implies P = 5$
* Vùng A $\to$ Khối Xanh lam (`blue_cube`)
* Vùng B $\to$ Khối Vàng (`yellow_cube`)
* Vùng C $\to$ Khối Đỏ (`red_cube`)

**Kế hoạch đa bước tự động sinh ra và thực thi:**
```text
LLM PLAN:
    pick(blue_cube)
    place(blue_cube, zone_a)
    pick(yellow_cube)
    place(yellow_cube, zone_b)
    pick(red_cube)
    place(red_cube, zone_c)
    home()
-----------------------------------------------------------------
EXECUTION:
pick(blue_cube) ............ SUCCESS
place(blue_cube, zone_a) ... SUCCESS
pick(yellow_cube) .......... SUCCESS
place(yellow_cube, zone_b) . SUCCESS
pick(red_cube) ............. SUCCESS
place(red_cube, zone_c) .... SUCCESS
home() ..................... SUCCESS
-----------------------------------------------------------------
TASK SUCCESS
```

---

### Kịch bản 3: Xử lý Xung đột Vùng đích bị chiếm (Conflict Resolution - Yêu cầu cốt lõi Bài 3)
Giả sử trên bàn, camera phát hiện `zone_b` đang bị `blue_cube` chiếm chỗ. Người dùng yêu cầu:
```text
Put the red cube in Zone B.
```
**Camera cung cấp trạng thái; lớp an toàn bổ sung bước dời vật cản vào kế hoạch của LLM:**
1. Camera xác định `zone_b` đang có `blue_cube`.
2. Hệ thống tìm vị trí đệm tạm phù hợp (`zone_temp_3`).
3. Di chuyển `blue_cube` ra `zone_temp_3`.
4. Gắp `red_cube` đặt vào `zone_b`.
5. Đưa tay máy về vị trí nghỉ (`home`).

**Đầu ra chuẩn hóa:**
```text
=================================================================
USER COMMAND:
  Put the red cube in Zone B.
-----------------------------------------------------------------
LLM REASONING (9Router Online / Gemini 3.5 Flash-lite):
  Camera phát hiện Zone B đang bị chiếm bởi 'blue_cube'. Cần kiểm tra vùng, di dời 'blue_cube' ra vị trí tạm 'zone_temp_3', sau đó gắp 'red_cube' đặt vào 'zone_b'.
-----------------------------------------------------------------
LLM PLAN:
    pick(blue_cube)
    place(blue_cube, zone_temp_3)
    pick(red_cube)
    place(red_cube, zone_b)
    home()
-----------------------------------------------------------------
EXECUTION:
pick(blue_cube) .............. SUCCESS
place(blue_cube, zone_temp_3)  SUCCESS
pick(red_cube) ............... SUCCESS
place(red_cube, zone_b) ...... SUCCESS
home() ....................... SUCCESS
-----------------------------------------------------------------
TASK SUCCESS
=================================================================
```

---

### Kịch bản 4: Kiểm tra Từ chối Kế hoạch Sai (Task Validator)
Nếu người dùng nhập câu lệnh vô nghĩa hoặc yêu cầu thao tác vượt quá thẩm quyền:
```text
[UR3-LLM] Nhap cau lenh > Move the apple to zone X.
```
**Phản hồi từ Validator:**
```text
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
[PLAN VALIDATOR] KE HOACH BI TU CHOI THUC THI!
Ly do: Object 'apple' khong hop le trong pick!
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
```

---

## 6. Danh mục Thư viện Robot Skills

| Tên Skill | Tham số | Chức năng | Cơ chế điều khiển / MoveIt 2 API |
| :--- | :--- | :--- | :--- |
| `detect_objects()` | Không | Gọi camera nhận diện vị trí 5 khối hộp qua HSV | `OpenCV Color Segmentation + Topic Bridge` |
| `check_zone(zone)` | `zone` | Kiểm tra trạng thái chiếm giữ của một vùng | `Camera Perception Scene State` |
| `find_free_position()` | Không | Tìm vị trí vùng đệm tạm đang còn trống | `Internal Space Allocation` |
| `home()` | Không | Đưa robot về tư thế an toàn quan sát toàn cảnh | `MoveGroup (Joint Constraints)` |
| `open_gripper()` | Không | Mở ngón kẹp trong Gazebo và cập nhật PlanningScene | `/gripper/left_cmd`, `/gripper/right_cmd` |
| `close_gripper(object)` | `object` | Đóng ngón kẹp, kiểm tra độ lệch tâm rồi giữ bằng joint vật lý Gazebo và cập nhật PlanningScene | `/gripper/left_cmd`, `/gripper/right_cmd` |
| `move_above(target)` | `object`/`zone` | Di chuyển đến điểm an toàn trên vật/vùng, mặc định cao $20\,\text{cm}$ | `MoveGroup (Cartesian Path / Pose Target)` |
| `move_to_zone(zone)` | `zone` | Di chuyển đến phía trên vùng Zone | `MoveGroup (Cartesian Path / Pose Target)` |
| `pick(object)` | `object` | Chu trình gắp: tiếp cận $\to$ hạ $\to$ kẹp vật lý $\to$ nhấc | `Cartesian Path Planning (No Teleport)` |
| `place(object, zone)` | `object`, `zone` | Chu trình đặt: tiếp cận $\to$ hạ $\to$ mở kẹp $\to$ nhấc | `Cartesian Path Planning (No Teleport)` |
| `swap(obj_a, obj_b)` | `object_a`, `object_b` | Hoán đổi 2 khối hộp sử dụng vùng đệm tạm | `Combined Primitive Skills` |
| `stack(top, bottom)` | `top`, `bottom` | Xếp chồng khối này lên trên đỉnh khối kia | `Cartesian Path Planning` |
| `inspect_scene()` | Không | Báo cáo chi tiết vị trí 5 khối và trạng thái các zone | `Scene State Query` |


---

## 7. Kiểm tra Bài 03 sau khi sửa

- Camera được gắn cố định trên cao để thấy đồng thời 5 block và 3 zone. Camera gắn tay có thể hỗ trợ tinh chỉnh điểm gắp ở gần, nhưng tự nó không cho ảnh toàn cảnh liên tục khi tay che vùng đích.
- Hệ thống chỉ nhận lệnh khi ảnh mới nhận diện đủ 5 block. Sau khi hoàn tất, camera xác nhận các cube đã đặt nằm đúng vùng; nếu không, kết quả là `TASK FAILED`.
- Trước khi gắp, MoveIt nhận collision box của những cube còn lại; cube mục tiêu được bỏ khỏi world collision object khi tiếp xúc và được gắn vào kẹp trong PlanningScene. Chuyển ngang thực hiện ở độ cao an toàn.
- Demo bắt buộc có xung đột: từ cảnh ban đầu, ra lệnh `Put the blue cube in zone B`, chờ hoàn tất; sau đó `Put the red cube in zone B`. Kế hoạch thứ hai phải dời blue cube sang vùng tạm còn trống rồi mới đặt red cube. Quay đồng thời Gazebo, camera annotated, terminal và kết quả cuối.
- Kiểm tra camera: `ros2 topic echo --once /scene/camera_state`. Trường `camera_active` phải là `true`, `detected_count` phải là `5`. Nếu thiếu block, đưa robot về home, kiểm tra ánh sáng và hiệu chuẩn HSV trước khi thử lại.

Gazebo trên máy thử nghiệm chạy khoảng 0,15–0,20 lần thời gian thực ở chế độ không GUI; tốc độ quan sát được phụ thuộc mạnh vào CPU/GPU. Kiểm tra trực tiếp cube được nâng, đi theo kẹp và rời kẹp khi mở sau mỗi thay đổi phiên bản Gazebo hoặc physics engine. Video demo và báo cáo nộp môn học cần ghi từ một lần chạy Gazebo hoàn chỉnh.
