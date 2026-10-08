# Package `ur3_llm_control` - Điều khiển Robot UR3 bằng LLM & Skill-based Planning

> **Báo cáo Thực hành Bài 2 - Môn học: HRI - Tương tác Người - Robot**  
> **Sinh viên thực hiện:** Lê Anh Tuấn Bằng  
> **Mã số sinh viên (MSSV):** `23020723`  
> **Nhánh Git bài tập:** `assignments_2`  

---

## 1. Tổng quan & Kiến trúc Hệ thống

Package `ur3_llm_control` triển khai hệ thống điều khiển cánh tay robot công nghiệp 6 bậc tự do **Universal Robots UR3/UR3e** bằng câu lệnh ngôn ngữ tự nhiên (tiếng Việt / tiếng Anh) thông qua mô hình ngôn ngữ lớn (LLM) và quy hoạch theo kỹ năng (Skill-based Planning).

Hệ thống tuân thủ nghiêm ngặt mô hình phân tầng:
```
Natural Language Command (Người dùng nhập câu lệnh tự nhiên)
        ↓
   LLM Planner (Kết nối 9Router / OpenAI API để phân tích ngữ cảnh)
        ↓
 Structured Plan (Kế hoạch JSON có cấu trúc chỉ chứa Robot Skills)
        ↓
  Plan Validator (Kiểm tra Whitelist Skills, Objects, Zones và Logic)
        ↓
  Skill Executor (Điều phối gọi các Robot Skills và in log chuẩn hóa)
        ↓
     MoveIt 2 (Lập kế hoạch quỹ đạo và tránh va chạm)
        ↓
    UR3 / UR3e (Mô phỏng vật lý trong Ignition Gazebo kèm Gripper)
```

> **Nguyên tắc cốt lõi:**  
> - LLM **chỉ** được sử dụng để hiểu yêu cầu, lựa chọn và sắp xếp các robot skill.  
> - LLM **tuyệt đối không** sinh góc khớp, trajectory hoặc can thiệp trực tiếp vào bộ điều khiển robot.

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
   api_key: "sk-4e6f373b240d5083-b803ys-53876dfb"
   model: "gemini/gemini-3.5-flash-lite"
   fallback_to_smart_planner: false  # Chạy 100% Online LLM
   ```
3. Mô hình **`gemini/gemini-3.5-flash-lite`** được tối ưu hóa phản hồi cực nhanh (~1s), phân tích chuẩn xác cả tiếng Anh và tiếng Việt, hỗ trợ cả JSON và SSE streaming chunks.
4. **Cảnh báo mất kết nối:** Nếu 9Router bị tắt hoặc mất mạng, hệ thống sẽ hiện thông báo cảnh báo rõ ràng trên console để bạn dễ dàng kiểm tra.

---

## 4. Hướng dẫn Khởi chạy Hệ thống

### Bước 1: Dọn dẹp các tiến trình ngầm cũ (Tránh lỗi controller)
```bash
killall -9 ruby ign gzserver gzclient rviz2 2>/dev/null || pkill -9 -f "ign gazebo"
```

### Bước 2: Build Workspace (nếu vừa clone về)
```bash
cd ~/ur3_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ur3_llm_control
source install/setup.bash
```

### Bước 3: Khởi chạy Trọn gói 1 Lệnh (Gazebo + MoveIt 2 + RViz + LLM)
```bash
source ~/ur3_ws/install/setup.bash
ros2 launch ur3_llm_control llm_robot.launch.py
```

*Hệ thống sẽ tự động:*
1. Mở Gazebo với bàn thao tác, 3 khối hộp màu (`red_cube`, `yellow_cube`, `blue_cube`) và 4 vùng (`zone_a`, `zone_b`, `zone_c`, `zone_temp`).
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

### Kịch bản 3: Kiểm tra Từ chối Kế hoạch Sai (Task Validator)
Nếu người dùng nhập câu lệnh vô nghĩa hoặc yêu cầu thao tác vượt quá thẩm quyền:
```text
[UR3-LLM] Nhap cau lenh > Move the green apple to zone X.
```
**Phản hồi từ Validator:**
```text
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
[PLAN VALIDATOR] KE HOACH BI TU CHOI THUC THI!
Ly do: Kế hoạch rỗng hoặc chứa vật thể không hợp lệ.
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
```

---

## 6. Danh mục Thư viện Robot Skills

| Tên Skill | Tham số | Chức năng | MoveIt 2 API |
| :--- | :--- | :--- | :--- |
| `home()` | Không | Đưa robot về tư thế an toàn quan sát toàn cảnh | `MoveGroup (Joint Constraints)` |
| `open_gripper()` | Không | Mở kẹp, gỡ vật khỏi PlanningScene | `ApplyPlanningScene (REMOVE)` |
| `close_gripper(object)` | `object` | Đóng kẹp, gắn vật vào robot trong MoveIt | `ApplyPlanningScene (ADD)` |
| `move_above(target)` | `object`/`zone` | Di chuyển đến điểm trên không cách vật/vùng $14\,\text{cm}$ | `MoveGroup (Position & Orient Constraints)` |
| `move_to_zone(zone)` | `zone` | Di chuyển đến phía trên vùng Zone | `MoveGroup (Position & Orient Constraints)` |
| `pick(object)` | `object` | Chu trình gắp: tiếp cận $\to$ hạ $\to$ kẹp $\to$ nhấc | `Cartesian Path Planning` |
| `place(object, zone)` | `object`, `zone` | Chu trình đặt: tiếp cận $\to$ hạ $\to$ nhả $\to$ rút lui | `Cartesian Path Planning` |
| `inspect_state()` | Không | Báo cáo vị trí các vật và trạng thái kẹp | Trạng thái nội bộ |
