# Báo cáo kỹ thuật – Bài thực hành 03

**Sinh viên:** Lê Anh Tuấn Bằng (23020723)  
**Nhánh mã nguồn:** `assignments_3`  
**Môi trường thử:** ROS 2 Humble, MoveIt 2, Ignition Gazebo 6, UR3.

## 1. Kiến trúc và phân quyền

Người dùng ra lệnh tự nhiên → LLM tạo danh sách skill và tham số → lớp lập kế hoạch an toàn đối chiếu trạng thái camera → `TaskValidator` kiểm tra whitelist, tiền điều kiện và vùng đang bị chiếm → `SkillExecutor` gọi các skill → MoveIt 2 lập quỹ đạo và kiểm tra va chạm → UR3 và gripper thực thi trong Gazebo. LLM không sinh góc khớp hay quỹ đạo.

Cảnh gồm một UR3, gripper hai ngón, một camera RGB cố định phía trên, bàn, năm cube, ba zone đích và ba vị trí đệm. Các cube ban đầu nằm ở năm khay nguồn. Camera phân đoạn màu HSV, chuyển tâm ảnh sang tọa độ bàn bằng `CameraInfo`, rồi công bố vị trí và vùng chiếm giữ. Vị trí nguồn trong YAML chỉ phục vụ tạo cảnh; trước khi nhận lệnh, robot phải có ảnh mới nhận đủ năm cube.

## 2. Gắp, giữ và nhả vật

Gripper dùng hai khớp tịnh tiến có đệm silicone và collision geometry. Robot mở ngón, đưa kẹp đến tọa độ camera, hạ xuống, khép ngón và kiểm tra sai lệch tâm. Khi ngón đã bao quanh cube, Gazebo tạo một `DetachableJoint` để giữ vật theo mô phỏng vật lý. Khi đặt, robot chờ vật ổn định trên bàn, tháo joint, mở ngón từ từ rồi rút kẹp. Không skill nào gọi `set_pose` của cube. PlanningScene đồng thời gắn hộp va chạm của vật vào gripper khi đang cầm và dùng vị trí camera cho các vật còn lại.

Đoạn chuyển ngang dài dùng MoveGroup. Đoạn hạ/nhấc ngắn ưu tiên Cartesian Path với `avoid_collisions=true`, yêu cầu gần như toàn bộ quỹ đạo hợp lệ và từ chối bước nhảy khớp lớn. Khi Cartesian không hợp lệ, hệ thống chuyển sang MoveGroup. Sau khi thả, tay rút lên độ cao chuyển ngang trước khi về home; home có tối đa ba lần lập kế hoạch có kiểm tra va chạm.

## 3. Xử lý zone bị chiếm

Nếu lệnh yêu cầu đưa `red_cube` vào `zone_b` đang có `blue_cube`, trạng thái camera khiến lớp an toàn chọn một vị trí đệm còn trống, rồi chèn chuỗi:

```text
pick(blue_cube) → place(blue_cube, zone_temp_3)
pick(red_cube)  → place(red_cube, zone_b)
home()
```

Bộ chọn ưu tiên `zone_temp_3` vì nằm trên dải thao tác thuận của UR3, sau đó thử hai vùng tạm còn lại. Khi cả ba vùng tạm đã có vật, hệ thống có thể dùng một zone đích khác đang trống; nếu không có vị trí hợp lệ, kế hoạch bị từ chối trước khi robot di chuyển. Sau khi về home, camera phải xác nhận vị trí cuối của mọi cube được đặt trong kế hoạch.

## 4. Camera đặt ở đâu?

Camera cố định trên cao là lựa chọn phù hợp cho bài này vì một ảnh có thể thấy năm cube và ba zone đích, từ đó lập kế hoạch trước khi tay máy che khuất một phần bàn. Camera gắn ở cổ tay có thể cải thiện căn chỉnh cục bộ khi gắp, nhưng cần hiệu chuẩn hand–eye và không thay thế được ảnh toàn cảnh liên tục. Vì vậy hệ thống dùng camera cố định làm nguồn trạng thái môi trường; camera cổ tay là phần mở rộng tùy chọn.

## 5. Kiểm thử và giới hạn

- `pytest`: kiểm tra việc dời vật cản, từ chối kế hoạch không có vị trí trống, đầu ra LLM sai kiểu, thứ tự do người dùng chỉ định và 720 cách bố trí năm cube vào sáu zone. Kết quả gần nhất: **6 passed**.
- `colcon build --packages-select ur3_llm_control`: **thành công**.
- Gazebo: đã quan sát cube được nâng và đi theo gripper; `purple_cube → zone_a` và `blue_cube → zone_b` được camera xác nhận. Ca tự động đầy đủ xử lý xung đột đã báo **TASK SUCCESS**; ảnh cuối xác nhận `blue_cube → zone_temp_3` và `red_cube → zone_b`.
- Physics giữ ở 1 kHz để cube 4 cm ổn định khi nhả; `ros2_control` chạy 500 Hz, camera 5 Hz và MoveIt Servo không dùng đến được tắt để giảm tải. Tốc độ quan sát vẫn phụ thuộc CPU/GPU và hệ số thời gian thực của Gazebo.
- Luồng LLM online cần `NINE_ROUTER_API_KEY` do người dùng cung cấp qua biến môi trường. Khóa cũ đã bị xóa khỏi bản mã hiện tại và cần thu hồi/đổi vì từng xuất hiện trong lịch sử Git. Gateway local trả HTTP 401 khi không có khóa, nên phép thử tích hợp online chưa hoàn tất ở môi trường này.
- Video demo cần được quay từ một lần chạy Gazebo/terminal với hai lệnh ở mục 3; file video chưa được đưa vào repository.

## 6. Cách chạy demo

```bash
cd ~/ur3_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ur3_llm_control
source install/setup.bash
export NINE_ROUTER_API_KEY="<khoa-moi>"
ros2 launch ur3_llm_control llm_robot.launch.py
```

Nhập lần lượt `Put the blue cube in Zone B.` và `Put the red cube in Zone B.`. Quan sát Gazebo, `/camera/image_annotated`, log `LLM PLAN`, `EXECUTION`, `TASK SUCCESS`, và ảnh cuối. README của package mô tả cấu hình và các skill chi tiết hơn.
