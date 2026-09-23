# Workspace `ur3_ws` - Universal Robots UR3 Draw Letters

Tài liệu hướng dẫn chi tiết về cách cấu hình, biên dịch và chạy 2 chức năng của package `my_ur3_draw`:
- **Chức năng 1 (Chức năng gốc):** Vẽ theo biên dạng file ảnh (`letter.png`).
- **Chức năng 2 (Chức năng mới):** Nhập chữ cái / ký tự bất kỳ để tay máy robot UR3 vẽ ra chữ tương ứng.

👉 Xem tài liệu hướng dẫn đầy đủ tại: **[`src/my_ur3_draw/README.md`](src/my_ur3_draw/README.md)**

---

## Tóm Tắt Lệnh Chạy Nhanh

### 1. Biên dịch workspace
```bash
cd ~/ur3_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select my_ur3_draw
source install/setup.bash
```

### 2. Chức năng 1: Vẽ từ ảnh mặc định (`letter.png`)
- **Chạy trọn gói 1 lệnh cả mô phỏng:**
  ```bash
  ros2 launch my_ur3_draw ur3_draw_sim.launch.py
  ```
- **Chạy node riêng biệt (khi mô phỏng đã mở):**
  ```bash
  ros2 launch my_ur3_draw draw_letter.launch.py
  ```

### 3. Chức năng 2: Nhập chữ cái bất kỳ để vẽ
- **Chạy trọn gói 1 lệnh cả mô phỏng:**
  ```bash
  # Vẽ chữ A:
  ros2 launch my_ur3_draw ur3_draw_sim.launch.py letter:=A

  # Vẽ chữ B:
  ros2 launch my_ur3_draw ur3_draw_sim.launch.py letter:=B

  # Vẽ chữ hoặc từ khác:
  ros2 launch my_ur3_draw ur3_draw_sim.launch.py letter:=UR3
  ```
- **Chạy riêng biệt qua Launch:**
  ```bash
  ros2 launch my_ur3_draw draw_letter.launch.py letter:=A
  ```
- **Chạy tương tác hỏi nhập từ bàn phím Console:**
  ```bash
  ros2 run my_ur3_draw draw_letter_node
  ```

