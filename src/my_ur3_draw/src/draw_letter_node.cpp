#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <cmath>
#include <atomic>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <vector>
#include <algorithm>

// HÀM TRÍCH XUẤT VÀ SẮP XẾP CÁC ĐƯỜNG CONTOUR TỪ MA TRẬN ẢNH NHỊ PHÂN
std::vector<std::vector<cv::Point>> extractSortedContours(const cv::Mat& binary)
{
    std::vector<std::vector<cv::Point>> raw_contours;
    std::vector<cv::Vec4i> hierarchy;
    // Sử dụng RETR_TREE để phân biệt nét ngoài (parent == -1) và các lỗ rỗng bên trong
    cv::findContours(binary, raw_contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    struct ContourInfo {
        size_t index;
        int x;
        int y;
        int parent;
        double area;
    };

    std::vector<ContourInfo> valid_contours;
    for (size_t i = 0; i < raw_contours.size(); ++i) {
        double area = cv::contourArea(raw_contours[i]);
        if (area < 50.0) continue;
        cv::Rect bbox = cv::boundingRect(raw_contours[i]);
        int parent = hierarchy.empty() ? -1 : hierarchy[i][3];
        valid_contours.push_back({i, bbox.x, bbox.y, parent, area});
    }

    // Sắp xếp thứ tự vẽ tự nhiên:
    // 1. Nhóm theo vị trí X (từ trái sang phải) để đọc/vẽ chữ theo thứ tự
    // 2. Trong cùng một ký tự: vẽ nét bao ngoài trước (parent == -1), sau đó đến các lỗ rỗng
    std::sort(valid_contours.begin(), valid_contours.end(), [](const ContourInfo& a, const ContourInfo& b) {
        int x_group_a = a.x / 30;
        int x_group_b = b.x / 30;
        if (x_group_a != x_group_b) {
            return x_group_a < x_group_b;
        }
        if ((a.parent == -1) != (b.parent == -1)) {
            return a.parent == -1; // nét ngoài vẽ trước
        }
        return a.y < b.y;
    });

    std::vector<std::vector<cv::Point>> result;
    for (const auto& item : valid_contours) {
        std::vector<cv::Point> approx;
        cv::approxPolyDP(raw_contours[item.index], approx, 2.0, true);
        if (approx.size() >= 2) {
            result.push_back(approx);
        }
    }
    return result;
}

// HÀM TẠO DANH SÁCH WAYPOINTS CHO TỪNG ĐƯỜNG CONTOUR RIÊNG BIỆT
// Mỗi contour là một hành trình độc lập: Hover -> Plunge -> Draw Contour -> Retract to Hover
std::vector<std::vector<geometry_msgs::msg::Pose>> generateContoursWaypoints(
    const cv::Mat& img,
    const geometry_msgs::msg::Pose& start_pose,
    double target_width_m)
{
    std::vector<std::vector<geometry_msgs::msg::Pose>> all_contours_waypoints;
    if (img.empty()) {
        return all_contours_waypoints;
    }

    cv::Mat binary;
    if (img.channels() > 1) {
        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        cv::threshold(gray, binary, 128, 255, cv::THRESH_BINARY_INV);
    } else {
        cv::threshold(img, binary, 128, 255, cv::THRESH_BINARY_INV);
    }

    std::vector<std::vector<cv::Point>> sorted_contours = extractSortedContours(binary);
    double scale_factor = target_width_m / (double)img.cols;
    const double HOVER_OFFSET = 0.08; // Lùi bút 8cm trên không

    for (const auto& contour_pts : sorted_contours) {
        std::vector<geometry_msgs::msg::Pose> contour_wps;

        // 1. Điểm bay trên không (hover) thẳng hàng với điểm bắt đầu nét vẽ
        geometry_msgs::msg::Pose hover_pose = start_pose;
        hover_pose.position.y -= contour_pts[0].x * scale_factor;
        hover_pose.position.z -= contour_pts[0].y * scale_factor;
        hover_pose.position.x = start_pose.position.x - HOVER_OFFSET;
        contour_wps.push_back(hover_pose);

        // 2. Hạ bút thẳng đứng vào mặt phẳng vẽ (chạm bảng)
        geometry_msgs::msg::Pose draw_pose = hover_pose;
        draw_pose.position.x = start_pose.position.x;
        contour_wps.push_back(draw_pose);

        // 3. Quét dọc theo viền nét vẽ trên mặt bảng
        for (size_t i = 1; i < contour_pts.size(); ++i) {
            geometry_msgs::msg::Pose p = start_pose;
            p.position.y -= contour_pts[i].x * scale_factor;
            p.position.z -= contour_pts[i].y * scale_factor;
            p.position.x = start_pose.position.x;
            contour_wps.push_back(p);
        }

        // 4. Khép kín vòng nét vẽ trên mặt bảng
        contour_wps.push_back(draw_pose);

        // 5. Nhấc bút thẳng đứng lùi ra 8cm (tuyệt đối không di chuyển ngang khi đang nhấc bút)
        contour_wps.push_back(hover_pose);

        all_contours_waypoints.push_back(contour_wps);
    }

    return all_contours_waypoints;
}

// HÀM TƯƠNG THÍCH CŨ (Nối tất cả các contour thành một chuỗi)
std::vector<geometry_msgs::msg::Pose> generateWaypointsFromMat(
    const cv::Mat& img, 
    geometry_msgs::msg::Pose start_pose, 
    double target_width_m) 
{
    std::vector<geometry_msgs::msg::Pose> flat_waypoints;
    auto contours_wps = generateContoursWaypoints(img, start_pose, target_width_m);
    for (const auto& c_wps : contours_wps) {
        flat_waypoints.insert(flat_waypoints.end(), c_wps.begin(), c_wps.end());
    }
    return flat_waypoints;
}

// HÀM CHUYỂN ĐỔI ẢNH TỪ FILE THÀNH DANH SÁCH CONTOUR WAYPOINTS
std::vector<std::vector<geometry_msgs::msg::Pose>> generateContoursFromImage(
    const std::string& image_path, 
    const geometry_msgs::msg::Pose& start_pose, 
    double target_width_m) 
{
    cv::Mat img = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
    return generateContoursWaypoints(img, start_pose, target_width_m);
}

// HÀM RENDER CHỮ CÁI BẤT KỲ THÀNH ẢNH RÕ NÉT CĂN GIỮA
cv::Mat renderTextToImage(const std::string& text, int canvas_w = 400, int canvas_h = 400)
{
    cv::Mat img = cv::Mat(canvas_h, canvas_w, CV_8UC1, cv::Scalar(255));
    if (text.empty()) return img;

    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    int baseline = 0;
    
    double fontScale = 1.0;
    int thickness = 2;
    cv::Size initialSize = cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);
    
    double scale_x = (double)(canvas_w * 0.75) / (double)std::max(1, initialSize.width);
    double scale_y = (double)(canvas_h * 0.70) / (double)std::max(1, initialSize.height);
    fontScale = std::min(scale_x, scale_y);
    fontScale = std::max(0.5, fontScale);
    thickness = std::max(2, (int)(fontScale * 2.2));

    cv::Size textSize = cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);
    cv::Point textOrg(
        std::max(10, (canvas_w - textSize.width) / 2),
        std::max(textSize.height, (canvas_h + textSize.height) / 2)
    );

    cv::putText(img, text, textOrg, fontFace, fontScale, cv::Scalar(0), thickness, cv::LINE_AA);
    return img;
}

// HÀM CHUYỂN ĐỔI CHỮ CÁI THÀNH DANH SÁCH CÁC CONTOURS WAYPOINTS
std::vector<std::vector<geometry_msgs::msg::Pose>> generateContoursFromText(
    const std::string& text,
    const geometry_msgs::msg::Pose& start_pose,
    double target_width_m)
{
    int canvas_w = std::max(400, (int)text.length() * 140);
    int canvas_h = 400;
    cv::Mat img = renderTextToImage(text, canvas_w, canvas_h);
    return generateContoursWaypoints(img, start_pose, target_width_m);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("draw_letter_node", rclcpp::NodeOptions().parameter_overrides({{"use_sim_time", true}}));

  // KHAI BÁO CÁC PARAMETERS ĐỂ TÙY BIẾN HOẶC DÙNG TRONG LAUNCH FILE
  node->declare_parameter<std::string>("letter", "");
  node->declare_parameter<std::string>("image_path", "");
  node->declare_parameter<double>("target_width", 0.15);
  node->declare_parameter<bool>("interactive", false);

  std::string letter_param = node->get_parameter("letter").as_string();
  std::string image_path_param = node->get_parameter("image_path").as_string();
  double target_width = node->get_parameter("target_width").as_double();
  bool interactive = node->get_parameter("interactive").as_bool();

  // Xác định đường dẫn file letter.png mặc định
  std::string default_image_path = "/home/anhbang/ur3_ws/src/my_ur3_draw/letter.png";
  try {
      std::string pkg_share = ament_index_cpp::get_package_share_directory("my_ur3_draw");
      std::string share_img = pkg_share + "/letter.png";
      std::ifstream f(share_img.c_str());
      if (f.good()) {
          default_image_path = share_img;
      }
  } catch (...) {
  }

  std::string target_letter = letter_param;
  std::string target_image = image_path_param.empty() ? default_image_path : image_path_param;

  // Nếu chưa truyền letter: Cho phép người dùng nhập trực tiếp từ bàn phím console
  if (target_letter.empty() && image_path_param.empty()) {
      if (interactive || isatty(STDIN_FILENO)) {
          std::cout << "\n======================================================\n";
          std::cout << "  UR3 DRAW LETTER - DIEU KHIEN VE TAY MAY ROBOT\n";
          std::cout << "======================================================\n";
          std::cout << "Nhap chu cai bat ky muon ve (vi du: A, B, C, UR3, ...)\n";
          std::cout << "Hoac nhan ENTER de giu nguyen ve tu file letter.png: ";
          std::cout.flush();
          std::string input_line;
          if (std::getline(std::cin, input_line)) {
              while (!input_line.empty() && isspace(input_line.front())) input_line.erase(input_line.begin());
              while (!input_line.empty() && isspace(input_line.back())) input_line.pop_back();
              target_letter = input_line;
          }
      }
  }

  rclcpp::QoS marker_qos(100);
  marker_qos.reliable();
  marker_qos.transient_local();
  auto marker_pub = node->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", marker_qos);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner = std::thread([&executor]() { executor.spin(); });

  moveit::planning_interface::MoveGroupInterface move_group(node, "ur_manipulator");
  rclcpp::sleep_for(std::chrono::seconds(2));

  // Xóa sạch các marker mẫu cũ (planned_guide) nếu có trong RViz theo yêu cầu người dùng
  visualization_msgs::msg::Marker clear_guide;
  clear_guide.header.frame_id = move_group.getPlanningFrame();
  clear_guide.header.stamp = rclcpp::Time(0);
  clear_guide.ns = "planned_guide";
  clear_guide.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_pub->publish(clear_guide);

  // 1. TƯ THẾ HƯỚNG NGANG (READY POSE)
  RCLCPP_INFO(node->get_logger(), "Di chuyen ve tu the viet bang...");
  std::vector<double> ready_pose = {0.0, -M_PI/2.0, M_PI/2.0, 0.0, M_PI/2.0, 0.0};
  move_group.setJointValueTarget(ready_pose);
  auto move_result = move_group.move();
  if (move_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_WARN(node->get_logger(), "Lan 1 chua xong, cho them 2s de controller san sang va thu lai...");
      rclcpp::sleep_for(std::chrono::seconds(2));
      move_group.setStartStateToCurrentState();
      move_group.setJointValueTarget(ready_pose);
      move_result = move_group.move();
  }
  rclcpp::sleep_for(std::chrono::seconds(1));
  move_group.setStartStateToCurrentState(); 

  // 2. TẠO DANH SÁCH QUỸ ĐẠO CHO TỪNG ĐƯỜNG CONTOUR
  geometry_msgs::msg::Pose start_pose = move_group.getCurrentPose().pose; 
  std::vector<std::vector<geometry_msgs::msg::Pose>> all_contours_waypoints;

  if (!target_letter.empty()) {
      RCLCPP_INFO(node->get_logger(), "Dang tao quy dao cho chu cai / tu: '%s' (do rong: %.2fm)...", target_letter.c_str(), target_width);
      all_contours_waypoints = generateContoursFromText(target_letter, start_pose, target_width);
  } else {
      RCLCPP_INFO(node->get_logger(), "Dang quet va trich xuat anh tu: %s (do rong: %.2fm)...", target_image.c_str(), target_width);
      all_contours_waypoints = generateContoursFromImage(target_image, start_pose, target_width);
  }

  if (all_contours_waypoints.empty()) {
      if (!target_letter.empty()) {
          RCLCPP_ERROR(node->get_logger(), "Loi: Khong the tao waypoints cho chu '%s'!", target_letter.c_str());
      } else {
          RCLCPP_ERROR(node->get_logger(), "Loi: Khong the tao waypoints. Kiem tra xem co file letter.png tai: %s chua!", target_image.c_str());
      }
      rclcpp::shutdown();
      spinner.join();
      return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Tong so net ve can thuc hien: %zu net.", all_contours_waypoints.size());

  // 3. THIẾT LẬP THREAD VẼ REAL-TIME MARKER CHUẨN XÁC, TRIỆT TIÊU TOÀN BỘ NÉT THỪA
  std::atomic<bool> is_running{true};
  std::atomic<int> active_contour{-1};
  std::atomic<bool> stroke_completed_for_contour{false};
  std::vector<visualization_msgs::msg::Marker> completed_strokes;

  std::thread marker_thread([&]() {
      visualization_msgs::msg::Marker path_marker;
      path_marker.header.frame_id = move_group.getPlanningFrame();
      path_marker.ns = "realtime_path";
      path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      path_marker.action = visualization_msgs::msg::Marker::ADD;
      path_marker.pose.position.x = 0.0;
      path_marker.pose.position.y = 0.0;
      path_marker.pose.position.z = 0.0;
      path_marker.pose.orientation.x = 0.0;
      path_marker.pose.orientation.y = 0.0;
      path_marker.pose.orientation.z = 0.0;
      path_marker.pose.orientation.w = 1.0;
      path_marker.scale.x = 0.008; // Nét mực đỏ dày 8mm sắc nét
      path_marker.color.a = 1.0;
      path_marker.color.r = 1.0;
      path_marker.color.g = 0.0;
      path_marker.color.b = 0.0;
      path_marker.lifetime = rclcpp::Duration::from_seconds(0);

      int marker_id = 0;
      bool was_pen_down = false;
      int last_handled_contour = -1;

      while (is_running.load()) {
          int c_id = active_contour.load();
          if (c_id < 0) {
              // Robot đang bay chuyển tiếp giữa các nét hoặc ở Home: Tuyệt đối KHÔNG ghi nhận nét vẽ
              if (was_pen_down) {
                  if (path_marker.points.size() >= 2) {
                      completed_strokes.push_back(path_marker);
                      marker_pub->publish(path_marker);
                  }
                  was_pen_down = false;
              }
              rclcpp::sleep_for(std::chrono::milliseconds(20));
              continue;
          }

          if (c_id != last_handled_contour) {
              last_handled_contour = c_id;
              was_pen_down = false;
              stroke_completed_for_contour.store(false);
          }

          // Khi nét vẽ của contour này đã hoàn thành (đã nhấc bút), tuyệt đối khóa không cho thêm điểm
          if (stroke_completed_for_contour.load()) {
              rclcpp::sleep_for(std::chrono::milliseconds(20));
              continue;
          }

          geometry_msgs::msg::Pose real_pose = move_group.getCurrentPose().pose;

          // Tiêu chuẩn hạ bút: Phải áp sát mặt phẳng bảng trong phạm vi 3mm (0.003m)
          bool is_on_board = (std::abs(real_pose.position.x - start_pose.position.x) <= 0.003);

          if (is_on_board) {
              if (!was_pen_down) {
                  marker_id++;
                  path_marker.id = marker_id;
                  path_marker.points.clear();
                  was_pen_down = true;
              }
              path_marker.header.stamp = rclcpp::Time(0);

              // Cố định toạ độ X trên mặt phẳng bảng để nét vẽ phẳng hoàn hảo
              geometry_msgs::msg::Point pt = real_pose.position;
              pt.x = start_pose.position.x;

              if (path_marker.points.empty()) {
                  path_marker.points.push_back(pt);
              } else {
                  const auto& last_p = path_marker.points.back();
                  double dist = std::hypot(pt.y - last_p.y, pt.z - last_p.z);
                  if (dist > 0.002) {
                      path_marker.points.push_back(pt);
                  }
              }

              if (path_marker.points.size() >= 2) {
                  marker_pub->publish(path_marker);
              }
          } else {
              // Bút rời khỏi mặt bảng (đang nhấc bút lùi lại)
              if (was_pen_down) {
                  if (path_marker.points.size() >= 2) {
                      // Khép kín vòng nét vẽ nếu điểm đầu và cuối gần nhau
                      const auto& first_p = path_marker.points.front();
                      const auto& last_p = path_marker.points.back();
                      if (std::hypot(first_p.y - last_p.y, first_p.z - last_p.z) < 0.015) {
                          path_marker.points.push_back(first_p);
                      }
                      completed_strokes.push_back(path_marker);
                      marker_pub->publish(path_marker);
                  }
                  was_pen_down = false;
                  // Đánh dấu nét này đã xong hoàn toàn! Không bao giờ vẽ thêm cho contour này
                  stroke_completed_for_contour.store(true);
              }
          }

          rclcpp::sleep_for(std::chrono::milliseconds(20));
      }

      if (was_pen_down && path_marker.points.size() >= 2) {
          completed_strokes.push_back(path_marker);
          marker_pub->publish(path_marker);
      }

      // Giữ tất cả nét vẽ hiển thị vĩnh viễn trên RViz
      for (const auto& stroke : completed_strokes) {
          marker_pub->publish(stroke);
      }
  });

  // 4. BƯỚC ĐỆM: RÚT BÚT LÙI RA 8CM TRÊN KHÔNG TRƯỚC KHI TIẾN HÀNH VẼ
  const double HOVER_OFFSET = 0.08;
  geometry_msgs::msg::Pose initial_hover = start_pose;
  initial_hover.position.x -= HOVER_OFFSET;
  std::vector<geometry_msgs::msg::Pose> prep_waypoints = {initial_hover};
  moveit_msgs::msg::RobotTrajectory prep_traj;
  if (move_group.computeCartesianPath(prep_waypoints, 0.01, 0.0, prep_traj) > 0.9) {
      move_group.execute(prep_traj);
  }

  // 5. THỰC HIỆN TỪNG CONTOUR RIÊNG BIỆT (KHÔNG BO GÓC CHUYỂN TIẾP TRÊN KHÔNG)
  for (size_t c = 0; c < all_contours_waypoints.size(); ++c) {
      RCLCPP_INFO(node->get_logger(), "Dang ve net %zu / %zu...", c + 1, all_contours_waypoints.size());

      move_group.setStartStateToCurrentState();
      moveit_msgs::msg::RobotTrajectory contour_traj;
      double fraction = move_group.computeCartesianPath(all_contours_waypoints[c], 0.01, 0.0, contour_traj);

      if (fraction < 0.85) {
          RCLCPP_WARN(node->get_logger(), "Net %zu chi lap ke hoach duoc %.2f%%, bo qua de dam bao an toan.", c + 1, fraction * 100.0);
          continue;
      }

      // Kích hoạt ghi nhận nét vẽ cho contour này
      stroke_completed_for_contour.store(false);
      active_contour.store((int)c);

      move_group.execute(contour_traj);

      // Ngắt kích hoạt ngay khi hoàn thành contour
      active_contour.store(-1);
      rclcpp::sleep_for(std::chrono::milliseconds(200));
  }

  is_running.store(false);
  marker_thread.join();

  if (!target_letter.empty()) {
      RCLCPP_INFO(node->get_logger(), "HOAN THANH VE CHU '%s'!", target_letter.c_str());
  } else {
      RCLCPP_INFO(node->get_logger(), "HOAN THANH VE TU ANH!");
  }

  // 6. QUAY VỀ TRẠNG THÁI HOME (Up Pose)
  rclcpp::sleep_for(std::chrono::seconds(1));
  RCLCPP_INFO(node->get_logger(), "Quay ve trang thai Home (Up Pose)...");
  std::vector<double> home_pose = {0.0, -M_PI/2.0, 0.0, -M_PI/2.0, 0.0, 0.0};
  move_group.setJointValueTarget(home_pose);
  move_group.move();

  rclcpp::shutdown();
  spinner.join(); 
  return 0;
}