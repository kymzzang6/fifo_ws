#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "TmLocalCamera.hxx"
#include "TmFrame.hxx"
#include "TmControl.hxx"

#include <vector>
#include <algorithm>
#include <memory>
#include <numeric>

using namespace TmSDK;
using namespace std::chrono_literals;

class ThermalCalculatorNode : public rclcpp::Node
{
public:
  ThermalCalculatorNode() : Node("thermal_calculator_node")
  {
    // Publishers (파이썬에게 계산된 온도 Float32 값만 반환)
    face_temp_pub_ = create_publisher<std_msgs::msg::Float32>("/thermal/face_temp", 10);
    lhand_temp_pub_ = create_publisher<std_msgs::msg::Float32>("/thermal/lhand_temp", 10);
    rhand_temp_pub_ = create_publisher<std_msgs::msg::Float32>("/thermal/rhand_temp", 10);

    // Subscribers (파이썬에서 BBox 수신)
    face_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/bbox/face", 10, std::bind(&ThermalCalculatorNode::face_cb, this, std::placeholders::_1));
    lhand_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/bbox/l_hand", 10, std::bind(&ThermalCalculatorNode::lhand_cb, this, std::placeholders::_1));
    rhand_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/bbox/r_hand", 10, std::bind(&ThermalCalculatorNode::rhand_cb, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "🔥 TmSDK C++ 온도 계산 전담 서버 시작 (상위 30%% 평균 모드)");

    // 카메라 연결
    auto cam_list = TmLocalCamera::GetCameraList();
    if (cam_list.empty()) {
      RCLCPP_ERROR(get_logger(), "❌ 열화상 카메라 없음");
      return;
    }

    cam_ = std::make_shared<TmLocalCamera>();
    if (!cam_->Open(&cam_list[0])) {
      RCLCPP_ERROR(get_logger(), "❌ 연결 실패");
      return;
    }

    width_ = cam_->GetWidth();
    height_ = cam_->GetHeight();
    RCLCPP_INFO(get_logger(), "✅ 연결 성공! 해상도: %dx%d", width_, height_);

    cam_->SetTempUnit(TempUnit::CELSIUS); // 온도를 섭씨로 설정
    if (cam_->pTmControl) {
      cam_->pTmControl->RunFlatFieldCorrection();
    }

    // 약 30FPS 주기
    timer_ = create_wall_timer(33ms, std::bind(&ThermalCalculatorNode::timer_callback, this));
  }

private:
  std::vector<float> face_bbox_ = {-1, -1, -1, -1};
  std::vector<float> lhand_bbox_ = {-1, -1, -1, -1};
  std::vector<float> rhand_bbox_ = {-1, -1, -1, -1};

  void face_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg) { face_bbox_ = msg->data; }
  void lhand_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg) { lhand_bbox_ = msg->data; }
  void rhand_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg) { rhand_bbox_ = msg->data; }

  // 🌟 박스 내부 픽셀들을 모아 상위 30%의 온도를 찾아 섭씨로 반환
  double get_top30_avg_temp(TmFrame& frame, const std::vector<float>& bbox) {
    if (bbox[0] == -1.0) return -1.0;

    int x1 = std::max(0, static_cast<int>(bbox[0]));
    int y1 = std::max(0, static_cast<int>(bbox[1]));
    int x2 = std::min(width_ - 1, static_cast<int>(bbox[2]));
    int y2 = std::min(height_ - 1, static_cast<int>(bbox[3]));

    if (x1 >= x2 || y1 >= y2) return -1.0;

    // 박스 안의 모든 픽셀 원본(Raw) 데이터를 저장할 벡터
    std::vector<double> raw_pixels;
    // 예상되는 픽셀 개수만큼 메모리 미리 할당 (속도 최적화)
    raw_pixels.reserve((x2 - x1 + 1) * (y2 - y1 + 1));

    for (int y = y1; y <= y2; ++y) {
      for (int x = x1; x <= x2; ++x) {
        double raw = frame.GetPixel(x, y);
        if (raw > 0.0) { // 유효한 픽셀만 추가
          raw_pixels.push_back(raw);
        }
      }
    }

    if (raw_pixels.empty()) return -1.0;

    // 🌟 내림차순(가장 뜨거운 것부터) 정렬 (C++ algorithm 헤더 활용)
    std::sort(raw_pixels.begin(), raw_pixels.end(), std::greater<double>());

    // 상위 30%에 해당하는 개수 계산 (최소 1개는 포함하도록 보정)
    size_t top_count = std::max<size_t>(1, static_cast<size_t>(raw_pixels.size() * 0.3));

    // 상위 30% 픽셀값들의 합계 구하기
    double sum_raw = 0.0;
    for (size_t i = 0; i < top_count; ++i) {
        sum_raw += raw_pixels[i];
    }
    
    // 평균 Raw 값 계산
    double avg_raw = sum_raw / top_count - 10;

    // TmSDK를 이용해 최종 평균 Raw 값을 섭씨(℃) 온도로 변환하여 리턴
    return cam_->GetTemperature(avg_raw);
  }

  void timer_callback()
  {
    if (!cam_ || !cam_->IsOpen()) return;

    TmFrame frame;
    if (!cam_->QueryFrame(&frame, width_, height_)) return;
    if (frame.IsEmpty()) return;

    // 1. 얼굴 온도 계산 및 퍼블리시
    double face_temp = get_top30_avg_temp(frame, face_bbox_);
    if (face_temp > 0.0) {
      std_msgs::msg::Float32 msg;
      msg.data = static_cast<float>(face_temp);
      face_temp_pub_->publish(msg);
    }

    // 2. 왼손 온도
    double l_temp = get_top30_avg_temp(frame, lhand_bbox_);
    if (l_temp > 0.0) {
      std_msgs::msg::Float32 msg;
      msg.data = static_cast<float>(l_temp);
      lhand_temp_pub_->publish(msg);
    }

    // 3. 오른손 온도
    double r_temp = get_top30_avg_temp(frame, rhand_bbox_);
    if (r_temp > 0.0) {
      std_msgs::msg::Float32 msg;
      msg.data = static_cast<float>(r_temp);
      rhand_temp_pub_->publish(msg);
    }
  }

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr face_temp_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr lhand_temp_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rhand_temp_pub_;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr face_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr lhand_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr rhand_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<TmLocalCamera> cam_;
  int width_ = 0;
  int height_ = 0;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ThermalCalculatorNode>());
  rclcpp::shutdown();
  return 0;
}
