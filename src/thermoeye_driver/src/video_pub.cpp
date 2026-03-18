#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

#include "TmLocalCamera.hxx"
#include "TmFrame.hxx"
#include "TmControl.hxx"

using namespace TmSDK;
using namespace std::chrono_literals;

class ThermalVideoNode : public rclcpp::Node
{
public:
  ThermalVideoNode() : Node("thermal_video_node")
  {
    // 비디오 토픽 퍼블리셔 생성
    video_pub_ = create_publisher<sensor_msgs::msg::Image>("/thermal/video", 10);
    
    RCLCPP_INFO(get_logger(), "TmSDK 열화상 비디오 노드 시작");

    // 카메라 연결
    auto cam_list = TmLocalCamera::GetCameraList();
    if (cam_list.empty()) {
      RCLCPP_ERROR(get_logger(), "카메라 없음");
      return;
    }

    cam_ = std::make_shared<TmLocalCamera>();
    if (!cam_->Open(&cam_list[0])) {
      RCLCPP_ERROR(get_logger(), "연결 실패");
      return;
    }

    width_ = cam_->GetWidth();
    height_ = cam_->GetHeight();
    RCLCPP_INFO(get_logger(), "연결 성공: 해상도 %dx%d", width_, height_);

    cam_->SetTempUnit(TempUnit::CELSIUS);
    if (cam_->pTmControl) {
      cam_->pTmControl->RunFlatFieldCorrection();
    }

    // 30FPS (약 33ms) 주기로 타이머 콜백 실행
    timer_ = create_wall_timer(33ms, std::bind(&ThermalVideoNode::timer_callback, this));
  }

private:
  void timer_callback()
  {
    if (!cam_ || !cam_->IsOpen()) return;

    TmFrame frame;
    if (!cam_->QueryFrame(&frame, width_, height_)) return;
    if (frame.IsEmpty()) return;

    // 1. 온도 데이터를 담을 OpenCV Float 행렬 생성
    cv::Mat temp_mat(height_, width_, CV_32FC1);
    double min_temp = 1000.0, max_temp = -1000.0;

    // 2. 픽셀 순회하며 온도 변환 및 화면 내 최고/최저 온도 탐색
    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < width_; ++x) {
        double raw = frame.GetPixel(x, y);
        double temp = cam_->GetTemperature(raw);
        
        temp_mat.at<float>(y, x) = static_cast<float>(temp);
        
        if (temp < min_temp) min_temp = temp;
        if (temp > max_temp) max_temp = temp;
      }
    }

    // 3. 시각화를 위해 8비트 이미지로 Min-Max 정규화
    cv::Mat draw_mat;
    if (max_temp > min_temp) {
      temp_mat.convertTo(draw_mat, CV_8UC1, 255.0 / (max_temp - min_temp), -min_temp * 255.0 / (max_temp - min_temp));
    } else {
      draw_mat = cv::Mat::zeros(height_, width_, CV_8UC1);
    }

    // 4. 열화상 특유의 컬러맵 적용 (INFERNO 또는 JET 추천)
    cv::Mat color_mat;
    cv::applyColorMap(draw_mat, color_mat, cv::COLORMAP_INFERNO);

    // 5. ROS 2 Image 메시지로 변환 후 퍼블리시
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", color_mat).toImageMsg();
    msg->header.stamp = this->now();
    msg->header.frame_id = "thermal_camera_link";

    video_pub_->publish(*msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr video_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<TmLocalCamera> cam_;
  int width_ = 0;
  int height_ = 0;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ThermalVideoNode>());
  rclcpp::shutdown();
  return 0;
}
