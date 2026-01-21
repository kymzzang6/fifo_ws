#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include "TmLocalCamera.hxx"
#include "TmFrame.hxx"
#include "TmControl.hxx"

using namespace TmSDK;
using namespace std::chrono_literals;

class ThermalNode : public rclcpp::Node
{
public:
  ThermalNode() : Node("thermal_node")
  {
    temp_pub_ = create_publisher<std_msgs::msg::Float32>("thermal/target_temp", 10);
    
    declare_parameter<int>("target_x", -1);
    declare_parameter<int>("target_y", -1);

    RCLCPP_INFO(get_logger(), "TmSDK 열화상 노드 시작");

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
    RCLCPP_INFO(get_logger(), "연결: %dx%d", width_, height_);

    cam_->SetTempUnit(TempUnit::CELSIUS);
    if (cam_->pTmControl) {
      cam_->pTmControl->RunFlatFieldCorrection();
    }

    timer_ = create_wall_timer(33ms, std::bind(&ThermalNode::timer_callback, this));
  }

private:
  void timer_callback()
  {
    if (!cam_ || !cam_->IsOpen()) return;

    int tx = get_parameter("target_x").as_int();
    int ty = get_parameter("target_y").as_int();
    if (tx < 0 || tx >= width_) tx = width_ / 2;
    if (ty < 0 || ty >= height_) ty = height_ / 2;

    TmFrame frame;
    if (!cam_->QueryFrame(&frame, width_, height_)) return;
    if (frame.IsEmpty()) return;

    double raw = frame.GetPixel(tx, ty);
    if (raw <= 0.0) return;

    double temp = cam_->GetTemperature(raw);

    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(temp);
    temp_pub_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr temp_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<TmLocalCamera> cam_;
  int width_ = 0;
  int height_ = 0;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ThermalNode>());
  rclcpp::shutdown();
  return 0;
}
