#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>

using namespace std::chrono_literals;

class JointMover : public rclcpp::Node
{
public:
  JointMover()
  : Node("joint_mover")
  {
    pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    // 1. Mouvement de l'Axe Y (Horizontal, 40 étapes, 0.0 à 0.4 et retour)
    for (int i = 0; i <= 400; ++i) path_y_.push_back(0.0 + i * 0.001); 
    for (int i = 400; i >= 0; --i) path_y_.push_back(0.0 + i * 0.001); 

    // 2. Mouvement de l'Axe Z (Vertical, 40 étapes, 0.0 à -0.4 et retour)
    for (int i = 0; i <= 400; ++i) path_z_.push_back(0.0 - i * 0.001); 
    for (int i = 400; i >= 0; --i) path_z_.push_back(0.0 - i * 0.001); 

    normalize_paths();
    index_ = 0;
    
    // Intervalle du timer réduit à 50ms (6x plus rapide)
    timer_ = this->create_wall_timer(1ms, std::bind(&JointMover::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "JointMover started: %zu steps at 50ms interval", path_y_.size());
  }

private:
  void normalize_paths()
  {
    size_t ly = path_y_.size();
    size_t lz = path_z_.size();
    if (ly == lz) return;

    if (ly > lz) {
      std::vector<double> new_z;
      new_z.reserve(ly);
      for (size_t i = 0; i < ly; ++i) {
        size_t idx = (i * lz) / ly;
        new_z.push_back(path_z_[idx]);
      }
      path_z_.swap(new_z);
    } else {
      std::vector<double> new_y;
      new_y.reserve(lz);
      for (size_t i = 0; i < lz; ++i) {
        size_t idx = (i * ly) / lz;
        new_y.push_back(path_y_[idx]);
      }
      path_y_.swap(new_y);
    }
  }

  void timer_callback()
  {
    if (index_ >= path_y_.size()) index_ = 0;
    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = this->now();
    msg.name = {"joint_y", "joint_z"};
    msg.position = { path_y_[index_], path_z_[index_] };

    pub_->publish(msg);

    ++index_;
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<double> path_y_;
  std::vector<double> path_z_;
  size_t index_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointMover>());
  rclcpp::shutdown();
  return 0;
}