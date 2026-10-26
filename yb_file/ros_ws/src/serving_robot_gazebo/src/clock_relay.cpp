#include <mutex>
#include <gz/transport/Node.hh>
#include <gz/msgs/clock.pb.h>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
// Preserve exact simulator time while bounding /clock fanout to 100 Hz.
// Safety freshness remains wall based; pausing simulation never fabricates time.
class SimClock : public rclcpp::Node {
 public:
  SimClock():Node("team_sim_clock") {
    out_=create_publisher<rosgraph_msgs::msg::Clock>("/clock",rclcpp::ClockQoS());
    transport_.Subscribe("/clock",&SimClock::onClock,this);
  }
 private:
  void onClock(const gz::msgs::Clock &in) {
    std::lock_guard<std::mutex> guard(lock_);
    int64_t ns=static_cast<int64_t>(in.sim().sec())*1000000000LL+in.sim().nsec();
    if (last_>=0 && ns>=last_ && ns-last_<10000000LL)return;
    last_=ns;rosgraph_msgs::msg::Clock msg;msg.clock.sec=in.sim().sec();msg.clock.nanosec=in.sim().nsec();out_->publish(msg);
  }
  std::mutex lock_;int64_t last_=-1;
  gz::transport::Node transport_;rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr out_;
};
int main(int argc,char **argv) {rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<SimClock>());rclcpp::shutdown();}
