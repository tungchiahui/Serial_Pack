#include "rclcpp/rclcpp.hpp"
#include "serial_transport/serial_transport.hpp"
#include <wire_protocol/protocol.hpp>
#include <functional>
#include <rclcpp/logging.hpp>


using namespace std::chrono_literals;

class Serial_Node: public rclcpp::Node
{
  public:
    Serial_Node()
      : Node("serial_node_cpp")
    {
      RCLCPP_INFO(this->get_logger(),"serial_node启动!");

      //==================================================
      // ROS2 参数
      //==================================================

      //声明并设置默认参数
      this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
      this->declare_parameter<int>("baud_rate", 115200);

      //参数读取
      serial_config.port_name_ = this->get_parameter("port").as_string();
      serial_config.baud_rate_ = this->get_parameter("baud_rate").as_int();
      //硬参数
      serial_config.character_size_ = 8;
      serial_config.parity_ = asio::serial_port_base::parity::none;
      serial_config.stop_bits_ = asio::serial_port_base::stop_bits::one;
      serial_config.flow_control_ = asio::serial_port_base::flow_control::none;

      //开启串口与串口异步接收
      serial_driver.start(serial_config, std::bind(&Serial_Node::serial_receive_callback,this,std::placeholders::_1));

      // 创建两个定时器模拟两个 topic
      //模拟/cmd_vel这种高频消息
      timer1_ = this->create_wall_timer(10ms,std::bind(&Serial_Node::timer1_callback,this));
      //模拟/set_mode这种低频消息
      timer2_ = this->create_wall_timer(500ms,std::bind(&Serial_Node::timer2_callback,this));

    }

    ~Serial_Node()
    {
      
    }

  private:
    void serial_receive_callback(std::span<const uint8_t> msg)
    {
      std::string str(
          reinterpret_cast<const char*>(msg.data()),
          msg.size()
      );

      RCLCPP_INFO(this->get_logger(),"接收到: %s",str.c_str());
    }

    void timer1_callback()
    {
      // 模拟不断变化的速度命令
      ++cmd_count_;

      fp64 t = cmd_count_ * 0.01;

      uint32_t seq = static_cast<uint32_t>(cmd_count_);

      fp32 vx = static_cast<fp32>(std::sin(t));
      fp32 vy = static_cast<fp32>(std::cos(t));
      fp32 wz = static_cast<fp32>(0.5 * std::sin(t));


      RCLCPP_DEBUG(this->get_logger(),"[ROS] cmd_vel: seq=%u, vx=%.2f, vy=%.2f, wz=%.2f",seq,vx,vy,wz);
      
      std::vector<std::int32_t> int32_vec;
      int32_vec.push_back(std::bit_cast<std::int32_t>(seq));

      std::vector<fp32> fp32_vec;
      fp32_vec.push_back(vx);
      fp32_vec.push_back(vy);
      fp32_vec.push_back(wz);

      
                
      //异步发送数据
      serial_driver.async_write(frame);
    }

    void timer2_callback()
    {
      // 模拟不断变化的模式命令
      ++mode_;

      if (mode_ > 3)
      {
        mode_ = 0;
      }

      RCLCPP_DEBUG(this->get_logger(),"[set_mode] mode=%d",mode_);

      uint32_t seq = ++event_count_;
      int32_t mode = mode_;

      std::vector<int32_t> int32_vec;
      int32_vec.push_back(std::bit_cast<std::int32_t>(seq));
      int32_vec.push_back(mode);


      std::vector<uint8_t> frame;
      frame.push_back(0x66);

      //异步发送数据
      serial_driver.async_write(frame);
      }

    //ROS
    rclcpp::TimerBase::SharedPtr timer1_;
    rclcpp::TimerBase::SharedPtr timer2_;

    //Serial
    serial_transport::Serial_Config serial_config;
    serial_transport::SerialTransport serial_driver;

    //模拟数据
    uint64_t cmd_count_{0};
    int32_t mode_{0};
    uint32_t event_count_{0};

};


int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);

  auto node_ = std::make_shared<Serial_Node>();
  rclcpp::spin(node_);

  rclcpp::shutdown();
  return 0;
}