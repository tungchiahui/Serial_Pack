#include "rclcpp/rclcpp.hpp"
#include "serial_transport/serial_transport.hpp"
#include <cstdint>
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
      serial_config_.port_name_ = this->get_parameter("port").as_string();
      serial_config_.baud_rate_ = this->get_parameter("baud_rate").as_int();
      //硬参数
      serial_config_.character_size_ = 8;
      serial_config_.parity_ = asio::serial_port_base::parity::none;
      serial_config_.stop_bits_ = asio::serial_port_base::stop_bits::one;
      serial_config_.flow_control_ = asio::serial_port_base::flow_control::none;

      //串口包协议回调函数设置
      if(!protocol_.set_unpack_callback(0x01, &Serial_Node::handle_cmd_vel, this))
      {
        RCLCPP_ERROR(this->get_logger(), "cmd_vel 协议回调注册失败");
        return;
      }
      if(!protocol_.set_unpack_callback(0x02, &Serial_Node::handle_set_mode, this))
      {
        RCLCPP_ERROR(this->get_logger(), "set_mode 协议回调注册失败");
        return;
      }

      //回调注册完毕后，再开启串口与异步接收
      serial_driver_.start(serial_config_, std::bind(&Serial_Node::serial_receive_callback,this,std::placeholders::_1));

      // 创建两个定时器模拟两个 topic
      //模拟/cmd_vel这种高频消息
      timer1_ = this->create_wall_timer(10ms,std::bind(&Serial_Node::timer1_callback,this));
      //模拟/set_mode这种低频消息
      timer2_ = this->create_wall_timer(500ms,std::bind(&Serial_Node::timer2_callback,this));

    }

  private:
    void handle_cmd_vel(std::uint32_t seq, float vx, float vy, float wz)
    {
      RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "[RX cmd_vel] seq=%u vx=%.3f vy=%.3f wz=%.3f",
          static_cast<uint32_t>(seq), vx, vy, wz);
    }

    void handle_set_mode(std::uint32_t seq, std::int32_t mode)
    {
      RCLCPP_INFO(this->get_logger(), "[RX set_mode] seq=%u mode=%d",
                  static_cast<uint32_t>(seq), static_cast<int32_t>(mode));
    }


    void serial_receive_callback(std::span<const uint8_t> msg)
    {
      //解包
      protocol_.feed(msg);
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
      
      auto frame = protocol_.pack(0x01, seq,vx,vy,wz);
                
      //异步发送数据
      serial_driver_.async_write(frame);
    }

    void timer2_callback()
    {
      // 模拟不断变化的模式命令
      ++mode_;

      if (mode_ > 3)
      {
        mode_ = 0;
      }

      uint32_t seq = ++event_count_;

      RCLCPP_DEBUG(this->get_logger(),"[set_mode] seq = %u,mode=%d",seq,mode_);

      auto frame = protocol_.pack(0x02, seq,mode_);

      //异步发送数据
      serial_driver_.async_write(frame);
      }

    //ROS
    rclcpp::TimerBase::SharedPtr timer1_;
    rclcpp::TimerBase::SharedPtr timer2_;

    //注意，protocol_一定要比serial_driver_早。
    // protocol_ 先构造、后析构；串口析构时先 stop() 并等待接收线程退出。
    wire_protocol::CallbackProtocol protocol_;

    //Serial
    serial_transport::Serial_Config serial_config_;
    serial_transport::SerialTransport serial_driver_;

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
