#include "serial_transport/serial_driver.hpp"
#include "rclcpp/rclcpp.hpp"
#include <asio.hpp>
#include <system_error>
#include <cstddef>
#include <functional>
#include <deque>
#include <vector>

using namespace std::chrono_literals;

class Serial_Node: public rclcpp::Node
{
  public:
    Serial_Node()
      : Node("serial_node_cpp"),
        io_context_(),
        work_guard_(asio::make_work_guard(io_context_)),
        serial_port_(io_context_)
    {
      RCLCPP_INFO(this->get_logger(),"serial_node启动!");

      //==================================================
      // ROS2 参数
      //==================================================

      //声明并设置默认参数
      this->declare_parameter<std::string>("port", "/dev/pts/6");
      this->declare_parameter<int>("baud_rate", 115200);

      //参数读取
      port_name_ = this->get_parameter("port").as_string();
      baud_rate_ = this->get_parameter("baud_rate").as_int();

      //硬参数
      character_size_ = 8;
      parity_ = asio::serial_port_base::parity::none;
      stop_bits_ = asio::serial_port_base::stop_bits::one;
      flow_control_ = asio::serial_port_base::flow_control::none;

      asio::post(io_context_,
          [this]()
          {
              open_serial();
          });
      //==================================================
      // 创建两个定时器模拟两个 topic
      //==================================================

      //模拟/cmd_vel这种高频消息
      timer1_ = this->create_wall_timer(10ms,std::bind(&Serial_Node::timer1_callback,this));
      //模拟/set_mode这种低频消息
      timer2_ = this->create_wall_timer(500ms,std::bind(&Serial_Node::timer2_callback,this));

      //==================================================
      // 创建独立 Asio 线程
      //==================================================
      //这种短任务没必要用std::bind了，直接用lambda最直观
      asio_thread_ = std::jthread([this]()->void
                                    {
                                      RCLCPP_INFO(this->get_logger(),"Asio线程启动");
                                      io_context_.run();
                                      RCLCPP_INFO(this->get_logger(),"Asio线程退出");
                                    });
    }

    ~Serial_Node() override
    {
      RCLCPP_INFO(this->get_logger(),"准备关闭Serial_Node");

      // 不直接在ROS主线程操作serial_port_，防止跨线程操作serial_port_
      // 而是把关闭任务交给Asio线程
      asio::post(io_context_,
        [this]()->void
              {
                stopping_ = true;

                std::error_code ec;

                // 取消重连
                reconnect_timer_.cancel(ec);

                if (serial_port_.is_open())
                {
                  //取消当前这个串口上正在挂起的异步操作
                  serial_port_.cancel(ec);
                  //关闭这个串口设备
                  serial_port_.close(ec);
                }
                send_queue_.clear();

                // 允许io_context在处理完剩余任务之后退出，但这一条不一定比上面cancel()更慢，上面只是投递了任务
                work_guard_.reset();

              });

      //让线程执行join()，但改用jthread后，这两条也可以被取消了
      if (asio_thread_.joinable())
      {
        asio_thread_.join();
      }

      RCLCPP_INFO(this->get_logger(),"Serial_Node关闭完成");
    }


  private:

    //======================================================
    // 主线程ROS线程
    //======================================================
    void timer1_callback()
    {
      // 模拟不断变化的速度命令
      ++cmd_count_;

      fp64 t = cmd_count_ * 0.01;

      fp32 vx = static_cast<fp32>(std::sin(t));
      fp32 vy = static_cast<fp32>(std::cos(t));
      fp32 wz = static_cast<fp32>(0.5 * std::sin(t));

      uint32_t seq = static_cast<uint32_t>(cmd_count_);


      RCLCPP_DEBUG(this->get_logger(),"[ROS] cmd_vel: seq=%u, vx=%.2f, vy=%.2f, wz=%.2f",seq,vx,vy,wz);
      
      //--------------------------------------------------
      // 重点：
      // ROS回调不直接操作串口。
      // 只把“我要发送什么”
      // post给io_context。
      //--------------------------------------------------
      asio::post(io_context_,
        [this,seq,vx,vy,wz]()->void
              {
                //这里是asio的线程

                //模拟一下打包的场景
                std::vector<uint8_t> frame;

                frame.push_back(0xA1);  // timer1标识

                auto append = [&frame](const auto & value)
                {
                  const auto * p = reinterpret_cast<const uint8_t *>(&value);

                  frame.insert(frame.end(),p,p + sizeof(value));
                };

                append(seq);
                append(vx);
                append(vy);
                append(wz);

                frame.push_back(0x1A);  // 尾标识

                //加入发送队列
                enqueue_write(std::move(frame));
              });
      
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

      //--------------------------------------------------
      // 重点：
      // ROS回调不直接操作串口。
      // 只把“我要发送什么”
      // post给io_context。
      //--------------------------------------------------
      asio::post(
        io_context_,
        [this, seq, mode]()
        {
            std::vector<uint8_t> frame;

            frame.push_back(0xB2);  // timer2标识

            auto append = [&frame](const auto & value)
            {
              const auto * p = reinterpret_cast<const uint8_t *>(&value);

              frame.insert(frame.end(),p,p + sizeof(value));
            };

            append(seq);
            append(mode);

            frame.push_back(0x2B);

            enqueue_write(std::move(frame));
        });
      }


    //======================================================
    // TX
    //======================================================

    void enqueue_write(std::vector<uint8_t> frame)
    {
      // 注意：
      // 这个函数只会从Asio线程中调用。

      if (serial_port_.is_open() == false)
      {
        return;
      }

      send_queue_.push_back(std::move(frame));

      // 如果之前没有正在发送的数据
      // 那么启动第一次发送。
      // 如果已经在发送，则只需要排队。
      if (!active_write_) 
      {
        start_async_write();
      }
    }

    void start_async_write()
    {
      // 已经有一个 write 在进行
      if (active_write_)
      {
        return;
      }

      // 已经有一个 write 在进行
      if (send_queue_.empty())
      {
        return;
      }

      // 从等待队列取出一帧
      active_write_ =std::make_shared<std::vector<uint8_t>>(std::move(send_queue_.front()));

      //取出完毕后，把取出去的frame开除队列
      send_queue_.pop_front();

      // 再复制一份 shared_ptr 给 handler
      auto frame = active_write_;

      asio::async_write(serial_port_,asio::buffer(*frame),
      [this,frame](const std::error_code & ec,std::size_t bytes_transferred)->void
      {
        // 当前 write 已经结束
        active_write_.reset();

        if (ec)
        {
          handle_serial_error(ec);
          return;
        }

        //只有在debug状态下才能看到
        RCLCPP_DEBUG(this->get_logger(),"[Asio] 发送完成: %zu bytes",bytes_transferred);

        // 如果队列中还有数据，继续发送下一帧（不过这个在开头判断过了，这里可以优化，但为了代码可读性，没优化掉）
        if (!send_queue_.empty())
        {
          start_async_write();
        }
      });
    }

    //======================================================
    // RX
    //======================================================
    void start_async_read()
    {
      // 注意：
      // 这个函数只会从Asio线程中调用。

      if (serial_port_.is_open() == false)
      {
        return;
      }

      serial_port_.async_read_some(asio::buffer(rx_buffer_),
      std::bind(&Serial_Node::async_read_callback,this,std::placeholders::_1,std::placeholders::_2));
    }

    void async_read_callback(const std::error_code & ec,std::size_t bytes_transferred)
    {
      if(ec)
      {
        handle_serial_error(ec);
        return;
      }
      RCLCPP_INFO(this->get_logger(),"[Asio] 收到 %zu bytes",bytes_transferred);

      // 重新注册下一次异步接收
      start_async_read();
    }
    //======================================================
    // 读写失败处理函数
    //======================================================
    void handle_serial_error(const std::error_code & ec)
    {
      if (stopping_ || ec == asio::error::operation_aborted)
      {
          return;
      }
      RCLCPP_ERROR(this->get_logger(),"串口通信异常: %s",ec.message().c_str());
      std::error_code ignore_ec;

      if (serial_port_.is_open())
      {
          serial_port_.cancel(ignore_ec);
          serial_port_.close(ignore_ec);
      }
      // 当前简单策略：
      // 断线后历史发送数据全部丢弃
      send_queue_.clear();

      schedule_reconnect();
    }

    //======================================================
    // 串口重连机制
    //======================================================
    void open_serial()
    {
      if (stopping_)
      {
          return;
      }

      std::error_code ec;

      //如果现在是开着的，说明是重连，所以先关闭串口
      if (serial_port_.is_open())
      {
        serial_port_.close(ec);
      }

      //打开串口
      serial_port_.open(port_name_, ec);

      if (ec)
      {
        RCLCPP_ERROR(this->get_logger(),"串口打开失败:%s",ec.message().c_str());

        schedule_reconnect();

        return;

      }

      try
      {
        serial_port_.set_option(asio::serial_port_base::baud_rate(baud_rate_));

        serial_port_.set_option(asio::serial_port_base::character_size(character_size_));

        serial_port_.set_option(asio::serial_port_base::parity(parity_));

        serial_port_.set_option(asio::serial_port_base::stop_bits(stop_bits_));

        serial_port_.set_option(asio::serial_port_base::flow_control(flow_control_));

        RCLCPP_INFO(this->get_logger(),"串口打开成功: %s, baud=%d",port_name_.c_str(),baud_rate_);
      }
      catch (const std::system_error & e)
      {
        RCLCPP_ERROR(this->get_logger(),"串口配置失败: %s",e.what());

        std::error_code ignore_ec;
        serial_port_.close(ignore_ec);

        schedule_reconnect();
        return;
      }

      start_async_read();

    }

    void schedule_reconnect()
    {
      if (stopping_ || reconnect_pending_)
      {
          return;
      }
      reconnect_pending_ = true;

      reconnect_timer_.expires_after(1s);
      reconnect_timer_.async_wait([this](const std::error_code & ec)->void
      {

        reconnect_pending_ = false;

        if (ec == asio::error::operation_aborted)
        {
          return;
        }
        if (ec)
        {
          RCLCPP_ERROR(this->get_logger(),"重连定时器错误: %s",ec.message().c_str());return;
        }

        RCLCPP_INFO(this->get_logger(),"尝试重新连接串口...");
        open_serial();
      });
    }


    //======================================================
    // ROS
    //======================================================
    rclcpp::TimerBase::SharedPtr timer1_;
    rclcpp::TimerBase::SharedPtr timer2_;

    uint64_t cmd_count_{0};
    int32_t mode_{0};
    uint32_t event_count_{0};

    //======================================================
    // Asio
    //======================================================
    asio::io_context io_context_;
    asio::executor_work_guard<asio::io_context::executor_type> work_guard_;
    asio::serial_port serial_port_;
    std::jthread asio_thread_;

    //重连机制
    asio::steady_timer reconnect_timer_{io_context_};
    bool stopping_{false};
    bool reconnect_pending_{false};

    //======================================================
    // Serial
    //======================================================
    std::string port_name_;
    uint32_t baud_rate_;
    uint32_t character_size_;
    asio::serial_port_base::parity::type parity_;
    asio::serial_port_base::stop_bits::type stop_bits_;
    asio::serial_port_base::flow_control::type flow_control_;

    //RX
    std::array<uint8_t, 1024> rx_buffer_{};

    //TX
    std::deque<std::vector<uint8_t>> send_queue_;
    std::shared_ptr<std::vector<uint8_t>> active_write_; //防止send_queue_.clear()造成的buffer 悬空风险


};

int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);

  auto node_ = std::make_shared<Serial_Node>();
  rclcpp::spin(node_);

  rclcpp::shutdown();
  return 0;
}
