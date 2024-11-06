// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
// #include <cerror.h>
#include <csignal>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "CSVPublisher.hpp"

using namespace std::chrono_literals;

std::atomic_bool stopped;

class TrajectoryFromFileNode : public rclcpp::Node {
  public:
    TrajectoryFromFileNode()
        : Node("trajectory_from_file_pub"), count_(0) {
        {
            auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
            param_desc.description = "Path to the input CSV file";
            this->declare_parameter("input_file", "", param_desc);
        }
        {
            auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
            param_desc.description = "The name of the topic the data will be published on. Default \"/trajectory_from_file/joint_command\".";
            this->declare_parameter("output_topic_name", "/trajectory_from_file/joint_command", param_desc);
        }
        {
            auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
            param_desc.description = "Time in seconds between outputting the samples.";
            this->declare_parameter("sample_period", 0.001, param_desc);
        }
        {
            auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
            param_desc.description = "Set a value between 0 and 1 so reduce output frequency. Set value >1 to increase output speed.";
            this->declare_parameter("speed_factor", 1.0, param_desc);
        }
        /*{
            auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
            param_desc.description = "auto|fixed|file";
            this->declare_parameter("time_source", "", param_desc);
        }*/
        
        signal(SIGTERM, [](int signum) {
            stopped.store(true);
        });

        t = now();
        delta_t.reserve(1000000);
    }
    
    ~TrajectoryFromFileNode() {
        RCLCPP_INFO(this->get_logger(), "~TrajectoryFromFileNode()");
    }
    
    void configure() {
        auto input_file = this->get_parameter("input_file").as_string();  
        auto port_name = this->get_parameter("output_topic_name").as_string();
        std::shared_ptr<rclcpp::Node> nodep = shared_from_this();
        std::vector<std::string> joint_names;
        csv = std::make_unique<trajectory_from_file::JointPositionsFromCSVPublisher>(input_file,port_name,nodep,joint_names);
        
        sample_period = this->get_parameter("sample_period").as_double();     
        speed_factor = this->get_parameter("speed_factor").as_double();        
        
        if (sample_period <= 0 || !std::isfinite(sample_period)) {
            throw std::runtime_error("Invalid output period "+ std::to_string(sample_period));
        }
        
        if (speed_factor <= 0 || !std::isfinite(speed_factor)) {
            throw std::runtime_error("Invalid speed factor "+ std::to_string(speed_factor));
        }
        
        RCLCPP_INFO(this->get_logger(), "Setting up timer at period %.6fs.", sample_period/speed_factor);
        
        output_period_us = (unsigned int)(sample_period/speed_factor*1e6);
        
        /*timer_ = this->create_wall_timer(
            std::chrono::duration<double, std::ratio<1>>(sample_period/speed_factor), 
            std::bind(&TrajectoryFromFileNode::timer_callback, this)
        );*/
    }

    void update() {
        auto t_last = t;
        t = now();
        
        rclcpp::Duration delta = t - t_last;;
        delta_t.push_back((int)(delta.seconds()*1e6));
        csv->update();
    }

    void busy_cycle() {
        t = now();
        std::this_thread::sleep_for(std::chrono::microseconds{output_period_us});
        auto t_next = now();
        while (!csv->isFinished() && !stopped.load()) {
            update();
            
            //rclcpp::spin_node_once(shared_from_this(), std::chrono::microseconds{output_period_us/2});
            
            t_next += std::chrono::microseconds{output_period_us};
            while( ((int)((now() - t_next).seconds()*1e6)) < 10/2 ) {
                std::this_thread::sleep_for(std::chrono::microseconds{10});
            }
        }        
    }

    void writeTimes() {
        
        time_file.open("/tmp/time_file.txt");
        time_file<<"error_dur"<<std::endl;
        for(auto x : delta_t) {
            time_file<<x<<std::endl;
        }
        time_file.close();
    }
    
  private:
    void timer_callback() {
        //auto message = std_msgs::msg::String();
        //message.data = "Hello, world! " + std::to_string(count_++);
        //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        //publisher_->publish(message);
        update();
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
    std::unique_ptr<trajectory_from_file::JointPositionsFromCSVPublisher> csv;
    std::ofstream time_file;
    rclcpp::Time t = now();
    std::vector<int> delta_t;
    
    double sample_period = 1.0;
    double speed_factor = 1.0;
    unsigned int output_period_us = 1000;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    {
        auto ptr = std::make_shared<TrajectoryFromFileNode>();
        ptr->configure();
        RCLCPP_INFO(ptr->get_logger(), "Init csv done.");
        
        struct sched_param p;
        p.sched_priority = 99;
        std::cerr << "enabling SCHED_FIFO with priority "
                << p.sched_priority << "\n";
        int r = sched_setscheduler(0, SCHED_FIFO, &p);
        if (r == -1) {
            std::cerr << "sched_setscheduler() failed: '" << strerror(errno)
                    << "'\n";
            //exit(EXIT_FAILURE);
        }

        
        ptr->busy_cycle();
        //rclcpp::executors::StaticSingleThreadedExecutor executor;
        //executor.add_node(ptr);
        //executor.spin();

        //rclcpp::spin(ptr);
        rclcpp::shutdown();
        ptr->writeTimes();
    }
    return 0;
}
