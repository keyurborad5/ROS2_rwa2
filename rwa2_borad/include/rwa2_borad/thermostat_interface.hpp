#pragma once
#include<rclcpp/rclcpp.hpp>
#include<std_msgs/msg/int32.hpp>
#include<memory.h>

namespace rwa2{
    class ThermostateInterface: public rclcpp::Node{
        public:
        //creating constructor
        ThermostateInterface(std::string node_name): Node(node_name){
            //initiating attribute publisher_
            publisher_ =this->create_publisher<std_msgs::msg::Int32>("temperature", 10); //creating publisher with msg type bool and ( topic , buffer size)
            timer_ = this->create_wall_timer(std::chrono::milliseconds(333), std::bind(&rwa2::ThermostateInterface::thermostat_timer_cb, this));
            subscriber_=this->create_subscription<std_msgs::msg::Int32>("temperature",10, std::bind(&rwa2::ThermostateInterface::thermostat_sub_cb,this, std::placeholders::_1));
            
        }
        private:
        //Creating an attribute named publisher_ of type shown below
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscriber_;
        void thermostat_sub_cb(const std_msgs::msg::Int32::SharedPtr msg);


        std_msgs::msg::Int32 current_temperature_;

        void thermostat_timer_cb();



    };// Class ThermostateInterface
}//Namespace rwa2