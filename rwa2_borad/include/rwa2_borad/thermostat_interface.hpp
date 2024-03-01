#pragma once
#include<rclcpp/rclcpp.hpp>
#include<std_msgs/msg/int32.hpp>
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include<memory.h>

namespace rwa2{
    class ThermostateInterface: public rclcpp::Node{
        public:
        //creating constructor
        ThermostateInterface(std::string node_name): Node(node_name){

            srand(time(NULL));
            //-----------------
            //PARAMETERS
            //-----------------
            this->declare_parameter("thermostat_name","thermostat");
            this->declare_parameter("target_temperature",30);
            this->declare_parameter("mode","away");
            mode_ = this->get_parameter("mode").as_string();
            thermostate_name_ = this->get_parameter("thermostat_name").as_string();
            target_temperature_ = this->get_parameter("target_temperature").as_int();

            parameter_cb_= this->add_on_set_parameters_callback(std::bind(&rwa2::ThermostateInterface::parameter_cb,this,std::placeholders::_1));
            generate_random_temp();
            timer3_ = this->create_wall_timer(std::chrono::seconds(10), std::bind(&rwa2::ThermostateInterface::parameter_timer_cb, this));


            //initiating attribute publisher_
            publisher_ =this->create_publisher<std_msgs::msg::Int32>("temperature", 10); //creating publisher with msg type bool and ( topic , buffer size)
            timer_ = this->create_wall_timer(std::chrono::milliseconds(333), std::bind(&rwa2::ThermostateInterface::thermostat_timer_cb, this));
            subscriber_=this->create_subscription<std_msgs::msg::Int32>("temperature",10, std::bind(&rwa2::ThermostateInterface::thermostat_sub_cb,this, std::placeholders::_1));
            timer2_ = this->create_wall_timer(std::chrono::milliseconds(333), std::bind(&rwa2::ThermostateInterface::thermostat_sub_timer_cb, this));
            

        }       
        private:
        //Creating an attribute named publisher_ of type shown below
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr timer2_;
        rclcpp::TimerBase::SharedPtr timer3_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscriber_;
        void thermostat_sub_cb(const std_msgs::msg::Int32::SharedPtr msg);

        OnSetParametersCallbackHandle::SharedPtr parameter_cb_;
        std_msgs::msg::Int32 current_temperature_;
        std::string thermostate_name_;
        std::string mode_;

        int target_temperature_;
        int received_temperature_;
        rcl_interfaces::msg::SetParametersResult parameter_cb(const std::vector<rclcpp::Parameter>& parameters);
        void thermostat_timer_cb();
        void thermostat_sub_timer_cb();
        void parameter_timer_cb();
        void generate_random_temp();



    };// Class ThermostateInterface
}//Namespace rwa2