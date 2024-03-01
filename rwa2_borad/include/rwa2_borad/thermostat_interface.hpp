#pragma once
#include<rclcpp/rclcpp.hpp>
#include<std_msgs/msg/int32.hpp>
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include<memory.h>
#include<random>
/**
 * @brief creating the namespace
 * 
 */
namespace rwa2{
    /**
     * @brief defineing the class ThermostateInterface and inhereting public of Class Node
     * 
     */
    class ThermostateInterface: public rclcpp::Node{
        public:
        //creating constructor
        ThermostateInterface(std::string node_name): Node(node_name){
            // Generating random number for seeder
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<> dis(1,1000);
            //  seeding random number
            srand(dis(gen));
            //-----------------
            //PARAMETERS
            //-----------------
            this->declare_parameter("thermostat_name","thermostat");
            this->declare_parameter("target_temperature",30);
            this->declare_parameter("mode","away");
            mode_ = this->get_parameter("mode").as_string();
            thermostate_name_ = this->get_parameter("thermostat_name").as_string();
            target_temperature_ = this->get_parameter("target_temperature").as_int();

            // Parameter callback
            parameter_cb_= this->add_on_set_parameters_callback(std::bind(&rwa2::ThermostateInterface::parameter_cb,this,std::placeholders::_1));
            generate_random_temp();
            // Timer for parameter call back
            timer3_ = this->create_wall_timer(std::chrono::seconds(10), std::bind(&rwa2::ThermostateInterface::parameter_timer_cb, this));


            //initiating attribute publisher_
            publisher_ =this->create_publisher<std_msgs::msg::Int32>("temperature", 10); //creating publisher with msg type bool and ( topic , buffer size)
            timer_ = this->create_wall_timer(std::chrono::milliseconds(333), std::bind(&rwa2::ThermostateInterface::thermostat_timer_cb, this));
            
            //initiating attribute subscriber_
            subscriber_=this->create_subscription<std_msgs::msg::Int32>("temperature",10, std::bind(&rwa2::ThermostateInterface::thermostat_sub_cb,this, std::placeholders::_1));
            timer2_ = this->create_wall_timer(std::chrono::milliseconds(333), std::bind(&rwa2::ThermostateInterface::thermostat_sub_timer_cb, this));
            

        }       
        private:
        //Creating an attribute named publisher_ of type shown below
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
        //creatimg timer attributed
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr timer2_;
        rclcpp::TimerBase::SharedPtr timer3_;
        //Creating an attribute named subscriber_ of type shown below
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscriber_;
        //Creating aatribute for pamaetercallback handler
        OnSetParametersCallbackHandle::SharedPtr parameter_cb_;
        std_msgs::msg::Int32 current_temperature_;
        //Attribute for the parameters
        std::string thermostate_name_;
        std::string mode_;

        //Attributes for the meathods
        int target_temperature_;
        int received_temperature_;
        /**
         * @brief Subscriber callback method
         * 
         * @param msg argument passed of the same topic message type 
         */
        void thermostat_sub_cb(const std_msgs::msg::Int32::SharedPtr msg);
        /**
         * @brief Parameter callback method to update the parameter value as well as update its attribute during runtime
         * 
         * @param parameters Parsed the vector of parameters and its values to be updated
         * @return rcl_interfaces::msg::SetParametersResult returns if the task is sucessfulty completed or not
         */
        rcl_interfaces::msg::SetParametersResult parameter_cb(const std::vector<rclcpp::Parameter>& parameters);
        /**
         * @brief Thermostat timer callback method
         * 
         */
        void thermostat_timer_cb();
        /**
         * @brief Thermostat subscriber timer callback
         * 
         */
        void thermostat_sub_timer_cb();
        /**
         * @brief parameter timer callback method
         * 
         */
        void parameter_timer_cb();
        /**
         * @brief for genertating random temperature for the first time
         * 
         */
        void generate_random_temp();



    };// Class ThermostateInterface
}//Namespace rwa2