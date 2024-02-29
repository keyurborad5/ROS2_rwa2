#include"thermostat_interface.hpp"
#include <cstdlib>

void rwa2::ThermostateInterface::thermostat_timer_cb(){
   
    //generating random number and publishing it
    current_temperature_.data=rand()%41;
    publisher_->publish(current_temperature_);
    RCLCPP_INFO_STREAM(this->get_logger(),"Generated temperature :"<< current_temperature_.data);

}

void rwa2::ThermostateInterface::thermostat_sub_cb(const std_msgs::msg::Int32::SharedPtr msg ){
    RCLCPP_INFO_STREAM(this->get_logger(),"Current temperature :"<< msg->data);
    RCLCPP_INFO_STREAM(this->get_logger(),"Thermostate name :"<< thermostate_name_);
    RCLCPP_INFO_STREAM(this->get_logger(),"target_temperature :"<< target_temperature_);


}

int main(int argc, char** argv){
    rclcpp::init(argc,argv);
    auto node=std::make_shared<rwa2::ThermostateInterface>("thermostat_house");
    rclcpp::spin(node);
    rclcpp::shutdown();
}