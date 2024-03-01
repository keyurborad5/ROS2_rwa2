#include"thermostat_interface.hpp"
#include <cstdlib>

    rcl_interfaces::msg::SetParametersResult rwa2::ThermostateInterface::parameter_cb(const std::vector<rclcpp::Parameter>& parameters){
        rcl_interfaces::msg::SetParametersResult result;
        result.successful= true;
        result.reason="success";
        for (const auto &param : parameters){
            if(param.get_name()=="mode"){
                mode_=param.as_string();
                if (mode_=="eco"){
                    target_temperature_-=3;
                    if (target_temperature_<0){
                        target_temperature_=0;
                    }
                    else if (target_temperature_>40){
                        target_temperature_=40;
                    }
                }
                else if (mode_=="night"){
                    target_temperature_=18;
                }     
            }
            else if (param.get_name()=="thermostat_name"){
                RCLCPP_INFO_STREAM(this->get_logger(),"This is not allowed");
            }
            else if (param.get_name()=="target_temperature"){
                RCLCPP_INFO_STREAM(this->get_logger(),"This is not allowed");
            }
            else{
                result.successful = false;
                result.reason = "parameter name not found";
            }
        }
        return result;
    }


void rwa2::ThermostateInterface::generate_random_temp(){
        current_temperature_.data=rand()%41;
}

void rwa2::ThermostateInterface::thermostat_timer_cb(){
   
    if (mode_!="away" && current_temperature_.data!=target_temperature_){
        //generating random number and publishing it
        // current_temperature_.data=rand()%41;
        
        if (current_temperature_.data>target_temperature_){
            current_temperature_.data-=1;
        }
        else if (current_temperature_.data<target_temperature_){
            current_temperature_.data+=1;
        }
        publisher_->publish(current_temperature_);
        RCLCPP_INFO_STREAM(this->get_logger(),"Generated temperature :"<< current_temperature_.data);
    }

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