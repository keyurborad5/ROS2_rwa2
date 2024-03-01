#include"thermostat_interface.hpp"
#include <cstdlib>

//Method implementation for runtime updation of the attribute values as parameter values is updated from CLI  
rcl_interfaces::msg::SetParametersResult rwa2::ThermostateInterface::parameter_cb(const std::vector<rclcpp::Parameter>& parameters){
    rcl_interfaces::msg::SetParametersResult result;
    result.successful= true;
    result.reason="success";
    for (const auto &param : parameters){
        //enters only if the parameter name is mode
        if(param.get_name()=="mode"){
            mode_=param.as_string();
            //enter only if the parameter value is eco
            if (mode_=="eco"){
                //updating target temperature as instructed
                target_temperature_= this->get_parameter("target_temperature").as_int();
                target_temperature_-=3;
                //verifying that the target temperature stays inthe bound of (0-40)
                if (target_temperature_<0){
                    target_temperature_=0;
                }
                else if (target_temperature_>40){
                    target_temperature_=40;
                }
                RCLCPP_INFO_STREAM(this->get_logger(),"Target temperature Changed to: "<<target_temperature_);

            }
            //Enters if the parameter value is night
            else if (mode_=="night"){
                //updating the target temperature value as instructed
                target_temperature_=18;
                RCLCPP_INFO_STREAM(this->get_logger(),"Target temperature Changed to : "<<target_temperature_);
            }
        }
        // Logging the warning that user cannot alter these parameter values
        else if (param.get_name()=="thermostat_name"){
            RCLCPP_WARN_STREAM(this->get_logger(),"This is not allowed");
        }
        else if (param.get_name()=="target_temperature"){
            RCLCPP_WARN_STREAM(this->get_logger(),"This is not allowed");
        }
        else{
            result.successful = false;
            result.reason = "parameter name not found";
        }
    }
    return result;
}

//Method for generrating the random temperature for the first time of running code
void rwa2::ThermostateInterface::generate_random_temp(){
        current_temperature_.data=rand()%41;
        RCLCPP_INFO_STREAM(this->get_logger(),"Current temperature :"<< current_temperature_.data);
        if (mode_=="away"){
            this->set_parameter(rclcpp::Parameter("mode","away"));
            RCLCPP_INFO_STREAM(this->get_logger(),"Started with mode away");
        }
        else if (mode_=="night"){
            this->set_parameter(rclcpp::Parameter("mode","night"));
            RCLCPP_INFO_STREAM(this->get_logger(),"Started with mode Night");
        }
        else if (mode_=="eco"){
            this->set_parameter(rclcpp::Parameter("mode","eco"));
            RCLCPP_INFO_STREAM(this->get_logger(),"Started with mode Eco");
        }

}

//Method implementation of parameter timer callback which changes the mode every 10sec as per the logic defined in the method below
void rwa2::ThermostateInterface::parameter_timer_cb(){
    if (mode_=="away"){
        RCLCPP_INFO_STREAM(this->get_logger(),"Mode changed to night");
        this->set_parameter(rclcpp::Parameter("mode","night"));
    }
    else if (mode_=="night"){
        RCLCPP_INFO_STREAM(this->get_logger(),"Mode changed to eco");
        this->set_parameter(rclcpp::Parameter("mode","eco"));
    }
    else if (mode_=="eco"){
        RCLCPP_INFO_STREAM(this->get_logger(),"Mode changed to away");
        this->set_parameter(rclcpp::Parameter("mode","away"));
    }

}

//Method implementation of the thermostat timer publisher callback
void rwa2::ThermostateInterface::thermostat_timer_cb(){
    //stops publishing and updating current temperature once reached at target temperature or selected mode is away
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
        // RCLCPP_INFO_STREAM(this->get_logger(),"Generated temperature :"<< current_temperature_.data);
    }

}

//Method implementation of thermostat subscriber callback
void rwa2::ThermostateInterface::thermostat_sub_cb(const std_msgs::msg::Int32::SharedPtr msg ){
    // RCLCPP_INFO_STREAM(this->get_logger(),"Current temperature :"<< msg->data);
    received_temperature_=msg->data;
    // RCLCPP_INFO_STREAM(this->get_logger(),"target_temperature :"<< target_temperature_);
}

//Method implementation of the thermostat subscriber timer callback for continous logging to output screen unless mode is  away
void rwa2::ThermostateInterface::thermostat_sub_timer_cb(){
    if (mode_!="away" && received_temperature_!=0){
        RCLCPP_INFO_STREAM(this->get_logger(),"Current temperature :"<< received_temperature_);
    }
}
// Main method implementation
int main(int argc, char** argv){
    rclcpp::init(argc,argv);
    auto node=std::make_shared<rwa2::ThermostateInterface>("thermostat_house");
    rclcpp::spin(node);
    rclcpp::shutdown();
}