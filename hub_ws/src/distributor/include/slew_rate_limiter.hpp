#pragma once

#include "rclcpp/rclcpp.hpp"


class slew_rate_limiter{

    private:
        double prev_value{0};
        rclcpp::Node::SharedPtr node;
        double max_change{0};
        rclcpp::Time previous_time;
        
    public:
    /**
     * @param max_change the maximum change of the value in units/second
     * @param Node the node reference to supply
     */
    slew_rate_limiter(double max_change, rclcpp::Node::SharedPtr node){
        this->max_change = max_change;
        this->node = node;
        previous_time = node->get_clock()->now();
    };
    float calculate(double input){

        rclcpp::Time current_time = node->get_clock()->now(); 

        auto elapsed_time = (current_time - previous_time).seconds();
        prev_value += std::clamp(input - prev_value, -max_change * elapsed_time,
                   max_change * elapsed_time);    

        previous_time = current_time;
        
        return prev_value;
    };

};