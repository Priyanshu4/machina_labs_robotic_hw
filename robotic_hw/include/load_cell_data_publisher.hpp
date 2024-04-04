#ifndef LOAD_CELL_DATA_PUBLISHER_HPP_
#define LOAD_CELL_DATA_PUBLISHER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "robotic_hw_interfaces/srv/get_load_cell_data.hpp"
#include "robotic_hw_interfaces/msg/load_cell_data_array.hpp"
#include <vector>
#include <memory>

class LoadCellDataPublisher : public rclcpp::Node
{
public:
    LoadCellDataPublisher();

private:
    void publish_timer_callback();
    void service_call_timer_callback();
    void handle_service_response(rclcpp::Client<robotic_hw_interfaces::srv::GetLoadCellData>::SharedFuture future, size_t index);

    std::vector<std::string> service_names_;
    double publish_frequency_;
    double service_call_frequency_;
    std::vector<rclcpp::Client<robotic_hw_interfaces::srv::GetLoadCellData>::SharedPtr> load_cell_clients_;
    rclcpp::Publisher<robotic_hw_interfaces::msg::LoadCellDataArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::TimerBase::SharedPtr service_call_timer_;
    robotic_hw_interfaces::msg::LoadCellDataArray load_cell_data_array_msg_;
};

#endif // LOAD_CELL_DATA_PUBLISHER_HPP_
