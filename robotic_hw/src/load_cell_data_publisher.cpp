#include "load_cell_data_publisher.hpp"
#include <chrono>
#include <functional>

using namespace std::chrono_literals;

LoadCellDataPublisher::LoadCellDataPublisher() : Node("load_cell_data_publisher")
{
    // Create parameter for names of services to call
    this->declare_parameter<std::vector<std::string>>("service_names", {"get_load_cell_data"});
    this->get_parameter("service_names", service_names_);

    // Create parameter for the publish frequency (500 Hz by default)
    this->declare_parameter<double>("publish_frequency", 500.0);
    this->get_parameter("publish_frequency", publish_frequency_);

    // Create parameter for the service call frequency 
    this->declare_parameter<double>("service_call_frequency", 200.0);
    this->get_parameter("service_call_frequency", service_call_frequency_);

    // Create clients for each service
    for (const auto& service_name : service_names_)
    {
        auto client = this->create_client<robotic_hw_interfaces::srv::GetLoadCellData>(service_name);
        load_cell_clients_.push_back(client);
        while (!client->wait_for_service(1s))
        {
            RCLCPP_WARN(this->get_logger(), "%s service not available, waiting...", service_name.c_str());
        }
        RCLCPP_INFO(this->get_logger(), "%s service available.", service_name.c_str());
    }

    load_cell_data_array_msg_.load_cells.resize(service_names_.size());

    publisher_ = this->create_publisher<robotic_hw_interfaces::msg::LoadCellDataArray>("load_cells", 10);
    publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000 / publish_frequency_)),
        std::bind(&LoadCellDataPublisher::publish_timer_callback, this));

    service_call_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000 / service_call_frequency_)),
        std::bind(&LoadCellDataPublisher::service_call_timer_callback, this));
}

void LoadCellDataPublisher::publish_timer_callback()
{
    publisher_->publish(load_cell_data_array_msg_);
}

void LoadCellDataPublisher::service_call_timer_callback()
{
    for (size_t i = 0; i < load_cell_clients_.size(); ++i)
    {
        if (!load_cell_clients_[i]->service_is_ready())
        {
            RCLCPP_ERROR(this->get_logger(), "Load cell data service %zu not available.", i);
            load_cell_data_array_msg_.load_cells[i].data_ready = false;
            continue;
        }

        auto request = std::make_shared<robotic_hw_interfaces::srv::GetLoadCellData::Request>();
        auto result_future = load_cell_clients_[i]->async_send_request(request,
            [this, i](std::shared_future<std::shared_ptr<robotic_hw_interfaces::srv::GetLoadCellData::Response>> future) {
            this->handle_service_response(future, i); // Use 'this' to call a member function
            });
    }
}

void LoadCellDataPublisher::handle_service_response(rclcpp::Client<robotic_hw_interfaces::srv::GetLoadCellData>::SharedFuture future, size_t index)
{
    auto response = future.get();
    load_cell_data_array_msg_.load_cells[index] = response->load_cell_data;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LoadCellDataPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
