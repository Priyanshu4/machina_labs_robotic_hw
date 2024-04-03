import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from functools import partial
from robotic_hw_interfaces.srv import GetLoadCellData
from robotic_hw_interfaces.msg import LoadCellDataArray, LoadCellData

class LoadCellDataPublisher(Node):
    """ Calls GetLoadCellData services to get the latest load cell data and publishes it at specified frequency.
        The services are specified as parameters to this node, allowiing the node to publish information from any number of load cells.
        Calling services and publishing are decoupled in different timers. This means old data may be published several times if the service call is slow.
    """

    def __init__(self) -> None:
        super().__init__('load_cell_data_publisher')


        # Create parameter for service names
        self.declare_parameter('service_names', ['get_load_cell_data']) 
        self.service_names = self.get_parameter('service_names').get_parameter_value().string_array_value

        # Create parameter for publish frequency (default 500 Hz)
        self.declare_parameter('publish_frequency', 500.0)
        self.publish_frequency = self.get_parameter('publish_frequency').get_parameter_value().double_value

        # Create parameter for service call frequency (default 500 Hz)
        self.declare_parameter('service_call_frequency', 200.0)
        self.service_call_frequency = self.get_parameter('service_call_frequency').get_parameter_value().double_value

        # Create a client for each service
        self.load_cell_clients = []
        for service_name in self.service_names:
            client = self.create_client(GetLoadCellData, service_name)
            self.load_cell_clients.append(client)
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn(f'{service_name} service not available, waiting...')
            self.get_logger().info(f'{service_name} service available.')
        self.futures = set() # Store the active futures so they are not garbage collected before they complete

        # Create request object (same for all services)
        self.req = GetLoadCellData.Request()

        # Create publisher
        self.publisher_ = self.create_publisher(LoadCellDataArray, 'load_cells', qos_profile_sensor_data)
        
        # Define message to publish. Service call timer will update this message and publish_timer will publish it.
        self.load_cell_data_array_msg = LoadCellDataArray() 
        self.load_cell_data_array_msg.load_cells = [LoadCellData(data_ready=False) for _ in self.service_names]

        # Create timer to call service and store results
        self.service_call_timer = self.create_timer(1/self.service_call_frequency, self.service_call_timer_callback)

        # Create timer to publish data
        self.publish_timer = self.create_timer(1/self.publish_frequency, self.publish_timer_callback)

    def publish_timer_callback(self) -> None:
        """ Publishes the latest load cell data.
        """
        self.publisher_.publish(self.load_cell_data_array_msg)

    def service_call_timer_callback(self) -> None:
        """ Calls the GetLoadCellData service asynchronously for each load cell.
            The future callback will process the result and store it in the LoadCellDataArray message.
        """
        # For each client, call the service asynchronously
        for i, client in enumerate(self.load_cell_clients): 
            if client.service_is_ready():
                future = client.call_async(self.req)
                future.add_done_callback(partial(self.future_callback, client_index=i))
                self.futures.add(future)
            else:
                # If the service is not available, log an error and set the future to None
                # Set data_ready to False for that LoadCellData message
                self.get_logger().error(f'Load cell data service {i} not available.')
                self.load_cell_data_array_msg.load_cells[i] = LoadCellData(data_ready=False) 
                return

    def future_callback(self, future, client_index: int = None) -> None:
        """ When the service call is done, this function is called to process the result.
            We store the result in the LoadCellDataArray message  at index client_index.
        """
        try:
            self.load_cell_data_array_msg.load_cells[client_index] = future.result().load_cell_data
            self.futures.remove(future)
        except Exception as e:
            self.get_logger().error(f'Exception while calling service: {e}')
            return
        
def main(args=None) -> None:
    rclpy.init(args=args)
    load_cell_data_client = LoadCellDataPublisher()

    try:
        rclpy.spin(load_cell_data_client)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()
        load_cell_data_client.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()