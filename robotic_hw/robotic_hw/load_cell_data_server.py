import rclpy
from rclpy.node import Node
import socket
import numpy as np
from typing import Optional
from builtin_interfaces.msg import Time
from robotic_hw_interfaces.srv import GetLoadCellData

class LoadCellDataServer(Node):
    """ Interfaces with the load cell through a socket and exposes filtered data through the GetLoadCellData service.
    """

    def __init__(self) -> None:
        super().__init__('load_cell_data_server')
        
        # Parameters
        self.declare_parameter('server_address', '127.0.0.3')
        self.declare_parameter('server_port', 10000)
        self.declare_parameter('number_of_samples', 5)
        self.declare_parameter('request_frequency', 200.0)

        self.sensor_server_address = self.get_parameter('server_address').get_parameter_value().string_value
        self.sensor_server_port = self.get_parameter('server_port').get_parameter_value().integer_value
        self.number_of_samples = self.get_parameter('number_of_samples').get_parameter_value().integer_value
        self.request_frequency = self.get_parameter('request_frequency').get_parameter_value().double_value

        # Init Service
        self.srv = self.create_service(GetLoadCellData, 'get_load_cell_data', self.get_load_cell_data_callback)
        
        # Create socket to read data from the load cell
        self.socket_connected = False
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connect_socket()

        # Store the latest filtered sample from the load cell
        self.latest_filtered_sample: Optional[np.ndarray] = None
        self.latest_sample_timestamp: Optional[Time] = None
            
        # Run a timer to request samples from the load cell
        self.sample_timer = self.create_timer(1/self.request_frequency, self.request_samples)

        # Run a timer to periodically retry connecting to the sensor in case of socket disconnect
        self.retry_socket_connect = self.create_timer(3.0, self.connect_socket)

    def get_load_cell_data_callback(self, request: GetLoadCellData.Request, response: GetLoadCellData.Response) -> None:
        """ Callback for the get_load_cell_data service. 
            Returns the latest filtered sample from the load cell.
        """
        if not self.socket_connected or self.latest_filtered_sample is None:
            response.data_ready = False
            response.force_x, response.force_y, response.force_z = 0.0, 0.0, 0.0
            response.timestamp = Time()
        else:
            response.data_ready = True
            response.force_x, response.force_y, response.force_z = self.latest_filtered_sample[:3]
            response.timestamp = self.latest_sample_timestamp

        return response   
    
    def connect_socket(self) -> None:
        """ Attempts to connect the socket to the sensor.
            Sets self.socket_connected to True if successful, False otherwise. 
        """
        if not self.socket_connected:
            try:
                self.sock.connect((self.sensor_server_address, self.sensor_server_port))
                self.get_logger().info(f"Connecting to {self.sensor_server_address} port {self.sensor_server_port}")
            except (ConnectionRefusedError, ConnectionAbortedError):
                self.get_logger().error(f"Connection to {self.sensor_server_address} port {self.sensor_server_port} refused. Will retry periodically.")
                self.socket_connected = False
                return 
            self.get_logger().info(f"Connected to {self.sensor_server_address} port {self.sensor_server_port}")
            self.socket_connected = True
            
    def request_samples(self) -> None:
        """ Requests samples from the load cell. Called periodically by the timer.
            Averages the samples to remove noise and stores in self.latest_filtered_sample
            Stores the timestamp at which the data is received in self.latest_sample_timestamp

            Here I chose to average the samples because the assignment that the service makes filtered data avaiable.
            Taking the mean over the most recent samples is a simple example of a low-pass filter for noise reduction. 
        """

        if not self.socket_connected:
            return
            
        # Try to request samples from the load cell
        message_string = str(self.number_of_samples)
        message = message_string.encode()
        try:
            self.sock.sendall(message)
            byte_data = self.sock.recv(10000)
        except (OSError, ConnectionResetError):
            self.get_logger().error(f"Connection to load cell lost.")
            # Close the socket, reconnect will be tried by the retry_socket_connect timer
            self.sock.close()
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket_connected = False
            return

        # Filter data and store
        data = np.frombuffer(byte_data).reshape(self.number_of_samples, -1)
        self.latest_filtered_sample = np.mean(data, axis=0)
        self.latest_sample_timestamp = self.get_clock().now().to_msg()

    def clean_shutdown(self) -> None:
        """ Closes the socket. Called when node is shutdown.
        """
        self.get_logger().info('Shutting down, closing load cell socket.')
        self.sock.close()

def main(args=None) -> None:
    rclpy.init(args=args)
    load_cell_data_server = LoadCellDataServer()
    rclpy.get_default_context().on_shutdown(load_cell_data_server.clean_shutdown)

    try:
        rclpy.spin(load_cell_data_server)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()
        load_cell_data_server.destroy_node()

if __name__ == '__main__':
    main()
