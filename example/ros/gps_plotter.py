import rclpy
from rclpy.node import Node
# Import the correct message type
from px4_msgs.msg import VehicleGlobalPosition
import matplotlib.pyplot as plt
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

qos_profile = QoSProfile(depth=10)
qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT





class GPSPlotter(Node):
    def __init__(self):
        super().__init__('gps_plotter')
        self.subscription = self.create_subscription(
            VehicleGlobalPosition,
            '/fmu/out/vehicle_global_position',
            self.listener_callback,
            qos_profile)


        # Initialize plot
        self.fig, self.ax = plt.subplots()
        self.lon_data = []
        self.lat_data = []
        self.scatter = self.ax.scatter([], [], c='blue', s=2)
        self.ax.set_xlabel('Longitude')
        self.ax.set_ylabel('Latitude')
        self.ax.set_title('Real-Time GPS Plot')
        plt.ion()  # Enable interactive mode
        plt.show()

    def listener_callback(self, msg):
        # Extract and print data
        lat = msg.lat
        lon = msg.lon
        print(f"Received message: latitude={lat}, longitude={lon}")

        # Append new data
        self.lat_data.append(lat)
        self.lon_data.append(lon)

        # Update scatter plot data
        data = np.column_stack((self.lon_data, self.lat_data))
        self.scatter.set_offsets(data)

        # Update axes limits to include new data
        self.ax.set_xlim(min(self.lon_data) - 0.0001, max(self.lon_data) + 0.0001)
        self.ax.set_ylim(min(self.lat_data) - 0.0001, max(self.lat_data) + 0.0001)

        # Redraw the plot
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    gps_plotter = GPSPlotter()

    try:
        while rclpy.ok():
            rclpy.spin_once(gps_plotter, timeout_sec=0.1)
            plt.pause(0.001)  # Process Matplotlib events
    except KeyboardInterrupt:
        pass

    gps_plotter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
