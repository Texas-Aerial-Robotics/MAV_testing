from ros_tools import ensure_ros_environment
ensure_ros_environment()

import rclpy
from rclpy.node import Node
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw
from std_msgs.msg import String
import asyncio


class DroneNode(Node):
    def __init__(self):
        super().__init__('drone_node')
        self.drone = System(mavsdk_server_address="localhost", port=50051)
        self.create_subscription(String, 'drone_command', self.command_callback, 10)
        self.get_logger().info("Drone node initialized!")
        self.loop = asyncio.get_event_loop()

    async def connect(self):
        self.get_logger().info("Connecting to the drone...")
        await self.drone.connect()
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.get_logger().info("Drone connected!")
                break

    async def command_callback(self, msg):
        command = msg.data
        self.get_logger().info(f"Received command: {command}")
        if command == "takeoff":
            await self.takeoff()
        elif command == "land":
            await self.land()
        else:
            self.get_logger().warn(f"Unknown command: {command}")

    async def takeoff(self):
        try:
            self.get_logger().info("Arming the drone...")
            await self.drone.action.arm()
            self.get_logger().info("Taking off...")
            await self.drone.action.takeoff()
        except Exception as e:
            self.get_logger().error(f"Failed to take off: {e}")

    async def land(self):
        try:
            self.get_logger().info("Landing the drone...")
            await self.drone.action.land()
        except Exception as e:
            self.get_logger().error(f"Failed to land: {e}")


async def main_async():
    rclpy.init()
    drone_node = DroneNode()

    # Run the connection asynchronously
    await drone_node.connect()

    # Use asyncio to manage both ROS 2 and MAVSDK
    try:
        while rclpy.ok():
            rclpy.spin_once(drone_node, timeout_sec=0.1)
            await asyncio.sleep(0.1)
    except KeyboardInterrupt:
        drone_node.get_logger().info("Shutting down...")
    finally:
        drone_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    asyncio.run(main_async())
