#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, SetEntityState, GetEntityState
from gazebo_msgs.msg import EntityState

class DroneLauncher(Node):
    def __init__(self):
        super().__init__("drone_launcher_node")

        # We assume the drone is named "iris" in Gazebo (the default if you used "model://iris")
        self.drone_name = "iris_depth_camera"

        # Paths & model name
        # Update this to the path of your big_box/model.sdf
        self.box_sdf_path = "/home/madan/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/big_box/model.sdf"
        self.box_model_name = "big_box"

        # Create service clients
        self.get_state_cli = self.create_client(GetEntityState, "/get_entity_state")
        self.spawn_cli = self.create_client(SpawnEntity, "/spawn_entity")
        self.set_state_cli = self.create_client(SetEntityState, "/set_entity_state")

    def launch_box(self):
        # 1) Get the drone's current pose
        self.get_logger().info(f"Requesting pose of drone [{self.drone_name}]...")
        self.get_state_cli.wait_for_service()
        get_req = GetEntityState.Request()
        get_req.name = self.drone_name

        future = self.get_state_cli.call_async(get_req)
        rclpy.spin_until_future_complete(self, future)
        if not future.result():
            self.get_logger().error("Failed to call /get_entity_state.")
            return

        drone_state = future.result()
        if not drone_state.success:
            self.get_logger().warn(f"/get_entity_state returned unsuccessful: {drone_state.status_message}")
            return

        # The drone's pose
        drone_pose = drone_state.state.pose

        # 2) Spawn the big box near the drone
        #    For example, spawn slightly in front of the drone's X-axis (0.5m ahead).
        #    We'll reuse the drone's orientation. This is optional—use any offset you like.
        spawn_pose = drone_pose
        spawn_pose.position.x += 0.5  # half a meter in front
        spawn_req = SpawnEntity.Request()
        spawn_req.name = self.box_model_name
        spawn_req.xml = open(self.box_sdf_path, 'r').read()
        spawn_req.robot_namespace = ""  # optional
        spawn_req.initial_pose = spawn_pose
        spawn_req.reference_frame = "world"

        self.spawn_cli.wait_for_service()
        future_spawn = self.spawn_cli.call_async(spawn_req)
        rclpy.spin_until_future_complete(self, future_spawn)
        if not future_spawn.result():
            self.get_logger().error("Failed to spawn the big_box model.")
            return

        spawn_res = future_spawn.result()
        if not spawn_res.success:
            self.get_logger().warn(f"Spawn failed: {spawn_res.status_message}")
            return
        else:
            self.get_logger().info(f"Spawned big_box at drone's position. Message: {spawn_res.status_message}")

        # 3) Give the box an initial velocity
        #    We'll just set a linear velocity in the drone's forward direction (roughly +x in the world frame).
        #    If you want to be more precise, you'd rotate the velocity vector by the drone's orientation.
        #    But for simplicity, let's just do +x in world coordinates.
        set_req = SetEntityState.Request()
        set_req.state.name = self.box_model_name
        set_req.state.twist.linear.x = 5.0  # 5 m/s forward
        set_req.state.twist.linear.y = 0.0
        set_req.state.twist.linear.z = 0.0

        self.set_state_cli.wait_for_service()
        future_set = self.set_state_cli.call_async(set_req)
        rclpy.spin_until_future_complete(self, future_set)
        if future_set.result():
            self.get_logger().info("Big box launched!")
        else:
            self.get_logger().error("Failed to set big_box velocity.")

def main(args=None):
    rclpy.init(args=args)
    node = DroneLauncher()
    node.launch_box()
    rclpy.shutdown()

if __name__ == "__main__":
    main()