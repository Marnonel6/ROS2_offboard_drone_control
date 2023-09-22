"""
TODO
"""
# ROS
import rclpy
from rclpy.node import Node
# PX4
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)
import asyncio
# Python/Other

class DroneControl(Node):
    """
    TODO
    """
    def __init__(self):
        """
        Initialize function
        """
        super().__init__('drone_control')

        # Initialize drone
        self.loop = asyncio.get_event_loop()
        self.droneSetup_future = self.loop.run_until_complete(self.droneInit())
        self.get_logger().info(f"self.droneSetup_future = {self.droneSetup_future}")
        self.get_logger().info("--- Drone State: ARMED ---")

    async def droneInit(self):
        """
        droneInit function connects to the serial port (TELEM 2) of the drone,
        establishes a connection and arms the drone once the connection is secured 
        """
        self.get_logger().info("Initialize drone connection...")
        # Create a MAVSDK instance
        self.drone = System()

        # Connect to the Raspberry Pi to the Vehicle serial port
        await self.drone.connect(system_address="serial:///dev/serial0:57600")
        self.get_logger().info("Waiting for drone to connect...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.get_logger().info("-- Connected to drone!")
                break

        self.get_logger().info("Waiting for drone to have a global position estimate...")
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                self.get_logger().info("-- Global position estimate OK")
                break

        self.get_logger().info("Drone connection initialized")

        await self.drone.action.arm()

def main(args=None):
    rclpy.init(args=args)

    drone_control = DroneControl()

    rclpy.spin(drone_control)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    drone_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()