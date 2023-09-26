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
        # self.get_logger().info(f"self.droneSetup_future = {self.droneSetup_future}")
        self.get_logger().info("--- Drone State: ARMED ---")

        # NOTE TEST: Take-off -> Hover -> Land -> Disarm
        self.loop_test = asyncio.get_event_loop()
        self.test_future = self.loop_test.run_until_complete(self.take_off_land())


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

    async def take_off_land(self):
        """
        Take-off -> Hover -> Land -> Disarm
        """

        self.get_logger().info("-- Setting initial location setpoint")
        await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

        self.get_logger().info("-- Starting offboard")
        try:
            await self.drone.offboard.start()
            self.get_logger().info("--- Drone State: OFFBOARD ---")
        except OffboardError as error:
            self.get_logger().info(f"Starting offboard mode failed with error code: \
                                   {error._result.result}")
            self.get_logger().info("-- Disarming")
            await self.drone.action.disarm()
            self.get_logger().info("--- Drone State: DISARMED ---")
            return

        self.get_logger().info("-- Go 0m North, 0m East, -5m Down within local coordinate system")
        await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -5.0, 0.0))

        # Check if drone has reached set position
        async for position in self.drone.telemetry.position_velocity_ned():
            # self.get_logger().info(f"position.position.down_m = {position.position.down_m}")
            if position.position.down_m < -4.5:
                self.get_logger().info("-- Reached target position")
                break

        # Hover at target position for 2 seconds
        self.get_logger().info("-- Hovering for 2 seconds")
        await asyncio.sleep(2)

        self.get_logger().info("Landing ...")
        await self.drone.action.land()

        # Check if drone has landed
        async for is_landed in self.drone.telemetry.landed_state():
            if is_landed:
                self.get_logger().info("-- Landed")
                break

        # NOTE Disarming gives an error: 
        # mavsdk.action.ActionError: COMMAND_DENIED_NOT_LANDED: 'Command Denied Not Landed';
        # origin: disarm(); params: ()
        # await self.drone.action.disarm()
        # self.get_logger().info("--- Drone State: DISARMED ---")

        # NOTE stopping offboard goes to HOLD state, when the drone has detected landing it
        # disarms itself
        # self.get_logger().info("-- Stopping offboard")
        # try:
        #     await self.drone.offboard.stop()
        #     self.get_logger().info("--- Drone State: HOLD ---")
        # except OffboardError as error:
        #     self.get_logger().info(f"Stopping offboard mode failed with error code: \
        #                              {error._result.result}")


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