import asyncio
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed
import pygame

def init():
    print("Initializing pygame...")
    pygame.init()
    pygame.display.set_mode((400, 400))
    print("Pygame initialized.")

def getKey(keyName):
    ans = False
    for eve in pygame.event.get(): pass
    keyInput = pygame.key.get_pressed()
    myKey = getattr(pygame, 'K_{}'.format(keyName))
    if keyInput[myKey]:
        ans = True
    pygame.display.update()
    return ans

def get_keyboard_input():
    forward, right, down, yaw_speed = 0, 0, 0, 0
    speed = 2.5  # meters/second
    yaw_speed_rate = 50  # degrees/second

    if getKey("a"):
        right = -speed
    elif getKey("d"):
        right = speed
    if getKey("UP"):
        down = -speed  # Going up decreases the down speed in body frame
    elif getKey("DOWN"):
        down = speed
    if getKey("w"):
        forward = speed
    elif getKey("s"):
        forward = -speed
    if getKey("q"):
        yaw_speed = yaw_speed_rate
    elif getKey("e"):
        yaw_speed = -yaw_speed_rate

    return [forward, right, down, yaw_speed]

async def main():
    print("Connecting to drone...")
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected to drone!")
            break

    print("Waiting for global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    print("-- Arming")
    await drone.action.arm()

    print("-- Taking off")
    await drone.action.takeoff()

    # Wait for the drone to reach a stable altitude
    await asyncio.sleep(5)

    # Initial setpoint before starting offboard mode
    initial_velocity = VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
    await drone.offboard.set_velocity_body(initial_velocity)

    print("-- Setting offboard mode")
    await drone.offboard.start()

    while True:
        vals = get_keyboard_input()
        velocity = VelocityBodyYawspeed(vals[0], vals[1], vals[2], vals[3])
        await drone.offboard.set_velocity_body(velocity)

        # Breaking the loop and landing if 'l' key is pressed
        if getKey("l"):
            print("-- Landing")
            await drone.action.land()
            break

        await asyncio.sleep(0.1)

if __name__ == "__main__":
    init()
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
