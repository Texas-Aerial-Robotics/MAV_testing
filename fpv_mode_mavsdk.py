import asyncio
from mavsdk import System
from mavsdk.offboard import VelocityNedYaw
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
    lr, fb, ud, yv = 0, 0, 0, 0
    speed = 5

    if getKey("a"):
        lr = -speed
    elif getKey("d"):
        lr = speed
    if getKey("UP"):
        ud = speed
    elif getKey("DOWN"):
        ud = -speed
    if getKey("w"):
        fb = speed
    elif getKey("s"):
        fb = -speed
    if getKey("q"):
        yv = speed
    elif getKey("e"):
        yv = -speed

    return [lr, fb, ud, yv]

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
    initial_velocity = VelocityNedYaw(north_m_s=0.0, east_m_s=0.0, down_m_s=0.0, yaw_deg=0.0)
    await drone.offboard.set_velocity_ned(initial_velocity)

    print("-- Setting offboard mode")
    await drone.offboard.start()

    while True:
        vals = get_keyboard_input()
        velocity = VelocityNedYaw(north_m_s=vals[1], east_m_s=vals[0], down_m_s=-vals[2], yaw_deg=vals[3])
        await drone.offboard.set_velocity_ned(velocity)

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
