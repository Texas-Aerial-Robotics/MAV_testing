from mavsdk import System
import asyncio

async def connect_and_takeoff(address, port, drone_id):
    print(f"[Drone {drone_id}] Connecting to {address}:{port}...")
    drone = System(mavsdk_server_address=address, port=port)
    await drone.connect()

    print(f"[Drone {drone_id}] Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"[Drone {drone_id}] Connected!")
            break

    print(f"[Drone {drone_id}] Arming...")
    await drone.action.arm()

    print(f"[Drone {drone_id}] Taking off...")
    await drone.action.takeoff()

    # Hover for 10 seconds
    await asyncio.sleep(10)

    print(f"[Drone {drone_id}] Hovering. Takeoff complete!")

async def main():
    drones = [
        {"address": "127.0.0.1", "port": 50051, "id": 1},
        {"address": "127.0.0.1", "port": 50052, "id": 2},
    ]

    tasks = [
        connect_and_takeoff(drone["address"], drone["port"], drone["id"])
        for drone in drones
    ]
    await asyncio.gather(*tasks)

print("Starting multiple drone operations...")
asyncio.run(main())
