from mavsdk import System
import asyncio
import cv2
from Video import Video  # Import the Video class

async def video_display(video_stream, drone_id):
    """Handle video display for a single drone"""
    window_name = f'Drone {drone_id} Camera'
    print(f"Starting video display for Drone {drone_id}")
    frame_count = 0
    
    while True:
        if video_stream.frame_available():
            frame = video_stream.frame()
            if frame is not None:
                frame_count += 1
                # Make a copy of the frame before modifying it
                display_frame = frame.copy()
                
                if frame_count % 30 == 0:  # Print every 30 frames
                    print(f"Drone {drone_id}: Received frame {frame_count}")
                    print(f"Frame shape: {frame.shape}")
                
                cv2.putText(display_frame, f"Drone {drone_id}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.imshow(window_name, display_frame)
                cv2.waitKey(1)
        
        await asyncio.sleep(0.01)

async def drone_operation(address, port, drone_id, video_port):
    
        print(f"[Drone {drone_id}] Starting video stream on port {video_port}")
        video_stream = Video(port=video_port)
        video_task = asyncio.create_task(video_display(video_stream, drone_id))
        
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
        
        print(f"[Drone {drone_id}] Hovering and streaming video...")
        await asyncio.sleep(30)
        

async def main():
    drones = [
        {"address": "127.0.0.1", "port": 50051, "id": 1, "video_port": 5601},
        {"address": "127.0.0.1", "port": 50052, "id": 2, "video_port": 5602},
    ]
    
    tasks = [
        drone_operation(
            drone["address"],
            drone["port"],
            drone["id"],
            drone["video_port"]
        )
        for drone in drones
    ]
    
    await asyncio.gather(*tasks)

if __name__ == "__main__":
    print("Starting multiple drone operations with video streams...")
    asyncio.run(main())
   