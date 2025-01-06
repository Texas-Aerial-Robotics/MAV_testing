import asyncio
import cv2
from Video import Video

from field_traversal import connect_and_setup, start_offboard_mode,move_north_south, move_east, land_and_cleanup

from aruco_tracking import find_aruco_markers, execute_precision_landing

async def get_position(drone):
    async for position in drone.telemetry.position():
        print(f"Position received: {position}")
        return position


async def go_to_position(drone2, drone1):
    drone1_position = await get_position(drone1)

    await drone2.action.goto_location(
        drone1_position.latitude_deg,
        drone1_position.longitude_deg,
        drone1_position.absolute_altitude_m,  # Use absolute altitude from drone1
        0
    )
async def video_display(video_port, queue):
    """Handle video display for a single drone"""
    window_name = f'Drone Camera {video_port}'
    frame_count = 0

    video_stream = Video(port=video_port)
    found_count = 0
    found_aruco = False


    
    while True:
        if video_stream.frame_available():
            frame = video_stream.frame()
            if frame is not None:
                frame_count += 1
                # Make a copy of the frame before modifying it
                display_frame = frame.copy()
                
                # if frame_count % 30 == 0:  # Print every 30 frames
                #     print(f"Drone {drone_id}: Received frame {frame_count}")
                #     print(f"Frame shape: {frame.shape}")

                bbx, ids = find_aruco_markers(display_frame)
                if ids is not None:
                    cv2.putText(display_frame, f"Marker ID: {ids[0]}", (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    # Get marker center coordinates
                    marker = bbx[0][0]
                    center_x = int((marker[0][0] + marker[2][0]) / 2)
                    center_y = int((marker[0][1] + marker[2][1]) / 2)
                    cv2.putText(display_frame, f"Pos: ({center_x},{center_y})", (10, 90),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    found_count += 1

                    if found_count == 10:
                        # pretty sure we've found a marker
                        found_aruco = True
                        await queue.put(found_aruco)
                        found_count = 0
                        found_aruco = False




                cv2.putText(display_frame, f"Video Feed: {video_port}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.imshow(window_name, display_frame)

                cv2.waitKey(1)
        
        await asyncio.sleep(0.01)

async def scouting(drone, target_altitude):


        if not await start_offboard_mode(drone, target_altitude):
            return

        # Execute flight path
        current_north = 0
        current_east = 0

        current_north = await move_north_south(drone, target_altitude, current_north, current_east, 25, "north")

        current_east = await move_east(drone, target_altitude, current_north, current_east, 20)

        current_north = await move_north_south(drone, target_altitude, current_north, current_east, 55, "south")

        current_east = await move_east(drone, target_altitude, current_north, current_east, 20)

        current_north = await move_north_south(drone, target_altitude, current_north, current_east, 55, "north")

        current_east = await move_east(drone, target_altitude, current_north, current_east, 20)

        current_north = await move_north_south(drone, target_altitude, current_north, current_east, 55, "south")

        current_east = await move_east(drone, target_altitude, current_north, current_east, 20)

        current_north = await move_north_south(drone, target_altitude, current_north, current_east, 55, "north")

        current_east = await move_east(drone, target_altitude, current_north, current_east, 20)

        current_north = await move_north_south(drone, target_altitude, current_north, current_east, 55, "south")

        current_east = await move_east(drone, target_altitude, current_north, current_east, 20)

        current_north = await move_north_south(drone, target_altitude, current_north, current_east, 55, "north")

        await land_and_cleanup(drone)

async def main():


    scout_drone, target_altitude = await connect_and_setup(50051)




    scouting_queue = asyncio.Queue()
    delivery_queue = asyncio.Queue()

    async def moniter_queue():

        delivery_drone = None
        while True:
            if not scouting_queue.empty():
                found_aruco = await scouting_queue.get()

                print(found_aruco)

                if found_aruco:
                    delivery_drone, target_altitude = await connect_and_setup(50052)

                    await start_offboard_mode(delivery_drone, target_altitude + 5)

                    break
            await asyncio.sleep(0.01)

        await go_to_position(delivery_drone, scout_drone)

        await asyncio.sleep(10)

        landing_task = asyncio.create_task(execute_precision_landing(delivery_drone, Video(5602)))
        await landing_task  # Wait for landing to complete


    tasks = [

        scouting(scout_drone, target_altitude),
        video_display(5601, scouting_queue),
        moniter_queue()

    ]

    await asyncio.gather(*tasks)








if __name__ == "__main__":
    asyncio.run(main())
   