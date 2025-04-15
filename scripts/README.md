# Drone Operation Scripts

This folder contains a collection of scripts for interacting with hardware, primarily for NVIDIA Jetson or similar Linux targets.

## MAVLink Routing
- `mav.sh`, `mavscout.sh`, `mavscout-bcast.sh`

These scripts start MAVSDK server and MAVproxy, to allow for simultaneous autonomous control through other programs and ground station communication. `mav.sh` broadcasts to `10.42.0.255` on port 14550, while `mavscout.sh` uses port 14560. `mavscout-bcast.sh` broadcasts to `255.255.255.255` on port 14560, so it is more suitable for other networks.

To use with QGroundControl, add the link in Application Settings > Comm Links. Set the port according to the script being used, and set the address to the address of the flight computer. Select "Automatically Connect on Start," and save the link. Finally, connect and return to the main page. The flight computer should be connected.

These scripts will attempt to automatically connect to the flight controller using serial. Ensure that the flight controller is correctly wired over UART (TX to RX, RX to TX) and that the baud rate is set to 57600 baud for the the telemetry port in use.

When using Jetson Orin Nano flight computers, the onboard UART pins cannot be used because they cannot supply enough current to overcome the pullups in Cube flight controllers. Instead, use an external UART to USB adapter configured for 3.3V, and connect to one of the Jetson's USB ports.


## GStreamer Pipeline
- `gst.sh`, `gst-zed.sh`

For consistency with SITL simulation and additional flexibility with camera parameters, GStreamer pipelines are used to read video from cameras. `gst.sh` starts a pipeline using V4L2 with the device specified in the script. Camera parameters can be added to the `extra-controls` section. For a list of parameters, run `v4l2-ctl --all`. `gst-zed.sh` provides a similar pipeline for use with the `zedsrc` source provided by ZEDSDK for ZED cameras.
