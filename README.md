# Scripting Drone Navigation with MAVSDK

This project demonstrates how to use **MAVSDK** to autonomously navigate TARS drones. It provides an environment for testing Python-based drone control scripts using MAVSDK and related tools.


## Sources:

- PyMAVlink: https://mavlink.io/en/mavgen_python/
- PyMAVlink (extra): https://pypi.org/project/pymavlink/
- Mission Planner: https://ardupilot.org/planner/docs/mission-planner-installation.html
- QGroundControl: https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html
- jMAVSim: https://github.com/PX4/jMAVSim , https://docs.px4.io/main/en/sim_jmavsim/index.html
- MAVSDK: https://mavsdk.mavlink.io/main/en/index.html

---

## 1. Project Overview

This repository aims to provide scripts and guidance on using **MAVSDK** (and its Python bindings) for drone navigation. It includes:

- Examples demonstrating connection to a drone (real or simulated).
- Scripts to command basic autonomous flight maneuvers.
- References to essential tools like **QGroundControl**, **Mission Planner**, and more.

---

## 2. Prerequisites

Make sure you have the following installed and properly set up:

- **Python 3.7+**  
- **MAVSDK-Python**  
- **PyMAVlink** (optional, for lower-level MAVLink handling)  
- **PX4 or ArduPilot** firmware (for simulations or real drone flight controllers)  
- A **Flight Controller** connected via USB, serial, or UDP (if testing with real hardware)  
- A **Simulator** such as **Gazebo** (if testing in simulation)

