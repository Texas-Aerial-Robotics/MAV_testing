# Scripting drone navigation with Pymavlink
---

This project will test pymavlink to navigate TARs drones autonomously.

## Sources:

- PyMAVlink: https://mavlink.io/en/mavgen_python/
- PyMAVlink (extra): https://pypi.org/project/pymavlink/
- Mission Planner: https://ardupilot.org/planner/docs/mission-planner-installation.html
- QGroundControl: https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html
- jMAVSim: https://github.com/PX4/jMAVSim , https://docs.px4.io/main/en/sim_jmavsim/index.html


RUN THIS TO START MAVSDK_SERVER

/home/tar/px4_pymavlink_testing/myenv/lib/python3.8/site-packages/mavsdk/bin/mavsdk_server serial:///dev/ttyACM0:57600


RUN THIS TO START FPV_MODE SCRIPT

python /home/tar/px4_pymavlink_testing/simulation/pymavlink_testing/example/fpv_mode_mavsdk.py