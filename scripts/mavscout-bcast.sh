#!/bin/sh
mavproxy.py --out=udpbcast:255.255.255.255:14560 --out=127.0.0.1:14560 --daemon --mav20 &
sleep 10 &&
~/.local/lib/python3.8/site-packages/mavsdk/bin/mavsdk_server udpin://127.0.0.1:14560 &
#gst-launch-1.0 zedsrc ! timeoverlay  ! queue max-size-time=0 max-size-bytes=0 max-size-buffers=0 ! autovideoconvert !  x264enc byte-stream=true tune=zerolatency speed-preset=ultrafast bitrate=3000 !  h264parse ! rtph264pay config-interval=-1 pt=96 ! queue !  udpsink clients=127.0.0.1:5601 max-bitrate=3000000 sync=false async=false
