#!/bin/bash
ffmpeg -re -f lavfi -i "testsrc2=size=640x480:rate=30" \
  -c:v libx264 -preset ultrafast -tune zerolatency \
  -f rtsp -rtsp_transport tcp rtsp://localhost:554/live/robot_front &

ffmpeg -re -f lavfi -i "testsrc=size=640x480:rate=30" \
  -c:v libx264 -preset ultrafast -tune zerolatency \
  -f rtsp -rtsp_transport tcp rtsp://localhost:554/live/robot_back &

echo "Test streams started:"
echo "  rtsp://localhost:554/live/robot_front"
echo "  rtsp://localhost:554/live/robot_back"
echo "Press Ctrl+C to stop"
wait
