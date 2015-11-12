#!/bin/bash
/var/lib/cloud9/mjpg-streamer-code-182/mjpg-streamer/mjpg_streamer -i "/var/lib/cloud9/mjpg-streamer-code-182/mjpg-streamer/input_uvc.so -f 30 -r 1920X1080" -o "/var/lib/cloud9/mjpg-streamer-code-182/mjpg-streamer/output_http.so -w /var/lib/cloud9/mjpg-streamer-code-182/mjpg-streamer/www -p 8082" &
