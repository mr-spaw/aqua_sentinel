#!/bin/bash
echo "Monitoring MQTT sensor data..."
mosquitto_sub -t "pipes/sensors1/all/#" \
              -t "pipes/sensors2/all/#" \
              -t "pipes/sensors3/all/#" \
              -t "pipes/sensors4/all/#" \
              -t "pipes/system/summary1" \
              -t "pipes/system/summary2" \
              -t "pipes/system/summary3" \
              -t "pipes/system/summary4" -v