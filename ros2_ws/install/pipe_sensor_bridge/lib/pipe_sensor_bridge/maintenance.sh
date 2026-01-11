#!/bin/bash
# Simple Maintenance Control

echo "Pipe Sensor Maintenance"

ACTION=$1
ESP_ID=$2
SENSOR=$3

case $ACTION in
    reset_sensor)
        echo "Resetting sensor $SENSOR on ESP$ESP_ID..."
        ros2 topic pub --once /maintenance/request std_msgs/msg/String "{data: '{\"esp_id\": $ESP_ID, \"action\": \"reset_sensor_to_initial\", \"sensor_id\": $SENSOR}'}"
        ;;
        
    reset_all)
        echo "Resetting ALL sensors on ESP$ESP_ID..."
        ros2 topic pub --once /maintenance/request std_msgs/msg/String "{data: '{\"esp_id\": $ESP_ID, \"action\": \"reset_all_to_initial\"}'}"
        ;;
        
    *)
        echo "Usage:"
        echo "  $0 reset_sensor <esp_id> <sensor_id>"
        echo "  $0 reset_all <esp_id>"
        exit 1
        ;;
esac

echo "Done!"