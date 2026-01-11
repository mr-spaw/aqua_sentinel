```bash
#!/bin/bash
echo "Testing valve control..."
echo "Format: sensor_id:valve_state (0=closed, 1=open)"
echo ""

# Close valve on sensor 25
ros2 topic pub --once /valve_control std_msgs/msg/String "{data: '25:0'}" #example set
sleep 1

# Open valve on sensor 100  
ros2 topic pub --once /valve_control std_msgs/msg/String "{data: '100:1'}"
sleep 1

# Close valve on sensor 200
ros2 topic pub --once /valve_control std_msgs/msg/String "{data: '200:0'}"
sleep 1

echo "Test complete!"
echo "Check MQTT: mosquitto_sub -t 'pipes/control/valve#' -v"
```

Or even simpler:

```bash
#!/bin/bash
echo "Testing valve control..."

ros2 topic pub --once /valve_control std_msgs/msg/String "{data: '25:0'}"
sleep 1
ros2 topic pub --once /valve_control std_msgs/msg/String "{data: '100:1'}"
sleep 1
ros2 topic pub --once /valve_control std_msgs/msg/String "{data: '200:0'}"

echo "Done!"
```