# MQTT Linux Sensor Bridge

## 📦 Launch Configuration

### **Main Launch File** (`pipe_bridge.launch.py`)

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    return LaunchDescription([
        # ROS2-MQTT Bridge Node
        Node(
            package='pipe_sensor_bridge',
            executable='mqtt_bridge',
            name='mqtt_bridge',
            output='screen',
            parameters=[{
                'sensors_per_esp': 75,
                'max_esp_count': 4,
                'package_path': '/home/arka/aqua_sentinel/ros2_ws/src/pipe_sensor_bridge'
            }]
        ),
        
        # Init Config Publisher Node
        Node(
            package='pipe_sensor_bridge',
            executable='init_config_publisher',
            name='init_config_publisher',
            output='screen',
            parameters=[{
                'json_path': '/home/arka/aqua_sentinel/simulator/init_sensor.json',
                'esp_count': 4,
                'sensors_per_esp': 0  # Auto-calculate
            }]
        )
    ])
```

---

## 🔄 Complete Build & Run Cycle

### **Step 1: Build the Package**
```bash
cd ~/aqua_sentinel/ros2_ws
colcon build --packages-select pipe_sensor_bridge
```


### **Step 2: Source the Workspace**
```bash
source install/setup.bash
```

### **Step 3: Manage MQTT Broker**
```bash
# Stop existing broker
sudo systemctl stop mosquitto

# Start fresh broker
sudo systemctl start mosquitto

# Check status
sudo systemctl status mosquitto
# Should show: "Active: active (running)"
```

### **Step 4: Launch the System**
```bash
ros2 launch pipe_sensor_bridge pipe_bridge.launch.py
```

---

## ROS2 Command Reference

### **Monitoring Commands**

#### **1. Check System Status**
```bash
# List all active nodes
ros2 node list

# Get node information
ros2 node info /mqtt_bridge
ros2 node info /init_config_publisher

# List all available topics
ros2 topic list
```

#### **2. Monitor Sensor Data**
```bash
# Monitor ESP2 sensors (real-time)
ros2 topic echo /esp2/sensors

# Monitor all ESPs in separate terminals
# Terminal 1: ros2 topic echo /esp1/sensors
# Terminal 2: ros2 topic echo /esp2/sensors
# Terminal 3: ros2 topic echo /esp3/sensors
# Terminal 4: ros2 topic echo /esp4/sensors

# One-time snapshot
ros2 topic echo /esp2/sensors --once
```

#### **3. Check Parameters**
```bash
# List all parameters
ros2 param list

# Get specific parameters
ros2 param get /mqtt_bridge sensors_per_esp

ros2 param get /mqtt_bridge max_esp_count

ros2 param get /init_config_publisher esp_count
```

---

### **Control Commands**

#### **1. Valve Control Examples**
```bash
# Open valve on sensor 25 (ESP1)
ros2 topic pub --once /valve_control std_msgs/msg/String "{data: '25:1'}"

# Close valve on sensor 100 (ESP2)
ros2 topic pub --once /valve_control std_msgs/msg/String "{data: '100:0'}"

# Open valve on sensor 225 (ESP4)
ros2 topic pub --once /valve_control std_msgs/msg/String "{data: '225:1'}"

# Open valve on sensor 441 (out of range → ESP4)
ros2 topic pub --once /valve_control std_msgs/msg/String "{data: '441:1'}"

# Batch valve control (open multiple valves)
for i in {1..10}; do
  ros2 topic pub --once /valve_control std_msgs/msg/String "{data: '$i:1'}"
  sleep 0.1
done
```

#### **2. Maintenance Commands**
```bash
# Reset single sensor (auto-detect ESP)
ros2 topic pub /maintenance/request std_msgs/msg/String '{"data": "{\"sensor_id\": 441, \"action\": \"reset_sensor_to_initial\"}"}' --once

# Reset specific sensor on ESP2
ros2 topic pub /maintenance/request std_msgs/msg/String '{"data": "{\"esp_id\": 2, \"sensor_id\": 100, \"action\": \"reset_sensor_to_initial\"}"}' --once

# Reset multiple sensors on ESP3
ros2 topic pub /maintenance/request std_msgs/msg/String '{"data": "{\"esp_id\": 3, \"action\": \"reset_to_initial\", \"sensor_ids\": [200, 201, 202]}"}' --once

# Reset ALL sensors on ESP4
ros2 topic pub /maintenance/request std_msgs/msg/String '{"data": "{\"esp_id\": 4, \"action\": \"reset_all_to_initial\"}"}' --once

# Monitor maintenance acknowledgments
ros2 topic echo /maintenance/command
```

---

### **Debugging & Verification Commands**

#### **1. Verify MQTT Communication**
```bash
# Monitor all MQTT traffic
mosquitto_sub -t "#" -v

# Monitor specific ESP topics
mosquitto_sub -t "pipes/sensors2/#" -v          # ESP2 sensor data
mosquitto_sub -t "pipes/control/valve2" -v      # ESP2 valve commands
mosquitto_sub -t "pipes/config/init2" -v        # ESP2 configuration
mosquitto_sub -t "pipes/system/summary2" -v     # ESP2 system summary

# Test MQTT publishing (manual test)
mosquitto_pub -t "pipes/sensors2/all/0" -m '{"timestamp": 12345, "chunk": 0, "total_chunks": 1, "sensors": [{"sensor_id": 100, "pressure_bar": 2.5, "level_pct": 75}]}'
```

#### **2. Check Log Files**
```bash
# Navigate to log directory
cd ~/aqua_sentinel/ros2_ws/src/pipe_sensor_bridge/sensor_logs/

# List log files
ls -la

# View ESP2 log file
tail -f esp2_sensors.json
```

#### **3. System Diagnostics**
```bash
# Check ROS2 system
ros2 doctor
ros2 daemon status

# Check topic statistics
ros2 topic info /esp2/sensors
ros2 topic hz /esp2/sensors
ros2 topic bw /esp2/sensors

# Check service calls (if any)
ros2 service list
ros2 service type /your_service_name
```

---

