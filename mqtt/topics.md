# **Aqua Sentinel: MQTT-ROS Bridge for Water Distribution Network Simulation**

## **System Overview**

Aqua Sentinel is a distributed sensor network simulation system that models water distribution and sewage networks using ESP32 microcontrollers as sensor nodes, communicating through MQTT, and integrating with ROS2 for centralized monitoring and control.

### **Architecture**
```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          ROS2 (Linux System)                                │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐   │
│  │   Node:     │    │   Node:     │    │   Node:     │    │   Node:     │   │
│  │ ESP1 Bridge │    │ ESP2 Bridge │    │ ESP3 Bridge │    │ ESP4 Bridge │   │
│  └──────┬──────┘    └──────┬──────┘    └──────┬──────┘    └──────┬──────┘   │
│         │                   │                   │                   │       │
└─────────┼───────────────────┼───────────────────┼───────────────────┼───────┘
          │                   │                   │                   │
          ▼                   ▼                   ▼                   ▼
    ┌───────────┐      ┌───────────┐      ┌───────────┐      ┌───────────┐
    │  ESP32-1  │      │  ESP32-2  │      │  ESP32-3  │      │  ESP32-4  │
    │ (Fresh)   │      │ (Mixed)   │      │ (Mixed)   │      │ (Sewage)  │
    └───────────┘      └───────────┘      └───────────┘      └───────────┘
```

## **ESP32 Sensor Node (Arduino Code)**

### **Key Features**
- **Physical Simulation**: Realistic fluid dynamics simulation using Darcy-Weisbach equations
- **Dual Pipe Types**: Fresh water (pressurized) and sewage (open-channel) systems
- **Degradation Simulation**: Progressive leaks and blockages with realistic failure modes
- **Temperature Effects**: Viscosity adjustments based on temperature variations
- **Batch Publishing**: Efficient MQTT communication with chunking for large sensor counts
- **Configuration Management**: Dynamic sensor configuration from ROS2 controller

### **Physics Model Details**

#### **1. Reynolds Number Calculation**
```cpp
float calculateReynolds(float velocity, float diameter, float viscosity) {
  return (WATER_DENSITY * velocity * diameter) / viscosity;
}
```

#### **2. Friction Factor Models**
- **Laminar flow (Re < 2300)**: `f = 64/Re`
- **Transition zone (2300 < Re < 4000)**: Churchill equation with cubic interpolation
- **Turbulent flow (Re > 4000)**: Haaland equation

#### **3. Head Loss Calculation**
```cpp
float headLoss = frictionFactor * (length/diameter) * (velocity²)/(2*GRAVITY)
```

#### **4. Temperature Effects**
```cpp
float effectiveViscosity = 0.00179f * exp(-0.024f * temperature);
```

### **Data Flow**

#### **Configuration Phase**
```
ROS2 (Linux) → MQTT → ESP32
    ↓
Topic: pipes/config/init4
Payload: JSON with sensor configurations
```

#### **Monitoring Phase**
```
ESP32 → MQTT → ROS2 (Linux)
    ↓
Topics: 
- pipes/sensors4/all         (Batch data)
- pipes/sensors4/[id]       (Individual sensors)
- pipes/system/summary4     (System summary)
```

#### **Control Phase**
```
ROS2 (Linux) → MQTT → ESP32
    ↓
Topics:
- pipes/control/valve4      (Valve control)
- pipes/maintenance/esp4    (Maintenance commands)
```

## **ROS2 Topics Structure**

### **Sensor Data Topics**
- `/esp1/sensors` - ESP32 #1 sensor data (typically fresh water)
- `/esp2/sensors` - ESP32 #2 sensor data (mixed)
- `/esp3/sensors` - ESP32 #3 sensor data (mixed)
- `/esp4/sensors` - ESP32 #4 sensor data (typically sewage)

### **Control Topics**
- `/valve_control` - Valve operation commands
- `/maintenance/command` - System maintenance and reset commands
- `/maintenance/request` - Maintenance status requests

### **System Topics**
- `/parameter_events` - ROS2 parameter changes
- `/rosout` - System logging

## **MQTT Topic Structure**

### **ESP-Specific Topics (Replace ESP_CHANGE with ID: 1,2,3,4)**

#### **Configuration Topics**
```
pipes/config/init[ESP_ID]           # Initial configuration
```

#### **Sensor Data Topics**
```
pipes/sensors[ESP_ID]/all           # All sensors (batched)
pipes/sensors[ESP_ID]/[sensor_id]   # Individual sensor
```

#### **Control Topics**
```
pipes/control/valve[ESP_ID]         # Valve control
pipes/maintenance/esp[ESP_ID]       # Maintenance commands
```

#### **System Topics**
```
pipes/system/summary[ESP_ID]        # System summary
```

## **Message Formats**

### **Configuration Message (ROS2 → ESP32)**
```json
{
  "timestamp": 1640995200,
  "dt": 1.0,
  "sensors": [
    {
      "sensor_id": 101,
      "pipe_id": 201,
      "type": "fresh",
      "pressure_kpa": 250.5,
      "level_pct": 85.2,
      "valve": 1
    },
    // ... more sensors
  ]
}
```

### **Sensor Data Message (ESP32 → ROS2)**
```json
{
  "sensor_id": 101,
  "pipe_id": 201,
  "type": "fresh",
  "pressure_kpa": 248.75,
  "level_pct": 84.8,
  "valve": 1,
  "flow_rate_Ls": 12.5,
  "velocity_ms": 1.42,
  "temperature_C": 15.5,
  "reynolds": 12500,
  "friction_factor": 0.0234,
  "head_loss_m": 3.21,
  "degrading": 0,
  "leak_coeff_pct": 0.0,
  "blockage_coeff_pct": 0.0
}
```

### **System Summary Message**
```json
{
  "timestamp": 1640995200,
  "total_sensors": 50,
  "fresh_water_count": 30,
  "sewage_count": 20,
  "degrading_count": 3,
  "closed_valves": 2,
  "avg_pressure_fresh_kpa": 245.5,
  "avg_level_sewage_pct": 78.2,
  "total_flow_fresh_Ls": 375.0,
  "total_flow_sewage_Ls": 280.5
}
```

## **Failure Modes & Degradation**

### **Fresh Water Pipes (70% leaks, 30% blockages)**
1. **Leaks**: Start at 1% coefficient, increase 0.001f/sec
2. **Blockages**: Start at 1% coefficient, increase 0.0005f/sec

### **Sewage Pipes (70% blockages, 30% leaks)**
1. **Blockages**: Start at 1% coefficient, increase 0.001f/sec
2. **Leaks**: Start at 1% coefficient, increase 0.0004f/sec

### **Degradation Schedule**
- First failure: 60 seconds after initialization
- Subsequent failures: Random between 15-45 seconds

## **Maintenance Operations**

### **Reset Commands**
```json
// Reset specific sensors
{
  "reset_to_initial": [101, 102, 103]
}

// Reset single sensor
{
  "reset_sensor_to_initial": 101
}

// Reset all sensors
{
  "reset_all_to_initial": true
}
```

### **Valve Control**
```json
{
  "sensor_id": 101,
  "valve": 1  // 1=open, 0=closed
}
```

## **Setup Instructions**

### **1. ESP32 Setup**
```bash
# Install required libraries
PlatformIO Libraries:
- WiFi
- PubSubClient
- ArduinoJson (6.19.4 or newer)

# Configuration
Update in code:
1. WIFI_SSID and WIFI_PASSWORD
2. MQTT_SERVER (Linux system IP)
3. ESP_ID in all topics (1,2,3,4)
```

### **2. ROS2 Setup**
```bash
# Create workspace
mkdir -p ~/aqua_sentinel_ws/src
cd ~/aqua_sentinel_ws/src

# Clone bridge packages
git clone <mqtt_bridge_package>
git clone <sensor_processing_package>

# Build
colcon build
source install/setup.bash
```

### **3. MQTT Broker Setup**
```bash
# Install Mosquitto
sudo apt install mosquitto mosquitto-clients

# Start broker
mosquitto -c /etc/mosquitto/mosquitto.conf
```

## **Running the System**

### **Step 1: Start MQTT Broker**
```bash
mosquitto -v
```

### **Step 2: Flash ESP32 Devices**
```bash
# For each ESP32 (1-4):
1. Update ESP_ID in code
2. Flash using PlatformIO or Arduino IDE
3. Monitor serial output at 115200 baud
```

### **Step 3: Start ROS2 Bridge**
```bash
# Terminal 1: MQTT to ROS2 bridge
ros2 run mqtt_bridge mqtt_bridge_node

# Terminal 2: Sensor processor
ros2 run sensor_processor sensor_node

# Terminal 3: Control interface
ros2 run control_ui control_node
```

### **Step 4: Monitor Topics**
```bash
# View all sensors from ESP1
ros2 topic echo /esp1/sensors

# View system summary
ros2 topic echo /pipes/system/summary1

# Send valve control command
ros2 topic pub /valve_control std_msgs/String '{"sensor_id": 101, "valve": 0}'
```

## **Troubleshooting**

### **Common Issues**

1. **ESP32 won't connect to WiFi/MQTT**
   - Verify IP address in `MQTT_SERVER`
   - Check WiFi credentials
   - Ensure broker is running

2. **No data on ROS2 topics**
   - Check MQTT bridge is running
   - Verify topic names match (ESP_ID)
   - Monitor MQTT traffic: `mosquitto_sub -t '#' -v`

3. **Physics simulation issues**
   - Check serial monitor for errors
   - Verify sensor count doesn't exceed `TOTAL_PIPES`
   - Monitor memory usage (ESP32 limited)

4. **Degradation not occurring**
   - Check `nextDegradeTime` logic
   - Verify `remainingToDegrade` counter
   - Monitor serial for degradation messages

### **Debug Commands**
```bash
# View all MQTT messages
mosquitto_sub -t 'pipes/#' -v

# Test ESP32 connection
mosquitto_pub -t 'pipes/config/init1' -m '{"test": true}'

# Check ROS2 node status
ros2 node list
ros2 topic list
ros2 topic info /esp1/sensors
```
