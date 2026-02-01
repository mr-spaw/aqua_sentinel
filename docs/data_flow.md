# Data Flow Documentation

## 1. System Initialization
```
Simulation → init_sensor.json → pipe_sensor_bridge → MQTT Broker → ESP32 Devices
```

## 2. Continuous Sensor Data Collection
```
ESP32 Hardware → ESP32 Firmware → MQTT Broker → mqtt_subscriber.cpp → ROS 2 DDS Topics
```

**Topics Published:**
- `/esp1/sensors`
- `/esp2/sensors`
- `/esp3/sensors`
- `/esp4/sensors`

## 3. Physics Computation Flow
```
ROS Topics → Simulation Bridge → Physics Engine → UDP Publisher → UDP Streams
```

**UDP Destinations:**
- Simulation (External)
- RL Agent (rl.py)
- Debug Module

## 4. Control Decision Flow (Non-RL Mode)
```
ROS Topics → Controller → Leak Detection → Decision Engine → Action Execution
```

**Action Execution Paths:**
1. Valve Control: `Controller → /valve_control → ROS Topic → ESP32`
2. Maintenance: `Controller → /maintenance/command → ROS Topic → ESP32`
3. Status Update: `Controller → TCP → Simulation`

## 5. Control Decision Flow (RL Mode)
```
UDP Stream → RL Agent → RL Model → REST API → Controller → Action Execution
```

**Parallel Monitoring:**
```
ROS Topics → Controller → Critical Monitor → Status Update → Simulation
```

## 6. Maintenance Flow
```
Controller/RL Agent → /maintenance/command → pipe_sensor_bridge → MQTT → ESP32
ESP32 → Execution → Response → MQTT → pipe_sensor_bridge → /maintenance/request → Controller
```

## 7. Debug and Monitoring Flow
```
UDP Streams → Debug Module → JSON Logger → Log Files
ROS Topics → Controller → Status Monitor → TCP → Simulation
```

## 8. Error/Alert Flow
```
ESP32 → Anomaly Detection → MQTT → pipe_sensor_bridge → ROS Topics → Controller
Controller → Alert Processing → TCP → Simulation + Local Logging
```

## 9. Real-Time Data Flow Summary

### Primary Data Pipeline (100ms cycle):
```
ESP32 → MQTT → ROS → [Simulation Bridge + Controller] → Actions
```

### Secondary Pipeline (RL Mode - 200ms cycle):
```
ESP32 → ROS → Simulation Bridge → UDP → RL Agent → REST → Controller → Actions
```

### Monitoring Pipeline (1s cycle):
```
All Components → Status Aggregation → TCP → Simulation
```

## 10. Command Response Flow
```
Simulation/Controller → Command → ROS → MQTT → ESP32
ESP32 → Execution → Status → MQTT → ROS → Controller → TCP → Simulation
```

## 11. Data Persistence Flow
```
Debug Module → JSON Logs → File System
Controller → Event Logs → TCP → Simulation Database
RL Agent → Training Data → Model Updates
```

## 12. Emergency Override Flow
```
Controller (Critical Detection) → Direct Command → ROS → ESP32
ESP32 → Immediate Response → Status → MQTT → ROS → Controller
```

## Flow Characteristics:
- **Frequency**: 10Hz (sensor data), 5Hz (physics), 1Hz (control decisions)
- **Latency**: <100ms (sensor to control), <200ms (RL mode)
- **Reliability**: QoS 2 for config, QoS 1 for sensor data, TCP for critical commands
- **Throughput**: ~100 messages/sec peak, ~40 messages/sec average