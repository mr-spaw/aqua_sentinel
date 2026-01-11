# ESP32 Pipe Network Sensor Simulator

## 📋 Project Overview
This ESP32-based system simulates a large-scale pipe network monitoring system with realistic fluid dynamics, degradation modeling, and MQTT-based communication. It simulates 200+ sensors (configurable) for both fresh water and sewage pipe networks.

## 🎯 Key Features
- **Realistic Physics Simulation**: Fluid dynamics with Reynolds numbers, friction factors, head loss calculations
- **Degradation Modeling**: Progressive leaks and blockages with unique failure patterns
- **Dynamic Configuration**: Receives sensor configurations via MQTT from a central Linux system
- **Maintenance Control**: Remote valve control and sensor state reset capabilities
- **Efficient MQTT Communication**: Batched JSON publishing with chunking for large datasets
- **Dual Pipe Types**: Fresh water (pressurized) and sewage (gravity flow) systems

## 🔧 Hardware Requirements
- ESP32-WROOM-32 Development Board
- WiFi connectivity
- Compatible with PlatformIO or Arduino IDE

## 📊 System Architecture

### Sensor Types
1. **Fresh Water Pipes**:
   - Pressurized systems (2-8 bar)
   - Leaks cause pressure drops
   - Blockages cause flow reduction

2. **Sewage Pipes**:
   - Gravity flow systems
   - Blockages cause level rise
   - Leaks cause level drops

### Physics Models
- **Reynolds Number**: Determines laminar vs turbulent flow
- **Colebrook-White Equation**: Friction factor calculation
- **Darcy-Weisbach Equation**: Head loss calculation
- **Orifice Equation**: Leak flow rate calculation

## 📡 MQTT Topics

### Configuration Topics
| Topic | Direction | Purpose | Format |
|-------|-----------|---------|--------|
| `pipes/config/initx` | ESP ← Linux | Initial sensor configuration | JSON array of sensor definitions |
| `pipes/sensorsx/{sensor_id}` | ESP → Linux | Individual sensor data | JSON sensor reading |
| `pipes/sensorsx/all` | ESP → Linux | Complete sensor dataset | JSON array (chunked if >100 sensors) |
| `pipes/sensorsx/all/chunk_{n}` | ESP → Linux | Chunked data for large sets | JSON partial array |

### Control Topics
| Topic | Direction | Purpose | Format |
|-------|-----------|---------|--------|
| `pipes/control/valvex` | ESP ← Linux | Remote valve control | `{"sensor_id": X, "valve": 0/1}` |
| `pipes/maintenance/espx` | ESP ← Linux | Maintenance commands | Various reset commands |

### System Topics
| Topic | Direction | Purpose |
|-------|-----------|---------|
| `pipes/system/summaryx` | ESP → Linux | System statistics |
| `pipes/system/debugx` | ESP → Linux | Debug information |

## 🔄 Workflow

### 1. Initialization Phase
```
ESP32 Boot → Connect WiFi → Connect MQTT → Subscribe to Topics → Wait for Config
```
- Waits up to 5 minutes for configuration from Linux
- Falls back to default simulation if no config received

### 2. Configuration Phase
Linux system publishes to `pipes/config/init4`:
```json
{
  "timestamp": 1234567890,
  "dt": 1.0,
  "sensors": [
    {
      "sensor_id": 1001,
      "pipe_id": 2001,
      "type": "fresh",
      "pressure_bar": 3.5,
      "level_pct": 85.0,
      "valve": 1
    },
    // ... more sensors
  ]
}
```

### 3. Operational Phase
- **2-second update interval** for physics simulation
- **Progressive degradation** (random failures every 15-45 seconds)
- **MQTT publishing** every 2 seconds
- **Valve control** via MQTT commands

### 4. Maintenance Operations
Supported commands via `pipes/maintenance/esp4`:

| Command | JSON Format | Action |
|---------|-------------|--------|
| Single Reset | `{"reset_sensor_to_initial": 1001}` | Reset sensor 1001 to initial config |
| Multiple Reset | `{"reset_to_initial": [1001, 1002, 1003]}` | Reset multiple sensors |
| Full Reset | `{"reset_all_to_initial": true}` | Reset all sensors to initial state |

## 📈 Sensor Data Format

### Published Sensor Data
```json
{
  "sensor_id": 1001,
  "pipe_id": 2001,
  "type": "fresh",
  "pressure_bar": 3.215,
  "level_pct": 85.423,
  "valve": 1,
  "flow_rate_Ls": 5.32,
  "velocity_ms": 1.45,
  "temperature_C": 15.3,
  "reynolds": 45000,
  "friction_factor": 0.0215,
  "head_loss_m": 12.3,
  "degrading": 1,
  "leak_coeff_pct": 15.5,
  "blockage_coeff_pct": 0.0
}
```

## ⚙️ Configuration Constants

### System Limits
```cpp
#define TOTAL_PIPES        200       // Maximum sensors (RAM limited)
#define UPDATE_INTERVAL    2000      // Physics update interval (ms)
#define PHYSICS_DT         0.1f      // Physics time step
#define MQTT_BATCH_SIZE    100       // Sensors per MQTT message
#define MAX_SENSORS_PER_MESSAGE 100  // Chunking threshold
```

### Physical Constants
```cpp
#define GRAVITY            9.81f     // m/s²
#define WATER_DENSITY      1000.0f   // kg/m³
#define WATER_VISCOSITY    0.001f    // Pa·s
#define AIR_PRESSURE       101325.0f // Pa
#define PIPE_ROUGHNESS     0.000015f // m
```

## 🚀 Setup Instructions

### 1. WiFi Configuration
```cpp
const char* WIFI_SSID = "YourNetwork";
const char* WIFI_PASSWORD = "YourPassword";
```

### 2. MQTT Configuration
```cpp
const char* MQTT_SERVER = "192.168.1.100";  // Linux broker IP
const int MQTT_PORT = 1883;
const char* MQTT_CLIENT_ID = "ESP32_PipeSystem4";
```

### 3. Topic Configuration (ESP-specific)
```cpp
// Change these for each ESP in the network:
const char* MQTT_TOPIC = "pipes/sensors4";      // Update number
const char* MQTT_CLIENT_ID = "ESP32_PipeSystem4"; // Update number
```

## 🔍 Debugging

### Serial Monitor Output
```
===========================================
   ESP4 - PIPE SENSOR SIMULATOR
===========================================
Connecting to WiFi: iPhone
WiFi connected!
IP address: 192.168.1.150
Connecting to MQTT broker...connected!
Subscribed to topics:
  - pipes/config/init4
  - pipes/maintenance/esp4
  - pipes/control/valve4
===========================================
WAITING FOR CONFIGURATION FROM LINUX...
===========================================
```

### Monitoring
- **Serial Monitor**: 115200 baud
- **MQTT Explorer**: Monitor all topics
- **System Summary**: Published to `pipes/system/summary4`

## ⚠️ Limitations & Considerations

### Memory Constraints
- ESP32-WROOM-32 has limited RAM (~520KB)
- `TOTAL_PIPES` set to 200 for stable operation
- JSON buffers pre-allocated to avoid heap fragmentation

### Network Considerations
- WiFi stability crucial for MQTT communication
- Automatic reconnection implemented
- Chunked messaging for large sensor counts

### Performance
- Physics simulation optimized with substeps
- MQTT publishing optimized with batch processing
- Degradation scheduling prevents simultaneous failures

## 📝 Customization Guide

### Adding New Sensor Types
1. Add to `PipeType` enum
2. Implement physics in `updatePhysics()`
3. Add type-specific constants

### Modifying Degradation Patterns
Adjust in `activateRandomDegradation()`:
- Failure probability distributions
- Degradation rates
- Leak vs blockage ratios

### Changing Update Rates
Modify constants:
- `UPDATE_INTERVAL`: Overall update frequency
- `PHYSICS_SUBSTEPS`: Simulation fidelity
- `PHYSICS_DT`: Time step size

## 🔗 Integration with Linux System

### Required Linux Components
1. **MQTT Broker** (Mosquitto recommended)
2. **Configuration Manager** (Python/Node.js script)
3. **Data Consumer** (Database/Visualization)

### Configuration Flow
```
Linux → MQTT(config) → ESP32 → Store Initial State → Begin Simulation
```

### Data Collection Flow
```
ESP32 → MQTT(sensor_data) → Linux → Process → Store → Visualize
```

## 🆘 Troubleshooting

### Common Issues
1. **WiFi Connection Fails**: Check SSID/password, signal strength
2. **MQTT Connection Fails**: Verify broker IP/port, network connectivity
3. **Memory Issues**: Reduce `TOTAL_PIPES`, optimize JSON buffers
4. **Sensor Data Missing**: Check config received, verify sensor IDs

### Debug Commands
- Serial command: `AT+GMR` (firmware version)
- MQTT test: `mosquitto_sub -t "pipes/#" -v`
- Network test: `ping ESP32_IP`

