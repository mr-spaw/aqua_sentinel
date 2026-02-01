# Software System Architecture Diagram

## System Overview

Here is the corrected and aligned ASCII chart. I have fixed the broken borders, realigned the connecting arrows, and cleaned up the internal spacing for the Controller and ESP32 sections to make the data flow clear.

```text
┌──────────────────────────────────────────────────────────────────────────┐
│                          SIMULATION (External)                           │
└──────────────────┬──────────────────┬─────────────────┬──────────────────┘
                   │                  │                 │
           (init_sensor.json) (TCP to Controller) (UDP to RL/Sim)
                   │                  │                 │
┌──────────────────▼──────────────────▼─────────────────▼──────────────────┐
│                       LINUX/ROS 2 DDS SYSTEM                             │
├──────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│   ┌──────────────────────────────────────────────────────────────────┐   │
│   │                     pipe_sensor_bridge Node                      │   │
│   │  ┌──────────────┐     ┌──────────────┐      ┌───────────────┐    │   │
│   │  │init_config_  │────►│MQTT          │─────►│mqtt_subscriber│    │   │
│   │  │publisher.cpp │     │Client        │      │.cpp           │    │   │
│   │  └──────────────┘     └──────────────┘      └───────┬───────┘    │   │
│   │                                                     │            │   │
│   └─────────────────────────────────────────────────────┼────────────┘   │
│                                                         │                │
│   ┌─────────────────────────────────────────────────────▼────────────┐   │
│   │                     ROS 2 DDS Layer                              │   │
│   │  ┌────────────────────────────────────────────────────────────┐  │   │
│   │  │                          Topics                            │  │   │
│   │  ├────────────────────────────────────────────────────────────┤  │   │
│   │  │ • /esp1/sensors                                            │  │   │
│   │  │ • /esp2/sensors                                            │  │   │
│   │  │ • /esp3/sensors                                            │  │   │
│   │  │ • /esp4/sensors                                            │  │   │
│   │  │ • /maintenance/command                                     │  │   │
│   │  │ • /maintenance/request                                     │  │   │
│   │  │ • /valve_control                                           │  │   │
│   │  └────────────────────────────────────────────────────────────┘  │   │
│   └──────────────────────────────────────────────────────────────────┘   │
│                                                                          │
│   ┌──────────────────────────────────────────────────────────────────┐   │
│   │                     Simulation Bridge                            │   │
│   │  ┌──────────────┐     ┌──────────────┐      ┌───────────────┐    │   │
│   │  │Topic         │────►│Physics       │─────►│UDP            │    │   │
│   │  │Subscriber    │     │Computation   │      │Publisher      │    │   │
│   │  └──────────────┘     └──────────────┘      └───────────────┘    │   │
│   └──────────────────────────────────────────────────────────────────┘   │
│                                                                          │
│   ┌──────────────────────────────────────────────────────────────────┐   │
│   │                     Controller Node                              │   │
│   │  ┌────────────────────────────────────────────────────────────┐  │   │
│   │  │                     controller.py                          │  │   │
│   │  ├────────────────────────────────────────────────────────────┤  │   │
│   │  │   Mode: RL            │         Mode: Non-RL               │  │   │
│   │  │   ┌────────┐          │         ┌─────────────────────┐    │  │   │
│   │  │   │REST    │◄───┼─────┼─────────┤Leak/Blockage        │    │  │   │
│   │  │   │API     │    │     │         │Detection            │    │  │   │
│   │  │   │Client  │    │     │         └──────────┬──────────┘    │  │   │
│   │  │   └────────┘    │     │                    │               │  │   │
│   │  │                 │     │         ┌──────────▼──────────┐    │  │   │
│   │  │                 │     │         │Valve Control        │    │  │   │
│   │  │                 │     │         │Maintenance Actions  │    │  │   │
│   │  │                 │     │         └─────────────────────┘    │  │   │
│   │  └─────────────────┴─────┴────────────────────────────────────┘  │   │
│   │            │                                      │              │   │
│   │  ┌─────────▼──────────────────────┐    ┌──────────▼───────────┐  │   │
│   │  │TCP Client to Simulation        │    │ROS Topic Publisher   │  │   │
│   │  │(Status Updates)                │    │(Control Commands)    │  │   │
│   │  └────────────────────────────────┘    └──────────────────────┘  │   │
│   └──────────────────────────────────────────────────────────────────┘   │
│                                                                          │
│   ┌──────────────────────────────────────────────────────────────────┐   │
│   │                     RL Agent (rl.py)                             │   │
│   │  ┌──────────────┐     ┌──────────────┐      ┌───────────────┐    │   │
│   │  │UDP Listener  │────►│RL Model      │─────►│REST API       │    │   │
│   │  │(RL Inputs)   │     │Computation   │      │Server         │    │   │
│   │  └──────────────┘     └──────────────┘      └───────────────┘    │   │
│   └──────────────────────────────────────────────────────────────────┘   │
│                                                                          │
│   ┌──────────────────────────────────────────────────────────────────┐   │
│   │                     Debug Module                                 │   │
│   │  ┌──────────────┐     ┌──────────────┐                           │   │
│   │  │UDP Listener  │────►│JSON Logger   │                           │   │
│   │  │(RL Data)     │     │              │                           │   │
│   │  └──────────────┘     └──────────────┘                           │   │
│   └──────────────────────────────────────────────────────────────────┘   │
│                                                                          │
└─────────────────────────────────────┬────────────────────────────────────┘
                                      │
                                      │ MQTT 
                                      ▼
┌──────────────────────────────────────────────────────────────────────────┐
│                          ESP32 DEVICES (x4)                              │
│   ┌──────────────────────────────────────────────────────────────────┐   │
│   │                       ESP32 Firmware                             │   │
│   │  ┌──────────────┐     ┌──────────────┐      ┌───────────────┐    │   │
│   │  │MQTT Client   │◄───►│Sensor Value  │◄───►│Sensor          │    │   │
│   │  │              │     │Degradation   │      │Hardware       │    │   │
│   │  └──────────────┘     └──────────────┘      └───────────────┘    │   │
│   └──────────────────────────────────────────────────────────────────┘   │
│                                                                          │
│   Sensor Types: Fresh Water & Sewage Water                               │
└──────────────────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────────────────┐
│                    EXTERNAL NETWORK COMMUNICATION                        │
│                                                                          │
│   ┌─────────────┐       ┌─────────────┐        ┌─────────────┐           │
│   │ MQTT Broker │       │ UDP Ports   │        │ TCP Ports   │           │
│   │ (Mosquitto) │       │ (RL/Sim)    │        │ (Controller)│           │
│   └─────────────┘       └─────────────┘        └─────────────┘           │
│                                                                          │
└──────────────────────────────────────────────────────────────────────────┘

```

## Component Descriptions

### 1. **Simulation (External)**
- Generates initial sensor configuration (`init_sensor.json`)
- Receives TCP updates from Controller
- Sends UDP packets with RL inputs/outputs
- Computes physics based on sensor data

### 2. **Pipe Sensor Bridge Node**
- **init_config_publisher.cpp**: Publishes initial sensor config via MQTT
- **mqtt_subscriber.cpp**: Listens to ESP32 sensor updates via MQTT
- Accumulates data and broadcasts to ROS 2 DDS layer

### 3. **ROS 2 DDS Topics**
- `/esp{1-4}/sensors`: Live sensor data from ESP32 devices
- `/maintenance/command`: Maintenance commands to ESP32
- `/maintenance/request`: Maintenance requests from ESP32
- `/valve_control`: Valve open/close commands

### 4. **Simulation Bridge**
- Subscribes to ESP sensor topics
- Computes physics models
- Publishes RL vectors via UDP

### 5. **Controller Node (controller.py)**
#### Non-RL Mode:
- Listens to ESP sensor topics
- Detects leaks and blockages
- Controls valves and maintenance actions
- Sends status to Simulation via TCP

#### RL Mode:
- Accepts valve/maintenance commands from RL Agent via REST API
- Executes RL decisions while monitoring critical conditions
- Sends background status to Simulation via TCP

### 6. **RL Agent (rl.py)**
- Listens to UDP streams from Simulation Bridge
- Computes RL decisions using sensor data
- Provides REST API for Controller to fetch commands
- Outputs sensor value recommendations for valve control

### 7. **Debug Module**
- Listens to UDP streams
- Logs RL data as JSON files for debugging

### 8. **ESP32 Devices (x4)**
- Runs firmware with sensor degradation logic
- Communicates via MQTT with Linux system
- Reports fresh/sewage water sensor values
- Responds to maintenance and valve commands

## Data Flow Sequence

1. **Initialization**: 
   - Simulation → `init_sensor.json` → pipe_sensor_bridge → MQTT → ESP32

2. **Continuous Sensor Data**:
   - ESP32 → MQTT → pipe_sensor_bridge → ROS Topics

3. **Physics Computation**:
   - ROS Topics → Simulation Bridge → Physics Model → UDP → Simulation/RL

4. **Control Decision**:
   - **Non-RL**: ROS Topics → Controller → Leak Detection → Valve Control → ROS Topics/TCP
   - **RL Mode**: ROS Topics/UDP → RL Agent → REST API → Controller → Valve Control → ROS Topics/TCP

5. **Debugging**:
   - UDP Streams → Debug Module → JSON Logs

## Communication Protocols
- **ROS 2 DDS**: Internal system communication
- **MQTT**: Linux ↔ ESP32 communication
- **TCP**: Controller ↔ Simulation (status updates)
- **UDP**: Simulation Bridge ↔ Simulation/RL (high-frequency data)
- **REST API**: RL Agent ↔ Controller (command interface)