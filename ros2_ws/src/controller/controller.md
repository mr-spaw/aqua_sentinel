# **PIPE NETWORK CONTROL SYSTEM**


## **🎯 SYSTEM OVERVIEW**

### **System Purpose**
Industrial-scale water distribution pipe network monitoring and control system with:
- **Real-time leak detection** using pressure/flow analysis
- **Reinforcement Learning (RL)** for optimal valve control
- **Multi-modal communication** (ROS2, UDP, REST API)
- **Predictive maintenance** with anomaly detection
- **Emergency response automation**
- **Comprehensive logging and persistence**

### **Key Features**
- **Dual Operation Modes**: RL Training vs Automatic Rule-based
- **Multi-threaded Processing**: Parallel sensor data processing
- **Persistence**: Automatic state saving/loading
- **REST API**: External control interface
- **Real-time Monitoring**: TCP/UDP data streaming
- **Safety Systems**: Emergency shutdown protocols

---

## **🏗️ ARCHITECTURE DESIGN**

### **Data Flow**
1. **Sensor Data Acquisition**
   - ROS2: `/esp1/sensors`, `/esp2/sensors`, etc.
   - UDP: Port 8888 with JSON format
   - REST API: Port 5000 for external control

2. **Data Processing Pipeline**
   ```
   Raw Data → Parsing → Feature Extraction → Normalization → Integration
   ```

3. **Decision Making**
   ```
   State → RL Model → Action Selection → Validation → Execution
   ```

---

## **📦 INSTALLATION & DEPENDENCIES**

### **System Requirements**
- **Ubuntu 22.04+** (ROS2 Humble recommended)
- **Python 3.8+**
- **Minimum 8GB RAM** (16GB recommended)
- **GPU** (optional, for accelerated training)

### **Installation Script**
```bash
# System dependencies
sudo apt update
sudo apt install -y \
    python3-numpy \
    python3-scipy \
    python3-pandas \
    python3-psutil \
    python3-yaml \
    python3-tqdm \
    python3-sklearn

# PyTorch (with CUDA if available)
pip3 install torch torchvision torchaudio

# RL Libraries
pip3 install \
    gymnasium \
    stable-baselines3 \
    sb3-contrib

# Data Processing
pip3 install \
    cloudpickle \
    protobuf \
    msgpack \
    lz4

# Logging & Utilities
pip3 install loguru rich flask flask-cors
```

### **ROS2 Installation (Optional)**
```bash
# Install ROS2 Humble
sudo apt install -y ros-humble-desktop
source /opt/ros/humble/setup.bash

# Create ROS2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## **🔢 ALGORITHM PROCESS**

### **Main Algorithm Flow**

#### **1. Initialization Phase**
```python
# Step 1: System Configuration
config = IndustrialConfig()  # Load all parameters

# Step 2: Logging Setup
directories, loggers = setup_logging()  # Multi-level logging

# Step 3: Component Initialization
data_collector = EnhancedIndustrialDataCollector(config)
env = EnhancedIndustrialPipeEnv(config, data_collector)
agent = EnhancedIndustrialRLAgent(config)

# Step 4: Persistence Loading
persistence_manager.load_system_state()  # Resume from last state
persistence_manager.load_replay_buffer()  # Load experience
```

#### **2. Data Processing Flow**
```python
def process_sensor_data(data):
    # Step 1: UDP Message Parsing
    parsed = udp_handler.parse_udp_message(data)
    
    # Step 2: Feature Extraction
    features = processor.extract_features(parsed)
    
    # Step 3: Scaling & Normalization
    scaled = scaler.transform(features)
    
    # Step 4: State Integration
    state = {
        'system': scaled['system'],
        'zones': scaled['zones'],
        'valves': scaled['valves'],
        'volumes': scaled['volumes'],
        'hourly_usage': scaled['hourly_usage']
    }
    
    # Step 5: Leak Prediction
    leak_pred = leak_predictor.predict(state)
    
    return integrated_state
```

#### **3. RL Training Loop**
```python
def train_agent():
    # Step 1: Environment Reset
    obs = env.reset()
    
    for timestep in range(total_timesteps):
        # Step 2: Action Selection
        action = model.predict(obs, deterministic=False)
        
        # Step 3: Environment Step
        next_obs, reward, done, info = env.step(action)
        
        # Step 4: Experience Storage
        replay_buffer.add(obs, action, reward, next_obs, done)
        
        # Step 5: Model Training
        if timestep > learning_starts:
            model.train(gradient_steps=1)
        
        # Step 6: Periodic Saving
        if timestep % checkpoint_freq == 0:
            persistence_manager.save_model_checkpoint()
        
        obs = next_obs
```

#### **4. Deployment Loop**
```python
def deploy_agent():
    while running:
        # Step 1: Get Current State
        state = data_collector.get_latest_integrated_state()
        
        # Step 2: RL Decision Making
        action = model.predict(state, deterministic=True)
        
        # Step 3: Action Execution
        executed = env.execute_action(action)
        
        # Step 4: Performance Monitoring
        performance_monitor.update(executed)
        
        # Step 5: Alert Checking
        alerts = alert_system.check_alerts(state)
        
        # Step 6: Periodic Persistence
        if step % autosave_freq == 0:
            persistence_manager.save_system_state()
```

---

## **🎮 CONTROL ALGORITHMS**

### **1. Valve Control Algorithm**
```python
class ValveController:
    def control_valve(self, sensor_id, action_value):
        """
        Valve Control Logic:
        
        Parameters:
        - sensor_id: Target valve (0-19)
        - action_value: [-1.0, 1.0], where:
          • -1.0 = Fully close
          • 0.0 = No change
          • 1.0 = Fully open
        
        Safety Checks:
        1. Rate limiting (max 12 changes/hour)
        2. Pressure bounds validation
        3. Emergency override detection
        4. Command timeout handling
        """
        
        # Validate action
        if abs(action_value) < config.VALVE_ACTION_THRESHOLD:
            return False  # Ignore small changes
        
        # Apply rate limiting
        if not self._check_rate_limit(sensor_id):
            return False
        
        # Check pressure safety
        if not self._validate_pressure_safety(sensor_id, action_value):
            return self._emergency_closure(sensor_id)
        
        # Execute command
        success = data_collector.send_valve_command(
            valve_id=sensor_id,
            command=action_value,
            priority=self._calculate_priority(action_value),
            emergency=False
        )
        
        return success
```

### **2. Leak Detection Algorithm**
```python
class LeakDetector:
    def detect_leak(self, pressure_data, flow_data):
        """
        Multi-stage leak detection:
        
        1. Pressure Trend Analysis:
           - Calculate pressure drop rate (kPa/min)
           - Compare against thresholds
        
        2. Flow Analysis:
           - Expected vs actual flow comparison
           - Mass balance calculations
        
        3. Zone Correlation:
           - Multi-zone pressure correlation
           - Pattern matching
        
        4. Machine Learning:
           - Transformer-based anomaly detection
           - Probability estimation
        """
        
        # Stage 1: Quick pressure check
        if pressure < config.MIN_PRESSURE:
            return {'probability': 0.9, 'type': 'CRITICAL_LEAK'}
        
        # Stage 2: Trend analysis
        drop_rate = self._calculate_pressure_trend(pressure_history)
        if drop_rate < config.PRESSURE_DROP_RATE_CRITICAL:
            return {'probability': 0.7, 'type': 'TRENDING_LEAK'}
        
        # Stage 3: ML prediction
        ml_prob = leak_predictor.predict(pressure_data, flow_data)
        
        return {
            'probability': ml_prob,
            'type': 'ML_DETECTED',
            'location': self._localize_leak(pressure_data)
        }
```

### **3. Emergency Response Protocol**
```python
class EmergencyProtocol:
    def handle_emergency(self, leak_probability, severity):
        """
        Emergency Response Sequence:
        
        1. CRITICAL (>95% probability):
           - Immediate valve closure
           - Maintenance dispatch
           - System notification
        
        2. HIGH (80-95%):
           - Gradual valve adjustment
           - Increased monitoring
           - Alert operators
        
        3. MEDIUM (60-80%):
           - Flow rerouting
           - Pressure adjustment
           - Watch mode
        """
        
        if leak_probability > config.EMERGENCY_SHUTDOWN_THRESHOLD:
            # Stage 1: Immediate isolation
            self._close_upstream_valves()
            self._open_alternate_paths()
            
            # Stage 2: Notify systems
            self._send_emergency_alerts()
            
            # Stage 3: Dispatch maintenance
            self._request_maintenance_team()
            
            return 'SYSTEM_SHUTDOWN'
        
        elif leak_probability > 0.8:
            return 'GRADUAL_CONTROL'
        
        else:
            return 'MONITOR_ONLY'
```

---

## **🤖 RL ALGORITHMS**

### **1. Soft Actor-Critic (SAC) Implementation**
```python
class EnhancedSAC:
    """
    SAC Algorithm with Industrial Adaptations:
    
    Key Features:
    - Temperature auto-tuning
    - Prioritized experience replay
    - Multi-critic networks
    - Delayed policy updates
    """
    
    def __init__(self, env, config):
        # Actor Network
        self.actor = ActorNetwork(
            input_dim=env.observation_space.shape[0],
            output_dim=env.action_space.shape[0],
            hidden_dims=[256, 256, 128]
        )
        
        # Critic Networks (Twin for stability)
        self.critic1 = CriticNetwork(
            input_dim=env.observation_space.shape[0] + env.action_space.shape[0],
            hidden_dims=[256, 256, 128]
        )
        self.critic2 = CriticNetwork(...)  # Twin
        
        # Temperature (entropy) auto-tuning
        self.target_entropy = -torch.prod(torch.Tensor(env.action_space.shape)).item()
        self.log_alpha = torch.zeros(1, requires_grad=True)
        
        # Replay Buffer with prioritization
        self.replay_buffer = PrioritizedReplayBuffer(
            capacity=config.BUFFER_SIZE,
            alpha=0.6,  # Prioritization exponent
            beta=0.4    # Importance sampling exponent
        )
```

### **2. Reward Engineering**
```python
class RewardCalculator:
    """
    Multi-component reward function for pipe networks:
    
    Components:
    1. Pressure compliance: Maintain 103kPa baseline
    2. Supply efficiency: Maximize water delivery
    3. Leak prevention: Minimize losses
    4. Energy efficiency: Minimize pumping
    5. Valve operation: Minimize wear
    6. Maintenance: Balance cost vs benefit
    """
    
    def calculate_reward(self, state, action, next_state):
        rewards = {}
        
        # 1. Pressure reward (quadratic penalty)
        pressure_diff = abs(state['pressure'] - 103.0)
        rewards['pressure'] = -config.REWARD_WEIGHTS['pressure'] * (pressure_diff ** 2)
        
        # 2. Efficiency reward
        efficiency = state['supply_efficiency'] / 100.0
        rewards['efficiency'] = config.REWARD_WEIGHTS['efficiency'] * efficiency
        
        # 3. Leak prevention reward
        leak_penalty = state['leak_probability'] * config.REWARD_WEIGHTS['leak_prevention']
        rewards['leak'] = -leak_penalty
        
        # 4. Valve operation cost
        valve_changes = abs(action['valve_changes'])
        rewards['valve'] = -config.REWARD_WEIGHTS['valve_operation'] * valve_changes
        
        # 5. Maintenance cost
        if action['maintenance_requested']:
            rewards['maintenance'] = -config.REWARD_WEIGHTS['maintenance_cost']
        
        # Total weighted reward
        total_reward = sum(rewards.values())
        
        return total_reward, rewards
```

### **3. Transformer-based State Representation**
```python
class StateTransformer(nn.Module):
    """
    Transformer encoder for temporal state representation:
    
    Architecture:
    - Multi-head self-attention
    - Positional encoding for time series
    - Zone-wise attention mechanisms
    - Feature aggregation layers
    """
    
    def __init__(self, config):
        super().__init__()
        
        # Input projection
        self.input_proj = nn.Linear(config.STATE_DIM, config.HIDDEN_DIM)
        
        # Transformer layers
        self.transformer_layers = nn.ModuleList([
            nn.TransformerEncoderLayer(
                d_model=config.HIDDEN_DIM,
                nhead=config.NUM_HEADS,
                dim_feedforward=config.FF_DIM,
                dropout=config.DROPOUT
            )
            for _ in range(config.NUM_LAYERS)
        ])
        
        # Zone attention (spatial awareness)
        self.zone_attention = nn.MultiheadAttention(
            embed_dim=config.HIDDEN_DIM,
            num_heads=config.NUM_HEADS,
            dropout=config.DROPOUT
        )
        
    def forward(self, x, zone_features=None):
        # Temporal encoding
        x = self.input_proj(x)
        
        # Transformer processing
        for layer in self.transformer_layers:
            x = layer(x)
        
        # Zone attention if available
        if zone_features is not None:
            zone_emb = self.zone_proj(zone_features)
            x, _ = self.zone_attention(x, zone_emb, zone_emb)
        
        return x
```

---

## **📋 PREREQUISITES**

### **Hardware Requirements**
- **CPU**: 4+ cores (8+ recommended)
- **RAM**: 8GB minimum (16GB for training)
- **Storage**: 10GB free space for logs/models
- **Network**: Ethernet for UDP/ROS2 communication

### **Software Requirements**
- **Ubuntu 20.04 LTS or newer**
- **Python 3.8+** with pip
- **ROS2 Humble** (optional, for ROS2 mode)
- **NVIDIA CUDA** (optional, for GPU acceleration)

### **Network Configuration**
```bash
# Open required ports
sudo ufw allow 5000/tcp  # REST API
sudo ufw allow 8888/udp  # UDP data
sudo ufw allow 9999/tcp  # TCP broadcast
```

---

## **⚙️ CONFIGURATION**

### **Main Configuration File Structure**
```python
@dataclass
class IndustrialConfig:
    # Hardware
    USE_GPU: bool = True
    NUM_WORKERS: int = 8
    
    # Network
    UDP_PORT: int = 8888
    REST_API_PORT: int = 5000
    ROS2_ENABLED: bool = True
    
    # RL Algorithm
    RL_ALGORITHM: str = "SAC"  # Options: SAC, PPO, TQC, TD3
    GAMMA: float = 0.99
    LEARNING_RATE: float = 3e-4
    BUFFER_SIZE: int = 50000
    
    # Safety Parameters
    MAX_PRESSURE: float = 150.0
    MIN_PRESSURE: float = 80.0
    EMERGENCY_SHUTDOWN_THRESHOLD: float = 0.95
    
    # Persistence
    LOAD_LAST_MODEL: bool = True
    SAVE_ON_SHUTDOWN: bool = True
    AUTOSAVE_FREQUENCY: int = 1000
```


---


## **📈 PERFORMANCE MONITORING**

### **Key Metrics Tracked**
```python
metrics = {
    'cpu_usage': psutil.cpu_percent(),
    'memory_usage': psutil.virtual_memory().percent,
    'disk_usage': psutil.disk_usage('/').percent,
    'active_threads': threading.active_count(),
    'queue_sizes': {
        'sensor_queue': sensor_queue.qsize(),
        'action_queue': action_queue.qsize(),
        'broadcast_queue': tcp_broadcast_queue.qsize()
    },
    'rl_performance': {
        'episode_reward': episode_reward,
        'average_reward': np.mean(recent_rewards),
        'valve_changes': valve_change_count,
        'leaks_prevented': leaks_prevented,
        'response_time': average_response_time
    }
}
```

### **Monitoring Dashboard**
Access via: `http://localhost:5000/metrics`

### **Alert Thresholds**
```python
ALERT_THRESHOLDS = {
    'cpu_usage': 90.0,        # >90% CPU usage
    'memory_usage': 85.0,     # >85% memory usage
    'queue_backlog': 1000,    # >1000 items in queue
    'response_time': 5.0,     # >5s response time
    'packet_loss': 0.1,       # >10% UDP packet loss
}
```

---

## **🔧 TROUBLESHOOTING**

### **Common Issues & Solutions**

#### **1. UDP Connection Issues**
```bash
# Check if port is open
sudo netstat -tulpn | grep :8888

# Test UDP connectivity
echo '{"test": "message"}' | nc -u localhost 8888

# Check firewall settings
sudo ufw status
```

#### **2. ROS2 Communication Problems**
```bash
# Check ROS2 topics
ros2 topic list

# Test sensor data
ros2 topic echo /esp1/sensors

# Check node status
ros2 node list
```

#### **3. API Connection Issues**
```bash
# Test API endpoint
curl -X POST http://localhost:5000/health

# Check API logs
tail -f ~/.industrial_pipe_rl/logs/restapi_logs/api_messages.log
```

#### **4. Memory Issues**
```python
# Reduce buffer size in config
config.BUFFER_SIZE = 25000  # From 50000

# Enable garbage collection
import gc
gc.collect()

# Monitor memory usage
import psutil
print(f"Memory: {psutil.virtual_memory().percent}%")
```

#### **5. Training Convergence Problems**
```python
# Adjust learning parameters
config.LEARNING_RATE = 1e-4  # Reduce learning rate
config.BATCH_SIZE = 128      # Smaller batches
config.GAMMA = 0.95          # Shorter discount horizon

# Enable more exploration
config.LEARNING_STARTS = 5000  # More random exploration
```

### **Debug Mode**
```bash
# Enable debug logging
python3 industrial_pipe_rl.py --mode deploy --log-level debug

# Verbose ROS2 output
export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity} {time}] [{name}]: {message}'
export RCUTILS_LOGGING_SEVERITY=DEBUG
```

### **Performance Optimization**
```python
# For better performance:
config.NUM_WORKERS = os.cpu_count()  # Use all cores
config.USE_GPU = torch.cuda.is_available()  # Enable GPU
config.PREFETCH_FACTOR = 4  # Prefetch more data

# Reduce logging overhead
config.LOGGING_LEVEL = 'WARNING'  # Less verbose
```

---

## **🚀 ROS2 LAUNCH COMMANDS**
### **1. Launch RL Agent Only**
```bash
# RL launch (deployment mode)
ros2 launch controller rl_agent.launch.py
```

### **2. Launch Control System Only**
```bash
# Basic launch (RL mode enabled)
ros2 launch controller control.launch.py

# Automatic mode (no RL)
ros2 launch industrial_pipe_rl control.launch.py rl_mode:=false
  --debug
```


## **🔧 BUILD AND INSTALL**

### **1. Build the Package**
```bash
# Navigate to workspace
cd ~/ros2_ws

# Build the package
colcon build --packages-select controller

```

### **2. Source the Workspace**
```bash
# Source the setup file
source ~/ros2_ws/install/setup.bash

# Add to .bashrc for persistence
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### ⚠️ IMPORTANT NOTE

The RL component is currently in DEVELOPMENT and not fully integrated.
This system is running in RULE-BASED AUTOMATIC MODE by default.
RL training and deployment are experimental features, thus expect bugs and crashes.