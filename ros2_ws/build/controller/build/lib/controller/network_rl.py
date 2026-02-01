#!/usr/bin/env python3
"""
INDUSTRIAL RL AGENT FOR PIPE MONITORING - COMPLETE SYSTEM WITH PERSISTENCE
ROS2 + UDP + REST API + Advanced RL Integration
"""

import json
import time
import socket
import threading
import numpy as np
import pandas as pd
from datetime import datetime, timedelta
from typing import Dict, List, Tuple, Optional, Any, Deque
from collections import deque, defaultdict, OrderedDict
from dataclasses import dataclass, field
import asyncio
import select
import pickle
import zlib
import logging
import logging.handlers
from enum import Enum, IntEnum
import statistics
import hashlib
from concurrent.futures import ThreadPoolExecutor, ProcessPoolExecutor
import psutil
import gc
import warnings
import os
import sys
import shutil
from pathlib import Path
warnings.filterwarnings('ignore')

# REST API Libraries
from flask import Flask, request, jsonify
from flask_cors import CORS
import requests

# Deep Learning Libraries
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader, TensorDataset
from torch.cuda.amp import autocast, GradScaler

# RL Libraries
import gymnasium as gym
from gymnasium import spaces
from stable_baselines3 import PPO, SAC, TD3
from stable_baselines3.common.vec_env import SubprocVecEnv, DummyVecEnv, VecMonitor, VecNormalize
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback, EvalCallback
from stable_baselines3.common.noise import OrnsteinUhlenbeckActionNoise, NormalActionNoise
from sb3_contrib import TQC, RecurrentPPO

# ROS2 Libraries
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
    from sensor_msgs.msg import Imu, Temperature, FluidPressure
    from std_msgs.msg import Float32, String, Int32, Bool, Header
    from geometry_msgs.msg import Twist, Vector3
    from nav_msgs.msg import Odometry
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    import message_filters
    ROS2_AVAILABLE = True
except ImportError:
    print("ROS2 not available, running in simulation mode")
    ROS2_AVAILABLE = False

# ============================================
# Enhanced Logging Configuration
# ============================================

def setup_logging(base_dir="~/.industrial_pipe_rl"):
    """Setup comprehensive logging structure"""
    
    # Base directory
    base_path = Path(base_dir).expanduser()
    base_path.mkdir(parents=True, exist_ok=True)
    
    # Create all subdirectories
    directories = {
        'base': base_path,
        'logs': base_path / 'logs',
        'models': base_path / 'models',
        'data': base_path / 'data',
        'cache': base_path / 'cache',
        'replay_buffers': base_path / 'replay_buffers',
        'checkpoints': base_path / 'checkpoints',
        'ros2_logs': base_path / 'logs' / 'ros2',
        'udp_logs': base_path / 'logs' / 'udp',
        'restapi_logs': base_path / 'logs' / 'restapi',
        'training_logs': base_path / 'logs' / 'training',
        'reward_logs': base_path / 'logs' / 'rewards',
        'processing_logs': base_path / 'logs' / 'processing',
        'alert_logs': base_path / 'logs' / 'alerts',
        'performance_logs': base_path / 'logs' / 'performance'
    }
    
    # Create all directories
    for dir_path in directories.values():
        dir_path.mkdir(parents=True, exist_ok=True)
    
    # Configure root logger
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - [%(filename)s:%(lineno)d] - %(message)s',
        handlers=[
            logging.FileHandler(directories['logs'] / 'main.log', mode='a'),
            logging.handlers.RotatingFileHandler(
                directories['logs'] / 'main_rotating.log',
                maxBytes=10*1024*1024,
                backupCount=10
            ),
            logging.StreamHandler()
        ]
    )
    
    # Create specialized loggers with file handlers
    def create_specialized_logger(name, log_file, level=logging.INFO):
        logger = logging.getLogger(name)
        logger.setLevel(level)
        
        # Remove existing handlers
        logger.handlers.clear()
        
        # Create file handler
        file_handler = logging.FileHandler(
            log_file,  # This should be the full path
            mode='a'
        )
        file_handler.setLevel(level)
        formatter = logging.Formatter('%(asctime)s - %(message)s')
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)
        
        # Add console handler for important logs
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.WARNING)
        logger.addHandler(console_handler)
        
        return logger
    
    # Create all specialized loggers with CORRECT paths
    loggers = {
        'ros2_logger': create_specialized_logger('ros2', directories['ros2_logs'] / 'ros2_messages.log'),
        'udp_logger': create_specialized_logger('udp', directories['udp_logs'] / 'udp_messages.log'),
        'restapi_logger': create_specialized_logger('restapi', directories['restapi_logs'] / 'api_messages.log'),
        'training_logger': create_specialized_logger('training', directories['training_logs'] / 'training_progress.log'),
        'reward_logger': create_specialized_logger('rewards', directories['reward_logs'] / 'reward_details.log'),
        'processing_logger': create_specialized_logger('processing', directories['processing_logs'] / 'data_processing.log'),
        'alert_logger': create_specialized_logger('alerts', directories['alert_logs'] / 'alert_messages.log'),
        'performance_logger': create_specialized_logger('performance', directories['performance_logs'] / 'system_performance.log'),
        'action_logger': create_specialized_logger('actions', directories['logs'] / 'action_history.log')  # Fixed: direct to logs directory
    }
    
    logger = logging.getLogger(__name__)
    logger.info(f"Logging initialized. Base directory: {base_path}")
    
    return directories, loggers

# Setup logging
directories, loggers = setup_logging()
logger = logging.getLogger(__name__)

# ============================================
# Enhanced Industrial Configuration with Persistence
# ============================================

@dataclass
class IndustrialConfig:
    """Complete industrial configuration with persistence"""
    
    # === Hardware ===
    USE_GPU: bool = torch.cuda.is_available()
    GPU_IDS: List[int] = field(default_factory=lambda: [0])
    NUM_WORKERS: int = 8
    BATCH_SIZE: int = 256
    PREFETCH_FACTOR: int = 2
    PIN_MEMORY: bool = True
    
    # === ROS2 ===
    ROS2_ENABLED: bool = ROS2_AVAILABLE
    ROS2_NODES: Dict[str, str] = field(default_factory=lambda: {
        'sensors': '/pipe_sensors/data',
        'valve_status': '/valves/status',
        'system_status': '/system/status',
        'pressure_zones': '/pressure_zones'
    })
    
    ROS2_CONTROL_TOPICS: Dict[str, str] = field(default_factory=lambda: {
        'valve_commands': '/valves/commands',
        'maintenance_requests': '/maintenance/requests',
        'emergency_alerts': '/alerts/emergency',
        'system_commands': '/system/commands'
    })
    
    # === UDP Configuration ===
    UDP_ENABLED: bool = True
    UDP_HOST: str = "0.0.0.0"
    UDP_PORT: int = 8888
    UDP_BUFFER_SIZE: int = 65535
    UDP_TIMEOUT: float = 0.01
    UDP_MAX_PACKETS: int = 1000
    
    # === REST API ===
    REST_API_ENABLED: bool = True
    REST_API_HOST: str = "0.0.0.0"
    REST_API_PORT: int = 5000
    REST_API_DEBUG: bool = False
    REST_API_KEY: str = "RL_aqua_sential"
    REST_ALLOWED_IDS: List[str] = field(default_factory=lambda: ["RL_aqua_sential"])
    
    # === Neural Network ===
    HIDDEN_DIMS: List[int] = field(default_factory=lambda: [512, 256, 128, 64])
    ATTENTION_HEADS: int = 8
    TRANSFORMER_LAYERS: int = 3
    DROPOUT_RATE: float = 0.1
    LAYER_NORM_EPS: float = 1e-5
    USE_BATCHNORM: bool = True
    ACTIVATION: str = "gelu"
    
    # === RL Algorithm ===
    RL_ALGORITHM: str = "SAC"
    GAMMA: float = 0.99
    TAU: float = 0.005
    LEARNING_RATE: float = 3e-4
    BUFFER_SIZE: int = 50000  # Reduced from 1M to save memory
    LEARNING_STARTS: int = 1000
    TRAIN_FREQUENCY: int = 1
    GRADIENT_STEPS: int = 1
    MAX_EPISODE_STEPS: int = 1000
    
    # === Training ===
    TOTAL_TIMESTEPS: int = 1000000
    EVAL_FREQUENCY: int = 10000
    EVAL_EPISODES: int = 10
    CHECKPOINT_FREQUENCY: int = 50000
    SAVE_REPLAY_BUFFER: bool = True
    AUTOSAVE_FREQUENCY: int = 1000  # Save every 1000 steps
    
    # === Valve Control ===
    VALVE_ACTION_THRESHOLD: float = 0.1
    MAINTENANCE_ACTION_THRESHOLD: float = 0.7
    MAX_VALVE_CHANGES_PER_HOUR: int = 12
    VALVE_RESPONSE_TIMEOUT: float = 5.0
    
    # === Reward Engineering ===
    REWARD_WEIGHTS: Dict[str, float] = field(default_factory=lambda: {
        'pressure_compliance': 2.0,
        'supply_efficiency': 1.5,
        'energy_efficiency': 0.5,
        'leak_prevention': 3.0,
        'valve_operation_cost': -0.1,
        'maintenance_cost': -1.0,
        'reservoir_stability': 0.3,
        'flow_stability': 0.8,
        'water_quality': 0.5,
        'response_time': 0.2,
        'sensor_health': 0.3
    })
    
    # === Predictive Analytics ===
    PREDICTION_HORIZON: int = 24
    LEAK_PREDICTION_WINDOW: int = 12
    ANOMALY_DETECTION_THRESHOLD: float = 3.0
    
    # === Data Processing ===
    HISTORY_LENGTH: int = 10  # Reduced from 100 to save memory
    FEATURE_SCALING: str = "robust"
    DATA_AUGMENTATION: bool = True
    SYNTHETIC_DATA_RATIO: float = 0.1
    
    # === Monitoring ===
    MONITORING_INTERVAL: int = 60
    ALERT_THRESHOLDS: Dict[str, float] = field(default_factory=lambda: {
        'leak_probability': 0.8,
        'pressure_drop_rate': -2.0,
        'efficiency_drop': 0.1,
        'valve_oscillation': 10,
        'sensor_failure': 0.3,
        'communication_loss': 0.5
    })
    
    # === Safety ===
    MAX_PRESSURE: float = 150.0
    MIN_PRESSURE: float = 80.0
    EMERGENCY_SHUTDOWN_THRESHOLD: float = 0.95
    MAX_FLOW_RATE: float = 100.0
    MIN_FLOW_RATE: float = 5.0
    
    # === Persistence ===
    LOAD_LAST_MODEL: bool = True
    LOAD_REPLAY_BUFFER: bool = True
    SAVE_ON_SHUTDOWN: bool = True
    COMPRESS_SAVES: bool = True
    
    def __post_init__(self):
        """Create necessary directories and setup device"""
        # Use global directories
        self.MODEL_SAVE_PATH = str(directories['models'])
        self.DATA_PATH = str(directories['data'])
        self.LOG_PATH = str(directories['logs'])
        self.CACHE_PATH = str(directories['cache'])
        self.REPLAY_BUFFER_PATH = str(directories['replay_buffers'])
        self.CHECKPOINT_PATH = str(directories['checkpoints'])
        
        # Set device
        if self.USE_GPU and torch.cuda.is_available():
            self.DEVICE = torch.device(f"cuda:{self.GPU_IDS[0]}")
        else:
            self.DEVICE = torch.device("cpu")
        logger.info(f"Using device: {self.DEVICE}")
        
        # ROS2 QoS profiles
        if ROS2_AVAILABLE:
            self.SENSOR_QOS = QoSProfile(
                depth=10,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE
            )
            
            self.CONTROL_QOS = QoSProfile(
                depth=5,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
            )
        
        logger.info("Configuration initialized with persistence enabled")

# ============================================
# Persistence Manager
# ============================================

class PersistenceManager:
    """Manages saving and loading of all system state"""
    
    def __init__(self, config: IndustrialConfig):
        self.config = config
        self.state_file = Path(config.CACHE_PATH) / "system_state.json"
        self.last_save_time = None
        
    def save_system_state(self, agent: 'EnhancedIndustrialRLAgent', 
                         data_collector: 'EnhancedIndustrialDataCollector'):
        """Save complete system state"""
        try:
            state = {
                'timestamp': datetime.now().isoformat(),
                'agent': {
                    'is_training': agent.is_training,
                    'current_timestep': agent.current_timestep,
                    'episode_count': getattr(agent, 'episode_count', 0),
                    'total_reward': getattr(agent, 'total_reward', 0.0),
                    'performance_summary': agent.performance_monitor.get_summary(),
                    'active_alerts': agent.alert_system.get_active_alerts()
                },
                'data_collector': {
                    'stats': data_collector.get_statistics(),
                    'packet_count': data_collector.processor.udp_handler.packet_count
                },
                'environment': {
                    'episode_step': getattr(agent.env, 'episode_step', 0),
                    'episode_reward': getattr(agent.env, 'episode_reward', 0.0),
                    'metrics': getattr(agent.env, 'metrics', {})
                },
                'config': {
                    'RL_ALGORITHM': self.config.RL_ALGORITHM,
                    'LEARNING_RATE': self.config.LEARNING_RATE,
                    'BUFFER_SIZE': self.config.BUFFER_SIZE
                }
            }
            
            with open(self.state_file, 'w') as f:
                json.dump(state, f, indent=2)
            
            self.last_save_time = datetime.now()
            logger.info(f"System state saved to {self.state_file}")
            
        except Exception as e:
            logger.error(f"Failed to save system state: {e}")
    
    def load_system_state(self) -> Optional[Dict]:
        """Load system state"""
        try:
            if self.state_file.exists():
                with open(self.state_file, 'r') as f:
                    state = json.load(f)
                
                logger.info(f"System state loaded from {self.state_file}")
                return state
            else:
                logger.info("No previous system state found")
                return None
                
        except Exception as e:
            logger.error(f"Failed to load system state: {e}")
            return None
    
    def save_replay_buffer(self, model):
        """Save replay buffer"""
        if hasattr(model, 'replay_buffer'):
            buffer_file = Path(self.config.REPLAY_BUFFER_PATH) / "replay_buffer.pkl"
            try:
                with open(buffer_file, 'wb') as f:
                    pickle.dump(model.replay_buffer, f)
                logger.info(f"Replay buffer saved ({len(model.replay_buffer)} transitions)")
                
                # Save buffer info
                info_file = Path(self.config.REPLAY_BUFFER_PATH) / "buffer_info.json"
                buffer_info = {
                    'timestamp': datetime.now().isoformat(),
                    'size': len(model.replay_buffer),
                    'algorithm': self.config.RL_ALGORITHM
                }
                with open(info_file, 'w') as f:
                    json.dump(buffer_info, f, indent=2)
                    
            except Exception as e:
                logger.error(f"Failed to save replay buffer: {e}")
    
    def load_replay_buffer(self, model):
        """Load replay buffer"""
        if self.config.LOAD_REPLAY_BUFFER:
            buffer_file = Path(self.config.REPLAY_BUFFER_PATH) / "replay_buffer.pkl"
            if buffer_file.exists():
                try:
                    with open(buffer_file, 'rb') as f:
                        model.replay_buffer = pickle.load(f)
                    logger.info(f"Replay buffer loaded ({len(model.replay_buffer)} transitions)")
                except Exception as e:
                    logger.warning(f"Failed to load replay buffer: {e}")
    
    def save_model_checkpoint(self, model, step: int, suffix: str = ""):
        """Save model checkpoint"""
        checkpoint_file = Path(self.config.CHECKPOINT_PATH) / f"model_checkpoint_{step}{suffix}.zip"
        try:
            model.save(str(checkpoint_file))
            logger.info(f"Model checkpoint saved: {checkpoint_file.name}")
            
            # Save checkpoint info
            info_file = Path(self.config.CHECKPOINT_PATH) / f"checkpoint_info_{step}{suffix}.json"
            checkpoint_info = {
                'timestamp': datetime.now().isoformat(),
                'step': step,
                'algorithm': self.config.RL_ALGORITHM,
                'file': checkpoint_file.name
            }
            with open(info_file, 'w') as f:
                json.dump(checkpoint_info, f, indent=2)
                
        except Exception as e:
            logger.error(f"Failed to save model checkpoint: {e}")
    
    def find_latest_model(self):
        """Find latest saved model"""
        model_files = list(Path(self.config.MODEL_SAVE_PATH).glob("*.zip"))
        if model_files:
            # Sort by modification time
            model_files.sort(key=lambda x: x.stat().st_mtime, reverse=True)
            return str(model_files[0])
        return None

# ============================================
# Enhanced UDP Message Handler with Logging
# ============================================

class UDPMessageHandler:
    """Handles UDP messages with comprehensive logging"""
    
    def __init__(self, config: IndustrialConfig):
        self.config = config
        self.packet_buffer = deque(maxlen=config.UDP_MAX_PACKETS)
        self.last_packet_time = None
        self.packet_count = 0
        self.udp_logger = loggers['udp_logger']
        self.packet_log_file = directories['udp_logs'] / "packet_details.log"
        
        # CSV logging for packets
        self.csv_logger = logging.getLogger('udp_csv')
        self.csv_logger.handlers.clear()
        
        csv_handler = logging.FileHandler(
            directories['udp_logs'] / "packets.csv",
            mode='a'
        )
        csv_handler.setLevel(logging.INFO)
        
        # Custom formatter for CSV
        class CSVFormatter(logging.Formatter):
            def format(self, record):
                return f"{record.timestamp},{record.source},{record.packet_size},{record.data_type}"
        
        csv_handler.setFormatter(CSVFormatter())
        self.csv_logger.addHandler(csv_handler)
        self.csv_logger.propagate = False
        
    def parse_udp_message(self, data: bytes) -> Optional[Dict]:
        """Parse UDP message in your format with logging"""
        try:
            # Parse JSON
            message = json.loads(data.decode('utf-8'))
            
            # Extract metadata
            metadata = message.get('metadata', {})
            packets = message.get('packets', [])
            
            if not packets:
                return None
            
            # Process first packet (most recent)
            packet = packets[0]
            packet_data = packet.get('data', {})
            
            # Extract key metrics
            processed_data = {
                'timestamp': packet.get('timestamp', datetime.now().isoformat()),
                'avg_flow_rate': packet_data.get('avg_flow_rate', 0.0),
                'cumulative_volumes': packet_data.get('cumulative_volumes', []),
                'day_type': packet_data.get('day_type', 'weekday'),
                'episode_progress': packet_data.get('episode_progress', 0.0),
                'flow_variance': packet_data.get('flow_variance', 0.0),
                'hourly_usage': packet_data.get('hourly_usage', []),
                'last_action_times': packet_data.get('last_action_times', []),
                'non_revenue_water': packet_data.get('non_revenue_water', 0.0),
                'peak_flow_rate': packet_data.get('peak_flow_rate', 0.0),
                'pressure_compliance': packet_data.get('pressure_compliance', 0.0),
                'pump_capacity_available': packet_data.get('pump_capacity_available', 0.0),
                'recent_action_count': packet_data.get('recent_action_count', 0),
                'reservoir_level_pct': packet_data.get('reservoir_level_pct', 0.0),
                'reservoir_trend': packet_data.get('reservoir_trend', 'stable'),
                'response_time': packet_data.get('response_time', 0.0),
                'sensor_count': packet_data.get('sensor_count', 0),
                'service_continuity': packet_data.get('service_continuity', 0.0),
                'sim_time_cos': packet_data.get('sim_time_cos', 0.0),
                'sim_time_sin': packet_data.get('sim_time_sin', 0.0),
                'supply_efficiency': packet_data.get('supply_efficiency', 0.0),
                'supply_margin': packet_data.get('supply_margin', 0.0),
                'time_of_day': packet_data.get('time_of_day', 0.0),
                'valve_states': packet_data.get('valve_states', []),
                'zone_count': packet_data.get('zone_count', 0),
                'zones': packet_data.get('zones', [])
            }
            
            # Store in buffer
            self.packet_buffer.append(processed_data)
            self.last_packet_time = datetime.now()
            self.packet_count += 1
            
            # Log UDP packet
            self.udp_logger.info(f"UDP Packet received: {processed_data['timestamp']}")
            
            # Log detailed packet info to CSV
            log_record = logging.LogRecord(
                name='udp_csv',
                level=logging.INFO,
                pathname='',
                lineno=0,
                msg='',
                args=(),
                exc_info=None
            )
            log_record.timestamp = processed_data['timestamp']
            log_record.source = 'UDP'
            log_record.packet_size = len(data)
            log_record.data_type = 'sensor_data'
            self.csv_logger.handle(log_record)
            
            # Save packet to JSON file for debugging
            if self.packet_count % 100 == 0:  # Save every 100 packets
                packet_file = directories['udp_logs'] / f"packet_{self.packet_count:06d}.json"
                with open(packet_file, 'w') as f:
                    json.dump(processed_data, f, indent=2)
            
            return processed_data
            
        except json.JSONDecodeError as e:
            self.udp_logger.warning(f"UDP JSON decode error: {e}")
            return None
        except Exception as e:
            self.udp_logger.error(f"UDP parsing error: {e}")
            return None
    
    def get_latest_data(self) -> Optional[Dict]:
        """Get latest UDP data"""
        if self.packet_buffer:
            return self.packet_buffer[-1]
        return None
    
    def get_statistics(self) -> Dict:
        """Get UDP statistics"""
        return {
            'packets_received': self.packet_count,
            'buffer_size': len(self.packet_buffer),
            'last_packet_time': self.last_packet_time.isoformat() if self.last_packet_time else None,
            'packets_per_second': self._calculate_packet_rate(),
            'total_bytes': self.packet_count * 1024  # Approximate
        }
    
    def _calculate_packet_rate(self) -> float:
        """Calculate packet reception rate"""
        if self.packet_count < 2 or not self.last_packet_time:
            return 0.0
        
        elapsed = (datetime.now() - self.last_packet_time).total_seconds()
        if elapsed > 0:
            return self.packet_count / elapsed
        return 0.0

# ============================================
# Enhanced Data Processor with Logging
# ============================================

class EnhancedSensorDataProcessor:
    """Enhanced processor for your specific UDP format with logging"""
    
    def __init__(self, config: IndustrialConfig):
        self.config = config
        self.scaler = RobustScaler()
        self.feature_names = []
        self.sensor_stats = defaultdict(lambda: {'mean': 0.0, 'std': 1.0, 'count': 0})
        self.valve_stats = defaultdict(lambda: {'position_history': deque(maxlen=100)})
        self.udp_handler = UDPMessageHandler(config)
        self.processing_logger = loggers['processing_logger']
        
        # CSV logging for processed features
        self.feature_log_file = directories['processing_logs'] / "processed_features.csv"
        self._init_feature_logging()
        
    def _init_feature_logging(self):
        """Initialize feature logging"""
        if not self.feature_log_file.exists():
            with open(self.feature_log_file, 'w') as f:
                f.write("timestamp,system_features_count,zone_features_count,valve_features_count\n")
    
    def process_udp_data(self, udp_message: Dict) -> Dict[str, np.ndarray]:
        """Process your specific UDP message format with logging"""
        try:
            # Parse UDP message
            parsed_data = self.udp_handler.parse_udp_message(
                json.dumps(udp_message).encode() if isinstance(udp_message, dict) else udp_message
            )
            
            if not parsed_data:
                return {}
            
            processed = {}
            
            # System metrics
            system_features = np.array([
                parsed_data['avg_flow_rate'],
                parsed_data['reservoir_level_pct'] / 100.0,
                parsed_data['supply_efficiency'] / 100.0 if parsed_data['supply_efficiency'] > 100.0 else parsed_data['supply_efficiency'],
                parsed_data['non_revenue_water'] / 100.0 if parsed_data['non_revenue_water'] > 100.0 else parsed_data['non_revenue_water'],
                parsed_data['pressure_compliance'],
                parsed_data['peak_flow_rate'],
                parsed_data['pump_capacity_available'],
                parsed_data['time_of_day'] / 24.0,
                1.0 if parsed_data['day_type'] == 'weekday' else 0.0,
                parsed_data['sim_time_cos'],
                parsed_data['sim_time_sin'],
                parsed_data['episode_progress'],
                parsed_data['flow_variance'],
                min(parsed_data['sensor_count'] / 500.0, 1.0),
                parsed_data['response_time'] / 100.0 if parsed_data['response_time'] > 0 else 0.0,
                parsed_data['service_continuity'],
                parsed_data['supply_margin'] / 100.0 if parsed_data['supply_margin'] > 100.0 else parsed_data['supply_margin']
            ], dtype=np.float32)
            
            processed['system'] = system_features
            
            # Zone features
            zone_features = []
            zones = parsed_data.get('zones', [])
            for i, zone in enumerate(zones[:20]):
                zone_features.extend([
                    zone.get('avg_pressure', 0.0) / 300.0,
                    zone.get('flow', 0.0),
                    zone.get('flow_to_pressure_ratio', 0.0),
                    1.0 if zone.get('leak_flag', False) else 0.0,
                    1.0 if zone.get('pressure_violation', False) else 0.0,
                    1.0 if zone.get('overflow_flag', False) else 0.0,
                    zone.get('leak_severity', 0.0),
                    zone.get('pressure_variance', 0.0),
                    zone.get('historical_flow', 0.0)
                ])
            
            if len(zone_features) < 180:
                zone_features.extend([0.0] * (180 - len(zone_features)))
            
            processed['zones'] = np.array(zone_features[:180], dtype=np.float32)
            
            # Valve states
            valve_states = np.array(parsed_data.get('valve_states', [])[:100], dtype=np.float32) / 100.0
            if len(valve_states) < 100:
                valve_states = np.pad(valve_states, (0, 100 - len(valve_states)), mode='constant')
            
            processed['valves'] = valve_states
            
            # Cumulative volumes
            cumulative_volumes = np.array(parsed_data.get('cumulative_volumes', [])[:50], dtype=np.float32)
            if len(cumulative_volumes) > 0:
                max_volume = np.max(cumulative_volumes) if np.max(cumulative_volumes) > 0 else 1.0
                cumulative_volumes = cumulative_volumes / max_volume
            
            if len(cumulative_volumes) < 50:
                cumulative_volumes = np.pad(cumulative_volumes, (0, 50 - len(cumulative_volumes)), mode='constant')
            
            processed['volumes'] = cumulative_volumes[:50]
            
            # Hourly usage
            hourly_usage = np.array(parsed_data.get('hourly_usage', [])[:24], dtype=np.float32)
            if len(hourly_usage) > 0:
                max_usage = np.max(hourly_usage) if np.max(hourly_usage) > 0 else 1.0
                hourly_usage = hourly_usage / max_usage
            
            processed['hourly_usage'] = hourly_usage[:24]
            
            # Last action times
            last_action_times = parsed_data.get('last_action_times', [])
            if last_action_times:
                current_time = time.time()
                time_since_actions = [current_time - t for t in last_action_times[:20]]
                time_since_actions = np.array(time_since_actions, dtype=np.float32) / 3600.0
                
                if len(time_since_actions) < 20:
                    time_since_actions = np.pad(time_since_actions, (0, 20 - len(time_since_actions)), mode='constant')
                
                processed['action_times'] = time_since_actions[:20]
            
            # Log processing statistics
            self.processing_logger.info(
                f"Processed features - System: {len(system_features)}, "
                f"Zones: {len(processed['zones'])}, "
                f"Valves: {len(processed['valves'])}"
            )
            
            # Log to CSV
            with open(self.feature_log_file, 'a') as f:
                f.write(f"{datetime.now().isoformat()},"
                       f"{len(system_features)},"
                       f"{len(processed['zones'])},"
                       f"{len(processed['valves'])}\n")
            
            return processed
            
        except Exception as e:
            self.processing_logger.error(f"Error processing UDP data: {e}")
            return {}

# ============================================
# Robust Scaler for Industrial Data
# ============================================

class RobustScaler:
    """Robust scaler for industrial data"""
    
    def __init__(self):
        self.center_ = None
        self.scale_ = None
        
    def fit(self, X):
        """Fit scaler"""
        if len(X) == 0:
            return self
        
        self.center_ = np.median(X, axis=0)
        q75, q25 = np.percentile(X, [75, 25], axis=0)
        self.scale_ = q75 - q25
        self.scale_[self.scale_ == 0] = 1.0
        
        return self
    
    def transform(self, X):
        """Transform data"""
        if self.center_ is None or self.scale_ is None:
            return X
        
        return (X - self.center_) / self.scale_
    
    def fit_transform(self, X):
        """Fit and transform"""
        self.fit(X)
        return self.transform(X)

# ============================================
# Enhanced Industrial Data Collector with Logging
# ============================================

class EnhancedIndustrialDataCollector:
    """Enhanced data collector for industrial deployment with logging"""
    
    def __init__(self, config: IndustrialConfig):
        self.config = config
        self.processor = EnhancedSensorDataProcessor(config)
        
        # Data buffers
        self.ros2_buffer = deque(maxlen=1000)
        self.udp_buffer = deque(maxlen=config.UDP_MAX_PACKETS)
        self.integrated_buffer = deque(maxlen=10000)
        
        # ROS2 node
        self.ros2_node = None
        self.ros2_executor = None
        
        # Loggers
        self.ros2_logger = loggers['ros2_logger']
        self.action_logger = loggers['action_logger']
        
        # Command logging
        self.command_log_file = directories['logs'] / "command_history.csv"
        self._init_command_logging()
        
        if config.ROS2_ENABLED:
            self._init_ros2()
        
        # UDP collector
        if config.UDP_ENABLED:
            self._start_udp_collector()
        
        # Synchronization
        self.lock = threading.Lock()
        self.running = True
        
        # Statistics
        self.stats = {
            'ros2_messages': 0,
            'udp_messages': 0,
            'integration_errors': 0,
            'valve_commands_sent': 0,
            'maintenance_requests_sent': 0,
            'emergency_alerts_sent': 0,
            'last_update': datetime.now()
        }
        
        logger.info(f"Enhanced Data Collector initialized")
    
    def _init_command_logging(self):
        """Initialize command logging"""
        if not self.command_log_file.exists():
            with open(self.command_log_file, 'w') as f:
                f.write("timestamp,command_type,id,value,priority,emergency,success\n")
    
    def _init_ros2(self):
        """Initialize ROS2"""
        try:
            if not ROS2_AVAILABLE:
                logger.warning("ROS2 not available")
                return
            
            rclpy.init()
            self.ros2_node = ROS2PipeNode(self.config, self)
            self.ros2_executor = MultiThreadedExecutor(num_threads=4)
            self.ros2_executor.add_node(self.ros2_node)
            
            ros2_thread = threading.Thread(target=self._ros2_processing_loop, daemon=True)
            ros2_thread.start()
            
            self.ros2_logger.info("ROS2 initialized")
            
        except Exception as e:
            logger.error(f"ROS2 initialization error: {e}")
            self.config.ROS2_ENABLED = False
    
    def _ros2_processing_loop(self):
        """ROS2 processing loop"""
        try:
            while self.running and rclpy.ok():
                self.ros2_executor.spin_once(timeout_sec=0.1)
        except Exception as e:
            self.ros2_logger.error(f"ROS2 processing error: {e}")
        finally:
            if self.ros2_node:
                self.ros2_node.destroy_node()
            rclpy.shutdown()
    
    def _start_udp_collector(self):
        """Start UDP collector for your format"""
        def udp_collector():
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind((self.config.UDP_HOST, self.config.UDP_PORT))
            sock.settimeout(self.config.UDP_TIMEOUT)
        
            logger.info(f"UDP collector on {self.config.UDP_HOST}:{self.config.UDP_PORT}")
            print(f"🚀 UDP LISTENER STARTED on {self.config.UDP_HOST}:{self.config.UDP_PORT}")
            print(f"   Buffer size: {self.config.UDP_BUFFER_SIZE}")
            print(f"   Timeout: {self.config.UDP_TIMEOUT}s")
        
            while self.running:
                try:
                    data, addr = sock.recvfrom(self.config.UDP_BUFFER_SIZE)
                
                    print(f"\n📦 [UDP RAW] Packet received!")
                    print(f"   From: {addr[0]}:{addr[1]}")
                    print(f"   Size: {len(data):,} bytes")
                    print(f"   Time: {datetime.now().strftime('%H:%M:%S.%f')[:-3]}")
                
                    # Show first 200 characters
                    try:
                        text_preview = data.decode('utf-8')[:200]
                        if '{' in text_preview:
                            print(f"   Preview: {text_preview[:100]}...")
                    except:
                        print(f"   Binary data detected")
                
                    try:
                        # Parse your UDP format
                        udp_json = json.loads(data.decode('utf-8'))
                    
                        with self.lock:
                            self.udp_buffer.append({
                            'timestamp': datetime.now(),
                            'data': udp_json,
                            'source': addr
                            })
                            self.stats['udp_messages'] += 1
                        
                            print(f"✅ [UDP BUFFER] Added to buffer")
                            print(f"   Buffer size: {len(self.udp_buffer)}/{self.config.UDP_MAX_PACKETS}")
                            print(f"   Total UDP messages: {self.stats['udp_messages']}")
                        
                            # Process immediately
                            self._integrate_data()
                        
                    except json.JSONDecodeError as e:
                        print(f"❌ [UDP ERROR] JSON decode error: {e}")
                        logger.warning(f"UDP JSON error: {e}")
                    except Exception as e:
                        print(f"❌ [UDP ERROR] Processing error: {e}")
                        logger.error(f"UDP processing error: {e}")
                        self.stats['integration_errors'] += 1
                    
                except socket.timeout:
                    continue
                except Exception as e:
                    print(f"❌ [UDP SOCKET ERROR] {e}")
                    logger.error(f"UDP socket error: {e}")
                    time.sleep(1)
    
    def process_ros2_data(self, data_type: str, data: Dict):
        """Process ROS2 data"""
        with self.lock:
            self.ros2_buffer.append({
                'timestamp': datetime.now(),
                'type': data_type,
                'data': data
            })
            self.stats['ros2_messages'] += 1
            self.ros2_logger.info(f"ROS2 {data_type}: {data}")
            self._integrate_data()
    
    def _integrate_data(self):
        """Integrate all data sources"""
        try:
            print(f"\n🔄 [INTEGRATE] Starting integration...")
        
            # Get latest data
            ros2_data = {}
            if self.ros2_buffer:
                latest_ros2 = self.ros2_buffer[-1]
                ros2_data[latest_ros2['type']] = latest_ros2['data']
                print(f"   ROS2 data available: {list(ros2_data.keys())}")
            else:
                print(f"   No ROS2 data")
            
            udp_data = {}
            if self.udp_buffer:
                latest_udp = self.udp_buffer[-1]
                udp_data = latest_udp['data']
                print(f"   UDP data available: YES ({len(self.udp_buffer)} in buffer)")
            
            # Show UDP structure
            if isinstance(udp_data, dict):
                print(f"   UDP keys: {list(udp_data.keys())}")
                if 'metadata' in udp_data:
                    print(f"   Metadata: {udp_data['metadata'].get('save_time', 'No timestamp')}")
                if 'packets' in udp_data:
                    packets = udp_data.get('packets', [])
                    print(f"   Packets: {len(packets)}")
                    if packets:
                        print(f"   First packet size: {packets[0].get('size', 'Unknown')}")
            else:
                print(f"❌ [INTEGRATE ERROR] NO UDP DATA IN BUFFER!")
            
            # Process through enhanced processor
            print(f"   Processing UDP data through processor...")
            integrated_state = self.processor.process_udp_data(udp_data)
        
            print(f"   Processed state keys: {list(integrated_state.keys())}")
            for key, value in integrated_state.items():
                if isinstance(value, np.ndarray):
                    print(f"     {key}: array shape {value.shape}")
                elif isinstance(value, (int, float)):
                    print(f"     {key}: {value}")
                
            # Add ROS2 data if available
            if ros2_data:
                print(f"   Adding ROS2 data...")
                for key, data in ros2_data.items():
                    if 'sensor' in key:
                        sensor_id = data.get('sensor_id', 0)
                        if sensor_id < 20:
                            integrated_state[f'sensor_{sensor_id}'] = np.array([
                                data.get('pressure', 0.0),
                                data.get('flow_rate', 0.0),
                                data.get('temperature', 20.0),
                                data.get('valve_position', 0.0),
                                data.get('battery_level', 100.0) / 100.0
                        ], dtype=np.float32)
                        
            full_state = {
                'timestamp': datetime.now(),
                'ros2_data': ros2_data,
                'udp_data': udp_data,
                'integrated_state': integrated_state,
                'processed': True
            }
        
            self.integrated_buffer.append(full_state)
            self.stats['last_update'] = datetime.now()
        
            print(f"✅ [INTEGRATE SUCCESS] Added to integrated buffer")
            print(f"   Integrated buffer size: {len(self.integrated_buffer)}")
            print(f"   Has integrated_state: {'integrated_state' in full_state}")
            print(f"   Has data: {bool(integrated_state)}")
        
        except Exception as e:
            print(f"❌ [INTEGRATE ERROR] {e}")
            logger.error(f"Data integration error: {e}")
            self.stats['integration_errors'] += 1
    
    def get_latest_integrated_state(self) -> Optional[Dict]:
        """Get latest integrated state"""
        with self.lock:
            if self.integrated_buffer:
                state = self.integrated_buffer[-1]
                print(f"\n📊 [GET STATE] Returning integrated state")
                print(f"   Buffer size: {len(self.integrated_buffer)}")
                print(f"   Has 'processed': {state.get('processed', False)}")
                print(f"   Has integrated_state: {'integrated_state' in state}")
            
                if state.get('processed', False) and 'integrated_state' in state:
                    integrated = state['integrated_state']
                    print(f"   State keys: {list(integrated.keys())}")
                    return state
                else:
                    print(f"❌ [GET STATE] State not properly processed")
                    return None
            else:
                print(f"❌ [GET STATE] No data in integrated buffer!")
            return None
    
    def get_ros2_node(self) -> Optional[Any]:
        """Get ROS2 node"""
        return self.ros2_node
    
    def send_valve_command(self, valve_id: int, command: float, priority: int = 0, emergency: bool = False) -> bool:
        """Send valve command with logging"""
        success = False
        
        if self.ros2_node:
            success = self.ros2_node.publish_valve_command(valve_id, command, priority, emergency)
        else:
            logger.warning("No ROS2 node for valve command")
        
        # Update statistics
        with self.lock:
            self.stats['valve_commands_sent'] += 1
        
        # Log command
        self.action_logger.info(
            f"Valve command - ID: {valve_id}, Command: {command:.3f}, "
            f"Priority: {priority}, Emergency: {emergency}, Success: {success}"
        )
        
        # Log to CSV
        with open(self.command_log_file, 'a') as f:
            f.write(f"{datetime.now().isoformat()},valve,{valve_id},{command},{priority},{emergency},{success}\n")
        
        return success
    
    def send_maintenance_request(self, sensor_id: int, priority: int, issue_type: str, description: str) -> bool:
        """Send maintenance request with logging"""
        success = False
        
        if self.ros2_node:
            success = self.ros2_node.publish_maintenance_request(sensor_id, priority, issue_type, description)
        else:
            logger.warning("No ROS2 node for maintenance request")
        
        # Update statistics
        with self.lock:
            self.stats['maintenance_requests_sent'] += 1
        
        # Log request
        self.action_logger.info(
            f"Maintenance request - Sensor: {sensor_id}, Priority: {priority}, "
            f"Issue: {issue_type}, Success: {success}"
        )
        
        # Log to CSV
        with open(self.command_log_file, 'a') as f:
            f.write(f"{datetime.now().isoformat()},maintenance,{sensor_id},{priority},,,{success}\n")
        
        return success
    
    def send_emergency_alert(self, leak_probability: float, location: Tuple[float, float], 
                           severity: int, confidence: float, action: str) -> bool:
        """Send emergency alert with logging"""
        success = False
        
        if self.ros2_node:
            x, y = location
            success = self.ros2_node.publish_emergency_alert(leak_probability, x, y, severity, confidence, action)
        else:
            logger.warning("No ROS2 node for emergency alert")
        
        # Update statistics
        with self.lock:
            self.stats['emergency_alerts_sent'] += 1
        
        # Log alert
        self.action_logger.critical(
            f"Emergency alert - Leak prob: {leak_probability:.2%}, "
            f"Location: ({location[0]:.2f}, {location[1]:.2f}), "
            f"Severity: {severity}, Action: {action}, Success: {success}"
        )
        
        # Log to CSV
        with open(self.command_log_file, 'a') as f:
            f.write(f"{datetime.now().isoformat()},emergency,0,{leak_probability},{severity},,{success}\n")
        
        return success
    
    def get_statistics(self) -> Dict:
        """Get statistics"""
        with self.lock:
            stats = self.stats.copy()
            stats['buffer_sizes'] = {
                'ros2': len(self.ros2_buffer),
                'udp': len(self.udp_buffer),
                'integrated': len(self.integrated_buffer)
            }
            stats['last_update'] = stats['last_update'].isoformat() if isinstance(stats['last_update'], datetime) else str(stats['last_update'])
        return stats
    
    def stop(self):
        """Stop collector"""
        self.running = False
        if self.ros2_executor:
            self.ros2_executor.shutdown()

# ============================================
# Enhanced Neural Network Architectures
# ============================================

class TransformerEncoderLayer(nn.Module):
    """Transformer encoder layer for sequential data"""
    
    def __init__(self, d_model: int, nhead: int, dropout: float = 0.1):
        super().__init__()
        self.self_attn = nn.MultiheadAttention(d_model, nhead, dropout=dropout)
        self.linear1 = nn.Linear(d_model, d_model * 4)
        self.dropout = nn.Dropout(dropout)
        self.linear2 = nn.Linear(d_model * 4, d_model)
        self.norm1 = nn.LayerNorm(d_model)
        self.norm2 = nn.LayerNorm(d_model)
        self.dropout1 = nn.Dropout(dropout)
        self.dropout2 = nn.Dropout(dropout)
        self.activation = nn.GELU()
        
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        # Self attention
        attn_output, _ = self.self_attn(x, x, x)
        x = x + self.dropout1(attn_output)
        x = self.norm1(x)
        
        # Feed forward
        ff_output = self.linear2(self.dropout(self.activation(self.linear1(x))))
        x = x + self.dropout2(ff_output)
        x = self.norm2(x)
        
        return x

class EnhancedLeakPredictor(nn.Module):
    """Enhanced leak predictor for industrial data"""
    
    def __init__(self, input_dim: int, config: IndustrialConfig):
        super().__init__()
        self.config = config
        
        # Feature embedding
        self.input_projection = nn.Linear(input_dim, 512)
        
        # Transformer layers
        self.transformer_layers = nn.ModuleList([
            TransformerEncoderLayer(512, config.ATTENTION_HEADS, config.DROPOUT_RATE)
            for _ in range(config.TRANSFORMER_LAYERS)
        ])
        
        # Temporal convolution for time series
        self.temporal_conv = nn.Conv1d(512, 512, kernel_size=3, padding=1)
        
        # Prediction heads
        self.leak_head = nn.Sequential(
            nn.Linear(512, 256),
            nn.LayerNorm(256),
            nn.GELU(),
            nn.Dropout(config.DROPOUT_RATE),
            nn.Linear(256, 128),
            nn.GELU(),
            nn.Linear(128, 1),
            nn.Sigmoid()
        )
        
        self.location_head = nn.Sequential(
            nn.Linear(512, 256),
            nn.LayerNorm(256),
            nn.GELU(),
            nn.Dropout(config.DROPOUT_RATE),
            nn.Linear(256, 2)  # x, y coordinates
        )
        
        self.severity_head = nn.Sequential(
            nn.Linear(512, 256),
            nn.LayerNorm(256),
            nn.GELU(),
            nn.Dropout(config.DROPOUT_RATE),
            nn.Linear(256, 128),
            nn.GELU(),
            nn.Linear(128, 3),  # Low, Medium, High
            nn.Softmax(dim=-1)
        )
        
        self.zone_attention = nn.MultiheadAttention(512, 8, dropout=config.DROPOUT_RATE)
        
    def forward(self, x: torch.Tensor, zone_features: Optional[torch.Tensor] = None) -> Dict[str, torch.Tensor]:
        # Input projection
        x = self.input_projection(x)
        x = x.unsqueeze(0)  # Add batch dimension
        
        # Transformer layers
        for layer in self.transformer_layers:
            x = layer(x)
        
        # Global average pooling
        x = x.mean(dim=0)
        
        # Zone attention if zone features provided
        if zone_features is not None:
            zone_features = zone_features.unsqueeze(0)
            attn_output, _ = self.zone_attention(x.unsqueeze(0), zone_features, zone_features)
            x = x + attn_output.squeeze(0)
        
        # Predictions
        leak_prob = self.leak_head(x)
        location = self.location_head(x)
        severity = self.severity_head(x)
        
        return {
            'leak_probability': leak_prob,
            'location': location,
            'severity': severity,
            'features': x
        }

# ============================================
# Enhanced RL Environment with Reward Logging
# ============================================

class EnhancedIndustrialPipeEnv(gym.Env):
    """Enhanced industrial pipe environment with comprehensive logging"""
    
    def __init__(self, config: IndustrialConfig, data_collector: EnhancedIndustrialDataCollector):
        super().__init__()
        
        self.config = config
        self.data_collector = data_collector
        
        # State tracking
        self.state_history = deque(maxlen=config.HISTORY_LENGTH)
        self.action_history = deque(maxlen=100)
        
        # Enhanced leak predictor
        self.leak_predictor = EnhancedLeakPredictor(
            input_dim=512,
            config=config
        ).to(config.DEVICE)
        
        # Loggers
        self.reward_logger = loggers['reward_logger']
        self.performance_logger = loggers['performance_logger']
        
        # Reward logging
        self.reward_log_file = directories['reward_logs'] / "reward_details.csv"
        self._init_reward_logging()
        
        # Load pre-trained model if exists
        self._load_leak_predictor()
        
        # Define action space (valves + maintenance + system)
        self.action_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(45,),  # 20 valves + 20 maintenance + 5 system
            dtype=np.float32
        )
        
        # Define observation space
        obs_dim = self._calculate_observation_dim()
        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(obs_dim,),
            dtype=np.float32
        )
        
        # Reward tracking
        self.reward_components = defaultdict(float)
        self.episode_step = 0
        self.episode_reward = 0.0
        self.total_rewards = []  # Track rewards for logging
        self.episode_start_time = None
        
        # Performance metrics
        self.metrics = {
            'leaks_prevented': 0,
            'valve_changes': 0,
            'maintenance_actions': 0,
            'pressure_violations': 0,
            'efficiency_score': 0.0,
            'response_time': 0.0,
            'episode_duration': 0.0
        }
        
        logger.info(f"Enhanced Environment initialized with obs dim: {obs_dim}")
    
    def _init_reward_logging(self):
        """Initialize reward logging"""
        if not self.reward_log_file.exists():
            with open(self.reward_log_file, 'w') as f:
                f.write("timestamp,episode,step,total_reward,pressure,efficiency,flow_stability,"
                       "reservoir,valve_cost,maintenance_cost,success_rate,leak_probability\n")
    
    def _calculate_observation_dim(self) -> int:
        """Calculate observation dimension"""
        # Basic features from UDP: system(17) + zones(180) + valves(100) + volumes(50) + hourly_usage(24) + action_times(20)
        basic_features = 17 + 180 + 100 + 50 + 24 + 20  # 391 features
        
        # Historical features
        historical_features = basic_features * self.config.HISTORY_LENGTH
        
        # Leak predictions
        leak_features = 6  # prob + location(2) + severity(3)
        
        return basic_features + historical_features + leak_features
    
    def reset(self, seed: Optional[int] = None, options: Optional[Dict] = None):
        """Reset environment"""
        super().reset(seed=seed)
        
        # Log episode end if this is not the first episode
        if self.episode_step > 0:
            self._log_episode_summary()
        
        # Clear history
        self.state_history.clear()
        self.action_history.clear()
        
        # Reset metrics
        self.episode_step = 0
        self.episode_reward = 0.0
        self.reward_components.clear()
        self.metrics = {k: 0 for k in self.metrics.keys()}
        self.total_rewards = []
        self.episode_start_time = datetime.now()
        
        # Get initial observation
        initial_obs = self._get_observation_from_data()
        
        # Fill history
        for _ in range(self.config.HISTORY_LENGTH):
            self.state_history.append(initial_obs[:391])  # Basic features
        
        self.reward_logger.info(f"Episode started at {self.episode_start_time}")
        
        return initial_obs, {}
    
    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        """Take action"""
        self.episode_step += 1
        
        # Execute action
        executed_actions = self._execute_action(action)
        
        # Get new state
        new_obs = self._get_observation_from_data()
        
        # Calculate reward
        reward, reward_info = self._calculate_reward(executed_actions)
        self.episode_reward += reward
        self.total_rewards.append(reward)
        
        # Check termination
        terminated = self.episode_step >= self.config.MAX_EPISODE_STEPS
        truncated = False
        
        # Update metrics
        info = {
            'episode_step': self.episode_step,
            'episode_reward': self.episode_reward,
            'reward_components': reward_info,
            'metrics': self.metrics.copy(),
            'actions_executed': executed_actions
        }
        
        # Predict leaks
        leak_predictions = self._predict_leaks(new_obs)
        info['leak_predictions'] = leak_predictions
        
        # Log reward details
        self._log_reward_details(reward, reward_info, leak_predictions)
        
        # Emergency check
        if leak_predictions['probability'] > self.config.EMERGENCY_SHUTDOWN_THRESHOLD:
            self._emergency_protocol()
            truncated = True
        
        # Update episode duration
        if self.episode_start_time:
            self.metrics['episode_duration'] = (datetime.now() - self.episode_start_time).total_seconds()
        
        return new_obs, reward, terminated, truncated, info
    
    def _log_episode_summary(self):
        """Log episode summary"""
        if self.episode_step > 0:
            episode_summary = {
                'episode_steps': self.episode_step,
                'total_reward': self.episode_reward,
                'average_reward': np.mean(self.total_rewards) if self.total_rewards else 0.0,
                'max_reward': np.max(self.total_rewards) if self.total_rewards else 0.0,
                'min_reward': np.min(self.total_rewards) if self.total_rewards else 0.0,
                'metrics': self.metrics.copy(),
                'timestamp': datetime.now().isoformat()
            }
            
            # Log to file
            summary_file = directories['reward_logs'] / f"episode_{int(time.time())}.json"
            with open(summary_file, 'w') as f:
                json.dump(episode_summary, f, indent=2)
            
            self.reward_logger.info(
                f"Episode completed - Steps: {self.episode_step}, "
                f"Total Reward: {self.episode_reward:.2f}, "
                f"Avg Reward: {episode_summary['average_reward']:.2f}"
            )
    
    def _log_reward_details(self, reward: float, reward_info: Dict, leak_predictions: Dict):
        """Log detailed reward information"""
        # Log to CSV
        with open(self.reward_log_file, 'a') as f:
            f.write(f"{datetime.now().isoformat()},"
                   f"{getattr(self, 'episode_count', 0)},"
                   f"{self.episode_step},"
                   f"{reward:.4f},"
                   f"{reward_info.get('pressure', 0):.4f},"
                   f"{reward_info.get('efficiency', 0):.4f},"
                   f"{reward_info.get('flow_stability', 0):.4f},"
                   f"{reward_info.get('reservoir', 0):.4f},"
                   f"{reward_info.get('valve_cost', 0):.4f},"
                   f"{reward_info.get('maintenance_cost', 0):.4f},"
                   f"{reward_info.get('success_rate', 0):.4f},"
                   f"{leak_predictions.get('probability', 0):.4f}\n")
        
        # Log to reward logger
        self.reward_logger.debug(
            f"Step {self.episode_step}: Total={reward:.3f}, "
            f"Pressure={reward_info.get('pressure', 0):.3f}, "
            f"Efficiency={reward_info.get('efficiency', 0):.3f}"
        )
    
    def _get_observation_from_data(self) -> np.ndarray:
        """Get observation from integrated data"""
        latest_state = self.data_collector.get_latest_integrated_state()
        
        if latest_state and latest_state.get('processed', False):
            integrated_state = latest_state['integrated_state']
            
            # Extract features in order
            feature_vectors = []
            
            # System features
            if 'system' in integrated_state:
                feature_vectors.append(integrated_state['system'])
            else:
                feature_vectors.append(np.zeros(17, dtype=np.float32))
            
            # Zone features
            if 'zones' in integrated_state:
                feature_vectors.append(integrated_state['zones'])
            else:
                feature_vectors.append(np.zeros(180, dtype=np.float32))
            
            # Valve states
            if 'valves' in integrated_state:
                feature_vectors.append(integrated_state['valves'])
            else:
                feature_vectors.append(np.zeros(100, dtype=np.float32))
            
            # Volumes
            if 'volumes' in integrated_state:
                feature_vectors.append(integrated_state['volumes'])
            else:
                feature_vectors.append(np.zeros(50, dtype=np.float32))
            
            # Hourly usage
            if 'hourly_usage' in integrated_state:
                feature_vectors.append(integrated_state['hourly_usage'])
            else:
                feature_vectors.append(np.zeros(24, dtype=np.float32))
            
            # Action times
            if 'action_times' in integrated_state:
                feature_vectors.append(integrated_state['action_times'])
            else:
                feature_vectors.append(np.zeros(20, dtype=np.float32))
            
            # Combine
            basic_features = np.concatenate(feature_vectors)
            
        else:
            # Synthetic data
            basic_features = np.random.randn(391).astype(np.float32) * 0.1
        
        # Historical features
        historical_features = []
        for i in range(-self.config.HISTORY_LENGTH, 0):
            idx = max(0, len(self.state_history) + i)
            if idx < len(self.state_history):
                historical_features.append(self.state_history[idx])
            else:
                historical_features.append(np.zeros_like(basic_features))
        
        historical_features = np.concatenate(historical_features)
        
        # Leak predictions
        leak_features = np.random.rand(6).astype(np.float32) * 0.1
        
        # Combine
        full_obs = np.concatenate([basic_features, historical_features, leak_features])
        
        # Update history
        self.state_history.append(basic_features)
        
        return full_obs.astype(np.float32)
    
    def _execute_action(self, action: np.ndarray) -> Dict:
        """Execute action through ROS2"""
        executed = {
            'valve_changes': [],
            'maintenance_requests': [],
            'system_controls': [],
            'success': True
        }
        
        success_count = 0
        total_actions = 0
        
        # Valve actions (first 20)
        valve_actions = action[:20]
        for i, valve_action in enumerate(valve_actions):
            if abs(valve_action) > self.config.VALVE_ACTION_THRESHOLD:
                total_actions += 1
                
                command = np.clip(valve_action, -1.0, 1.0)
                
                success = self.data_collector.send_valve_command(
                    valve_id=i,
                    command=command,
                    priority=1 if abs(command) > 0.5 else 0,
                    emergency=False
                )
                
                if success:
                    success_count += 1
                    executed['valve_changes'].append({
                        'valve_id': i,
                        'action': command,
                        'success': True
                    })
                    self.metrics['valve_changes'] += 1
                else:
                    executed['valve_changes'].append({
                        'valve_id': i,
                        'action': command,
                        'success': False
                    })
        
        # Maintenance actions (next 20)
        maintenance_actions = action[20:40]
        for i, maint_action in enumerate(maintenance_actions):
            if maint_action > self.config.MAINTENANCE_ACTION_THRESHOLD:
                total_actions += 1
                
                if maint_action > 0.9:
                    priority = 3
                    issue = "CRITICAL_FAILURE"
                elif maint_action > 0.7:
                    priority = 2
                    issue = "SENSOR_DRIFT"
                else:
                    priority = 1
                    issue = "ROUTINE_MAINTENANCE"
                
                success = self.data_collector.send_maintenance_request(
                    sensor_id=i,
                    priority=priority,
                    issue_type=issue,
                    description=f"RL agent maintenance request"
                )
                
                if success:
                    success_count += 1
                    executed['maintenance_requests'].append({
                        'sensor_id': i,
                        'priority': priority,
                        'success': True
                    })
                    self.metrics['maintenance_actions'] += 1
                else:
                    executed['maintenance_requests'].append({
                        'sensor_id': i,
                        'priority': priority,
                        'success': False
                    })
        
        # System controls
        system_controls = action[40:45]
        executed['system_controls'] = system_controls.tolist()
        
        # Success rate
        if total_actions > 0:
            executed['success_rate'] = success_count / total_actions
        else:
            executed['success_rate'] = 1.0
        
        self.action_history.append(executed.copy())
        
        return executed
    
    def _calculate_reward(self, executed_actions: Dict) -> Tuple[float, Dict]:
        """Calculate reward from UDP data"""
        reward_components = {}
        total_reward = 0.0
        
        latest_state = self.data_collector.get_latest_integrated_state()
        
        if latest_state and latest_state.get('processed', False):
            state = latest_state['integrated_state']
            
            if 'system' in state:
                system = state['system']
                
                # Pressure compliance reward
                pressure_reward = system[4] * self.config.REWARD_WEIGHTS['pressure_compliance']
                reward_components['pressure'] = pressure_reward
                total_reward += pressure_reward
                
                # Efficiency reward
                efficiency_reward = system[2] * self.config.REWARD_WEIGHTS['supply_efficiency']
                reward_components['efficiency'] = efficiency_reward
                total_reward += efficiency_reward
                
                # Flow stability reward
                flow_variance = system[12]
                flow_stability_reward = (1.0 - min(flow_variance, 1.0)) * self.config.REWARD_WEIGHTS['flow_stability']
                reward_components['flow_stability'] = flow_stability_reward
                total_reward += flow_stability_reward
                
                # Reservoir stability reward
                reservoir_level = system[1]
                reservoir_reward = reservoir_level * self.config.REWARD_WEIGHTS['reservoir_stability']
                reward_components['reservoir'] = reservoir_reward
                total_reward += reservoir_reward
            
            # Valve operation cost
            valve_cost = len(executed_actions['valve_changes']) * self.config.REWARD_WEIGHTS['valve_operation_cost']
            reward_components['valve_cost'] = valve_cost
            total_reward += valve_cost
            
            # Maintenance cost
            maint_cost = len(executed_actions['maintenance_requests']) * self.config.REWARD_WEIGHTS['maintenance_cost']
            reward_components['maintenance_cost'] = maint_cost
            total_reward += maint_cost
            
            # Success rate bonus
            success_rate = executed_actions.get('success_rate', 0.0)
            success_bonus = success_rate * self.config.REWARD_WEIGHTS['response_time']
            reward_components['success_rate'] = success_bonus
            total_reward += success_bonus
            
        else:
            # Synthetic rewards
            reward_components = {
                'pressure': 0.8 * self.config.REWARD_WEIGHTS['pressure_compliance'],
                'efficiency': 0.85 * self.config.REWARD_WEIGHTS['supply_efficiency'],
                'flow_stability': 0.7 * self.config.REWARD_WEIGHTS['flow_stability'],
                'reservoir': 0.6 * self.config.REWARD_WEIGHTS['reservoir_stability'],
                'valve_cost': len(executed_actions['valve_changes']) * self.config.REWARD_WEIGHTS['valve_operation_cost'],
                'maintenance_cost': len(executed_actions['maintenance_requests']) * self.config.REWARD_WEIGHTS['maintenance_cost'],
                'success_rate': executed_actions.get('success_rate', 0.0) * self.config.REWARD_WEIGHTS['response_time']
            }
            total_reward = sum(reward_components.values())
        
        # Store components
        for key, value in reward_components.items():
            self.reward_components[key] += value
        
        return total_reward, reward_components
    
    def _predict_leaks(self, observation: np.ndarray) -> Dict:
        """Predict leaks"""
        try:
            obs_tensor = torch.FloatTensor(observation[:512]).unsqueeze(0).to(self.config.DEVICE)
            
            with torch.no_grad():
                predictions = self.leak_predictor(obs_tensor)
            
            leak_predictions = {
                'probability': predictions['leak_probability'].cpu().numpy()[0][0],
                'location': predictions['location'].cpu().numpy()[0],
                'severity': predictions['severity'].cpu().numpy()[0]
            }
            
            # Alert if high probability
            if leak_predictions['probability'] > self.config.ALERT_THRESHOLDS['leak_probability']:
                self.data_collector.send_emergency_alert(
                    leak_probability=leak_predictions['probability'],
                    location=tuple(leak_predictions['location']),
                    severity=np.argmax(leak_predictions['severity']),
                    confidence=0.8,
                    action="INSPECT_AND_ISOLATE"
                )
            
        except Exception as e:
            logger.error(f"Leak prediction error: {e}")
            leak_predictions = {
                'probability': 0.0,
                'location': [0.0, 0.0],
                'severity': [1.0, 0.0, 0.0]
            }
        
        return leak_predictions
    
    def _emergency_protocol(self):
        """Emergency shutdown"""
        logger.critical("EMERGENCY PROTOCOL")
        
        for i in range(20):
            self.data_collector.send_valve_command(
                valve_id=i,
                command=-1.0,
                priority=3,
                emergency=True
            )
        
        self.data_collector.send_emergency_alert(
            leak_probability=1.0,
            location=(0.0, 0.0),
            severity=3,
            confidence=1.0,
            action="SYSTEM_SHUTDOWN"
        )
    
    def _load_leak_predictor(self):
        """Load leak predictor"""
        model_path = f"{self.config.MODEL_SAVE_PATH}/leak_predictor.pth"
        if os.path.exists(model_path):
            try:
                self.leak_predictor.load_state_dict(torch.load(model_path, map_location=self.config.DEVICE))
                self.leak_predictor.eval()
                logger.info(f"Loaded leak predictor from {model_path}")
            except Exception as e:
                logger.warning(f"Failed to load leak predictor: {e}")

# ============================================
# Enhanced REST API Server with Logging
# ============================================

class IndustrialRLAPI:
    """REST API for RL control with comprehensive logging"""
    
    def __init__(self, config: IndustrialConfig, rl_agent: 'EnhancedIndustrialRLAgent'):
        self.config = config
        self.rl_agent = rl_agent
        self.app = Flask(__name__)
        CORS(self.app)
        
        self.restapi_logger = loggers['restapi_logger']
        self.request_history = deque(maxlen=1000)
        
        # API logging
        self.api_log_file = directories['restapi_logs'] / "api_requests.csv"
        self._init_api_logging()
        
        self.setup_routes()
        
        logger.info(f"REST API configured on {config.REST_API_HOST}:{config.REST_API_PORT}")
    
    def _init_api_logging(self):
        """Initialize API logging"""
        if not self.api_log_file.exists():
            with open(self.api_log_file, 'w') as f:
                f.write("timestamp,endpoint,method,status_code,client_ip,"
                       "unique_id,parameters,response_time_ms\n")
    
    def setup_routes(self):
        """Setup API routes"""
        
        @self.app.before_request
        def before_request():
            """Log before request"""
            request.start_time = time.time()
        
        @self.app.after_request
        def after_request(response):
            """Log after request"""
            # Calculate response time
            response_time = (time.time() - request.start_time) * 1000
            
            # Log to CSV
            with open(self.api_log_file, 'a') as f:
                f.write(f"{datetime.now().isoformat()},"
                       f"{request.path},"
                       f"{request.method},"
                       f"{response.status_code},"
                       f"{request.remote_addr},"
                       f"{request.json.get('unique_id', '') if request.is_json else ''},"
                       f"{json.dumps(request.json) if request.is_json else ''},"
                       f"{response_time:.2f}\n")
            
            # Log to restapi logger
            self.restapi_logger.info(
                f"{request.method} {request.path} - "
                f"Status: {response.status_code}, "
                f"Time: {response_time:.2f}ms, "
                f"IP: {request.remote_addr}"
            )
            
            return response
        
        @self.app.route('/rl_control', methods=['POST'])
        def rl_control():
            """Main RL control endpoint"""
            return self.handle_rl_control()
        
        @self.app.route('/status', methods=['GET'])
        def status():
            """System status"""
            return self.get_status()
        
        @self.app.route('/valve_control', methods=['POST'])
        def valve_control():
            """Direct valve control"""
            return self.handle_valve_control()
        
        @self.app.route('/maintenance', methods=['POST'])
        def maintenance():
            """Maintenance requests"""
            return self.handle_maintenance()
        
        @self.app.route('/emergency', methods=['POST'])
        def emergency():
            """Emergency controls"""
            return self.handle_emergency()
        
        @self.app.route('/metrics', methods=['GET'])
        def metrics():
            """Performance metrics"""
            return self.get_metrics()
        
        @self.app.route('/logs', methods=['GET'])
        def get_logs():
            """Get recent logs"""
            return self.get_recent_logs()
        
        @self.app.route('/health', methods=['GET'])
        def health():
            """Health check"""
            return jsonify({
                'status': 'healthy',
                'timestamp': datetime.now().isoformat(),
                'service': 'industrial_pipe_rl_api',
                'version': '2.0.0',
                'persistence': 'enabled'
            })
        
        @self.app.route('/udp_status', methods=['GET'])
        def udp_status():
            """UDP status"""
            return self.get_udp_status()
        
        @self.app.route('/system_state', methods=['GET'])
        def system_state():
            """Get system state"""
            return self.get_system_state()
    
    def handle_rl_control(self):
        """Handle RL control requests"""
        try:
            data = request.get_json()
            
            if not data:
                return jsonify({'error': 'No JSON data'}), 400
            
            unique_id = data.get('unique_id')
            if not unique_id or unique_id not in self.config.REST_ALLOWED_IDS:
                return jsonify({'error': 'Invalid unique_id'}), 401
            
            sensor_id = data.get('sensor_id')
            if sensor_id is None:
                return jsonify({'error': 'sensor_id required'}), 400
            
            valve_cmd = data.get('valve')
            maintenance_cmd = data.get('maintenance')
            
            # Track request
            self.request_history.append({
                'timestamp': datetime.now().isoformat(),
                'unique_id': unique_id,
                'sensor_id': sensor_id,
                'valve_cmd': valve_cmd,
                'maintenance_cmd': maintenance_cmd,
                'ip': request.remote_addr
            })
            
            results = {
                'sensor_id': sensor_id,
                'timestamp': datetime.now().isoformat(),
                'actions_executed': [],
                'status': 'success'
            }
            
            # Valve control
            if valve_cmd is not None:
                command_value = -1.0 if valve_cmd == 0 else 1.0
                success = self.rl_agent.data_collector.send_valve_command(
                    valve_id=sensor_id,
                    command=command_value,
                    priority=2,
                    emergency=False
                )
                results['actions_executed'].append({
                    'type': 'valve',
                    'command': 'close' if valve_cmd == 0 else 'open',
                    'success': success,
                    'message': f"Valve {'closed' if valve_cmd == 0 else 'opened'}"
                })
            
            # Maintenance request
            if maintenance_cmd is not None and maintenance_cmd == 1:
                success = self.rl_agent.data_collector.send_maintenance_request(
                    sensor_id=sensor_id,
                    priority=2,
                    issue_type="API_REQUESTED_MAINTENANCE",
                    description=f"Maintenance requested via REST API"
                )
                results['actions_executed'].append({
                    'type': 'maintenance',
                    'command': 'request',
                    'success': success,
                    'message': "Maintenance request sent"
                })
            
            return jsonify(results), 200
            
        except Exception as e:
            self.restapi_logger.error(f"API error: {e}")
            return jsonify({'error': str(e)}), 500
    
    def handle_valve_control(self):
        """Direct valve control"""
        try:
            data = request.get_json()
            
            if not data:
                return jsonify({'error': 'No data'}), 400
            
            required = ['unique_id', 'valve_id', 'command']
            for field in required:
                if field not in data:
                    return jsonify({'error': f'Missing {field}'}), 400
            
            if data['unique_id'] not in self.config.REST_ALLOWED_IDS:
                return jsonify({'error': 'Invalid unique_id'}), 401
            
            success = self.rl_agent.data_collector.send_valve_command(
                valve_id=data['valve_id'],
                command=float(data['command']),
                priority=data.get('priority', 1),
                emergency=data.get('emergency', False)
            )
            
            return jsonify({
                'valve_id': data['valve_id'],
                'command': data['command'],
                'success': success,
                'timestamp': datetime.now().isoformat()
            }), 200 if success else 500
            
        except Exception as e:
            self.restapi_logger.error(f"Valve control error: {e}")
            return jsonify({'error': str(e)}), 500
    
    def handle_maintenance(self):
        """Maintenance request"""
        try:
            data = request.get_json()
            
            if not data:
                return jsonify({'error': 'No data'}), 400
            
            required = ['unique_id', 'sensor_id', 'issue_type']
            for field in required:
                if field not in data:
                    return jsonify({'error': f'Missing {field}'}), 400
            
            if data['unique_id'] not in self.config.REST_ALLOWED_IDS:
                return jsonify({'error': 'Invalid unique_id'}), 401
            
            success = self.rl_agent.data_collector.send_maintenance_request(
                sensor_id=data['sensor_id'],
                priority=data.get('priority', 1),
                issue_type=data['issue_type'],
                description=data.get('description', 'API maintenance request')
            )
            
            return jsonify({
                'sensor_id': data['sensor_id'],
                'issue_type': data['issue_type'],
                'success': success,
                'timestamp': datetime.now().isoformat()
            }), 200 if success else 500
            
        except Exception as e:
            self.restapi_logger.error(f"Maintenance error: {e}")
            return jsonify({'error': str(e)}), 500
    
    def handle_emergency(self):
        """Emergency control"""
        try:
            data = request.get_json()
            
            if not data:
                return jsonify({'error': 'No data'}), 400
            
            required = ['unique_id', 'action']
            for field in required:
                if field not in data:
                    return jsonify({'error': f'Missing {field}'}), 400
            
            if data['unique_id'] not in self.config.REST_ALLOWED_IDS:
                return jsonify({'error': 'Invalid unique_id'}), 401
            
            action = data['action']
            
            if action == 'shutdown':
                self.rl_agent._emergency_shutdown()
                message = "Emergency shutdown initiated"
            elif action == 'close_all_valves':
                for i in range(20):
                    self.rl_agent.data_collector.send_valve_command(
                        valve_id=i,
                        command=-1.0,
                        priority=3,
                        emergency=True
                    )
                message = "All valves closed"
            else:
                return jsonify({'error': 'Unknown action'}), 400
            
            return jsonify({
                'action': action,
                'success': True,
                'message': message,
                'timestamp': datetime.now().isoformat()
            }), 200
            
        except Exception as e:
            self.restapi_logger.error(f"Emergency error: {e}")
            return jsonify({'error': str(e)}), 500
    
    def get_status(self):
        """Get system status"""
        try:
            agent_status = self.rl_agent.get_status()
            
            return jsonify({
                'system': {
                    'status': 'online',
                    'timestamp': datetime.now().isoformat(),
                    'training': agent_status['training'],
                    'timestep': agent_status['timestep'],
                    'episode': agent_status.get('episode', 0),
                    'total_reward': agent_status.get('total_reward', 0.0)
                },
                'data_collection': agent_status['data_stats'],
                'performance': agent_status['performance'],
                'alerts': agent_status['alerts'],
                'api': {
                    'requests_processed': len(self.request_history),
                    'active_connections': threading.active_count() - 1,
                    'last_request': self.request_history[-1]['timestamp'] if self.request_history else None
                },
                'persistence': {
                    'last_save': agent_status.get('last_save_time'),
                    'model_loaded': agent_status.get('model_loaded', False),
                    'buffer_loaded': agent_status.get('buffer_loaded', False)
                }
            }), 200
            
        except Exception as e:
            self.restapi_logger.error(f"Status error: {e}")
            return jsonify({'error': str(e)}), 500
    
    def get_metrics(self):
        """Get performance metrics"""
        try:
            agent_status = self.rl_agent.get_status()
            
            return jsonify({
                'timestamp': datetime.now().isoformat(),
                'rl_agent': agent_status['performance'],
                'api': {
                    'total_requests': len(self.request_history),
                    'successful_requests': sum(1 for r in self.request_history 
                                             if 'valve_cmd' in r or 'maintenance_cmd' in r),
                    'requests_last_hour': sum(1 for r in self.request_history 
                                             if datetime.fromisoformat(r['timestamp']) > 
                                             datetime.now() - timedelta(hours=1)),
                    'avg_response_time': agent_status.get('avg_response_time', 0.0)
                },
                'system': {
                    'cpu_percent': psutil.cpu_percent(),
                    'memory_percent': psutil.virtual_memory().percent,
                    'active_threads': threading.active_count(),
                    'disk_usage': psutil.disk_usage('/').percent
                }
            }), 200
            
        except Exception as e:
            self.restapi_logger.error(f"Metrics error: {e}")
            return jsonify({'error': str(e)}), 500
    
    def get_udp_status(self):
        """Get UDP status"""
        try:
            udp_stats = self.rl_agent.data_collector.processor.udp_handler.get_statistics()
            
            return jsonify({
                'timestamp': datetime.now().isoformat(),
                'udp_status': udp_stats,
                'udp_enabled': self.config.UDP_ENABLED,
                'udp_host': self.config.UDP_HOST,
                'udp_port': self.config.UDP_PORT,
                'log_directory': str(directories['udp_logs'])
            }), 200
            
        except Exception as e:
            self.restapi_logger.error(f"UDP status error: {e}")
            return jsonify({'error': str(e)}), 500
    
    def get_system_state(self):
        """Get complete system state"""
        try:
            state = self.rl_agent.persistence_manager.load_system_state()
            if state:
                return jsonify(state), 200
            else:
                return jsonify({'message': 'No saved state found'}), 404
        except Exception as e:
            self.restapi_logger.error(f"System state error: {e}")
            return jsonify({'error': str(e)}), 500
    
    def get_recent_logs(self):
        """Get recent logs"""
        try:
            limit = request.args.get('limit', 100, type=int)
            log_type = request.args.get('type', 'main')
            
            log_files = {
                'main': directories['logs'] / 'main.log',
                'ros2': directories['ros2_logs'] / 'ros2_messages.log',
                'udp': directories['udp_logs'] / 'udp_messages.log',
                'api': directories['restapi_logs'] / 'api_messages.log'
            }
            
            if log_type not in log_files:
                return jsonify({'error': 'Invalid log type'}), 400
            
            log_file = log_files[log_type]
            if log_file.exists():
                with open(log_file, 'r') as f:
                    lines = f.readlines()[-limit:]
                return jsonify({
                    'log_type': log_type,
                    'lines': lines,
                    'total_lines': len(lines)
                }), 200
            else:
                return jsonify({'error': 'Log file not found'}), 404
                
        except Exception as e:
            self.restapi_logger.error(f"Log retrieval error: {e}")
            return jsonify({'error': str(e)}), 500
    
    def run(self):
        """Run API server"""
        logger.info(f"Starting REST API on {self.config.REST_API_HOST}:{self.config.REST_API_PORT}")
        
        def run_server():
            self.app.run(
                host=self.config.REST_API_HOST,
                port=self.config.REST_API_PORT,
                debug=self.config.REST_API_DEBUG,
                threaded=True,
                use_reloader=False
            )
        
        api_thread = threading.Thread(target=run_server, daemon=True)
        api_thread.start()
        
        time.sleep(2)
        
        # Test connection
        try:
            response = requests.get(f"http://{self.config.REST_API_HOST}:{self.config.REST_API_PORT}/health", timeout=2)
            if response.status_code == 200:
                self.restapi_logger.info("REST API started successfully")
            else:
                self.restapi_logger.warning(f"REST API health check failed: {response.status_code}")
        except:
            self.restapi_logger.warning("REST API may not be accessible")
    
    def stop(self):
        """Stop API server"""
        logger.info("Stopping REST API")

# ============================================
# Enhanced RL Agent with Persistence
# ============================================

class EnhancedIndustrialRLAgent:
    """Enhanced industrial RL agent with persistence"""
    
    def __init__(self, config: IndustrialConfig):
        self.config = config
        
        # Initialize persistence manager
        self.persistence_manager = PersistenceManager(config)
        
        # Load previous state if exists
        self.previous_state = self.persistence_manager.load_system_state()
        self.episode_count = self.previous_state.get('agent', {}).get('episode_count', 0) if self.previous_state else 0
        self.total_reward = self.previous_state.get('agent', {}).get('total_reward', 0.0) if self.previous_state else 0.0
        
        # Initialize components
        self.data_collector = EnhancedIndustrialDataCollector(config)
        time.sleep(2)
        
        # Create environment
        self.env = EnhancedIndustrialPipeEnv(config, self.data_collector)
        
        # Initialize RL model
        self.model = self._create_rl_model()
        
        # Load previous model if exists
        self.model_loaded = False
        self.buffer_loaded = False
        self._load_previous_model()
        
        # Initialize REST API
        self.rest_api = IndustrialRLAPI(config, self)
        
        # Training state
        self.is_training = False
        self.current_timestep = 0
        self.last_save_time = None
        
        # Performance monitoring
        self.performance_monitor = PerformanceMonitor()
        
        # Alert system
        self.alert_system = AlertSystem(config)
        
        # Start REST API
        if self.config.REST_API_ENABLED:
            self.rest_api.run()
        
        logger.info(f"Enhanced Industrial RL Agent initialized (Episode: {self.episode_count}, Total Reward: {self.total_reward:.2f})")
    
    def _create_rl_model(self):
        """Create RL model with custom buffer management"""
        vec_env = DummyVecEnv([lambda: self.env])
        vec_env = VecMonitor(vec_env, self.config.LOG_PATH)
    
        obs_dim = self.env.observation_space.shape[0]
        logger.info(f"Observation dimension: {obs_dim}")
    
        # Calculate memory needed for standard buffer
        buffer_memory_gb = (obs_dim * self.config.BUFFER_SIZE * 4 * 6) / (1024**3)
        logger.info(f"Buffer memory needed: {buffer_memory_gb:.2f} GB")
    
        if buffer_memory_gb > 1.0:  # If > 1GB, use PPO
            logger.warning(f"Buffer too large ({buffer_memory_gb:.2f}GB), forcing PPO")
            return self._create_ppo_model(vec_env)
    
        # Otherwise use SAC with adjusted buffer
        effective_buffer = min(self.config.BUFFER_SIZE, 50000)
    
        if self.config.RL_ALGORITHM == "SAC":
            model = SAC(
                "MlpPolicy",
                vec_env,
                learning_rate=self.config.LEARNING_RATE,
                buffer_size=effective_buffer,
                learning_starts=min(self.config.LEARNING_STARTS, 5000),
                batch_size=min(self.config.BATCH_SIZE, 256),
                tau=self.config.TAU,
                gamma=self.config.GAMMA,
                train_freq=self.config.TRAIN_FREQUENCY,
                gradient_steps=1,
                policy_kwargs=dict(
                    net_arch=dict(pi=[256, 128], qf=[256, 128]),
                    activation_fn=nn.GELU
                ),
                tensorboard_log=self.config.LOG_PATH,
                verbose=1,
                device=self.config.DEVICE
            )
        elif self.config.RL_ALGORITHM == "TQC":
            model = TQC(
                "MlpPolicy",
                vec_env,
                learning_rate=self.config.LEARNING_RATE,
                buffer_size=effective_buffer,
                learning_starts=min(self.config.LEARNING_STARTS, 5000),
                batch_size=min(self.config.BATCH_SIZE, 256),
                tau=self.config.TAU,
                gamma=self.config.GAMMA,
                train_freq=self.config.TRAIN_FREQUENCY,
                gradient_steps=1,
                policy_kwargs=dict(
                    net_arch=dict(pi=[256, 128], qf=[256, 128]),
                    activation_fn=nn.GELU
                ),
                tensorboard_log=self.config.LOG_PATH,
                verbose=1,
                device=self.config.DEVICE
            )
        else:
            model = self._create_ppo_model(vec_env)
    
        return model

    def _create_ppo_model(self, vec_env):
        """Create PPO model (memory efficient)"""
        return PPO(
            "MlpPolicy",
            vec_env,
            learning_rate=self.config.LEARNING_RATE,
            n_steps=2048,
            batch_size=min(self.config.BATCH_SIZE, 256),
            n_epochs=10,
            gamma=self.config.GAMMA,
            gae_lambda=0.95,
            clip_range=0.2,
            policy_kwargs=dict(
                net_arch=dict(pi=[512, 256], vf=[512, 256]),
                activation_fn=nn.GELU
            ),
            tensorboard_log=self.config.LOG_PATH,
            verbose=1,
            device=self.config.DEVICE
        )
    
    def _load_previous_model(self):
        """Load previous model and replay buffer"""
        if self.config.LOAD_LAST_MODEL:
            # Try to find latest model
            latest_model = self.persistence_manager.find_latest_model()
            if latest_model:
                try:
                    self.model = self.model.load(latest_model, env=self.env)
                    self.model_loaded = True
                    logger.info(f"Loaded previous model from {latest_model}")
                    
                    # Load replay buffer
                    self.persistence_manager.load_replay_buffer(self.model)
                    self.buffer_loaded = True
                    
                except Exception as e:
                    logger.warning(f"Failed to load previous model: {e}")
    
    def _save_current_state(self, force: bool = False):
        """Save current state"""
        current_time = datetime.now()
        
        # Save if forced or enough time has passed
        if force or (self.last_save_time is None or 
                    (current_time - self.last_save_time).total_seconds() > 300):  # Every 5 minutes
        
            # Save system state
            self.persistence_manager.save_system_state(self, self.data_collector)
            
            # Save model checkpoint
            if hasattr(self.model, 'save'):
                checkpoint_name = f"_autosave_{int(time.time())}"
                self.persistence_manager.save_model_checkpoint(self.model, self.current_timestep, checkpoint_name)
            
            # Save replay buffer
            if self.config.SAVE_REPLAY_BUFFER:
                self.persistence_manager.save_replay_buffer(self.model)
            
            self.last_save_time = current_time
            logger.info(f"State saved at step {self.current_timestep}")
    
    def train(self):
        """Train the agent with persistence"""
        logger.info("Starting training")
        self.is_training = True
        
        # Create callbacks
        callbacks = [
            CheckpointCallback(
                save_freq=self.config.CHECKPOINT_FREQUENCY,
                save_path=self.config.MODEL_SAVE_PATH,
                name_prefix="industrial_pipe_rl"
            ),
            TrainingCallback(self),
            EvalCallback(
                self.env,
                best_model_save_path=self.config.MODEL_SAVE_PATH + "/best/",
                log_path=self.config.LOG_PATH,
                eval_freq=self.config.EVAL_FREQUENCY,
                n_eval_episodes=self.config.EVAL_EPISODES
            ),
            PersistenceCallback(self)  # Custom callback for persistence
        ]
        
        try:
            self.model.learn(
                total_timesteps=self.config.TOTAL_TIMESTEPS,
                callback=callbacks,
                tb_log_name=f"{self.config.RL_ALGORITHM}_industrial",
                progress_bar=True
            )
            
            # Save final model
            self.model.save(f"{self.config.MODEL_SAVE_PATH}/final_model")
            logger.info("Training completed")
            
            # Save final state
            self._save_current_state(force=True)
            
        except KeyboardInterrupt:
            logger.info("Training interrupted")
            self.model.save(f"{self.config.MODEL_SAVE_PATH}/interrupted_model")
            self._save_current_state(force=True)
        except Exception as e:
            logger.error(f"Training error: {e}")
            self._save_current_state(force=True)
            raise
        finally:
            self.is_training = False
    
    def deploy(self):
        """Deploy agent"""
        logger.info("Deploying RL agent")
        
        # Try to load best model
        model_path = f"{self.config.MODEL_SAVE_PATH}/best_model.zip"
        if os.path.exists(model_path):
            self.model = self.model.load(model_path, env=self.env)
            logger.info(f"Loaded best model from {model_path}")
        else:
            logger.warning("No trained model found, using current model")
        
        print("\n" + "="*60)
        print("ENHANCED INDUSTRIAL RL AGENT WITH PERSISTENCE")
        print("="*60)
        print(f"REST API: http://{self.config.REST_API_HOST}:{self.config.REST_API_PORT}")
        print(f"UDP Port: {self.config.UDP_PORT}")
        print(f"ROS2: {'ENABLED' if self.config.ROS2_ENABLED else 'DISABLED'}")
        print(f"Algorithm: {self.config.RL_ALGORITHM}")
        print(f"Model Loaded: {self.model_loaded}")
        print(f"Buffer Loaded: {self.buffer_loaded}")
        print(f"Episode Count: {self.episode_count}")
        print(f"Total Reward: {self.total_reward:.2f}")
        print("\nLog Directories:")
        print(f"  Main: {directories['logs']}")
        print(f"  ROS2: {directories['ros2_logs']}")
        print(f"  UDP: {directories['udp_logs']}")
        print(f"  REST API: {directories['restapi_logs']}")
        print(f"  Rewards: {directories['reward_logs']}")
        print(f"  Models: {directories['models']}")
        print("\nExample API Calls:")
        print(f'  Close valve: curl -X POST http://localhost:{self.config.REST_API_PORT}/rl_control \\')
        print('    -H "Content-Type: application/json" \\')
        print('    -d \'{"unique_id": "RL_aqua_sential", "sensor_id": 1, "valve": 0}\'')
        print("\n" + "="*60)
        print("Agent is now running. Press Ctrl+C to stop.\n")
        
        obs = self.env.reset()[0]
        self.episode_count += 1
        
        while True:
            try:
                action, _ = self.model.predict(obs, deterministic=True)
                obs, reward, done, _, info = self.env.step(action)
                
                # Update total reward
                self.total_reward += reward
                
                self.performance_monitor.update(info)
                alerts = self.alert_system.check_alerts(info)
                if alerts:
                    self._handle_alerts(alerts)
                
                # Periodic saving
                if self.env.episode_step % self.config.AUTOSAVE_FREQUENCY == 0:
                    self._save_current_state()
                
                if self.env.episode_step % 100 == 0:
                    self._log_deployment_status(info)
                
                if done:
                    obs = self.env.reset()[0]
                    self.episode_count += 1
                    logger.info(f"Episode {self.episode_count} completed")
                    
                    # Save episode summary
                    self._save_current_state()
                
                time.sleep(0.1)
                
            except KeyboardInterrupt:
                logger.info("Deployment stopped by user")
                self._save_current_state(force=True)
                break
            except Exception as e:
                logger.error(f"Deployment error: {e}")
                self._save_current_state(force=True)
                time.sleep(1)
    
    def _handle_alerts(self, alerts: List[Dict]):
        """Handle alerts"""
        for alert in alerts:
            logger.warning(f"ALERT: {alert['type']} - {alert['message']}")
            
            if (alert['type'] == 'leak' and 
                alert['probability'] > self.config.EMERGENCY_SHUTDOWN_THRESHOLD):
                self._emergency_shutdown()
    
    def _emergency_shutdown(self):
        """Emergency shutdown"""
        logger.critical("EMERGENCY SHUTDOWN")
        
        ros2_node = self.data_collector.get_ros2_node()
        if ros2_node:
            for i in range(20):
                ros2_node.publish_valve_command(
                    valve_id=i,
                    command=-1.0,
                    priority=3,
                    emergency=True
                )
        
        self.alert_system.send_emergency_notification("Emergency shutdown")
        self.is_training = False
        
        # Save state before shutdown
        self._save_current_state(force=True)
    
    def _log_deployment_status(self, info: Dict):
        """Log status"""
        metrics = info.get('metrics', {})
        leak_pred = info.get('leak_predictions', {})
        actions = info.get('actions_executed', {})
        
        log_msg = (
            f"Episode: {self.episode_count} | "
            f"Step: {self.env.episode_step} | "
            f"Reward: {self.env.episode_reward:.2f} | "
            f"Total Reward: {self.total_reward:.2f} | "
            f"Leak Prob: {leak_pred.get('probability', 0):.3f} | "
            f"Valve Changes: {len(actions.get('valve_changes', []))} | "
            f"Success Rate: {actions.get('success_rate', 0):.2f}"
        )
        
        logger.info(log_msg)
    
    def get_status(self) -> Dict:
        """Get agent status"""
        return {
            'training': self.is_training,
            'timestep': self.current_timestep,
            'episode': self.episode_count,
            'total_reward': self.total_reward,
            'performance': self.performance_monitor.get_summary(),
            'data_stats': self.data_collector.get_statistics(),
            'alerts': self.alert_system.get_active_alerts(),
            'api_enabled': self.config.REST_API_ENABLED,
            'udp_enabled': self.config.UDP_ENABLED,
            'ros2_enabled': self.config.ROS2_ENABLED,
            'model_loaded': self.model_loaded,
            'buffer_loaded': self.buffer_loaded,
            'last_save_time': self.last_save_time.isoformat() if self.last_save_time else None,
            'log_directories': {k: str(v) for k, v in directories.items()}
        }
    
    def shutdown(self):
        """Shutdown agent"""
        logger.info("Shutting down RL agent")
        self.is_training = False
        
        # Save final state
        if self.config.SAVE_ON_SHUTDOWN:
            self._save_current_state(force=True)
        
        if self.config.REST_API_ENABLED:
            self.rest_api.stop()
        
        self.data_collector.stop()
        time.sleep(1)
        
        logger.info("Agent shutdown complete")

# ============================================
# Supporting Classes
# ============================================

class TrainingCallback(BaseCallback):
    """Training callback"""
    
    def __init__(self, agent: 'EnhancedIndustrialRLAgent', verbose: int = 0):
        super().__init__(verbose)
        self.agent = agent
        self.training_logger = loggers['training_logger']
    
    def _on_step(self) -> bool:
        self.agent.current_timestep = self.num_timesteps
        
        if self.num_timesteps % 1000 == 0:
            logger.info(f"Training timestep: {self.num_timesteps}")
            self.training_logger.info(f"Training step: {self.num_timesteps}")
        
        return True

class PersistenceCallback(BaseCallback):
    """Callback for periodic persistence"""
    
    def __init__(self, agent: 'EnhancedIndustrialRLAgent', save_frequency: int = 1000):
        super().__init__()
        self.agent = agent
        self.save_frequency = save_frequency
        self.last_save = 0
    
    def _on_step(self) -> bool:
        if self.num_timesteps - self.last_save >= self.save_frequency:
            self.agent._save_current_state(force=True)
            self.last_save = self.num_timesteps
        
        return True

class PerformanceMonitor:
    """Performance monitor"""
    
    def __init__(self):
        self.history = deque(maxlen=1000)
        self.metrics = defaultdict(list)
        self.performance_logger = loggers['performance_logger']
        
        # Performance logging
        self.performance_log_file = directories['performance_logs'] / "performance_metrics.csv"
        self._init_performance_logging()
    
    def _init_performance_logging(self):
        """Initialize performance logging"""
        if not self.performance_log_file.exists():
            with open(self.performance_log_file, 'w') as f:
                f.write("timestamp,step,episode_reward,avg_reward,valve_changes,"
                       "maintenance_actions,leaks_prevented,pressure_violations\n")
    
    def update(self, info: Dict):
        """Update metrics"""
        self.history.append(info)
        
        if 'metrics' in info:
            for key, value in info['metrics'].items():
                if isinstance(value, (int, float)):
                    self.metrics[key].append(value)
        
        # Log performance metrics
        if info.get('episode_step', 0) % 10 == 0:  # Log every 10 steps
            with open(self.performance_log_file, 'a') as f:
                f.write(f"{datetime.now().isoformat()},"
                       f"{info.get('episode_step', 0)},"
                       f"{info.get('episode_reward', 0):.4f},"
                       f"{np.mean(self.metrics.get('efficiency_score', [0])):.4f},"
                       f"{info.get('metrics', {}).get('valve_changes', 0)},"
                       f"{info.get('metrics', {}).get('maintenance_actions', 0)},"
                       f"{info.get('metrics', {}).get('leaks_prevented', 0)},"
                       f"{info.get('metrics', {}).get('pressure_violations', 0)}\n")
    
    def get_summary(self) -> Dict:
        """Get summary"""
        summary = {}
        
        for key, values in self.metrics.items():
            if values:
                summary[f"{key}_mean"] = np.mean(values[-100:])
                summary[f"{key}_std"] = np.std(values[-100:])
                summary[f"{key}_min"] = np.min(values[-100:]) if len(values) > 0 else 0
                summary[f"{key}_max"] = np.max(values[-100:]) if len(values) > 0 else 0
        
        summary['total_steps'] = len(self.history)
        summary['recent_rewards'] = [h.get('episode_reward', 0) for h in list(self.history)[-10:]]
        
        return summary

class AlertSystem:
    """Alert system"""
    
    def __init__(self, config: IndustrialConfig):
        self.config = config
        self.active_alerts = {}
        self.alert_history = deque(maxlen=1000)
        self.alert_logger = loggers['alert_logger']
        
        # Alert logging
        self.alert_log_file = directories['alert_logs'] / "alerts.csv"
        self._init_alert_logging()
    
    def _init_alert_logging(self):
        """Initialize alert logging"""
        if not self.alert_log_file.exists():
            with open(self.alert_log_file, 'w') as f:
                f.write("timestamp,alert_type,severity,value,message,action_taken\n")
    
    def check_alerts(self, info: Dict) -> List[Dict]:
        """Check for alerts"""
        alerts = []
        
        leak_pred = info.get('leak_predictions', {})
        leak_prob = leak_pred.get('probability', 0.0)
        
        if leak_prob > self.config.ALERT_THRESHOLDS['leak_probability']:
            alerts.append({
                'type': 'leak',
                'severity': 'high',
                'probability': leak_prob,
                'message': f"High leak probability: {leak_prob:.2%}",
                'timestamp': datetime.now().isoformat()
            })
        
        metrics = info.get('metrics', {})
        efficiency = metrics.get('efficiency_score', 0.0)
        
        if efficiency < 0.5:
            alerts.append({
                'type': 'efficiency',
                'severity': 'medium',
                'value': efficiency,
                'message': f"Low efficiency: {efficiency:.2%}",
                'timestamp': datetime.now().isoformat()
            })
        
        # Log all alerts
        for alert in alerts:
            alert_id = hashlib.md5(json.dumps(alert).encode()).hexdigest()[:8]
            self.active_alerts[alert_id] = alert
            self.alert_history.append(alert)
            
            # Log to file
            with open(self.alert_log_file, 'a') as f:
                f.write(f"{alert['timestamp']},"
                       f"{alert['type']},"
                       f"{alert['severity']},"
                       f"{alert.get('value', alert.get('probability', 0)):.4f},"
                       f"{alert['message']},"
                       f"{'logged'}\n")
            
            # Log to alert logger
            self.alert_logger.warning(
                f"ALERT: {alert['type']} - {alert['message']} "
                f"(Severity: {alert['severity']})"
            )
        
        return alerts
    
    def send_emergency_notification(self, message: str):
        """Send emergency notification"""
        logger.critical(f"EMERGENCY: {message}")
        self.alert_logger.critical(f"EMERGENCY: {message}")
        print(f"\n🚨 EMERGENCY: {message}\n")
    
    def get_active_alerts(self) -> List[Dict]:
        """Get active alerts"""
        current_time = datetime.now()
        expired = []
        
        for alert_id, alert in self.active_alerts.items():
            alert_time = datetime.fromisoformat(alert['timestamp'])
            if (current_time - alert_time).seconds > 3600:
                expired.append(alert_id)
        
        for alert_id in expired:
            del self.active_alerts[alert_id]
        
        return list(self.active_alerts.values())

# ============================================
# ROS2 Node (if ROS2 available)
# ============================================

if ROS2_AVAILABLE:
    class ROS2PipeNode(Node):
        """ROS2 node for pipe monitoring with logging"""
        
        def __init__(self, config: IndustrialConfig, data_collector: EnhancedIndustrialDataCollector):
            super().__init__('industrial_pipe_rl_node')
            self.config = config
            self.data_collector = data_collector
            self.ros2_logger = data_collector.ros2_logger
            
            # Setup publishers and subscribers
            self._setup_communication()
            
            # ROS2 message logging
            self.message_log_file = directories['ros2_logs'] / "ros2_messages.csv"
            self._init_ros2_logging()
            
            self.get_logger().info('ROS2 Pipe Node initialized')
        
        def _init_ros2_logging(self):
            """Initialize ROS2 logging"""
            if not self.message_log_file.exists():
                with open(self.message_log_file, 'w') as f:
                    f.write("timestamp,topic,message_type,size,data\n")
        
        def _setup_communication(self):
            """Setup ROS2 communication"""
            # Setup publishers and subscribers
            pass
        
        def publish_valve_command(self, valve_id: int, command: float, priority: int = 0, emergency: bool = False):
            """Publish valve command with logging"""
            try:
                # ROS2 message publishing logic
                message = f"Valve {valve_id}: {command:.3f}"
                self.get_logger().info(message)
                
                # Log to CSV
                with open(self.message_log_file, 'a') as f:
                    f.write(f"{datetime.now().isoformat()},"
                           f"/valves/commands,"
                           f"ValveCommand,"
                           f"{len(message)},"
                           f"{message}\n")
                
                self.ros2_logger.info(f"Published valve command: {message}")
                return True
            except Exception as e:
                self.get_logger().error(f"Error publishing valve command: {e}")
                self.ros2_logger.error(f"Valve command error: {e}")
                return False
        
        def publish_maintenance_request(self, sensor_id: int, priority: int, issue_type: str, description: str):
            """Publish maintenance request with logging"""
            try:
                # ROS2 message publishing logic
                message = f"Sensor {sensor_id}: {issue_type} (Priority: {priority})"
                self.get_logger().warning(message)
                
                # Log to CSV
                with open(self.message_log_file, 'a') as f:
                    f.write(f"{datetime.now().isoformat()},"
                           f"/maintenance/requests,"
                           f"MaintenanceRequest,"
                           f"{len(message)},"
                           f"{message}\n")
                
                self.ros2_logger.warning(f"Published maintenance request: {message}")
                return True
            except Exception as e:
                self.get_logger().error(f"Error publishing maintenance request: {e}")
                self.ros2_logger.error(f"Maintenance request error: {e}")
                return False
        
        def publish_emergency_alert(self, leak_probability: float, location_x: float, location_y: float, 
                                   severity: int, confidence: float, action: str):
            """Publish emergency alert with logging"""
            try:
                # ROS2 message publishing logic
                message = f"Leak {leak_probability:.2%} at ({location_x:.2f}, {location_y:.2f}) - {action}"
                self.get_logger().critical(message)
                
                # Log to CSV
                with open(self.message_log_file, 'a') as f:
                    f.write(f"{datetime.now().isoformat()},"
                           f"/alerts/emergency,"
                           f"EmergencyAlert,"
                           f"{len(message)},"
                           f"{message}\n")
                
                self.ros2_logger.critical(f"Published emergency alert: {message}")
                return True
            except Exception as e:
                self.get_logger().error(f"Error publishing emergency alert: {e}")
                self.ros2_logger.error(f"Emergency alert error: {e}")
                return False

# ============================================
# ROS2 Node Wrapper
# ============================================

def ros2_main(args=None):
    """ROS2 entry point for executable"""
    import rclpy
    
    # Initialize ROS2
    rclpy.init(args=args)
    
    # Parse arguments
    import argparse
    parser = argparse.ArgumentParser(description='Industrial RL Agent')
    parser.add_argument('--mode', default='deploy', help='Operation mode')
    parser.add_argument('--api-port', type=int, default=5000, help='REST API port')
    parser.add_argument('--api-host', default='0.0.0.0', help='REST API host')
    parser.add_argument('--udp-port', type=int, default=8888, help='UDP port')
    parser.add_argument('--log-level', default='info', help='Log level')
    parser.add_argument('--load-last', action='store_true', help='Load last model')
    parser.add_argument('--no-persistence', action='store_true', help='Disable persistence')
    
    # Get ROS2 args
    ros_args = rclpy.utilities.remove_ros_args(args)[1:] if args else []
    parsed_args = parser.parse_args(ros_args)
    
    print(f"\n{'='*60}")
    print("INDUSTRIAL RL AGENT - ROS2 EXECUTABLE")
    print(f"{'='*60}")
    print(f"Mode: {parsed_args.mode}")
    print(f"REST API: http://{parsed_args.api_host}:{parsed_args.api_port}")
    print(f"UDP Port: {parsed_args.udp_port}")
    print(f"Persistence: {'Enabled' if not parsed_args.no_persistence else 'Disabled'}")
    print(f"Load Last Model: {parsed_args.load_last}")
    print(f"Log Level: {parsed_args.log_level}")
    print(f"{'='*60}\n")
    
    # Create configuration
    config = IndustrialConfig()
    config.ROS2_ENABLED = True
    config.REST_API_PORT = parsed_args.api_port
    config.REST_API_HOST = parsed_args.api_host
    config.UDP_PORT = parsed_args.udp_port
    config.LOAD_LAST_MODEL = parsed_args.load_last
    config.SAVE_ON_SHUTDOWN = not parsed_args.no_persistence
    
    # Set log level
    log_levels = {'debug': logging.DEBUG, 'info': logging.INFO, 
                  'warning': logging.WARNING, 'error': logging.ERROR}
    logging.getLogger().setLevel(log_levels.get(parsed_args.log_level, logging.INFO))
    
    try:
        # Create and run agent
        agent = EnhancedIndustrialRLAgent(config)
        
        if parsed_args.mode == 'train':
            agent.train()
        elif parsed_args.mode == 'deploy':
            agent.deploy()
        elif parsed_args.mode == 'api-only':
            print(f"REST API running at: http://{parsed_args.api_host}:{parsed_args.api_port}")
            print(f"Log directory: {directories['logs']}")
            print("Press Ctrl+C to stop\n")
            try:
                while rclpy.ok():
                    time.sleep(1)
            except KeyboardInterrupt:
                pass
        elif parsed_args.mode == 'test':
            test_udp_connection(config)
        
        agent.shutdown()
        
    except KeyboardInterrupt:
        logger.info("RL Agent stopped by user")
    except Exception as e:
        logger.error(f"RL Agent error: {e}")
        raise
    finally:
        rclpy.shutdown()
    
    return 0

# ============================================
# Main Execution
# ============================================

def main():
    """Main entry point"""
    import argparse
    
    parser = argparse.ArgumentParser(description="Enhanced Industrial Pipe Network RL Agent")
    parser.add_argument("--mode", choices=["train", "deploy", "api-only", "test"], 
                       default="deploy", help="Operation mode")
    parser.add_argument("--config", type=str, help="Config file path")
    parser.add_argument("--model", type=str, help="Model path")
    parser.add_argument("--ros2", action="store_true", help="Enable ROS2")
    parser.add_argument("--udp", action="store_true", help="Enable UDP")
    parser.add_argument("--no-api", action="store_true", help="Disable REST API")
    parser.add_argument("--api-port", type=int, default=5000, help="REST API port")
    parser.add_argument("--api-host", type=str, default="0.0.0.0", help="REST API host")
    parser.add_argument("--udp-port", type=int, default=8888, help="UDP port")
    parser.add_argument("--load-last", action="store_true", help="Load last model")
    parser.add_argument("--no-persistence", action="store_true", help="Disable persistence")
    parser.add_argument("--log-dir", type=str, default="~/.industrial_pipe_rl", help="Log directory")
    
    args = parser.parse_args()
    
    # Re-initialize logging with custom directory if provided
    global directories, loggers
    directories, loggers = setup_logging(args.log_dir)
    
    try:
        # Load configuration
        config = IndustrialConfig()
        
        # Override based on arguments
        if args.ros2 is not None:
            config.ROS2_ENABLED = args.ros2
        if args.udp is not None:
            config.UDP_ENABLED = args.udp
        if args.api_port:
            config.REST_API_PORT = args.api_port
        if args.api_host:
            config.REST_API_HOST = args.api_host
        if args.udp_port:
            config.UDP_PORT = args.udp_port
        if args.no_api:
            config.REST_API_ENABLED = False
        if args.load_last:
            config.LOAD_LAST_MODEL = args.load_last
        if args.no_persistence:
            config.SAVE_ON_SHUTDOWN = False
            config.LOAD_LAST_MODEL = False
            config.LOAD_REPLAY_BUFFER = False
        
        # Create agent
        agent = EnhancedIndustrialRLAgent(config)
        
        print("\n" + "="*60)
        print("ENHANCED INDUSTRIAL PIPE NETWORK RL AGENT")
        print("="*60)
        print(f"Mode: {args.mode.upper()}")
        print(f"REST API: {'ENABLED' if config.REST_API_ENABLED else 'DISABLED'}")
        if config.REST_API_ENABLED:
            print(f"API URL: http://{config.REST_API_HOST}:{config.REST_API_PORT}")
        print(f"UDP: {'ENABLED' if config.UDP_ENABLED else 'DISABLED'}")
        if config.UDP_ENABLED:
            print(f"UDP Port: {config.UDP_PORT}")
        print(f"ROS2: {'ENABLED' if config.ROS2_ENABLED else 'DISABLED'}")
        print(f"Device: {config.DEVICE}")
        print(f"Algorithm: {config.RL_ALGORITHM}")
        print(f"Persistence: {'ENABLED' if config.SAVE_ON_SHUTDOWN else 'DISABLED'}")
        print(f"Load Last Model: {'YES' if config.LOAD_LAST_MODEL else 'NO'}")
        print(f"Log Directory: {directories['logs']}")
        print("\nLog Subdirectories:")
        print(f"  ROS2: {directories['ros2_logs']}")
        print(f"  UDP: {directories['udp_logs']}")
        print(f"  REST API: {directories['restapi_logs']}")
        print(f"  Rewards: {directories['reward_logs']}")
        print(f"  Training: {directories['training_logs']}")
        print("="*60 + "\n")
        
        # Execute based on mode
        if args.mode == "train":
            print("Starting training mode...")
            agent.train()
        elif args.mode == "deploy":
            print("Starting deployment mode...")
            agent.deploy()
        elif args.mode == "api-only":
            print("Starting API-only mode...")
            print(f"REST API at: http://{config.REST_API_HOST}:{config.REST_API_PORT}")
            print("Press Ctrl+C to stop\n")
            try:
                while True:
                    time.sleep(1)
            except KeyboardInterrupt:
                pass
        elif args.mode == "test":
            print("Starting test mode...")
            test_udp_connection(config)
        
    except KeyboardInterrupt:
        logger.info("Program interrupted")
    except Exception as e:
        logger.error(f"Fatal error: {e}", exc_info=True)
        raise
    finally:
        if 'agent' in locals():
            agent.shutdown()

def test_udp_connection(config: IndustrialConfig):
    """Test UDP connection"""
    print(f"\nTesting UDP connection on port {config.UDP_PORT}...")
    
    # Create test UDP message
    test_message = {
        "metadata": {
            "save_time": datetime.now().isoformat(),
            "total_packets": 1,
            "saved_packets": 1,
            "duration": 0.1
        },
        "packets": [
            {
                "timestamp": datetime.now().isoformat(),
                "packet_number": 1,
                "source": "127.0.0.1:12345",
                "size": 100,
                "complete": True,
                "data": {
                    "avg_flow_rate": 0.5,
                    "reservoir_level_pct": 75.0,
                    "supply_efficiency": 85.0,
                    "pressure_compliance": 0.9,
                    "valve_states": [100.0] * 50,
                    "zones": [
                        {
                            "avg_pressure": 200.0,
                            "flow": 0.1,
                            "leak_flag": False,
                            "pressure_violation": False
                        }
                    ] * 20
                }
            }
        ]
    }
    
    # Send test message
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.sendto(json.dumps(test_message).encode(), (config.UDP_HOST, config.UDP_PORT))
        print("✓ Test UDP message sent")
        print(f"  Logs will be saved to: {directories['udp_logs']}")
    except Exception as e:
        print(f"✗ Failed to send UDP message: {e}")
    finally:
        sock.close()

if __name__ == "__main__":
    main()