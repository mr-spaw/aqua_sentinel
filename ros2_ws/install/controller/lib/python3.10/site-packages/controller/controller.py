#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import threading
import time
from datetime import datetime
from flask import Flask, request, jsonify
import socket
from queue import Queue, Empty
from typing import Dict, List, Set, Tuple, Optional, Any
from concurrent.futures import ThreadPoolExecutor, as_completed
from multiprocessing import cpu_count
from collections import deque
import select
import itertools

class DualModePipeMonitorNode(Node):
    def __init__(self):
        super().__init__('dual_mode_pipe_monitor_node')
        
        # ========== MODE SELECTION ==========
        self.RL_TRAINING = False  # SET TO True TO ENABLE RL MODE
        self.RL_LOG_FILE = '/tmp/rl_training_log.jsonl'
        self.RL_UNIQUE_ID = "RL_aqua_sential"
        
        # Configuration
        self.SAFETY_MODE = True
        self.MIN_CRITICAL_DURATION = 30.0 if not self.RL_TRAINING else 120.0
        self.RECOVERY_HYSTERESIS = 3.0
        
        # ========== ENHANCED MULTI-THREADING ==========
        # Maximize CPU utilization for sensor processing
        self.NUM_WORKERS = cpu_count()  # Use ALL cores
        self.NUM_CLASSIFIER_THREADS = max(8, cpu_count())  # Dedicated classifier threads
        self.NUM_ACTION_THREADS = 4  # Action processing threads
        self.NUM_BROADCAST_THREADS = 2  # TCP broadcast threads
        
        # Per-sensor thread pools for true parallel processing
        self.sensor_thread_pools = {}  # sensor_id -> dedicated ThreadPoolExecutor
        self.MAX_THREADS_PER_SENSOR = 2  # Each sensor gets its own processing threads
        
        # ========== THRESHOLDS FOR 103 kPa BASELINE ==========
        self.THRESHOLDS = {
            'fresh': {
                # Level-based detection
                'leak_critical': 1.0,
                'leak_mid': 10.0,
                'block_critical': 99.0,
                'block_mid': 95.0,
                
                # PRESSURE THRESHOLDS (Updated for 103 kPa baseline)
                'pressure_critical': 90.0,      # <90 kPa = CRITICAL in automatic mode
                'pressure_warning': 95.0,       # <95 kPa = MID_CRITICAL in both modes
                'pressure_severe': 80.0,        # <80 kPa = SEVERE CRITICAL
                
                # PRESSURE TREND DETECTION (for early leak detection)
                'pressure_drop_rate_critical': -2.0,   # -2 kPa/min = CRITICAL
                'pressure_drop_rate_warning': -0.5,    # -0.5 kPa/min = MID_CRITICAL
            },
            'sewage': {
                'leak_critical': 1.0,
                'leak_mid': 10.0,
                'block_critical': 99.0,
                'block_mid': 95.0,
                
                # Sewage pressure (atmospheric baseline ~101 kPa)
                'pressure_critical': 100.0,     # <100 kPa = CRITICAL
                'pressure_warning': 100.5,      # <100.5 kPa = MID_CRITICAL
                'pressure_drop_rate_critical': -1.0,   # Sewage pressure should be stable
            }
        }
        
        # ========== PARALLEL PROCESSING QUEUES ==========
        # Use regular Queue instead of PriorityQueue to avoid comparison issues
        # We'll process all items equally fast with parallel workers
        self.sensor_queue = Queue(maxsize=10000)
        
        # Separate queues for different processing stages
        self.classification_queue = Queue(maxsize=5000)
        self.validation_queue = Queue(maxsize=5000)
        self.action_decision_queue = Queue(maxsize=2000)
        
        self.tcp_broadcast_queue = Queue(maxsize=5000)
        self.action_queue = Queue(maxsize=1000)
        self.log_queue = Queue(maxsize=1000)
        
        # ========== THREAD-SAFE DATA STRUCTURES ==========
        # Use separate locks for different data to reduce contention
        self.sensors = {}
        self.critical_sensors = {}
        self.active_actions = set()
        self.recent_actions = {}
        
        # Individual locks for better concurrency
        self.sensor_locks = {}  # sensor_id -> individual lock
        self.global_sensor_lock = threading.RLock()
        self.action_lock = threading.Lock()
        self.tcp_lock = threading.Lock()
        self.critical_lock = threading.Lock()
        
        # ========== PRESSURE HISTORY FOR TREND ANALYSIS ==========
        self.pressure_history = {}  # sensor_id -> deque of (timestamp, pressure)
        self.pressure_history_locks = {}  # sensor_id -> lock
        self.HISTORY_WINDOW = 10
        
        # Critical tracking (for RL feedback)
        self.critical_counters = {}
        self.critical_start_times = {}
        
        # RL FAILSAFE: Auto-action if RL doesn't respond
        self.RL_FAILSAFE_TIMEOUT = 120.0  # 2 minutes
        self.rl_pending_actions = {}  # sensor_id -> failsafe_timer
        
        self.tcp_clients = []
        
        # ========== THREAD POOLS ==========
        # Multiple thread pools for different tasks
        self.classification_pool = ThreadPoolExecutor(
            max_workers=self.NUM_CLASSIFIER_THREADS,
            thread_name_prefix='Classifier'
        )
        self.action_pool = ThreadPoolExecutor(
            max_workers=self.NUM_ACTION_THREADS,
            thread_name_prefix='Action'
        )
        self.broadcast_pool = ThreadPoolExecutor(
            max_workers=self.NUM_BROADCAST_THREADS,
            thread_name_prefix='Broadcast'
        )
        
        # ========== ROS2 PUBLISHERS ==========
        self.valve_pub = self.create_publisher(String, '/valve_control', 10)
        self.maintenance_pub = self.create_publisher(String, '/maintenance/request', 10)
        
        # ========== ROS2 SUBSCRIBERS ==========
        # Each subscriber gets its own callback executor
        self.create_subscription(String, '/esp1/sensors', 
                               lambda msg: self._parallel_sensor_callback(msg, 1), 10)
        self.create_subscription(String, '/esp2/sensors', 
                               lambda msg: self._parallel_sensor_callback(msg, 2), 10)
        self.create_subscription(String, '/esp3/sensors', 
                               lambda msg: self._parallel_sensor_callback(msg, 3), 10)
        self.create_subscription(String, '/esp4/sensors', 
                               lambda msg: self._parallel_sensor_callback(msg, 4), 10)
        
        self.create_subscription(String, '/valve_control',
                                self.valve_echo_callback, 10)
        self.create_subscription(String, '/maintenance/request',
                                self.maintenance_echo_callback, 10)
        
        # RL mode setup
        if self.RL_TRAINING:
            try:
                self.rl_log_file = open(self.RL_LOG_FILE, 'a')
                self.get_logger().info(f'RL logging enabled: {self.RL_LOG_FILE}')
            except Exception as e:
                self.get_logger().error(f'RL log file error: {e}')
                self.RL_TRAINING = False
            
            # Start REST API
            self._start_rest_api_server()
        
        # Start all worker threads
        self._start_worker_threads()
        
        self.get_logger().info('='*60)
        mode_name = 'RL MODE (API CONTROL)' if self.RL_TRAINING else 'AUTOMATIC MODE (RULE-BASED)'
        self.get_logger().info(f'PIPE MONITOR STARTED - {mode_name}')
        self.get_logger().info(f'Total CPU Cores: {cpu_count()}')
        self.get_logger().info(f'Classifier Threads: {self.NUM_CLASSIFIER_THREADS}')
        self.get_logger().info(f'Action Threads: {self.NUM_ACTION_THREADS}')
        self.get_logger().info(f'Broadcast Threads: {self.NUM_BROADCAST_THREADS}')
        
        if self.RL_TRAINING:
            self.get_logger().info(f'REST API: http://0.0.0.0:5000/rl_control')
            self.get_logger().info(f'Unique ID: {self.RL_UNIQUE_ID}')
            self.get_logger().info(f'AUTOMATIC ACTIONS: DISABLED (waiting for API commands)')
        else:
            self.get_logger().info(f'AUTOMATIC ACTIONS: ENABLED (rule-based control)')
        
        self.get_logger().info(f'Critical Duration: {self.MIN_CRITICAL_DURATION}s')
        self.get_logger().info(f'Pressure Baseline: Fresh=103kPa, Sewage=101kPa')
        self.get_logger().info(f'Pressure Critical: <{self.THRESHOLDS["fresh"]["pressure_critical"]}kPa')
        self.get_logger().info('='*60)
    
    def _get_sensor_lock(self, sensor_id: int) -> threading.RLock:
        """Get or create individual lock for a sensor"""
        if sensor_id not in self.sensor_locks:
            with self.global_sensor_lock:
                if sensor_id not in self.sensor_locks:
                    self.sensor_locks[sensor_id] = threading.RLock()
        return self.sensor_locks[sensor_id]
    
    def _get_sensor_thread_pool(self, sensor_id: int) -> ThreadPoolExecutor:
        """Get or create dedicated thread pool for a sensor"""
        if sensor_id not in self.sensor_thread_pools:
            with self.global_sensor_lock:
                if sensor_id not in self.sensor_thread_pools:
                    self.sensor_thread_pools[sensor_id] = ThreadPoolExecutor(
                        max_workers=self.MAX_THREADS_PER_SENSOR,
                        thread_name_prefix=f'Sensor_{sensor_id}'
                    )
        return self.sensor_thread_pools[sensor_id]
    
    def _parallel_sensor_callback(self, msg: String, esp_id: int):
        """
        Parallel sensor callback - immediately dispatches to worker threads
        Each sensor is processed completely in parallel
        """
        try:
            data = json.loads(msg.data)
            sensors_list = data.get('sensors', [data]) if 'sensors' in data else [data]
            
            current_time = time.time()
            
            # Submit each sensor to its own thread pool immediately
            for sensor_data in sensors_list:
                if 'sensor_id' not in sensor_data:
                    continue
                
                sensor_id = sensor_data['sensor_id']
                
                # Submit to queue for processing (simple dict, no comparison needed)
                try:
                    self.sensor_queue.put_nowait({
                        'timestamp': current_time,
                        'sensor_data': sensor_data,
                        'sensor_id': sensor_id
                    })
                except:
                    # Queue full - process immediately in dedicated thread
                    pool = self._get_sensor_thread_pool(sensor_id)
                    pool.submit(self._process_single_sensor, sensor_data, current_time)
                
        except Exception as e:
            self.get_logger().error(f'Parallel callback error: {e}')
    
    def _process_single_sensor(self, sensor_data: dict, arrival_time: float):
        """
        Process a single sensor completely in parallel
        This function runs in its own thread
        """
        try:
            sensor_id = sensor_data.get('sensor_id')
            if sensor_id is None:
                return
            
            # Step 1: Classify sensor (fast, parallel)
            classification_result = self._classify_sensor_fast(sensor_data)
            if not classification_result:
                return
            
            sensor_id, status, issue_type, level, pressure, pipe_type, pressure_drop_rate = classification_result
            
            # Step 2: Update sensor state (thread-safe)
            state_updated = self._update_sensor_state(
                sensor_id, status, issue_type, level, pressure, 
                pipe_type, pressure_drop_rate, arrival_time
            )
            
            if not state_updated:
                return
            
            # Step 3: Handle critical state changes (parallel)
            if status == 'CRITICAL':
                self._handle_critical_sensor(
                    sensor_id, issue_type, pipe_type, level, 
                    pressure, pressure_drop_rate, arrival_time
                )
            
            # Step 4: Broadcast status updates (async)
            if status in ['CRITICAL', 'MID_CRITICAL']:
                self.broadcast_pool.submit(
                    self._broadcast_status_update,
                    sensor_id, status, issue_type, pipe_type,
                    level, pressure, pressure_drop_rate
                )
            
        except Exception as e:
            self.get_logger().error(f'Single sensor processing error: {e}')
    
    def _update_sensor_state(self, sensor_id: int, status: str, issue_type: str,
                            level: float, pressure: float, pipe_type: str,
                            pressure_drop_rate: float, current_time: float) -> bool:
        """Thread-safe sensor state update"""
        try:
            sensor_lock = self._get_sensor_lock(sensor_id)
            
            with sensor_lock:
                # Initialize sensor if new
                if sensor_id not in self.sensors:
                    self.sensors[sensor_id] = {
                        'current_status': 'NORMAL',
                        'issue_type': None,
                        'pressure_kpa': pressure,
                        'level_pct': level,
                        'valve': 1,
                        'pipe_type': pipe_type,
                        'last_critical_time': None,
                        'action_pending': False,
                        'last_update': datetime.now(),
                        'recovery_time': None,
                        'critical_count': 0,
                        'pressure_drop_rate': 0.0,
                    }
                
                state = self.sensors[sensor_id]
                old_status = state['current_status']
                
                # Update all fields
                state['level_pct'] = level
                state['pressure_kpa'] = pressure
                state['pipe_type'] = pipe_type
                state['current_status'] = status
                state['issue_type'] = issue_type
                state['pressure_drop_rate'] = pressure_drop_rate
                state['last_update'] = datetime.now()
                
                # Track critical state transitions
                critical_issue_types = ["LEAK", "BLOCKAGE", "LEAK_TREND", "LEAK_PRESSURE"]
                
                if status == "CRITICAL" and issue_type in critical_issue_types:
                    if sensor_id not in self.critical_sensors:
                        with self.critical_lock:
                            self.critical_sensors[sensor_id] = current_time
                            self.critical_start_times[sensor_id] = current_time
                            self.critical_counters[sensor_id] = 1
                        
                        state['last_critical_time'] = current_time
                        state['action_pending'] = True
                        
                        # RL MODE: Start failsafe timer
                        if self.RL_TRAINING:
                            self._start_rl_failsafe_timer(sensor_id, issue_type)
                    else:
                        # Increment critical counter
                        with self.critical_lock:
                            self.critical_counters[sensor_id] = self.critical_counters.get(sensor_id, 0) + 1
                
                elif old_status == "CRITICAL" and status != "CRITICAL":
                    if sensor_id in self.critical_sensors:
                        if state['recovery_time'] is None:
                            state['recovery_time'] = current_time
                        elif current_time - state['recovery_time'] >= self.RECOVERY_HYSTERESIS:
                            with self.critical_lock:
                                if sensor_id in self.critical_sensors:
                                    del self.critical_sensors[sensor_id]
                            state['action_pending'] = False
                            state['recovery_time'] = None
                            state['critical_count'] = 0
                
                elif status == "CRITICAL":
                    state['recovery_time'] = None
                
                return True
                
        except Exception as e:
            self.get_logger().error(f'State update error for sensor {sensor_id}: {e}')
            return False
    
    def _handle_critical_sensor(self, sensor_id: int, issue_type: str, pipe_type: str,
                                level: float, pressure: float, pressure_drop_rate: float,
                                current_time: float):
        """Handle critical sensor detection"""
        try:
            # Log critical event
            if "LEAK" in issue_type:
                icon = "[LEAK]"
                alert_type = "LEAK"
            elif "BLOCKAGE" in issue_type:
                icon = "[BLOCKAGE]"
                alert_type = "BLOCKAGE"
            else:
                icon = "[ISSUE TYPE]"
                alert_type = issue_type
            
            trend_str = f" (Δ{pressure_drop_rate:+.2f} kPa/min)" if abs(pressure_drop_rate) > 0.1 else ""
            
            self.get_logger().warn(
                f'{icon} CRITICAL {alert_type}: Sensor {sensor_id} ({pipe_type}) - '
                f'Level={level:.1f}% P={pressure:.1f}kPa{trend_str}'
            )
            
            # Async broadcast
            self.broadcast_pool.submit(
                self._broadcast_critical_alert,
                sensor_id, issue_type, pipe_type, level, pressure, pressure_drop_rate
            )
            
        except Exception as e:
            self.get_logger().error(f'Critical handler error: {e}')
    
    def _broadcast_status_update(self, sensor_id: int, status: str, issue_type: str,
                                 pipe_type: str, level: float, pressure: float,
                                 pressure_drop_rate: float):
        """Broadcast status update to TCP clients"""
        try:
            self.tcp_broadcast_queue.put({
                'type': 'STATUS_UPDATE',
                'sensor_id': sensor_id,
                'status': status,
                'issue_type': issue_type,
                'pipe_type': pipe_type,
                'pressure': pressure,
                'level': level,
                'pressure_drop_rate': pressure_drop_rate,
                'timestamp': datetime.now().isoformat()
            })
        except Exception as e:
            self.get_logger().error(f'Broadcast error: {e}')
    
    def _broadcast_critical_alert(self, sensor_id: int, issue_type: str, pipe_type: str,
                                  level: float, pressure: float, pressure_drop_rate: float):
        """Broadcast critical alert"""
        try:
            self.tcp_broadcast_queue.put({
                'type': 'CRITICAL_ALERT',
                'sensor_id': sensor_id,
                'issue_type': issue_type,
                'pipe_type': pipe_type,
                'pressure': pressure,
                'level': level,
                'pressure_drop_rate': pressure_drop_rate,
                'timestamp': datetime.now().isoformat()
            })
        except Exception as e:
            self.get_logger().error(f'Critical broadcast error: {e}')
    
    def _classify_sensor_fast(self, sensor_data: dict) -> tuple:
        """Ultra-fast classification with TREND analysis for BOTH modes"""
        try:
            sensor_id = sensor_data['sensor_id']
            level = sensor_data['level_pct']
            pipe_type = sensor_data.get('type', 'fresh')
            pressure = sensor_data.get('pressure_kpa', 0.0)
            
            thresh = self.THRESHOLDS[pipe_type]
            
            # ========== UPDATE PRESSURE HISTORY ==========
            current_time = time.time()
            
            # Get or create pressure history lock
            if sensor_id not in self.pressure_history_locks:
                with self.global_sensor_lock:
                    if sensor_id not in self.pressure_history_locks:
                        self.pressure_history_locks[sensor_id] = threading.Lock()
            
            pressure_lock = self.pressure_history_locks[sensor_id]
            
            with pressure_lock:
                if sensor_id not in self.pressure_history:
                    self.pressure_history[sensor_id] = deque(maxlen=self.HISTORY_WINDOW)
                
                self.pressure_history[sensor_id].append((current_time, pressure))
                
                # Calculate pressure trend
                pressure_drop_rate = 0.0
                if len(self.pressure_history[sensor_id]) >= 5:
                    history = list(self.pressure_history[sensor_id])
                    time_diff = history[-1][0] - history[0][0]
                    
                    if time_diff > 0:
                        pressure_diff = history[-1][1] - history[0][1]
                        pressure_drop_rate = (pressure_diff / time_diff) * 60.0  # kPa/min
            
            # ========== PRIORITY 1: Critical level thresholds ==========
            if level <= thresh['leak_critical']:  # ≤1%
                return sensor_id, "CRITICAL", "LEAK", level, pressure, pipe_type, pressure_drop_rate
            elif level >= thresh['block_critical']:  # ≥99%
                return sensor_id, "CRITICAL", "BLOCKAGE", level, pressure, pipe_type, pressure_drop_rate
            
            # ========== PRIORITY 2: Pressure trend detection ==========
            if pipe_type == 'fresh' and 'pressure_drop_rate_critical' in thresh:
                if pressure_drop_rate < thresh['pressure_drop_rate_critical']:  # < -2 kPa/min
                    return sensor_id, "CRITICAL", "LEAK_TREND", level, pressure, pipe_type, pressure_drop_rate
                elif pressure_drop_rate < thresh.get('pressure_drop_rate_warning', -0.5):  # < -0.5 kPa/min
                    return sensor_id, "MID_CRITICAL", "LEAK_TREND", level, pressure, pipe_type, pressure_drop_rate
            
            # ========== PRIORITY 3: Absolute pressure thresholds ==========
            if pressure < thresh.get('pressure_severe', 80.0):
                return sensor_id, "CRITICAL", "LEAK_PRESSURE", level, pressure, pipe_type, pressure_drop_rate
            elif pressure < thresh['pressure_critical']:
                return sensor_id, "CRITICAL", "LEAK_PRESSURE", level, pressure, pipe_type, pressure_drop_rate
            elif pressure < thresh.get('pressure_warning', 95.0):
                return sensor_id, "MID_CRITICAL", "LOW_PRESSURE", level, pressure, pipe_type, pressure_drop_rate
            
            # ========== PRIORITY 4: Mid-level alerts ==========
            if level <= thresh['leak_mid']:  # ≤10%
                return sensor_id, "MID_CRITICAL", "LEAK", level, pressure, pipe_type, pressure_drop_rate
            elif level >= thresh['block_mid']:  # ≥95%
                return sensor_id, "MID_CRITICAL", "BLOCKAGE", level, pressure, pipe_type, pressure_drop_rate
            
            # ========== NORMAL STATE ==========
            return sensor_id, "NORMAL", None, level, pressure, pipe_type, pressure_drop_rate
            
        except KeyError:
            return None
    
    def _start_rest_api_server(self):
        """Start Flask REST API server for RL agent control"""
        self.flask_app = Flask(__name__)
        
        @self.flask_app.route('/rl_control', methods=['POST'])
        def rl_control():
            try:
                data = request.get_json()
                
                # Validate unique ID
                if data.get('unique_id') != self.RL_UNIQUE_ID:
                    return jsonify({
                        'success': False,
                        'error': 'Invalid unique_id'
                    }), 403
                
                sensor_id = data.get('sensor_id')
                if sensor_id is None:
                    return jsonify({
                        'success': False,
                        'error': 'Missing sensor_id'
                    }), 400
                
                # Get current sensor state
                sensor_lock = self._get_sensor_lock(sensor_id)
                with sensor_lock:
                    if sensor_id not in self.sensors:
                        return jsonify({
                            'success': False,
                            'error': f'Sensor {sensor_id} not found'
                        }), 404
                    
                    sensor_state = self.sensors[sensor_id].copy()
                
                # Calculate critical duration
                critical_duration = 0.0
                critical_count = 0
                
                with self.critical_lock:
                    if sensor_id in self.critical_start_times:
                        critical_duration = time.time() - self.critical_start_times[sensor_id]
                    
                    if sensor_id in self.critical_counters:
                        critical_count = self.critical_counters[sensor_id]
                
                action_taken = None
                
                # Process VALVE command
                if 'valve' in data:
                    valve_state = data['valve']
                    if valve_state not in [0, 1]:
                        return jsonify({
                            'success': False,
                            'error': 'valve must be 0 or 1'
                        }), 400
                    
                    self._publish_valve_action(sensor_id, valve_state)
                    action_taken = f"valve_{'close' if valve_state == 0 else 'open'}"
                    
                    self.get_logger().info(
                        f'[RL-API] Valve {"close" if valve_state == 0 else "open"} '
                        f'sensor {sensor_id} (critical={critical_duration:.1f}s, count={critical_count})'
                    )
                
                # Process MAINTENANCE command
                if 'maintenance' in data:
                    maintenance_action = data['maintenance']
                    if maintenance_action not in [0, 1]:
                        return jsonify({
                            'success': False,
                            'error': 'maintenance must be 0 or 1'
                        }), 400
                    
                    if maintenance_action == 1:
                        issue_type = sensor_state.get('issue_type', 'UNKNOWN')
                        self._publish_maintenance(sensor_id, issue_type)
                        action_taken = "maintenance"
                        
                        self.get_logger().info(
                            f'[RL-API] Maintenance on sensor {sensor_id} '
                            f'(critical={critical_duration:.1f}s, count={critical_count})'
                        )
                        
                        # Reset critical counters
                        with sensor_lock:
                            if sensor_id in self.critical_counters:
                                del self.critical_counters[sensor_id]
                            if sensor_id in self.critical_start_times:
                                del self.critical_start_times[sensor_id]
                            if sensor_id in self.critical_sensors:
                                del self.critical_sensors[sensor_id]
                            
                            # Cancel failsafe timer if exists
                            if sensor_id in self.rl_pending_actions:
                                timer = self.rl_pending_actions[sensor_id]
                                if timer:
                                    timer.cancel()
                                del self.rl_pending_actions[sensor_id]
                
                # Build response
                response = {
                    'success': True,
                    'sensor_id': sensor_id,
                    'action': action_taken,
                    'critical_duration_seconds': round(critical_duration, 2),
                    'critical_count': critical_count,
                    'sensor_state': {
                        'pressure_kpa': sensor_state.get('pressure_kpa'),
                        'level_pct': sensor_state.get('level_pct'),
                        'status': sensor_state.get('current_status'),
                        'issue_type': sensor_state.get('issue_type'),
                        'pressure_drop_rate': sensor_state.get('pressure_drop_rate', 0.0),
                        'valve': sensor_state.get('valve'),
                    },
                    'timestamp': datetime.now().isoformat()
                }
                
                return jsonify(response), 200
                
            except Exception as e:
                self.get_logger().error(f'REST API error: {e}')
                return jsonify({
                    'success': False,
                    'error': str(e)
                }), 500
        
        def run_flask():
            self.flask_app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)
        
        flask_thread = threading.Thread(target=run_flask, daemon=True)
        flask_thread.start()
        self.get_logger().info('[RL-API] REST API server started on port 5000')
    
    def _publish_valve_action(self, sensor_id: int, valve_state: int):
        """Publish valve control (0=close, 1=open)"""
        try:
            msg = String()
            msg.data = f'{sensor_id}:{valve_state}'
            self.valve_pub.publish(msg)
            
            sensor_lock = self._get_sensor_lock(sensor_id)
            with sensor_lock:
                if sensor_id in self.sensors:
                    self.sensors[sensor_id]['valve'] = valve_state
            
            self.get_logger().info(
                f'[VALVE] {"Closed" if valve_state == 0 else "Opened"} valve for sensor {sensor_id}'
            )
            
            # Log to TCP broadcast
            self.tcp_broadcast_queue.put({
                'type': 'VALVE_ACTION',
                'sensor_id': sensor_id,
                'valve_state': valve_state,
                'timestamp': datetime.now().isoformat()
            })
            
        except Exception as e:
            self.get_logger().error(f'Valve publish error: {e}')
    
    def _start_rl_failsafe_timer(self, sensor_id: int, issue_type: str):
        """Start 2-minute failsafe timer for RL mode"""
        def failsafe_action():
            try:
                sensor_lock = self._get_sensor_lock(sensor_id)
                with sensor_lock:
                    if sensor_id not in self.sensors:
                        return
                    
                    # Check if still critical
                    if self.sensors[sensor_id].get('current_status') != 'CRITICAL':
                        return
                
                self.get_logger().warn(
                    f'[FAILSAFE] RL timeout for sensor {sensor_id} - '
                    f'Forcing automatic action after 2 minutes'
                )
                
                # Get critical duration for feedback
                critical_duration = 0.0
                critical_count = 0
                
                with self.critical_lock:
                    if sensor_id in self.critical_start_times:
                        critical_duration = time.time() - self.critical_start_times[sensor_id]
                    if sensor_id in self.critical_counters:
                        critical_count = self.critical_counters[sensor_id]
                
                # Execute forced action
                if "LEAK" in issue_type:
                    self._publish_valve_action(sensor_id, 0)  # Force close
                    
                    def delayed_maint():
                        self._publish_maintenance(sensor_id, issue_type)
                        
                        # Send failsafe feedback
                        self._send_failsafe_feedback(
                            sensor_id, 
                            "forced_valve_close_and_maintenance",
                            critical_duration,
                            critical_count
                        )
                    
                    threading.Timer(30.0, delayed_maint).start()
                    
                elif "BLOCKAGE" in issue_type:
                    self._publish_maintenance(sensor_id, issue_type)
                    
                    # Send failsafe feedback
                    self._send_failsafe_feedback(
                        sensor_id,
                        "forced_maintenance",
                        critical_duration,
                        critical_count
                    )
                
                # Clean up
                with sensor_lock:
                    if sensor_id in self.rl_pending_actions:
                        del self.rl_pending_actions[sensor_id]
                    if sensor_id in self.critical_counters:
                        del self.critical_counters[sensor_id]
                    if sensor_id in self.critical_start_times:
                        del self.critical_start_times[sensor_id]
                
            except Exception as e:
                self.get_logger().error(f'Failsafe action error: {e}')
        
        # Create and store timer
        timer = threading.Timer(self.RL_FAILSAFE_TIMEOUT, failsafe_action)
        self.rl_pending_actions[sensor_id] = timer
        timer.start()
        
        self.get_logger().info(
            f'[RL-FAILSAFE] Started 2-minute timer for sensor {sensor_id}'
        )
    
    def _send_failsafe_feedback(self, sensor_id: int, action: str, 
                                critical_duration: float, critical_count: int):
        """Send failsafe feedback to TCP clients"""
        feedback = {
            'type': 'FAILSAFE_ACTION',
            'sensor_id': sensor_id,
            'action': action,
            'reason': 'RL_timeout_2min',
            'critical_duration_seconds': round(critical_duration, 2),
            'critical_count': critical_count,
            'timestamp': datetime.now().isoformat()
        }
        
        self.tcp_broadcast_queue.put(feedback)
        
        self.get_logger().warn(
            f'[FAILSAFE-FEEDBACK] Sensor {sensor_id}: {action} '
            f'(critical for {critical_duration:.1f}s, count={critical_count})'
        )
    
    def _start_worker_threads(self):
        """Start all high-performance worker threads"""
        
        # Sensor queue processors (multiple threads for parallel processing)
        for i in range(self.NUM_CLASSIFIER_THREADS):
            thread = threading.Thread(
                target=self._sensor_queue_worker,
                name=f'SensorQueueWorker-{i}',
                daemon=True
            )
            thread.start()
        
        # Critical monitor (in automatic mode)
        if not self.RL_TRAINING:
            for i in range(self.NUM_ACTION_THREADS):
                thread = threading.Thread(
                    target=self._critical_monitor_worker,
                    name=f'CriticalMonitor-{i}',
                    daemon=True
                )
                thread.start()
        
        # Action processors
        for i in range(self.NUM_ACTION_THREADS):
            thread = threading.Thread(
                target=self._action_processor_worker,
                name=f'ActionProcessor-{i}',
                daemon=True
            )
            thread.start()
        
        # TCP broadcast workers
        for i in range(self.NUM_BROADCAST_THREADS):
            thread = threading.Thread(
                target=self._tcp_broadcast_worker,
                name=f'TCPBroadcast-{i}',
                daemon=True
            )
            thread.start()
        
        # Single-instance workers
        workers = [
            ('TCPServer', self._tcp_server_worker, 0.01),
            ('LogWriter', self._log_writer_worker, 0.1),
        ]
        
        for name, target, sleep_time in workers:
            thread = threading.Thread(
                target=self._high_perf_worker_wrapper,
                args=(name, target, sleep_time),
                name=name,
                daemon=True
            )
            thread.start()
        
        self.get_logger().info(f'Started {self.NUM_CLASSIFIER_THREADS + self.NUM_ACTION_THREADS*2 + self.NUM_BROADCAST_THREADS + 2} worker threads')
    
    def _sensor_queue_worker(self):
        """Process sensors from regular queue"""
        while rclpy.ok():
            try:
                # Get sensor task from queue
                task = self.sensor_queue.get(timeout=0.01)
                
                # Process in parallel
                self._process_single_sensor(task['sensor_data'], task['timestamp'])
                
                self.sensor_queue.task_done()
                
            except Empty:
                time.sleep(0.001)
            except Exception as e:
                self.get_logger().error(f'Sensor queue worker error: {e}')
                time.sleep(0.001)
    
    def _high_perf_worker_wrapper(self, name: str, target, sleep_time: float):
        """High-performance worker wrapper"""
        self.get_logger().info(f'Starting {name}')
        while rclpy.ok():
            try:
                target()
                if sleep_time > 0:
                    time.sleep(sleep_time)
            except Exception as e:
                self.get_logger().error(f'{name} error: {e}')
                time.sleep(1)
    
    def valve_echo_callback(self, msg: String):
        try:
            if ':' in msg.data:
                parts = msg.data.split(':')
                if len(parts) == 2:
                    sensor_id = int(parts[0])
                    valve_state = int(parts[1])
                    
                    sensor_lock = self._get_sensor_lock(sensor_id)
                    with sensor_lock:
                        if sensor_id in self.sensors:
                            self.sensors[sensor_id]['valve'] = valve_state
        except:
            pass
    
    def maintenance_echo_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            sensor_id = data.get('sensor_id')
            if sensor_id:
                self.get_logger().info(f'[OK] Maintenance echo: sensor {sensor_id}')
        except:
            pass
    
    def _critical_monitor_worker(self):
        """Monitor critical sensors - ONLY acts in AUTOMATIC mode"""
        while rclpy.ok():
            try:
                # RL MODE: Don't take automatic actions, just monitor
                if self.RL_TRAINING:
                    time.sleep(0.1)
                    continue
                
                current_time = time.time()
                
                with self.critical_lock:
                    critical_snapshot = list(self.critical_sensors.items())
                
                actions_to_queue = []
                
                for sensor_id, first_critical_time in critical_snapshot:
                    if sensor_id in self.active_actions:
                        continue
                    
                    if sensor_id in self.recent_actions:
                        time_since_action = current_time - self.recent_actions[sensor_id]
                        if time_since_action < 300.0:
                            continue
                    
                    elapsed = current_time - first_critical_time
                    if elapsed >= self.MIN_CRITICAL_DURATION:
                        sensor_lock = self._get_sensor_lock(sensor_id)
                        with sensor_lock:
                            if sensor_id in self.sensors:
                                state = self.sensors[sensor_id]
                                if state['action_pending']:
                                    actions_to_queue.append((sensor_id, state['issue_type']))
                                    state['action_pending'] = False
                
                for sensor_id, issue_type in actions_to_queue:
                    self.action_queue.put({
                        'type': 'CRITICAL_ACTION',
                        'sensor_id': sensor_id,
                        'issue_type': issue_type,
                        'timestamp': current_time
                    })
                
                time.sleep(0.1)
                
            except Exception as e:
                self.get_logger().error(f'Critical monitor error: {e}')
                time.sleep(1)
    
    def _action_processor_worker(self):
        """Process actions - ONLY in AUTOMATIC mode"""
        while rclpy.ok():
            try:
                # RL MODE: Don't process automatic actions
                if self.RL_TRAINING:
                    time.sleep(0.1)
                    continue
                
                action = self.action_queue.get(timeout=0.01)
                
                if action['type'] == 'CRITICAL_ACTION':
                    sensor_id = action['sensor_id']
                    issue_type = action['issue_type']
                    
                    # Submit to action pool for parallel processing
                    self.action_pool.submit(
                        self._execute_critical_action,
                        sensor_id,
                        issue_type,
                        action['timestamp']
                    )
                
                self.action_queue.task_done()
                
            except Empty:
                time.sleep(0.001)
            except Exception as e:
                self.get_logger().error(f'Action processor error: {e}')
                time.sleep(0.001)
    
    def _execute_critical_action(self, sensor_id: int, issue_type: str, timestamp: float):
        """Execute action - AUTOMATIC mode only"""
        try:
            current_time = time.time()
            
            if sensor_id in self.recent_actions:
                time_since_action = current_time - self.recent_actions[sensor_id]
                if time_since_action < 300.0:
                    return
            
            with self.action_lock:
                if sensor_id in self.active_actions:
                    return
                self.active_actions.add(sensor_id)
            
            self.get_logger().warn(f'[AUTO-ACTION] Taking action on sensor {sensor_id} - Issue: {issue_type}')
            
            self.recent_actions[sensor_id] = current_time
            
            # Handle all leak types (including pressure-based ones)
            if "LEAK" in issue_type:
                self._publish_valve_action(sensor_id, 0)
                
                def delayed_maintenance():
                    try:
                        self._publish_maintenance(sensor_id, issue_type)
                    finally:
                        with self.action_lock:
                            self.active_actions.discard(sensor_id)
                
                threading.Timer(30.0, delayed_maintenance).start()
                
            elif "BLOCKAGE" in issue_type:
                self._publish_maintenance(sensor_id, issue_type)
                with self.action_lock:
                    self.active_actions.discard(sensor_id)
            
            self.get_logger().info(f'[OK] Action scheduled for sensor {sensor_id}')
            
        except Exception as e:
            self.get_logger().error(f'Action error for sensor {sensor_id}: {e}')
            with self.action_lock:
                if sensor_id in self.active_actions:
                    self.active_actions.discard(sensor_id)
    
    def _publish_maintenance(self, sensor_id: int, issue_type: str):
        try:
            msg = String()
            maintenance_data = {
                "sensor_id": sensor_id,
                "action": "reset_sensor_to_initial",
                "issue_type": issue_type
            }
            msg.data = json.dumps(maintenance_data)
            self.maintenance_pub.publish(msg)
            self.get_logger().info(f'[MAINT] Published maintenance for sensor {sensor_id}')
            
            self.tcp_broadcast_queue.put({
                'type': 'MAINTENANCE_ACTION',
                'sensor_id': sensor_id,
                'action': 'maintenance_request',
                'issue_type': issue_type,
                'timestamp': datetime.now().isoformat()
            })
            
        except Exception as e:
            self.get_logger().error(f'Maintenance publish error: {e}')
    
    def _tcp_server_worker(self):
        """TCP server worker - handles incoming connections"""
        try:
            server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind(('0.0.0.0', 9999))
            server.listen(100)
            server.setblocking(False)
            
            while rclpy.ok():
                try:
                    client, address = server.accept()
                    client.setblocking(False)
                    with self.tcp_lock:
                        self.tcp_clients.append(client)
                    self.get_logger().info(f'[TCP] Client connected: {address}')
                    
                    welcome_msg = {
                        'type': 'SYSTEM_STATUS',
                        'message': 'Connected to Pipe Monitor',
                        'mode': 'RL_TRAINING' if self.RL_TRAINING else 'AUTOMATIC',
                        'timestamp': datetime.now().isoformat()
                    }
                    try:
                        client.sendall((json.dumps(welcome_msg) + '\n').encode())
                    except:
                        pass
                        
                except BlockingIOError:
                    self._check_tcp_clients()
                    time.sleep(0.01)
                    
        except Exception as e:
            self.get_logger().error(f'TCP server error: {e}')
            time.sleep(1)
    
    def _check_tcp_clients(self):
        """Check TCP clients for disconnections"""
        disconnected = []
        with self.tcp_lock:
            for client in self.tcp_clients:
                try:
                    ready_to_read, _, _ = select.select([client], [], [], 0)
                    if ready_to_read:
                        try:
                            data = client.recv(1, socket.MSG_PEEK)
                            if not data:
                                disconnected.append(client)
                        except:
                            disconnected.append(client)
                except:
                    disconnected.append(client)
            
            for client in disconnected:
                try:
                    client.close()
                except:
                    pass
                self.tcp_clients.remove(client)
                self.get_logger().info(f'[TCP] Client disconnected')
    
    def _tcp_broadcast_worker(self):
        """Broadcast messages to all TCP clients"""
        while rclpy.ok():
            try:
                data = self.tcp_broadcast_queue.get(timeout=0.01)
                message = json.dumps(data) + '\n'
                
                disconnected = []
                with self.tcp_lock:
                    for client in self.tcp_clients:
                        try:
                            client.sendall(message.encode())
                        except:
                            disconnected.append(client)
                    
                    for client in disconnected:
                        try:
                            client.close()
                        except:
                            pass
                        self.tcp_clients.remove(client)
                
                self.tcp_broadcast_queue.task_done()
                
            except Empty:
                time.sleep(0.001)
            except Exception as e:
                self.get_logger().error(f'TCP broadcast error: {e}')
                time.sleep(0.001)
    
    def _log_writer_worker(self):
        """Write logs to file (optional)"""
        while rclpy.ok():
            try:
                log_entry = self.log_queue.get(timeout=0.1)
                
                if self.RL_TRAINING and hasattr(self, 'rl_log_file'):
                    try:
                        self.rl_log_file.write(json.dumps(log_entry) + '\n')
                        self.rl_log_file.flush()
                    except:
                        pass
                
                self.log_queue.task_done()
                
            except Empty:
                pass
            except Exception as e:
                self.get_logger().error(f'Log writer error: {e}')
                time.sleep(0.1)
    
    def get_sensor_summary(self) -> dict:
        """Get summary of all sensors"""
        with self.global_sensor_lock:
            summary = {
                'total_sensors': len(self.sensors),
                'critical_sensors': len(self.critical_sensors),
                'mode': 'RL_TRAINING' if self.RL_TRAINING else 'AUTOMATIC',
                'active_threads': threading.active_count(),
                'sensors': {}
            }
            
            for sensor_id, state in self.sensors.items():
                summary['sensors'][sensor_id] = {
                    'status': state['current_status'],
                    'issue_type': state['issue_type'],
                    'level_pct': state['level_pct'],
                    'pressure_kpa': state['pressure_kpa'],
                    'valve': state['valve'],
                    'pipe_type': state['pipe_type'],
                    'pressure_drop_rate': state.get('pressure_drop_rate', 0.0)
                }
            
            return summary
    
    def cleanup(self):
        """Cleanup resources"""
        self.get_logger().info('Shutting down...')
        
        # Cancel all timers
        for timer in self.rl_pending_actions.values():
            if timer:
                timer.cancel()
        
        # Close TCP clients
        with self.tcp_lock:
            for client in self.tcp_clients:
                try:
                    client.close()
                except:
                    pass
            self.tcp_clients.clear()
        
        # Shutdown all thread pools
        self.get_logger().info('Shutting down thread pools...')
        self.classification_pool.shutdown(wait=False)
        self.action_pool.shutdown(wait=False)
        self.broadcast_pool.shutdown(wait=False)
        
        # Shutdown sensor-specific pools
        for sensor_id, pool in self.sensor_thread_pools.items():
            pool.shutdown(wait=False)
        
        # Close RL log file
        if self.RL_TRAINING and hasattr(self, 'rl_log_file'):
            try:
                self.rl_log_file.close()
            except:
                pass
        
        self.get_logger().info('Shutdown complete')


def main(args=None):
    rclpy.init(args=args)
    node = DualModePipeMonitorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutdown requested')
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()