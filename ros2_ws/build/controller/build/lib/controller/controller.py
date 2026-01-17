#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import threading
import time
from datetime import datetime
import socket
from queue import Queue, Empty
from typing import Dict, List, Set
from concurrent.futures import ThreadPoolExecutor, as_completed
from multiprocessing import cpu_count

class UltraFastPipeMonitorNode(Node):
    def __init__(self):
        super().__init__('ultrafast_pipe_monitor_node')
        
        # Configuration
        self.SAFETY_MODE = True
        self.MIN_CRITICAL_DURATION = 30.0  # 30 seconds before action
        self.RECOVERY_HYSTERESIS = 3.0  # 3 seconds recovery hysteresis
        
        # Use CPU cores efficiently
        self.NUM_WORKERS = max(4, cpu_count() - 2)
        
        # Optimized thresholds
        self.THRESHOLDS = {
            'fresh': {
                'leak_critical': 1.0,
                'leak_mid': 10.0,
                'block_critical': 99.0,
                'block_mid': 95.0,
                'pressure_critical': 55.0,
            },
            'sewage': {
                'leak_critical': 1.0,
                'leak_mid': 10.0,
                'block_critical': 99.0,
                'block_mid': 95.0,
                'pressure_critical': 50.0,
            }
        }
        
        # Fast data structures
        self.sensors = {}  # sensor_id -> dict state
        self.critical_sensors = {}  # sensor_id -> first_critical_time
        self.active_actions = set()
        self.recent_actions = {}  # sensor_id -> last_action_time
        
        # High-performance queues
        self.tcp_broadcast_queue = Queue(maxsize=5000)
        self.action_queue = Queue(maxsize=1000)
        self.batch_queue = Queue(maxsize=1000)
        self.log_queue = Queue(maxsize=1000)  # ADDED THIS LINE
        
        # Fast locks
        self.sensor_lock = threading.RLock()
        self.action_lock = threading.Lock()
        self.tcp_lock = threading.Lock()
        
        # TCP clients
        self.tcp_clients = []
        
        # Thread pools for parallel processing
        self.classification_pool = ThreadPoolExecutor(max_workers=self.NUM_WORKERS)
        self.action_pool = ThreadPoolExecutor(max_workers=4)
        
        # Publishers
        self.valve_pub = self.create_publisher(String, '/valve_control', 10)
        self.maintenance_pub = self.create_publisher(String, '/maintenance/request', 10)
        
        # Subscribers
        self.sensor_batch = []
        self.batch_timer = None
        self.BATCH_SIZE = 50
        self.BATCH_TIMEOUT = 0.05  # 50ms
        
        self.create_subscription(String, '/esp1/sensors', 
                               self._batch_sensor_callback, 10)
        self.create_subscription(String, '/esp2/sensors', 
                               self._batch_sensor_callback, 10)
        self.create_subscription(String, '/esp3/sensors', 
                               self._batch_sensor_callback, 10)
        self.create_subscription(String, '/esp4/sensors', 
                               self._batch_sensor_callback, 10)
        
        # Echo subscribers
        self.create_subscription(String, '/valve_control',
                                self.valve_echo_callback, 10)
        self.create_subscription(String, '/maintenance/request',
                                self.maintenance_echo_callback, 10)
        
        # Start worker threads
        self._start_worker_threads()
        
        self.get_logger().info('='*60)
        self.get_logger().info('ULTRAFAST PIPE MONITOR STARTED')
        self.get_logger().info(f'Workers: {self.NUM_WORKERS} | Batch size: {self.BATCH_SIZE}')
        self.get_logger().info(f'Using {cpu_count()} CPU cores')
        self.get_logger().info('='*60)
    
    def _start_worker_threads(self):
        """Start all high-performance worker threads"""
        threads = [
            ('BatchProcessor', self._batch_processor_worker, 0.001),  # 1ms sleep
            ('CriticalMonitor', self._critical_monitor_worker, 0.1),  # 100ms sleep
            ('ActionProcessor', self._action_processor_worker, 0.001),
            ('TCPBroadcast', self._tcp_broadcast_worker, 0.001),
            ('TCPServer', self._tcp_server_worker, 0.01),
            ('LogWriter', self._log_writer_worker, 0.1),
        ]
        
        for name, target, sleep_time in threads:
            thread = threading.Thread(
                target=self._high_perf_worker_wrapper,
                args=(name, target, sleep_time),
                name=name,
                daemon=True
            )
            thread.start()
    
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
    
    def _classify_sensor_fast(self, sensor_data: dict) -> tuple:
        """Ultra-fast classification - pure function for parallel processing"""
        try:
            sensor_id = sensor_data['sensor_id']
            level = sensor_data['level_pct']
            pipe_type = sensor_data.get('type', 'fresh')
            pressure = sensor_data.get('pressure_kpa', 0.0)
            
            thresh = self.THRESHOLDS[pipe_type]
            
            # Fast threshold checks
            if level <= thresh['leak_critical']:
                return sensor_id, "CRITICAL", "LEAK", level, pressure, pipe_type
            elif level <= thresh['leak_mid']:
                return sensor_id, "MID_CRITICAL", "LEAK", level, pressure, pipe_type
            elif level >= thresh['block_critical']:
                return sensor_id, "CRITICAL", "BLOCKAGE", level, pressure, pipe_type
            elif level >= thresh['block_mid']:
                return sensor_id, "MID_CRITICAL", "BLOCKAGE", level, pressure, pipe_type
            elif pressure < thresh['pressure_critical']:
                return sensor_id, "LOW", None, level, pressure, pipe_type
            else:
                return sensor_id, "NORMAL", None, level, pressure, pipe_type
        except KeyError:
            return None
    
    def _batch_sensor_callback(self, msg: String):
        """Batch sensor data for parallel processing"""
        try:
            data = json.loads(msg.data)
            sensors_list = data.get('sensors', [data]) if 'sensors' in data else [data]
            
            # Add to batch
            self.sensor_batch.extend(sensors_list)
            
            # Process batch if full or timer expired
            if len(self.sensor_batch) >= self.BATCH_SIZE:
                self._process_sensor_batch()
            elif self.batch_timer is None:
                self.batch_timer = threading.Timer(self.BATCH_TIMEOUT, self._process_sensor_batch)
                self.batch_timer.start()
                
        except Exception as e:
            self.get_logger().error(f'Batch callback error: {e}')
    
    def _process_sensor_batch(self):
        """Process sensor batch in parallel"""
        if not self.sensor_batch:
            if self.batch_timer:
                self.batch_timer.cancel()
                self.batch_timer = None
            return
        
        # Reset timer
        if self.batch_timer:
            self.batch_timer.cancel()
            self.batch_timer = None
        
        batch_to_process = self.sensor_batch[:self.BATCH_SIZE]
        self.sensor_batch = self.sensor_batch[self.BATCH_SIZE:]
        
        # Process in parallel
        futures = []
        for sensor_data in batch_to_process:
            if 'sensor_id' in sensor_data:
                future = self.classification_pool.submit(
                    self._classify_sensor_fast, sensor_data
                )
                futures.append(future)
        
        # Process results as they complete
        current_time = time.time()
        critical_updates = []
        
        for future in as_completed(futures):
            try:
                result = future.result(timeout=0.01)
                if not result:
                    continue
                    
                sensor_id, status, issue_type, level, pressure, pipe_type = result
                
                # Update sensor state (fast)
                with self.sensor_lock:
                    # Initialize sensor state if not exists
                    if sensor_id not in self.sensors:
                        self.sensors[sensor_id] = {
                            'current_status': 'NORMAL',
                            'issue_type': None,
                            'pressure_kpa': pressure,
                            'level_pct': level,
                            'valve': 1,
                            'pipe_type': pipe_type,
                            'leak_coeff_pct': 0.0,
                            'blockage_coeff_pct': 0.0,
                            'last_critical_time': None,
                            'action_pending': False,
                            'last_update': datetime.now(),
                            'recovery_time': None,
                            'critical_count': 0
                        }
                    
                    state = self.sensors[sensor_id]
                    old_status = state['current_status']
                    
                    # Update state
                    state['level_pct'] = level
                    state['pressure_kpa'] = pressure
                    state['pipe_type'] = pipe_type
                    state['current_status'] = status
                    state['issue_type'] = issue_type
                    state['last_update'] = datetime.now()
                    
                    # Handle critical state
                    if status == "CRITICAL":
                        if sensor_id not in self.critical_sensors:
                            self.critical_sensors[sensor_id] = current_time
                            state['last_critical_time'] = current_time
                            state['action_pending'] = True
                            
                            critical_updates.append((
                                sensor_id, issue_type, pipe_type, level, pressure
                            ))
                    
                    # Handle recovery
                    elif old_status == "CRITICAL" and status != "CRITICAL":
                        if sensor_id in self.critical_sensors:
                            if state['recovery_time'] is None:
                                state['recovery_time'] = current_time
                            elif current_time - state['recovery_time'] >= self.RECOVERY_HYSTERESIS:
                                del self.critical_sensors[sensor_id]
                                state['action_pending'] = False
                                state['recovery_time'] = None
                                state['critical_count'] = 0
                    
                    # Reset recovery timer if still critical
                    elif status == "CRITICAL":
                        state['recovery_time'] = None
                    
                    # Queue for broadcast
                    if status in ["MID_CRITICAL", "CRITICAL"] and status != old_status:
                        self.tcp_broadcast_queue.put({
                            'type': 'STATUS_UPDATE',
                            'sensor_id': sensor_id,
                            'status': status,
                            'issue_type': issue_type,
                            'pipe_type': pipe_type,
                            'pressure': pressure,
                            'level': level,
                            'timestamp': datetime.now().isoformat()
                        })
                
            except Exception as e:
                continue
        
        # Process critical alerts
        for sensor_id, issue_type, pipe_type, level, pressure in critical_updates:
            icon = "" if issue_type == "LEAK" else " warn"
            self.get_logger().warn(
                f'{icon} CRITICAL {issue_type}: Sensor {sensor_id} ({pipe_type}) - '
                f'Level={level:.1f}% P={pressure:.1f}kPa'
            )
            
            self.tcp_broadcast_queue.put({
                'type': 'CRITICAL_ALERT',
                'sensor_id': sensor_id,
                'issue_type': issue_type,
                'pipe_type': pipe_type,
                'pressure': pressure,
                'level': level,
                'timestamp': datetime.now().isoformat()
            })
        
        # Schedule next batch if needed
        if self.sensor_batch:
            self.batch_timer = threading.Timer(self.BATCH_TIMEOUT, self._process_sensor_batch)
            self.batch_timer.start()
    
    def _batch_processor_worker(self):
        """Process batched sensor data - dummy function for wrapper"""
        pass
    
    def valve_echo_callback(self, msg: String):
        """Fast valve echo"""
        try:
            if ':' in msg.data:
                parts = msg.data.split(':')
                if len(parts) == 2:
                    sensor_id = int(parts[0])
                    valve_state = int(parts[1])
                    with self.sensor_lock:
                        if sensor_id in self.sensors:
                            self.sensors[sensor_id]['valve'] = valve_state
        except:
            pass
    
    def maintenance_echo_callback(self, msg: String):
        """Fast maintenance echo"""
        try:
            data = json.loads(msg.data)
            sensor_id = data.get('sensor_id')
            if sensor_id:
                self.get_logger().info(f'Maintenance echo: sensor {sensor_id}')
        except:
            pass
    
    def _critical_monitor_worker(self):
        """Fast critical monitor"""
        try:
            current_time = time.time()
            
            # Get snapshot of critical sensors
            with self.sensor_lock:
                critical_snapshot = list(self.critical_sensors.items())
            
            actions_to_queue = []
            
            for sensor_id, first_critical_time in critical_snapshot:
                # Skip if in active actions or recent actions
                if sensor_id in self.active_actions:
                    continue
                
                if sensor_id in self.recent_actions:
                    time_since_action = current_time - self.recent_actions[sensor_id]
                    if time_since_action < 300.0:  # 5 minute cooldown
                        continue
                
                # Check critical duration
                elapsed = current_time - first_critical_time
                if elapsed >= self.MIN_CRITICAL_DURATION:
                    with self.sensor_lock:
                        if sensor_id in self.sensors:
                            state = self.sensors[sensor_id]
                            if state['action_pending']:
                                actions_to_queue.append((sensor_id, state['issue_type']))
                                state['action_pending'] = False
            
            # Queue actions
            for sensor_id, issue_type in actions_to_queue:
                self.action_queue.put({
                    'type': 'CRITICAL_ACTION',
                    'sensor_id': sensor_id,
                    'issue_type': issue_type,
                    'timestamp': current_time
                })
                
        except Exception as e:
            self.get_logger().error(f'Critical monitor error: {e}')
    
    def _action_processor_worker(self):
        """Parallel action processor"""
        try:
            action = self.action_queue.get_nowait()
            
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
            pass
        except Exception as e:
            self.get_logger().error(f'Action processor error: {e}')
    
    def _execute_critical_action(self, sensor_id: int, issue_type: str, timestamp: float):
        """Execute action in parallel thread"""
        try:
            current_time = time.time()
            
            # Check cooldown
            if sensor_id in self.recent_actions:
                time_since_action = current_time - self.recent_actions[sensor_id]
                if time_since_action < 300.0:  # 5 minute cooldown
                    return
            
            # Mark as active
            with self.action_lock:
                if sensor_id in self.active_actions:
                    return
                self.active_actions.add(sensor_id)
            
            self.get_logger().warn(f'⚡ Taking action on sensor {sensor_id} - Issue: {issue_type}')
            
            # Record action time
            self.recent_actions[sensor_id] = current_time
            
            if issue_type == "LEAK":
                # Publish valve close
                self._publish_valve_close(sensor_id)
                
                # Non-blocking wait using timer
                def delayed_maintenance():
                    try:
                        self._publish_maintenance(sensor_id, issue_type)
                    finally:
                        with self.action_lock:
                            self.active_actions.discard(sensor_id)
                
                threading.Timer(30.0, delayed_maintenance).start()
                
            elif issue_type == "BLOCKAGE":
                # Immediate maintenance (non-blocking)
                self._publish_maintenance(sensor_id, issue_type)
                with self.action_lock:
                    self.active_actions.discard(sensor_id)
            
            self.get_logger().info(f'Action scheduled for sensor {sensor_id}')
            
        except Exception as e:
            self.get_logger().error(f'Action error for sensor {sensor_id}: {e}')
            with self.action_lock:
                if sensor_id in self.active_actions:
                    self.active_actions.discard(sensor_id)
    
    def _publish_valve_close(self, sensor_id: int):
        """Non-blocking valve close"""
        try:
            msg = String()
            msg.data = f'{sensor_id}:0'
            self.valve_pub.publish(msg)
            self.get_logger().info(f' Published valve close for sensor {sensor_id}')
        except Exception as e:
            self.get_logger().error(f'Valve publish error: {e}')
    
    def _publish_maintenance(self, sensor_id: int, issue_type: str):
        """Non-blocking maintenance request"""
        try:
            msg = String()
            maintenance_data = {
                "sensor_id": sensor_id,
                "action": "reset_sensor_to_initial",
                "issue_type": issue_type
            }
            msg.data = json.dumps(maintenance_data)
            self.maintenance_pub.publish(msg)
            self.get_logger().info(f' Published maintenance for sensor {sensor_id}')
            
            # TCP broadcast
            self.tcp_broadcast_queue.put({
                'sensor_id': sensor_id,
                'maintenance': 1,
                'issue_type': issue_type,
                'timestamp': datetime.now().isoformat()
            })
            
        except Exception as e:
            self.get_logger().error(f'Maintenance publish error: {e}')
    
    def _tcp_server_worker(self):
        """TCP server worker"""
        try:
            server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind(('0.0.0.0', 9999))
            server.listen(100)
            server.settimeout(0.1)  # 100ms timeout
            
            while rclpy.ok():
                try:
                    client, address = server.accept()
                    with self.tcp_lock:
                        self.tcp_clients.append(client)
                    self.get_logger().info(f' TCP client connected: {address}')
                except socket.timeout:
                    continue
                except Exception as e:
                    self.get_logger().error(f'TCP accept error: {e}')
                    time.sleep(1)
                    
        except Exception as e:
            self.get_logger().error(f'TCP server error: {e}')
    
    def _tcp_broadcast_worker(self):
        """TCP broadcast worker"""
        try:
            data = self.tcp_broadcast_queue.get_nowait()
            message = json.dumps(data) + '\n'
            
            disconnected = []
            with self.tcp_lock:
                for client in self.tcp_clients:
                    try:
                        client.sendall(message.encode())
                    except:
                        disconnected.append(client)
                
                # Cleanup
                for client in disconnected:
                    try:
                        client.close()
                    except:
                        pass
                    self.tcp_clients.remove(client)
            
            self.tcp_broadcast_queue.task_done()
            
        except Empty:
            pass
        except Exception as e:
            self.get_logger().error(f'TCP broadcast error: {e}')
    
    def _log_writer_worker(self):
        """Log writer worker - simplified"""
        try:
            # Just drain the log queue
            while True:
                entry = self.log_queue.get_nowait()
                # Optional: write to file or database here
                self.log_queue.task_done()
        except Empty:
            pass
        except Exception as e:
            self.get_logger().error(f'Log writer error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = UltraFastPipeMonitorNode()
    
    # Use multi-threaded executor for ROS
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        # Shutdown thread pools
        node.classification_pool.shutdown(wait=True)
        node.action_pool.shutdown(wait=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()