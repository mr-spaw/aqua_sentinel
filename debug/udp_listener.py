#!/usr/bin/env python3
"""
Optimized UDP Listener - Single CSV file with all data
Checks for missing fields and handles them gracefully
"""

import socket
import json
import csv
import time
import threading
from datetime import datetime
from pathlib import Path
import signal
import sys
import matplotlib.pyplot as plt
from collections import defaultdict, deque
import numpy as np

class RealTimePlotter:
    """Real-time plotting for simulation data"""
    
    def __init__(self, max_points=500):
        self.max_points = max_points
        
        # Data buffers
        self.timestamps = deque(maxlen=max_points)
        self.reservoir_level = deque(maxlen=max_points)
        self.sewage_level = deque(maxlen=max_points)
        self.pressure = deque(maxlen=max_points)
        self.flow = deque(maxlen=max_points)
        self.valve_state = deque(maxlen=max_points)
        
        # Figure setup
        plt.ion()
        self.fig, self.axes = plt.subplots(2, 2, figsize=(12, 8))
        self.fig.suptitle('Water Distribution System - Real-time Monitoring', fontsize=14)
        
        # Subplot titles
        self.axes[0, 0].set_title('Reservoir Levels')
        self.axes[0, 1].set_title('Zone Pressure')
        self.axes[1, 0].set_title('Water Flow')
        self.axes[1, 1].set_title('Valve State')
        
        # Formatting
        for ax in self.axes.flat:
            ax.grid(True, alpha=0.3)
            ax.set_xlabel('Time (s)')
        
        self.last_update = time.time()
        
    def update_data(self, data):
        """Update data buffers with new data"""
        current_time = time.time()
        self.timestamps.append(current_time)
        
        # Extract data with safe defaults
        self.reservoir_level.append(data.get("reservoir_level_pct", 0))
        self.sewage_level.append(data.get("sewage_level_pct", 0))
        
        # Extract pressure from first zone if available
        pressure_val = 0
        if "zones" in data and len(data["zones"]) > 0:
            pressure_val = data["zones"][0].get("avg_pressure", 0)
        self.pressure.append(pressure_val)
        
        # Extract flow from first zone if available
        flow_val = 0
        if "zones" in data and len(data["zones"]) > 0:
            flow_val = data["zones"][0].get("flow", 0)
        self.flow.append(flow_val)
        
        # Extract average valve state
        valve_val = 0
        if "valve_states" in data and len(data["valve_states"]) > 0:
            valve_val = np.mean(data["valve_states"])
        self.valve_state.append(valve_val)
    
    def update_plot(self):
        """Update plot with current data"""
        if len(self.timestamps) < 2:
            return
        
        # Convert timestamps to relative time
        rel_times = [t - self.timestamps[0] for t in self.timestamps]
        
        # 1. Reservoir Levels
        self.axes[0, 0].clear()
        if self.reservoir_level:
            self.axes[0, 0].plot(rel_times, self.reservoir_level, 'b-', linewidth=2, label='Fresh')
        if self.sewage_level:
            self.axes[0, 0].plot(rel_times, self.sewage_level, 'r-', linewidth=2, label='Sewage')
        self.axes[0, 0].set_title('Reservoir Levels')
        self.axes[0, 0].set_ylabel('Level (%)')
        self.axes[0, 0].grid(True, alpha=0.3)
        self.axes[0, 0].legend()
        
        # 2. Pressure
        self.axes[0, 1].clear()
        if self.pressure:
            self.axes[0, 1].plot(rel_times, self.pressure, 'g-', linewidth=2)
        self.axes[0, 1].set_title('Zone Pressure')
        self.axes[0, 1].set_ylabel('Pressure (kPa)')
        self.axes[0, 1].grid(True, alpha=0.3)
        
        # 3. Flow
        self.axes[1, 0].clear()
        if self.flow:
            self.axes[1, 0].plot(rel_times, self.flow, 'c-', linewidth=2)
        self.axes[1, 0].set_title('Water Flow')
        self.axes[1, 0].set_ylabel('Flow (m³/s)')
        self.axes[1, 0].grid(True, alpha=0.3)
        
        # 4. Valve State
        self.axes[1, 1].clear()
        if self.valve_state:
            self.axes[1, 1].plot(rel_times, self.valve_state, 'm-', linewidth=2)
        self.axes[1, 1].set_title('Average Valve State')
        self.axes[1, 1].set_ylabel('Valve (%)')
        self.axes[1, 1].grid(True, alpha=0.3)
        
        # Set x-axis labels
        for ax in self.axes.flat:
            ax.set_xlabel('Time (s)')
        
        self.fig.tight_layout()
        plt.draw()
        plt.pause(0.01)
    
    def show(self):
        """Show the plot window"""
        try:
            plt.show(block=True)
        except KeyboardInterrupt:
            plt.close()


class OptimizedUDPListener:
    def __init__(self, port=8888, output_dir="simulation_data", enable_plotting=False):
        self.port = port
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)
        
        # Create UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.settimeout(0.1)
        self.sock.bind(('', port))
        
        # Statistics
        self.message_count = 0
        self.error_count = 0
        self.last_stat_print = time.time()
        self.start_time = None
        
        # Single CSV file for all data
        self.csv_file = None
        self.csv_writer = None
        self.csv_row_count = 0
        
        # Buffer for efficient writing
        self.data_buffer = []
        self.buffer_max_size = 100  # Write to CSV every 100 messages
        
        # Control flags
        self.running = False
        self.thread = None
        
        # Real-time plotting
        self.enable_plotting = enable_plotting
        self.plotter = None
        self.plot_thread = None
        if enable_plotting:
            self.plotter = RealTimePlotter()
            print("✓ Real-time plotting enabled")
        
        # Track data structure for debugging
        self.first_message = True
        self.field_warnings = defaultdict(int)
        
        print(f"UDP Listener initialized on port {port}")
        print(f"Output: {output_dir}/simulation_data.csv")
        print(f"Plotting: {'Enabled' if enable_plotting else 'Disabled'}")
    
    def _create_csv_writer(self):
        """Create single CSV file with all data"""
        if self.csv_file is None:
            # Create single CSV file
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = self.output_dir / f"simulation_data_{timestamp}.csv"
            
            self.csv_file = open(filename, 'w', newline='')
            
            # Initialize with comprehensive field list
            print("Creating CSV with comprehensive field list...")
            
            # Create a comprehensive field list
            all_fields = [
                "timestamp", "source_ip", "message_id",
                "reservoir_level_pct", "reservoir_trend",
                "pump_capacity_available", "supply_margin",
                "time_of_day", "day_type",
                "non_revenue_water", "supply_efficiency",
                "pressure_compliance", "service_continuity",
                "response_time",
                "sewage_level_pct", "sewage_status", "sewage_needs_treatment",
                "sewage_time_since_treatment", "last_treatment_event_id",
                "time_since_last_treatment", "time_since_last_recharge",
                "zone0_pressure", "zone0_flow", "zone0_leak_flag", "zone0_pressure_violation",
                "avg_valve_state", "active_valves_count",
                "peak_flow_rate", "avg_flow_rate", "flow_variance",
                "zone_count", "sensor_count"
            ]
            
            # Add hourly usage fields
            for hour in range(24):
                all_fields.append(f"hour_{hour}_usage")
            
            # Initialize CSV writer with all fields
            self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=all_fields)
            self.csv_writer.writeheader()
            self.csv_file.flush()
            
            print(f"✓ CSV file created: {filename}")
            print(f"✓ Using {len(all_fields)} fields in CSV")
        
        return self.csv_file
    
    def _determine_fields_from_data(self, data):
        """Determine CSV fields from the data structure"""
        fields = ["timestamp", "source_ip", "message_id"]
        
        # Basic fields - always include these
        basic_fields = [
            "reservoir_level_pct", "reservoir_trend",
            "pump_capacity_available", "supply_margin",
            "time_of_day", "day_type",
            "non_revenue_water", "supply_efficiency",
            "pressure_compliance", "service_continuity",
            "response_time"
        ]
        fields.extend(basic_fields)
        
        # Sewage data fields - always include these
        sewage_fields = [
            "sewage_level_pct", "sewage_status", "sewage_needs_treatment",
            "sewage_time_since_treatment", "last_treatment_event_id",
            "time_since_last_treatment", "time_since_last_recharge"
        ]
        fields.extend(sewage_fields)
        
        # Zone summary fields - always include
        zone_fields = [
            "zone0_pressure", "zone0_flow", "zone0_leak_flag",
            "zone0_pressure_violation"
        ]
        fields.extend(zone_fields)
        
        # Valve summary fields - always include
        valve_fields = ["avg_valve_state", "active_valves_count"]
        fields.extend(valve_fields)
        
        # System metrics - always include
        metric_fields = ["peak_flow_rate", "avg_flow_rate", "flow_variance"]
        fields.extend(metric_fields)
        
        # Counts - always include
        fields.extend(["zone_count", "sensor_count"])
        
        # Hourly usage fields - add for all 24 hours
        for hour in range(24):
            fields.append(f"hour_{hour}_usage")
        
        # Debug: Check if fields exist in data
        if self.first_message:
            print("\n=== FIELD CHECK ===")
            for field in basic_fields + sewage_fields + zone_fields + valve_fields + metric_fields:
                if field not in data:
                    print(f"⚠ Warning: '{field}' not in first message data")
            print("=" * 40)
        
        return fields
    
    def _create_csv_row(self, data, timestamp, addr, message_id):
        """Create a single CSV row from data"""
        row = {
            "timestamp": timestamp,
            "source_ip": addr[0],
            "message_id": message_id
        }
        
        # Basic reservoir data with defaults
        basic_fields = [
            "reservoir_level_pct", "reservoir_trend",
            "pump_capacity_available", "supply_margin",
            "time_of_day", "day_type",
            "non_revenue_water", "supply_efficiency",
            "pressure_compliance", "service_continuity",
            "response_time"
        ]
        
        for field in basic_fields:
            row[field] = data.get(field, 0)
        
        # Sewage data with defaults
        sewage_fields = [
            "sewage_level_pct", "sewage_status", "sewage_needs_treatment",
            "sewage_time_since_treatment", "last_treatment_event_id",
            "time_since_last_treatment", "time_since_last_recharge"
        ]
        
        for field in sewage_fields:
            row[field] = data.get(field, 0)
        
        # Zone summary (from first zone)
        if "zones" in data and len(data["zones"]) > 0:
            zone0 = data["zones"][0]
            row["zone0_pressure"] = zone0.get("avg_pressure", 0)
            row["zone0_flow"] = zone0.get("flow", 0)
            row["zone0_leak_flag"] = zone0.get("leak_flag", False)
            row["zone0_pressure_violation"] = zone0.get("pressure_violation", False)
        else:
            row["zone0_pressure"] = 0
            row["zone0_flow"] = 0
            row["zone0_leak_flag"] = False
            row["zone0_pressure_violation"] = False
        
        # Valve summary
        if "valve_states" in data and len(data["valve_states"]) > 0:
            valve_states = data["valve_states"]
            row["avg_valve_state"] = np.mean(valve_states)
            row["active_valves_count"] = sum(1 for v in valve_states if v > 0)
        else:
            row["avg_valve_state"] = 0
            row["active_valves_count"] = 0
        
        # System metrics
        metric_fields = ["peak_flow_rate", "avg_flow_rate", "flow_variance"]
        for field in metric_fields:
            row[field] = data.get(field, 0)
        
        # Counts
        row["zone_count"] = data.get("zone_count", 0)
        row["sensor_count"] = data.get("sensor_count", 0)
        
        # Hourly usage - initialize all hours to 0
        for hour in range(24):
            row[f"hour_{hour}_usage"] = 0
        
        # Fill in current hour if data available
        if "hourly_usage" in data and isinstance(data["hourly_usage"], list):
            for hour, usage in enumerate(data["hourly_usage"]):
                if hour < 24:
                    row[f"hour_{hour}_usage"] = usage
        
        return row
    
    def _write_csv_buffer(self):
        """Write buffered data to CSV"""
        if not self.data_buffer or self.csv_writer is None:
            return
        
        try:
            for row in self.data_buffer:
                # Ensure all fields exist in row
                for field in self.csv_writer.fieldnames:
                    if field not in row:
                        row[field] = 0  # Default value for missing fields
                
                self.csv_writer.writerow(row)
            
            self.csv_file.flush()
            self.csv_row_count += len(self.data_buffer)
            self.data_buffer.clear()
            
            if self.csv_row_count % 500 == 0:
                print(f"✓ Written {self.csv_row_count} rows to CSV")
                
        except Exception as e:
            print(f"❌ Error writing to CSV: {e}")
            print(f"  Row keys: {list(row.keys()) if 'row' in locals() else 'No row'}")
            print(f"  CSV fields: {self.csv_writer.fieldnames if self.csv_writer else 'No writer'}")
    
    def _process_message(self, data, timestamp, addr):
        """Process a single message"""
        try:
            # Create CSV row
            row = self._create_csv_row(data, timestamp, addr, self.message_count)
            
            # Add to buffer
            self.data_buffer.append(row)
            
            # Write buffer if full
            if len(self.data_buffer) >= self.buffer_max_size:
                self._write_csv_buffer()
            
            # Update plotter
            if self.enable_plotting and self.plotter:
                self.plotter.update_data(data)
                
                # Update plot periodically
                current_time = time.time()
                if current_time - self.plotter.last_update > 1.0:  # Update every 1 second
                    self.plotter.update_plot()
                    self.plotter.last_update = current_time
            
            # Debug first message
            if self.first_message:
                self.first_message = False
                print("\n=== FIRST MESSAGE RECEIVED ===")
                print(f"Keys in data: {list(data.keys())}")
                
                if "zones" in data:
                    print(f"Number of zones: {len(data['zones'])}")
                    if data['zones']:
                        print(f"First zone keys: {list(data['zones'][0].keys())}")
                
                if "valve_states" in data:
                    print(f"Number of valve states: {len(data['valve_states'])}")
                    print(f"Sample valve states: {data['valve_states'][:5]}")
                
                print("=" * 40 + "\n")
            
            return True
            
        except Exception as e:
            print(f"❌ Error processing message: {e}")
            import traceback
            traceback.print_exc()
            self.error_count += 1
            return False
    
    def _listen_loop(self):
        """Main listening loop"""
        print(f"\n🎧 Starting UDP listener on port {self.port}...")
        print("   Press Ctrl+C to stop\n")
        
        self.start_time = time.time()
        
        # Initialize CSV writer immediately with all fields
        self._create_csv_writer()
        
        while self.running:
            try:
                # Try to receive data
                try:
                    data, addr = self.sock.recvfrom(65535)
                except socket.timeout:
                    continue
                
                # Process received data
                try:
                    json_str = data.decode('utf-8')
                    json_data = json.loads(json_str)
                    
                    self.message_count += 1
                    timestamp = datetime.now().isoformat()
                    
                    # Process the message
                    success = self._process_message(json_data, timestamp, addr)
                    
                    # Print statistics every 10 seconds
                    current_time = time.time()
                    if current_time - self.last_stat_print > 10:
                        self._print_statistics()
                        self.last_stat_print = current_time
                    
                except (UnicodeDecodeError, json.JSONDecodeError) as e:
                    print(f"⚠ Invalid data from {addr}: {e}")
                    self.error_count += 1
                    
            except Exception as e:
                print(f"❌ Error in listen loop: {e}")
                self.error_count += 1
    
    def _print_statistics(self):
        """Print current statistics"""
        if self.start_time is None:
            return
            
        elapsed = time.time() - self.start_time
        msg_rate = self.message_count / max(1, elapsed)
        
        print(f"\n{'='*50}")
        print(f"📊 STATISTICS (Last 10 seconds)")
        print(f"{'='*50}")
        print(f"  Total messages: {self.message_count}")
        print(f"  Message rate: {msg_rate:.1f} msg/sec")
        print(f"  Errors: {self.error_count}")
        print(f"  CSV rows: {self.csv_row_count}")
        print(f"  Buffer size: {len(self.data_buffer)}")
        
        # Print field warnings if any
        if self.field_warnings:
            print(f"  Missing fields: {dict(self.field_warnings)}")
        
        print(f"{'='*50}")
    
    def start(self):
        """Start listening in a background thread"""
        if self.running:
            print("⚠ Listener is already running")
            return
        
        self.running = True
        self.thread = threading.Thread(target=self._listen_loop, daemon=True)
        self.thread.start()
        
        # Start plotter thread if enabled
        if self.enable_plotting and self.plotter:
            self.plot_thread = threading.Thread(target=self.plotter.show, daemon=True)
            self.plot_thread.start()
        
        print("✓ UDP listener started")
    
    def stop(self):
        """Stop listening and clean up"""
        print("\n🛑 Stopping UDP listener...")
        self.running = False
        
        # Write any remaining buffered data
        if self.data_buffer:
            print(f"  Writing {len(self.data_buffer)} buffered rows...")
            self._write_csv_buffer()
        
        # Wait for thread to finish
        if self.thread:
            self.thread.join(timeout=2.0)
        
        # Close CSV file
        if self.csv_file:
            self.csv_file.close()
            print(f"  CSV file closed ({self.csv_row_count} total rows)")
        
        # Close socket
        self.sock.close()
        
        # Print final statistics
        self._print_statistics()
        print("✓ UDP listener stopped")
    
    def save_current_data(self):
        """Force save current data"""
        print("\n💾 Saving current data...")
        self._write_csv_buffer()


def signal_handler(sig, frame):
    """Handle Ctrl+C signal"""
    print("\n\n🛑 Ctrl+C pressed. Stopping listener...")
    global listener
    if 'listener' in globals():
        listener.stop()
    sys.exit(0)


def main():
    """Main function"""
    import argparse
    
    parser = argparse.ArgumentParser(
        description="Optimized UDP Listener for Water Distribution Simulation",
        epilog="Example: python3 udp_listener.py --plot"
    )
    
    parser.add_argument("-p", "--port", type=int, default=8888,
                       help="UDP port (default: 8888)")
    parser.add_argument("-o", "--output", type=str, default="simulation_data",
                       help="Output directory (default: simulation_data)")
    parser.add_argument("--plot", action="store_true",
                       help="Enable real-time plotting")
    
    args = parser.parse_args()
    
    # Create listener
    global listener
    listener = OptimizedUDPListener(
        port=args.port,
        output_dir=args.output,
        enable_plotting=args.plot
    )
    
    # Set up signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    # Start listening
    try:
        listener.start()
        
        # Keep main thread alive
        while listener.running:
            time.sleep(1)
            
    except KeyboardInterrupt:
        pass
    finally:
        listener.stop()


if __name__ == "__main__":
    main()