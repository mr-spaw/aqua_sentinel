import socket
import struct
import time
import json
import zlib
import binascii
from datetime import datetime
import os
import threading
from collections import deque

class RawUDPMonitor:
    def __init__(self, port=8888, save_dir="captured_data", save_mode="individual"):
        self.port = port
        self.packet_count = 0
        self.byte_count = 0
        self.start_time = time.time()
        
        # Statistics
        self.packet_sizes = []
        self.fragmented_packets = 0
        self.complete_packets = 0
        self.corrupted_packets = 0
        self.saved_packets = 0
        
        # Data storage
        self.save_dir = save_dir
        self.save_mode = save_mode  # "individual" or "single"
        self.data_buffer = []
        
        # For reassembling fragmented packets
        self.fragment_buffer = {}
        
        # Create save directory
        self.create_save_directory()
        
        print(f"🔬 RAW UDP MONITOR - Port {port}")
        print(f"💾 Saving data to: {os.path.abspath(save_dir)}")
        print(f"📁 Save mode: {save_mode}")
        print("=" * 80)
        
    def create_save_directory(self):
        """Create directory for saving data"""
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            print(f"✓ Created directory: {self.save_dir}")
            
        # Create metadata file
        metadata = {
            "start_time": datetime.now().isoformat(),
            "port": self.port,
            "save_mode": self.save_mode,
            "description": "Water distribution network simulation data"
        }
        
        metadata_file = os.path.join(self.save_dir, "metadata.json")
        with open(metadata_file, 'w') as f:
            json.dump(metadata, f, indent=2)
            
        # If single file mode, create the main data file
        if self.save_mode == "single":
            self.single_file_path = os.path.join(self.save_dir, "captured_data.json")
            # Start with empty array
            with open(self.single_file_path, 'w') as f:
                json.dump([], f)
            
    def capture_all_traffic(self):
        """Capture ALL UDP traffic on port 8888"""
        # Create raw socket (requires root)
        try:
            sock = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.ntohs(0x0003))
            print("✓ Raw socket created (requires root privileges)")
        except PermissionError:
            print("✗ ERROR: Need root privileges for raw socket!")
            print("  Run with: sudo python3 script.py")
            return
            
        print("Listening for ALL UDP packets...")
        print("Press Ctrl+C to stop\n")
        
        try:
            while True:
                # Receive packet
                packet, addr = sock.recvfrom(65535)
                self.packet_count += 1
                self.byte_count += len(packet)
                
                # Parse packet
                self.parse_raw_packet(packet)
                    
                # Print summary every 20 packets
                if self.packet_count % 20 == 0:
                    self.print_summary()
                    
        except KeyboardInterrupt:
            print("\n\n🛑 Capture stopped by user")
            self.save_remaining_data()  # Final save
            self.print_final_summary()
                
    def save_complete_json(self, json_data):
        """Save complete JSON data to file"""
        if not json_data:
            return
            
        self.saved_packets += 1
        
        if self.save_mode == "individual":
            # Save each packet as individual file
            if 'header' in json_data:
                # Use timestamp and step count for filename
                ts = json_data['header'].get('timestamp', int(time.time()))
                step = json_data['header'].get('step_count', self.saved_packets)
                filename = f"packet_{ts}_{step:08d}.json"
            else:
                filename = f"packet_{self.saved_packets:06d}.json"
                
            filepath = os.path.join(self.save_dir, filename)
            
            try:
                with open(filepath, 'w') as f:
                    json.dump(json_data, f, indent=2)
                print(f"   💾 Saved to: {filename}")
            except Exception as e:
                print(f"✗ Error saving file: {e}")
                
        elif self.save_mode == "single":
            # Append to single file
            self.data_buffer.append(json_data)
            
            # Save every 10 packets to avoid memory issues
            if len(self.data_buffer) >= 10:
                self.save_to_single_file()
                
    def save_to_single_file(self):
        """Save buffered data to single file"""
        if not self.data_buffer:
            return
            
        try:
            # Read existing data
            existing_data = []
            if os.path.exists(self.single_file_path):
                with open(self.single_file_path, 'r') as f:
                    existing_data = json.load(f)
                    
            # Append new data
            existing_data.extend(self.data_buffer)
            
            # Save back to file
            with open(self.single_file_path, 'w') as f:
                json.dump(existing_data, f, indent=2)
                
            print(f"   💾 Appended {len(self.data_buffer)} packets to single file")
            self.data_buffer.clear()
            
        except Exception as e:
            print(f"✗ Error saving to single file: {e}")
            
    def save_remaining_data(self):
        """Save any remaining data in buffer"""
        if self.save_mode == "single" and self.data_buffer:
            self.save_to_single_file()
            
        # Create summary file
        self.create_summary_file()
        
    def parse_raw_packet(self, packet):
        """Parse raw Ethernet packet"""
        try:
            # Ethernet header (14 bytes): dst(6) + src(6) + type(2)
            eth_length = 14
            eth_header = packet[:eth_length]
            
            # Parse Ethernet header
            dst_mac = binascii.hexlify(eth_header[0:6]).decode('utf-8')
            src_mac = binascii.hexlify(eth_header[6:12]).decode('utf-8')
            eth_protocol = struct.unpack('!H', eth_header[12:14])[0]
            
            # Only process IP packets (0x0800)
            if eth_protocol != 8:
                return
                
            # Parse IP header (20 bytes)
            ip_header = packet[eth_length:eth_length+20]
            iph = struct.unpack('!BBHHHBBH4s4s', ip_header)
            
            version_ihl = iph[0]
            version = version_ihl >> 4
            ihl = version_ihl & 0xF
            ip_header_length = ihl * 4
            
            ttl = iph[5]
            protocol = iph[6]
            src_ip = socket.inet_ntoa(iph[8])
            dst_ip = socket.inet_ntoa(iph[9])
            
            # Only process UDP (protocol 17)
            if protocol != 17:
                return
                
            # Get UDP header (8 bytes)
            udp_header_start = eth_length + ip_header_length
            udp_header = packet[udp_header_start:udp_header_start+8]
            udph = struct.unpack('!HHHH', udp_header)
            
            src_port = udph[0]
            dst_port = udph[1]
            udp_length = udph[2]
            
            # Only process our port
            if dst_port != self.port:
                return
                
            # Get UDP data
            data_start = udp_header_start + 8
            data = packet[data_start:data_start + (udp_length - 8)]
            
            # Check IP fragmentation flags
            flags_offset = iph[4]
            flags = (flags_offset >> 13) & 0x7
            fragment_offset = flags_offset & 0x1FFF
            more_fragments = (flags & 0x2) >> 1
            
            is_fragmented = fragment_offset > 0 or more_fragments
            
            # Display packet info
            self.display_packet_info(
                self.packet_count, src_ip, src_port, dst_ip, dst_port,
                len(data), fragment_offset, more_fragments,
                src_mac, dst_mac
            )
            
            # Try to parse as JSON
            json_data = self.analyze_data(data, fragment_offset, more_fragments)
            
            # Store size
            self.packet_sizes.append(len(data))
            
        except Exception as e:
            self.corrupted_packets += 1
            print(f"✗ Error parsing packet: {e}")
            
    def analyze_data(self, data, fragment_offset, more_fragments):
        """Analyze the UDP payload"""
        if fragment_offset > 0 or more_fragments:
            self.fragmented_packets += 1
            print(f"   ⚠ FRAGMENTED: Offset={fragment_offset}, More={more_fragments}")
            return None
            
        # Try multiple decoding methods
        decoded = False
        json_data = None
        
        # Method 1: Try as UTF-8 JSON
        try:
            text = data.decode('utf-8')
            if text.strip().startswith('{') and text.strip().endswith('}'):
                json_data = json.loads(text)
                self.complete_packets += 1
                print(f"   ✓ COMPLETE JSON: {len(text):,} characters")
                self.print_json_summary(json_data)
                decoded = True
                
                # Save the complete JSON
                self.save_complete_json(json_data)
                return json_data
        except:
            pass
            
        # Method 2: Try as compressed JSON
        if not decoded and len(data) > 10:
            try:
                # Check for GZIP header
                if data[:2] == b'\x1f\x8b':
                    decompressed = zlib.decompress(data, 16 + zlib.MAX_WBITS)
                    text = decompressed.decode('utf-8')
                    json_data = json.loads(text)
                    self.complete_packets += 1
                    print(f"   ✓ GZIP JSON: {len(data):,} → {len(text):,} bytes")
                    self.print_json_summary(json_data)
                    decoded = True
                    
                    # Save the complete JSON
                    self.save_complete_json(json_data)
                    return json_data
            except:
                pass
                
        # Method 3: Try as partial JSON
        if not decoded:
            try:
                text = data.decode('utf-8', errors='ignore')
                # Look for JSON-like content
                if '{' in text and '}' in text:
                    start = text.find('{')
                    end = text.rfind('}') + 1
                    if start < end:
                        json_str = text[start:end]
                        try:
                            json_data = json.loads(json_str)
                            print(f"   ✓ PARTIAL JSON: {len(json_str):,} chars (in {len(data):,} bytes)")
                            decoded = True
                            
                            # Save the partial JSON
                            self.save_complete_json(json_data)
                            return json_data
                        except:
                            pass
            except:
                pass
                
        if not decoded:
            print(f"   📦 RAW DATA: {len(data):,} bytes")
            # Show first 100 bytes in hex
            hex_dump = binascii.hexlify(data[:100]).decode('utf-8')
            print(f"      Hex: {hex_dump[:80]}...")
            
        return None
            
    def print_json_summary(self, json_data):
        """Print summary of JSON data"""
        try:
            if 'header' in json_data:
                ts = json_data['header'].get('timestamp', 0)
                sim_time = json_data['header'].get('sim_time_sec', 0)
                step = json_data['header'].get('step_count', 0)
                print(f"      Timestamp: {datetime.fromtimestamp(ts).strftime('%H:%M:%S')}")
                print(f"      Sim Time: {sim_time:.1f}s, Step: {step:,}")
                
            if 'global_state' in json_data:
                zones = len(json_data['global_state'].get('zones', []))
                reservoir = json_data['global_state'].get('reservoir_level_pct', 0)
                print(f"      Zones: {zones}, Reservoir: {reservoir:.1f}%")
                
            if 'building_water_usage' in json_data:
                buildings = len(json_data['building_water_usage'].get('buildings', []))
                demand = json_data['building_water_usage'].get('current_demand_m3s', 0)
                print(f"      Buildings: {buildings}, Demand: {demand:.3f} m³/s")
                
        except:
            pass
            
    def display_packet_info(self, seq, src_ip, src_port, dst_ip, dst_port, 
                           size, fragment_offset, more_fragments, src_mac, dst_mac):
        """Display formatted packet information"""
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        
        print(f"\n[{timestamp}] Packet #{seq}")
        print(f"   From: {src_ip}:{src_port} ({src_mac[:8]}...)")
        print(f"   To:   {dst_ip}:{dst_port} ({dst_mac[:8]}...)")
        print(f"   Size: {size:,} bytes")
        
        if fragment_offset > 0 or more_fragments:
            print(f"   ⚠ Fragmented: Offset={fragment_offset}, More={more_fragments}")
            
    def print_summary(self):
        """Print periodic summary"""
        elapsed = time.time() - self.start_time
        
        if elapsed > 0:
            rate = self.packet_count / elapsed
            bandwidth = self.byte_count / elapsed / 1024  # KB/s
            
            print("\n" + "=" * 80)
            print("📊 LIVE STATISTICS:")
            print(f"   Packets: {self.packet_count:,} ({rate:.1f}/sec)")
            print(f"   Data Rate: {bandwidth:.1f} KB/s")
            print(f"   Total Data: {self.byte_count/1024/1024:.2f} MB")
            print(f"   Saved Packets: {self.saved_packets:,}")
            
            if self.packet_sizes:
                avg_size = sum(self.packet_sizes) / len(self.packet_sizes)
                max_size = max(self.packet_sizes)
                min_size = min(self.packet_sizes)
                print(f"   Packet Sizes: Avg={avg_size:.0f}, Min={min_size}, Max={max_size}")
                
            print(f"   Complete JSON Packets: {self.complete_packets}")
            print(f"   Fragmented Packets: {self.fragmented_packets}")
            print(f"   Corrupted Packets: {self.corrupted_packets}")
            print("=" * 80 + "\n")
            
    def create_summary_file(self):
        """Create a summary file with statistics"""
        summary = {
            "capture_summary": {
                "start_time": datetime.fromtimestamp(self.start_time).isoformat(),
                "end_time": datetime.now().isoformat(),
                "duration_seconds": time.time() - self.start_time,
                "total_packets": self.packet_count,
                "total_bytes": self.byte_count,
                "saved_packets": self.saved_packets,
                "complete_packets": self.complete_packets,
                "fragmented_packets": self.fragmented_packets,
                "corrupted_packets": self.corrupted_packets
            },
            "throughput": {
                "packets_per_second": self.packet_count / (time.time() - self.start_time) if (time.time() - self.start_time) > 0 else 0,
                "bytes_per_second": self.byte_count / (time.time() - self.start_time) if (time.time() - self.start_time) > 0 else 0,
                "megabytes_total": self.byte_count / (1024 * 1024)
            }
        }
        
        if self.packet_sizes:
            summary["packet_sizes"] = {
                "average": sum(self.packet_sizes) / len(self.packet_sizes),
                "minimum": min(self.packet_sizes),
                "maximum": max(self.packet_sizes),
                "total_samples": len(self.packet_sizes)
            }
            
        summary_file = os.path.join(self.save_dir, "summary.json")
        with open(summary_file, 'w') as f:
            json.dump(summary, f, indent=2)
        print(f"📋 Summary saved to: {summary_file}")
            
    def print_final_summary(self):
        """Print final statistics"""
        elapsed = time.time() - self.start_time
        
        print("\n" + "=" * 80)
        print("🎯 FINAL CAPTURE REPORT")
        print("=" * 80)
        print(f"Capture Duration: {elapsed:.1f} seconds")
        print(f"Total Packets: {self.packet_count:,}")
        print(f"Total Bytes: {self.byte_count:,} ({self.byte_count/1024/1024:.2f} MB)")
        print(f"Saved JSON Packets: {self.saved_packets:,}")
        
        if elapsed > 0:
            print(f"Average Rate: {self.packet_count/elapsed:.1f} packets/sec")
            print(f"Average Bandwidth: {self.byte_count/elapsed/1024:.1f} KB/s")
            
        print(f"\nPacket Analysis:")
        print(f"  Complete JSON Packets: {self.complete_packets}")
        print(f"  Fragmented Packets: {self.fragmented_packets}")
        print(f"  Corrupted Packets: {self.corrupted_packets}")
        
        if self.packet_sizes:
            print(f"\nPacket Size Distribution:")
            sizes = {}
            for size in self.packet_sizes:
                bucket = (size // 100) * 100
                sizes[bucket] = sizes.get(bucket, 0) + 1
                
            for bucket in sorted(sizes.keys()):
                print(f"  {bucket:5,}-{bucket+99:<5,} bytes: {sizes[bucket]:4,} packets")
                
        print(f"\nData saved to: {os.path.abspath(self.save_dir)}")
        if self.save_mode == "individual":
            print(f"  Individual JSON files: {self.saved_packets} files")
        elif self.save_mode == "single":
            print(f"  Single JSON file: captured_data.json")
        print("=" * 80)

# Enhanced simple version with saving
def simple_udp_monitor():
    """Simple UDP monitor that works without root and saves data"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    
    # Bind to all interfaces
    sock.bind(('0.0.0.0', 8888))
    
    # Increase buffer size
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)  # 1MB
    
    # Ask for save mode
    print("\nChoose save mode:")
    print("1. Individual files (each packet as separate .json)")
    print("2. Single file (all packets in one .json array)")
    save_choice = input("Enter choice (1 or 2): ").strip()
    save_mode = "individual" if save_choice == "1" else "single"
    
    # Create save directory
    save_dir = "simple_capture"
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    
    packet_count = 0
    start_time = time.time()
    data_buffer = []
    saved_count = 0
    
    print(f"📡 Simple UDP Monitor (no root required)")
    print(f"💾 Saving data to: {os.path.abspath(save_dir)}")
    print(f"📁 Save mode: {save_mode}")
    print("=" * 80)
    print("Listening on UDP port 8888...")
    print("Press Ctrl+C to stop\n")
    
    # Single file path
    single_file_path = os.path.join(save_dir, "captured_data.json")
    if save_mode == "single":
        with open(single_file_path, 'w') as f:
            json.dump([], f)
    
    try:
        while True:
            data, addr = sock.recvfrom(65535)
            packet_count += 1
            
            current_time = time.time()
            timestamp = datetime.now()
            
            print(f"\n[{timestamp.strftime('%H:%M:%S.%f')[:-3]}] Packet #{packet_count} from {addr[0]}:{addr[1]}")
            print(f"   Size: {len(data):,} bytes")
            
            # Try to parse as JSON
            json_data = None
            try:
                text = data.decode('utf-8')
                if text.strip().startswith('{') and text.strip().endswith('}'):
                    json_data = json.loads(text)
                    print(f"   ✓ COMPLETE JSON")
                    saved_count += 1
                    
                    # Save based on mode
                    if save_mode == "individual":
                        # Save as individual file
                        if 'header' in json_data:
                            ts = json_data['header'].get('timestamp', int(time.time()))
                            step = json_data['header'].get('step_count', saved_count)
                            filename = f"packet_{ts}_{step:08d}.json"
                        else:
                            filename = f"packet_{saved_count:06d}.json"
                            
                        filepath = os.path.join(save_dir, filename)
                        with open(filepath, 'w') as f:
                            json.dump(json_data, f, indent=2)
                        print(f"   💾 Saved to: {filename}")
                        
                    elif save_mode == "single":
                        # Add to buffer
                        data_buffer.append(json_data)
                        
                        # Save every 10 packets
                        if len(data_buffer) >= 10:
                            # Read existing data
                            existing_data = []
                            if os.path.exists(single_file_path):
                                with open(single_file_path, 'r') as f:
                                    existing_data = json.load(f)
                                    
                            # Append new data
                            existing_data.extend(data_buffer)
                            
                            # Save back to file
                            with open(single_file_path, 'w') as f:
                                json.dump(existing_data, f, indent=2)
                                
                            print(f"   💾 Appended {len(data_buffer)} packets to single file")
                            data_buffer.clear()
                            
                    # Show summary
                    if 'header' in json_data:
                        ts = json_data['header'].get('timestamp', 0)
                        sim_time = json_data['header'].get('sim_time_sec', 0)
                        step = json_data['header'].get('step_count', 0)
                        print(f"      Timestamp: {datetime.fromtimestamp(ts).strftime('%H:%M:%S')}")
                        print(f"      Sim Time: {sim_time:.1f}s, Step: {step:,}")
                        
                else:
                    print(f"   📦 RAW TEXT: {len(text):,} chars")
                    
            except json.JSONDecodeError:
                print(f"   📦 INVALID JSON: {len(data):,} bytes")
            except UnicodeDecodeError:
                print(f"   📦 BINARY DATA: {len(data):,} bytes")
            
            # Print stats every 10 packets
            if packet_count % 10 == 0:
                elapsed = current_time - start_time
                rate = packet_count / elapsed
                print(f"\n📊 {packet_count} packets, {rate:.1f}/sec, {saved_count} saved")
                
    except KeyboardInterrupt:
        print(f"\n\n📊 Captured {packet_count} packets in {time.time()-start_time:.1f}s")
        print(f"💾 Saved {saved_count} JSON packets")
        
        # Save any remaining data
        if save_mode == "single" and data_buffer:
            try:
                existing_data = []
                if os.path.exists(single_file_path):
                    with open(single_file_path, 'r') as f:
                        existing_data = json.load(f)
                        
                existing_data.extend(data_buffer)
                
                with open(single_file_path, 'w') as f:
                    json.dump(existing_data, f, indent=2)
                    
                print(f"💾 Final save: Appended {len(data_buffer)} packets")
            except Exception as e:
                print(f"✗ Error saving final data: {e}")
        
    except Exception as e:
        print(f"\nError: {e}")

if __name__ == "__main__":
    print("=" * 80)
    print("🌊 WATER DISTRIBUTION NETWORK MONITOR")
    print("=" * 80)
    print("Choose monitoring mode:")
    print("1. RAW socket (requires root, shows EVERYTHING, saves JSON data)")
    print("2. Simple UDP (no root, may miss broadcast packets, saves JSON data)")
    
    choice = input("\nEnter choice (1 or 2): ").strip()
    
    if choice == "1":
        # Ask for save directory
        save_dir = input("Enter save directory [default: captured_data]: ").strip()
        if not save_dir:
            save_dir = "captured_data"
            
        print("\nChoose save mode:")
        print("1. Individual files (each packet as separate .json)")
        print("2. Single file (all packets in one .json array)")
        save_choice = input("Enter choice (1 or 2): ").strip()
        save_mode = "individual" if save_choice == "1" else "single"
            
        monitor = RawUDPMonitor(port=8888, save_dir=save_dir, save_mode=save_mode)
        monitor.capture_all_traffic()
    else:
        simple_udp_monitor()