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
    def __init__(self, port=8888, save_dir="captured_data", max_packets=1000):
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
        self.max_packets = max_packets
        self.data_buffer = deque(maxlen=max_packets)
        
        # File handling
        self.output_file = None
        self.packet_file = None
        self.last_save_time = time.time()
        self.save_interval = 5  # Save to disk every 5 seconds
        self.auto_save = True
        
        # For reassembling fragmented packets
        self.fragment_buffer = {}
        
        # Create save directory
        self.create_save_directory()
        
        print(f" RAW UDP MONITOR - Port {port}")
        print(f" Saving data to: {os.path.abspath(save_dir)}")
        print("=" * 80)
        
    def create_save_directory(self):
        """Create directory for saving data"""
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            print(f" Created directory: {self.save_dir}")
            
        # Create metadata file
        metadata = {
            "start_time": datetime.now().isoformat(),
            "port": self.port,
            "max_packets": self.max_packets
        }
        
        metadata_file = os.path.join(self.save_dir, "metadata.json")
        with open(metadata_file, 'w') as f:
            json.dump(metadata, f, indent=2)
            
        # Create packet log file
        packet_log = os.path.join(self.save_dir, "packets.log")
        self.packet_file = open(packet_log, 'a')
        
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
                
                # Auto-save to disk periodically
                if self.auto_save and (time.time() - self.last_save_time) > self.save_interval:
                    self.save_to_disk()
                    self.last_save_time = time.time()
                    
                # Print summary every 20 packets
                if self.packet_count % 20 == 0:
                    self.print_summary()
                    
        except KeyboardInterrupt:
            print("\n\n Capture stopped by user")
            self.save_to_disk()  # Final save
            self.print_final_summary()
        finally:
            if self.packet_file:
                self.packet_file.close()
                
    def save_packet_data(self, packet_info, json_data=None, raw_data=None):
        """Save packet data to buffer and optionally to disk"""
        packet_record = {
            "timestamp": datetime.now().isoformat(),
            "packet_number": self.packet_count,
            "source_ip": packet_info.get('src_ip'),
            "source_port": packet_info.get('src_port'),
            "dest_ip": packet_info.get('dst_ip'),
            "dest_port": packet_info.get('dst_port'),
            "size": packet_info.get('size'),
            "fragmented": packet_info.get('fragmented', False),
            "complete": json_data is not None
        }
        
        if json_data:
            packet_record["data"] = json_data
        elif raw_data:
            # Convert binary data to base64 for JSON storage
            packet_record["raw_data"] = binascii.b2a_base64(raw_data).decode('utf-8').strip()
            packet_record["raw_size"] = len(raw_data)
            
        # Add to buffer
        self.data_buffer.append(packet_record)
        
        # Log to packet file
        if self.packet_file:
            log_entry = {
                "time": datetime.now().strftime("%H:%M:%S.%f"),
                "seq": self.packet_count,
                "src": f"{packet_info.get('src_ip')}:{packet_info.get('src_port')}",
                "dst": f"{packet_info.get('dst_ip')}:{packet_info.get('dst_port')}",
                "size": packet_info.get('size'),
                "complete": json_data is not None
            }
            self.packet_file.write(json.dumps(log_entry) + "\n")
            
        self.saved_packets += 1
        
    def save_to_disk(self, filename=None):
        """Save buffered data to disk"""
        if not self.data_buffer:
            return
            
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"capture_{timestamp}.json"
            
        filepath = os.path.join(self.save_dir, filename)
        
        data_to_save = {
            "metadata": {
                "save_time": datetime.now().isoformat(),
                "total_packets": self.packet_count,
                "saved_packets": len(self.data_buffer),
                "duration": time.time() - self.start_time
            },
            "packets": list(self.data_buffer)
        }
        
        try:
            with open(filepath, 'w') as f:
                json.dump(data_to_save, f, indent=2, default=str)
            print(f" Saved {len(self.data_buffer)} packets to {filename}")
            
            # Also create a summary file
            self.create_summary_file()
            
        except Exception as e:
            print(f"Error saving to disk: {e}")
            
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
            
            # Create packet info dict
            packet_info = {
                'src_ip': src_ip,
                'src_port': src_port,
                'dst_ip': dst_ip,
                'dst_port': dst_port,
                'size': len(data),
                'fragmented': is_fragmented,
                'mac_src': src_mac,
                'mac_dst': dst_mac
            }
            
            # Try to parse as JSON
            json_data = self.analyze_data(data, fragment_offset, more_fragments)
            
            # Save packet data
            self.save_packet_data(packet_info, json_data, data if json_data is None else None)
            
            # Store size
            self.packet_sizes.append(len(data))
            
        except Exception as e:
            self.corrupted_packets += 1
            print(f"✗ Error parsing packet: {e}")
            
    def analyze_data(self, data, fragment_offset, more_fragments):
        """Analyze the UDP payload"""
        if fragment_offset > 0 or more_fragments:
            self.fragmented_packets += 1
            print(f"    FRAGMENTED: Offset={fragment_offset}, More={more_fragments}")
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
                print(f" COMPLETE JSON: {len(text):,} characters")
                self.print_json_summary(json_data)
                decoded = True
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
                            return json_data
                        except:
                            pass
            except:
                pass
                
        if not decoded:
            print(f" RAW DATA: {len(data):,} bytes")
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
                print(f"      Timestamp: {datetime.fromtimestamp(ts).strftime('%H:%M:%S')}")
                print(f"      Sim Time: {sim_time:.1f}s")
                
            if 'global_state' in json_data:
                zones = len(json_data['global_state'].get('zones', []))
                reservoir = json_data['global_state'].get('reservoir_level_pct', 0)
                print(f"      Zones: {zones}, Reservoir: {reservoir:.1f}%")
                
            if 'building_water_usage' in json_data:
                buildings = len(json_data['building_water_usage'].get('buildings', []))
                print(f"      Buildings: {buildings}")
                
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
            print(f"    Fragmented: Offset={fragment_offset}, More={more_fragments}")
            
    def print_summary(self):
        """Print periodic summary"""
        elapsed = time.time() - self.start_time
        
        if elapsed > 0:
            rate = self.packet_count / elapsed
            bandwidth = self.byte_count / elapsed / 1024  # KB/s
            
            print("\n" + "=" * 80)
            print("LIVE STATISTICS:")
            print(f"   Packets: {self.packet_count:,} ({rate:.1f}/sec)")
            print(f"   Data Rate: {bandwidth:.1f} KB/s")
            print(f"   Total Data: {self.byte_count/1024/1024:.2f} MB")
            print(f"   Saved Packets: {self.saved_packets}/{len(self.data_buffer)} in buffer")
            
            if self.packet_sizes:
                avg_size = sum(self.packet_sizes) / len(self.packet_sizes)
                max_size = max(self.packet_sizes)
                min_size = min(self.packet_sizes)
                print(f"   Packet Sizes: Avg={avg_size:.0f}, Min={min_size}, Max={max_size}")
                
            print(f"   Complete Packets: {self.complete_packets}")
            print(f"   Fragmented Packets: {self.fragmented_packets}")
            print(f"   Corrupted Packets: {self.corrupted_packets}")
            print("=" * 80 + "\n")
            
    def print_final_summary(self):
        """Print final statistics"""
        elapsed = time.time() - self.start_time
        
        print("\n" + "=" * 80)
        print(" FINAL CAPTURE REPORT")
        print("=" * 80)
        print(f"Capture Duration: {elapsed:.1f} seconds")
        print(f"Total Packets: {self.packet_count:,}")
        print(f"Total Bytes: {self.byte_count:,} ({self.byte_count/1024/1024:.2f} MB)")
        print(f"Saved Packets: {self.saved_packets:,}")
        
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
    
    # Create save directory
    save_dir = "simple_capture"
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    
    packet_count = 0
    start_time = time.time()
    data_buffer = []
    last_save_time = time.time()
    
    print(" Simple UDP Monitor (no root required)")
    print(f" Saving data to: {os.path.abspath(save_dir)}")
    print("=" * 80)
    print("Listening on UDP port 8888...")
    print("Press Ctrl+C to stop\n")
    
    try:
        while True:
            data, addr = sock.recvfrom(65535)
            packet_count += 1
            
            current_time = time.time()
            timestamp = datetime.now()
            
            # Create packet record
            packet_record = {
                "timestamp": timestamp.isoformat(),
                "packet_number": packet_count,
                "source": f"{addr[0]}:{addr[1]}",
                "size": len(data),
                "complete": False,
                "data": None
            }
            
            print(f"\n[{timestamp.strftime('%H:%M:%S.%f')[:-3]}] Packet #{packet_count} from {addr[0]}:{addr[1]}")
            print(f"   Size: {len(data):,} bytes")
            
            # Try to parse
            try:
                text = data.decode('utf-8')
                if text.strip().startswith('{') and text.strip().endswith('}'):
                    json_data = json.loads(text)
                    print(f"   COMPLETE JSON")
                    packet_record["complete"] = True
                    packet_record["data"] = json_data
                    
                    # Show key info
                    if 'header' in json_data:
                        ts = json_data['header'].get('timestamp', 0)
                        sim_time = json_data['header'].get('sim_time_sec', 0)
                        print(f"      Time: {datetime.fromtimestamp(ts).strftime('%H:%M:%S')}")
                        print(f"      Sim: {sim_time:.1f}s")
                        
                else:
                    # Check for partial JSON
                    start = text.find('{')
                    end = text.rfind('}') + 1
                    if start >= 0 and end > start:
                        json_str = text[start:end]
                        try:
                            json_data = json.loads(json_str)
                            print(f"   PARTIAL JSON ({len(json_str):,} chars)")
                            packet_record["data"] = json_data
                        except:
                            packet_record["raw_data"] = text[:500] + "..." if len(text) > 500 else text
                            print(f"    RAW: {len(text):,} chars")
                            if len(text) > 100:
                                print(f"      Preview: {text[:100]}...")
                    else:
                        packet_record["raw_data"] = text[:500] + "..." if len(text) > 500 else text
            except:
                # Binary data
                packet_record["raw_data_base64"] = binascii.b2a_base64(data).decode('utf-8').strip()
                print(f"    BINARY: {len(data):,} bytes")
                # Show hex dump
                hex_data = binascii.hexlify(data[:50]).decode('utf-8')
                print(f"      Hex: {hex_data}...")
            
            # Add to buffer
            data_buffer.append(packet_record)
            
            # Auto-save every 10 packets
            if packet_count % 10 == 0:
                elapsed = current_time - start_time
                rate = packet_count / elapsed
                print(f"\n {packet_count} packets, {rate:.1f}/sec")
                
            # Save to disk every 30 seconds or every 100 packets
            if current_time - last_save_time > 30 or len(data_buffer) >= 100:
                save_data_to_file(save_dir, data_buffer, packet_count, start_time)
                data_buffer.clear()
                last_save_time = current_time
                
    except KeyboardInterrupt:
        print(f"\n\n Captured {packet_count} packets in {time.time()-start_time:.1f}s")
        # Save remaining data
        if data_buffer:
            save_data_to_file(save_dir, data_buffer, packet_count, start_time)
        
    except Exception as e:
        print(f"\nError: {e}")
        if data_buffer:
            save_data_to_file(save_dir, data_buffer, packet_count, start_time)

def save_data_to_file(save_dir, data_buffer, packet_count, start_time):
    """Save buffered data to file"""
    if not data_buffer:
        return
        
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"capture_{timestamp}.json"
    filepath = os.path.join(save_dir, filename)
    
    data_to_save = {
        "metadata": {
            "save_time": datetime.now().isoformat(),
            "total_packets": packet_count,
            "saved_packets": len(data_buffer),
            "duration": time.time() - start_time
        },
        "packets": data_buffer
    }
    
    try:
        with open(filepath, 'w') as f:
            json.dump(data_to_save, f, indent=2, default=str)
        print(f" Saved {len(data_buffer)} packets to {filename}")
    except Exception as e:
        print(f"✗ Error saving to disk: {e}")

if __name__ == "__main__":
    print("Choose monitoring mode:")
    print("1. RAW socket (requires root, shows EVERYTHING, saves all data)")
    print("2. Simple UDP (no root, may miss broadcast packets, saves data)")
    
    choice = input("\nEnter choice (1 or 2): ").strip()
    
    if choice == "1":
        # Optional: Ask for save directory
        save_dir = input("Enter save directory [default: captured_data]: ").strip()
        if not save_dir:
            save_dir = "captured_data"
            
        monitor = RawUDPMonitor(port=8888, save_dir=save_dir)
        monitor.capture_all_traffic()
    else:
        simple_udp_monitor()