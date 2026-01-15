import socket
import json
import time

PORT = 8888

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(('', PORT))
sock.settimeout(1.0)

print(f"Listening on UDP port {PORT}...")
packet_count = 0
start_time = time.time()

while True:
    try:
        data, addr = sock.recvfrom(65535)
        packet_count += 1
        
        # Try to parse as JSON
        try:
            json_data = json.loads(data.decode('utf-8'))
            zones = json_data.get('zone_count', 0)
            sensors = json_data.get('sensor_count', 0)
            
            print(f"📦 Packet {packet_count}: {len(data)} bytes")
            print(f"   From: {addr}, Zones: {zones}, Sensors: {sensors}")
            print(f"   Time: {json_data.get('time_of_day', 0):.1f}h")
            print(f"   Reservoir: {json_data.get('reservoir_level_pct', 0):.1f}%")
        except:
            print(f"📦 Packet {packet_count}: {len(data)} bytes (Not JSON)")
        
        # Calculate rate
        elapsed = time.time() - start_time
        if elapsed >= 5.0:
            rate = packet_count / elapsed
            print(f"\n📈 Rate: {rate:.1f} packets/sec ({packet_count} total)\n")
            packet_count = 0
            start_time = time.time()
            
    except socket.timeout:
        continue
    except KeyboardInterrupt:
        print("\nStopping receiver...")
        break

sock.close()