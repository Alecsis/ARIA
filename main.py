from radio import Radio
import time
import struct

radio = Radio()
print("RX ready")

# Simple pass-through - just forward the filtered angles to Serial
# Processing expects: "roll/pitch/yaw" format

while True:
    if radio.available():
        data = radio.recv()
        
        if not data:
            continue

        # New format: we're sending just 16 bytes (counter + roll + pitch + yaw)
        if len(data) == 16:
            try:
                unpacked = struct.unpack('<I3f', data)
                counter = unpacked[0]
                roll = unpacked[1]
                pitch = unpacked[2]
                yaw = unpacked[3]
                
                # Print in the simple format Processing expects
                # Format: roll/pitch/yaw
                print(f"{roll:.3f}/{pitch:.3f}/{yaw:.3f}")
                
                # Optional: print debug info
                if counter % 20 == 0:
                    print(f"# Counter: {counter}")
                
            except Exception as e:
                print("Decode error:", e)
        
        # Keep old format for compatibility during transition
        elif len(data) == 32:
            try:
                unpacked = struct.unpack('<I7f', data)
                counter = unpacked[0]
                gx, gy, gz = unpacked[1], unpacked[2], unpacked[3]
                alt = unpacked[4]
                ax, ay, az = unpacked[5], unpacked[6], unpacked[7]
                
                # For now, we're not sending gyro/accel separately anymore
                # But we can still print it for debugging
                print(f"Legacy packet: {counter}")
                
            except Exception as e:
                print("Decode error:", e)
        else:
            print(f"Unknown packet length: {len(data)}")
    
    time.sleep(0.001)  # 1ms poll rate