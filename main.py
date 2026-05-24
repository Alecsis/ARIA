from radio import Radio
import time
import struct

radio = Radio()
print("RX ready")

while True:
    if radio.available():
        data = radio.recv()
        
        if not data:
            continue

        # Ensure we received the full 32-byte payload before unpacking
        if len(data) == 32:
            try:
                # UNPACK THE 32 BYTES
                unpacked = struct.unpack('<I7f', data)
                
                counter = unpacked[0]
                gx, gy, gz = unpacked[1], unpacked[2], unpacked[3]
                alt = unpacked[4]
                ax, ay, az = unpacked[5], unpacked[6], unpacked[7]

                print("----- PACKET OK -----")
                print(f"Counter: {counter}")
                # Labeling gx and ax clearly as the active Pitch channel we isolated
                print(f"Gyro (Pitch/Roll/Yaw): {gx:.2f}, {gy:.2f}, {gz:.2f}")
                print(f"Accel(Pitch/Roll/Yaw): {ax:.2f}, {ay:.2f}, {az:.2f}")
                print(f"Alt(ft): {alt:.2f}")
                print("---------------------")

            except Exception as e:
                print("Decode error:", e)
        else:
            print("Received malformed packet length:", len(data))

    time.sleep(0.01) # Faster poll rate on the receiver side