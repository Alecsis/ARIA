from radio import Radio
import time

radio = Radio()
print("RX ready")

last = {}

while True:
    if radio.available():
        data = radio.recv()
        if not data:
            continue

        try:
            msg = data.decode().strip()
            parts = msg.split("|")

            if parts[0] != "AVO":
                print("Bad packet:", msg)
                continue

            pkt_type = parts[1]

            # =====================
            # PACKET A (GYRO)
            # =====================
            if pkt_type == "A":
                counter = parts[2]
                gx = float(parts[3])
                gy = float(parts[4])
                gz = float(parts[5])

                last["counter"] = counter

                print("\n--- PACKET A ---")
                print("Counter:", counter)
                print("Gyro:", gx, gy, gz)

            # =====================
            # PACKET B (SENSORS)
            # =====================
            elif pkt_type == "B":
                alt = float(parts[2])
                ax = float(parts[3])
                ay = float(parts[4])
                az = float(parts[5])

                print("\n--- PACKET B ---")
                print("Alt(ft):", alt)
                print("Accel:", ax, ay, az)

            else:
                print("Unknown packet type:", msg)

        except Exception as e:
            print("Decode error:", e)

    time.sleep(0.05)