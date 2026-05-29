from radio import Radio
import time

# Initialize the radio wrapper
radio = Radio()

print("RX ready - Listening for data...")

while True:
    if radio.available():
        # Retrieve the raw bytes payload
        raw_data = radio.recv()
        
        if raw_data is not None:
            try:
                # The NRF24L01 driver pads messages with null bytes (\x00) up to the payload_size.
                # rstrip(b'\x00') removes that padding before decoding to a string.
                clean_msg = raw_data.rstrip(b'\x00').decode('utf-8')
                
                print("Received:", clean_msg)
                
            except Exception as e:
                print("Failed to decode packet:", e)
                
    # Small pause to keep from thrashing the CPU core completely
    time.sleep(0.005)