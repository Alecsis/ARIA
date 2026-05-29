from radio import Radio
import time
from machine import Pin, Timer
from ws2812 import WS2812  # Custom Seeed Studio Library

# ==========================================
# XIAO RP2040 NEOPIXEL (POWER + DATA CONFIG)
# ==========================================
led_power = Pin(11, Pin.OUT)
led_power.value(1) 

pixel = WS2812(12, 1)

# Status color profiles (Toned down)
COLOR_RED    = (30, 0, 0)    
COLOR_GREEN  = (0, 30, 0)    
COLOR_BLACK  = (0, 0, 0)     

# State flags
system_status = "STARTUP"   # STARTUP, LINK_OK, LINK_FAIL
current_color = COLOR_BLACK
blink_toggle = False

last_packet_time = time.ticks_ms()
last_radio_reset = time.ticks_ms()

def led_timer_callback(timer):
    """Background color driver that watches link performance"""
    global system_status, current_color, blink_toggle, last_packet_time
    
    # Timeout check: If no packet for over 1000ms, mark link failure
    if system_status == "LINK_OK" and time.ticks_diff(time.ticks_ms(), last_packet_time) > 1000:
        system_status = "LINK_FAIL"

    if system_status == "STARTUP":
        current_color = COLOR_RED
        
    elif system_status == "LINK_OK":
        current_color = COLOR_GREEN
        
    elif system_status == "LINK_FAIL":
        blink_toggle = not blink_toggle
        current_color = COLOR_RED if blink_toggle else COLOR_BLACK

    pixel.pixels_fill(current_color)
    pixel.pixels_show()

# Run visual checks every 200ms
led_timer = Timer(-1)
led_timer.init(period=200, mode=Timer.PERIODIC, callback=led_timer_callback)

# ==========================================
# RADIO RECEIVER ENGINE
# ==========================================
system_status = "STARTUP"
radio = Radio()
print("RX ready - Watchdog Armed")

while True:
    now = time.ticks_ms()
    
    # WATCHDOG: If stuck in STARTUP or LINK_FAIL for > 5 seconds, soft-reset the radio hardware
    if system_status != "LINK_OK" and time.ticks_diff(now, last_packet_time) > 5000:
        if time.ticks_diff(now, last_radio_reset) > 5000: # Don't spam resets
            print("WARNING: Radio link dead. Re-initializing radio object...")
            try:
                # FIXED: Overwrite the variable with a fresh class instance to clear memory and SPI registers
                radio = Radio() 
            except Exception as reset_error:
                print("Radio recovery failed:", reset_error)
            
            last_radio_reset = now
            # Prevent immediate re-trigger by extending the baseline timestamp
            last_packet_time = now 

    if radio.available():
        data = radio.recv()
        
        if not data:
            continue

        try:
            message = data.decode().strip()
            print(message)
            
            # Feed the watchdog! Packets are flowing smoothly
            last_packet_time = time.ticks_ms()
            system_status = "LINK_OK"
            
        except Exception as e:
            pass
    
    time.sleep(0.001)