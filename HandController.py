# Simple game buzzer code
# By Brian Schmalz, brian@schmalzhaus.com
# July 2025
# Based on sample code by Adafruit
"""
Hand Controller

Code for game hand controller project. July 2025, Brian Schmalz, brian@schmalzhaus.com

This code is meant to run on an AdaFruit Feather RP2040 RFM95 board. It is part of
a system with eight hand controllers and one base station. Players push the
button on their hand controller and the base station records who pushed their
button first, displaying the result on a screen.

Pins available on this board:
board.A0 (GPIO26)
board.A1 (GPIO27)
board.A2 (GPIO28)
board.A3 (GPIO29)
board.BOOT board.BUTTON board.D7 (GPIO7)
board.D0 board.RX (GPIO1)
board.D1 board.TX (GPIO0)
board.D10 (GPIO10)
board.D11 (GPIO11)
board.D12 (GPIO12)
board.D13 board.LED (GPIO13)
board.D24 (GPIO24)
board.D25 (GPIO25)
board.D4 board.NEOPIXEL (GPIO4)
board.D5 (GPIO5)
board.D6 (GPIO6)
board.D9 (GPIO9)
board.MISO (GPIO8)
board.MOSI (GPIO15)
board.RFM_CS (GPIO16)
board.RFM_IO0 (GPIO21)
board.RFM_IO1 (GPIO22)
board.RFM_IO2 (GPIO23)
board.RFM_IO3 (GPIO19)
board.RFM_IO4 (GPIO20)
board.RFM_IO5 (GPIO18)
board.RFM_RST (GPIO17)
board.SCK (GPIO14)
board.SCL (GPIO3)
board.SDA (GPIO2)

When device becomes corrupted and you can't write to it anymore:
>>> import storage
>>> storage.erase_filesystem()
"""

######################
#  HAND CONTROLLER   #
######################

# All packets sent between any nodes will consist of the folowing:
# Each packet will consist of four bytes.
# Packet Structure:
#   Byte 1: Source address (0 = base station, 1 - 8 = hand controllers)
#   Byte 2: Destination address (0 = base station, 1 - 8 = hand controllers)
#   Byte 3: Status byte
#   Byte 4: Battery voltage (0 = 0.0v, 255 = 4.7v)
#
# Status byte for hand controller -> base station
# 0 = Heartbeat only. Button not pushed.
# 1 = Button has been pushed.
#
# Status byte for base station -> hand controller (ACK packets)
# 0 = Turn your LED to green (system reset)
# 1 = Turn your LED to red (your button press has been recorded)
#
# Battery voltage byte will always be 0 in packets sent from base station.
# Heartbeats will be sent from hand controller to base station every 1s.
# When a hand controller button push happens, the hand controller will
#   immediately send a packet with status = 1 to the base station.
# When the hand controller button is released, a heartbeat packet is
#   immediately sent.
# Every packet received at the base station must be acknowledged by an ACK
#   packet.
# The ACK packet has the base station's address (0) as the first byte.
# The ACK packet has the destination hand controller's address (1-8) as the
#   second byte.
# The base station has a screen. This screen displays the current
#   state of every hand controller (pressed/unpressed/no comms) in real
#   time as well as the controller's battery level.
# The base station has a 'Reset' button. When this button is pressed,
#   the base station resets the display and begins watching for the first
#   button push from each hand controller and records the order in which
#   their buttons were pushed.
# When in the 'reset' state (waiting for first controller's button push)
#   the base station will ack with a status byte value of 0 which will make
#   all of the hand controler's neopixels green, indicating that the system
#   is waiting for the first person to press their button.
# Once the base station receives the first button push message from a
#   hand controller, it will begin acking that hand controller with a status
#   byte value of 1 which will make the hand controller's neopixel turn red,
#   indicating that the button push has been registered at the base station.
# After reset, the base station will display its neopixel as green. Once it
#   receives the first button press it will display as red.

import board
import digitalio
import neopixel
import adafruit_rfm9x
import time
import struct

# LED setup (for heartbeat)
led = digitalio.DigitalInOut(board.LED)
led.direction = digitalio.Direction.OUTPUT

# Vibe motor output
motor = digitalio.DigitalInOut(board.SDA)
motor.direction = digitalio.Direction.OUTPUT

# Set up the four debug GPIOs as outputs
dbg0 = digitalio.DigitalInOut(board.A0)
dbg0.direction = digitalio.Direction.OUTPUT
dbg1 = digitalio.DigitalInOut(board.A1)
dbg1.direction = digitalio.Direction.OUTPUT
dbg2 = digitalio.DigitalInOut(board.A2)
dbg2.direction = digitalio.Direction.OUTPUT
dbg3 = digitalio.DigitalInOut(board.A3)
dbg3.direction = digitalio.Direction.OUTPUT

# Set up NeoPixel.
pixel = neopixel.NeoPixel(board.NEOPIXEL, 1)
pixel.brightness = 1.0

# Define the possible NeoPixel colors.
color_values = [
    (255, 0, 0),
    (0, 255, 0),
    (0, 0, 255),
    (255, 255, 0),
    (255, 0, 255),
]

COLOR_RED = 0
COLOR_GREEN = 1
COLOR_BLUE = 2
COLOR_YELLOW = 3
COLOR_PURPLE = 4

# Start off with our LED showing blue until we start getting replies
# from the base station.
pixel.fill(color_values[COLOR_YELLOW])

print("Reset: Waiting 5s")

# Define radio frequency in MHz. Must match your
# module. Can be a value like 915.0, 433.0, etc.
RADIO_FREQ_MHZ = 915.0

# Define Chip Select and Reset pins for the radio module.
CS = digitalio.DigitalInOut(board.RFM_CS)
RESET = digitalio.DigitalInOut(board.RFM_RST)

# Initialize RFM95 radio
rfm95 = adafruit_rfm9x.RFM9x(board.SPI(), CS, RESET, RADIO_FREQ_MHZ)
rfm95.spreading_factor = 7
rfm95.signal_bandwidth = 500000
rfm95.tx_power = 23

# Set up the onboard button
btn1 = digitalio.DigitalInOut(board.BUTTON)
btn1.direction = digitalio.Direction.INPUT
btn1.pull = digitalio.Pull.UP

# Set up the big red button input
btn2 = digitalio.DigitalInOut(board.SCL)
btn2.direction = digitalio.Direction.INPUT
btn2.pull = digitalio.Pull.UP

# Read in our address, which is stored in the "my_address.txt" file.
with open("my_address.txt", mode='rt') as f:
    line = f.readline()
    MY_ADDRESS = int(line)
    if MY_ADDRESS > 8:
        MY_ADDRESS = 8
        print("Error in reading address. Too big.")
    if MY_ADDRESS == 0:
        MY_ADDRESS = 1
        print("Error in reading address. Too small.")
    print("Read in my address of ", MY_ADDRESS, " from disk")

# Set our Radiohead addresses
rfm95.node = MY_ADDRESS             # The 'from' field in all packets we send
rfm95.destination = 10              # The 'to' field : use Base Station's addr


# Space for the heartbeat packet back to the base station
heartbeat_pkt = bytearray(5)

# Stores the time at which the last heartbeat packet went out
heartbeat_time = time.monotonic()

time.sleep(5.0)

print("Main Loop: sending heartbeats, waiting for button push")
# Start off with our LED showing blue until we start getting replies
# from the base station.
pixel.fill(color_values[COLOR_BLUE])

dbg0.value = False
dbg1.value = False
dbg2.value = False
dbg3.value = False

# Keep track of last button state for debouncing
old_btn = True
next_heartbeat_time = time.monotonic()
motor_end_time = 0.0
motor_fire = False
sync_time = 0.0
sync_time_local = 0.0
base_station_time = 0.0
btn_press_time_local = 0.0
btn_press_time_global = 0.0
last_pkt_red = False

while True:
    rfm95.listen()

    # Look for button press
    if btn1.value == False or btn2.value == False:
        if old_btn == True:
            # Record the local time of this button press
            btn_press_time_local = time.monotonic()
            # Compute the base station global time that the button was pressed
            btn_press_time_global = base_station_time + (btn_press_time_local - sync_time_local)
            old_btn = False
            pixel.fill(color_values[COLOR_PURPLE])
            print("Got button push ", btn_press_time_global)
    else:
        old_btn = True

    # See if it's time for a heartbeat packet
    if time.monotonic() >= next_heartbeat_time:
        next_heartbeat_time = time.monotonic() + 10.0  # Default next heartbeat time, only used when no sync received from base station
        # Toggle the red LED on the board
        led.value = not led.value
        dbg3.value = True
        rfm95.node = MY_ADDRESS             # The 'from' field in all packets we send
        rfm95.destination = 10              # The 'to' field : use Base Station's addr
        # Fill in time of button push
        btn_push_time_bytes = bytearray(struct.pack('f', btn_press_time_global))
        rfm95.send(btn_push_time_bytes, destination = 10, node = MY_ADDRESS)
        dbg3.value = False
        print("Sent heartbeat ", btn_press_time_global)

    # In a non-blocking way, look to see if we've received a packet
    if rfm95.rx_done():
        # We have received a packet, so pull it out of the radio and parse it
        dbg0.value = True
        packet = rfm95.receive(timeout = 0.1, with_header = True)
        dbg0.value = False
        # If no packet was received during the timeout then None is returned.
        if packet is not None:
            # Check the packet length
            if len(packet) == 9:    # 4 header bytes + 5 data bytes
                hc_dst_addr = int.from_bytes(packet[0:1], "big")
                hc_src_addr = int.from_bytes(packet[1:2], "big")
                hc_status = int.from_bytes(packet[4:5], "big")
                # Check for valid address and status byte values
                if hc_src_addr == 10:
                    if hc_dst_addr == 255: # Broadcast
                        sync_time_local = time.monotonic()
                        next_heartbeat_time = sync_time_local + (MY_ADDRESS * 20.0 / 1000.0)
                        # Extract the global time float from the sync packet
                        base_station_time = struct.unpack('f', packet[5:9])[0]

                        if hc_status & (1 << (MY_ADDRESS-1)):
                            pixel.fill(color_values[COLOR_RED])
                            print("Got a RED sync ", base_station_time)
                            last_pkt_red = True
                            if motor_fire == False:
                                motor.value = True
                                motor_end_time = time.monotonic() + 3.0
                                motor_fire = True
                        else:
                            pixel.fill(color_values[COLOR_GREEN])
                            print("Got a GREEN sync ", base_station_time)
                            if last_pkt_red:
                                last_pkt_red = False
                                btn_press_time_global = 0.0
                            motor_fire = False
#                else:
                        # Do nothing, these are packets for other hand controllers
#            else:
                    # Do nothing, these are packets from other hand controllers
            else:
                print("Packet received with bad length of ", len(packet))
    
    if motor_end_time > 0.0:
        if time.monotonic() > motor_end_time:
            motor_end_time = 0.0
            motor.value = False

