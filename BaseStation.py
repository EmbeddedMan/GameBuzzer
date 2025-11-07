# Simple game buzzer code
# By Brian Schmalz, brian@schmalzhaus.com
# July 2025
# Based on sample code by Adafruit
"""
Base Station

Code for game hand controller project. July 2025, Brian Schmalz, brian@schmalzhaus.com

This code is meant to run on an AdaFruit Feather RP2040 RFM95 board. It is part of
a system with eight hand controllers and one base station. Players push the
button on their hand controller and the base station records who pushed their
button first, displaying the result on a screen.

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

###################
#  BASE STATION   #
###################

# All packets sent between any nodes will consist of the following:
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
import displayio
from fourwire import FourWire
import terminalio
from adafruit_display_text import label
import adafruit_ili9341
import digitalio
import neopixel
import adafruit_rfm9x
import time
import struct

# LED setup (for heartbeat)
led = digitalio.DigitalInOut(board.LED)
led.direction = digitalio.Direction.OUTPUT

# Beeper output setup
beeper = digitalio.DigitalInOut(board.D11)
beeper.direction = digitalio.Direction.OUTPUT
beeper.value = False

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
    (0, 255, 255),
    (255, 0, 255),
    (255, 255, 0),
    (0, 0, 255),
    (0, 255, 0),
    (255,255,255),
    (0,0,0)
]

COLOR_RED = 0
COLOR_GREEN = 1
COLOR_BLUE = 2
COLOR_YELLOW = 3
COLOR_PURPLE = 4
COLOR_BLACK = 5
COLOR_WHITE = 6

print("Reset: board booted")
# Start off with our LED showing green
pixel.fill(color_values[COLOR_YELLOW])

# Define radio frequency in MHz. Must match your
# module. Can be a value like 915.0, 433.0, etc.
RADIO_FREQ_MHZ = 915.0

# Define Chip Select and Reset pins for the radio module.
CS = digitalio.DigitalInOut(board.RFM_CS)
RESET = digitalio.DigitalInOut(board.RFM_RST)

# Initialize RFM95 radio
rfm95 = adafruit_rfm9x.RFM9x(board.SPI(), CS, RESET, RADIO_FREQ_MHZ)
rfm95.spreading_factor = 7
rfm95.node = 10      # Because we are the base station we are node 10
rfm95.signal_bandwidth = 500000
rfm95.tx_power = 23

# Set up the onboard button
btn = digitalio.DigitalInOut(board.BUTTON)
btn.direction = digitalio.Direction.INPUT
btn.pull = digitalio.Pull.UP

# Set up big red button (game reset)
reset_btn = digitalio.DigitalInOut(board.D10)
reset_btn.direction = digitalio.Direction.INPUT
reset_btn.pull = digitalio.Pull.UP

# Keep track of what 'mode' we are in
base_station_is_reset = True

# Stores the time at which we reset and started waiting for button press packets
# And we start each boot being in reset mode
reset_start_time = time.monotonic()

# Stores the absolute time of reception of the first button push packet from
# each hand controller. 0.0 means the base station has not received a button
# push packet since the last reset.
button_push_times = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
heartbeat_times = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
hc_btn_order = [0, 0, 0, 0, 0, 0, 0, 0]

sync_pkt = bytearray(5)

# Initialize the LCD
displayio.release_displays()

spi = board.SPI()
tft_cs = board.D25
tft_dc = board.D24
display_bus = FourWire(spi, command=tft_dc, chip_select=tft_cs, reset=board.D12)
display = adafruit_ili9341.ILI9341(display_bus, width=480, height=320)

splash = displayio.Group()
display.root_group = splash

text_group = displayio.Group(scale=4, x=140, y=150)
text = "Book Club"
text_area = label.Label(terminalio.FONT, text=text, color=color_values[COLOR_BLACK])
text_group.append(text_area)  # Subgroup for text scaling
splash.append(text_group)

# Keep the Book Club splash screen up there for a bit
time.sleep(2.0)

# Wait to receive packets.
print("Main loop: starting time sync packets")
# Start off with our LED showing green
pixel.fill(color_values[COLOR_GREEN])

dbg0.value = False
dbg1.value = False
dbg2.value = False
dbg3.value = False

#text_group = displayio.Group(scale=2, x=57, y=50)
#text = "12345"
#text_area = label.Label(terminalio.FONT, text=text, color=0xFFFFFF)
#text_group.append(text_area)  # Subgroup for text scaling
#splash.append(text_group)

# Keep track of last button state for debouncing
old_btn = True
any_btn_pushed = False
beeper_off_time = 0.0

next_sync_time = time.monotonic()
time_bytes_ms = 0
hc_btn_push_time = 0.0
hc_btn_push_time_ms = 0

# When non-zero, causes us to ignore all received packets
packet_rx_resume_time = 0.0

# Blank the LCD and display green background
green_grp = displayio.Group()
display.root_group = green_grp

color_bitmap = displayio.Bitmap(480, 320, 1)
color_palette = displayio.Palette(1)
color_palette[0] = color_values[COLOR_GREEN]
bg_sprite = displayio.TileGrid(color_bitmap, pixel_shader=color_palette, x=0, y=0)
green_grp.append(bg_sprite)

text_group = displayio.Group(scale=4, x=140, y=130)
text = "Next Quiz"
text_area = label.Label(terminalio.FONT, text=text, color=color_values[COLOR_BLACK])
text_group.append(text_area)  # Subgroup for text scaling
green_grp.append(text_group)

text_group = displayio.Group(scale=4, x=140, y=170)
text = "Question!"
text_area = label.Label(terminalio.FONT, text=text, color=color_values[COLOR_BLACK])
text_group.append(text_area)  # Subgroup for text scaling
green_grp.append(text_group)

while True:
    dbg2.value = True
    rfm95.listen()
    
    if beeper_off_time > 0.0:
        if time.monotonic() >= beeper_off_time:
            beeper_off_time = 0.0
            beeper.value = False

    # Look for button press to reset our state
    if btn.value == False or reset_btn.value == False:
        if old_btn == True:
            old_btn = False
            base_station_is_reset = True
            button_push_times = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            hc_btn_push_time = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            hc_btn_order = [0, 0, 0, 0, 0, 0, 0, 0]
            pixel.fill(color_values[COLOR_GREEN])
            # Blank the LCD and display green background
            green_grp = displayio.Group()
            display.root_group = green_grp

            color_bitmap = displayio.Bitmap(480, 320, 1)
            color_palette = displayio.Palette(1)
            color_palette[0] = color_values[COLOR_GREEN]
            bg_sprite = displayio.TileGrid(color_bitmap, pixel_shader=color_palette, x=0, y=0)
            green_grp.append(bg_sprite)

            text_group = displayio.Group(scale=4, x=140, y=130)
            text = "Next Quiz"
            text_area = label.Label(terminalio.FONT, text=text, color=color_values[COLOR_BLACK])
            text_group.append(text_area)  # Subgroup for text scaling
            green_grp.append(text_group)

            text_group = displayio.Group(scale=4, x=140, y=170)
            text = "Question!"
            text_area = label.Label(terminalio.FONT, text=text, color=color_values[COLOR_BLACK])
            text_group.append(text_area)  # Subgroup for text scaling
            green_grp.append(text_group)
            
            any_btn_pushed = False
            
            # Set blanking time to ignore any hand controller packets for 1.5s
            packet_rx_resume_time = time.monotonic() + 1.5

            print("System now reset")
    else:
        old_btn = True

    # Has 1 second gone by? Time to send a sync packet?
    if time.monotonic() >= next_sync_time:
        #next_sync_time += 0.300
        next_sync_time += 1.0
        rfm95.destination = 255     # Broadcast to all hand controllers
        # Build up status byte based on each hand controller's state

        if any_btn_pushed == True:
            sync_pkt[0] = 255
        else:
            sync_pkt[0] = 0
        # Copy over the float time as four bytes
        sync_time = time.monotonic()
        #time_bytes = bytearray(struct.pack("f", sync_time))
        time_bytes_ms = int(sync_time * 1000.0)
        sync_pkt[1] = (int(time_bytes_ms) >> 24) & 0xFF
        sync_pkt[2] = (int(time_bytes_ms) >> 16) & 0xFF
        sync_pkt[3] = (int(time_bytes_ms) >> 8) & 0xFF
        sync_pkt[4] = (int(time_bytes_ms)) & 0xFF
        dbg1.value = True
        rfm95.send(sync_pkt)
        dbg1.value = False
        print("S: ", time_bytes_ms, " ", hc_btn_push_time, " ", hex(sync_pkt[1]), " ", hex(sync_pkt[2]), " ", hex(sync_pkt[3]), " ", hex(sync_pkt[4]))     # TODO: print out each HC's state? (G/R)

    dbg2.value = False
    
    # In a non-blocking way, look to see if we've received a packet
    if rfm95.rx_done():
        dbg0.value = True
        packet = rfm95.receive(timeout = 0.1, with_header = True)
        dbg0.value = False

        # Toggle the red LED on the board on each received packet
        led.value = not led.value

        # Ignore this packet if we have just been reset. If the resume_time is zero, or if
        # the current time is after the resume_time, then process the packet.
        if packet_rx_resume_time < time.monotonic():
            packet_rx_resume_time = 0.0
            if packet is not None:
                dbg3.value = True
                # Check the packet length
                if len(packet) == 8:
                    hc_dst_addr = int.from_bytes(packet[0:1], "big")
                    hc_src_addr = int.from_bytes(packet[1:2], "big")
#                    hc_btn_push_time = struct.unpack('f', packet[4:8])[0]
                    hc_btn_push_time_ms = (int.from_bytes(packet[4:5], "big") << 24) + (int.from_bytes(packet[5:6], "big") << 16) + (int.from_bytes(packet[6:7], "big") << 8) + int.from_bytes(packet[7:8], "big")
                    hc_btn_push_time = hc_btn_push_time_ms / 1000.0

                    print("Pkt: Hdr: ", [hex(x) for x in packet[0:4]])
                    if hc_src_addr > 0 and hc_src_addr <= 8:
                        if hc_dst_addr == 10:
                            if hc_btn_push_time == 0.0:  # heartbeat packet, button not pushed
                                heartbeat_times[hc_src_addr - 1] = time.monotonic()
                                print(" got heartbeat")
                            else:
                                if any_btn_pushed == False:
                                    beeper.value = True
                                    any_btn_pushed = True
                                    beeper_off_time = time.monotonic() + 2.0
                                if button_push_times[hc_src_addr - 1] == 0.0:
                                    print("Got button push from hand controller ", hc_src_addr)
                                    button_push_times[hc_src_addr - 1] = hc_btn_push_time

                                    # Blank the LCD and display red background
                                    red_grp = displayio.Group()
                                    display.root_group = red_grp

                                    color_bitmap = displayio.Bitmap(480, 320, 1)
                                    color_palette = displayio.Palette(1)
                                    color_palette[0] = color_values[COLOR_RED]
                                    bg_sprite = displayio.TileGrid(color_bitmap, pixel_shader=color_palette, x=0, y=0)
                                    red_grp.append(bg_sprite)

                                    text_group = displayio.Group(scale=3, x=10, y=25)
                                    text = "Player pushed"
                                    text_area = label.Label(terminalio.FONT, text=text, color=color_values[COLOR_WHITE])
                                    text_group.append(text_area)  # Subgroup for text scaling
                                    red_grp.append(text_group)
                                    
                                    hc_top_time = 1000000.0
                                    hc_top = 0
                                    for hc in range(8):
                                        if button_push_times[hc] != 0.0:
                                            if button_push_times[hc] < hc_top_time:
                                                hc_top_time = button_push_times[hc]
                                                hc_btn_order[0] = hc + 1
                                        
                                    hc_top_time = 1000000.0
                                    for hc in range(8):
                                        if button_push_times[hc] != 0.0:
                                            if hc_btn_order[0] != hc + 1:
                                                if button_push_times[hc] < hc_top_time:
                                                    hc_top_time = button_push_times[hc]
                                                    hc_btn_order[1] = hc + 1

                                    hc_top_time = 1000000.0
                                    for hc in range(8):
                                        if button_push_times[hc] != 0.0:
                                            if hc_btn_order[0] != hc + 1:
                                                if hc_btn_order[1] != hc + 1:
                                                    if button_push_times[hc] < hc_top_time:
                                                        hc_top_time = button_push_times[hc]
                                                        hc_btn_order[2] = hc + 1

                                    if hc_btn_order[0] > 0:
                                        text_group1 = displayio.Group(scale=6, x=10, y=100)
                                        text = str(hc_btn_order[0])
                                        text_area = label.Label(terminalio.FONT, text=text, color=color_values[COLOR_WHITE])
                                        text_group1.append(text_area)  # Subgroup for text scaling
                                        red_grp.append(text_group1)
                                        
                                    if hc_btn_order[1] > 0:
                                        text_group1 = displayio.Group(scale=6, x=60, y=100)
                                        text = str(hc_btn_order[1])
                                        text_area = label.Label(terminalio.FONT, text=text, color=color_values[COLOR_WHITE])
                                        text_group1.append(text_area)  # Subgroup for text scaling
                                        red_grp.append(text_group1)

                                    if hc_btn_order[2] > 0:
                                        text_group1 = displayio.Group(scale=6, x=110, y=100)
                                        text = str(hc_btn_order[2])
                                        text_area = label.Label(terminalio.FONT, text=text, color=color_values[COLOR_WHITE])
                                        text_group1.append(text_area)  # Subgroup for text scaling
                                        red_grp.append(text_group1)

                                pixel.fill(color_values[COLOR_RED])
                        else:
                            print("Got a packet with a bad destination address of ", hc_dst_addr)
                    else:
                        print("Got a packet with a bad source address of ", hc_src_addr)
                else:
                    print("Packet received with bad length of ", len(packet))
                dbg3.value = False
