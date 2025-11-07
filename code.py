# SPDX-FileCopyrightText: 2023 Kattni Rembor for Adafruit Industries
# SPDX-License-Identifier: MIT

"""
CircuitPython Feather RP2040 RFM95 Packet Receive Demo

This demo waits for a "button" packet. When the first packet is received, the NeoPixel LED
lights up red. The next packet changes it to green. The next packet changes it to blue.
Subsequent button packets cycle through the same colors in the same order.

This example is meant to be paired with the Packet Send Demo code running
on a second Feather RP2040 RFM95 board.

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

"""

import board
import digitalio
import neopixel
import adafruit_rfm9x
import time

# LED setup (for heartbeat)
led = digitalio.DigitalInOut(board.LED)
led.direction = digitalio.Direction.OUTPUT

# Set up NeoPixel.
pixel = neopixel.NeoPixel(board.NEOPIXEL, 1)
pixel.brightness = 0.5

# Define the possible NeoPixel colors.
color_values = [
    (255, 0, 0),
    (0, 255, 0),
    (0, 0, 255),
]
COLOR_RED = 0
COLOR_GREEN = 1
COLOR_BLUE = 2

# Define radio frequency in MHz. Must match your
# module. Can be a value like 915.0, 433.0, etc.
RADIO_FREQ_MHZ = 915.0

# Define Chip Select and Reset pins for the radio module.
CS = digitalio.DigitalInOut(board.RFM_CS)
RESET = digitalio.DigitalInOut(board.RFM_RST)

# Initialise RFM95 radio
rfm95 = adafruit_rfm9x.RFM9x(board.SPI(), CS, RESET, RADIO_FREQ_MHZ)

# Set up the onboard button
btn = digitalio.DigitalInOut(board.BUTTON)
btn.direction = digitalio.Direction.INPUT
btn.pull = digitalio.Pull.UP

###################
#  BASE STATION   #
###################

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

# Define the two types of ack packets the base station will send to the hand controllers
ack_green_pkt = bytes([0, 0, 0, 0])
ack_red_pkt = bytes([0, 0, 1, 0])

# Keep track of what 'mode' we are in
base_station_is_reset = True

# Stores the time at which we reset and started waiting for button press packets
# And we start each boot being in reset mode
reset_start_time = time.monotonic()

# Stores the absolute time of reception of the first button push packet from
# each hand controller. 0.0 means the base station has not received a button
# push packet since the last reset.
button_push_time = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


# Wait to receive packets.
print("Reset: waiting for hand controllers to press buttons")
# Start off with our LED showing green
pixel.fill(color_values[COLOR_GREEN])

while True:
    # Toggle the red LED on the board
    led.value = not led.value

    # Look for button press to reset our state
    if not btn.value:
        base_station_is_reset = True
        button_push_time = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        pixel.fill(color_values[COLOR_GREEN])
        print("System now reset")

    # Look for a new packet - wait up to 0.25 second:
    packet = rfm95.receive(timeout=0.25)
    # If no packet was received during the timeout then None is returned.
    if packet is not None:
        # Check the packet length
        if len(packet) == 4:
            hc_src_addr = int.from_bytes(packet[0:1], "big")
            hc_dst_addr = int.from_bytes(packet[1:2], "big")
            hc_status = int.from_bytes(packet[2:3], "big")
            hc_battery = int.from_bytes(packet[3:4], "big")
            if hc_src_addr != 0 and hc_src_addr <= 8:
                if hc_dst_addr == 0:
                    if hc_status == 0:  # heartbeat packet
                        if button_push_time[hc_src_addr] == 0.0:
                            rfm95.send(bytes([0, hc_src_addr, 0, 0]))
                        else:
                            rfm95.send(bytes([0, hc_src_addr, 1, 0]))
                    else:
                        if hc_status == 1:  # button push packet
                            if button_push_time[hc_src_addr] == 0.0:
                                print("Got button push from hand controller ", hc_src_addr)
                                button_push_time[hc_src_addr] = time.monotonic();
                            pixel.fill(color_values[COLOR_RED])
                            rfm95.send(bytes([0, hc_src_addr, 1, 0]))
                        else:
                            print("Got an unknown status byte")
                else:
                    print("Got a packet with a bad destination address of ", hc_dst_addr)
            else:
                print("Got a packet with a bad source address of ", hc_src_addr)
        else:
            print("Packet recevied with bad length of ", len(packet))
