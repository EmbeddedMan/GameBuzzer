/*
# Simple game buzzer code
# By Brian Schmalz, brian@schmalzhaus.com
# July 2025
"""
Base Station

Code for game hand controller project. July 2025, Brian Schmalz, brian@schmalzhaus.com

This code is meant to run on an AdaFruit Feather RP2040 RFM95 board. It is part of
a system with eight hand controllers and one base station. Players push the
button on their hand controller and the base station records who pushed their
button first, displaying the result on a screen.

(Pinout of Feather RP2040 RFM in CircuitPython)
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
*/

#include <SPI.h>
#include <RH_RF95.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7796S_kbv.h>
//#include <Adafruit_ImageReader_LittleFS.h>
#include <LittleFS.h>
#include <string.h>

#define NEOPIXEL_PIN    4
#define BUTTON1_PIN     10  // Red pushbutton
#define BUTTON2_PIN     7   // Boot button on Feather
#define BEEPER_PIN      11

#define TFT_DC          24
#define TFT_CS          25
#define TFT_RST         -1  // Connected, but I don't know to what GPIO pin

#define DBG0_PIN        26
#define DBG1_PIN        27
#define DBG2_PIN        28
#define DBG3_PIN        29

// Feather RP2040 w/Radio
#define RFM95_CS        16
#define RFM95_INT       21
#define RFM95_RST       17

#define RF95_FREQ       915.0
#define BASE_STATION_RH_ADDRESS 10    // Base Station radio address

#define NUMPIXELS       1

// Crappy dumb hack to get named colors like the Pyhton code had
#define COLOR_RED       pixel.Color(255, 0, 0)
#define COLOR_GREEN     pixel.Color(0, 255, 0)
#define COLOR_BLUE      pixel.Color(0, 0, 255)
#define COLOR_YELLOW    pixel.Color(255, 255, 0)
#define COLOR_PURPLE    pixel.Color(255, 0, 255)


// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Neopixel object
Adafruit_NeoPixel pixel(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

//Adafruit_ImageReader_LittleFS reader()

// Use hardware SPI for LCD
Adafruit_ST7796S_kbv tft = Adafruit_ST7796S_kbv(TFT_CS, TFT_DC, TFT_RST);

// Base Station global variables
bool base_station_is_reset;
uint8_t sync_pkt[5];
uint32_t reset_start_time;
uint32_t button_push_times[8];
uint32_t heartbeat_times[8];
uint8_t hc_btn_order[8];
bool any_btn_pushed;
uint32_t beeper_off_time;
uint32_t next_sync_time;
uint32_t hc_btn_push_time;
uint32_t hc_btn_push_time_ms;
uint32_t packet_rx_resume_time;
bool old_btn;


void setup() 
{
  // Set up debug outputs
  pinMode(DBG0_PIN, OUTPUT);
  digitalWrite(DBG0_PIN, LOW);
  pinMode(DBG1_PIN, OUTPUT);
  digitalWrite(DBG1_PIN, LOW);
  pinMode(DBG2_PIN, OUTPUT);
  digitalWrite(DBG2_PIN, LOW);
  pinMode(DBG3_PIN, OUTPUT);
  digitalWrite(DBG3_PIN, LOW);
  
  Serial.begin(115200);
  delay(3000);
  Serial.println("Game Buzzer Base Station");

  // LED Setup (for heartbeat)
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Beeper output
  pinMode(BEEPER_PIN, OUTPUT);
  digitalWrite(BEEPER_PIN, LOW);

  // Fire up the neopixel
  pixel.begin();
  pixel.setPixelColor(0, COLOR_YELLOW);
  pixel.show();

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) 
  {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) 
  {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
  rf95.setSpreadingFactor(7);
  rf95.setSignalBandwidth(500000);

  // Set up the onboard button
  pinMode(BUTTON2_PIN, INPUT_PULLUP);

  // Set up the big red button (game reset)
  pinMode(BUTTON1_PIN, INPUT_PULLUP);

  // Keep track of what 'mode' we are in
  base_station_is_reset = true;

  // Stores the time at which we reset and started waiting for button press packets
  // And we start each boot being in reset mode
  reset_start_time = millis();

  // Stores the absolute time of reception of the first button push packet from
  // each hand controller. 0.0 means the base station has not received a button
  // push packet since the last reset.
  memset(button_push_times, 0x00, sizeof(button_push_times));
  memset(heartbeat_times, 0x00, sizeof(heartbeat_times));
  memset(hc_btn_order, 0x00, sizeof(hc_btn_order));
  memset(sync_pkt, 0x00, sizeof(sync_pkt));

  // Init display
  tft.begin();
  // And display splash screen
  tft.setRotation(1);
  tft.invertDisplay(true);
  tft.fillScreen(ST7796S_WHITE);
  tft.setCursor(140, 140);
  tft.setTextColor(ST7796S_BLACK);  
  tft.setTextSize(4);
  tft.println("Book Club");

  // Keep the Book Club splash screen up there for a bit
  delay(2000);

  // Wait to receive packets
  Serial.println("Main Loop: starting time sync packets");
  // Start off with our LED showing green
  pixel.setPixelColor(0, COLOR_GREEN);
  pixel.show();

  // Toggle all debug outputs to make sure they're working
  digitalWrite(DBG0_PIN, HIGH);
  digitalWrite(DBG1_PIN, HIGH);
  digitalWrite(DBG2_PIN, HIGH);
  digitalWrite(DBG3_PIN, HIGH);
  digitalWrite(DBG0_PIN, LOW);
  digitalWrite(DBG1_PIN, LOW);
  digitalWrite(DBG2_PIN, LOW);
  digitalWrite(DBG3_PIN, LOW);

  old_btn = true;
  any_btn_pushed = false;
  beeper_off_time = 0;
  next_sync_time = millis();
  hc_btn_push_time = 0;
  hc_btn_push_time_ms = 0;

  // When non-zero, causes us to ignore all received packets
  packet_rx_resume_time = 0;

  // Blank the LCD and display green background
  tft.fillScreen(ST7796S_GREEN);

  tft.setCursor(140, 120);
  tft.setTextColor(ST7796S_BLACK);  
  tft.setTextSize(4);
  tft.println("Next Quiz");
  tft.setCursor(150, 160);
  tft.println("Question");
}

void loop() 
{
#if 0
  // Look for button press
  if (digitalRead(BUTTON1_PIN) == false || digitalRead(BUTTON2_PIN) == false)
  {
    // Only register the very first button press - after that, wait for a reset from base station
    if (btn_press_time_global == 0)
    {
      // We have a button press!
      // Record the local time
      btn_press_time_local = millis();
      // Compute the base station global time that the button pressed at
      btn_press_time_global = base_station_time + (btn_press_time_local - sync_time_local);
      pixel.setPixelColor(0, COLOR_PURPLE);
      pixel.show();
      Serial.print(millis());
      Serial.print(" Button pushed. Global time = ");
      Serial.println(btn_press_time_global);
    }
  }

  // See if its time for a heartbeat packet
  if (millis() >= next_heartbeat_time)
  {
    Serial.print(millis());
    Serial.print(" Sent heartbeat, ");
    if (btn_press_time_global)
    {
      Serial.print(" button pushed at global time = ");
      Serial.println(btn_press_time_global);
    }
    else
    {
      Serial.println(" no button push");
    }

    next_heartbeat_time = millis() + 10 * 1000; // Default next heartbeat time. Only used when no sync packet received from base station.
    // Toggle the read LED on the board
    if (digitalRead(LED_BUILTIN))
    {
      digitalWrite(LED_BUILTIN, LOW);
    }
    else
    {
      digitalWrite(LED_BUILTIN, HIGH);
    }
    digitalWrite(DBG3_PIN, HIGH);
    rf95.setHeaderFrom(my_address);
    rf95.setHeaderTo(10);   // Use base station's address for every packet
    // Fill in time of button push
    heartbeat_pkt[0] = btn_press_time_global & 0xFF;
    heartbeat_pkt[1] = (btn_press_time_global >> 8) & 0xFF;
    heartbeat_pkt[2] = (btn_press_time_global >> 16) & 0xFF;
    heartbeat_pkt[3] = (btn_press_time_global >> 24) & 0xFF;
    rf95.send(heartbeat_pkt, 4);
    digitalWrite(DBG3_PIN, LOW);
    //digitalWrite(DBG3_PIN, HIGH);
    //rf95.waitPacketSent();
    //digitalWrite(DBG3_PIN, LOW);
  }

  // In a non-blocking way, look to see if we've received a new packet from the base station
  packet_len = 10;
  if (rf95.recv(packet, &packet_len))
  {
    // packet_len now set to the length of the recived packet, not including RadioHead header
    if (packet_len != 5)
    {
      Serial.print(millis());
      Serial.print(" Error RX length not 5. Len = ");
      Serial.print(packet_len);
    }
    else
    {
      hc_dst_addr = rf95.headerTo();
      hc_src_addr = rf95.headerFrom();
      //headerId();
      hc_status = packet[0];
      // Check for propper addressing and status byte values
      if (hc_src_addr == 10 && hc_dst_addr == 255)
      {
        sync_time_local = millis();
        next_heartbeat_time = sync_time_local + (my_address * 20);
        // Extract the global time from the sync packet
        base_station_time = (packet[1] << 24) | (packet[2] << 16) | (packet[3] << 8) | packet[4];

        if (hc_status & (1 << (my_address - 1)))
        {
          pixel.setPixelColor(0, COLOR_RED);
          pixel.show();
          Serial.print(millis());
          Serial.print(" Got a RED sync ");
          Serial.println(base_station_time);
          last_pkt_red = true;
          if (motor_fire == false)
          {
            digitalWrite(VIBE_MOTOR_PIN, HIGH);
            motor_end_time = millis() + 3000;
            motor_fire = true;
          }
        }
        else
        {
          pixel.setPixelColor(0, COLOR_GREEN);
          pixel.show();
          Serial.print(millis());
          Serial.print(" Got a GREEN sync ");
          Serial.println(base_station_time);
          if (last_pkt_red)
          {
            last_pkt_red = false;
            btn_press_time_global = 0;
            
          }
          motor_fire = false;
        }
      }
    }
  }

  if (motor_end_time > 0)
  {
    if (millis() > motor_end_time)
    {
      motor_end_time = 0;
      digitalWrite(VIBE_MOTOR_PIN, LOW);
    }
  }
#endif
}
