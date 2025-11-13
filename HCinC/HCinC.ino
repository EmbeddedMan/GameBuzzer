/*
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
*/

#include <SPI.h>
#include <RH_RF95.h>
#include <Adafruit_NeoPixel.h>
#include "LittleFS.h"

#define NEOPIXEL_PIN    4
#define VIBE_MOTOR_PIN  2
#define BUTTON1_PIN     3   // Top red pushbutton
#define BUTTON2_PIN     7   // Boot button on Feather

#define DBG0_PIN        26
#define DBG1_PIN        27
#define DBG2_PIN        28
#define DBG3_PIN        29

// Feather RP2040 w/Radio
#define RFM95_CS        16
#define RFM95_INT       21
#define RFM95_RST       17

#define RF95_FREQ       915.0

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

// HandController address (from 1 to 8) of me
uint8_t my_address;

bool old_btn;
uint32_t next_heartbeat_time;
uint32_t motor_end_time;
bool motor_fire;
uint32_t sync_time;
uint32_t sync_time_local;
uint32_t base_station_time;
uint32_t btn_press_time_local;
uint32_t btn_press_time_global;
bool last_pkt_red;
uint8_t heartbeat_pkt[5] = {0};
uint32_t heartbeat_time = 0;
uint32_t btn_press_time_debounce;
uint8_t packet[20];                 // Stores incoming packet
uint8_t packet_len = 0;
uint8_t hc_dst_addr;
uint8_t hc_src_addr;
uint8_t hc_status;


void setup() 
{
  File address_file;

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
  delay(2000);
  Serial.println("Game Buzzer Hand Controller");

  // LED Setup (for heartbeat)
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Fire up the neopixel
  pixel.begin();
  pixel.setPixelColor(0, COLOR_YELLOW);
  pixel.show();

  Serial.println("Waiting 2s");

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

  // Set up the big red button
  pinMode(BUTTON1_PIN, INPUT_PULLUP);

  // Set up the vibe motor output
  pinMode(VIBE_MOTOR_PIN, OUTPUT);
  digitalWrite(VIBE_MOTOR_PIN, LOW);

  if (LittleFS.begin())
  {
    Serial.println("File system init OK.");
  }
  else
  {
    Serial.println("File system init fail.");
  }

  // Open address file
  address_file = LittleFS.open("/MyAddress.txt", "r");
  if (address_file)
  {
    my_address = address_file.readString().toInt();
    address_file.close();
    Serial.print("Read my address of ");
    Serial.print(my_address);
    Serial.println(" from file system.");
  }
  else
  {
    Serial.println("Err: No myAddress.txt file present. Using default address of 8.");
    my_address = 8;
  }

  // Set our RadioHead addresses
  rf95.setHeaderFrom(my_address);
  rf95.setHeaderTo(10);   // Use base station's address for every packet
  rf95.setThisAddress(my_address);
  
  heartbeat_time = millis();

  Serial.println("Main Loop: sending heartbeats, waiting for button push");

  pixel.setPixelColor(0, COLOR_BLUE);
  pixel.show();

  next_heartbeat_time = millis();
  motor_end_time = 0;
  motor_fire = false;
  sync_time = 0;
  sync_time_local = 0;
  base_station_time = 0;
  btn_press_time_local = 0;
  btn_press_time_global = 0;
  last_pkt_red = false;

  // Toggle all debug outputs to make sure they're working
  digitalWrite(DBG0_PIN, HIGH);
  digitalWrite(DBG1_PIN, HIGH);
  digitalWrite(DBG2_PIN, HIGH);
  digitalWrite(DBG3_PIN, HIGH);
  digitalWrite(DBG0_PIN, LOW);
  digitalWrite(DBG1_PIN, LOW);
  digitalWrite(DBG2_PIN, LOW);
  digitalWrite(DBG3_PIN, LOW);
}

void loop() 
{
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
      // Serial.print("RX PKT Src ");
      // Serial.print(hc_src_addr);
      // Serial.print(" Dst ");
      // Serial.print(hc_dst_addr);
      // Serial.print(" status ");
      // Serial.println(hc_status);
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
}
