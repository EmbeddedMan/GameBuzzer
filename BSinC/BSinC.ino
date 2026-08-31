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

The LCD screen uses a ST7796S display controller and a FT6336U touch controller.

****** NOTE 1 *********

In order for the SPI bus that talks to the LCD screen to max out at 62.5MHz, you must
overclock the MCU core speed to 250 Mhz. This is done in the Tools->CPU Speed->250 MHZ
setting in the Arduino IDE.

****** NOTE 2 *********

To convert a PNG image into a bitmap format that can be directly read into the display (RGB656),
use the following image magick command:
magick "./../origonal images/Buzzer Activated.png" -filter Mitchell -resize 480x -flip -type TrueColor -depth 16 -compress none -define bmp:subtype=RGB565 BuzzerActivated.bmp
The -flip option is necessary because Adafruit_GFX reads bitmap images from the lower left rather than the upper left.
Note that this command line as listed above resizes the image and uses the Mitchell filter for doing so.

****** NOTE 3 *********
The pins on the LCD header go as follows (from bottom to top)
- VCC
- GND
- LCD_CS
- LCD_RST
- LCD_RS
- SDI (MOSI)
- SCK
- LED
- SDO (MISO)
- CTP_SCL
- CTP_RST
- CTP_SDA
- CTP_INT
- SD_CS

(Pinout of Feather RP2040 RFM in CircuitPython)
board.A0 (GPIO26)   DBG0
board.A1 (GPIO27)   DBG1
board.A2 (GPIO28)   DBG2
board.A3 (GPIO29)   DBG3
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
#include <FT6336U.h>
#include <LittleFS.h>
#include <string.h>
#include <hardware/clocks.h> // Required for clock configuration functions

#define FileSys LittleFS

#define SCREEN_WIDTH    480
#define MAX_IMAGE_WIDTH SCREEN_WIDTH
#define SCREEN_HEIGHT   320
int16_t xpos = 0;
int16_t ypos = 0;

/*            BASE STATION PIN MAP                */
#define BOARD_TX        0   // Board defined UART TX (unused, breakout)
#define BOARD_RX        1   // Board defined UART RX (unused, breakout)
#define BOARD_SDA       2   // Board defined I2C SDA (LCD touch screen)
#define BOARD_SCL       3   // Board defined I2C SCL (LCD touch screen)
#define NEOPIXEL_PIN    4   // Board defined Neopixel data output
#define TOUCH_N_RST     5   // LCD touch controller reset (breakout)
#define NEOHAIN_PIN     6   // Chain of Neopixels around box edge (breakout)
#define BUTTON2_PIN     7   // Boot button on Feather
#define BOARD_MISO      8   // Board defined MISO for SPI1 - used by radio
#define TOUCH_N_INT     9   // LCD touch controller interrupt (breakout)
#define BUTTON1_PIN     10  // Red game reset pushbutton (breakout)
#define BEEPER_PIN      11  // Beeper control output (breakout)
#define BOARD_D12       12  // Unused (breakout)
#define BOARD_D13       13  // Unused (breakout)
#define BAORD_SCK       14  // Board defined SCK for SPI1- used by radio and LCD (breakout)
#define BOARD_MOSI      15  // Board defined MOSI for SPI1 - used by radio and LCD (breakout)
#define RFM95_CS        16  // Radio SPI1 Chip Select output
#define RFM95_RST       17  // Radio Reset output
#define RFM95_IO5       18  // Radio GPIO
#define RFM95_IO3       19  // Radio GPIO
#define RFM95_IO4       20  // Radio GPIO
#define RFM95_INT       21  // Radio Interrupt input (RFM_IO0 on schematic)
#define RFM95_IO1       22  // Radio GPIO
#define RFM95_IO2       23  // Radio GPIO
#define TFT_DC          24  // LCD Data/Command output (breakout)
#define TFT_CS          25  // LCD SPI1 Chip Select (breakout)
#define TFT_RST         -1  // Connected, but I don't know to what GPIO pin
#define DBG0_PIN        26  // General Purpose Debug Output (breakout)
#define DBG1_PIN        27  // General Purpose Debug Output (breakout)
#define DBG2_PIN        28  // General Purpose Debug Output (breakout)
#define DBG3_PIN        29  // General Purpose Debug Output (breakout)

#define RF95_FREQ       915.0
#define BASE_STATION_RH_ADDRESS 10    // Base Station radio address

#define TIME_SYNC_PACKET_PERIOD_MS  140 // Number of milliseconds between time sync packets - sets overall repeition rate for entier system

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

// LCD display object : use hardware SPI for LCD
Adafruit_ST7796S_kbv tft = Adafruit_ST7796S_kbv(TFT_CS, TFT_DC, TFT_RST);

// LCD touch controller object
FT6336U ft6336u(TOUCH_N_RST, TOUCH_N_INT);

// Base Station global variables
bool base_station_is_reset;
uint8_t packet[10];
uint8_t sync_pkt[10];
uint32_t reset_start_time;
uint32_t button_push_times[8];
uint32_t heartbeat_times[8];
uint8_t hc_btn_order[8];
uint8_t hc_seen_reset[8];       // When BS button pushed, remember HC with buttons pushed. Only clear those once they send back unpushed packets.
bool any_btn_pushed;
uint32_t beeper_off_time;
uint32_t next_sync_time;
uint32_t hc_btn_push_time;
uint32_t hc_btn_push_time_ms;
uint32_t packet_rx_resume_time;
uint32_t btn_press_time;
bool old_btn;
uint32_t sync_time_ms;
uint32_t time_bytes_ms;
uint8_t packet_len;
uint8_t hc_dst_addr;
uint8_t hc_src_addr;
bool user_touch_happened;

void setup() 
{
  // 1. FORCE the peripheral clock to run at the full 250 MHz CPU speed
  clock_configure(
    clk_peri,
    0, // No auxiliary mux changes
    CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS, // Source from system clock
    rp2040.f_cpu(), // Match the current CPU frequency (250000000 Hz)
    rp2040.f_cpu()  // Match the integer division rate
  );

  // Set up debug outputs
  pinMode(TOUCH_N_RST, OUTPUT);
  pinMode(TOUCH_N_INT, INPUT_PULLUP);
  
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

  Serial.printf("RP2040 F_CPU = %u\n", rp2040.f_cpu());

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
  rf95.setTxPower(10, false);
  rf95.setSpreadingFactor(7);
  rf95.setSignalBandwidth(500000);
  rf95.setThisAddress(10);

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
  tft.begin(62500000);
  Serial.printf("Actual SPI bus speed = %u\n", spi_get_baudrate(spi1));

  // And display splash screen
  tft.setRotation(1);
  tft.invertDisplay(true);

//  tft.fillScreen(ST7796S_WHITE);
//  tft.setCursor(140, 140);
//  tft.setTextColor(ST7796S_BLACK);  
//  tft.setTextSize(1);
//  tft.println("Book Club");
//  delay(1000);

  if (!LittleFS.begin()) {
    Serial.println("LittleFS init failed");
    while(1);
  }

  Serial.println("Listing LittleFS root directory:");
  
  // Open root directory
  File root = LittleFS.open("/", "r");
  if (!root || !root.isDirectory()) {
    Serial.println("Failed to open root directory");
    return;
  }

  // Iterate through all files and directories
  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR:  ");
      Serial.println(file.name());
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("\tSIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
  
  // Read in and display the splash screen
  draw_bmp("/BookClubSplash.bmp", 0, 0);

  pinMode(TOUCH_N_INT, INPUT_PULLUP);

  // Init the touch controller
  ft6336u.begin();

  pinMode(TOUCH_N_INT, INPUT_PULLUP);
  Serial.print("FT6336U Firmware Version: ");
  Serial.println(ft6336u.read_firmware_id());
  Serial.print("FT6336U Device Mode: ");
  Serial.println(ft6336u.read_device_mode());

  attachInterrupt(digitalPinToInterrupt(TOUCH_N_INT), touch_ISR, FALLING);

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

  draw_bmp("/NextQuizQuestionResized.bmp", 0, 0);

  // Blank the LCD and display green background
  //tft.fillScreen(ST7796S_GREEN);

  //tft.setCursor(140, 120);
  //tft.setTextColor(ST7796S_BLACK);  
  //tft.setTextSize(4);
  //tft.println("Next Quiz");
  //tft.setCursor(150, 160);
  //tft.println("Question");
}

// Called whenever there is a falling edge on the touch controller's interrupt line
void touch_ISR(void)
{
  user_touch_happened = true;
}
  
void loop() 
{
  uint32_t i;

  digitalWrite(DBG2_PIN, HIGH);

  if(user_touch_happened) {
    user_touch_happened = false;
    if (ft6336u.read_td_status())
    {
      //Serial.print("FT6336U Touch Event/ID 1: (");
      //Serial.print(ft6336u.read_touch1_event()); Serial.print(" / "); Serial.print(ft6336u.read_touch1_id()); Serial.println(")");
      //Serial.print("FT6336U Touch Position 1: (");
      Serial.printf("\nTouch at %3u,%3u", ft6336u.read_touch1_x(), ft6336u.read_touch1_y());
      //Serial.print("FT6336U Touch Weight/MISC 1: (");
      //Serial.print(ft6336u.read_touch1_weight()); Serial.print(" / "); Serial.print(ft6336u.read_touch1_misc()); Serial.println(")");
      //Serial.print("FT6336U Touch Event/ID 2: (");
      //Serial.print(ft6336u.read_touch2_event()); Serial.print(" / "); Serial.print(ft6336u.read_touch2_id()); Serial.println(")");
      //Serial.print("FT6336U Touch Position 2: (");
      //Serial.print(ft6336u.read_touch2_x()); Serial.print(" , "); Serial.print(ft6336u.read_touch2_y()); Serial.println(")");
      //Serial.print("FT6336U Touch Weight/MISC 2: (");
      //Serial.print(ft6336u.read_touch2_weight()); Serial.print(" / "); Serial.print(ft6336u.read_touch2_misc()); Serial.println(")");
    }
  }


  if (beeper_off_time)
  {
    if (millis() >= beeper_off_time)
    {
      beeper_off_time = 0;
      // Turn the beeper off
      digitalWrite(BEEPER_PIN, LOW);
    }
  }

  // Look for button press to reset our state
  if (digitalRead(BUTTON1_PIN) == false || digitalRead(BUTTON2_PIN) == false)
  {
    // We have a button press!
    // Record the local time
    btn_press_time = millis();
    
    rf95.setModeIdle();

    base_station_is_reset = true;
    memset(button_push_times, 0x00, sizeof(button_push_times));
    memset(heartbeat_times, 0x00, sizeof(heartbeat_times));
    memset(hc_btn_order, 0x00, sizeof(hc_btn_order));

    pixel.setPixelColor(0, COLOR_GREEN);
    pixel.show();
    Serial.print(millis());
    Serial.println(" System is now reset");

    // Blank the LCD and display green background
    //tft.fillScreen(ST7796S_GREEN);

    //tft.setCursor(140, 120);
    //tft.setTextColor(ST7796S_BLACK);  
    //tft.setTextSize(4);
    //tft.println("Next Quiz");
    //tft.setCursor(150,  160);
    //tft.println("Question");
    draw_bmp("/NextQuizQuestionResized.bmp", 0, 0);
  
    any_btn_pushed = false;

    // Figure out which HCs have sent button pushed packets. For those HCs, set the hc_seen_reset to false. For all others
    // set it to true. When we see a non-button push packet from a HC, we then set it's hc_seen_reset. Once they are all
    // true, we know that all HCs have been 'reset', and we can finish the reset cycle and start the next game up.
    for (i=0; i < 8; i++)
    {
      if (button_push_times[i] > 0)
      {
        hc_seen_reset[i] = false;
      }
      else
      {
        hc_seen_reset[i] = true;
      }
    }

    // Set blanking time to ignore any hand controller packets for 1.5s
    /// TODO: We can make this smarter, right? We can wait for every handle to turn green, then turn off the blanking
    packet_rx_resume_time = millis() + 5000;
    // Do not reset the sync time I think - hand controller rely on this being very constant and not changing
    // next_sync_time = millis() + 1110;
  }

  //delay(1);

  // Has enough time gone by? Time to send a sync packet?
  if (millis() >= next_sync_time)
  {
    next_sync_time = millis() + TIME_SYNC_PACKET_PERIOD_MS;
    rf95.setHeaderFrom(10);
    rf95.setHeaderTo(255); // Broadcast to all hand controllers
    // Build up status byte based on each hand controller's state

    if (any_btn_pushed)
    {
      sync_pkt[0] = 255;
    }
    else
    {
      sync_pkt[0] = 0;
    }
    // Copy over the current global time as four bytes
    sync_time_ms = millis();
    sync_pkt[1] = (sync_time_ms >> 24) & 0xFF;
    sync_pkt[2] = (sync_time_ms >> 16) & 0xFF;
    sync_pkt[3] = (sync_time_ms >> 8) & 0xFF;
    sync_pkt[4] = sync_time_ms & 0xFF;

    // If there is a packet waiting in the radio, flush it before sending
    packet_len = 10;
    if (rf95.available())
    {
      digitalWrite(DBG0_PIN, HIGH);
      rf95.recv(packet, &packet_len);
      digitalWrite(DBG0_PIN, LOW);
    }
    /// // JUST FOR TESTING: Skip sending every 10th time sync packet
    ///static int8_t time_sink_skip = 10;
    ///if (time_sink_skip > 1)
    ///{
      digitalWrite(DBG1_PIN, HIGH);
      rf95.send(sync_pkt, 5);
      digitalWrite(DBG1_PIN, LOW);
    ///}
    ///time_sink_skip--;
    ///if (time_sink_skip <= 0)
    ///{
    ///  time_sink_skip = 10;
    ///}
    delay(2);
    Serial.println();
    if (any_btn_pushed)
    {
      Serial.print(millis());
      Serial.print(" Red   Sync sent: ");
      Serial.print(sync_time_ms);
    }
    else
    {
      Serial.print(millis());
      Serial.print(" Green Sync sent: ");
      Serial.print(sync_time_ms);
    }
  }

  digitalWrite(DBG2_PIN, LOW);

  // In a non-blocking way, look to see if we've received a packet
  packet_len = 10;
  if (rf95.available())
  {
    digitalWrite(DBG0_PIN, HIGH);
    if (rf95.recv(packet, &packet_len))
    {
      digitalWrite(DBG0_PIN, LOW);

      // Toggle the red LED on the board on each received packet
      if (digitalRead(LED_BUILTIN))
      {
        digitalWrite(LED_BUILTIN, LOW);
      }
      else
      {
        digitalWrite(LED_BUILTIN, HIGH);
      }

      // Check packet length
      if (packet_len == 4)
      {
        hc_dst_addr = rf95.headerTo();
        hc_src_addr = rf95.headerFrom();
        //headerId();
        hc_btn_push_time_ms = (packet[0] << 24) | (packet[1] << 16) | (packet[2] << 8) | packet[3];

        if (hc_src_addr > 0 && hc_src_addr <= 8) // Pkt must come from HC addressed 1 through 8
        {
          if (hc_dst_addr == 10)  // and it must come to us, base station, addr 10
          {
            if (hc_btn_push_time_ms == 0) // If time = 0, this is a heartbeat packet, no button push
            {
              heartbeat_times[hc_src_addr - 1] = millis();
              hc_seen_reset[hc_src_addr - 1] = true;
              Serial.print(" $ ");
              Serial.print(hc_src_addr);
              Serial.print(" : hb ");
            }
            else
            {
              // Have we timed out of any ongoing blanking period?
              if (packet_rx_resume_time < millis())
              {
                packet_rx_resume_time = 0;
              }
              // If we are in a blanking period after a system reset (BS button push), then check to see
              // if all of the HC that had button pushes have checked in with 'no button push' packets. If not,
              // then ignore this packet.
              if (packet_rx_resume_time)
              {
                bool any_waiting_for_reset = false;
                for (i=0; i < 8; i++)
                {
                  if (hc_seen_reset[i] == false)
                  {
                    any_waiting_for_reset = true;
                  }
                }
                if (any_waiting_for_reset == false)
                {
                  // All HCs with button pushes have checked in with plain heartbeat packets. So we don't need to
                  // be in a blanking period anymore
                  packet_rx_resume_time = 0;
                }
              }

              if (!packet_rx_resume_time)
              {
                // We got a button push packet from a hand controller
                // Is this the first button press of any of the hand controllers for this question?
                if (any_btn_pushed == false)
                {
                  // Yes, then start the bepper up
                  //digitalWrite(BEEPER_PIN, HIGH);
                  any_btn_pushed = true;
                  beeper_off_time = millis() + 2000;
                }
                // Only do stuff if this is the very first button press packet from this hand controller for this question
                if (button_push_times[hc_src_addr - 1] == 0)
                {
                  rf95.setModeIdle();

                  Serial.print(" $ ");
                  Serial.print(hc_src_addr);
                  Serial.print(" : bp ");
                  Serial.print(hc_btn_push_time_ms);
                  button_push_times[hc_src_addr - 1] = hc_btn_push_time_ms;

                  // Blank the LCD and display red background
                  tft.fillScreen(ST7796S_WHITE);
                  draw_bmp("/BuzzerActivated.bmp", 0, 0);
                  draw_bmp("/Ania.bmp", 10, 50);
                  draw_bmp("/Brian.bmp", 10, 92);
                  draw_bmp("/Emily.bmp", 10, 134);
                  draw_bmp("/Grant.bmp", 10, 176);
                  //tft.setCursor(0, 25);
                  //tft.setTextColor(ST7796S_WHITE);  
                  //tft.setTextSize(3);
                  //tft.println("   Player button pushes");
                  //tft.println("        in order:");

                  // Sort hand controllers in order that they pushed their buttons
                  // button_push_times[] is zero for a hand controller if they haven't pushed their button
                  // or a global millisecond since base station boot time if they have.
                  // Walk through all 8 times[], find the smallest one. Print out its index.
                  // Find the next smllest one, print it out. Until there are no more times left.
                  uint8_t outer, inner, smallest_index = 0;
                  bool printed[8] = {false, false, false, false, false, false, false, false};
                  uint32_t smallest_time = 0xFFFFFFFF;

                  Serial.println();
                  // Print out the times at start of sort
                  for (outer = 0; outer < 8; outer++)
                  {
                    Serial.print(outer);
                    Serial.print(":");
                    Serial.print(button_push_times[outer]);
                    Serial.println();
                  }

                  tft.setCursor(30, 90);
                  for (outer = 0; outer < 8; outer++)
                  {
                    smallest_time = 0;
                    Serial.print("> Outer = ");
                    Serial.print(outer);
                    // Find the outerith smallest push time that hasn't been printed yet
                    for (inner = 0; inner < 8; inner++)
                    {
                      if ((button_push_times[inner] != 0) && (printed[inner] != true))
                      {
                        if (button_push_times[inner] > smallest_time)
                        {
                          smallest_time = button_push_times[inner];
                          smallest_index = inner;
                        }
                      }
                    }
                    Serial.print(" smallest = ");
                    Serial.print(smallest_time);
                    Serial.print(" at index ");
                    Serial.println(smallest_index);
                    // Smallest time should now be shortest unpushed time, at index smallest_index
                    if (smallest_time != 0)
                    {
                      // Print out smallest_index
                      printed[smallest_index] = true;
                      tft.print(smallest_index + 1);
                      tft.print("  ");
                    }
                  }
                }
              }
            }
          }
          else
          {
            Serial.print(millis());
            Serial.print(" Got a packet with a bad destingation address of ");
            Serial.println(hc_dst_addr);
          }
        }
        else
        {
          Serial.print(millis());
          Serial.print(" Got a packet with a bad source address of ");
          Serial.println(hc_src_addr);
        }
      }
      else
      {
        Serial.print(millis());
        Serial.print(" Got a packet with a bad length of ");
        Serial.println(packet_len);
      }
      digitalWrite(DBG3_PIN, LOW);
    }
    digitalWrite(DBG0_PIN, LOW);
  }
}

// Custom bitmap drawing code, for this project, and this display.
// To create files that work, use the image magick command line at the top of this sketch.
//
// Pass in the filename of the bitmap file, and the upper left X and Y coordinates you want to draw it on the screen.
// Filename must have a leading forward slash on it.
// This function opens the file, reads it out line by line and writes it to the screen.
int32_t draw_bmp(const char * filename, uint16_t x_loc, uint16_t y_loc)
{
  uint16_t max_line[SCREEN_WIDTH] = {0};
  int32_t retval = 0;     // Start off with no errors
  uint16_t width = 0;
  uint16_t height = 0;

  if (!filename)
  {
    retval = -1;
    return(retval);
  }
  // Open image file from LittleFS (ensure leading slash)
  File imgFile = LittleFS.open(filename, "r");
  if (!imgFile) {
    Serial.printf("Failed to open image file %s\n", filename);
    retval = -1;
    return(retval);
  }

  // Read in the image from LittleFS. We don't have enough RAM to store a full framebuffer,
  // so we will read in each iamge one horizontal line at a time, draw that, then read the next, etc.
  // First we have to read past the BMP header bytes (168 bytes worth). This block contains the initial
  // file header (BITMAPFILEHEADER) - 14 bytes, extended information header (BITMAPV5HEADER) - 124 bytes
  imgFile.readBytes((char*)max_line, 138);

  // Confirm that we have a header of the right size. So we check the BitmapOffset field of the 14 byte header
  if ((max_line[5] + (max_line[6] << 16)) != 138)
  {
    Serial.printf("Got incorrect header size of %u, %u\n", max_line[5], max_line[6]);
    retval = -1;
    return(retval);
  }

  // Extract the image dimensions from the bitmap header. We don't have to be very smart about this
  // because we know exactly how each image got created and what bitmap header format it uses.
  width = max_line[9] + (max_line[10] << 16);
  height = max_line[11] + (max_line[12] << 16);

  // Check that our width and height are not larger than our screen
  if ((width > SCREEN_WIDTH) || (height > SCREEN_HEIGHT))
  {
    Serial.printf("Image too large to fit on screen.\n");
    retval = -1;
    return(retval);
  }

  // This tells the display we are going to feed it pixels for this rectangular area
  //tft.setAddrWindow(x_loc, y_loc, width, height);

  // Read out the image pixels, one horizontal line at a time, and place at the right point on the screen
  for (int h = 0; h < height; h++) 
  {
    //digitalWrite(DBG0_PIN, HIGH);
    imgFile.readBytes((char*)max_line, width * 2);
    //digitalWrite(DBG0_PIN, LOW);

    digitalWrite(DBG1_PIN, HIGH);
    tft.drawRGBBitmap(x_loc, y_loc + h, max_line, width, 1);
    digitalWrite(DBG1_PIN, LOW);
  }
  imgFile.close();
  return(retval);
}
