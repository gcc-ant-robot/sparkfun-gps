#include <SPI.h>

#include <RH_RF95.h>

#include <Wire.h>

#include <Adafruit_GFX.h>

#include <Adafruit_SSD1306.h>

//#define BUFLEN(5 * RH_RF95_MAX_MESSAGE_LEN) //max size of data burst we can handle - (5 full RF buffers) - just arbitrarily large
#define BUFLEN 15000
#define RFWAITTIME 500 //maximum milliseconds to wait for next LoRa packet - used to be 600 - may have been too long

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(12, 6);

// LED is turned on at 1st LoRa reception and off when nothing else received. It gives an indication of how long the incoming data stream is.
#define LED 13

#define GPS_RADIO_BAUD 115200

////////////////// LCD Variables
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET 4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, & Wire, OLED_RESET);

// GPS Variables
#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_Ublox_GPS

SFE_UBLOX_GPS myGPS;
long last_screen = 0; //Simple local timer. Limits amount if I2C traffic to Ublox module.

void setup() {
    pinMode(LED, OUTPUT);

    // LCD SETUP
    // put your setup code here, to run once:
    Wire.begin(); // turn on I2C for GPS
    Wire.setClock(100000); //Increase I2C clock speed to 400kHz

    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
        SerialUSB.println(F("SSD1306 allocation failed"));
        for (;;); // Don't proceed, loop forever
    }
    display.clearDisplay();

    display.setTextSize(1); // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.setCursor(0, 0); // Start at top-left corner
    display.println("Initializing Radio...");
    display.display();

    SerialUSB.begin(115200);
    Serial1.begin(GPS_RADIO_BAUD);
    SerialUSB.println("Feather LoRa RX");

    while (!rf95.init()) {
        SerialUSB.println("LoRa radio init failed");
        while (1);
    }
    SerialUSB.println("LoRa radio init OK!");

    // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
    if (!rf95.setFrequency(RF95_FREQ)) {
        SerialUSB.println("setFrequency failed");
        while (1);
    }
    SerialUSB.print("Set Freq to: ");
    Serial.println(RF95_FREQ);

    // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

    // The default transmitter power is 13dBm, using PA_BOOST.
    // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
    // you can set transmitter powers from 5 to 23 dBm:
    rf95.setTxPower(15, false);

    display.println("Initializing GPS...");
    display.display();
    if (myGPS.begin() == false) //Connect to the Ublox module using Wire port
    {
        SerialUSB.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
        while (1);
    }

    myGPS.hardReset();
    delay(10000);
    Serial1.begin(GPS_RADIO_BAUD);
    if (myGPS.begin() == false) { //Connect to the Ublox module using Wire port 
        SerialUSB.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
        while (1);
    }
    myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
    myGPS.setSerialRate(GPS_RADIO_BAUD, COM_PORT_UART2); // Make sure that the UART2 port on the GPS is at 460800 Baud
    delay(1000);

    display.println("Setup Complete.");
    display.display();
    delay(1000);
}


uint8_t buf[BUFLEN];
unsigned buflen;

uint8_t rfbuflen; // lenth of recieved message
uint8_t * bufptr;
unsigned long lastTime, curTime;

void loop() {

    bufptr = buf;

    if (rf95.available()) {
        digitalWrite(LED, HIGH);
        rfbuflen = RH_RF95_MAX_MESSAGE_LEN;
        if(rf95.recv(bufptr, &rfbuflen)) {
            bufptr += rfbuflen;
            lastTime = millis(); // last time we got a packet...

            // Special thanks to ktrussell's Serial_to_Lora project, on which our packetization code is based:
            // https://github.com/ktrussell/Serial_to_LoRa

            while(((millis()-lastTime) < RFWAITTIME) && ((bufptr-buf) < (BUFLEN-RH_RF95_MAX_MESSAGE_LEN))) {
                // here we're waiting to see if we get more lora packets in this burst.  the transmitter times out every 500 ms, or if we run out of space, then we have to push stuff to the array
                if (rf95.available()) {
                    // another lora packet recieved as part of this burst.
                    rfbuflen = RH_RF95_MAX_MESSAGE_LEN;
                    if(rf95.recv(bufptr, &rfbuflen)) {
                        bufptr += rfbuflen; // remember that the recv makes rfbuflen the lenght of the message
                        lastTime=millis();
                    } else {
                        SerialUSB.println("Recieve Failed");
                    }
                }
            }
        } else {
            SerialUSB.println("Recieve Failed");
        }
        // push data!
        buflen = bufptr - buf; // total bytes recieved
        Serial1.write(buf, buflen); // send to gps
        digitalWrite(LED, LOW);
        SerialUSB.print(buflen);
        SerialUSB.println(" Bytes Recieved.");
    }

    check_gps();
}

void check_gps() {

    //Query module only every second. Doing it more often will just cause I2C traffic.
    if (millis() - last_screen > 8000) {
        last_screen = millis(); //Update the timer

        display.clearDisplay();
        display.setCursor(0, 0); // Start at top-left corner

        byte fixType = myGPS.getFixType();
        display.print("Fix: ");
        if (fixType == 0) display.print(F("No fix"));
        else if (fixType == 1) display.print(F("Dead Reconing"));
        else if (fixType == 2) display.print(F("2D"));
        else if (fixType == 3) display.print(F("3D"));
        else if (fixType == 4) display.print(F("GNSS+Dead reckoning"));
        display.println();

        byte RTK = myGPS.getCarrierSolutionType();
        display.print("RTK: ");
        //    display.print(RTK);
        if (RTK == 0) display.print("No");
        if (RTK == 1) display.print(F("Float"));
        if (RTK == 2) display.print(F("Fixed"));
        display.println();

        uint32_t accuracy = myGPS.getHorizontalAccuracy(); // Convert from mm * 10^-1 to m
        // Convert the horizontal accuracy (mm * 10^-1) to a float
        float f_accuracy = accuracy;
        // Now convert to m
        f_accuracy = f_accuracy / 10000.0; // Convert from mm * 10^-1 to m
        display.print("Accuracy: ");
        display.print(f_accuracy, 4);
        display.println("m");
        //    display.print("");
        if (myGPS.getRELPOSNED() == true) {
            display.print(myGPS.relPosInfo.relPosLength/100);    
            SerialUSB.print("Rel Pos: ");
            SerialUSB.println(myGPS.relPosInfo.relPosLength/100);
            display.print("m | RSSI: ");
        } else {
            display.print("NA | RSSI: ");
        }

        display.println(rf95.lastRssi(), DEC);

        display.display();

    }
}