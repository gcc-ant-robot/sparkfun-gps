#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_Ublox_GPS

#include <Wire.h> //Needed for I2C to GPS

#include <SPI.h>

#include <RH_RF95.h>

#define LED 13

//////////////// LCD
#include <Adafruit_GFX.h>

#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET 4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, & Wire, OLED_RESET);

#define GPS_RADIO_BAUD 115200

///////////////////////
// RADIO VARIABLES
///////////////////////
#define RFM95_CS 12
#define RFM95_INT 6
//#define RTCM_START 0xd3
#define BUFLEN 20000 //max size of data burst we can handle
#define SER_TIMEOUT 100 //Timeout in millisecs for reads into buffer from serial - needs to be longer than bit time at our baud rate and any other delay between packets from GPS
// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0
#define PACKET_SIZE 250
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

///////////////////////
// GPS VARIABLES
///////////////////////
SFE_UBLOX_GPS myGPS;

void setup() {
	// put your setup code here, to run once:

	///////////////////////
	// GENERAL SETUP
	///////////////////////
	pinMode(LED, OUTPUT);
	for (int jj = 0; jj < 5; jj++) {
		digitalWrite(LED, HIGH);
		delay(50);
		digitalWrite(LED, LOW);
		delay(50);
	}

	SerialUSB.begin(115200);
	SerialUSB.println("Welcome to UBLOX -> RFM96 Base Station!");
	delay(100);
	//  Serial1.begin(GPS_RADIO_BAUD);
	Wire.begin(); // turn on I2C for GPS
	Wire.setClock(100000); //Increase I2C clock speed to 400kHz

	// LCD Setup
	if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
		Serial.println(F("SSD1306 allocation failed"));
		for (;;); // Don't proceed, loop forever
	}
	display.clearDisplay();

	display.clearDisplay();
	display.setTextSize(1); // Normal 1:1 pixel scale
	display.setTextColor(SSD1306_WHITE); // Draw white text
	display.setCursor(0, 0); // Start at top-left corner
	display.println(F("Setup in Progress..."));
	display.println(F("Resetting GPS..."));
	display.display();

	///////////////////////
	// RADIO SETUP
	///////////////////////
	SerialUSB.println("Feather LoRa TX");
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
	SerialUSB.println(RF95_FREQ);
	// Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

	// The default transmitter power is 13dBm, using PA_BOOST.
	// If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
	// you can set transmitter powers from 5 to 23 dBm:
	rf95.setTxPower(23, false);
	Serial1.setTimeout(SER_TIMEOUT);

	///////////////////////
	// GPS SETUP
	///////////////////////
	SerialUSB.println("Starting GPS Config");
	if (myGPS.begin() == false) { //Connect to the Ublox module using Wire port 
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
	myGPS.setUART1Output(COM_TYPE_RTCM3); // enable RTCM output
	myGPS.setUART2Output(0); // disable uart 2
    myGPS.setSerialRate(GPS_RADIO_BAUD, COM_PORT_UART1); // Make sure that the UART1 port on the GPS is at GPS_RADIO_BAUD Baud

	myGPS.saveConfiguration(); //Save the current settings to flash and BBR
	SerialUSB.println("Enabling RTCM on F9P UART1.");

	boolean gps_response = true;
	gps_response &= myGPS.enableRTCMmessage(UBX_RTCM_1005, COM_PORT_UART1, 10); //Enable message 1005 to output through I2C port, message every 5 second
	gps_response &= myGPS.enableRTCMmessage(UBX_RTCM_1074, COM_PORT_UART1, 1);
	gps_response &= myGPS.enableRTCMmessage(UBX_RTCM_1084, COM_PORT_UART1, 1);
	// gps_response &= myGPS.enableRTCMmessage(UBX_RTCM_1094, COM_PORT_UART1, 1); // GLONASS Not Necessary -> conserve bandwidth: // https://www.ardusimple.com/configuration-files/ does not send 1124
	// gps_response &= myGPS.enableRTCMmessage(UBX_RTCM_1124, COM_PORT_UART1, 1); // https://www.ardusimple.com/configuration-files/ does not send 1124
	gps_response &= myGPS.enableRTCMmessage(UBX_RTCM_1230, COM_PORT_UART1, 5); //Enable message every 10 seconds
	// Use COM_PORT_UART1
	// for the above six messages to direct RTCM messages out UART1
	// COM_PORT_UART1, COM_PORT_USB, COM_PORT_SPI are also available
	// For example: gps_response &= myGPS.enableRTCMmessage(UBX_RTCM_1005, COM_PORT_UART1, 10);

	if (gps_response == true) {
		SerialUSB.println("RTCM messages enabled");
	} else {
		SerialUSB.println("RTCM failed to enable. Are you sure you have an ZED-F9P?");
		while (1); //Freeze
	}

	//Check if Survey is in Progress before initiating one
	gps_response = myGPS.getSurveyStatus(2000); //Query module for SVIN status with 2000ms timeout (request can take a long time)
	if (gps_response == false) {
		SerialUSB.println("Failed to get Survey In status");
		while (1); //Freeze
	}
	if (myGPS.svin.active == true) {
		SerialUSB.print("Survey already in progress.");
	} else {
		//Start survey
		//The ZED-F9P is slightly different than the NEO-M8P. See the Integration manual 3.5.8 for more info.
		//gps_response = myGPS.enableSurveyMode(300, 2.000); //Enable Survey in on NEO-M8P, 300 seconds, 2.0m
		gps_response = myGPS.enableSurveyMode(60, 2.5); //Enable Survey in, 60 seconds, 5.0m
		if (gps_response == false) {
			SerialUSB.println("Survey start failed");
			while (1);
		}
		SerialUSB.println("Survey started. This will run until 60s has passed and less than 5m accuracy is achieved.");
	}

	while (SerialUSB.available()) SerialUSB.read(); //Clear buffe
	//Begin waiting for survey to complete
	while (myGPS.svin.valid == false) {
		if (SerialUSB.available()) {
			byte incoming = SerialUSB.read();
			if (incoming == 'x') {
				//Stop survey mode
				gps_response = myGPS.disableSurveyMode(); //Disable survey
				SerialUSB.println("Survey stopped");
				break;
			}
		}

		gps_response = myGPS.getSurveyStatus(2000); //Query module for SVIN status with 2000ms timeout (req can take a long time)
		if (gps_response == true) {
			display.clearDisplay();
			display.setCursor(0, 0); // Start at top-left corner
			display.println(F("Survey-in In Progress"));
			display.print("Time Elapsed: ");
			display.println((String) myGPS.svin.observationTime);
			display.print("Accuracy: ");
			display.print((String) myGPS.svin.meanAccuracy);
			display.display();

			SerialUSB.print("Press x to end survey - ");
			SerialUSB.print("Time elapsed: ");
			SerialUSB.print((String) myGPS.svin.observationTime);

			SerialUSB.print(" Accuracy: ");
			SerialUSB.print((String) myGPS.svin.meanAccuracy);
			SerialUSB.println();
		} else {
			SerialUSB.println("SVIN request failed");
			display.clearDisplay();
			display.println("SVIN request failed");
			display.display();
		}

		delay(1000);
	}
	SerialUSB.println("Survey valid!");
	SerialUSB.println("Base survey complete! RTCM now broadcasting.");

	display.clearDisplay();
	display.println("Survey Complete!");
	display.display();
	delay(1000);

	//  myGPS.setUART1Output(COM_TYPE_RTCM3); //Set the I2C port to output UBX and RTCM sentences (not really an option, turns on NMEA as well)
	SerialUSB.println("Done with Setup");
}

char progress[] = {
	'-',
	'\\',
	'|',
	'/'
};

uint8_t progress_ind;
uint8_t buf[BUFLEN];
int bytesRead;
long start_time;
int bytesSent;
int bytesLeft;
uint8_t * ptr;

void loop() {
	
	// Special thanks to ktrussell's Serial_to_Lora project, on which packetization code is based:
	// https://github.com/ktrussell/Serial_to_LoRa

	while (!Serial1.available());

	if (Serial1.available()) {
		digitalWrite(LED, HIGH);

		bytesRead = Serial1.readBytes(buf,BUFLEN);
		start_time = millis();

		SerialUSB.print(bytesRead);
		SerialUSB.println(" Bytes Read from GPS");

		bytesSent = 0;
		bytesLeft = bytesRead - bytesSent;

		ptr = buf; // where we are in the stack

		while (bytesLeft > 0) {
			if (bytesLeft > PACKET_SIZE) {
				rf95.send(ptr, PACKET_SIZE);
				ptr += PACKET_SIZE;
				bytesSent +=PACKET_SIZE;
			} else {
				rf95.send(ptr, bytesLeft);
				ptr += bytesLeft;
				bytesSent += bytesLeft;
			}
			bytesLeft = bytesRead - bytesSent;
		}

		SerialUSB.print("All Bytes Sent in ");
		SerialUSB.print(millis() - start_time);
		SerialUSB.print(" ms.\n\n");
		digitalWrite(LED, LOW);

	}

	display.clearDisplay();
	display.setCursor(0, 0); // Start at top-left corner
	display.println("Survey Complete!");
	display.print("Broadcasting RTCM ");
	display.println(progress[progress_ind]);
	display.print("Survey Accuracy ");
	display.print((String) myGPS.svin.meanAccuracy);
	display.println("m");
	display.print("Survey Time ");
	display.print((String) myGPS.svin.observationTime);
	display.println("s");
	progress_ind = (progress_ind + 1) % 4;
	display.display();
}
