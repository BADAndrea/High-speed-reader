/*
Name:		ESP_HighSpeedPressure 2.0.ino
Created:	08/05/2019 15:22:27
Author:		Andrea Storoni (andrea.storoni@gmail.com)
*/

//Objective: read 4-20ma signal at high speed -> ESP8266 + MCP3008 in order to have 2 channels at 7 kHz/ch approximately (10 bit),
//data is reported via websocket.
//V2.1 uses MCP3202, smaller footprint, 12bit

#define V2_1
#include <FS.h> //SPIFFS library
#include <ESP8266WiFi.h>
#include <WebSocketsServer.h>
#include <ESP8266WebServer.h>

#ifdef V2_1
#include <SPI.h>
#else
#include <Adafruit_MCP3008.h>
#endif

//#define DEBUG
#ifdef DEBUG
#define DEBUG_PRINTLN(x) Serial.println(x)
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTF2(x, y) Serial.printf(x, y)
#define DEBUG_PRINTF3(x, y, z) Serial.printf(x, y, z)
#define DEBUG_PRINTF7(x, y, z, a, b, c, d) Serial.printf(x, y, z, a, b, c, d)
#define SERIAL_BEGIN(x) Serial.begin(x)
#else
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINT(x)
#define SERIAL_BEGIN(x)
#define DEBUG_PRINTF2(x, y)
#define DEBUG_PRINTF3(x, y, z)
#define DEBUG_PRINTF7(x, y, z, a, b, c, d)
#endif

constexpr char ssid[] = "HSR-2";
constexpr char password[] = "12345678";
constexpr int TP_Enabled = 2; //number of channels enabled
constexpr int vector_length = 4000 * TP_Enabled + 1; //4000 readings for 2 channels + 1 value for duration

int iteration = 0;
unsigned int min_time_between_ADC = 1000; //us, starting value
char buffer[5];
unsigned long last_data_point = 0;
unsigned int start_acquisition_time = 0;
int data_acquisition_timespan = 0;

bool reading = false;
int16_t y_values_1[vector_length];
size_t values_vector_length;

#ifdef V2_1
#define CS_3202 D8 //enable pin for SPI interface
#else
Adafruit_MCP3008 MCP3008;
#endif // MCP3202

ESP8266WebServer server(80); //Create a webserver object that listens for HTTP request on port 80
WebSocketsServer webSocket(81); //Create a websocket object that listens for ws request on port 81

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
	DEBUG_PRINTF3("webSocketEvent(%d, %d, ...)\r\n", num, type);
	switch (type) {
	case WStype_DISCONNECTED:
		DEBUG_PRINTF2("[%u] Disconnected!\r\n", num);
		break;
	case WStype_CONNECTED:
#ifdef DEBUG
		IPAddress ip = webSocket.remoteIP(num);
		DEBUG_PRINTF7("[%u] Connected from %d.%d.%d.%d url: %s\r\n", num, ip[0], ip[1], ip[2], ip[3], payload);
#endif
		itoa(num, buffer, 10);
		webSocket.sendTXT(num, (const char*)"ID", strlen("ID"));
		webSocket.sendTXT(num, (const char*)buffer, strlen(buffer));
		if (reading) {
			webSocket.sendTXT(num, (const char*)"start", strlen("start")); //send to newly connected client update on current esp state
		}
		else {
			webSocket.sendTXT(num, (const char*)"stop", strlen("stop"));
		}
		itoa(min_time_between_ADC, buffer, 10);
		webSocket.sendTXT(num, (const char*)buffer, strlen(buffer));
		break;
	case WStype_TEXT: //deal with commands received via websocket
		DEBUG_PRINTF3("[%u] get Text: %s\r\n", num, payload);
		if (strcmp("start", (const char *)payload) == 0) {
			DEBUG_PRINTLN("Start");
			start_acquisition_time = millis();
			reading = true;
			webSocket.broadcastTXT((const char*)"start", strlen("start"));
		}
		else if (strcmp("stop", (const char *)payload) == 0) {
			DEBUG_PRINTLN("Stop");
			reading = false;
			webSocket.broadcastTXT((const char*)"stop", strlen("stop"));
		}
		else if (strcmp("plusfifty", (const char *)payload) == 0) {
			min_time_between_ADC += 50;
			DEBUG_PRINT("Minimum time between ADC readings: ");
			DEBUG_PRINT(min_time_between_ADC);
			DEBUG_PRINTLN(" us");
			itoa(min_time_between_ADC, buffer, 10);
			webSocket.broadcastTXT((const char*)buffer, strlen(buffer));
		}
		else if (strcmp("minusfifty", (const char *)payload) == 0) {
			min_time_between_ADC -= 50;
			DEBUG_PRINT("Minimum time between ADC readings: ");
			DEBUG_PRINT(min_time_between_ADC);
			DEBUG_PRINTLN(" us");
			itoa(min_time_between_ADC, buffer, 10);
			webSocket.broadcastTXT((const char*)buffer, strlen(buffer));
		}
		else if (strcmp("disconnect", (const char*)payload) == 0) {
			DEBUG_PRINT("Disconnect all clients");
			webSocket.disconnect(); //does it works?
		}
		else if (strcmp("rt", (const char*)payload) == 0) {
			DEBUG_PRINT("RoundTrip packet");
			webSocket.broadcastTXT("rt", strlen("rt"));
		}
		else if (atoi((const char *)payload) >= 0) {
			min_time_between_ADC = atoi((const char *)payload);
			DEBUG_PRINT("Minimum time between ADC readings: ");
			DEBUG_PRINT(min_time_between_ADC);
			DEBUG_PRINTLN(" us");
			itoa(min_time_between_ADC, buffer, 10);
			webSocket.broadcastTXT((const char*)buffer, strlen(buffer));
		}
		break;
	case WStype_BIN:
		DEBUG_PRINTF3("[%u] get binary length: %u\r\n", num, length);
		break;
	default:
		DEBUG_PRINTF2("Invalid WStype [%d]\r\n", type);
		break;
	}
}

#ifdef V2_1
int Read3202(int CHANNEL, int CS) { //https://community.openenergymonitor.org/t/mcp3202-adc-spi-and-esp/5285/6
	int msb;
	int lsb;
	int commandBytes = B10100000;// channel 0
	if (CHANNEL == 1) commandBytes = B11100000; // channel 1
	digitalWrite(CS, LOW);
	SPI.transfer(B00000001);// Start bit
	msb = SPI.transfer(commandBytes);
	msb = msb & B00001111;
	lsb = SPI.transfer(0x00);
	digitalWrite(CS, HIGH);
	return ((int)msb) << 8 | lsb;
}
#endif

void UpdatePressureReadings(int16_t vector[]) { //write on same vector all sensor values
#ifdef V2_1
	vector[iteration] = Read3202(0, CS_3202);
	vector[iteration + 1] = Read3202(1, CS_3202);
#else
	vector[iteration] = MCP3008.readADCDifference(0);
	vector[iteration + 1] = MCP3008.readADCDifference(2);
	//vector[iteration + 2] = MCP3008.readADCDifference(4);
	//vector[iteration + 3] = MCP3008.readADCDifference(6);
#endif
	iteration += TP_Enabled;
}

void setup() {
	SERIAL_BEGIN(115200);
	DEBUG_PRINTLN("");

	values_vector_length = sizeof(y_values_1);
#ifdef V2_1
	pinMode(CS_3202, OUTPUT);
	digitalWrite(CS_3202, HIGH);
	SPI.begin();
	SPI.setClockDivider(SPI_CLOCK_DIV2);
#else
	MCP3008.begin(D8, D6, D7, D5); //connect to MCP3008
#endif

	WiFi.mode(WIFI_AP);
	WiFi.softAP(ssid, password); //Start Hot-Spot

	DEBUG_PRINT("IP address: ");
	DEBUG_PRINTLN(WiFi.softAPIP());

	SPIFFS.begin(); //Start the SPI Flash Files System

	
	server.onNotFound([]() { //If the client requests any URI
		server.send(404, "text/plain", "404: Not Found"); //otherwise, respond with a 404 (Not Found) error
	});
	
	server.serveStatic("/", SPIFFS, "/index.html", "max-age=2592000");
	server.serveStatic("/index.html", SPIFFS, "/index.html", "max-age=2592000");
	server.serveStatic("/canvasjs.min.js", SPIFFS, "/canvasjs.min.js", "max-age=2592000");
	server.serveStatic("/local.css", SPIFFS, "/local.css", "max-age=2592000");
	server.serveStatic("/settings.html", SPIFFS, "/settings.html", "max-age=2592000");
	server.begin(); //Actually start the server
	DEBUG_PRINTLN("HTTP server started");

	webSocket.begin();
	webSocket.onEvent(webSocketEvent);
}

void loop() {
	server.handleClient();
	webSocket.loop();

	if (reading) {
		if (micros() - last_data_point >= min_time_between_ADC) { //if enought time has passed, do next reading
			last_data_point = micros();
			UpdatePressureReadings(y_values_1);
		}

		if (iteration == vector_length - 1) { //if vector full, send data
			iteration = 0; //reset counter
			data_acquisition_timespan = millis() - start_acquisition_time;
			DEBUG_PRINT("Data acquisition lenght: ");
			DEBUG_PRINTLN(data_acquisition_timespan);
			if (data_acquisition_timespan <= 0) { //deal with overflow of millis (every 17 minutes?)
				data_acquisition_timespan = 4294967296 + data_acquisition_timespan;
			}
			y_values_1[vector_length - 1] = data_acquisition_timespan; //write time between first and last reading
			webSocket.sendBIN(0, (const uint8_t*)y_values_1, values_vector_length); //send data only to first client connected
			start_acquisition_time = millis();
		}
	}
}