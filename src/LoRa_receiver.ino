/*
 *  Initial test code for Maero, receiving side
 *  Adapted from code by Aaron.Lee from HelTec AutoMation, ChengDu, China
 *  https://github.com/HelTecAutomation/Heltec_ESP32
 */
 
 /*
  * Includes
  */

#include "Arduino.h"
#include "LoRaWan_APP.h"
#include <Wire.h>  
#include "HT_SSD1306Wire.h"
#include "WiFi.h"
#include <HTTPClient.h>

 /*
  * LoRa setup
  */

/********************************* lora  *********************************************/
#define RF_FREQUENCY                                868000000 // Hz

#define TX_OUTPUT_POWER                             14        // dBm

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 30 // Define the payload size here
/********************************* lora  *********************************************/

/*
 * Variables
 */

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

static RadioEvents_t RadioEvents;

int16_t txNumber;
bool sleepMode = true;   //lora_idle
int16_t Rssi, rxSize;      // rssi,rxSize;
String packet ;

// Network credentials
const char* ssid     = "xxx";
const char* password = "xxx";
String httpaddress = "xxx";
String param = "";
String sensor_name = "Lora_sensor";

/*
 * Methods
 */

void lora_init(void){
  Mcu.begin();
  txNumber = 0;
  Rssi = 0;
  RadioEvents.RxDone = OnRxDone;

  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                 LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                 LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                 0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
}

//Make wifi connection
void connectToWifi(){
  // Connect to Wi-Fi network with SSID and password
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  delay(1000);
  Serial.print("Connecting to ");
  Serial.print(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    Serial.print(".");
  }
  Serial.println("");
  
  // Print local IP address and start web server
  Serial.print("WiFi connected. IP address: ");
  Serial.println(WiFi.localIP());
}

void send_sensor_readings(float t, float h){
  if(WiFi.status()== WL_CONNECTED){
    HTTPClient http;
    param = "temp=" + String(t,1) +"&hum=" + String(h,1) + "&name=" + sensor_name;
    String serverPath = httpaddress + "?" + param;
    http.begin(serverPath.c_str());
    int httpResponseCode = http.GET();
    if (httpResponseCode>0) {
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);
      String payload = http.getString();
      Serial.println(payload);
    }
    else {
      Serial.print("Error code: ");
      Serial.println(httpResponseCode);
    }
     // Free resources
    http.end();
  }
  else{}
}

 /*
  * OLED screen
  */

SSD1306Wire  factory_display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED); // addr , freq , i2c group , resolution , rst

 /*
  * Setup and loop
  */

void setup(){
	Serial.begin(115200);
	factory_display.init();
	factory_display.clear();
  pinMode(LED, OUTPUT);
	digitalWrite(LED, HIGH); 
  delay(1000);

	lora_init();
	packet = "System initialized, waiting";
  factory_display.drawString(0, 0, packet);
  factory_display.display();
  delay(1000);
  factory_display.clear();
	digitalWrite(LED, LOW);  
}

void loop(){
  if(sleepMode){
    sleepMode = false;
    factory_display.clear();
    factory_display.drawString(0, 0, "Into RX mode");
    factory_display.drawString(0, 10, "Waiting for next packet");
    factory_display.display();
	  digitalWrite(LED, LOW);  
    Serial.println("Into RX mode");
    Radio.Rx(0);
  }
  Radio.IrqProcess();
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr){
  Rssi = rssi;
  rxSize = size;
  memcpy(rxpacket, payload, size );
  rxpacket[size]='\0';
  Radio.Sleep( );
  
  // Handle package
  String stringpacket = String(rxpacket);
  float temp = stringpacket.substring(0,4).toFloat();
  float hum = stringpacket.substring(5,9).toFloat();
  connectToWifi();
  send_sensor_readings(temp, hum);

  factory_display.clear();
  factory_display.drawString(0, 0, "Packet received!");
  //factory_display.drawString(0, 10, rxpacket);
  factory_display.drawString(0, 10, String(temp) + "°C "+ String(hum) + "%");
  factory_display.drawString(0, 30, "RSSI: "+(String)Rssi + " length " + (String)rxSize);
  //factory_display.drawString(0, 40, "Waiting for next packet");
  factory_display.display();
  Serial.printf("\r\nReceived packet \"%s\" with Rssi %d , length %d\r\n", rxpacket, Rssi, rxSize);
  digitalWrite(LED, HIGH);  
  delay(3000);
  sleepMode = true;
}
