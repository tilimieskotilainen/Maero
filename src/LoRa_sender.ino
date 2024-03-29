/*
 *  Initial test code for Maero, sending side
 * 
 *  Adapted from code by Aaron.Lee from HelTec AutoMation, ChengDu, China
 *  https://github.com/HelTecAutomation/Heltec_ESP32
 */
 
/*
 * Includes
 */

#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <Wire.h>  
#include "HT_SSD1306Wire.h"

#include "DHT.h"

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


/************** AM2302 ********************/
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321¨
#define DHTPIN 46
DHT dht(DHTPIN, DHTTYPE);

 /*
  * Variables
  */

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

static RadioEvents_t RadioEvents;

//double txNumber;
unsigned int txNumber;
bool lora_idle = true;
String packet;


void OnTxDone(void);
void OnTxTimeout(void);

 /*
  * Methods
  */

void lora_init(void){
  Mcu.begin();
  txNumber = 0;
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000); 
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
  
  // AM2302 
  dht.begin();
  float temperatureC = dht.readTemperature();
  float humidity = dht.readHumidity();
  Serial.print("Sensor: ");
  Serial.print(temperatureC);
  Serial.print ("°C ");
  Serial.print(humidity);
  Serial.println("%");

  factory_display.drawString(0, 0, "Preparing to send LoRa message");
  factory_display.drawString(0, 10, "" + String(temperatureC, 1) + " °C " + String(humidity, 1) + " %");
  factory_display.display();
  delay(1000);
	
  factory_display.clear();
  digitalWrite(LED, LOW);  
}

void loop(){
  if(lora_idle == true){
    delay(5000);
    //txNumber += 0.01;
    txNumber += 1;
    //sprintf(txpacket,"Test packet number %0.2f",txNumber);  //start a package

    // AM2302 LISÄYS
    float temperatureC = dht.readTemperature();
    float humidity = dht.readHumidity();
    Serial.print("Sensor: ");
    Serial.print(temperatureC);
    Serial.print ("°C ");
    Serial.print(humidity);
    Serial.println("%");
    
    sprintf(txpacket,"%0.1f %0.1f%",temperatureC, humidity);  //start a package
    Serial.printf("\r\nSending packet \"%s\" , length %d\r\n",txpacket, strlen(txpacket));
    digitalWrite(LED, HIGH); 
    factory_display.clear();
    factory_display.drawString(0, 0, "Sending packet " + String(txNumber) +".");
    // DALLAS LISÄYS
    factory_display.drawString(0, 10, "" + String(temperatureC, 1) + "°C " + String(humidity, 1) + "%");
    factory_display.display();
    
    Radio.Send((uint8_t *)txpacket, strlen(txpacket)); //send the package out	
    lora_idle = false;
    digitalWrite(LED, LOW); 
  }
  Radio.IrqProcess();
}

void OnTxDone(void){
  Serial.println("TX done......");
  lora_idle = true;
}

void OnTxTimeout(void){
  Radio.Sleep( );
  Serial.println("TX Timeout......");
  lora_idle = true;
}
