#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RH_RF95.h>

Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);



// Data Packet to Send
unsigned char loraData[9];

// How many times data transfer should occur, in seconds
const unsigned int sendInterval = 25;

// Pinout for Adafruit Feather M0 LoRa
//TinyLoRa lora = TinyLoRa(3, 8, 4);
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3


#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

unsigned long delayTime;


#define BUTTON_A  9
#define BUTTON_B  6
#define BUTTON_C  5

#define VBATPIN A7

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 905.5

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

void setup() {

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

    Serial.begin(9600);
    while(!Serial);    // time to get serial running
   Serial.println("OLED FeatherWing test");
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C for 128x32

  Serial.println("OLED begun");

  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.display();
  delay(1000);

  // Clear the buffer.
  display.clearDisplay();
  display.display();

    Serial.println(F("BME280 test"));

      // Initialize pin LED_BUILTIN as an output
        pinMode(LED_BUILTIN, OUTPUT);

// Initialize LoRa
Serial.println("Feather LoRa TX Test!");

// manual reset
digitalWrite(RFM95_RST, LOW);
delay(10);
digitalWrite(RFM95_RST, HIGH);
delay(10);

while (!rf95.init()) {
  Serial.println("LoRa radio init failed");
  Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
  while (1);
}
Serial.println("LoRa radio init OK!");

// Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
if (!rf95.setFrequency(RF95_FREQ)) {
  Serial.println("setFrequency failed");
  while (1);
}
Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

// you can set transmitter powers from 5 to 23 dBm:
rf95.setTxPower(23, false);

    unsigned status;

    // default settings
    status = bme.begin();
    // You can also pass in a Wire library object like &Wire2
    // status = bme.begin(0x76, &Wire2)
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        while (1) delay(10);
    }

    Serial.println("-- Default Test --");
    delayTime = 1000;

          // text display tests
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println("Bartos Media");
    display.println("LoRaWAN Sensor");
    display.println("Temp, humidity, pressure");
    display.setCursor(0,0);
    display.display(); // actually display all of the above

    delay(5000);

    display.clearDisplay();
    display.display();

    Serial.println();
}


void loop() {
    printValues();
    delay(delayTime);
    loraStuff();
    battVolt();
}
//this sends the data to lora, and updates the OLED display!
void loraStuff() {
    float h = bme.readHumidity();
  // Read temperature as Celsius (the default)
  float t = bme.readTemperature();
  float p = bme.readPressure() / 100.0F;
  float v = analogRead(VBATPIN);
  v *= 2;    // we divided by 2, so multiply back
  v *= 3.3;  // Multiply by 3.3V, our reference voltage
  v /= 1024; // convert to voltage
  Serial.print("Data being sent to TTN: ");
  Serial.print("Humidity: ");
  Serial.print(h);
  Serial.print(" %\t");
  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.print("Pressure hPa");
  Serial.print(p);
  Serial.println("");

  display.clearDisplay();
  display.display();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.print(t);
  display.println(" *C");
  display.print(h);
  display.println(" %");
  display.print(p);
  display.println(" hPa");
  display.display();
   // encode float as int
  int16_t tempInt = round(t * 100);
  int16_t humidInt = round(h * 100);
  int16_t pressInt = round(p *10);
  int16_t voltInt = round(v * 100);
    Serial.println(tempInt);
      Serial.println(humidInt);
  Serial.println(pressInt);
  Serial.println(voltInt);

  // encode int as bytes
  //byte payload[2];
  loraData[0] = 0x01;
  loraData[1] = highByte(tempInt);
  loraData[2] = lowByte(tempInt);

  loraData[3] = highByte(humidInt);
  loraData[4] = lowByte(humidInt);

  loraData[5] = highByte(pressInt);
  loraData[6] = lowByte(pressInt);

  loraData[7] = highByte(voltInt);
  loraData[8] = lowByte(voltInt);

    Serial.println("Sending LoRa Data...");
    display.println("packet sent!");
    display.display();
  rf95.send(loraData, sizeof(loraData));
    Serial.println("Waiting for packet to complete..."); 
  delay(10);
  rf95.waitPacketSent();


  // blink LED to indicate packet sent
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.println("delaying...");
  delay(sendInterval * 1000);
}

void printValues() {
    Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());
    Serial.println(" *C");

    Serial.print("Pressure = ");

    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(" hPa");

    Serial.print("Approx. Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");

    Serial.print("Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");

    Serial.println();
}

void battVolt() {
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  Serial.print("VBat: " ); Serial.println(measuredvbat);
  display.clearDisplay();
  display.display();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("Battery Voltage:");
  display.print(measuredvbat);
  display.println(" volts");
  display.println("LoRa CH: 6");
  display.println("Made by: Bartos Media");
  display.display();
  delay(5000);
}
