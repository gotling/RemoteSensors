#include <ESP8266WiFi.h>
#include <espnow.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_AM2320.h"

#define LED D2

Adafruit_AM2320 am2320 = Adafruit_AM2320();

// REPLACE WITH RECEIVER MAC Address
//40:91:51:4F:ED:0B
uint8_t broadcastAddress[] = {0x40, 0x91, 0x51, 0x4F, 0xED, 0x0B};

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  int random;
  float temp;
  float humid;
} struct_message;

// Create a struct_message called myData
struct_message myData;

unsigned long lastTime = 0;  
unsigned long timerDelay = 10000;  // send readings timer
unsigned long ledTimeout = 2000;  // seconds to keep led on after successfull send

// Callback when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0) {
    Serial.println("Delivery success");
    digitalWrite(LED_BUILTIN, LOW);
  } else {
    Serial.println("Delivery fail");
  }
}
 
void setup() {
  // Init Serial Monitor
  Serial.begin(9600);

  while (!Serial) {
    delay(10); // hang out until serial port opens
  }

  Wire.begin(4, 0);
  am2320.begin();

  pinMode(LED_BUILTIN, OUTPUT);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
}
 
void loop() {
  // Turn of LED after set time
  if ((millis() - lastTime) > ledTimeout) {
    digitalWrite(LED_BUILTIN, HIGH);
  }

  // Read and send data
  if ((millis() - lastTime) > timerDelay) {
    // Set values to send
    myData.random = random(1,99);
    myData.temp = am2320.readTemperature();
    myData.humid = am2320.readHumidity();

    Serial.print("Random: ");
    Serial.println(myData.random);
    Serial.print("Temp: ");
    Serial.println(myData.temp);
    Serial.print("Humidity: ");
    Serial.println(myData.humid);

    // Send message via ESP-NOW
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  
    lastTime = millis();
  }
}