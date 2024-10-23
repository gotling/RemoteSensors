#include <ESP8266WiFi.h>
#include <espnow.h>

#define LED D2

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  int random;
  float temp;
  float humid;
} struct_message;

// Create a struct_message called myData
struct_message myData;

unsigned long lastTime = 0;  
unsigned long ledTimeout = 2000;  // seconds to keep led on after successfull send
bool ledOn = false;

bool setupActive = true;
unsigned long setupTime = 0;
unsigned long setupTimeout = 10 * 60 * 1000;

// Callback function that will be executed when data is received
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Random: ");
  Serial.println(myData.random);
  Serial.print("Temp: ");
  Serial.println(myData.temp);
  Serial.print("Humidity: ");
  Serial.println(myData.humid);
  Serial.println();

  if (setupActive) {
    digitalWrite(LED_BUILTIN, LOW);
    ledOn = true;
    lastTime = millis();
  }
}
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  while (!Serial) {
    delay(100); // hang out until serial port opens
  }

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(OnDataRecv);

  setupTime = millis();
}

void loop() {
  if (setupActive) {
    if ((millis() - setupTime) > setupTimeout) {
      setupActive = false;
      Serial.println("Setup deactivated");
    }

    // Turn of LED after set time
    if (ledOn && ((millis() - lastTime) > ledTimeout)) {
      Serial.println("LED off");
      digitalWrite(LED_BUILTIN, HIGH);
      ledOn = false;
    }
  }
}
