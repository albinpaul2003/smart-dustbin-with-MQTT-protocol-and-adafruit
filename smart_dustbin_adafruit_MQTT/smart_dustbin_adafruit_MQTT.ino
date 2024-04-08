#include <WiFi.h>
#include <WiFiClient.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <ESP32Servo.h>
#include "NewPing.h"

// WiFi parameters
#define WLAN_SSID       "Albin’s iPhone"
#define WLAN_PASS       "1a2a3a4a5a"

// Adafruit IO parameters
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "albinpaul2003"
#define AIO_KEY         "aio_AmSu62vd5vKQfRScXQD0OANnvcSD"

// HC-SR04 pins
#define WASTE_LEVEL_TRIGGER_PIN 13 // Pin connected to waste level sensor trigger
#define WASTE_LEVEL_ECHO_PIN    12 // Pin connected to waste level sensor echo
#define APPROACH_TRIGGER_PIN    27 // Pin connected to approach sensor trigger
#define APPROACH_ECHO_PIN       26 // Pin connected to approach sensor echo

// Servo motor pin
#define SERVO_PIN 18 // Pin connected to servo motor

// Define the maximum distance your dustbin can hold (in cm)
#define MAX_DISTANCE    30 // Adjust according to your dustbin's capacity

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish dustbindistance = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/dustbindistance");

// Servo setup
Servo servo;

void setup() {
  Serial.begin(115200);
  
  // Initialize servo
  servo.attach(SERVO_PIN);

  // Set pin modes for sensors
  pinMode(WASTE_LEVEL_TRIGGER_PIN, OUTPUT);
  pinMode(WASTE_LEVEL_ECHO_PIN, INPUT);
  pinMode(APPROACH_TRIGGER_PIN, OUTPUT);
  pinMode(APPROACH_ECHO_PIN, INPUT);

  // Connect to WiFi
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Connect to Adafruit IO
  connectToAdafruitIO();
}

void loop() {
  // Measure distance for waste level sensor
  float wasteLevelDistance = measureWasteLevelDistance();

  // Measure distance for approach sensor
  bool approachDetected = checkApproach();

  // Publish waste level distance to Adafruit IO
  if (!isnan(wasteLevelDistance)) {
    Serial.print("Waste level distance: ");
    Serial.print(wasteLevelDistance);
    Serial.println(" cm");
    publishToAdafruitIO(wasteLevelDistance);
  }

  // Open/close the dustbin based on approach sensor feedback
  if (approachDetected) {
    openDustbin();
  } else {
    closeDustbin();
  }

  delay(1000); // Adjust as needed
}

void connectToAdafruitIO() {
  Serial.print("Connecting to Adafruit IO... ");
  int8_t ret;
  while ((ret = mqtt.connect()) != 0) {
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying connection...");
    delay(5000);
  }
  Serial.println("Adafruit IO Connected!");
}

float measureWasteLevelDistance() {
  // Clear trigger pin
  digitalWrite(WASTE_LEVEL_TRIGGER_PIN, LOW);
  delayMicroseconds(2);

  // Send a 10us pulse on trigger pin
  digitalWrite(WASTE_LEVEL_TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(WASTE_LEVEL_TRIGGER_PIN, LOW);

  // Measure the pulse from echo pin
  float duration = pulseIn(WASTE_LEVEL_ECHO_PIN, HIGH);

  // Convert the pulse duration to distance (in cm)
  float distance = duration * 0.034 / 2;

  return distance;
}

bool checkApproach() {
  digitalWrite(APPROACH_TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(APPROACH_TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(APPROACH_TRIGGER_PIN, LOW);

  long duration = pulseIn(APPROACH_ECHO_PIN, HIGH);
  float distance = duration * 0.034 / 2;

  // Threshold for approach detection (adjust as needed)
  return distance < 50; // Adjust this distance threshold according to your setup
}

void openDustbin() {
  // Open the dustbin (adjust angle as needed)
  servo.write(90); // 90 degrees for half-open, adjust as needed
}

void closeDustbin() {
  // Close the dustbin (adjust angle as needed)
  servo.write(0); // 0 degrees for closed, adjust as needed
}

void publishToAdafruitIO(float distance) {
  if (distance <= MAX_DISTANCE) {
    // Map distance to percentage
    int percentFull = map(distance, 0, MAX_DISTANCE, 100, 0);
    
    // Publish percentage to Adafruit IO feed
    if (dustbindistance.publish(percentFull)) {
      Serial.println("Data published to Adafruit IO successfully!");
    } else {
      Serial.println("Failed to publish data to Adafruit IO!");
    }
  } else {
    Serial.println("Distance measurement out of range.");
  }
}
