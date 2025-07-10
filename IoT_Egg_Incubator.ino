#include <Wire.h> // I2C communication
#include <Arduino.h> // Basic Arduino functions
#include <HTTPClient.h> // HTTP communication for IoT

// # === WiFi === #
#include <WiFi.h> // WiFi library to connect ESP32 to a Wi-Fi network
const char* ssid = "your-wifi-ssdi"; // Replace with your Wi-Fi SSID
const char* password = "your-wifi-password"; // Replace with your Wi-Fi password
WiFiClient client; // Create a Wi-Fi client object

// # === ThingSpeak === #
#include <ThingSpeak.h> // ThingSpeak library to send data to the ThingSpeak platform
#define channelID 0000000 // Channel ID
#define apiKey "your-thingspeak-api-key" // API key to write data

// # === Blynk === #
#define BLYNK_TEMPLATE_ID "your-blynk-template-id" // Define the Blynk template ID
#define BLYNK_TEMPLATE_NAME "your-blynk-template-name" // Define the Blynk project name, "Inkubator telur"
#define BLYNK_AUTH_TOKEN "your-blynk-auth-token" // Define the authentication token to connect the device to the Blynk app
#include <BlynkSimpleEsp32.h> // Include the Blynk library for ESP32 for easy integration with the Blynk app via Wi-Fi

// # === INDICATOR LEDS === #
#define ledIndDinamo 15 // LED indicator pin for motor/relay
#define ledIndKipas 23 // LED indicator pin for fan/motor driver
#define ledIndDimmer 13 // LED indicator pin for light/dimmer

// # === L298N MOTOR DRIVER (FAN) === #
#define motor1pin1 26 // Pin to control motor 1 direction
#define motor1pin2 25 // Pin to control motor 1 direction
#define motor2pin1 33 // Pin to control motor 2 direction
#define motor2pin2 32 // Pin to control motor 2 direction
#define pwmFan1 12 // PWM pin to control motor 1 speed
#define pwmFan2 14 // PWM pin to control motor 2 speed
int rpm = 0; // Define the RPM variable

// # === TEMPERATURE & HUMIDITY SENSOR === #
#include <DHT.h> // DHT22 library
#define dhtPin 27 // DHT sensor pin
DHT dht(dhtPin, DHT22); // Initialize DHT sensor

// # === LCD DISPLAY === #
#include <LiquidCrystal_I2C.h> // LCD library
LiquidCrystal_I2C lcd(0x27, 16, 2); // Initialize LCD (address 0x27, 16 columns, 2 rows)

// # === DIMMER === #
    #include <RBDdimmer.h>
    #define USE_SERIAL  Serial // Define the serial port used for debugging
    #define outputPin  19 // Define the PWM output pin for the dimmer
    #define zerocross  18 // For boards with CHANGEABLE input pins
    dimmerLamp dimmer(outputPin, zerocross); // Initialize the port for the dimmer on ESP32

    // Variables
    float temperature = 0.0;              // Current temperature
    float batasSuhu = 0.0;                // Target temperature
    int terangLampuSaatIni = 0;           // Current lamp brightness (0–100%)
    const int idleLampu = 35;             // Idle lamp brightness (35%)
    const float zonaPengereman = 5.3;     // Dynamic braking zone (5.3°C)
    const float zonaToleransi = 0.1;      // Stable tolerance zone (±0.1°C)

    // Additional variables
    float humidity = 0.0;                      // Current humidity
    const float batasKelembapan = 60.0;        // Target humidity (%)
    const float zonaToleransiKelembapan = 5.0; // Humidity tolerance (±5%)
    int kecepatanKipas = 0;                    // Fan speed (0–100%)

// # === RELAY === #
  // Declare the relay pin
  #define relayPin 5
  // Duration for relay to stay ON (in milliseconds)
  const unsigned long relayOnDuration = 25 * 1000; // 25 seconds
  // Interval between relay ON periods (in milliseconds)
  const unsigned long relayInterval = 2 * 60 * 60 * 1000; // 2 hours
  // Variable to store the last time the relay was turned ON
  unsigned long previousMillis = 0;
  // Variable to store the relay status
  bool isRelayOn = false;

void setup() {
  // INITIALIZATION
    // Initialize SERIAL MONITOR
    Serial.begin(9600);

    // Initialize LCD DISPLAY
    lcd.begin();
    lcd.backlight();

    // Initialize WiFi
    WiFi.begin(ssid, password);
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startTime < 10000) {
      delay(1000);
      Serial.println("Connecting to WiFi...");
      lcd.setCursor(0, 0);
      lcd.print("Connecting to");
      lcd.setCursor(0, 1);
      lcd.print("WiFi...");
    }
    Serial.print("Connected to: ");
    Serial.println(ssid);
    lcd.setCursor(0, 0);
    lcd.print("Connected to: ");
    lcd.setCursor(0, 1);
    lcd.print(ssid);

    // Initialize ThingSpeak with WiFiClient
    ThingSpeak.begin(client);

    // Initialize Blynk
    Blynk.begin(BLYNK_AUTH_TOKEN, ssid, password);  // Start Blynk connection
    Serial.println("Enter target temperature via Blynk (V0)");

    // Initialize DHT22
    dht.begin();

    // Initialize indicator LEDs
    pinMode(ledIndDinamo, OUTPUT);
    pinMode(ledIndKipas, OUTPUT);
    pinMode(ledIndDimmer, OUTPUT);

    // Initialize RELAY
    pinMode(relayPin, OUTPUT);
    digitalWrite(relayPin, HIGH);

    // Initialize L298N (Motor driver)
    pinMode(motor1pin1, OUTPUT);
    pinMode(motor1pin2, OUTPUT);
    pinMode(motor2pin1, OUTPUT);
    pinMode(motor2pin2, OUTPUT);
    pinMode(12, OUTPUT); // Motor 1 PWM
    pinMode(14, OUTPUT); // Motor 2 PWM

    // Initialize DIMMER
    dimmer.begin(NORMAL_MODE, ON);

  // Prompt for target temperature input via serial monitor
  Serial.println("Enter target temperature via Serial Monitor:");
}

/*FUNCTION bacaSuhuTarget
  This function reads the target temperature input from the Serial Monitor.
  If there is incoming data, the target temperature is read using Serial.parseFloat(),
  then displayed back in the Serial Monitor.
  A 3-second delay is added to avoid duplicate readings.*/
void bacaSuhuTarget() {
  // Check if there is data from Serial Monitor
  if (Serial.available() > 0) {
    if (Serial.available() > 1) {
      batasSuhu = Serial.parseFloat();  // Read the input temperature
      Serial.print("Target Temperature: ");
      Serial.println(batasSuhu);        // Display the input temperature
      delay(3000);                      // Avoid duplicate readings
    }
  }
}

/*FUNCTION kontrolLampu
  This function reads the temperature from the DHT22 sensor and adjusts lamp brightness
  based on the target temperature. If the temperature approaches the target, brightness
  is gradually reduced. The lamp stays idle if within tolerance zone, lights up fully
  if far below target, or dims/shuts off if above target. Dimmer controls brightness,
  and LED indicator reflects lamp status.*/
void kontrolLampu() {
  // Read temperature from DHT22
  temperature = dht.readTemperature();

  // Check if temperature is valid
  if (isnan(temperature)) {
    Serial.println("Failed to read temperature sensor!");
    delay(2000);
    return;
  }

  // Calculate error (difference between target and current temperature)
  float error = batasSuhu - temperature;

  // If within braking zone (1°C before reaching target)
  if (error > 0 && error <= zonaPengereman) {
    // Gradually reduce brightness toward idle
    terangLampuSaatIni = map(error * 70, 0, zonaPengereman * 70, idleLampu, 70);
  }
  // If within stable tolerance zone (±0.1°C from target)
  else if (abs(error) <= zonaToleransi) {
    terangLampuSaatIni = idleLampu; // Lamp stays at idle
  }
  // If far below target (outside braking zone)
  else if (error > zonaPengereman) {
    terangLampuSaatIni = 70; // Lamp full brightness
  }
  // If temperature is above target
  else if (error < -zonaToleransi) {
    float overheat = abs(error); // Amount of temperature above target
    terangLampuSaatIni = map(overheat * 70, 0, zonaPengereman * 70, idleLampu, 0);
    // Ensure brightness is within 0–idleLampu range
    terangLampuSaatIni = constrain(terangLampuSaatIni, 0, idleLampu);

    if (overheat > zonaPengereman) {
      terangLampuSaatIni = 0; // Turn off lamp if severely overheated
    }
  }

  // Limit brightness between 0–70%
  terangLampuSaatIni = constrain(terangLampuSaatIni, 0, 70);

  // Set lamp brightness
  dimmer.setPower(terangLampuSaatIni);

  // LED indicator
  digitalWrite(ledIndDimmer, terangLampuSaatIni > 0 ? HIGH : LOW);
}

/*FUNCTION kontrolKipas
  This function controls fan speed based on temperature and humidity.
  Data from DHT22 is compared to target values to calculate error.
  Fan is controlled via PWM, adjusting speed based on zones like
  tolerance, braking, or extreme humidity. LED indicator shows fan status.*/
void kontrolKipas() {
  humidity = dht.readHumidity();

  // Check if value is valid
  if (isnan(humidity)) {
    Serial.println("Failed to read humidity sensor!");
    return;
  }

  // Calculate temperature and humidity error
  float errorSuhu = batasSuhu - temperature;
  float errorKelembapan = batasKelembapan - humidity;

  // Fan control logic based on temperature
  if (errorSuhu > 0 && errorSuhu <= zonaPengereman) {
    kecepatanKipas = map(errorSuhu * 100, 0, zonaPengereman * 100, 20, 70);
    digitalWrite(motor1pin1, HIGH);
    digitalWrite(motor1pin2, LOW);
  } else if (errorSuhu > zonaPengereman) {
    kecepatanKipas = 80;
    digitalWrite(motor1pin1, HIGH);
    digitalWrite(motor1pin2, LOW);
  } else if (errorSuhu < -zonaToleransi) {
    kecepatanKipas = map(abs(errorSuhu) * 100, 0, zonaPengereman * 100, 70, 100);
    digitalWrite(motor1pin1, HIGH);
    digitalWrite(motor1pin2, LOW);
  } else if (abs(errorSuhu) <= zonaToleransi) {
    kecepatanKipas = 30; // Stable zone
    digitalWrite(motor1pin1, HIGH);
    digitalWrite(motor1pin2, LOW);
  }

  // Additional humidity control logic
  if (errorKelembapan < -zonaToleransiKelembapan) {
    kecepatanKipas = max(kecepatanKipas, 100); // Too humid
    digitalWrite(motor1pin1, HIGH);
    digitalWrite(motor1pin2, LOW);
  } else if (errorKelembapan > zonaToleransiKelembapan) {
    kecepatanKipas = max(kecepatanKipas, 30); // Too dry
    digitalWrite(motor1pin1, HIGH);
    digitalWrite(motor1pin2, LOW);
  }

  // Clamp fan speed to 0–100%
  kecepatanKipas = constrain(kecepatanKipas, 0, 100);

  // Control fan using PWM
  analogWrite(pwmFan1, map(kecepatanKipas, 0, 100, 0, 255));
  analogWrite(pwmFan2, map(kecepatanKipas, 0, 100, 0, 255));

  // LED indicator
  digitalWrite(ledIndKipas, kecepatanKipas > 0 ? HIGH : LOW);
}

/*FUNCTION controlRelayWithTiming
  This function controls the relay using millis()-based timing.
  The relay turns on for a set duration and off for a set interval.
  Relay status is displayed on the LCD for easier monitoring.*/
void controlRelayWithTiming() {
  unsigned long currentMillis = millis();
  
  if (isRelayOn && currentMillis - previousMillis >= relayOnDuration) {
    // Turn off relay after on-duration
    isRelayOn = false;
    digitalWrite(ledIndDimmer, HIGH);
    digitalWrite(relayPin, LOW);
    lcd.setCursor(0, 1);
    lcd.print("Relay: OFF         ");
  } else if (!isRelayOn && currentMillis - previousMillis >= relayInterval) {
    // Turn on relay after interval
    isRelayOn = true;
    digitalWrite(ledIndDimmer, LOW);
    previousMillis = currentMillis;
    digitalWrite(relayPin, HIGH);
    lcd.setCursor(0, 1);
    lcd.print("Relay: ON          ");
  }
}

/*FUNCTION lcdDisplay
  This function displays temperature, humidity, fan speed,
  lamp brightness, and target temperature on the LCD.
  The display switches every 3 seconds to show real-time data to the user.*/
void lcdDisplay() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("humid: ");
  lcd.print(humidity);
  lcd.print("%  ");
  lcd.setCursor(0, 1);
  lcd.print("temp: ");
  lcd.print(temperature);
  lcd.print((char) 223);
  lcd.print(" ");
  delay(3000);
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Fan: ");
  lcd.print(kecepatanKipas);
  lcd.print("%  ");
  lcd.setCursor(0, 1);
  lcd.print("Lamp: ");
  lcd.print(terangLampuSaatIni);
  lcd.print("%  ");
  delay(3000);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Target Temp:");
  lcd.setCursor(0, 1);
  lcd.print(batasSuhu);
  lcd.print((char) 223);
  lcd.print(" ");
  delay(3000);
}

/*BLYNK_WRITE(V0):
  Reads the temperature value from Blynk input (V0)
  and stores it in the target temperature variable, then prints it to the Serial Monitor.*/
BLYNK_WRITE(V0) {
  batasSuhu = param.asFloat(); // Get value from numeric input
  Serial.print("Target temperature set to: ");
  Serial.println(batasSuhu);
}

/*BLYNK_WRITE(V1):
  Reads the relay status from Blynk input (V1) to turn it on or off,
  control the indicator LED, and print the relay status to the Serial Monitor and LCD.*/
BLYNK_WRITE(V1) { 
  int dinamoStatus = param.asInt(); // Read status from Virtual Pin V1
  if (dinamoStatus == 1) {
    digitalWrite(relayPin, LOW); // Turn on relay
    isRelayOn = true;
    Serial.println("Relay ON (via Blynk)");
    lcd.print("Relay: OFF         ");
  } else {
    digitalWrite(relayPin, HIGH); // Turn off relay
    isRelayOn = false;
    Serial.println("Relay OFF (via Blynk)");
    lcd.print("Relay: ON          ");
  }

  digitalWrite(ledIndDinamo, isRelayOn ? HIGH : LOW); // LED Indicator
}

void loop () {
  // Read temperature from the DHT sensor and store it in the temperature variable.
  temperature = dht.readTemperature(); 
  // Read humidity from the DHT sensor and store it in the humidity variable.
  humidity = dht.readHumidity(); 

  // # === FUNCTION CALLS === #
  // Read target temperature from Serial Monitor
  bacaSuhuTarget();
  // Run Blynk
  Blynk.run();
  // Control Lamp
  kontrolLampu();
  // Control Fan
  kontrolKipas();
  // Control Relay and Timing
  controlRelayWithTiming();
  // Display Process
  lcdDisplay();

  // Display Fan data on Serial Monitor
  Serial.print("TEMP: ");
  Serial.print(temperature);
  Serial.print(" HUMID: ");
  Serial.print(humidity);
  Serial.print(" Fan Speed: ");
  Serial.print(kecepatanKipas);

  // Display Lamp data on Serial Monitor
  Serial.print(" Lamp Brightness: ");
  Serial.print(terangLampuSaatIni);
  Serial.print(" Target Temp: ");
  Serial.println(batasSuhu);

  // Send data to ThingSpeak
  ThingSpeak.setField(1, temperature);
  ThingSpeak.setField(2, humidity);
  ThingSpeak.setField(3, terangLampuSaatIni);
  ThingSpeak.setField(4, kecepatanKipas);

  // Send temperature and humidity data to the IoT platform ThingSpeak
  ThingSpeak.writeFields(channelID, apiKey);

  // Delay 0.1 second
  delay(100); // Update every 100 ms
}