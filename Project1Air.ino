// Import Library
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Arduino.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <AHTxx.h>
#include <DFRobot_ENS160.h>
#include <DHT.h>

// Set value
  //ENS160 + AHT 21
#define PIN_SDA D1
#define PIN_SCL D2
#define TEMPERATURE_CORRECTION -4 // 温度補正 -4 ℃
  // DHT Sensor
#define DHTPIN D3      // Pin where the DHT22 sensor is connected  
#define DHTTYPE DHT22  // Define the type of DHT sensor 
  // Dust Sensor PM2.5 GP2Y1010AU0F
int dustPin=0;
int ledPower=4;
int delayTime=280;
int delayTime2=40;
float offTime=9680; //10000-280-40
int dustVal=0;
char s[32];
float voltage = 0;
float dustdensity = 0;

// DHT Sensor
DHT dht(DHTPIN, DHTTYPE);

// ENS160 + AHT21
DFRobot_ENS160_I2C ENS160(&Wire, /*I2CAddr*/ 0x53);
AHTxx AHT21(AHTXX_ADDRESS_X38, AHT2x_SENSOR);
uint8_t gAQI;
uint16_t gTVOC;
uint16_t gECO2;

// Nhiệt độ, độ ẩm của AHT21
float calcMaxVaperPressure(float temp)
{
  // 飽和水蒸気圧
  return 6.1078 * pow(10, 7.5 * temp / (temp + 237.3));
}

float recalculateHumidity(float temp, float rh)
{
  // 温度を補正して湿度を再計算
  float temp2 = temp + TEMPERATURE_CORRECTION;
  return rh * calcMaxVaperPressure(temp) * (temp2 + 273.15) /
         ((temp + 273.15) * calcMaxVaperPressure(temp2));
}

// Update these with values suitable for your network.
const char* ssid = "Dinh Son";
const char* password = "Shinichi@0311";
const char* mqtt_server = "172.104.167.168";
const char* outTopic = "/vn/hn/haui/room1/dev3/data";
const char* inTopic = "/cmd/vn/hn/haui/room1/dev3";
const String clientID = "vn-hn-haui-room1-dev3";
int buzzerPin = D7;

JsonDocument doc;
WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE	(50)
char msg[MSG_BUFFER_SIZE];
int value = 0;

// Set up WIFI
void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    digitalWrite(BUILTIN_LED, LOW);   // Turn the LED on (Note that LOW is the voltage level
    digitalWrite(buzzerPin, LOW);
    // but actually the LED is on; this is because
    // it is active low on the ESP-01)
  } else {
    digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
    digitalWrite(buzzerPin, HIGH);
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(clientID.c_str(),"mqtttest","mqtttest")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("/clients", clientID.c_str());
      // ... and resubscribe
      client.subscribe(inTopic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {  
  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
  digitalWrite(buzzerPin, HIGH);
  Serial.begin(9600);  
  dht.begin();  
  pinMode(ledPower,OUTPUT);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback); 
  Wire.begin(PIN_SDA, PIN_SCL);

  // Check connect from ENS160 + AHT21
  Serial.println("ENS160 - Digital air quality sensor");
  while (ENS160.begin() != NO_ERR)
  {
    Serial.println("Communication with device failed, please check connection");
    delay(3000);
  }
  ENS160.setPWRMode(ENS160_STANDARD_MODE);

  while (!AHT21.begin(PIN_SDA, PIN_SCL))
  {
    Serial.println("Could not find a valid ATH21 sensor, check wiring!");
    delay(3000);
  }
  Serial.println("ATH20 sensor found");
}  

void loop() {  

  //ENS160 + AHT21 Sensor
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  unsigned long now = millis();
  if (now - lastMsg > 10000) {
    lastMsg = now;
    ++value;
   uint8_t sensorStatus = ENS160.getENS160Status();
  Serial.printf("Sensor operating status: %d\n", sensorStatus);
  /*//AHT21 Cảm biến
  float rawTemperature = AHT21.readTemperature();
  float rawHumidity = AHT21.readHumidity();
  float t = rawTemperature + TEMPERATURE_CORRECTION;
  float h = recalculateHumidity(rawTemperature, rawHumidity);
  ENS160.setTempAndHum(t, h);*/

  // DHT Sensor
  float humidity = dht.readHumidity();  // Read humidity  
  float temperature = dht.readTemperature(); // Read temperature in Celsius  
    // Check if any reads failed and exit early (to try again).  
  if (isnan(humidity) || isnan(temperature)) {  
    Serial.println("Failed to read from DHT sensor!");  
    return;  
  }  

  // Dust Sensor PM2.5 GP2Y1010AU0F
  digitalWrite(ledPower,LOW); // power on the LED
  delayMicroseconds(delayTime);
  dustVal=analogRead(dustPin); // read the dust value
  delayMicroseconds(delayTime2);
  digitalWrite(ledPower,HIGH); // turn the LED off
  delayMicroseconds(offTime);
  
  voltage = dustVal*0.0049;//dustval*5/1024
  dustdensity = 0.172*voltage-0.1;
  
  if (dustdensity < 0 )
  dustdensity = 0;
  if (dustdensity > 0.5)
  dustdensity = 0.5;
  String dataString = "";
  dataString += dtostrf(voltage, 9, 4, s);
  dataString += "V,";
  dataString += dtostrf(dustdensity*1000.0, 5, 2, s);
  dataString += "ug/m3";

  //AQI, TVOC, ECO2 from ENS160
  gAQI = ENS160.getAQI();
  gTVOC = ENS160.getTVOC();
  gECO2 = ENS160.getECO2();

  // Monitoring data
  // Dust Sensor PM2.5 GP2Y1010AU0F
  doc["temperature"] = temperature;
  doc["humidity"] = humidity;
  doc["aqi"] = gAQI;
  doc["voc"] = gTVOC;
  doc["co2"] = gECO2;
  doc["dust"] = dataString; //Cảm biến bụi
  String msg ;
  serializeJson(doc,msg);
    Serial.print("Publish message: ");
    Serial.println(msg);
    client.publish(outTopic, msg.c_str());
  } 
}