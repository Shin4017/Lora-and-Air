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
int ledPower=D4;
float offTime=9680; //10000-280-40
int dustVal=0;
char s[32];
float voltage = 0;
float dustdensity = 0;
int counter = 0;

// DHT Sensor
DHT dht(DHTPIN, DHTTYPE);

// ENS160 + AHT21
DFRobot_ENS160_I2C ENS160(&Wire, /*I2CAddr*/ 0x53);
AHTxx AHT21(AHTXX_ADDRESS_X38, AHT2x_SENSOR);
int gAQI[10];
int gTVOC[10];
int gECO2[10];
float humidity[10];
float temperature[10];
float dust[10];

/*
private function
*/
int filterADC(int m)
{
  static int flag_first = 0, _buff[10], sum;
  const int _buff_max = 10;
  int i;
  
  if(flag_first == 0)
  {
    flag_first = 1;
    for(i = 0, sum = 0; i < _buff_max; i++)
    {
      _buff[i] = m;
      sum += _buff[i];
    }
    return m;
  }
  else
  {
    sum -= _buff[0];
    for(i = 0; i < (_buff_max - 1); i++)
    {
      _buff[i] = _buff[i + 1];
    }
    _buff[9] = m;
    sum += _buff[9];
    
    i = sum / 10.0;
    return i;
  }
}
float getAvgFloat(float arr[], int n) {
    float sum = 0;

    // Find the sum of all elements
    for (int i = 0; i < n; i++) {
        sum += arr[i];
    }
      
      // Return the average
    return (float)sum / n;
}
int getAvgInt(int arr[], int n) {
    int sum = 0;

    // Find the sum of all elements
    for (int i = 0; i < n; i++) {
        sum += arr[i];
    }
      
      // Return the average
    return (int)sum / n;
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
  for (int i = 0; i < length; i++) {Serial.print((char)payload[i]);
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
  Serial.begin(115200);  
  dht.begin();  
  pinMode(ledPower,OUTPUT);
  digitalWrite(ledPower,HIGH);
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
  float h = dht.readHumidity();  // Read humidity  
  float t = dht.readTemperature(); // Read temperature in Celsius  
    // Check if any reads failed and exit early (to try again).  
  if (isnan(h) || isnan(t)) {  
    Serial.println("Failed to read from DHT sensor!"); 
    counter = 0; 
    yield();
  } 
  ENS160.setTempAndHum(t, h);
  temperature[counter] = t;
  humidity[counter] = h;
    //AQI, TVOC, ECO2 from ENS160
  gAQI[counter] = ENS160.getAQI();
  gTVOC[counter] = ENS160.getTVOC();
  gECO2[counter] = ENS160.getECO2();

  digitalWrite(ledPower,LOW); // power on the LED
  delayMicroseconds(280);
  dustVal=analogRead(dustPin); // read the dust value
  //dustVal = filterADC(dustVal);
  delayMicroseconds(40);
  digitalWrite(ledPower,HIGH); // turn the LED off
  voltage = dustVal*(5000/1024);
  //voltage = dustVal*(3.3/1024);
  dustdensity = 0.2*voltage;
  dust[counter] =  dustdensity;
  delayMicroseconds(9680);
  counter ++;
  delay (990);
  

  if (counter == 10) {
    counter = 0;
  /*//AHT21 Cảm biến
  float rawTemperature = AHT21.readTemperature();
  float rawHumidity = AHT21.readHumidity();
  float temperature = rawTemperature + TEMPERATURE_CORRECTION;
  float humidity = recalculateHumidity(rawTemperature, rawHumidity);
  ENS160.setTempAndHum(temperature, humidity);*/

  // DHT Sensor
  
  

  // Dust Sensor PM2.5 GP2Y1010AU0F
 
  /*
  if (dustdensity < 0 )
  dustdensity = 0;
  if (dustdensity > 0.5)
  dustdensity = 0.5;
  String dataString = "";
  //dataString += dtostrf(voltage, 9, 4, s);
  //dataString += "V,";
  dataString += dtostrf(dustdensity*1000.0, 5, 2, s);
  //dataString += "ug/m3";
*/


  // Monitoring data
  // Dust Sensor PM2.5 GP2Y1010AU0F
  doc["temperature"] = getAvgFloat(temperature,10);
  doc["humidity"] = getAvgFloat(humidity,10);
  doc["aqi"] = getAvgInt(gAQI,10);
  doc["voc"] = getAvgInt(gTVOC,10);
  doc["co2"] = getAvgInt(gECO2,10);
  doc["pm25"] = getAvgFloat(dust,10); //Cảm biến bụi
  String msg ;
  serializeJson(doc,msg);
    Serial.print("Publish message: ");
    Serial.println(msg);
    client.publish(outTopic, msg.c_str());
  } 
}