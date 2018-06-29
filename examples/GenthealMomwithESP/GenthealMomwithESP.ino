#include <WiFiEsp.h>
#include <WiFiEspClient.h>
#include <WiFiEspUdp.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>


// Update these with values suitable for your network.

char* ssid = "YOURSSID";
char* password = "YOURPASSWORD";
const char* mqtt_server = "YOURmqttSERVERip";
const char* clientID = "YOURCLIENTID";
const char* outTopic = "YOURCLIENTOUTTOPIC";
const char* inTopic = "YOURCLIENTINTOPIC";

int status = WL_IDLE_STATUS;   // the Wifi radio's status



float humidity, temp_c;  // Values read from sensor
// Generally, you should use "unsigned long" for variables that hold time
unsigned long previousMillis = 0;        // will store last temp was read
const long interval = 2000;              // interval at which to read sensor
 
WiFiEspClient espClient;
PubSubClient client(espClient);
char msg[50];

SoftwareSerial soft(8, 9);    // RX, TX

const int PIR_pin = 3;        
int Previous_Signal = LOW;

void setup() {
  Serial.begin(115200);
  soft.begin(9600);
                              // initialize ESP module
  WiFi.init(&soft);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  
  pinMode(PIR_pin, INPUT);

}

void loop() {
  
  int PIR_signal = digitalRead(PIR_pin);

  if (PIR_signal == HIGH && Previous_Signal == LOW)
  {
    Previous_Signal = PIR_signal;
    Serial.println("Presence Revealed");
    client.publish(outTopic, "Presence");
  }
  else if (PIR_signal == LOW && Previous_Signal == HIGH)
  {
    Previous_Signal = PIR_signal;
  }
  
  if (!client.connected()) {
    reconnect();
  }
  
  delay(100);
  client.loop();
}

void setup_wifi() {

  // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue
    while (true);
  }
  
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  // attempt to connect to WiFi network
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network
    status = WiFi.begin(ssid, password);
  }

  // you're connected now, so print out the data
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  // Conver the incoming byte array to a string
  payload[length] = '\0'; // Null terminator used to terminate the char array
  String message = (char*)payload;

  Serial.print("Message arrived on topic: [");
  Serial.print(topic);
  Serial.print("], ");
  Serial.println(message);

  if(message == "temperature"){
    gettemperature();
    Serial.print("Sending temperature:");
    Serial.println(temp_c);
    dtostrf(temp_c , 2, 2, msg);
    client.publish(outTopic, msg);
  } else if (message == "humidity"){
    gettemperature();
    Serial.print("Sending humidity:");
    Serial.println(humidity);
    dtostrf(humidity , 2, 2, msg);
    client.publish(outTopic, msg);
  }
}

void reconnect() {
  // Loop until we're reconnected
  delay(100);
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(clientID)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish(outTopic, clientID);
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

