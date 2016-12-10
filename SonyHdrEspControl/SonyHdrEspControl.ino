
//----------------------------------------------------------------------------------------------------------------------
// 
// SonyHdrEspControl by Rein Velt
// The Sony HDR AS20 camera can make 12 megapixel photos in photo mode but in interval mode it can only
// make 2 megapixel photos. This software/device was made as external interval thingy that
// triggers the camera every 10 seconds to make 12 megapixel photo
//
// This program is based on: WiFiClient from ESP libraries
//
// Camera handling by Reinhard Nickels https://glaskugelsehen.wordpress.com/
// tested with DSC-HX90V, more about protocol in documentation of CameraRemoteAPI https://developer.sony.com/develop/cameras/
// 
// Licenced under the Creative Commons Attribution-ShareAlike 3.0 Unported (CC BY-SA 3.0) licence:
// http://creativecommons.org/licenses/by-sa/3.0/
//
// Requires Arduino IDE with esp8266 core: https://github.com/esp8266/Arduino install by boardmanager
//----------------------------------------------------------------------------------------------------------------------
 
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#define DEBUG 1
#define INTERVAL_DELAY 8000   // pushbutoon on GPIO2
volatile int counter;
 
const char* ssid     = "DIRECT-WXH4:HDR-AS20";
const char* password = "WjNpHNke";     // your WPA2 password
 
const char* host     = "192.168.122.1";   // fixed IP of camera
const int httpPort   = 10000;
 
char JSON_1[] = "{\"version\":\"1.0\",\"id\":1,\"method\":\"getVersions\",\"params\":[]}";
char JSON_2[] = "{\"version\":\"1.0\",\"id\":1,\"method\":\"startRecMode\",\"params\":[]}";
char JSON_3[] = "{\"version\":\"1.0\",\"id\":1,\"method\":\"startLiveview\",\"params\":[]}";
char JSON_4[] = "{\"version\":\"1.0\",\"id\":1,\"method\":\"stopLiveview\",\"params\":[]}";
char JSON_5[] = "{\"version\":\"1.0\",\"id\":1,\"method\":\"actTakePicture\",\"params\":[]}";
// char JSON_6[]="{\"method\":\"getEvent\",\"params\":[true],\"id\":1,\"version\":\"1.0\"}";

#define STATE_INIT    0
#define STATE_CONNECT 1
#define STATE_CONNECTED 2
#define STATE_ACTION_RUN 3
#define STATE_ACTION_READY 4
 
unsigned long lastmillis;
unsigned long interval=0;
 
WiFiClient client;
int state;

 

 
void httpPost(char* jString) {
  if (DEBUG) {Serial.print("Msg send: ");Serial.println(jString);}
  Serial.print("connecting to ");
  Serial.println(host);
  if (!client.connect(host, httpPort)) {
    Serial.println("connection failed");
    return ;
  }
  else {
    Serial.print("connected to ");
    Serial.print(host);
    Serial.print(":");
    Serial.println(httpPort);
  }
 
  // We now create a URI for the request
  String url = "/sony/camera/";
 
  Serial.print("Requesting URL: ");
  Serial.println(url);
 
  // This will send the request to the server
  client.print(String("POST " + url + " HTTP/1.1\r\n" + "Host: " + host + "\r\n"));
  client.println("Content-Type: application/json");
  client.print("Content-Length: ");
  client.println(strlen(jString));
  // End of headers
  client.println();
  // Request body
  client.println(jString);
  Serial.println("wait for data");
  lastmillis = millis();
  while (!client.available() && millis() - lastmillis < 8000) {} // wait 8s max for answer
 
  // Read all the lines of the reply from server and print them to Serial
  while (client.available()) {
    String line = client.readStringUntil('\r');
    Serial.print(line);
  }
  Serial.println();
  Serial.println("----closing connection----");
  Serial.println();
  client.stop();
}
 



void connect()
{
   Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  lastmillis=millis();
  while (WiFi.status() != WL_CONNECTED && millis() - lastmillis < 32000) {   // wait for WiFi connection
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    state=STATE_CONNECTED;
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    delay(1000);
    httpPost(JSON_1);  // initial connect to camera
    //httpPost(JSON_2); // startRecMode
    //httpPost(JSON_3);  //startLiveview  - in this mode change camera settings  (skip to speedup operation)
 }
 else
 {
  state=STATE_INIT;
 }
}
void runinterval() {
  if (millis()>(interval+INTERVAL_DELAY)){
    interval=millis();
    Serial.println("interval..");
    //httpPost(JSON_4); //stopLiveview    (skip to speedup operation)
    httpPost(JSON_5);  //actTakePicture
    //httpPost(JSON_3);  //startLiveview    (skip to speedup operation)
   }
  
}

void loop()
{
  if (WiFi.status() != WL_CONNECTED) {
     state=STATE_INIT;
  }
  else
  {
    if (state==STATE_CONNECT)
    {
      state=STATE_CONNECTED;
    }
  }
  
  switch( state)
  {
      case STATE_INIT: state=STATE_CONNECT; connect(); break;
      case STATE_CONNECT:  break; 
      case STATE_ACTION_RUN: break;   
      case STATE_CONNECTED:
      case STATE_ACTION_READY:
        runinterval();
        break;
    
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  state=STATE_INIT;
}

