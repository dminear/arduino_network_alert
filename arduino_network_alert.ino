#include <SoftwareSerial.h> 
#include <SparkFunESP8266WiFi.h>
#include <SparkFunESP8266Client.h>
#include <SparkFunESP8266Server.h>
#include <Adafruit_NeoPixel.h>
#include "WS2812_Definitions.h"
//
// changed the buffer length to 280 in SparkFunESP8266WiFi.cpp

#define PIN 4        // WS2812 on pin 4
#define LED_COUNT 1  // only 1 of them

Adafruit_NeoPixel leds = Adafruit_NeoPixel(LED_COUNT, PIN, NEO_GRB + NEO_KHZ800);
unsigned long previousMillis = 0;
const unsigned long interval = 600000;  // 10 minutes

//////////////////////////////
// WiFi Network Definitions //
//////////////////////////////
// Replace these two character strings with the name and
// password of your WiFi network.
const char mySSID[] = "4704";
const char myPSK[] = "3pinkpigz";

IPAddress myIP;
int switchPin = 3;
volatile byte switchPinStatus = 1;

const String htmlHeader = "HTTP/1.1 200 OK\r\n"
                          "Content-Type: text/html\r\n"
                          "Content-Length: 55\r\n"
                          "Connection: close\r\n\r\n"
                          "<!DOCTYPE HTML>\r\n"
                          "<html>\r\n<body>Breaker!</body></html>\r\n";

const String httpCheckRequest1 = "POST /intercom/ip/";
const String httpCloseRequest2 = " HTTP/1.1\r\n" 
                           "Host: api.aududu.com\r\n"                        
                           "Connection: close\r\n\r\n";

const String httpAlertRequest1 = "POST /intercom/alert/";
                       
// "Content-Type: application/json\r\n"
 
char serverBuffer[21];
char serverBuffer2[11];
byte serverBufferIdx = 0;
byte soundAlarmState = 0;

void switchISR (void) {
  switchPinStatus = 0;
}
// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(115200);
  while (!Serial);

  // initialize digital pin 13 as an output.
  pinMode(13, OUTPUT);    // LED
  
  pinMode(switchPin, INPUT_PULLUP);     // switch
  attachInterrupt(digitalPinToInterrupt(switchPin), switchISR, LOW);  

  leds.begin();
  leds.show();
  leds.setPixelColor(0, RED);
  leds.show();
  //Serial.println( F("setup..."));
  initializeESP8266();
  
  connectESP8266();  // connectESP8266() connects to the defined WiFi network.
  
  leds.setPixelColor(0, GREEN);
  leds.show();

  // displayConnectInfo prints the Shield's local IP
  // and the network it's connected to.
  myIP = displayConnectInfo();

  clearLEDs();   // This function, defined below, turns all LEDs off...
  leds.show();   // ...but the LEDs don't actually update until you call this.
}

// the loop function runs over and over again forever
void loop() {
  unsigned long currentMillis = millis();
  int switchState = 1;
  ESP8266Server server = ESP8266Server(14252);
  ESP8266Client client;
  memset(serverBuffer, 0, sizeof(serverBuffer));
  memset(serverBuffer2, 0, sizeof(serverBuffer2));
  
  checkIn();  // initial powerup checkin
  
  tone(5,2000);   // some tones to indicate ready
  delay(50);
  tone(5,1000);
  delay(50);
  noTone(5);
  
  server.begin();   // start TCP server on port 14252
  
  // loop waiting for contact and checking paging switch
  while(1) {
    // checkin every time interval
    currentMillis = millis();
    if (currentMillis - previousMillis > interval) {
      previousMillis = currentMillis;
      checkIn();    // post request again 
    }

    // check paging switch status from ISR
    if (switchPinStatus==0) {
      switchPinStatus = 1;    // return to high
      tone(5,1000);           // feedback beep
      delay(200);
      noTone(5);    
      initiateAlert();
    }

    digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
 
    // check socket for command from server to sound alert
    client = server.available((uint8_t) 500);
    if (client) {
      Serial.println(F("Client connected"));
      unsigned int count = 0;
      while (client.connected()) {
        if (client.available()) {
          char c = client.read();
          //Serial.print(c);
          count++;
          
          // deal with a circular buffer to check content
          serverBuffer[serverBufferIdx++] = c;
          if (serverBufferIdx >= sizeof(serverBuffer)-1) {  // keep last null there
            serverBufferIdx = 0;
          }
        }     // end client available -- all data received
        else {
          count++;
        }
        // difficult to detect when the data is done. The ESP8266 sends
        // 2 CR/LFs at the beginning, so we can't look for that at the 
        // end of the headers.  We'll just look for 10000 times where
        // we don't see data available, and assume all the data came in.
        if (count == 1000) {
          //Serial.println(F("Sending HTML page"));  
          //client.print(htmlHeader);
          break;  // from while
        }
      }       // end while loop client connected or after break
      Serial.println();
      Serial.print(F("client gone with counter="));
      Serial.println(count);

      Serial.println(F("Sending HTML page"));  
      client.print(htmlHeader);

      if (strstr(serverBuffer, "alert") != NULL || count == 2) {
        //sound alert
        Serial.println(F("sound alert!"));
        soundAlarmState = 1;
      } else {      
        // try to rotate buffer around by at least 5 in case
        // "alert" is across the buffer ends
        for (int i = 15; i < 20; i++){
          serverBuffer2[i-15] = serverBuffer[i];
        }
        for (int i = 0; i < 5; i++) {
          serverBuffer2[i+5] = serverBuffer[i];
        }
        // now check serverBuffer2
        if (strstr(serverBuffer2, "alert") != NULL) {
          Serial.println(F("sound alert!"));
          soundAlarmState = 1;      
        } else {
        
         // not found
          Serial.println(F("no alert, buffers are:"));
          for(byte i=0;i < sizeof(serverBuffer)-1;i++){
            Serial.print(serverBuffer[i]);
          }
          Serial.println();
          for(byte i=0;i < sizeof(serverBuffer2)-1;i++){
            Serial.print(serverBuffer2[i]);
          }
          Serial.println();
        }
      }
      // give the web browser time to receive the data
      delay(1);
      // close the connection:
      client.stop();
      //Serial.println(F("Client disconnected"));
      if (soundAlarmState) {
        soundAlarmState = 0;
        soundAlarm();
      }
      memset(serverBuffer, 0, sizeof(serverBuffer));
      serverBufferIdx=0;
    }
    digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
  }
}

// do a HTTP POST with IP address
void checkIn() {
  ESP8266Client client;
  
  // ESP8266Client connect([server], [port]) is used to 
  // connect to a server (const char * or IPAddress) on
  // a specified port.
  // Returns: 1 on success, 2 on already connected,
  // negative on fail (-1=TIMEOUT, -3=FAIL).
  int retVal = client.connect("api.aududu.com", 80);
  if (retVal <= 0)
  {
    Serial.println(F("Failed to connect to server."));
    return;
  }
  client.print(httpCheckRequest1);
  myIP.printTo( client );  
  // print and write can be used to send data to a connected client connection.
  client.print(httpCloseRequest2);

  // available() will return the number of characters
  // currently in the receive buffer.
  while (client.available())
    Serial.write(client.read()); // read() gets the FIFO char
  
  // connected() is a boolean return value - 1 if the 
  // connection is active, 0 if it's closed.
  if (client.connected())
    client.stop(); // stop() closes a TCP connection.
}

// do a HTTP POST with IP address
void initiateAlert() {
  ESP8266Client client;
  
  // ESP8266Client connect([server], [port]) is used to 
  // connect to a server (const char * or IPAddress) on
  // a specified port.
  // Returns: 1 on success, 2 on already connected,
  // negative on fail (-1=TIMEOUT, -3=FAIL).
  int retVal = client.connect("api.aududu.com", 80);
  if (retVal <= 0)
  {
    Serial.println(F("Failed to connect to server."));
    return;
  }

  // print and write can be used to send data to a connected client connection.
  client.print(httpAlertRequest1);
  myIP.printTo( client );  
  client.print(httpCloseRequest2);

  // available() will return the number of characters
  // currently in the receive buffer.
  while (client.available())
    Serial.write(client.read()); // read() gets the FIFO char
  
  // connected() is a boolean return value - 1 if the 
  // connection is active, 0 if it's closed.
  if (client.connected())
    client.stop(); // stop() closes a TCP connection.
}

void ledsColor( unsigned long color ) {
    leds.setPixelColor(0, color);
    leds.show();
}

// activate the speaker and flash the LED
void soundAlarm() {  
  for(int j=0; j< 4; j++) {
    ledsColor(WHITE);
  
    for(int i=500;i<3000;i+=20) {
      tone(5,i);      // on pin 5
      delay(1);
    }
    clearLEDs();
  
    for(int i=3000;i>500;i-=20) {
      tone(5,i);
      delay(1);
    }
  }

  for(int j=0; j < 5; j++) {
    for(int i=0; i < 10; i++) {
      tone(5,900);
      delay(10);
      tone(5,1500);
      delay(10);
    }
    noTone(5);
    delay(100);
  }

  for(int i=0;i<4;i++){
    ledsColor(RED);    
    tone(5,800);
    delay(200);
    ledsColor(GREEN);
    tone(5,1000);
    delay(200);
    ledsColor(BLUE);    
    tone(5,1500);
    delay(200);
  }
  noTone(5);
  
  for(int i=0;i < 100; i++){
    ledsColor(random(65536)*256+random(256));
    delay(50);
    clearLEDs();
    delay(50);
  }
}


// Sets all LEDs to off, but DOES NOT update the display;
// call leds.show() to actually turn them off after this.
void clearLEDs(){
  for (int i=0; i<LED_COUNT; i++) {
    leds.setPixelColor(i, 0);
  }
  leds.show();
}

void initializeESP8266(){
  // esp8266.begin() verifies that the ESP8266 is operational
  // and sets it up for the rest of the sketch.
  // It returns either true or false -- indicating whether
  // communication was successul or not.
  int test = esp8266.begin( 4800 );
  if (test != true) {
    //Serial.println(F("Error initializing"));
    errorLoop(test);
  }
}

// errorLoop prints an error code, then loops forever.
void errorLoop(int error){
  while(1) {
  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
  leds.setPixelColor(0, RED);
  leds.show();
  delay(100);
  digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
  clearLEDs(); 
  leds.show();
  delay(100);
  }
}

void connectESP8266(){
  // The ESP8266 can be set to one of three modes:
  //  1 - ESP8266_MODE_STA - Station only
  //  2 - ESP8266_MODE_AP - Access point only
  //  3 - ESP8266_MODE_STAAP - Station/AP combo
  // Use esp8266.getMode() to check which mode it's in:
  int retVal = esp8266.getMode();
  if (retVal != ESP8266_MODE_STA) { // If it's not in station mode.
    // Use esp8266.setMode([mode]) to set it to a specified
    // mode.
    retVal = esp8266.setMode(ESP8266_MODE_STA);
    if (retVal < 0) {
      //Serial.println(F("Error setting mode."));
      errorLoop(retVal);
    }
  }
  //Serial.println(F("Mode set to station"));

  // esp8266.status() indicates the ESP8266's WiFi connect
  // status.
  // A return value of 1 indicates the device is already
  // connected. 0 indicates disconnected. (Negative values
  // equate to communication errors.)
  retVal = esp8266.status();
  if (retVal <= 0) {
    //Serial.print(F("Connecting to "));
    Serial.println(mySSID);
    // esp8266.connect([ssid], [psk]) connects the ESP8266
    // to a network.
    // On success the connect function returns a value >0
    // On fail, the function will either return:
    //  -1: TIMEOUT - The library has a set 30s timeout
    //  -3: FAIL - Couldn't connect to network.
    retVal = esp8266.connect(mySSID, myPSK);
    if (retVal < 0) {
      //Serial.println(F("Error connecting"));
      errorLoop(retVal);
    }
  }
}

IPAddress displayConnectInfo(){
  char connectedSSID[24];
  memset(connectedSSID, 0, 24);
  // esp8266.getAP() can be used to check which AP the
  // ESP8266 is connected to. It returns an error code.
  // The connected AP is returned by reference as a parameter.
  int retVal = esp8266.getAP(connectedSSID);
  if (retVal > 0) {
    //Serial.print(F("Connected to: "));
    Serial.println(connectedSSID);
  }

  // esp8266.localIP returns an IPAddress variable with the
  // ESP8266's current local IP address.
  IPAddress myIP = esp8266.localIP();
  //Serial.print(F("My IP: "));
  Serial.println(myIP);
  return myIP;
}

