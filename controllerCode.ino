// Include WiFi files
#include <WiFi.h>
#include <WiFiUdp.h>

// Define the network name/password we want to connect to
const char* ssid     = "";
const char* password = "";

// Variables and set up for Wifi
WiFiUDP UDPTestServer;
unsigned int UDPPort = 2150;                       // Define the target port
IPAddress localIPaddress(192, 168, 1, 150);        // Define IP Address
IPAddress targetIPaddress(192, 168, 1, 255);       // Define target IP Address

const int UDP_PACKET_SIZE = 48;                    // Define the packet size
char udpBuffer[UDP_PACKET_SIZE];                   // Initialize the send buffer
byte packetBuffer[UDP_PACKET_SIZE+1];              // Initialize the receive buffer

int sendtime; // Create a global variable for the controller values

void setup() {
  Serial.begin(115200);                            // Begin printing to terminal
  
  // Initiate AP mode
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid,password);
  delay(100);
  WiFi.softAPConfig(localIPaddress, IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
  // WiFi.begin(ssid,password);
    UDPTestServer.begin(UDPPort); // strange bug needs to come after WiFi.begin but before connect
    packetBuffer[UDP_PACKET_SIZE] = 0; // null terminate buffer 

  // Initialize pin modes
  pinMode(17, INPUT); 
  pinMode(5, INPUT);
    
}

//main code starts here
void loop(){ 
  int buttonState1 = digitalRead(17);             // Read button 1
  int buttonState2 = digitalRead(5);              // Read button 2
  if(buttonState1== HIGH)                         // If button 2 pressed, send 22
 {UDPsendData(21); }
  if(buttonState2== HIGH)                         // If button 2 pressed, send 22
 {UDPsendData(22); }                              
  sendtime=map(analogRead(A4),0,4095,1,50);       // Store the value on joystick 1
  sendUDP();                                      // Send the value
  delay(10);                                      // Delay to avoid backlogging
  sendtime=map(analogRead(A5),0,4095,206,255);    // Store the value on joystick 2
  sendUDP();                                      // Send the value
  delay(10);                                      // Delay to avoid backlogging 
  
}

// Function to send packets to the target ESP
void sendUDP(){
  udpBuffer[0] = sendtime&0xff;                         // Define the packets contents
  UDPTestServer.beginPacket(targetIPaddress, UDPPort);  // Create the packet
  UDPTestServer.printf("%s",udpBuffer);                 // Write to the packet
  UDPTestServer.endPacket();
    Serial.println(sendtime);}                           // End packet
