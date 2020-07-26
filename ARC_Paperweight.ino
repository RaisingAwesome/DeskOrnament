// ARC Paperweight Code
// Copyright 2020 by Sean J. Miller
//Note to self:  alert should be red and blue like a cop!!!!
#include <Ultrasonic.h>
#include <ss_oled.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include "Adafruit_ZeroTimer.h"
#include "arduino_secrets.h"

#define DEBUG true

#define USE_BACKBUFFER
#ifdef USE_BACKBUFFER
static uint8_t ucBackBuffer[1024];
#else
static uint8_t *ucBackBuffer = NULL;
#endif

#define SDA_PIN 18
#define SCL_PIN 19
#define RESET_PIN -1
#define OLED_ADDR 0x3c
#define FLIP180 0
#define INVERT 0
#define USE_HW_I2C 0
#define MY_OLED OLED_128x64
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
SSOLED ssoled;

String line_one="";
String line_two="";
String line_three="";
String Scroll="";
String LED1="GREEN";
String LED2="GREEN";
String LED3="GREEN";
String Status="";
boolean changed_display=false; boolean button_pressed=false;
int INTENSITY=255;

//Ultrasonic global variables
Ultrasonic ultrasonic(20,21);
double double_distance;
char char_distance[7];

//wifi global variables
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;
IPAddress server(192,168,1,239);  
WiFiClient client;

//timer globabl variables
float freq = 12.0; // 1 KHz
Adafruit_ZeroTimer zerotimer = Adafruit_ZeroTimer(3);
boolean heading_up=true;

void setup() {
  if (DEBUG) Serial.begin(9600);
  timer_setup();
  setupPins();
  delay(2000);
  setupOLED();
//  setupWifi();  //uncomment if you will need wifi
}

void loop() {
  if (DEBUG) Serial.println("Working");
  
  cls();
  showOLEDMessage("   Police",1);
  showOLEDMessage("       Lights        ",2);
  changed_display=true;
  showPoliceLights();
  delay(2000);
  cls();
  showOLEDMessage(" BLUE",3);
  LED1="BLUE";LED2="BLUE";LED3="BLUE";
  setTheLightsPerTheVariables();
  delay(2000);
  cls();
  showOLEDMessage("  RED",3);
  LED1="RED";LED2="RED";LED3="RED";
  setTheLightsPerTheVariables();
  delay(2000);
  cls();
  showOLEDMessage(" GREEN",3);
  LED1="GREEN";LED2="GREEN";LED3="GREEN";
  setTheLightsPerTheVariables();
  delay(2000);
  cls();
  showOLEDMessage(" YELLOW",3);
  LED1="YELLOW";LED2="YELLOW";LED3="YELLOW";
  setTheLightsPerTheVariables();
  delay(2000);
  getDistance();
  showDistanceMessage();
  delay(2000); 
}

void buttonPressed()
{//Called by the interrupt (ISR) when the button is pressed.
  if (button_pressed) return;
  detachInterrupt(digitalPinToInterrupt(13));
  button_pressed=true;
  
  if (DEBUG) Serial.println("Button Pressed!");
  showButtonPressedMessage();
  delayMicroseconds(1000000); //one second
  attachInterrupt(digitalPinToInterrupt(13), buttonPressed, FALLING);
  button_pressed=false;
}

void setupWIFI(){
  //sets up the WiFi on startup.
  if (WiFi.status() == WL_NO_MODULE) {
    if (DEBUG) Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    if (DEBUG) Serial.println("Please upgrade the firmware");
  }
  showConnectingMessage();
  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    if (DEBUG) Serial.print("Attempting to connect to SSID: ");
    if (DEBUG) Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
    
    // wait 10 seconds for connection:
    delay(3000);
  }
  changed_display=true;showIdleMessage();
  delay(500);
  if (DEBUG) Serial.println("Connected to wifi");
  printWifiStatus();
}

void setupOLED(){
  //Initializes the OLED display on startup
  int rc=0;
  rc = oledInit(&ssoled, MY_OLED, OLED_ADDR, FLIP180, INVERT, USE_HW_I2C, SDA_PIN, SCL_PIN, RESET_PIN, 400000L); // use standard I2C bus at 400Khz
  if (rc != OLED_NOT_FOUND)
  {
    char *msgs[] = {(char *)"SSD1306 @ 0x3C", (char *)"SSD1306 @ 0x3D",(char *)"SH1106 @ 0x3C",(char *)"SH1106 @ 0x3D"};
    oledFill(&ssoled, 0, 1);
    oledWriteString(&ssoled, 0,0,0,msgs[rc], FONT_NORMAL, 0, 1);
    oledSetBackBuffer(&ssoled, ucBackBuffer);
    delay(2000);
  }
  
  showBootMessage();  
  delay(2000);
}

void setupPins()
{//Sets the pin modes for the LEDs and sets up the interrupt for the button.
  // HSE 
  pinMode(9,OUTPUT);digitalWrite(9,LOW);
  pinMode(10,OUTPUT);digitalWrite(10,LOW);
  pinMode(11,OUTPUT);digitalWrite(11,LOW);

  //PSM
  pinMode(3,OUTPUT);digitalWrite(9,0);
  pinMode(5,OUTPUT);digitalWrite(10,0);
  pinMode(6,OUTPUT);digitalWrite(11,0);

  //MI
  pinMode(2,OUTPUT);digitalWrite(2,0);
  pinMode(A3,OUTPUT);digitalWrite(A3,0);
  pinMode(A2,OUTPUT);digitalWrite(A2,0);

   //button
  pinMode(13,INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(13), buttonPressed, FALLING);
}

void showButtonPressedMessage()
{
  cls();
  showOLEDMessage("Interrupt!",1);
  showOLEDMessage("        BUTTON         ",2); //line 2 is 20 wide
  showOLEDMessage("PRESSED",3);
  changed_display=true;
}

void showDistanceMessage()
{
  cls();
  showOLEDMessage("Distance",1);
  showOLEDMessage("                 ",2); //line 2 is 20 wide
  String my_string=String(double_distance,2) +' ft';
  showOLEDMessage(&my_string[0],3);
  changed_display=true;
}
void showBootMessage()
{//used in an IoT project on startup.  Not used now.
  cls();
  showOLEDMessage("CONNECTING",1);
  showOLEDMessage("",2); //line 2 is 20 wide
  showOLEDMessage("STANDBY",3);
  changed_display=true;
}
void showConnectingMessage()
{//used in an IoT project on startup.  Not used now.
  cls();
  showOLEDMessage("CONNECTING",1);
  showOLEDMessage("   GETTING DATA     ",2); //line 2 is 20 wide

  showOLEDMessage("STANDBY",3);
  changed_display=true;
}

void alertTheTeam()
{//This was another routine to support an IoT project that is presently not used.  It would show when they pressed the button.
  button_pressed=false;
  cls();
  showOLEDMessage("EXCELLENCE!",1);
  showOLEDMessage("       Stand By      ",2); //line 2 is 20 wide
  showOLEDMessage("SENDING",3);
  changed_display=true;
  sendButtonMessage(); 
  delay(1000);
  showIdleMessage();
}

void handleThings(){
  //Made this to short the Loop() for an IOT project.  It is presently not used
  getDistance();
  if (double_distance<6) {
    alertSocialDistancing();
    showPoliceLights();
  }
}

void getMessages(){
  //This function was used for an IoT project.  It hit a web site that did some magic and returned a string of text back to this device.
  if (DEBUG) Serial.println("\nStarting connection to server...");
  // if you get a connection, report back via serial:
  if (client.connect(server, 80)) {
    if (DEBUG) Serial.println("connected to server");
    // Make a HTTP request:
    client.println("GET /index.aspx?User=sean HTTP/1.1");
    client.println("Host:  192.168.1.239");
    client.println("Connection: close");
    client.println();
  }
  int aborter=0;
  while (!client.available()){
      if (button_pressed) return;  //abort if someone presses the button
      handleThings();
      delay(200);
      if (aborter++ >30) {
        client.stop();
        return;  //just bale if it doesn't respond quickly.
      }
  }
  
  String the_response;
  while (client.available()) {
    char c = client.read();
    if (DEBUG) Serial.write(c);
    the_response+=c;

  }
  // if the server's disconnected, stop the client:
  if (!client.connected()) {
    if (DEBUG) Serial.println();
    if (DEBUG) Serial.println("disconnecting from server.");
    client.stop();
  }
  parseTheResponse(the_response);
  if (Status=="ALERT") {
    changed_display=true;
    cls();
    showOLEDMessage("TEAM AWESOME",1);
    showOLEDMessage("EXCELLENCE REPORTED!",2); //line 2 is 20 wide
    showOLEDMessage(" YEAH!!",3);
    
    showPoliceLights();
    button_pressed=false;//prevent overpressing
    showPoliceLights();
    setTheLightsPerTheVariables();
    button_pressed=false;//prevent overpressing
  }
}

void sendButtonMessage(){
  //This method was called when the button is pushed for an IoT project.  It is not used at the moment.
  if (DEBUG) Serial.println("\nStarting connection to server...");
  // if you get a connection, report back via serial:
  if (client.connect(server, 80)) {
    if (DEBUG) Serial.println("connected to server");
    // Make a HTTP request:
    client.println("GET /index.aspx?User=sean&Action=Alert HTTP/1.1");
    client.println("Host:  192.168.1.239");
    client.println("Connection: close");
    client.println();
  }
  int aborter=0;
  while (!client.available()){
      delay(200);
      if (aborter++ >30) {
        client.stop();
        return;  //just bale if it doesn't respond quickly.
      }
  }
  
  String the_response;
  while (client.available()) {
    char c = client.read();
    if (DEBUG) Serial.write(c);
    the_response+=c;

  }
  // if the server's disconnected, stop the client:
  if (!client.connected()) {
    if (DEBUG) Serial.println();
    if (DEBUG) Serial.println("disconnecting from server.");
    client.stop();
  }
  parseTheResponse(the_response);
  if (Status=="ALERT") {
    changed_display=true;
    cls();
    showOLEDMessage("TEAM AWESOME",1);
    showOLEDMessage("EXCELLENCE REPORTED!",2); //line 2 is 20 wide
    showOLEDMessage(" YEAH!!",3);
    
    showPoliceLights();    
  } else
  {
    changed_display=true;
    cls();
    showOLEDMessage("TEAM AWESOME",1);
    showOLEDMessage("EXCELLENCE REPORTED ",2); //line 2 is 20 wide
    showOLEDMessage("NO RESPONSE!",3);
   
    showPoliceLights();    
  
  }
  setTheLightsPerTheVariables();
}

void showPoliceLights()
{//  A routine that simulates police flashers
    for (int ii=0;ii<15;ii++) {
      LED1="RED";
      LED2="RED";
      LED3="RED";
      setTheLightsPerTheVariables();
      delay(100);
      if (button_pressed) break;
      LED1="BLUE";
      LED2="BLUE";
      LED3="BLUE";
      setTheLightsPerTheVariables();
      delay(100);      
    }  
}

void setTheLightsPerTheVariables()
{ //I use variables to track and set each LED's using a string.  That way my code can always refer to the state of the device.

  if (LED1=="RED") {
    setLED(1,"RED");
  }else if(LED1=="GREEN") {
    setLED(1,"GREEN");
  }else if (LED1=="YELLOW") {
    setLED(1,"YELLOW");
  }else {
    setLED(1,"BLUE");
  }

  if (LED2=="RED") {
    setLED(2,"RED");
  }else if (LED2=="GREEN") {
    setLED(2,"GREEN");
  }else if (LED2=="YELLOW") {
    setLED(2,"YELLOW");
  }else {
    setLED(2,"BLUE");
  }
  
  if (LED3=="RED") {
    setLED(3,"RED");
  }else if (LED3=="GREEN") {
    setLED(3,"GREEN");
  }else if (LED3=="YELLOW") {
    setLED(3,"YELLOW");
  }else{
    setLED(3,"BLUE");
  } 
}

void setLED(int LED, String color)
{ //used to directly set a light color.
  if (LED==1) {
    if (color=="YELLOW") {
      if (DEBUG) Serial.println("hit yellow");
      digitalWrite(9,HIGH);//
      digitalWrite(10,HIGH);
      digitalWrite(11,LOW);
    }
    else if (color=="GREEN") {
      digitalWrite(9,0);
      digitalWrite(10,HIGH);//
      digitalWrite(11,0);//  
    }
    else if (color=="BLUE")
    {
      digitalWrite(9,0);
      digitalWrite(10,0);//
      digitalWrite(11,HIGH);//
    }
    else if (color=="RED")
    {
      digitalWrite(9,HIGH);//
      digitalWrite(10,0);
      digitalWrite(11,0);//
    }
  } else if (LED==2) {
    //if (DEBUG) Serial.println("here +" + color + " " + INTENSITY);
    if (color=="YELLOW") {
      digitalWrite(3,HIGH);     
      digitalWrite(5,HIGH);
      digitalWrite(6,0);
    }
    else if (color=="GREEN") {
      digitalWrite(3,0);
      digitalWrite(5,HIGH);
      digitalWrite(6,0);        
    }
    else if (color=="BLUE")
    {
      digitalWrite(3,0);
      digitalWrite(5,0);
      digitalWrite(6,HIGH);
    } 
    else if (color=="RED")
    {
      digitalWrite(3,HIGH);
      digitalWrite(5,0);
      digitalWrite(6,0);
    }
  } else if (LED==3) {
      if (DEBUG) Serial.println("here +" + color);
      if (color=="YELLOW") {
        digitalWrite(2,0);     
        digitalWrite(16,HIGH);
        digitalWrite(17,HIGH);
      }
      else if (color=="GREEN") {
        digitalWrite(2,0);
        digitalWrite(16,0);
        digitalWrite(17,HIGH);        
      }
      else if (color=="BLUE")
      {
        digitalWrite(2,INTENSITY);
        digitalWrite(16,0);
        digitalWrite(17,0);
      }
      else if (color=="RED")
      {
        digitalWrite(2,0);
        digitalWrite(16,HIGH);
        digitalWrite(17,0);
      }
  }
}

void cls()
{   // clear the screen
    oledFill(&ssoled, 0x0, 1);
}

void showOLEDMessage(char *the_message, int the_line)
{ //
  switch (the_line)
  {
    case 1: oledWriteString(&ssoled, 0,16,0,the_message, FONT_NORMAL, 0, 1); break;
    case 2: oledWriteString(&ssoled, 0,0,1,the_message, FONT_SMALL, 1, 1); break;
    case 3: oledWriteString(&ssoled, 0,0,3,the_message, FONT_LARGE, 0, 1); break;
  }
}

void showIdleMessage()
{ //routine to show something when not in alert mode.
  if (!changed_display) return;
  changed_display=false;
  cls();
  showOLEDMessage("YOUR STATUS",1);
  showOLEDMessage("",2); //line 2 is 20 wide
  showOLEDMessage("AWESOME!",3);
}

void alertSocialDistancing()
{ //Called whenever you want to go into an alert mode due to an object being too close.
  cls();
  showOLEDMessage("   ALARM",1);
  showOLEDMessage("  !SOCIAL DISTANCE!  ",2); //line 2 is 20 wide
  showOLEDMessage("BACK UP!",3);
  changed_display=true;
}

void parseTheResponse(String the_response)
{   // used to parse out returned webpage text for IoT projects.
    // some roughhousing to split this up into our strings.
    the_response=getFromHereToThere(the_response,"<span id=\"my_response\"", "</span>");  // get the lines in between the span.
    
    Scroll=the_response.substring(0,the_response.indexOf('\r')); the_response=the_response.substring(Scroll.length()+2, the_response.length());
    LED1=the_response.substring(0, the_response.indexOf('\r')); the_response=the_response.substring(LED1.length()+2, the_response.length());
    LED2=the_response.substring(0, the_response.indexOf('\r')); the_response=the_response.substring(LED2.length()+2, the_response.length());
    LED3=the_response.substring(0, the_response.indexOf('\r')); the_response=the_response.substring(LED3.length()+2, the_response.length());
    Status=the_response.substring(0, the_response.indexOf('\r'));
    if (DEBUG) Serial.println("hello"+Scroll+LED1+LED2+LED3+Status+"goodbye");
}

String getFromHereToThere(String the_str, String the_start, String the_end)
{ //This helps parse a string.  It is used with IoT projects.  See parseTheResponse to get a message from a web page and parse out its content
  int ii=0;int start_index=-1;int end_index=-1;
  for (ii=0;ii<(the_str.length()-1);ii++) {
    if (the_str.substring(ii,ii+the_start.length())==the_start)
      start_index=ii;
    else if (the_str.substring(ii,ii+the_end.length())==the_end)
      {end_index=ii; break;}
  }
  if (start_index==-1||end_index==-1) return the_str; else return the_str.substring(start_index+the_start.length()+1,end_index);
}

void getDistance()
{ //the main method to measure distance.  It stores it in double_distance
  double_distance = ((((float)ultrasonic.read())/2.54)/12);  //feet
  Serial.println(double_distance);
  if (double_distance<.5) double_distance=8.0;
  memset(char_distance,0,sizeof(char_distance));
  dtoa(double_distance, char_distance, 1);
  String the_string(char_distance);
  the_string+=" ft";
  the_string.toCharArray(char_distance, the_string.length()+1);
  if (DEBUG) Serial.println(the_string);
}

char* dtoa(double dN, char *cMJA, int iP) {
  //Used to convert a double to an ascii value for showing on the serial monitor.
  char *ret = cMJA; long lP=1; byte bW=iP;
  while (bW>0) { lP=lP*10;  bW--;  }
  long lL = long(dN); 
  double dD=(dN-double(lL))* double(lP); 
  if (dN>=0) {
    dD=(dD + 0.5); 
  } else 
    { dD=(dD-0.5); }
  long lR=abs(long(dD));  lL=abs(lL);  
  if (lR==lP) { lL=lL+1;  lR=0;  }
  if ((dN<0) & ((lR+lL)>0)) { *cMJA++ = '-';  } 
  ltoa(lL, cMJA, 10);
  if (iP>0) { while (*cMJA != '\0') { cMJA++; } *cMJA++ = '.'; lP=10; 
  while (iP>1) { 
    if (lR< lP) { *cMJA='0'; cMJA++; } lP=lP*10;  iP--; }
    ltoa(lR, cMJA, 10); 
  }
  return ret; 
}

void printWifiStatus() {
  // Debugging method for print the SSID of the network you're attached to:
  if (!DEBUG) return; 
  
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void TimerCallback0(void)
{ //this is a timer callback method to do whatever you like every fraction of a second.  Allows a threaded approach.
}

void TC3_Handler() {
  Adafruit_ZeroTimer::timerHandler(3);
}

void timer_setup() {
 //this is a timer callback method to do whatever you like every fraction of a second.  Allows a threaded approach.

  // Set up the flexible divider/compare
  uint8_t divider  = 1;
  uint16_t compare = 0;
  tc_clock_prescaler prescaler = TC_CLOCK_PRESCALER_DIV1;

  if ((freq < 24000000) && (freq > 800)) {
    divider = 1;
    prescaler = TC_CLOCK_PRESCALER_DIV1;
    compare = 48000000/freq;
  } else if (freq > 400) {
    divider = 2;
    prescaler = TC_CLOCK_PRESCALER_DIV2;
    compare = (48000000/2)/freq;
  } else if (freq > 200) {
    divider = 4;
    prescaler = TC_CLOCK_PRESCALER_DIV4;
    compare = (48000000/4)/freq;
  } else if (freq > 100) {
    divider = 8;
    prescaler = TC_CLOCK_PRESCALER_DIV8;
    compare = (48000000/8)/freq;
  } else if (freq > 50) {
    divider = 16;
    prescaler = TC_CLOCK_PRESCALER_DIV16;
    compare = (48000000/16)/freq;
  } else if (freq > 12) {
    divider = 64;
    prescaler = TC_CLOCK_PRESCALER_DIV64;
    compare = (48000000/64)/freq;
  } else if (freq > 3) {
    divider = 256;
    prescaler = TC_CLOCK_PRESCALER_DIV256;
    compare = (48000000/256)/freq;
  } else if (freq >= 0.75) {
    divider = 1024;
    prescaler = TC_CLOCK_PRESCALER_DIV1024;
    compare = (48000000/1024)/freq;
  } else {
    while (1) delay(10);
  }
  zerotimer.enable(false);
  zerotimer.configure(prescaler,       // prescaler
          TC_COUNTER_SIZE_16BIT,       // bit width of timer/counter
          TC_WAVE_GENERATION_MATCH_PWM // frequency or PWM mode
          );

  zerotimer.setCompare(0, compare);
  zerotimer.setCallback(true, TC_CALLBACK_CC_CHANNEL0, TimerCallback0);
  zerotimer.enable(true);
}
