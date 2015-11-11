/*
  Stu's IOT station Core Code
aa
Arduno Pro Mini
Pin 0 - serial
Pin 1 - serial
Pin 2 - 74hc595   Data
Pin 3 -           clock
Pin 4 - 			reset
Pin 5 - 			latch data finished
Pin 6 -led
Pin 7 - led
Pin 8 -led
Pin 9 - ESP 8266 reset pin
Pin 10 - SoftwareSerial RX
Pin 11 - Softserial TX
Pin 12 - PIR
Pin 13 - LED

Pin A0 - light dependant resistor
Pin A1 - 
Pin A2
Pin A3
Pin A4`I2C
Pin A5 I2C

AT Commands:
AT+CWMODE=1 – set the module operating mode to Station.
AT+RST – reset the module to finish changing the operating mode.
AT+CIPMUX=0 – only need 1 connection for now - server later?.
AT+CWLAP – check the available access points to connect to.
AT+CWJAP=”ssid”,”password” to connect to the access point.
AT+CIFSR – check the module ip and connection status.

AT+CIPSTART=4,”TCP”,”faceplateio.azurewebsites.net”,80 – start the connection 
XX cover for DNS fail - just retry the command)

AT+CIPSEND=4,42 -send 42 characters
When you receive > from the module send the following data (replacing \r\n with CR and LF characters):

GET /status.html HTTP/1.0\r\nHost: 192.168.88.35\r\n\r\n

You should get SEND OK and after that the content of the webpage requested in +IPD format. 

The +IPD frames will be maximum of 1024 bytes long. 
If the webpage content is larger it will be split into several frames.
AT+CIPCLOSE=4 to close the connection

Query AT+CIPSTAMAC? +CIPSTAMAC:mac OK Print current MAC ESP8266’s address. 

URL format:
	?mto= destination IP6 address
	&mfrom = my IP6 address
	&msg= message

	Message: (parser needs to strip numbers including decimals)
	Lux number as string
	Tmp number
	Prs number
	Pir number
	

  */
  
// definitions  
#define Enable_SR true
#define SW_SERIAL_RX_PIN 10
#define SW_SERIAL_TX_PIN 11
#define ESP_RESET_PIN 9  
#define char_CR 0x0D 
#define DEBUG_BMP true 
#define DEBUG_I2C true
#define DEBUG_WIFI true
#define ESPWAIT 5000
#define RledPin 8  // LED pin
#define GledPin 7
#define BledPin 6
#define LDRPin A0
#define PIRPin 12  // PIR detector Pin
#define SR_Data 2  // shift register Pin 14
#define SR_Clock 3 // shift register Pin 11
#define SR_Reset 4 // shift register Pin 10
#define SR_Latch 5 // shift register Pin 12
// includes 
#include "SoftwareSerial.h"
#include <Wire.h>
#include <Time.h>
#include <SFE_BMP180.h>
#include <String.h>
#include <avr/wdt.h>// watchdog

boolean reportPressure = false;
boolean reportTemperature = false;
boolean reportAltitude = false;
boolean reportPresence = false;
boolean echo_txrx = true; // echo the esp to the serial port


// int attempt=0;
// int retries =5;
int reset_counter=0;
int relay_value=0x01;

//String version = "1.0";
String myIPv6 = "1:1:1::0";
String serverIPv6 ="1:1:0";
// setup software serial port
SoftwareSerial mySerial(SW_SERIAL_RX_PIN, SW_SERIAL_TX_PIN); // RX, TX
//int i2cdevicecount = 0 ; // count of connected i2c devices
int val=0;
time_t timeNow;    // storage for local clock
//long millisNow;
long sendTimer;
long sendInterval = 120000;  // 2 mins
long sendCounter=0;
long rcvTimer=0;
long rcvInterval=30000;  // 30 secs
long rcvCounter=0;
boolean presence=false;
boolean lastPresence=false;

int wifi_State = 0 ;

String 	mySSID ="Q3";
String myPassword = "pollynet";
String myServer="faceplateio.azurewebsites.net";
String myPort="80";
boolean packetToSendFlag = false;
boolean packetToReceiveFlag=false;
char ESPmsgbuffer[128];
int ESPmsgCursor =0;
boolean ESPokFlag = false;
boolean ESPnoChangeFlag = false;
boolean ESPerrMsgFlag = false;
boolean ESPconnectFlag=false;
boolean ESPreadyFlag=false;
boolean ESPlinkedFlag=false;
boolean ESPfailFlag=false;
		
SFE_BMP180 pressure_sensor; // define the BMP180
double BMPaltitude,BMPpressure;//BMP180 variables
double BMPbaselinepressure; // baseline pressure
double BMPtemperature;  // Temperature of device
//double BMPTemperatureHistory[48];
//double BMPPressureHistory[48]; // 48 hour history

void setup() {// the setup function runs once when you press reset or power the board
	int ret;
	ret=setupPins(); // set the io pins
	ret=setupSerial(); // setup the serial comms
	ret=setupBMP(); // initialize the pressure sensor
	ret = getPressure();
	BMPbaselinepressure=BMPpressure;
	// hardResetESP(); // reset the wifi chip 
	flashme(8);           // flash to show we're alive 
	// scani2c(); // see what's out there 
	setTimers(); // setup the timed task timers	
}
int setupPins(){
	pinMode(LDRPin,INPUT);
	pinMode(RledPin,OUTPUT);
	pinMode(GledPin,OUTPUT);
	pinMode(BledPin,OUTPUT);
	pinMode(SR_Data,OUTPUT);
	pinMode(SR_Clock,OUTPUT);
	pinMode(SR_Reset,OUTPUT);
	pinMode(SR_Latch,OUTPUT);
	digitalWrite(SR_Latch,LOW);
	digitalWrite(SR_Reset,HIGH);
	digitalWrite(SR_Clock,LOW);
	
	digitalWrite(RledPin,HIGH);
	digitalWrite(GledPin,HIGH);
	digitalWrite(BledPin,HIGH);
// initialize digital pin 13 as an output.
	
	pinMode(13, OUTPUT);
  pinMode(12, INPUT);  // from PIR
  pinMode(ESP_RESET_PIN,OUTPUT); // ESP reset 
  digitalWrite(ESP_RESET_PIN,HIGH); // set it high
	return 0;
  }
int setupSerial(){
	 // Open serial communications and wait for port to open:
   Serial.begin(19200);
   delay(200);
   Serial.println("I'm awake.. are you?");
  // set the data rate for the SoftwareSerial port
   mySerial.begin(9600);
   delay(200);
	return 0;
   }
void ledColour(int d){
	digitalWrite(RledPin,HIGH);
	digitalWrite(GledPin,HIGH);
	digitalWrite(BledPin,HIGH);
		char c=d;
	if((c  & B00000001)>0){
	digitalWrite(RledPin,LOW);
	}
	if((c & B00000010)>0){
		digitalWrite(BledPin,LOW);
	}
	if((c & B00000100)>0){
		digitalWrite(GledPin,LOW);
		}
} 
void setColour(int r,int g,int b){
	analogWrite(RledPin,r);
	analogWrite(GledPin,g);
	analogWrite(BledPin,b);
}
int setTimers(){
	sendTimer=millis()+sendInterval;
	rcvTimer=millis()+rcvInterval;
}
void flashme(int times){
	for(int z=0;z<times;z++){
		digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
		delay(200);              // wait for a second
		digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
		delay(200); 
	}
}
// the loop function runs over and over again forever
void loop() {
 int ret=0;
ret=checkPresence();
if(echo_txrx){
  ret=checkSerial(); //disable echo for a moment
}
ret=checkWifi(); // look for wifi events 

ret=checkTimers(); // check for events due

}
void setShiftReg(char v){
Serial.print("Setting:");
Serial.print(v,HEX);  
  
  
  if(Enable_SR){
  // send data to Shift register
  
  // reset device
  digitalWrite(SR_Reset,LOW);
  delay(2);
  digitalWrite(SR_Reset,HIGH);
  delay(2);
  Serial.print("Relays");
  // iterate thru bits
	for(int z=0;z<8;z++){
		if(( v & 0x80)>0){
			digitalWrite(SR_Data,LOW);
			Serial.print("1");  
		} else{
			digitalWrite(SR_Data,HIGH);
			Serial.print("0");
		}
		v*=2;
		// cycle clock pin
		digitalWrite(SR_Clock,HIGH);
		delay(2);
		digitalWrite(SR_Clock,LOW);
		delay(2);
	}
  // set output latch
  digitalWrite(SR_Latch,HIGH);
  delay(2);
  digitalWrite(SR_Latch,LOW);
  Serial.println();
  }
}

int checkWifi(){
	// wifi state
	// 0 - unknown
	// 1 - sent reset
	// 2 - reset ok
	// 3 - sent mode
	// 4 - mode 0k
	// 5 - sent connectWifi
	// 6 - connected
	// 7 - asking for IP
	// 8 - got IP
	// 9 - sending packet
	int ret;
	//if(packetToSendFlag){
		switch(wifi_State){
			case 0:
				if(initESP()){
					setState(2);
				} 
				else{
					// try a reset
					setState(1);
				};
				printFlags();
			break;
			case 1:
     
				// need to wait here as previous reset didn't work
				hardResetESP(); // reset our Wifi chip
				//flushESP(5000);
				setState(0);
     
			break;
			case 2:
				
				// ok so chip is up, need to set station mode for wifi
				if( ESPsetWifiMode()) {
				setState(3);
				}
				else{
					flushESP(500);
					setState(0);
				}
				
			break;
			case 3:
				
        
				if( connectWifi()) {
					setState(4);
				}
				else{
					flushESP(500);
					setState(2);
				}
        
			break;
			case 4:
			
				if(ESPsetMux()){
					setState(5);
				}
				else{
					flushESP(500);
					setState(2);
				}
			break;
			case 5:
				// connected
				if(printIP()){
					// Serial.println("We have IP!!");
					setState(6);
				}
				else{
					//sendBreak();
					setState(4);
				}
			break;
			case 6://  we are connected to Wifi and have an IP address
				// now we bind TCP to the server
				
				if(ESPbindTCP()){
					setState(7);
				}
				else{
					sendBreak();
					boolean z =ESPsipClose();	
					setState(5);
				}
				
			
			break;
			case 7:
			
			if(ESPsetMode()){
				setState(8);
			}
			else{
				setState(6);
			}

			break;
			case 8:  // we're in steady state
			if(packetToReceiveFlag){

				if(getPacket()){
					Serial.println("Got!");
					rcvData(10000);
					packetToReceiveFlag=false;
					setState(8);
				}
				else{
					Serial.println("NOTGot");
					if(!ESPsipClose()){
						flushESP(5000);				
						
					}
				setState(6);
				}
			}
			else{
				if(packetToSendFlag){

					if(sendPacket()){
						Serial.println("Sent!");
						rcvData(20000);
						packetToSendFlag=false;
						setState(8);
					}
					else{
						Serial.println("NOTSent");
						if(ESPsipClose()){
						}
						else
						{						
							if(!sendBreak){
								flushESP(5000);				
							}
						}
					setState(6);
					}
				}
			}
			break;
			default:
			// we're out of control so reset state to zero
			{
			setState(0);
			}
			break;
		}
	// }
	return wifi_State;
}
void doJson(){
	int z=0;
	int d=0;
	int total;
	char command=ESPmsgbuffer[2];
	if(command=='R'){ // relays
		// next 8 digits are our binary outputs
		char r = 0x00;
		Serial.print("Relay Command:");
		for( z=0;z<8;z++){
			r*=2;
			if(ESPmsgbuffer[3+z]=='1'){
				r+=1;
				Serial.print("1");
			}	
			else{
				Serial.print("0");
			}
		}
			Serial.print(":");
		Serial.println(r,HEX);
		setShiftReg(r);
    }
	if(command=='C'){ // colour 
		// next 9 are our RGB values
		Serial.print("Colour Command:");
		z=1;
		total=0;
		for(d=0;d<9;d++){
			total+= (ESPmsgbuffer[3+d]-0x30)*z;
			z=z*10;
		}
		// total is our 9 digit number
		int r = total/1000000;
		int g = total/1000 % 1000;
		int b = total % 1000;
		setColour(r,g,b);
	}
		
}
boolean ESPsipClose(){
	mySerial.println("AT+CIPCLOSE");
	return ESPwaitMsg(6,1000);
}
boolean sendBreak(){
	mySerial.print("+++");
	delay(20);
	mySerial.println("+++");
	return ESPwaitMsg(2,2000);
	
}
void setState(int s){
	Serial.println("");
	Serial.println("State:"+(String)wifi_State+" To:"+(String) s);
	delay(100);
	wifi_State=s;
	ledColour(s);
}
int checkSerial(){
	int q=0;
if (mySerial.available()) {
     Serial.write(mySerial.read());
	q=q+1;
   }
   if (Serial.available()) {
     mySerial.write(Serial.read());
	 q=q+10;
   }
   return q;
}
int getLight(){
	return analogRead(LDRPin);
}
int checkPresence(){
  val= digitalRead(12);
	digitalWrite(13,val);
	if (val>0){
	  presence=true;
	} else { presence=false;}

	if(presence!=lastPresence){
	 lastPresence=presence;
	  Serial.print("Presence:");
	  Serial.println(presence);
	}
	return val;
}
boolean ESPbindTCP(){
	// bind a tcp socket to the server
	String client_conn = "AT+CIPSTART=\"TCP\",\"";
	client_conn += myServer;
	client_conn += "\",";
	client_conn += myPort;
	mySerial.println(client_conn);
	if(ESPwaitMsg(6,2000)){
		if(ESPconnectFlag || ESPokFlag || ESPlinkedFlag){
			if(ESPwaitMsg(8,2000)){	
			return true;
			}
		}	
	}
	return false;
}
boolean getPacket(){ // get data packet from host
	// Create Request
	String data1 = "GET /api/io/";
	data1 += "?mto=";
	data1 += myIPv6;
	data1 +=" HTTP/1.0\r\n";
	
	String data2 ="Host: ";
	data2 +=myServer;
	data2+="\r\n\r\n";
	
	int datalength=data1.length()+data2.length();
	String cmd ="AT+CIPSEND=";
	cmd += datalength;
	mySerial.println(cmd);
	if(mySerial.find(">")){
		mySerial.print(data1); // send data
		delay(5);
		mySerial.print(data2); // send data
		int count = ESPgetLine(2000);
			count = ESPgetLine(2000); // absorb the echos
		if(ESPwaitMsg(8,2000)){ // and wait for SEND OK
			if(ESPokFlag){
				return true;
			}
		
			else{
				Serial.println("GET:no OK to CIPSEND");
			}
		}
	} 
	else{
		Serial.println("GET:CIPSEND no > found");
	}
	return false;
}
boolean sendPacket(){ // send data packet to host
	// build our telemetry packet
	String msg ="Lux"+(String) getLight();
	msg+="Tmp"+(String) BMPtemperature;
	msg+="Prs"+(String) BMPpressure;
	msg+="Pir"+(String) digitalRead(PIRPin);
	
	// Create Request
	String data1 = "GET /api/io/";
	data1 += "?from=";
	data1 += myIPv6;
	data1 += "&to=";data1 += serverIPv6;
	data1 += "&msg=";
	data1 +=msg;
	data1 +=" HTTP/1.0\r\n";
	
	String data2 ="Host: ";
	data2 +=myServer;
	data2+="\r\n\r\n";
	int datalength=data1.length()+data2.length();
	// send it
	String cmd ="AT+CIPSEND=";	
	cmd+=datalength;
	mySerial.println(cmd);
	//Serial.print(data1);	// send CIPSEND command
	delay(10);
	if(mySerial.find(">")){
					
		mySerial.print(data1); // send data
		delay(5);
		mySerial.print(data2); // send data
		int count = ESPgetLine(2000);
		count = ESPgetLine(2000); // absorb the echos
		if(ESPwaitMsg(8,2000)){ // and wait for SEND OK
			if(ESPokFlag){
				return true;
			}
			else{
				Serial.println("PUT:CIPSEND no ok ");
			}
		}
	}
	else{
		Serial.println("PUT:CIPSEND no > found");		
	}
	return false;	
}
int printPage(int milliseconds){
	double t = millis()+milliseconds;
	ESPmsgCursor	=0;
	boolean line 	=false;
	int 	mycount		=0;
	while(millis()<t){
		mycount+=ESPgetLine(1000);
	}
	return mycount;
}
int  checkTimers(){
	int q=0;
	timeNow=millis();
	if(timeNow>sendTimer){
		sendTimer=timeNow+sendInterval;

		double p = getPressure();
		if(p>0){
			packetToSendFlag=true; // send forever for debug
			q=q+1;
		}
		else{
			Serial.println("Failed to read BMP180");
		}
	}
	
	if(timeNow>rcvTimer){
		rcvTimer=timeNow+rcvInterval;
		packetToReceiveFlag=true;
		q=q+1;
	}
	return q;
}
void rcvData(int n){
	
	if(printPage(n)>0){
		Serial.print("Received:");
			if(ESPmsgCursor>0){
				for(int z=0;z<ESPmsgCursor;z++){
					Serial.print(ESPmsgbuffer[z]);
				}
			// Interpret JSON
			doJson();
			}
		Serial.println();
	} 
}
int setupBMP(){
	if (pressure_sensor.begin())
    Serial.println("BMP180 init success");
  else
  {
    Serial.println("BMP180 init fail (disconnected?)");
  }
}
void flushESP(int msecs){
	long flushtime = millis()+(msecs);
	char r;
	while(millis()<flushtime){
	 if(mySerial.available()){
			r = mySerial.read();
			Serial.print(r);
	 }
	}
	Serial.println("<Flushed");
	delay(10);
}
void printFlags(){
	if(ESPokFlag) Serial.println("OK FLAG!");			
	if(ESPnoChangeFlag)Serial.println("No change FLAG!");
	if(ESPerrMsgFlag)Serial.println("Error FLAG!");
}
void clearFlags(){
	ESPokFlag=false;
	ESPnoChangeFlag=false;
	ESPerrMsgFlag=false;
	ESPconnectFlag=false;
	ESPreadyFlag=false;
	ESPlinkedFlag=false;
	ESPfailFlag=false;	
}
boolean initESP(){
	// wifi state
	// 0 - unknown
	// 1 - sent reset
	// 2 - reset ok
	
	// see if ESP is awake
	//flushESP(2000);
	// if(mySerial.find("ready")){
	boolean line=false;
	char r='N';
	Serial.println("Nudge ESP..");
	// mySerial.println("AT+RST");
	mySerial.println("AT");
	if(ESPwaitMsg(5,1000)){
		flushESP(3000);
			return true;
	  }		
	return false;
}
boolean chkFlags(){
	if(ESPfailFlag || ESPokFlag || ESPnoChangeFlag || ESPerrMsgFlag || ESPconnectFlag || ESPreadyFlag || ESPlinkedFlag){
			return true;
			}
	return false;
}	
boolean ESPwaitMsg(int lines, int msecs){
	clearFlags();  // clear the status flags
	for(int z=0;z<lines;z++){
		int n = ESPgetLine(msecs);
		if(chkFlags()){
			return true;
			}
	}
	return false;
}
int ESPgetLine(int msecs){
	// reads a line from ESP
	// returns character count
	// message is in msgbuffer[]
	long tim = millis();
    char r=0x00;
	char last=0x00;
	char beforethat=0x00;
	int charCount=0;
	boolean line=false;
	boolean json=false;
	while(millis()<tim+msecs){
		
		if(mySerial.available()){
			r = mySerial.read();
			charCount++;
			if(r=='[') {   // start of json capture
				json=true;
				ESPmsgCursor=0;
			}
			if(r==']') json=false; // end json
			if(json){
				ESPmsgbuffer[ESPmsgCursor]=r;
				ESPmsgCursor++;
				if(ESPmsgCursor>126){
					ESPmsgCursor=0;
					json=false;
				}
			}
			
			if(r==char_CR){
				line=true;
				Serial.println();
			}
			else{
				Serial.write(r);	
			}
			
			beforethat=last;
			last=r;
		}	
		
		if(line){
			// ESP Responded
			char test= beforethat;
			
			if(test=='K'){
				ESPokFlag=true;	
			}
			if(test=='y'){
				ESPreadyFlag=true;
			}
			if(test=='e'){
				ESPnoChangeFlag=true;
			}
			if(test=='R'){
				ESPerrMsgFlag=true;
			}	
			if(test=='T'){
				ESPconnectFlag=true;
			}	
			if(test=='d'){
				ESPlinkedFlag=true;
			}
			if(test=='L'){
				ESPfailFlag=true;
			}
		return charCount;
		}
	}
	return charCount;
}
boolean ESPsetWifiMode(){
	
	//flushESP(1000);
	mySerial.println("AT+CWMODE=1");
	
	if(ESPwaitMsg(5,2000)){
		if(!ESPerrMsgFlag){
			return true;
		}
		
	}
	return false;
	
}
boolean ESPsetMux(){
	//flushESP(500);
	mySerial.println("AT+CIPMUX=0");
	if(ESPwaitMsg(5,1000)){
		if(ESPokFlag){
			return true;
			// ***** SET MODE WENT HERE
		}
	}
	return false;
}
boolean ESPsetMode(){
	mySerial.println("AT+CIPMODE=0");
			if(ESPwaitMsg(5,1000)){
				if(ESPokFlag){
					// send +RST to lock in the params
					//mySerial.println("AT+RST");
					//delay(2000);
					return true;
				}
			}
}
boolean connectWifi(){
	
		String cmd2;
	//	cmd2.reserve(32);
		cmd2="AT+CWJAP=";  // 9 chars
		//cmd2="AT+CWJAP=";  // 9 chars
		//Serial.println(cmd2);
		cmd2+="\"";				// 10
		cmd2+=mySSID;			// 13
		//Serial.println(cmd2);
		cmd2+="\"";				//14
		cmd2+=",";
		cmd2+="\"";				//16
		//Serial.println(cmd2);
		cmd2+=myPassword;		//24	
		//Serial.println(cmd2);
		cmd2+="\"";				//25
		//Serial.println(cmd2);
		mySerial.println(cmd2);
		// delay(500);
		if(ESPwaitMsg(20,2000)){
			if(!ESPerrMsgFlag || ESPfailFlag){
				return true;
			}
		}
	return false;
}
boolean printIP(){

	mySerial.println("AT+CIFSR");
	delay(100);
	if(ESPwaitMsg(6,3000)){
		if(ESPerrMsgFlag){ // ignore the first response
			if(ESPwaitMsg(6,3000)){
				if(ESPokFlag){
					return true;
				}
			}
		}
	}
	return false;
}
void hardResetESP(){
	if(reset_counter<10){	
                flashme(2);
		digitalWrite(ESP_RESET_PIN,LOW);
		delay(200);
		digitalWrite(ESP_RESET_PIN,HIGH);
		Serial.println("**ESP Hard Reset**");	
	} 
	else {
			Serial.print("RST Count exceeded:");
			Serial.println(reset_counter);
	}
	reset_counter++;
	flushESP(5000); // pick up trash for 5 seconds
}
double getPressure() {
  char status;
  double T,P,p0,a,nullpoint;

  // You must first get a temperature measurement to perform a pressure reading.
  
  // Start a temperature measurement:
  // If request is successful, the number of ms to wait is returned.
  // If request is unsuccessful, 0 is returned.

  status = pressure_sensor.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:

    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Use '&T' to provide the address of T to the function.
    // Function returns 1 if successful, 0 if failure.

    status = pressure_sensor.getTemperature(BMPtemperature);
    if (status != 0)
    {
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pressure_sensor.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Use '&P' to provide the address of P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.
		T=BMPtemperature;
        status = pressure_sensor.getPressure(P,BMPtemperature);
        if (status != 0)
        {
			BMPpressure=P;
			BMPaltitude=BMPpressure-BMPbaselinepressure;
			return(P);
        }
        else if(DEBUG_BMP)Serial.println("BMPERR retrieving pressure\n");
      }
      else if(DEBUG_BMP)Serial.println("BMPERR starting pressure\n");
    }
    else if(DEBUG_BMP)Serial.println("BMPERR retrieving temp\n");
  }
  else if(DEBUG_BMP)Serial.println("BMPERR starting Temp\n");
  return(0);
}
int I2C_ClearBus() {
	/**
 * This routine turns off the I2C bus and clears it
 * on return SCA and SCL pins are tri-state inputs.
 * You need to call Wire.begin() after this to re-enable I2C
 * This routine does NOT use the Wire library at all.
 *
 * returns 0 if bus cleared
 *         1 if SCL held low.
 *         2 if SDA held low by slave clock stretch for > 2sec
 *         3 if SDA held low after 20 clocks.
 */
	if(DEBUG_I2C)Serial.println("Clearing I2C");
  TWCR &= ~(_BV(TWEN)); //Disable the Atmel 2-Wire interface so we can control the SDA and SCL pins directly

  pinMode(SDA, INPUT_PULLUP); // Make SDA (data) and SCL (clock) pins Inputs with pullup.
  pinMode(SCL, INPUT_PULLUP);

  delay(500);  // Wait 2.5 secs. This is strictly only necessary on the first power
  // up of the DS3231 module to allow it to initialize properly,
  // but is also assists in reliable programming of FioV3 boards as it gives the
  // IDE a chance to start uploaded the program
  // before existing sketch confuses the IDE by sending Serial data.

  boolean SCL_LOW = (digitalRead(SCL) == LOW); // Check is SCL is Low.
  if (SCL_LOW) { //If it is held low Arduno cannot become the I2C master. 
    return 1; //I2C bus error. Could not clear SCL clock line held low
  }

  boolean SDA_LOW = (digitalRead(SDA) == LOW);  // vi. Check SDA input.
  int clockCount = 20; // > 2x9 clock

  while (SDA_LOW && (clockCount > 0)) { //  vii. If SDA is Low,
    clockCount--;
  // Note: I2C bus is open collector so do NOT drive SCL or SDA high.
    pinMode(SCL, INPUT); // release SCL pullup so that when made output it will be LOW
    pinMode(SCL, OUTPUT); // then clock SCL Low
    delayMicroseconds(10); //  for >5uS
    pinMode(SCL, INPUT); // release SCL LOW
    pinMode(SCL, INPUT_PULLUP); // turn on pullup resistors again
    // do not force high as slave may be holding it low for clock stretching.
    delayMicroseconds(10); //  for >5uS
    // The >5uS is so that even the slowest I2C devices are handled.
    SCL_LOW = (digitalRead(SCL) == LOW); // Check if SCL is Low.
    int counter = 20;
    while (SCL_LOW && (counter > 0)) {  //  loop waiting for SCL to become High only wait 2sec.
      counter--;
      delay(100);
      SCL_LOW = (digitalRead(SCL) == LOW);
    }
    if (SCL_LOW) { // still low after 2 sec error
      return 2; // I2C bus error. Could not clear. SCL clock line held low by slave clock stretch for >2sec
    }
    SDA_LOW = (digitalRead(SDA) == LOW); //   and check SDA input again and loop
  }
  if (SDA_LOW) { // still low
    return 3; // I2C bus error. Could not clear. SDA data line held low
  }

  // else pull SDA line low for Start or Repeated Start
  pinMode(SDA, INPUT); // remove pullup.
  pinMode(SDA, OUTPUT);  // and then make it LOW i.e. send an I2C Start or Repeated start control.
  // When there is only one I2C master a Start or Repeat Start has the same function as a Stop and clears the bus.
  /// A Repeat Start is a Start occurring after a Start with no intervening Stop.
  delayMicroseconds(10); // wait >5uS
  pinMode(SDA, INPUT); // remove output low
  pinMode(SDA, INPUT_PULLUP); // and make SDA high i.e. send I2C STOP control.
  delayMicroseconds(10); // x. wait >5uS
  pinMode(SDA, INPUT); // and reset pins as tri-state inputs which is the default state on reset
  pinMode(SCL, INPUT);
  return 0; // all ok
}
