/*
  Stu's IOT station Core Code

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

AT+CIPSEND=42 -send 42 characters
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
	
JSON return:
Crrrgggbbb	change colour
Rnnnnnnnn	set relay outputs

  */
  
// definitions  
#define Enable_SR true
#define ENABLE_WATCHDOG false
#define MAX_JSON 64
#define EE_BLOCKSIZE 64 // must be less <= MAXJSON
#define SW_SERIAL_RX_PIN 10
#define SW_SERIAL_TX_PIN 11
#define ESP_RESET_PIN 9  
#define char_CR 0x0D 
#define DEBUG_BMP true 
#define DEBUG_WIFI true
#define DEBUGWATCHDOG true
#define ESPWAIT 5000
#define RledPin 3  // LED pwm pin
#define GledPin 5 // LED pwm pin
#define BledPin 6 // LED pwm pin
#define LDRPin A0 // Light sensor pin
#define PIRPin 12  // PIR detector Pin
#define SR_Data 2  // shift register Pin 14
#define SR_Clock 8 // shift register Pin 11
#define SR_Reset 4 // shift register Pin 10
#define SR_Latch 7 // shift register Pin 12
// includes 
#include "SoftwareSerial.h"
#include <Wire.h>
#include <Time.h>
#include <SFE_BMP180.h>
#include <String.h>
// #include <avr/wdt.h> // watchdog
#include <EEPROM.h>

boolean echo_txrx = true; // echo the esp to the serial port

int reset_counter=0;
// these values get overwritten by eeprom contents
String 	mySSID ="Q3";
String myPassword = "wpa-key";
String myServer="faceplateio.azurewebsites.net";
String myPort="80";
String myKey="QQQQQ";
String myIPv6 = "1:1:1::0";
String serverIPv6 ="1:1:0::0";

// setup software serial port
SoftwareSerial mySerial(SW_SERIAL_RX_PIN, SW_SERIAL_TX_PIN); // RX, TX

int val=0;
long timeNow;    // storage for local clock
long sendTimer;
long sendInterval = 300003;  // 5 mins
long sendCounter=0;
long rcvTimer=0;
long rcvInterval=30007;  // 30 secs
long rcvCounter=0;
boolean presence=false;
boolean lastPresence=false;

int wifi_State = 0 ;

boolean packetToSendFlag = false;
boolean packetToReceiveFlag=false;
char ESPmsgbuffer[MAX_JSON];
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

void setup() {		// the setup function runs once when you press reset or power the board
	//wdt_disable(); 	// disable the watchdog
	int ret;
	setupPins(); 	// set the io pins
	setupSerial(); 	// setup the serial comms
	ret=setupBMP(); // initialize the pressure sensor
	ret = getPressure();
	BMPbaselinepressure=BMPpressure;
	// hardResetESP(); // reset the wifi chip 
	flashme(8);           // flash to show we're alive 
	// scani2c(); // see what's out there 
	getConfig(); // get the configuration details
	setTimers(); // setup the timed task timers	
	//if(ENABLE_WATCHDOG)watchdogSetup(); // start the watchdog
	//setShiftReg(0); // set relays to off
}
/*
ISR(WDT_vect) {// Watchdog timer interrupt.
	//  careful not to use functions they may cause the interrupt to hang and
	// prevent a reset.
	wdt_disable();
	if(DEBUGWATCHDOG)Serial.println("Watchdog!!");

	// flash pin 13 to signal error
	pinMode(13,OUTPUT);
	for(int n=0;n<10;n++){
		digitalWrite(13,LOW);
		delay(20);
		digitalWrite(13,HIGH);
		delay(20);
	}
	asm volatile ("  jmp 0"); // Bootstrap back to zero
}

/*
void watchdogSetup(void) {
	if(DEBUGWATCHDOG)Serial.println("Setting Up Watchdog");
// cli(); // disable all interrupts
wdt_reset(); // reset the WDT timer

// WDTCSR configuration:
// WDIE = 1: Interrupt Enable
// WDE = 1 :Reset Enable
// WDP3 = 0; // :For 2000ms Time-out
// WDP2 = 1; // :For 2000ms Time-out
// WDP1 = 1; // :For 2000ms Time-out
// WDP0 = 1; // :For 2000ms Time-out


// Enter Watchdog Configuration mode:
WDTCSR |= (1<<WDCE) | (1<<WDE);
// Set Watchdog settings:
WDTCSR =  (1<<WDP3) | (1<<WDP2) | (1<<WDP1) | (0<<WDP0)  | (1<<WDIE);
wdt_reset(); // reset the WDT timer
//wdt_enable(); // enable watchdog
// sei(); // enable interrupts
pinMode(13,OUTPUT);
digitalWrite(13,LOW); // signal by pin 13 low
if(DEBUGWATCHDOG)Serial.println("Watchdog set");

}
*/
String getSerial(){
	String input="";
	boolean CRflag=false;
	char c[2]={0x00,0x00};
	char i;
	//c[0]='*';
	//c[1]=0x00;
	while (!CRflag){
		while(Serial.available()){
			i = Serial.read();
			if(i==char_CR){
				CRflag=true;
			}
			else{
			c[0]=i;
			input+=c;
			Serial.write(i);	
			}
		}
	}
	Serial.println("I:"+input);
	return input;
}
boolean validateConfig(){
	Serial.println("Validate Config");
	Serial.println("SSID:"+mySSID);
	//Serial.println(mySSID);
	Serial.println("WiFi Key:" + myPassword);
	// Serial.println( myPassword);
	Serial.println("Server:" + myServer);
	// Serial.println(myServer);
	Serial.println("Port:"+ myPort);
	// Serial.println( myPort);
	Serial.println("MyIPV6:"+myIPv6);
	//Serial.println( myIPv6);
	Serial.println("ServerIPV6:"+serverIPv6);
	// Serial.println( serverIPv6);
	Serial.print("Correct Y/N?");
	String inLine=getSerial();
	Serial.println();
	inLine.trim();
	if(inLine=="Y"){
		return	true;
	}
	return false;
}
boolean saveConfig(){
	clearBuffer();
	mySSID.toCharArray(ESPmsgbuffer,mySSID.length()+1);
		eepromWriteBlock(1);
	clearBuffer();
	myPassword.toCharArray(ESPmsgbuffer,myPassword.length()+1);
		eepromWriteBlock(2);
	clearBuffer();
	myServer.toCharArray(ESPmsgbuffer,myServer.length()+1);
		eepromWriteBlock(3);
	myPort.toCharArray(ESPmsgbuffer,myPort.length()+1);
	eepromWriteBlock(4);
	clearBuffer();
	myIPv6.toCharArray(ESPmsgbuffer,myIPv6.length()+1);
	eepromWriteBlock(5);
	
	clearBuffer();
	serverIPv6.toCharArray(ESPmsgbuffer,serverIPv6.length()+1);
	eepromWriteBlock(6);
	
	clearBuffer();
	myKey.toCharArray(ESPmsgbuffer,myKey.length()+1);
	eepromWriteBlock(7);
	
	clearBuffer();
	ESPmsgbuffer[0]='s';
	ESPmsgbuffer[1]='t';
	ESPmsgbuffer[2]='u';
	eepromWriteBlock(0);
	// check to make sure it's still valid
	readConfig();
	if(validateConfig()) {
		return true;
	}

	return false;
}
boolean readConfig(){
		eepromReadBlock(1);
		mySSID=ESPmsgbuffer;
		eepromReadBlock(2);
		myPassword=ESPmsgbuffer;
		eepromReadBlock(3);
		myServer=ESPmsgbuffer;
		eepromReadBlock(4);
		myPort=ESPmsgbuffer;
		eepromReadBlock(5);
		myIPv6=ESPmsgbuffer;
		eepromReadBlock(6);
		serverIPv6=ESPmsgbuffer;
		eepromReadBlock(7);
		myKey=ESPmsgbuffer;

	return false;
}
void clearBuffer(){
	for(int z=0;z<MAX_JSON;z++){
		ESPmsgbuffer[z]=0x00;
	}
}
boolean eepromWriteBlock(int block){ // gets Block n into messagebuffer
	for(int z=0;z<EE_BLOCKSIZE;z++){
		EEPROM.write(block*EE_BLOCKSIZE+z,ESPmsgbuffer[z]);
	}
	return true;
}
boolean eepromReadBlock(int block){ // gets Block n into messagebuffer
	ESPmsgCursor=0;
	for(int z=0;z<EE_BLOCKSIZE;z++){
		ESPmsgbuffer[z]=EEPROM.read((block*EE_BLOCKSIZE+z));
	}
	return true;
}
/* End of Setup routines - now the runtime code */
void loop() {  			// the loop function runs over and over again forever
//	wdt_reset(); 		// reset the watchdog
	int ret=0;
	ret=checkPresence();
	if(echo_txrx){
		ret=checkSerial(); //echo for a moment
	}
	ret=checkWifi(); // look for wifi events 
	ret=checkTimers(); // check for events due

}
void setShiftReg(char v){
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
		if(( v & 0x01)>0){
			digitalWrite(SR_Data,LOW);
			Serial.print("1");  
		} else{
			digitalWrite(SR_Data,HIGH);
			Serial.print("0");
		}
		v=v>>1;
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
	// 8 - got IP - steady state
	// 9 - sending packet / busy
	//int ret;
	//if(packetToSendFlag){
		switch(wifi_State){
			case 0:
				Serial.println("Nudge ESP");
				if( ESPcommand("AT",6,1000)){
					setState(2);
				} 
				else{
					// try a reset
					setState(1);
				};
				
			break;
			case 1:
     
				// need to wait here as previous reset didn't work
				hardResetESP(); // reset our Wifi chip
				//flushESP(5000);
				setState(0);
     
			break;
			case 2:
				// ok so chip is up, need to set station mode for wifi
				// set wifi mode
				if( ESPcommand("AT+CWMODE=1",6,3000)) {
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
				// set the session mux
				if(ESPcommand("AT+CIPMUX=0",5,1000)){
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
					// sendBreak();
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
						flushESP(10000);
						if(ESPsipClose()){
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
	int x=0; int y=0;
	ESPmsgCursor=0;
	int total;
	int val[4];
	char buffer[4];
	for(int p=0;p<MAX_JSON;p++){
		char command=ESPmsgbuffer[p];
		switch(command){
			case 'R':
			{				//Serial.print("Relay:");
				char r = 0x00;
				for( x=0;x<8;x++){
					r*=2;
					if(ESPmsgbuffer[p+x+1]=='1'){
						r+=1;
						//Serial.print("1");
					}	
					else{
						//Serial.print("0");
					}
				}
				setShiftReg(r);
				p+=8;

				break;
			}
			case 'C':{
				if(p<(MAX_JSON-10)){
					// next 9 are our RGB values
					Serial.print("Colour:");
					total=0;
					for(x=0;x<3;x++){
						for(y=0;y<3;y++){
							buffer[y]=ESPmsgbuffer[p+y+(x*3)+1];
						}
						total=atoi(buffer);
						Serial.println(total,DEC);
						val[x]=total;
					}
					setColour(val[0],val[1],val[2]);	
					p+=9;
				}
				break;
			}
			case ']':   // end of JSON
		{		p+=MAX_JSON; 
			break;
		}
		case 'T': // Time
		{
			p+=24;
			break;
		}
		default :
				
		break;
		}
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
	Serial.println("\r\nState:"+(String)wifi_State+" To:"+(String) s);
	int x=250/s;
	wifi_State=s;
	for(int z=0;z<s;z++){
		digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
		delay(x);              // wait for a second
		digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
		delay(x); 
	}
	
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
	//digitalWrite(13,val);
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
	String data1 = "GET /api/io/?mto=";
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
				Serial.println("GET:no OK");
			}
		}
	} 
	else{
		Serial.println("GET:no > found");
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
	data1 += "&to=";
	data1 += serverIPv6;
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
	cmd+="\r\n";
	mySerial.print(cmd);
	Serial.print(data1);	// send CIPSEND command
	// delay(10);
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

int printPage(int milliseconds, int okmax){ // return total chars read
	double t = millis()+milliseconds;
	int okcount=0;
	ESPmsgCursor	=0;
	boolean line 	=false;
	int 	mycount		=0;
	while((millis()<t)&(okcount<okmax)){
		ESPokFlag=false;
		mycount+=ESPgetLine(1000);
		if(ESPokFlag) okcount++;
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
			packetToSendFlag=true; 
			q=q+1;
		}
		else{
			Serial.println("BMP180 Failed");
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
	
	if(printPage(n,3)>0){ // wait for 3 oks
		Serial.print("Received:");
			if(ESPmsgCursor>0){
				for(int z=0;z<ESPmsgCursor;z++){
					Serial.print(ESPmsgbuffer[z]);
				}
			Serial.println();
			doJson();// Interpret JSON
			}
		Serial.println();
	} 
}
int setupBMP(){
	if (pressure_sensor.begin())
    Serial.println("BMPinit OK");
  else
  {
    Serial.println("BMPinit fail");
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
	Serial.println("<Flush");
	delay(10);
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
//		if(ENABLE_WATCHDOG)wdt_reset(); // reset the watchdog
		if(mySerial.available()){
			r = mySerial.read();
			charCount++;
			if(r=='[') {   // start of json capture
				json=true;
				ESPmsgCursor=0;
			}
			if(r==']'){ json=false; // end json
				ESPmsgbuffer[ESPmsgCursor]=r;
				ESPmsgCursor++;
				if(ESPmsgCursor>126){
					ESPmsgCursor=0;
				}
			}
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


boolean ESPsetMode(){
return ESPcommand("AT+CIPMODE=0",5,1000);

}
boolean connectWifi(){
	
		String cmd2="AT+CWJAP=";  // 9 chars
		cmd2+="\"";				// 10
		cmd2+=mySSID;			// 13
		//Serial.println(cmd2);
		cmd2+="\",";
		cmd2+="\"";				//16
		cmd2+=myPassword;		//24	
		cmd2+="\"";				//25
	//	return ESPcommandString(cmd2,6,5000);
	
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
	return ESPcommand("AT+CIFSR",6,3000);
}
void hardResetESP(){
	if(reset_counter<50){	
                flashme(2);
		digitalWrite(ESP_RESET_PIN,LOW);
		delay(200);
		digitalWrite(ESP_RESET_PIN,HIGH);
		Serial.println("*ESP RST*");	
	} 
	else {
			Serial.print(">RST Count**");
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
        else if(DEBUG_BMP)Serial.println("BMPERR-pressure");
      }
      else if(DEBUG_BMP)Serial.println("BMPERR startPrs");
    }
    else if(DEBUG_BMP)Serial.println("BMPERR-temp");
  }
  else if(DEBUG_BMP)Serial.println("BMPERR startTemp");
  return(0);
}
boolean ESPcommand(char* command, int lines, int timeout){
	mySerial.println(command);
	boolean r=ESPgetLine(500); // dump the echo
	if(ESPwaitMsg(lines,timeout)){
		if(ESPokFlag||ESPnoChangeFlag||ESPlinkedFlag||ESPconnectFlag){
			return true;
		}
	}
	return false;
}
boolean ESPcommandString(String command, int lines, int timeout){
	mySerial.println(command);
	boolean r=ESPgetLine(500); // dump the echo
	if(ESPwaitMsg(lines,timeout)){
		if(ESPokFlag||ESPnoChangeFlag||ESPlinkedFlag||ESPconnectFlag){
			return true;
		}
	}
	return false;
}
void setupPins(){
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
	pinMode(13, OUTPUT); // onboard LED
  pinMode(12, INPUT);  // from PIR
  pinMode(ESP_RESET_PIN,OUTPUT); // ESP reset 
  digitalWrite(ESP_RESET_PIN,HIGH); // set it high

  }
void setupSerial(){		// Open serial communications and wait for port to open: 
   Serial.begin(19200);
   delay(200);
   Serial.println("I'm OK, are you?");
  // set the data rate for the SoftwareSerial port
   mySerial.begin(9600);
   delay(200);
	
   }

void setColour(int r,int g,int b){
	analogWrite(RledPin,r);
	analogWrite(GledPin,g);
	analogWrite(BledPin,b);
}
void setTimers(){
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
boolean eepromValid(){
	clearBuffer();
	eepromReadBlock(0);
	Serial.println("EEPROM ID:"+ (String) ESPmsgbuffer);
	if(ESPmsgbuffer[0]=='s'){
		return true;
	}
	return false;
}
boolean getConfig(){ // get the config from eeprom
	double t=millis()+2000;
	char i=0x00;
	boolean enter =false;
	Serial.println("HIT CR NOW for config");
			while(millis()<t){
				while(Serial.available()){
					i=Serial.read();
					if(i==char_CR){
						enter = true;
					}
				}
			}
	if(enter){
		if(manualConfig()){
			return true;
		}
	}
	
	if(eepromValid()){
		if(readConfig()){
			return true;
		}
	}
	else{
		Serial.println("Bad EEPROM data");
		if(manualConfig()){
			return true;
		};
	}
	
	return false;
}
boolean manualConfig(){
	boolean t;
	String inLine="";
	Serial.println("Manual Config");
	if(eepromValid()){ // read the previous config if there was one.
		t=readConfig();
	}	
	while (validateConfig()==false){
		
		Serial.println("SSID:"+ mySSID);
		inLine=getSerial();
		inLine.trim();
		if(inLine.length()>0){
			mySSID=inLine;
		}
		Serial.println("WiFi Key:"+ myPassword);
		inLine=getSerial();
		inLine.trim();
		if(inLine.length()>0){
			myPassword=inLine;
		}
		Serial.println("Server:"+ myServer);
		inLine=getSerial();
		inLine.trim();
		if(inLine.length()>0){
			myServer=inLine;
		}
		Serial.println("Port:"+ myPort);
		inLine=getSerial();
		inLine.trim();
		if(inLine.length()>0){
			myPort=inLine;
		}
		Serial.println("MyIPV6:"+ myIPv6);
		inLine=getSerial();
		inLine.trim();
		if(inLine.length()>0){
			myIPv6=inLine;
		}
		Serial.println("ServerIPV6:"+ serverIPv6);
		inLine=getSerial();
		inLine.trim();
		if(inLine.length()>0){
			serverIPv6 = inLine;
		}
	}
	Serial.println("Saving Config..");
	if(saveConfig()){
		if(readConfig() && validateConfig()) return true;
	}
	
	return false;
}
