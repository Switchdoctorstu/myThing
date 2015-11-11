# myThing
Internet of Things core for Arduino and ESP8266
Arduno Pro Mini

Uses BMP180 on i2C bus to get temp and pressure
a PIR detector gives presence information
8 relays on 74hc595 shift register give outputs
RGB LED gives mood

Uses ESP8266 to communicate with Web service to deliver stats and get instructions.



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


URL format:
	?mto= destination IP6 address
	&mfrom = my IP6 address
	&msg= message

	Message: (parser needs to strip numbers including decimals)
	Lux number as string
	Tmp number
	Prs number
	Pir number
	
