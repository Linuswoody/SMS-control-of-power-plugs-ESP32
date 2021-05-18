
#include <Arduino.h>

/*  Code to use an ESP32 in combination with a GSM800 module to control relays by SMS.
 *
 *  Many thanks to George Bantique, TechToTinker Youtube channel for the core code! 
 *          https://techtotinker.blogspot.com
 *  
 *  That was just the starting point and code that I needed for this project - and I can highly recommend his Youtube-channel!
 *  2021-05-12  H.-Christian Militzer
 *  
 *  
 *  Currently the "last 2 letters" + ":" of the command 3-letter word are checked only ("?:")
 *  "MD:" --> CMD, "UM:" --> NUM and "MT:" --> CMT (for caller number)    
 *      Here are the breakdown of available option:
 *        Send command in this format:
 *            CMD: RE1 1<
 *              - turn ON the red LED --> Relais 1 on
 *            CMD: RE1 0<
 *              - turn OFF the red LED --> Relais 1 off
 *            CMD: RE2 1<
 *              - turn ON the green LED --> Relais 2 on
 *            CMD: RE2 0<
 *              - turn OFF the green LED --> Relais 2 off
 *            CMD: RE3 1<
 *              - turn ON the blue LED --> Relais 3 on
 *            CMD: RE3 0<
 *              - turn OFF the blue LED --> Relais 3 off
 *            CMD: ALL 1<
 *              - turn ON all the LED --> Relais all on
 *            CMD: ALL 0<
 *              - turn OFF all the LED --> Relais all off
 *            CMD: REP 1<
 *              - enable the reply for every SMS
 *            CMD: REP 0<
 *              - disable the reply, but will still reply to the following:
 *                CMD: REP 1<
 *                CMD: REP 0<
 *                CMD: GET A<
 *                CMD: GET S<
 *            CMD: GET S<
 *              - sends the current states of the Relays --> Status aller Relais
 *            NUM: +ZZxxxxxxxxxx<
 *              - changes the current default number
 *                ZZ - country code
 *                xx - 10-digit number
 * 
 *      NEW: 
 *            CMD: GET A<   --> Status der Sensoren (Temp, Hum, Press, Volt1, Volt2) - done
 * 
 *            CMD: NTY 1/0<   --> Notify with SMS when temp1 < 3 *C; get temp1 every 10 minutes
 * 
 *            CMD: NYV 1/0<   --> Notify with SMS when volt12_avg < 12.8 V; get volt12_avg every 10 min
 *
 *            CMD: ATN 0<   --> Relais 1 automatic mode OFF
 * 
 *            CMD: ATN 1<   --> Relay 1 switches depending on T < 3 C (freeze protection)
 * 
 *            CMD: ATN 2<   --> Relay 1 switches depending on T < 20 C (pleasant T)
 * 
 *            CMD: ATN 3<   --> Relais 1 interval program 1 (20 min ON & 23h 40 min OFF)
 * 
 *            CMD: ATN 4<   --> Relais 1 interval program 2 (30 min ON & 11h 30 min OFF)
 * 
 *            CMD: ATN 5<   --> Relais 2 interval program 1
 * 
 * *          CMD: ATN 6<   --> Relais 2 interval program 2
 * 
 *            CMD: ATN 9<   --> Relais 2 interval/automatic mode OFF
 * 
 *      When event is triggered let say in this tutorial, we just use a push button tactile switch    
 *      but you can expand it, for example you can use an intruder detection that will send an SMS
 *      when an intruder is detected.
 *      
 *      Please feel free to modify it to adapt to your specific needs, just leave this comments as      
 *      credits to me. Thank you, Christian
 */
#include <SoftwareSerial.h> //für Arduino
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
//#include <WiFi.h>

//#define SERIAL_DEBUG       // comment out this to remove serial debug
#define SMS_ENABLED        // comment out this to disable sms sending, to save from expenses
//#define SIMULATE_BY_SERIAL    // comment out this to use the actual SIM800L

SoftwareSerial SIM800L(16, 17);  // Arduino D11(as Rx pin (statt Rx-onboard pin) to SIM800L Tx pin - ESP32 GPIO 16 as Rx pin)
                                 // Arduino D12(as Tx pin) to SIM800L Rx pin - ESP32 GPIO 17 as TX  with 2k/10k bridge okay)
#define RELAY1   18   // for ESP32; Pin assignments for LED/Relais and Switch (Arduino: 7)
#define RELAY2   15   // was originally "6"  - Relays need to be low to be triggered "on"
#define RELAY3   35   // was originally "5" (Arduino)
#define SW_PIN   34   // 4 with Arduino - switch for the alarm - PIN might have to be changed
//#define RTS_PIN   14   // not in use, might be for SIM800
//#define CTS_PIN   15   // not in use, might be for SIM800
#define V1_PIN    2    // new: Input-PIN used for 12V-voltage line measurement volt12 (GPIO19 does not work)
// Pins for I2C on ESP32 are GPIO22 = SCL and GPIO 21 = SDA

#define BUFFER_SIZE   127       // incoming data buffer size
char bufferData[BUFFER_SIZE];   // holds the char data
int bufferIndex = 0;            // holds the position of the char in the buffer - was "char", has to be "int"

// This variable holds the SMS sender mobile number, 
// it will be updated when new sms arrived
char sender_num[15] = {'+', '4', '9', '1', '5', '7', '7', '6', '3', '4', '8', '7', '4', '4'}; // Originally just [14] --> too short
// This variable holds the mobile number of the owner, emergency sms will be sent to this
// when the event is triggered lets say the button press
char default_num[15] = {'+', '4', '9', '1', '5', '7', '7', '6', '3', '4', '8', '7', '4', '4'}; // Originally just [14] --> too short
char sender_cmd[7];     // holds the specific command when a new SMS is received
char message;
char mobile;

bool CMT_OK = false;    // CMT is for processing the SMS senders mobile number that comes with incoming SMS "CMT = "xxx"""
int cmt_pos_index = 0;  
int cmt_pos_count = 0;

bool CMD_OK = false;    // CMD is for processing the parsing the sms command
int cmd_pos_index = 0;
int cmd_pos_count = 0;

bool NUM_OK = false;    // NUM is for processing the CHANGE NUMBER option (only after NUM:-SMS)
int num_pos_index = 0;
int num_pos_count = 0;

Adafruit_BME280 bme;    // use I2C interface - Sensor normalerweise auf 0x76
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();

float temp1;            // Variables used for BME280 data
float hum1;
float press1;
float volt12;
float volt12_avg;
int offset =337;        // 337 - set the correction offset value for 12V-voltage - averaging required
const int runs = 10;    // 10 - number of values volt12 that are used to average
float volt230;

// Variables for notify-Temp-loop
bool notify_T_on = false;        // false - Switch for notify loop (by SMS "CMD: NTY 1<")
int auto_R1_mode = 0;            // 0 - Mode-Value for T-automation (by SMS "CMD: ATN 1" or higher)
bool sentAlarmT = false;         // false - used to control of Temp-Alarm has already been sent
bool isR1_set = true;            // true - keep info on relay state between direct SMS-control and notify-loop
float tempLow;                   
float tempHigh;
unsigned long alarmMillis = 0;   // 0 - time stamp for last reading
unsigned long readInterval = 5 * 60 * 1000; // read interval in minutes for T and V loop (5*60*1000 msec = 5 min)

bool isR2_set = true;          // keep info on relay 2 state between direct SMS-control and notify-loop
int auto_R2_mode = 0;          // keep info on relay2 mode (0 = OFF)

// Variables for interval timer setting in minutes
unsigned long interval_on = 0;            // relay 1
unsigned long interval_off = 0;
unsigned long intervalMillis1 = 0;        // time stamp for last reading interval relay 1
unsigned long interval_on_2 = 0;          // relay 2
unsigned long interval_off_2 = 0;
unsigned long intervalMillis2 = 0;        // time stamp for last reading interval relay 2

// Variables for notify-Volt-loop
bool notify_V_on = false;           // Switch for notify loop (by SMS "CMD: NYV 1<")
bool sentAlarm_V = false;           // used to control of Volt-Alarm has already been sent
float low_limit_V = 12.8;          // low voltage value to raise alarm on 12V line
unsigned long alarmMillis_V = 0;    // 
unsigned long readInterval_V = 20 * 60 * 1000; // in Minuten (20*60*1000 msec = 20 min)
unsigned long alarmTimer_V = 0;         // time of first low voltage event

// used to set GPIO pin high/low by SMS-CMD at the end of "process Command"-loop:
bool relay1State = true;   //    holds the state of LEDs / Relais 1 - 3
bool relay2State = true;   //    true:  means OFF
bool relay3State = true;   //    false: means ON

bool newData = false;

bool isReplyOn = true;      //    hold the settings for sending the status update for every sms 
                            //    true:  reply to every sms
                            //    false: reply only to some sms request like request for status

// Variables for Contact-alarm-loop:
bool currState = LOW;            // this variables are for the button debouncing; both were HIGH originally
bool prevState = LOW;
unsigned long startMillis = 0;
unsigned long debounceDelay = 50;

/*
 *  Function: void initializeGSM()
 *  Purpose:
 *    This function is for the SIM800L initializations
 */
void initializeGSM() {
#ifdef SIMULATE_BY_SERIAL
  Serial.println("AT");                   // Sends an ATTENTION command, reply should be OK
  Serial.println("AT+CMGF=1");            // Configuration for sending SMS
  Serial.println("AT+CNMI=1,2,0,0,0");    // Configuration for receiving SMS
  Serial.println("AT&W");                 // Save the configuration settings
#else // actual
  delay(4000);                //2000
  SIM800L.println("AT");                  // Sends an ATTENTION command, reply should be OK
  delay(1000);                //1000
  SIM800L.println("AT+CMGF=1");           // Configuration for sending SMS
  delay(1000);                //1000
  SIM800L.println("AT+CNMI=1,2,0,0,0");   // Configuration for receiving SMS (war: 1,2,0,0,0)
  delay(1000);                //1000
  SIM800L.println("AT&W");                // added afterwards
  Serial.println("AT&W, SIM initialized");        // Save the configuration settings
  delay(1000);
#endif // SIMULATE_BY_SERIAL
}

// function to set relays depending on readings and auto-function
void controlRelays() {
  if (auto_R1_mode > 0) {  // Sets the relay 1 states according to automatic-mode trigger event
    digitalWrite(RELAY1, isR1_set);  //relay 1
    Serial.println("R1 - Auto mode");
  }
  if (auto_R1_mode == 0){             // Sets the Relay state according to direct SMS-CMD trigger
    digitalWrite(RELAY1, relay1State);
    Serial.println("R1-switched by SMS");
  }
  if (auto_R2_mode > 0) {  //relay 2 - auto trigger
    digitalWrite(RELAY2, isR2_set);
    Serial.println("R2 - Auto mode");
  }
  if (auto_R2_mode == 0) {            //relay 2 - direct SMS trigger
    digitalWrite(RELAY2, relay2State); 
    Serial.println("R2-switched by SMS");
  }
  
  digitalWrite(RELAY3, relay3State);
}

float get_volt12() {    // function - from example sketch without additional device
  volt12 = 0;      
  int sensorValue = analogRead(V1_PIN);   // read the input on analog pin 0 (original "A0")
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 25V):
  volt12 = map(sensorValue, 0, 2047, 0, 2500) + offset;// map 0-1023 to 0-2500 and add correction offset
  // print out the value you read:
  volt12 /=100;               // divide by 100 to get the decimal values
  return (volt12);
}

float get_volt12_avg() {    // function - from example sketch without any circuit board
  volt12_avg = 0;
  float alt = 0;
  int i;
  delay(10);
  alt = get_volt12();
  delay(10);
  for (i = 1; i < runs; i++)
  {
    volt12 = get_volt12();
    volt12_avg = (0.9 * alt) + (0.1 * volt12);
    alt = volt12_avg;
    delay(20);
  }
  return (volt12_avg);
}

/*
float get_volt230() {                 // function for measuring 230V - not yet installed or coded
  float volt230 = 0;      
  int sensorValue = analogRead(A1);   // read the input on analog pin 1 (?)
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  float volt230 = sensorValue * (5.0 / 1023.0);
  // print out the value you read:
  Serial.println(volt230);
  return (volt230);
}
*/

/*
 *  Function: void sendSMS(String message, String mobile)
 *  Purpose:
 *    This function is use for sending the reply SMS
 */
void sendSMS(String message, String mobile) {
#ifdef SIMULATE_BY_SERIAL
  Serial.println("AT+CMGS=\"" + mobile + "\"\r"); //Mobile phone number to send message
  Serial.println(message);                        // This is the message to be sent.
  Serial.println((char)26);                       // ASCII code of CTRL+Z to finalized the sending of sms
#else // actual
  SIM800L.println("AT+CMGF=1");                    //Sets the GSM Module in Text Mode - war ausgemarket
  delay(1000);
  String initCall = "AT+CMGS=\"";
  initCall += mobile;
  initCall += "\"\r";
  delay(10);
  Serial.println(initCall);
  Serial.println(message);
  delay(10);
  SIM800L.println(initCall);
  delay(1000);
  SIM800L.println(message);                        // This is the message to be sent. original nur message
  delay(1000);
  SIM800L.println((char)26);                       // ASCII code of CTRL+Z to finalized the sending of sms
  delay(1000);
  Serial.println("message sent ");                //added for serial monitoring
#endif // SIMULATE_BY_SERIAL
}


/*
 *  Function: void processCommand()
 *  Purpose:
 *    This function execute the receive commands via SMS
 */   
void processCommand() {                    
  if (strcmp(sender_cmd, "RE1 1") == 0) {         // StringCompare is "0" if both strings are equal
    Serial.println("Relais 1 = an");              // 
    relay1State = false;                          // relay 1 ON & automation OFF
    auto_R1_mode = 0;
    if (isReplyOn) {
      sendSMS("Relay 1 On", sender_num);
    }
  } 
  else if (strcmp(sender_cmd, "RE1 0") == 0) {  // Relay 1 OFF
    Serial.println("Relais 1 = aus");             
    relay1State = true;                           // relay 1 OFF & automation OFF
    auto_R1_mode = 0;
    interval_on = 0;
    if (isReplyOn) {
      sendSMS("Relay 1 Off", sender_num);
    }
  } 
  else if (strcmp(sender_cmd, "NTY 1") == 0) {  // freeze alarm ON - no automation
    notify_T_on = true;
    auto_R1_mode = 0;
    if (isReplyOn) {
      sendSMS("Freeze temperature alarm is ON - automation disabled", sender_num);
    }
  }  
  else if (strcmp(sender_cmd, "NTY 0") == 0) {  // freeze alarm OFF & automation OFF
    notify_T_on = false;
    auto_R1_mode = 0; 
    if (isReplyOn) {
      sendSMS("Freeze temperature alarm is OFF", sender_num);
    }
  } 
  else if (strcmp(sender_cmd, "NYV 1") == 0) {  // Alarm 12V-loop ON
    notify_V_on = true;
    if (isReplyOn) {
      sendSMS("12 V line: Alarm is ON", sender_num);
    }
  }  
  else if (strcmp(sender_cmd, "NYV 0") == 0) {  // Alarm 12V-loop OFF
    notify_V_on = false;
    if (isReplyOn) {
      sendSMS("12 V line: Alarm is OFF", sender_num);
    }
  }  
  else if (strcmp(sender_cmd, "ATN 0") == 0) {  // Automation temperature OFF
    auto_R1_mode = 0;                             
    notify_T_on = false;
    relay1State = true;
    interval_on = 0;
    interval_on_2 = 0;
    if (isReplyOn) {
      sendSMS("All relay-1 programs are OFF - Freeze temperature alarm is ON", sender_num);
    }
  }   
  else if (strcmp(sender_cmd, "ATN 1") == 0) { 
    auto_R1_mode = 1;                             // automation freeze control ON
    isR1_set = true;
    notify_T_on = true;
    if (isReplyOn) {
      sendSMS("Relay-1 program No 1 is ON: Anti-Freeze", sender_num);
    }
  }  
  else if (strcmp(sender_cmd, "ATN 2") == 0) {
    auto_R1_mode = 2;                             // automation room temperature control ON
    isR1_set = true;
    notify_T_on = false;
    if (isReplyOn) {
      sendSMS("Relay-1 program No 2 is ON: keep 20 C", sender_num);
    }
  }  
  else if (strcmp(sender_cmd, "ATN 3") == 0) {  // intervall heating 1 without T control
    auto_R1_mode = 3;
    notify_T_on = false;
    interval_on = 0;
    isR1_set = true;
    if (isReplyOn) {
      sendSMS("Relay-1 program No 3 is ON: 20 min every 24h", sender_num);
    }
  }  
  else if (strcmp(sender_cmd, "ATN 4") == 0) {  // intervall heating 2 without T control
    auto_R1_mode = 4;
    notify_T_on = false;
    interval_on = 0;
    isR1_set = true;
    if (isReplyOn) {
      sendSMS("Relay-1 program No 4 is ON: 30min every 12h", sender_num);
    }
  }   
  else if (strcmp(sender_cmd, "ATN 5") == 0) {  // intervall for relay 2, program 1 ON
    auto_R2_mode = 1;
    isR2_set = true;
    interval_on_2 = 0;
    if (isReplyOn) {
      sendSMS("Relay-2 automation is ON: 20 min every 24h", sender_num);
    }
  }  
  else if (strcmp(sender_cmd, "ATN 6") == 0) {  // intervall for relay 2, program 2 ON
    auto_R2_mode = 2;
    isR2_set = true;
    interval_on_2 = 0;
    if (isReplyOn) {
      sendSMS("Relay-2 automation is ON: 20 min every 24h", sender_num);
    }
  } 
  else if (strcmp(sender_cmd, "ATN 9") == 0) {  // automation for relay 2 OFF
    auto_R2_mode = 0;
    isR2_set = true;
    relay2State = true;
    interval_on_2 = 0;
    if (isReplyOn) {
      sendSMS("Relay-2 automation is OFF", sender_num);
    }
  }
  else if (strcmp(sender_cmd, "RE2 1") == 0) {  // relay 2 ON
    relay2State = false;
    if (isReplyOn) {
      sendSMS("Relay 2 On", sender_num);
    }
  }
  else if (strcmp(sender_cmd, "RE2 0") == 0) {  // relay 2 OFF
    relay2State = true;
    interval_on_2 = 0;
    if (isReplyOn) {
      sendSMS("Relay 2 Off", sender_num);
    }
  } 
  else if (strcmp(sender_cmd, "RE3 1") == 0) {  // SMS-switch relay 3
    relay3State = false;
    if (isReplyOn) {
      sendSMS("Relay 3 On", sender_num);
    }
  } 
  else if (strcmp(sender_cmd, "RE3 0") == 0) {
    relay3State = true;
    if (isReplyOn) {
      sendSMS("Relay 3 Off", sender_num);
    }
  } 
  else if (strcmp(sender_cmd, "ALL 0") == 0) {      // all relays OFF & automation OFF
    relay1State = true;
    relay2State = true;
    relay3State = true;
    auto_R1_mode = 0;
    auto_R2_mode = 0;
    interval_on = 0;
    interval_on_2 = 0;
    if (isReplyOn) {
      sendSMS("All relays Off", sender_num);
    }
  } 
  else if (strcmp(sender_cmd, "ALL 1") == 0) {      // all relays ON
    relay1State = false;
    relay2State = false;
    relay3State = false;
    auto_R1_mode = 0;
    auto_R2_mode = 0;
    if (isReplyOn) {
      sendSMS("All LED On", sender_num);
    }
  } 
  else if (strcmp(sender_cmd, "REP 1") == 0) {       // This command is to turn ON the reply to every sms
    isReplyOn = true;
    #ifdef SMS_ENABLED
      sendSMS("Reply is now ON", sender_num);
    #endif //SMS_ENABLED
  } 
  else if (strcmp(sender_cmd, "REP 0") == 0) {      // This command is to turn OFF the reply to every sms
    isReplyOn = false;
    #ifdef SMS_ENABLED
      sendSMS("Reply is now OFF", sender_num);
    #endif //SMS_ENABLED
  } 
  else if (strcmp(sender_cmd, "GET S") == 0) {    // This command sends the current status of all LEDs 
    String statusSMS = "";                               // Create the status reply
    statusSMS = "Relais-1 is ";
    if (relay1State) {
      statusSMS += "OFF";
    } else statusSMS += "ON";
    statusSMS += ", Relais-2 is ";
    if (relay2State) {
      statusSMS += "OFF";
    } else statusSMS += "ON";
    statusSMS += ", Relais-3 is ";
    if (relay3State) {
      statusSMS += "OFF.";
    } else statusSMS += "ON.";
    delay(10);
    #ifdef SMS_ENABLED
      sendSMS(statusSMS, sender_num);
    #endif //SMS_ENABLED
  } 
    else if ((strcmp(sender_cmd, "GET A") == 0)) {    // This command sends the current information of all Sensors
                                                      // call function to measure voltage 1  --> volt12_avg
    volt12_avg = get_volt12_avg();                    // mean value from 10 measurements; single values vary +/- 0,15 V
    //float volt230 = get_volt230_avg();              //call funktion to measure voltage 2  --> volt230_avg
    //call function  --> temp1, hum1, press1
    sensors_event_t temp_event, pressure_event, humidity_event;
    bme_temp->getEvent(&temp_event);
    bme_pressure->getEvent(&pressure_event);
    bme_humidity->getEvent(&humidity_event);
    #ifdef SERIAL_DEBUG
      Serial.print(F("Temperature = "));
      Serial.print(temp_event.temperature);
      Serial.println(" *C");
      Serial.print(F("Humidity = "));
      Serial.print(humidity_event.relative_humidity);
      Serial.println(" %");
      Serial.print(F("Pressure = "));
      Serial.print(pressure_event.pressure);
      Serial.println(" hPa");
    #endif //SERIAL_DEBUG
    temp1 = temp_event.temperature;             // get data from BME280
    hum1 = humidity_event.relative_humidity;
    press1 = pressure_event.pressure;
    String statusSMS = "";                      // Create the status reply
    statusSMS = "Temp = ";
    statusSMS += temp1;
    statusSMS += " C, Hum = ";
    statusSMS += hum1;
    statusSMS += " %rF, Press = ";
    statusSMS += press1;
    statusSMS += " hPa, 12V-line: ";
    statusSMS += volt12;
    statusSMS += " V, 230V-line: ";
    statusSMS += volt230;
    statusSMS += " V";
    #ifdef SMS_ENABLED
      sendSMS(statusSMS, sender_num);
    #endif //SMS_ENABLED
  }
  controlRelays();
}

/*
 *  Function: void manageButton()
 *  Purpose:
 *    This is for polling the state of the button.
 *    Additional button debouncing is use to prevent
 *    false button presses.
 */
void manageButton() {
  bool currRead = digitalRead(SW_PIN); 
  
  if (currState != prevState) {
    startMillis = millis();
  }
  if ((millis() - startMillis) > debounceDelay) {
    if (currRead != currState) {
      currState = currRead;
      if (currState == LOW) {
        #ifdef SERIAL_DEBUG
                Serial.println("Alert!");
        #endif //SERIAL_DEBUG
      //          sendSMS("Contact alert! Event triggered", default_num);
      Serial.println("would have sent SMS Contact Alarm!"); 
      }
    }
  }
  prevState = currState;
}

/*
*   Function for interval automatic relay 1
*/
void intervalControl1() {         // control interval for relay 1
  if (interval_on < 1) {          // initial run --> interval settings
    isR1_set = false;             // turn relay 1 ON
    intervalMillis1 = millis();   // start time for first interval
    if (auto_R1_mode == 3) {          // 20min ON - 23h 40min OFF
      interval_on = 20 * 60 * 1000;
      interval_off = (23 * 60 * 60 * 1000) + (40 * 60 * 1000);
    }
    else if (auto_R1_mode == 4) {      // 30min ON - 11h 30min OFF
      interval_on = 30 * 60 * 1000;
      interval_off = (11 * 60 * 60 * 1000) + (30 * 60 * 1000);
    }
  }  
  if (!isR1_set) {         // control of ON period if false
    if ((millis() - intervalMillis1) > interval_on) {
      isR1_set = true;            // turn OFF if period is over
      intervalMillis1 = millis(); // new time stamp for OFF period
      delay(10);
    }
  }
  else if (isR1_set) {        // control of OFF period if true
    if ((millis() - intervalMillis1) > interval_off) {
      isR1_set = false;            // turn ON if period is over
      intervalMillis1 = millis(); // new time stamp for ON period
      delay(10);
    }
  }  
  controlRelays(); 
}

/*
*   Function for interval automatic relay 2
*/
void intervalControl2() {// control interval for relay 2
  if (interval_on_2 < 1) {          // initial run --> interval settings
    isR2_set = false;             // turn relay 1 ON
    intervalMillis2 = millis();   // start time for first interval
    if (auto_R2_mode == 1) {          // 20min ON - 23h 40min OFF
      interval_on_2 = 5000; //20 * 60 * 1000;
      interval_off_2 = 10000; //(23 * 60 * 60 * 1000) + (40 * 60 * 1000);
    }
    else if (auto_R2_mode == 2) {      // 30min ON - 11h 30min OFF
      interval_on_2 = 30 * 60 * 1000;
      interval_off_2 = (11 * 60 * 60 * 1000) + (30 * 60 * 1000);
    }
  }  
  if (!isR2_set) {         // control of ON period if false
    if ((millis() - intervalMillis2) > interval_on_2) {
      isR2_set = true;            // turn OFF if period is over
      intervalMillis2 = millis(); // new time stamp for OFF period
      delay(10);
    }
  }
  else if (isR2_set) {        // control of OFF period if true
    if ((millis() - intervalMillis2) > interval_off_2) {
      isR2_set = false;            // turn ON if period is over
      intervalMillis2 = millis(); // new time stamp for ON period
      delay(10);
    }
  }  
  controlRelays(); 
}

/*
 *  Function: void notifyTemp()
 *  Purpose:
 *    This is for measuring temp1 every 10 min and check for T < 3 *C --> SMS-alarm notification
 */
void notifyTemp() {           // temp-control used to control heater at relay 1 (RED)
  if ((millis() - alarmMillis) > readInterval) {
    alarmMillis = millis();     // update time for next check
    sensors_event_t temp_event; // get temp1
    bme_temp->getEvent(&temp_event);
    #ifdef SERIAL_DEBUG
      Serial.print(F("Temperature = "));
      Serial.print(temp_event.temperature);
      Serial.println(" *C");
    #endif //SERIAL_DEBUG
    temp1 = temp_event.temperature;      // get data from BME280 
    if (auto_R1_mode == 1) {
      tempLow = 3;
      tempHigh = 5;
    }
    else if (auto_R1_mode == 2) {
      tempLow = 19;
      tempHigh = 22;
    }
    if (temp1 < tempLow) {              // case T < 3 *C or 19 *C
      if (auto_R1_mode > 0) {           // if auto-heating is on --> turn on heater/relay 1
        isR1_set = false;               // turn heater (relay 1) ON
      }             
      if ((notify_T_on) & (auto_R1_mode < 2)) {         // send SMS only in automatic mode 0 & 1
        if (!sentAlarmT) {       // send SMS at < 3 *C if false
          sentAlarmT = true;
          String statusSMS = ""; 
          statusSMS = "Low Temp-Alarm. Temp1 = ";
          statusSMS += temp1;
          statusSMS += " *C.";
          if (auto_R1_mode == 1) {
            statusSMS += " Heater on relay 1 turned on.";     // message for antifreeze automatic mode
          }
          else {
            statusSMS += " No action has been taken yet. Automatic OFF";  // message for antifreeze notification mode
          }
          delay(10);
          sendSMS(statusSMS, default_num);
          #ifdef SERIAL_DEBUG
                Serial.println("Low-Temp Alert set! Relay1 high");
          #endif //SERIAL_DEBUG
        }
      }  
    }   // no action - SMS has been sent and in case of auto-mode relay 1 should be on

    else {                             // case T > 5 *C or 22 *C
      if ((auto_R1_mode > 0) & (temp1 >= tempHigh)) {    // if auto-heating is on --> turn off heater/relay 1
        isR1_set = true;                          // relay 1 OFF
      } 
      if ((notify_T_on) & (auto_R1_mode < 2)) {        // send SMS only in automatic mode 0 & 1
        if (sentAlarmT) {                              // send SMS at > 5 / 22 *C 
          sentAlarmT = false;
          String statusSMS = ""; 
          statusSMS = "Temp-Alarm canceled. Temp1 = ";
          statusSMS += temp1;
          statusSMS += " *C.";
          if (auto_R1_mode == 1) {
            statusSMS += " Heater on relay 1 turned OFF.";    // message for antifreeze automatic mode
          }
          else {
            statusSMS += " Automatic OFF";                    // message for antifreeze notification mode
          }
          delay(10);
          sendSMS(statusSMS, default_num);
          #ifdef SERIAL_DEBUG
                  Serial.println("Low-Temp Alert cancelled! Relay 1 low");
          #endif //SERIAL_DEBUG
        }
      }
    }
    controlRelays();
  }
}

/*
 *  Function: void notifyVolt()
 *  Purpose:
 *    This is for measuring volt12_avg every 10 min and check for U < 12.7 *C (external supply)
 *    --> SMS-alarm notification
 */
void notifyVolt() {
  if ((millis() - alarmMillis_V) > readInterval_V) {  // measure if readInterall has passed
    alarmMillis_V = millis();     // update time for next check
    get_volt12_avg();             // get 12V-line voltage
    #ifdef SERIAL_DEBUG
      Serial.print(F("12-Volt line = "));
      Serial.print(volt12_avg);
      Serial.println(" V");
    #endif //SERIAL_DEBUG
    if (volt12_avg < low_limit_V) {
      if (sentAlarm_V != true) {           // send primary alarm SMS only after first occurrance = sentAlarm_V false 
        sentAlarm_V = true;
        alarmTimer_V = millis();           // set timer to first low voltage event  
        String statusSMS = ""; 
        statusSMS = "Low Voltage-Alarm! 12V-line = ";
        statusSMS += volt12_avg;
        statusSMS += " V. Battery might discharge...";
        delay(10);
        sendSMS(statusSMS, default_num);
        #ifdef SERIAL_DEBUG
                Serial.println("Low Voltage-Alarm!");
        #endif //SERIAL_DEBUG
      }
      else  {                             // send following reminder SMS only every 12 hours   
        if ((millis() - alarmTimer_V) > (3 * 12 * readInterval_V)) {
          String statusSMS = ""; 
          statusSMS = "Reminder: Low Voltage-Alarm! 12V-line = ";
          statusSMS += volt12_avg;
          statusSMS += " V.";
          delay(10);
          sendSMS(statusSMS, default_num);
          #ifdef SERIAL_DEBUG
                Serial.println("Low-Voltage Alert still on!");
          #endif //SERIAL_DEBUG
        }  
      }
      // no action - SMS has been sent
    }
    else {                                // voltage back above lower limit
      if (sentAlarm_V) {
        sentAlarm_V = false;
        String statusSMS = ""; 
        statusSMS = "Low Voltage-Alarm cancelled! 12V-line = ";
        statusSMS += volt12_avg;
        statusSMS += " V.";
        delay(10);
        sendSMS(statusSMS, default_num);
        #ifdef SERIAL_DEBUG
                Serial.println("Low Voltage-Alarm cancelled");
        #endif //SERIAL_DEBUG
      }
    }  
  //controlRelays();    // not necessary yet
  }
}

/*
 *  Function: void manageSIM800()
 *  Purpose:
 *    This is for parsing the incoming serial data from SIM800L
 *    so that we can use it for controlling something.
 */
void manageSIM800() {
 
#ifdef SIMULATE_BY_SERIAL
  while (Serial.available() > 0) {
    bufferData[bufferIndex] = Serial.read();
    // Find the string "CMD:"
    //    if found, parse the command for processing
    if( //(bufferData[bufferIndex-3] == 'C') && 
        //(bufferData[bufferIndex-2] == 'M') && 
        (bufferData[bufferIndex-1] == 'D') && 
        (bufferData[bufferIndex] == ':')      )  {               
      //Serial.println("CMD"); 
      CMD_OK = true;      
      memset(sender_cmd, 0, sizeof(sender_cmd));
      cmd_pos_index = bufferIndex;  // get the position
      cmd_pos_count = 0;            // reset pos counter   
    }    
    // Finds the string "NUM:"
    //    if found, parse the command for processing
    if( //(bufferData[bufferIndex-3] == 'N') &&
        //(bufferData[bufferIndex-2] == 'U') && 
        (bufferData[bufferIndex-1] == 'M') && 
        (bufferData[bufferIndex] == ':')      )  {              
      //Serial.println("NUM"); 
      NUM_OK = true;
      memset(default_num, 0, sizeof(default_num));
      num_pos_index = bufferIndex;  // get the position
      num_pos_count = 0;            // reset pos counter 
      
    } 
    // Finds the string "CMT:"
    //    if found, parse the command for processing
    if( //(bufferData[bufferIndex-3] == 'C') && 
        //(bufferData[bufferIndex-2] == 'M') && 
        (bufferData[bufferIndex-1] == 'T') && 
        (bufferData[bufferIndex] == ':')      )  {            
      //Serial.println("CMT");  
      CMT_OK = true;
      memset(sender_num, 0, sizeof(sender_num));    
      cmt_pos_index = bufferIndex;  // get the position
      cmt_pos_count = 0;            // reset pos counter 
    }    
    // String "CMT:" is found, 
    //  parse the sender number for the reply
    //
    if ( ( (bufferIndex - cmt_pos_index) > 2 ) && (CMT_OK) ) {
      if ( cmt_pos_count < 13 ) {
        sender_num[cmt_pos_count] =  bufferData[bufferIndex];
        cmt_pos_count++;
      } else {
        //Serial.println(sender_num);
        CMT_OK = false; // done
      }
    }
    
    // String "NUM:" is found
    //  parse it then save it as default number then send a reply
    //
    if ( ( (bufferIndex - num_pos_index) > 1 ) && (NUM_OK) ) {
      if ( bufferData[bufferIndex] != '<' ) {
        default_num[num_pos_count] =  bufferData[bufferIndex];
        num_pos_count++;
      } else {
        char msg[32] = "Default # is now ";
        sendSMS((strcat( msg, default_num )), sender_num);                          // send it to the recipient number.
        NUM_OK = false; // done
        break;
      }
    } 
    // String "CMD:" is found,
    //  parse the command, then execute the command
    //
    if ( ( (bufferIndex - cmd_pos_index) > 1 ) && (CMD_OK)) {
      if (bufferData[bufferIndex] == '<') {
        //Serial.println(sender_cmd);
        processCommand();
        CMT_OK = false; // done
        break;
      } else {
        sender_cmd[cmd_pos_count] =  bufferData[bufferIndex];
        cmd_pos_count++;
      }
    } 
    bufferIndex++;                              
  }           

  memset(bufferData, 0, sizeof(bufferData));    // Initialize the string     
  bufferIndex = 0;
#else // actual SIM800L GSM Module
  while (SIM800L.available() > 0) {
    bufferData[bufferIndex] = SIM800L.read();
    Serial.print("Start manageSIM800 - bufferData = ");
    Serial.println(bufferData);
    delay(10);
    // Find the string "CMD:"
    //    if found, parse the command for processing
    if( //(bufferData[bufferIndex-3] == 'C') && 
        (bufferData[bufferIndex-2] == 'M') && 
        (bufferData[bufferIndex-1] == 'D') && 
        (bufferData[bufferIndex] == ':')      )  {               
      Serial.println("CMD"); 
      CMD_OK = true;                              //triggers reading command-part in loop 6
      memset(sender_cmd, 0, sizeof(sender_cmd));  //sets content back to 7x "0" 
      cmd_pos_index = bufferIndex;                // get the position
      cmd_pos_count = 0;                          // reset pos counter   
    }    
    // Finds the string "NUM:"
    // if found, parse the command for processing
    if( //(bufferData[bufferIndex-3] == 'N') &&
        (bufferData[bufferIndex-2] == 'U') && 
        (bufferData[bufferIndex-1] == 'M') && 
        (bufferData[bufferIndex] == ':')      )  {              
      Serial.println("NUM"); 
      NUM_OK = true;
      memset(default_num, 0, sizeof(default_num)); //sets default-No back to 15x "0", to store a new one
      num_pos_index = bufferIndex;  // get the position
      num_pos_count = 0;            // reset pos counter 
    } 
    // Finds the string "CMT:"      // Checks position of sender number and initiates reading if CMT_OK = true
    //    if found, parse the command for processing
    if( //(bufferData[bufferIndex-3] == 'C') && 
        (bufferData[bufferIndex-2] == 'M') && 
        (bufferData[bufferIndex-1] == 'T') && 
        (bufferData[bufferIndex] == ':')      )  {            
      Serial.println("CMT"); 
      CMT_OK = true;
      memset(sender_num, 0, sizeof(sender_num));    
      cmt_pos_index = bufferIndex;  // get the position
      cmt_pos_count = 0;            // reset pos counter 
    }    
    // String "CMT:" is found, 
    //  parse the sender number for the reply
    // +CMT: "+639175291539"
    if ( ( (bufferIndex - cmt_pos_index) > 2 ) && (CMT_OK) ) {
      if ( cmt_pos_count < 14 ) {       //was originally "<13" and too short, now "<14"
        sender_num[cmt_pos_count] =  bufferData[bufferIndex];
        cmt_pos_count++;
      } else {
        //Serial.println(sender_num);
        CMT_OK = false; // done
      }
    }
    // String "NUM:" is found
    //  parse it then save it as default number then send a reply
    //
    if ( ( (bufferIndex - num_pos_index) > 1 ) && (NUM_OK) ) {
      if ( bufferData[bufferIndex] != '<' ) {
        default_num[num_pos_count] =  bufferData[bufferIndex];
        num_pos_count++;
      } else {
        char msg[32] = "Default # is now ";
        sendSMS((strcat( msg, default_num )), sender_num);  // send it to the recipient number.
        NUM_OK = false; // done
        break;
      }
    } 
    // String "CMD:" is found,
    //  parse the command, then execute the command
    //  CMD: RE1 0<
    if ( ( (bufferIndex - cmd_pos_index) > 1 ) && (CMD_OK)) {
      //delay(10);
      Serial.println("Buffer :");
      Serial.println(bufferData[bufferIndex]);
      if (bufferData[bufferIndex] == '<') {
        processCommand();
        CMT_OK = false; // done
        CMD_OK = false; // newly added
        //delay(10);
        break;
      } else {
        sender_cmd[cmd_pos_count] =  bufferData[bufferIndex];
        cmd_pos_count++;
      }
    } 
    bufferIndex++;                              
  }        //end of while(SIM.available)   
  memset(bufferData, 0, sizeof(bufferData));    // Initialize the string     
  bufferIndex = 0;
#endif //SIMULATE_BY_SERIAL
}

/*
 *  Initializations
 */
void setup(){
  Serial.begin(9600);
  Serial.println("Initializing...");
  SIM800L.begin(9600);
  delay(5000);                        // gives enough time for the GSM module to bootup - was 2000
  initializeGSM();                    // set the necessary initializations for the GSM module
  pinMode(RELAY1, OUTPUT);            // set the pin directions of other pins - change to control relays
  pinMode(RELAY2, OUTPUT);
  pinMode(RELAY3, OUTPUT);
  pinMode(SW_PIN, INPUT_PULLUP);
  digitalWrite(RELAY1, relay1State);  // set the current states of LEDs
  digitalWrite(RELAY2, relay2State);
  digitalWrite(RELAY3, relay3State);
  memset(bufferData, 0, sizeof(bufferData)); // Initialize the string     
  bufferIndex = 0; 
  Serial.println("Setup done");
  delay(10);
  pinMode(V1_PIN, INPUT);
  adcAttachPin(V1_PIN);
  analogReadResolution(10);

  if (!bme.begin(0x76)) { // initializing the BME280 with I2C-Address 0x76
    Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
    while (1) delay(10);
  }

  alarmMillis = millis();   // Startwert
}

/*
 *    This is the main loop
 */
void loop(){
  manageButton();
  manageSIM800();
  if (notify_V_on) {
    notifyVolt();
  }
  if (notify_T_on) {
    notifyTemp();
  }
  if (auto_R1_mode > 2) {
    intervalControl1();
  }
  if (auto_R2_mode > 0) {
    intervalControl2();
  }
  delay(100);
}

