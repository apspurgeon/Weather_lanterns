// ****************************************************************************************************************************************************************
// ****************************************************************************************************************************************************************
// This is designed to run on an ESP8266 using FastLED, with optional use of Blynk for a phone App
// It connects to an NTP time server, and sunrise/set server to determine time of day (UTC) and when the sun
// will rise and set based on Longitude and Latitude co-ordinates.  
//
// There are 3 modes of operation as set by the nightmode var.  
// 0 = day time LEDs are yellow, night LEDs are blue and as sun rises/sets LEDs reflect the colours of that change
// 1 = night light mode.  day time LEDs are off, night LEDs are yellow and as sun rises/sets LEDs reflect the colours of that change
// 2 = night light mode.  day time LEDs are off, night LEDs are yellow  with no sun rise/set LEDs changes (e.g a hard on/off)
//
// 1st digit
// Mode 0:  During the day, the LEDs are yellow (sunshine), and night LEDs are blue (Night sky) and as sun rises/sets LEDs change colour with the sunrise/set.
// Mode 1:  Night light mode(1).  During the day time LEDs are off, at night the LEDs are yellow and as sun rises/sets LEDs reflect the colours of that change.
// Mode 2:  Night light mode(2).  During the day time LEDs are off, at night the LEDs are yellow  with no sun rise/set LEDs changes (The Lamp will just turn on and off at dawn and dusk).
//
// 2nd digit
// 0:  Last LED behaves the same as all the other LEDs.
// 1:  Last LED is Red,   2:  Last LED is Green,   3:  Last LED is Blue,  4:  Last LED is White
//
// 3rd digit
// 0:  No flashing of LEDs
// 1:  If within 5 mins of the top of the hour, the LEDs will flash
// 
// 4th digit
// 1-5:  LED Brightness from 50-255
//
// Example:  0315 = Is a sunrise/sunset (on during day/night) with the top light blue with LEDs flashing and full brightness
//
//
// Variables of interest are found between the 'Things to change' comment lines
// Including the time delay between updating LEDs, time between NTP time checks and time between sunrise/set API requests.
// Note: After NTP time is received, internal Millis clock tracks time fairly acuratley, sunrise/set times only change once a day.
// Note: NTP is UDP and can fail, there is a check that the new NTP time isn't too different from expected time.  If it is, it keeps using 
// current time, unless it fails 3 times.  Then it will use NTP time (assumed after 3 times the NTP is correct afterall compared to Millis)
// 
// Other variables are self evident (hopefully).  Wifi credentials are stored in Platformoi.ini file ann injected during build.
// If not using PlatformOI, you can enter directly into the code in the 1st lines of 'Things to change'
//
// lastly, there are also testUTC (normally 0), and allows to test what happends (LED colour/state) at a specificed time (entered as minutes from midnight).
//
// ****************************************************************************************************************************************************************
// ****************************************************************************************************************************************************************

#define xstr(s) str(s)
#define str(s) #s

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ESP8266HTTPClient.h>
#include <FastLED.h>
#include <ArduinoJson.h> 
#include <WiFiManager.h>
#include <DNSServer.h>
#include <FS.h>
//#include <BlynkSimpleEsp8266.h>
//#define BLYNK_PRINT Serial

//if using build flags for WiFi credentials
//platformio.ini
//build_flags =
//    -DSSID_NAME="SSID"
//    -DPASSWORD_NAME="password"
//    -DBLYNKCERT_NAME="1234567890"



//*************************
//*** Things to change  ***
//*************************

/*   Only use if hard coding SSID/Password or build flags (platformio).  Checks if build flags work, if not then allows for hard coding credentials
//  If using WifiManager to collect credentials then comment this section out.
#ifndef BLYNKCERT_NAME
#define BLYNKCERT_NAME "1234567890" //Default BLYNK Cert if not build flag from PlatformIO doesn't work
#endif

#ifndef SSID_NAME
#define SSID_NAME "WIFI_SSID" //Default SSID if not build flag from PlatformIO doesn't work
#endif

#ifndef PASSWORD_NAME
#define PASSWORD_NAME "WIFI_PASSWORD" //Default WiFi Password if not build flag from PlatformIO doesn't work
#endif
*/

//LED details
#define NUM_LEDS_PER_STRIP 13      //Number of LEDs per strip
#define PIN_LED D7                //I.O pin on ESP2866 device going to LEDs
#define COLOR_ORDER GRB           // LED stips aren't all in the same RGB order.  If colours are wrong change this  e.g  RBG > GRB.   :RBG=TARDIS

//Flash at top of hour
int flash_mins = 1;                 //minutes to flash after the hourh  e.g 1 = 1min:  5pm - 5.01pm
int flash_delay = 60;               //delay between LED flashes (ms)

//Max values for LEDs
const int green_max = 128;          //green (128 for yellow / 255 for orange)
const int blue_max = 255;           
const int red_max = 255;            

//What lamp looks like for day time
const int green_day = 128;          //green (128 for yellow / 255 for orange)
const int blue_day = 0;
const int red_day = 255;

//What lamp looks like for night
const int green_night = 0;
const int blue_night = 255;
const int red_night = 0;


//Lamp brightness
int howbright = 255;         //0-255 LED Brightness level

//SPIFFS Filenames
String HTTPfilename = "APIaddress.txt";             //Filename for storing Sunrise API HTTP address in SPIFFS
String Modefilename = "Mode.txt";                   //Filename for storing Sunrise Mode in SPIFFS (3 digits:  Mode, Top lamp, Flash)
String UTCfilename = "UTC.txt";                     //Filename for storing Sunrise UTC in SPIFFS

//Lightmode, TARDIS, Longitude/Latitude and UTC are stated here but overwritten when webpage credentials are entered (if using WiFi Manager)
const char *NTPServerName = "0.nz.pool.ntp.org";    //local NTP server
int localUTC = 12;                                  //Country UTC offset, needed for UTC for day/night calc  (+12 for NZ)  don't need to change for daylight saving as no needed for day/night
int lightmode = 0;                                  //0 = day/night (day = Yellow / night = Blue   e.g TARDIS Lamp)    1 = night light mode with sunrise/set colour changes (but off during daytime)    2 = night light mode without sunrise/set changes  (binary on (day) /off (night))
int TARDIS = 1;                                     //Used for my TARDIS lamp.  All LEDs work as per day/night lightmode, except 1 LED (last in strip) at the top of the TADIS which is forced Blue.

int NTPSecondstowait = 600;  //Wait between NTP pulls (sec)
int APISecondstowait = 3600; //Wait between Sunrise API pulls (sec)

const int LEDSecondstowait = 5;    //Wait between LED updates (sec)
const int minswithin = 60;         //Minutes within sunrise / sunset to begin the LED colour change sequence  (60 = phase starts 30mins before sunrise/set and end 30mins after)
const int change = 1;              //Speed of LED change in tones.  Recommend = 1


//*** TESTING use only.  See testing section at end of setup
const int TESTING = 0;             //*TESTING* Normal condition =0.    Force a UTC time (entered as minutes from midnight) for testing purposes  (making sure LEDs do as expected)
const int printthings = 1;         //*TESTING* Flag to enable/disable printing on informations


//*************************
//*** Things to change  *** 
//*************************




//Gets SSID/PASSWORD from platformio.ini build flags.  
//Comment one or the other out in the following  (e.g Build flags vs Wifi Manager)
//const char ssid[] = xstr(SSID_NAME);          //gets ssid from build flags
const char ssid[] = "";                         //define ssid or the ConnectAP function errors (even though not used when using WifiManager)
//const char pass[] = xstr(PASSWORD_NAME);      //gets pass from build flags
const char pass[] = "";                         //define pass or the ConnectAP function errors (even though not used when using WifiManager)
//const char auth[] = xstr(BLYNKCERT_NAME);       // your BLYNK Cert from build flags


//Wifi and internet variables
const unsigned int localPort = 2390; // local port to listen for UDP packets
WiFiUDP udp;                         // A UDP instance to let us send and receive packets over UDP
char sunrise_api_request[100];       //It should end up containing an adress like this "http://api.sunrise-sunset.org/json?lat=-41.2865&lng=174.7762";
char UTC[3];
IPAddress timeServer;

//NTP and Time variables
int RequestedTime = 0, TimeCheckLoop = 0;
int hour_actual = 200, dia_actual = 0, anyo = 0;
int timeout = 0, timeout_prev = 0;
int vera = 0, night = 0;                                                                        //1 = Night, 0 = Day
const int NTP_PACKET_SIZE = 48;                                                                 // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[NTP_PACKET_SIZE];                                                             //buffer to hold incoming and outgoing packets
unsigned long epoch = 0, lastepoch = 0, LastNTP = 0, LastAPI, LastLED, epochstart, startmillis; //Unix time in seconds
int lastepochcount = 0, totalfailepoch = 0;
int hour, minute, second;               //UTC time
int clock_minutes_from_midnight, local_clock_minutes_from_midnight; //Minutes from midnight
int NTPSeconds_to_wait = 1;         //Initial wait time between NTP/Sunrise pulls (1 sec)
String clock_AMPM;                  //AM/PM from NTP Server
int printNTP=0;                     //Set to 1 when a NTP is pulled.  The decodeepoch function used for both NTP epoch and millis epoch.  printNTP=1 in this fucnction only print new NTP results (time).
int SecondsSinceLastNTP;            //Counts seconds since last NTP pull
unsigned long epoch2;               //Used to calculate Millis drift/difference with NTP
int timefactor = 1;                     //accellerate time by this factor (for testing)

//LED Flash variables
int flash_phase = 0;               //If x minutes within top of hour flash the LEDs
int flash = 0;                     //flash = 0 (no flash)  flash = 1 (flash) set by user
int LastFlashmillis;               //Used for tracking delay
int flash_working = 0;             //working var for sinearray (ranges 0-31)

//Create an array with 0-255 sine wave with array 0-31
char sinetable[]= {127,152,176,198,217,233,245,252,254,252,245,233,217,198,176,152,128,103,79,57,38,22,38,57,79,103};

//Sunrise - Sunset API variables
int h_sunrise, hour_sunrise, minute_sunrise, sunrise_minutes_from_midnight, local_sunrise_minutes_from_midnight;
int SR_Phase = 0;                             //1 = in Sunrise phase (30 mins either side if minwithin = 60mins)
int h_sunset, hour_sunset, minute_sunset, sunset_minutes_from_midnight, local_sunset_minutes_from_midnight;
int SS_Phase = 0;                             //1 = in Sunset phase (30 mins either side if minwithin = 60mins)
int working_hourtomin = 0;                            //Used to convert hours into total minutes
float LED_phase;                              //0-255 in the phase of sunrise/set   0=begining 255=end
char SR_AMPM[1], SS_AMPM[1];                  //Sunrise/set AMPM
String AMPM, sunAPIresponse;
struct CRGB leds[NUM_LEDS_PER_STRIP];         //initiate FastLED with number of LEDs
String JSON_Extract(String);
char mode [4];                                //Used to get input from webpage
int SRSS_Flip = 0;                                 //Used to manipulate SR and SS varible if in nightlight mode

//LED Variables. Hold the value (0-255) of each primary colour
int green = 0; 
int blue = 0;
int red = 0;

//Functions declared
void nightlight ();                             //function if in nightlight mode
void daynight ();                               //function if in day night mode
void API_Request ();                            //Gets sunrise/set times from API
void DoTheLEDs ();                              //Update LED colours based on time (minutes from midnight UTC)
void checkflash ();                             //Check if flash is needed
void Request_Time ();                           //Get time from NTP time server
bool Check_Time ();                             //Check time is correct and ok  
void DecodeEpoch (unsigned long);               //Turn Unix epoch time into hours:minutes
void sendNTPpacket (const IPAddress &address);  //Get data
void ConnectToAP ();                            //connect to WiFi Access point
void WiFi_and_Credentials();                    //Get WiFi credentials if using WiFi manager option (also connects to access point)
void checkreset();                              //Check if reset button has been pressed
void sunrise_sunset();                          //Calculate sunrise/sunset LED colours

void setup()
{
  Serial.begin(9600);

  pinMode(0, INPUT);          //GPIO0 (D3) to GND to reset ESP2866
 
  FastLED.addLeds<WS2811, PIN_LED, COLOR_ORDER>(leds, NUM_LEDS_PER_STRIP);      //Initialise the LEDs

  WiFi_and_Credentials();     //Calls WiFi function to initiate.  either uses WifiManager to get Wifi and Longitude/Latitude data (And store API URL as SPIFFS file.)  r Standard WiFi connection with build flags.

  //Blynk setup (if being used).
  //Blynk.begin(auth, ssid, pass);

  //Test the LEDs
  fill_solid(leds, NUM_LEDS_PER_STRIP, CRGB(255, 0, 0));
    FastLED.setBrightness(howbright);
  FastLED.show();
  Serial.println("TEST:  Red");
  delay(1000);

  fill_solid(leds, NUM_LEDS_PER_STRIP, CRGB(0, 255, 0));
    FastLED.setBrightness(howbright);
  FastLED.show();
  Serial.println("TEST:  Green");
  delay(1000);
  
  fill_solid(leds, NUM_LEDS_PER_STRIP, CRGB(0, 0, 255));
    FastLED.setBrightness(howbright);
  FastLED.show();  
  Serial.println("TEST:  Blue");
  delay(1000);  

  fill_solid(leds, NUM_LEDS_PER_STRIP, CRGB(0, 0, 0));
    FastLED.setBrightness(howbright);
  FastLED.show();

  //Initiate time
  udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(udp.localPort());

  Request_Time(); //Get the time
  delay(2000);
  while (!Check_Time())
  { //If no time recieved then do this
    delay(2000);
    TimeCheckLoop++;

    if (TimeCheckLoop > 5)
    {                 //If not time received even after 5x 2sec delays, then try re-getting time
      Request_Time(); //Get the time
    }
  }


//Check if time factor testing >1.  if yes, then overide the NTP/API delays to 48hrs to have clock run from jusdt millis
if (TESTING != 0){

  howbright = 255;            //0-255 LED Brightness level
  flash = 0;                  //Turn off flash if in testing mode
  NTPSecondstowait = 600;     //Wait between NTP pulls (sec)
  APISecondstowait = 600;     //Wait between Sunrise API pulls (sec) 
  timefactor = 1;             //accellerate time by this factor
  lightmode = 1;              //overide lightmode
  TARDIS = 3;                 //overide top light
  localUTC = 12;              //overide UTC
  sprintf(sunrise_api_request, "http://api.sunrise-sunset.org/json?lat=-41.2865&lng=174.7762");

 Serial.print("TEST sunrise_api_request = ");
 Serial.println(sunrise_api_request);
 }

  epochstart = epoch;     //epoch pulled from NTP server, use initial epoch to set starting point for epochmillis
  startmillis = millis(); //get starting point for millis
  API_Request();          //Get sunrise/sunset times

    Serial.print("millis = ");
    Serial.print(millis());
    Serial.print(",   start millis = ");
    Serial.println(startmillis);
    Serial.print("epochstart = ");
    Serial.print(epochstart);
    Serial.print(",   epoch = ");
    Serial.println(epoch);

}



//Do the main execution loop
void loop()

{
  //Blynk.run();          //If Blynk being used

  checkreset();           //Has the GPIO (D3) been taken low to reset WiFiManager / clears SPIFFS?

  //Get epoch from millis count.  May get over writtem by NTP pull.  timefactor is for testing to accellerate time.
  epoch = epochstart + (((millis() - startmillis) / 1000) * timefactor);

  printNTP=0;

  //Check if it's time to display to get NTP time by checking Millis past against the wait period defined.
  //NTP pull is done periodically, counting Millis by internal count is very accurate so NTP not constantly needed.
  if (flash_phase == 0){      //Don't go to NTP during flash phase as it causes flicker

  SecondsSinceLastNTP = (millis() - LastNTP) / 1000; //How many seconds since LastNTP pull
  if (SecondsSinceLastNTP > NTPSeconds_to_wait)
  {
    Serial.print("millis = ");
    Serial.println(millis());
    Serial.print("start millis = ");
    Serial.println(startmillis);
    Serial.print("epochstart = ");
    Serial.println(epochstart);
    Serial.print("epoch = ");
    Serial.println(epoch);
    Serial.println("");

    Request_Time();         //Get the time
    printNTP=1;             //1 is a flag to serialprint the time (only used for NTP pull not for millis updates)
    delay(2000);
    while (!Check_Time())
    { //If no time recieved then do this
      delay(2000);
      TimeCheckLoop++;

      if (TimeCheckLoop > 5)
      { //If not time received even after 5x 2sec delays, then try re-getting time
        Request_Time();
      }
    }

    //Time confirmed received and more than wait period to pull NTP / Sunrise time
    LastNTP = millis(); //Set the LastNTP time to now - resets the wait time

    Serial.println();
    Serial.println("****************");
    Serial.println();

    yield();
    NTPSeconds_to_wait = NTPSecondstowait;        //Over write the initial wait period (1 sec) to the ongoing period (120 sec)
  }
  }

  //Epoch has been updated using NTP pull or counting Millis.  Now turn this into clock_minutes_from_midnight
  DecodeEpoch(epoch);     //Turn epoch time into Hours Minutes Seconds

  //Check if it's time to get Sunrise/Set times
  int SecondsSinceLastAPI = (millis() - LastAPI) / 1000;      //How many seconds since Last API pull
  if (SecondsSinceLastAPI > APISecondstowait)
  {
    LastAPI = millis();
    API_Request(); //get sunrise/sunset data
    yield();
  }

  //Check if it's time to display LEDs
  int SecondsSinceLastLED = (millis() - LastLED) / 1000;    //How many seconds since Last LED update
  if (SecondsSinceLastLED > LEDSecondstowait)
  {
    LastLED = millis();

    //Set the LED colours based on the Time and the Sun position.  Don't update LED colours when in flash mode (causes flicker)
    if (flash_phase == 0){
    DoTheLEDs();     
    }

    yield();
  } 

  //Check if flash is require and manipulate the brightness
  checkflash ();
}







//Update the time
void DecodeEpoch(unsigned long currentTime)
{
  // print the raw epoch time from NTP server
   if (printNTP ==1 && printthings ==1){
  Serial.print("The epoch UTC time is ");
  Serial.print(epoch);
  Serial.println();

  // print the hour, minute and second:
  Serial.print("The UTC time is ");      // UTC is the time at Greenwich Meridian (GMT)
  Serial.print((epoch % 86400L) / 3600); // print the hour (86400 equals secs per day)
  Serial.print(':');
  
  if (((epoch % 3600) / 60) < 10)
  {
    // In the first 10 minutes of each hour, we'll want a leading '0'
    Serial.print('0');
  }

  Serial.print((epoch % 3600) / 60); // print the minute (3600 equals secs per minute)
  Serial.print(':');

  if ((epoch % 60) < 10)
  {
    // In the first 10 seconds of each minute, we'll want a leading '0'
    Serial.print('0');
  }

  Serial.println(epoch % 60); // print the second
   }
  hour = (currentTime % 86400L) / 3600;

  minute = (currentTime % 3600) / 60;
  second = currentTime % 60;

  clock_AMPM = "AM";

  if (hour > 12)
  {
    hour = hour - 12;
    clock_AMPM = "PM";
  }

  if (printNTP ==1 && printthings ==1){
  Serial.print("UTC Hour: ");
  Serial.print(hour);
  Serial.print(",   Minute: ");
  Serial.print(minute);
  Serial.print(",   Second: ");
  Serial.println(second);
  Serial.println();
  }
  

  //Work out Hours/min into minutes from midnight to Calculate if it's AM or PM time
  working_hourtomin = hour;

  //PM add 12
  if (clock_AMPM == "PM")
  {
    working_hourtomin = hour + 12;
  }

  //Midnight = 0
  if (clock_AMPM == "AM" && hour == 12)
  {
    working_hourtomin = 0;
  }

  //Noon = 12
  if (clock_AMPM == "PM" && hour == 12)
  {
    working_hourtomin = 12;
  }

  clock_minutes_from_midnight = ((working_hourtomin * 60) + minute);

  //Get local minutes for day/night calc
  local_clock_minutes_from_midnight = clock_minutes_from_midnight+ (localUTC * 60);

  //If local_minutes is negative (e.g Negative UTC)
  if (local_clock_minutes_from_midnight > 1440)
  {
    local_clock_minutes_from_midnight = local_clock_minutes_from_midnight - 1440;
  }

    //If local_minutes is negative (e.g Negative UTC)
  if (local_clock_minutes_from_midnight <0 )
  {
    local_clock_minutes_from_midnight = local_clock_minutes_from_midnight + 1440;
  }


  if (printNTP ==1 && printthings ==1){
  Serial.print("UTC Clock - Mins from midnight = ");
  Serial.print(clock_minutes_from_midnight);
  Serial.print(",   Local - Clock - Mins from midnight = ");
  Serial.println(local_clock_minutes_from_midnight);
  Serial.println();
  Serial.println("****************");
  Serial.println();
  }
  
  //Work out Hours/min into minutes from midnight
  working_hourtomin = hour_sunrise;

  //PM add 12
  if (strcmp(SR_AMPM, "P") == 0)
  {
    working_hourtomin = hour_sunrise + 12;
  }

  //Midnight = 0
  if (strcmp(SR_AMPM, "A") == 0 && hour_sunrise == 12)
  {
    working_hourtomin = 0;
  }

  //Noon = 12
  if (strcmp(SR_AMPM, "P") == 0 && hour_sunrise == 12)
  {
    working_hourtomin = 12;
  }

  

  //UTC number of minutes from midnight until sunrise
  sunrise_minutes_from_midnight = ((working_hourtomin * 60) + minute_sunrise);   

  //Convert UTC sunrise_minutes_from_midnight into local_sunrise_minutes_from_midnight with UTC
  local_sunrise_minutes_from_midnight = sunrise_minutes_from_midnight + (localUTC * 60);

  //If local_minutes is greater than 1 day (e.g large postive UTC)
  if (local_sunrise_minutes_from_midnight > 1440)
  {
    local_sunrise_minutes_from_midnight = local_sunrise_minutes_from_midnight - 1440;
  }

  //If local_minutes is negative (e.g Negative UTC)
  if (local_sunrise_minutes_from_midnight <0 )
  {
    local_sunrise_minutes_from_midnight = local_sunrise_minutes_from_midnight + 1440;
  }

  //Work out Hours/min into minutes from midnight
  working_hourtomin = hour_sunset;

  //PM add 12
  if (strcmp(SS_AMPM, "P") == 0)
  {
    working_hourtomin = hour_sunset + 12;
  }

  //Midnight = 0
  if (strcmp(SS_AMPM, "A") == 0 && hour_sunset == 12)
  {
    working_hourtomin = 0;
  }

  //Noon = 12
  if (strcmp(SS_AMPM, "P") == 0 && hour_sunset == 12)
  {
    working_hourtomin = 12;  Serial.println();
  }

 
  sunset_minutes_from_midnight = ((working_hourtomin * 60) + minute_sunset);

  //Convert UTC sunrise_minutes_from_midnight into local_sunrise_minutes_from_midnight with UTC
  local_sunset_minutes_from_midnight = sunset_minutes_from_midnight + (localUTC * 60);

  //If local_minutes is greater than 1 day (e.g large postive UTC)
  if (local_sunset_minutes_from_midnight > 1440)
  {
    local_sunset_minutes_from_midnight = local_sunset_minutes_from_midnight - 1440;
  }
  
  //If local_minutes is negative (e.g Negative UTC)
  if (local_sunset_minutes_from_midnight <0 )
  {
    local_sunset_minutes_from_midnight = local_sunset_minutes_from_midnight + 1440;
  }

  if (printthings ==1 && printNTP == 1){
  Serial.print("UTC Hour: ");
  Serial.print(hour);
  Serial.print(",   Minute: ");
  Serial.print(minute);
  Serial.print(",   Second: ");
  Serial.println(second);
  Serial.println();

  Serial.print("local_sunrise_minutes_from_midnight = ");
  Serial.print(local_sunrise_minutes_from_midnight);
  Serial.print(",   local_sunset_minutes_from_midnight = ");
  Serial.println(local_sunset_minutes_from_midnight);
  Serial.print("sunrise_minutes_from_midnight = ");
  Serial.print(sunrise_minutes_from_midnight);
  Serial.print(",   sunset_minutes_from_midnight = ");
  Serial.println(sunset_minutes_from_midnight);
  Serial.println();
  }
}



//Get time from NTP Server
void Request_Time()
{
  epoch2=epoch;             //Used to check old (using millis) epoch against a new epoch from NTP to show drift.
  Serial.println("Getting Time");
  WiFi.hostByName(NTPServerName, timeServer);
  sendNTPpacket(timeServer); // send an NTP packet to a time server
}

bool Check_Time() //This returns a bool value based on UDP time being received and placed in epoch variable
{
  int cb = udp.parsePacket();
  if (!cb)
  {
    Serial.println("no packet yet");
    return false;
  }
  else
  {
    Serial.print("packet received, length=");
    Serial.println(cb);
    Serial.println();
    Serial.println("****************");
    Serial.println();

    // We've received a packet, read the data from it
    udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

    //the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, extract the two words:

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    //Serial.print("Seconds since Jan 1 1900 = " );
    // Serial.println(secsSince1900);

    // now convert NTP time into everyday time:
    //Serial.print("Unix time = ");
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    lastepoch = epoch; //Used to compare last known epoch time with new NTP

    // subtract seventy years:
    epoch = secsSince1900 - seventyYears;

    //Check if there are lost packets (epoch is wildly different from last time).  If yes, use last epoch
    //if first epoch time is wrong this is constantly fail.

    Serial.print("epoch:  ");
    Serial.println(epoch);

    if (abs(epoch - lastepoch) > 3600 && lastepoch != 0) //Check if the old and new epoch times are more than 60s x 60 (1hr) and lastepoch isn't 0 (not had a time before)
    {
      Serial.println("epoch vs oldepoch > 1hr");
      if (lastepochcount <= 3)
      {
        epoch = lastepoch; //If more than 1hr difference, and old/new different less than 'N' times
        lastepochcount = lastepochcount + 1;
        totalfailepoch = totalfailepoch + 1;

        Serial.println("Using oldepoch from millis");
        Serial.print("previously failed = ");
        Serial.println(totalfailepoch);
        Serial.println();
      }
      else
      {
        lastepochcount = 0; //It's different more than 'N' times, inital NTP must have been wrong.  Stay with last recieved epoch.
        lastepoch = epoch;
        Serial.println("Using new epoch even though different.  It's been different too many times - resetting");
        Serial.print("previously failed = ");
        Serial.print(totalfailepoch);
        Serial.println("Making internal clock = new NTP time");
        Serial.println();

        epochstart = epoch;     //Using NTP epoch time.  Reset the millis time variables to use this as new starting point
        startmillis = millis(); //Using NTP epoch time.  Reset the millis time variables to use this as new starting point
      }
    }
    else
    {
      Serial.print("epoch is good");
      Serial.print(",   previously failed = ");
      Serial.println(totalfailepoch);
      Serial.println("Making internal clock = new NTP time");
      Serial.println();

      lastepochcount = 0;     //With a good epoch reset the bad epoch counter to zero
      lastepoch = epoch;      //With a good epoch make lastepoch the new good one for next loop
      epochstart = epoch;     //Using NTP epoch time.  Reset the millis time variables to use this as new starting point
      startmillis = millis(); //Using NTP epoch time.  Reset the millis time variables to use this as new starting point
    }

      Serial.print("new NTP epoch = ");
      Serial.print(epoch);
      Serial.print(",   Millis epoch = ");
      Serial.print(epoch2);
      Serial.print(",   Difference (ms) = ");
      Serial.println(abs(epoch - epoch2));

    LastNTP = millis(); //Set the last millis time the NTP time was attempted
    RequestedTime = 0;
    TimeCheckLoop = 0;
    return true;
  }
}


// send an NTP request to the time server at the given address
void sendNTPpacket(const IPAddress &address) 
{
  Serial.println("sending NTP packet");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011; // LI, Version, Mode
  packetBuffer[1] = 0;          // Stratum, or type of clock
  packetBuffer[2] = 6;          // Polling Interval
  packetBuffer[3] = 0xEC;       // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}


// Get data from Sunrise API HTTP address
void API_Request()

{
  HTTPClient http;
  char buff[400];

  Serial.print("Getting Sunrise API data with: ");
  Serial.println(sunrise_api_request);
  http.begin(sunrise_api_request);

  int httpCode = http.GET(); //Serial.print("[HTTP] GET...\n");  Start connection and send HTTP header
  if (httpCode > 0)          // httpCode will be negative on error
  {
    // HTTP header has been send and Server response header has been handled
    //Serial.printf("[HTTP] GET... code: %d\n", httpCode);
    if (httpCode == HTTP_CODE_OK)
    {
      String payload = http.getString();
      payload.toCharArray(buff, 400);
      sunAPIresponse = payload; //Sunresponse is used by JSON function
      Serial.println("API response received");
      Serial.println();

      sscanf(JSON_Extract("sunrise").c_str(), "%d:%d:%*d %c", &hour_sunrise, &minute_sunrise, SR_AMPM); //Get JSON for sunrise (string) convert to const char and search for hours:minutes
      sscanf(JSON_Extract("sunset").c_str(), "%d:%d:%*d %c", &hour_sunset, &minute_sunset, SS_AMPM);    //Get JSON for sunset (string) convert to const char and search for hours:minutes

      Serial.print("hour_sunrise = ");
      Serial.print(hour_sunrise);
      Serial.print(",   minute_sunrise = ");
      Serial.print(minute_sunrise);
      Serial.print(",   SR AMPM = ");
      Serial.println(SR_AMPM);
      Serial.print("hour_sunset = ");
      Serial.print(hour_sunset);
      Serial.print(",   minute_sunset = ");
      Serial.print(minute_sunset);
      Serial.print(",   SS AMPM = ");
      Serial.println(SS_AMPM);
      Serial.println();
      Serial.println("****************");
      Serial.println();      
    }
  }
  else
  {
    Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
  }
  http.end();
}


//Connect to the WiFi and manage credentials
void WiFi_and_Credentials()
{
  //2 way to get WiFi.  ConnectToAP uses variables from build flags (platformio) or hard coded. 
  //WiFiManager will check if WiFi credentials are known (in Flash memory).  If not it will stary a webserver to collect details from user
  
  //Use one of the follow lines depending on approach (build flags vs WiFiManager)
  // ConnectToAP();           //Connect to Wifi (if not using Wifi Manager approach)
  WiFiManager wifiManager;  

  //SPIFFs section to Read and Write the saved credentials in Flash memory
  //Check if APIaddress.txt exists.  If not create it and store the http address for sunrise API, if yes read it.
  
  if(SPIFFS.begin())
  {
    Serial.println("SPIFFS Initialize....ok");
  }
  else
  {
    Serial.println("SPIFFS Initialization...failed");
  }

  if (SPIFFS.exists("\" & HTTPfilename") == true){
    Serial.println("File already exisits.  Read stored data.");

  //Read HTTP addresss file data
  File f = SPIFFS.open("\" & HTTPfilename", "r");
  
  if (!f) {
    Serial.println("file open failed");
  }
  else
  {
      Serial.println("Reading Data from HTTP file:");
      //Data from file

      size_t size = f.size();

      f.readBytes(sunrise_api_request, size);

      f.close();  //Close file
      Serial.printf("Read http adress for Sunrise/set API = %s\n", sunrise_api_request);
      Serial.println("File Closed");


//Read Mode file data
  File g = SPIFFS.open("\" & Modefilename", "r");
  
  if (!g) {
    Serial.println("file open failed");
  }
  else
  {
      Serial.println("Reading Data from Mode file:");
      //Data from file

      size_t size = g.size();

      g.readBytes(mode, size);

      g.close();  //Close file
      Serial.printf("Read Lamp Mode  = %s\n", mode);
      Serial.println("File Closed");
  }


//Read UTC file data
  File h = SPIFFS.open("\" & UTCfilename", "r");
  
  if (!h) {
    Serial.println("file open failed");
  }
  else
  {
      Serial.println("Reading Data from UTC file:");
      //Data from file

      size_t size = h.size();

      h.readBytes(UTC, size);

      h.close();  //Close file
      Serial.printf("Read UTC  = %s\n", UTC);
      Serial.println("File Closed");
  }


      //WiFiManager will read stored WiFi Data, if it can't connect it will create a website to get new credentials.
      wifiManager.autoConnect("WiFi_Lamp");    
    }
  }

  else {
    Serial.println("Filename DOESN'T exisit");

  //If file doesn't exist, get details from the user with wifimanager website
  //create http address and store in APIaddresst.txt file
  
  WiFiManagerParameter custom_longitude("Longitude", "Longitude", "", 10);
    WiFiManagerParameter custom_latitude("Latitude", "Latitude", "", 10);
  WiFiManagerParameter custom_mode("Mode", "Mode", "", 4);
    WiFiManagerParameter custom_UTC("UTC", "UTC", "", 3);

    wifiManager.addParameter(&custom_longitude);
  wifiManager.addParameter(&custom_latitude);
    wifiManager.addParameter(&custom_mode);
  wifiManager.addParameter(&custom_UTC);

  //WiFiManager will read stored WiFi Data, if it can't connect it will create a website to get new credentials
  wifiManager.autoConnect("WiFi_Lamp");
  
  //Check if new http address needed to be written to file.  If yes, create and write.
  //Example: sunrise_api_request = "http://api.sunrise-sunset.org/json?lat=-41.2865&lng=174.7762" 
  //Mode of the Lamp (e.g 0, 1, 2) as separate files in SPIFFS
  sprintf(sunrise_api_request, "http://api.sunrise-sunset.org/json?lat=%s&lng=%s",  + custom_latitude.getValue(), custom_longitude.getValue());
  sprintf (mode, "%s", + custom_mode.getValue());
  sprintf (UTC, "%s", + custom_UTC.getValue());

  //Create New HTTP File And Write Data to It
  //w=Write Open file for writing
  File f = SPIFFS.open("\" & HTTPfilename", "w");
  
  if (!f) {
    Serial.println("HTTP file open failed");
    }
  else
    {
      //Write data to file
      Serial.println("Writing Data to HTTP File");
      f.print(sunrise_api_request);
        Serial.println("New file written");
      f.close();  //Close file
    }


  //Create New Mode File And Write Data to It
  //w=Write Open file for writing
  File g = SPIFFS.open("\" & Modefilename", "w");
  
  if (!g) {
    Serial.println("Mode file open failed");
    }
  else
    {
      //Write data to file
      Serial.println("Writing Data to Mode File");
      g.print(mode);
        Serial.println("New file written");
      g.close();  //Close file
    }

  //Create New UTC File And Write Data to It
  //w=Write Open file for writing
  File h = SPIFFS.open("\" & UTCfilename", "w");
  
  if (!h) {
    Serial.println("UTC file open failed");
    }
  else
    {
      //Write data to file
      Serial.println("Writing Data to UTC File");
      h.print(UTC);
        Serial.println("New file written");
      h.close();  //Close file
    }  
  }

  //Check File content or web entered details and correct if needed
  //Get light mode from saved file and turn into an Int and check it's either 0, 1 or 2
  //Get TARDIS from saved file and turn into an Int and check it's either 0, 1  (light on top of TARDIS)
  char buffer[0];
  char buffer1[0];
  char buffer2[0];  
  char buffer3[0];
  buffer[0] = mode[0];
  buffer1[0] = mode[1];
  buffer2[0] = mode[2];
  buffer3[0] = mode[3];

  lightmode = atoi(buffer);
  TARDIS = atoi(buffer1);
  flash = atoi(buffer2);
  int howbright_temp = atoi(buffer3);

    //Check light mode is valid.  
    if (lightmode <0 || lightmode >2){
      Serial.println("Light mode incorrect - overriding");
      lightmode = 0;
    }
      Serial.print("Light mode used = ");
      Serial.println(lightmode);

    //Check Top light entry is valid
    if (TARDIS <0 || TARDIS >4){
      Serial.println("Top light incorrect - overriding");
      TARDIS = 3;
    }
      Serial.print("Top light used = ");
      Serial.println(TARDIS);

    //Check flash mode is valid.  
    if (flash <0 || flash >1){
      Serial.println("Flash mode incorrect - overriding");
      flash = 0;
    }
      Serial.print("Flash mode used = ");
      Serial.println(flash);

    //Check brightness is valid.  
    if (howbright_temp <1 || howbright_temp >5){
      Serial.println("Brightness incorrect - overriding");
      howbright_temp = 5;
    }
      Serial.print("Brightness (1-5) = ");
      Serial.println(howbright_temp);

      //Entry x 5 +5 gives range of 55-255 
      howbright = (howbright_temp * 50) + 5;

      Serial.print("Brightness = ");
      Serial.println(howbright);

    //Get UTC from saved file and turn into an Int and check it's between 0-24
    char buffer4[3];
    buffer4[0] = UTC[0];
    buffer4[1] = UTC[1];
    buffer4[2] = UTC[2];
    localUTC = atoi(buffer4);

    if (localUTC < -12 || localUTC > 12){
      Serial.println("UTC mode incorrect - overriding");
      localUTC = 12;
    }
      Serial.print("UTC used = ");
      Serial.println(localUTC);
      
}


void ConnectToAP()
{
  Serial.println("Attempting to Connect");
  randomSeed(analogRead(6));

  while (true)
  {
    delay(1000);
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, pass);
    for (int x = 0; x < 5; x++)
    {
      delay(1000);
      if (WiFi.status() == WL_CONNECTED)
      {
        Serial.print("WiFi connected in ");
        Serial.print(x);
        Serial.println(" seconds");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
        Serial.println();
        return;
      }
    }
  }
}


//JSON Function
String JSON_Extract(String lookfor)
{
  DynamicJsonBuffer jsonBuffer;
  JsonObject &root = jsonBuffer.parseObject(sunAPIresponse);
  JsonObject &data = root["results"];
  return data[lookfor];
}

/*
BLYNK_WRITE(V1) // Widget WRITEs to Virtual Pin
{   
  red_max  = param.asInt(); // getting first value
}

BLYNK_WRITE(V2) // Widget WRITEs to Virtual Pin
{   
  green_max = param.asInt(); // getting N value
}

BLYNK_WRITE(V3) // Widget WRITEs to Virtual Pin
{   
  blue_max = param.asInt(); // getting second value
  }
*/



//Calculate LED colours phases on the phase of the sun using sunrise API data and NTP time converted to local time (using UTC offset)
void DoTheLEDs()
{
  Serial.print("UTC Hour: ");
  Serial.print(hour);
  Serial.print(",   Minute: ");
  Serial.print(minute);
  Serial.print(",   Second: ");
  Serial.println(second);
  Serial.println();
 
  //Check for sunrise.  clock_minutes_from_midnight is time in minutes from midnight.  Sunrise/set minutes and clock are both UTC
  //Only compare UTC with UTC as local time (using UTC offset can change with daylight savings).  Local only for figuring out if it's night or day


  //Corrected Sunrise/Set and time variables
  int sunrise_minutes_from_midnight_corrected = sunrise_minutes_from_midnight;
  int sunset_minutes_from_midnight_corrected = sunset_minutes_from_midnight;
  int clock_minutes_from_midnight_corrected = clock_minutes_from_midnight;


  //e.g SR 0020 means 30mins before and 30 after would be 1430:0050.  Different timelines are difficult to compare.  Make 0020 = 1460 (1440 + 0020) then 30mins before/after:  1430:1490
  //Need to correct time 
  if (sunrise_minutes_from_midnight < (minswithin/2)){
      sunrise_minutes_from_midnight_corrected = sunrise_minutes_from_midnight + 1440;

      //if Sunrise corrected then correct time is also in the same way
      if (clock_minutes_from_midnight < (minswithin/2)){
      clock_minutes_from_midnight_corrected = clock_minutes_from_midnight + 1440;
    
      Serial.println("*sunrise period spans midnight UTC - corrected");
  }
    }

  if (sunset_minutes_from_midnight < (minswithin/2)){
      sunset_minutes_from_midnight_corrected = sunset_minutes_from_midnight + 1440;

      //if Sunrise corrected then correct time is also in the same way
      if (clock_minutes_from_midnight < (minswithin/2)){
      clock_minutes_from_midnight_corrected = clock_minutes_from_midnight + 1440;
    
      Serial.println("*sunset period spans midnight UTC - corrected");
  }
    }


  //Check for Sunrise phase
  if (clock_minutes_from_midnight_corrected >= (sunrise_minutes_from_midnight_corrected - (minswithin/2))  && clock_minutes_from_midnight_corrected <= (sunrise_minutes_from_midnight_corrected + (minswithin/2)))
  {
    SR_Phase = 1;
    LED_phase = ((clock_minutes_from_midnight_corrected - sunrise_minutes_from_midnight_corrected) + (minswithin / 2)) / (float)minswithin * 255;
  }
  else
  {
    SR_Phase = 0;
  }


  //Check for sunset.  clock_minutes_from_midnight is time in minutes from midnight.  Sunrise/set minutes is LOCAL time from API
  if (clock_minutes_from_midnight_corrected >= (sunset_minutes_from_midnight_corrected - (minswithin/2)) && clock_minutes_from_midnight_corrected <= (sunset_minutes_from_midnight_corrected + (minswithin / 2)))
  {
    SS_Phase = 1;
    LED_phase = ((clock_minutes_from_midnight_corrected - sunset_minutes_from_midnight_corrected) + (minswithin / 2)) / (float)minswithin * 255;
  }
  else
  {
    SS_Phase = 0;
  }

  //if it's not in sunrise or sunset sequence then find out if it's day (yellow) or night (blue) and set colour
  //Using Local UTC (don't care about daylight saving) for day or night
  if (local_clock_minutes_from_midnight > local_sunrise_minutes_from_midnight && local_clock_minutes_from_midnight < local_sunset_minutes_from_midnight)
  {
    night = 0;
  }
  else
  {
    night = 1;
  }

  if (printthings == 1){

    
  Serial.print("clock_minutes_from_midnight (UTC) = ");
  Serial.print(clock_minutes_from_midnight);
  Serial.print(",   clock_minutes_from_midnight (local) = ");
  Serial.println(local_clock_minutes_from_midnight);
  Serial.print("sunrise minutes_from_midnight (UTC) = ");
  Serial.print(sunrise_minutes_from_midnight);
  Serial.print(",   sunset_minutes_from_midnight (UTC) = ");
  Serial.println(sunset_minutes_from_midnight);
  Serial.print("SecondsSinceLastNTP: ");
  Serial.print(SecondsSinceLastNTP);
  Serial.print(",   Startmillis: ");
  Serial.print(startmillis);    
  Serial.print(",   epochstart: ");
  Serial.println(epochstart);  
  Serial.println();  
  Serial.print("flash_phase = ");
  Serial.print(flash_phase);
  Serial.print(",   night = ");
  Serial.print(night);
  Serial.print(",   Sunrise phase = ");
  Serial.print(SR_Phase);
  Serial.print(",   Sunset phase = ");
  Serial.println(SS_Phase);

  Serial.print("Mins to Sunset phase = ");
  Serial.print(sunset_minutes_from_midnight - clock_minutes_from_midnight - int(minswithin / 2));
  Serial.print(",   Mins to Sunrise phase = ");
  Serial.println(sunrise_minutes_from_midnight - clock_minutes_from_midnight - int(minswithin / 2));
  }

  //call function to select LED colours for either all day/night or just nightlight
  if (lightmode == 0)
  {
    daynight();
  }

  if (lightmode == 1 || lightmode == 2)
  {
    nightlight();
  }


  fill_solid(leds, NUM_LEDS_PER_STRIP, CRGB(red, green, blue));

  //Set the top light:  0=Same as other LEDs, 1=Red, 2=Green, 3=Blue, 4=White
  if (TARDIS ==1)
  {
    leds[NUM_LEDS_PER_STRIP - 1].setRGB(255, 0, 0);     //Light on top of TARDIS Red
  }

  if (TARDIS ==2)
  {
    leds[NUM_LEDS_PER_STRIP - 1].setRGB(0, 255, 0);     //Light on top of TARDIS Green
  }

  if (TARDIS ==3)
  {
    leds[NUM_LEDS_PER_STRIP - 1].setRGB(0, 0, 255);     //Light on top of TARDIS Blue
  }

  if (TARDIS ==4)
  {
    leds[NUM_LEDS_PER_STRIP - 1].setRGB(255, 255, 255);     //Light on top of TARDIS White
  }

  
  FastLED.setBrightness(howbright);
  FastLED.show();

  if (printthings == 1){
  Serial.print("LED_phase = ");
  Serial.print(LED_phase);
  Serial.print(",   red = ");
  Serial.print(red);
  Serial.print(",   blue = ");
  Serial.print(blue);
  Serial.print(",   green = ");
  Serial.println(green);
  Serial.println();
  Serial.println("****************");
  Serial.println();
  }
}


//daunight mode:  yellow during day, blue at night
void daynight()

{
  SRSS_Flip = 0;             //0=Normal sunrise/sunset, 1=Inverted sunrise/sunset colour change order (only used in nightlight mode 1 & 2)
  sunrise_sunset();

  //It's night and not in sunrise or sunset mode
  if (SR_Phase == 0 && SS_Phase == 0 && night == 1)
  {
    blue = blue_night;
    green = green_night;
    red = red_night;
  }

  //It's day and not in sunrise or sunset mode
  if (SR_Phase == 0 && SS_Phase == 0 && night == 0)
  {
    blue = blue_day;
    green = green_day;
    red = red_day;
  }
}


//nightlight mode:  Off during day, yellow during night
void nightlight()
{
  SRSS_Flip = 1;        //1=Inverted sunrise/sunset colour change order    
  sunrise_sunset();     //calculate LEDs

  //This section overrides the LED colours calculated from Sunrise/set phase.

  //Mode 0:  This nightlight function should not be called in daynight mode (mode 0)
  //Mode 1
  if (SR_Phase == 0 && SS_Phase == 0 && lightmode == 1 && night == 1)       //Not in Sunrise/set phase, I'm in mode 1 and it's night time - LEDs yellow
  {
    blue = blue_day;
    green = green_day;
    red = red_day;
  }

  if (SR_Phase == 0 && SS_Phase == 0 && lightmode == 1 && night == 0)      //Not in Sunrise/set phase, I'm in mode 1 and it's daytime - LEDs off
  {
    blue = 0;
    green = 0;
    red = 0;
  }

  //Mode 2
  if (lightmode == 2 && night == 0)      //Don't care about in Sunrise/set phase, I'm in mode 2 and it's daytime - LEDs off
  {
    blue = 0;
    green = 0;
    red = 0;
  }

  if (lightmode == 2 && night == 1)      //Don't care about Sunrise/set phase, I'm in mode 2 and it's night time - LEDs yellow
  {
    blue = blue_day;
    green = green_day;
    red = red_day;
  }


}

//Calculate sunrise/sunset LED colours
void sunrise_sunset(){

//This piece flips SR and SS phases if requested by use of SRSS_Flip (for nightlight)
    int SR_Phase_use;
    int SS_Phase_use;

  if (SR_Phase == 1 && SRSS_Flip == 0){
    SR_Phase_use = 1;
    SS_Phase_use = 0;
  }

  if (SR_Phase == 1 && SRSS_Flip == 1){
    SR_Phase_use = 0;
    SS_Phase_use = 1;
  }

  if (SS_Phase == 1 && SRSS_Flip == 0){
    SR_Phase_use = 0;
    SS_Phase_use = 1;
  }

  if (SS_Phase == 1 && SRSS_Flip == 1){
    SR_Phase_use = 1;
    SS_Phase_use = 0;
  }

Serial.print("SR_Phase_use = ");
Serial.print(SR_Phase_use);
Serial.print(",   SS_Phase_use = ");
Serial.println(SS_Phase_use);
Serial.println();

    //sunrise:  Start with blue reducing to zero, and red increasing, when blue 0 increase green
  if (SR_Phase_use == 1 && blue >= 0)
  {
    //in first part of sunrise (reduce blue / increase red)
    blue = blue_max - (LED_phase * 2); //LED Phase 0-128,  *2 as in 1st phase we need 0-255 of LED movement for blue & red
    red = 0 + (LED_phase  * 2);

    if (blue < 0)
    {
      blue = 0;
    }
    if (red > red_max)
    {
      red = red_max;
    }
    green = 0;     //force in case lamp started during phase
  }

  if (SR_Phase_use == 1 && blue == 0)
  {
    //in second part of sunrise (increase green)
    green = 0 + abs(LED_phase - 128);  //LED Phase 128-255, need to be 0 to start green LED movement (0-128)
    
    if (green > green_max)
    {
      green = green_max;
    }
   }

    
  //sunset:  Start with green reducing to zero, then reducing red to 0 and increasing blue
  if (SS_Phase_use == 1 && green > 0)
  {
    green = green_max - (LED_phase);     //LED Phase 0-128, reduce green from 128 > 0
    
    if (green < 0)
    {
      green = 0;
    }

    blue = 0;     //force in case lamp started during phase
  }

    if (SS_Phase_use == 1 && green == 0)
  {
    //in second part of sunset increase blue and rdecrease red
    blue = 0 + (abs(LED_phase-128) *2);           //LED Phase 128-255, subtract 128 (gives range 0-128) then x2 (range 0-255)
    red = red_max - (abs(LED_phase-128) *2); //LED Phase 128-255, subtract 128 (gives range 0-128) then x2 (range 0-255)

    if (blue > blue_max )
    {
      blue = blue_max;
    }

    if (red < 0)
    {
      red = 0;
    }
  }
}

//Check if flash is required and manipulate brightness
void checkflash (){ 

  //Check for x minutes within top of hour to do flash.  Only do this is flash is enabled in set up (flash =1)
  if (flash == 1){

  flash_phase = 0;
  if (minute < flash_mins){        //minutes = 0 at top of hour, so just check minutes is less or equal to the flash period
    flash_phase = 1;
  }
    }


  //If in flash mode then do the flash routine
  if (flash_phase == 1){

  //Check if the required time has passed to flash
  if (millis() - LastFlashmillis >= flash_delay){
    LastFlashmillis = millis();
    
    FastLED.setBrightness(howbright * sinetable[flash_working]);
    FastLED.show();

    flash_working = flash_working + 1;

    //return to begining of sequence
    if (flash_working == 26){
      flash_working = 0;
    }
    } 
  }
  }


//Check if reset button pressed.  D3 / GPIO0 held to ground.
void checkreset(){
    if (digitalRead(0) == 0){
      delay(500);             //500ms for button bounce
      if (digitalRead(0) == 0){

  //LEDs off 
  fill_solid(leds, NUM_LEDS_PER_STRIP, CRGB(0, 0, 0));
    FastLED.setBrightness(howbright);
      FastLED.show();        
    Serial.println("** RESET **");
    Serial.println("** RESET **");
    Serial.println("** RESET **");
      SPIFFS.remove("\" & HTTPfilename");
      SPIFFS.remove("\" & Modefilename");
      SPIFFS.remove("\" & UTCfilename");      
      SPIFFS.format();
      WiFi.disconnect();

    delay(2500);
      ESP.restart();
    }
      }
} 



