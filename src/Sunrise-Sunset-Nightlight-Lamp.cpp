// ****************************************************************************************************************************************************************
// ****************************************************************************************************************************************************************
// This is designed to run on an ESP8266 using FastLED, with optional use of Blynk for a phone App
// It connects to an NTP time server, and sunrise/set server to determine time of day (UTC) and when the sun
// will rise and set based on Longitude and Latitude co-ordinates.  
//
// There are 2 modes of operation as set by the nightmode var.  
// 0 = day time LEDs are yellow, night LEDs are blue and as sun rises/sets LEDs reflect the colours of that change
// 1 = night light mode.  day time LEDs are off, night LEDs are yellow and as sun rises/sets LEDs reflect the colours of that change
// 2 = night light mode.  day time LEDs are off, night LEDs are yellow  with no sun rise/set LEDs changes (e.g a hard on/off)
//
// Variables of interest are found between the 'Things to change' comment lines
// Including the time delay between updating LEDs, time between NTP time checks and time between sunrise/set API requests.
// Note: After NTP time is received, internal Millis clock tracks time fairly acuratley, sunrise/set times only change once a day.
// Note: NTP is UDP and can fail, there is a check that the new NTP time isn't too different from expected time.  If it is, it keeps using 
// current time, unless it fails 3 times.  Then it will use NTP time (assumed after 3 times the NTP is correct afterall compared to Millis)
// red/green/blue_nightlight variables allow you to specify the colour at night time.  These can be controlled by a Blynk app (virtual pins 1,2 & 3)
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
#include <BlynkSimpleEsp8266.h>
#include <FS.h>
#define BLYNK_PRINT Serial

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

#define NUM_LEDS_PER_STRIP 4  //Number of LEDs per strip
#define PIN_LED D7            //I.O pin on ESP2866 device going to LEDs
#define COLOR_ORDER RGB       // LED stips aren't all in the same RGB order.  If colours are wrong change this  e.g  RBG > GRB.   :RBG=TARDIS

String HTTPfilename = "APIaddress.txt";             //Filename for storing Sunrise API HTTP address in SPIFFS
const char *NTPServerName = "0.nz.pool.ntp.org";    //Your local NTP server
const int nzutc = 12;              //Country UTC offset, needed for UTC for day/night calc  (+12 for NZ)  don't need to change for daylight saving as no needed for day/night

int green_nightlight = 128;        //Night RGB LED settings for night lightmode
int blue_nightlight = 0;           //Night RGB LED settings for night light mode
int red_nightlight = 255;          //Night RGB LED settings for night light mode

const int howbright = 255;         //0-255 LED Brightness level
const int lightmode = 0;           //0 = day/night (TARDIS)    1 = night light mode with sunrise/set colour changes    2 = night light mode without sunrise/set changes  (binary on/off)
const int TARDIS = 1;              //Used for my TARDIS lamp (only works in lightmode = 0).  All LEDs work as per day/night lightmode, except 1 LED (last in strip) at the top of the TADIS which is forced Blue.

const int NTPSecondstowait = 600;  //Wait between NTP pulls (sec)
const int APISecondstowait = 3600; //Wait between Sunrise API pulls (sec)

const int LEDSecondstowait = 5;    //Wait between LED updates (sec)
const int minswithin = 60;         //Minutes within sunrise / sunset to begin the LED colour change sequence  (60 = phase starts 30mins before sunrise/set and end 30mins after)
const int change = 1;              //Speed of LED change in tones.  Recommend = 1

const int testUTC = 0;             //*TESTING* Normal condition =0.    Force a UTC time (entered as minutes from midnight) for testing purposes  (making sure LEDs do as expected)
const int testDayNight = 1;        //*TESTING* If testUTC !=0 then this gets used for testing purposes

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
int hour, minute, second;           //UTC time
int clock_minutes, nzclock_minutes; //Minutes from midnight
int NTPSeconds_to_wait = 1;         // -  Initial wait time between NTP/Sunrise pulls (1 sec)
String clock_AMPM;

//Sunrise - Sunset API variables
int h_sunrise, hour_sunrise, minute_sunrise;
int sunrise_minutes, nzsunrise_minutes; //Minutes from midnight
int SR_Phase = 0;                       //1 = in Sunrise phase (30 mins either side if minwithin = 60mins)
int h_sunset, hour_sunset, minute_sunset, sunset_minutes, nzsunset_minutes;
int SS_Phase = 0;                       //1 = in Sunset phase (30 mins either side if minwithin = 60mins)
int hourtomin = 0;                      //Used to convert hours into total minutes
float LED_phase;                        //0-255 in the phase of sunrise/set   0=begining 255=end
char SR_AMPM[1], SS_AMPM[1];
String AMPM, sunAPIresponse;
struct CRGB leds[NUM_LEDS_PER_STRIP];   //initiate FastLED with number of LEDs
String JSON_Extract(String);

//LED Variables. Hold the value (0-255) of each primary colour
int green = 0; 
int blue = 0;
int red = 0;

//What yellow looks like for day time
const int green_daynight = 128;  //Day RGB LED settings for day/night mode (Yellow)
const int blue_daynight = 0;     //Day RGB LED settings for day/night mode (Yellow)
const int red_daynight = 255;    //Day RGB LED settings for day/night mode (Yellow)

//Functions declared
void nightlight ();
void daynight ();
void API_Request ();
void DoTheLEDs ();
bool Check_Time ();
void DecodeEpoch (unsigned long);
void sendNTPpacket (const IPAddress &address);
void ConnectToAP ();
void Request_Time ();
void checkreset();
void WiFi_and_Credentials();



void setup()
{
  Serial.begin(9600);

  pinMode(0, INPUT);          //GPIO0 (D3) to GND to reset ESP2866
 
  FastLED.addLeds<WS2811, PIN_LED, COLOR_ORDER>(leds, NUM_LEDS_PER_STRIP);      //Initialise the LEDs

  WiFi_and_Credentials();     //Calls WiFi function to initiate.  either uses WifiManager to get Wifi and Longitude/Latitude data (And store API URL as SPIFFS file.)  r Standard WiFi connection with build flags.

  //Blynk setup (if being used).
  //Blynk.begin(auth, ssid, pass);


  //Initiate time
  Serial.println("Starting UDP");
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
  epochstart = epoch;     //epoch pulled from NTP server, use initial epoch to set starting point for epochmillis
  startmillis = millis(); //get starting point for millis
  API_Request();          //Get sunrise/sunset times
}



//Do the main execution loop
void loop()

{
  //Blynk.run();          //If Blynk being used
  
  checkreset();           //Has the GPIO (D3) been taken low to reset WiFiManager / clears SPIFFS?

  //Get epoch from millis count.  May get over writtem by NTP pull
  epoch = epochstart + (millis() - startmillis) / 1000;

  //Check if it's time to display to get NTP time by checking Millis past against the wait period defined.
  //NTP pull is done periodically, counting Millis by internal count is very accurate so NTP not constantly needed.
  int SecondsSinceLastNTP = (millis() - LastNTP) / 1000; //How many seconds since LastNTP pull
  if (SecondsSinceLastNTP > NTPSeconds_to_wait)
  {
    Request_Time();         //Get the time
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
    DecodeEpoch(epoch + SecondsSinceLastNTP);     //Turn epoch time into Hours Minutes Seconds.  Work out timing for LEDs.  Must go after API request
    NTPSeconds_to_wait = NTPSecondstowait;        //Over write the initial wait period (1 sec) to the ongoing period (120 sec)
  }

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
    DoTheLEDs();      //Set the LED colours based on the Time and the Sun position
    yield();
  }  
}



//Check if reset button pressed.  D3 / GPIO0 held to ground.
void checkreset(){
    if (digitalRead(0) == 0){
      delay(500);             //500ms for button bounce
      if (digitalRead(0) == 0){
    Serial.println("** RESET **");
    Serial.println("** RESET **");
    Serial.println("** RESET **");
      SPIFFS.remove("\" & HTTPfilename");
      SPIFFS.format();
      WiFi.disconnect();
    delay(2500);
      ESP.restart();
    }
      }
}


//Calculate LED colours phases on the phase of the sun using sunrise API data and NTP time.
void DoTheLEDs()
{
  //Check for sunrise.  Clock_minutes is time in minutes from midnight
  if (clock_minutes >= (sunrise_minutes - (minswithin / 2)) && clock_minutes <= (sunrise_minutes + (minswithin / 2)))
  {
    SR_Phase = 1;
    LED_phase = ((clock_minutes - sunrise_minutes) + (minswithin / 2)) / (float)minswithin * 255;
  }
  else
  {
    SR_Phase = 0;
  }

  //Check for sunset.  Clock_minutes is time in minutes from midnight
  if (clock_minutes >= (sunset_minutes - (minswithin / 2)) && clock_minutes <= (sunset_minutes + (minswithin / 2)))
  {
    SS_Phase = 1;
    LED_phase = ((clock_minutes - sunset_minutes) + (minswithin / 2)) / (float)minswithin * 255;
  }
  else
  {
    SS_Phase = 0;
  }

  //if it's not in sunrise or sunset sequence then find out if it's day (yellow) or night (blue) and set colour
  //Using nzutc estimate (don't care about daylight saving) for day or night
  if (nzclock_minutes > nzsunrise_minutes && nzclock_minutes < nzsunset_minutes)
  {
    night = 0;
  }
  else
  {
    night = 1;
  }


  //****** For TESTING purposes only ******
  if (testUTC != 0){
  clock_minutes = testUTC;  // Force the UTC time to for testing purposes (do the LEDs work as expected at this time)
  night = testDayNight;
  }
  //****** For TESTING purposes only ******


  Serial.print("night = ");
  Serial.println(night);
  Serial.print("Sunrise phase = ");
  Serial.println(SR_Phase);
  Serial.print("Sunset phase = ");
  Serial.println(SS_Phase);
  Serial.print("Mins to Sunset phase = ");
  Serial.println(sunset_minutes - clock_minutes - int(minswithin / 2));
  Serial.print("Mins to Sunrise phase = ");
  Serial.println(sunrise_minutes - clock_minutes - int(minswithin / 2));
  

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

  if (lightmode == 0 && TARDIS ==1)
  {
    leds[NUM_LEDS_PER_STRIP - 1].setRGB(0, 0, 255); //Light on top of TARDIS Blue - But only if in day/night mode and TARDIS ==1
  }

  FastLED.setBrightness(howbright);
  FastLED.show();

  Serial.print("LED_phase = ");
  Serial.print(LED_phase);
  Serial.println();
  Serial.print("red = ");
  Serial.print(red);
  Serial.print(",   blue = ");
  Serial.print(blue);
  Serial.print(",   green = ");
  Serial.println(green);
  Serial.println();
  Serial.println("****************");
  Serial.println();
}



//Update the time
void DecodeEpoch(unsigned long currentTime)
{
  // print the raw epoch time from NTP server
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
  hour = (currentTime % 86400L) / 3600;

  minute = (currentTime % 3600) / 60;
  second = currentTime % 60;

  clock_AMPM = "AM";

  if (hour > 12)
  {
    hour = hour - 12;
    clock_AMPM = "PM";
  }

  Serial.println();
  Serial.print("Hour: ");
  Serial.print(hour);
  Serial.print(",   Minute: ");
  Serial.print(minute);
  Serial.print(",   Second: ");
  Serial.println(second);
  Serial.println();

  

  //Work out Hours/min into minutes from midnight to Calculate if it's AM or PM time
  hourtomin = hour;

  //PM add 12
  if (clock_AMPM == "PM")
  {
    hourtomin = hour + 12;
  }

  //Midnight = 0
  if (clock_AMPM == "AM" && hour == 12)
  {
    hourtomin = 0;
  }

  //Noon = 12
  if (clock_AMPM == "PM" && hour == 12)
  {
    hourtomin = 12;
  }

  clock_minutes = ((hourtomin * 60) + minute);

  //Get to NZ minutes for day/night calc
  nzclock_minutes = clock_minutes + (nzutc * 60);

  if (nzclock_minutes > 1440)
  {
    nzclock_minutes = nzclock_minutes - 1440;
  }

  Serial.print("Clock - Mins from midnight = ");
  Serial.println(clock_minutes);
  Serial.print("NZ - Clock - Mins from midnight = ");
  Serial.println(nzclock_minutes);
  Serial.println();
  Serial.println("****************");
  Serial.println();

  //Work out Hours/min into minutes from midnight
  hourtomin = hour_sunrise;

  //PM add 12
  if (strcmp(SR_AMPM, "P") == 0)
  {
    hourtomin = hour_sunrise + 12;
  }

  //Midnight = 0
  if (strcmp(SR_AMPM, "A") == 0 && hour_sunrise == 12)
  {
    hourtomin = 0;
  }

  //Noon = 12
  if (strcmp(SR_AMPM, "P") == 0 && hour_sunrise == 12)
  {
    hourtomin = 12;
  }

  sunrise_minutes = ((hourtomin * 60) + minute_sunrise);

  //Get to NZ minutes for day/night calc
  nzsunrise_minutes = sunrise_minutes + (nzutc * 60);

  if (nzsunrise_minutes > 1440)
  {
    nzsunrise_minutes = nzsunrise_minutes - 1440;
  }

  Serial.print("Sunrise - Mins from midnight = ");
  Serial.println(sunrise_minutes);
  Serial.print("NZ - Sunrise - Mins from midnight = ");
  Serial.println(nzsunrise_minutes);

  //Work out Hours/min into minutes from midnight
  hourtomin = hour_sunset;

  //PM add 12
  if (strcmp(SS_AMPM, "P") == 0)
  {
    hourtomin = hour_sunset + 12;
  }

  //Midnight = 0
  if (strcmp(SS_AMPM, "A") == 0 && hour_sunset == 12)
  {
    hourtomin = 0;
  }

  //Noon = 12
  if (strcmp(SS_AMPM, "P") == 0 && hour_sunset == 12)
  {
    hourtomin = 12;
  }

  sunset_minutes = ((hourtomin * 60) + minute_sunset);

  //Get to NZ minutes for day/night calc
  nzsunset_minutes = sunset_minutes + (nzutc * 60);

  if (nzsunset_minutes > 1440)
  {
    nzsunset_minutes = nzsunset_minutes - 1440;
  }

  Serial.print("Sunset - Mins from midnight = ");
  Serial.println(sunset_minutes);
  Serial.print("NZ - Sunset - Mins from midnight = ");
  Serial.println(nzsunset_minutes);
  Serial.println();
}



//Get time from NTP Server
void Request_Time()
{
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
      Serial.println("epoch is good");
      Serial.print("previously failed = ");
      Serial.println(totalfailepoch);
      Serial.println("Making internal clock = new NTP time");
      Serial.println();

      lastepochcount = 0;     //With a good epoch reset the bad epoch counter to zero
      lastepoch = epoch;      //With a good epoch make lastepoch the new good one for next loop
      epochstart = epoch;     //Using NTP epoch time.  Reset the millis time variables to use this as new starting point
      startmillis = millis(); //Using NTP epoch time.  Reset the millis time variables to use this as new starting point
    }

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
      Serial.println(hour_sunrise);
      Serial.print("minute_sunrise = ");
      Serial.println(minute_sunrise);
      Serial.print("SR AMPM = ");
      Serial.println(SR_AMPM);
      Serial.print("hour_sunset = ");
      Serial.println(hour_sunset);
      Serial.print("minute_sunset = ");
      Serial.println(minute_sunset);
      Serial.print("SS AMPM = ");
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

  //Read File data
  File f = SPIFFS.open("\" & HTTPfilename", "r");
  
  if (!f) {
    Serial.println("file open failed");
  }
  else
  {
      Serial.println("Reading Data from File:");
      //Data from file

      size_t size = f.size();

      f.readBytes(sunrise_api_request, size);

      f.close();  //Close file
      Serial.print("READ: sunrise_api_request = ");
        Serial.println(sunrise_api_request);
      Serial.println("File Closed");

      //WiFiManager will read stored WiFi Data, if it can't connect it will create a website to get new credentials.
      wifiManager.autoConnect("WiFi_Lamp");    
    }
  }

  else {
    Serial.println("Filename DOESN'T exisit");

  //If file doesn't exist, get details from the user with wifimanager website
  //create http address and store in APIaddresst.txt file
  
  WiFiManagerParameter custom_longitude("Longitude", "Longitude", "longitude", 10);
    WiFiManagerParameter custom_latitude("Latitude", "Latitude", "latitude", 10);

    wifiManager.addParameter(&custom_longitude);
  wifiManager.addParameter(&custom_latitude);

  //WiFiManager will read stored WiFi Data, if it can't connect it will create a website to get new credentials
  wifiManager.autoConnect("WiFi_Lamp");
  

  //Check if new http address needed to be written to file.  If yes, create and write.
  //Example: sunrise_api_request = "http://api.sunrise-sunset.org/json?lat=-41.2865&lng=174.7762" 
  sprintf(sunrise_api_request, "http://api.sunrise-sunset.org/json?lat=%s&lng=%s",  + custom_latitude.getValue(), custom_longitude.getValue());

  Serial.print("New http adress for Sunrise/set API = ");
    Serial.println(sunrise_api_request);


  //Create New File And Write Data to It
  //w=Write Open file for writing
  File f = SPIFFS.open("\" & HTTPfilename", "w");
  
  if (!f) {
    Serial.println("file open failed");
    }
  else
    {
      //Write data to file
      Serial.println("Writing Data to File");
      f.print(sunrise_api_request);
        Serial.println("New file written");
      f.close();  //Close file
    }
      }
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
  red_nightlight  = param.asInt(); // getting first value
}

BLYNK_WRITE(V2) // Widget WRITEs to Virtual Pin
{   
  green_nightlight = param.asInt(); // getting N value
}

BLYNK_WRITE(V3) // Widget WRITEs to Virtual Pin
{   
  blue_nightlight = param.asInt(); // getting second value
  }
*/



void daynight()
{
  //sunrise:  Start with blue reducing to zero, and red increasing then green increases

  if (SR_Phase == 1 && blue >= 0)
  {
    //in first part of sunset (reduce blue, increase red)

    blue = blue_daynight - (LED_phase * change * 2); //work out 0 - 255 for sunset.  4 Because in 2 phases (/2) and green starts at 128 (/4)

    if (blue < 0)
    {
      blue = 0;
    }

    red = abs(255 - (LED_phase * change * 2)); //work out 0 - 255 for sunset.

    if (red >= 255)
    {
      red = 255;
    }
  }

  if (SR_Phase == 1 && blue == 0)
  {
    //in second part of sunrise (increase green)

    green = abs(255 - (LED_phase * change * 2)); //work out 0 - 255 for sunset.

    if (green > 255)
    {
      green = 255;
    }
  }

  //sunset:  Start with green reducing to zero, only then red reduces @ same rate that blue increases

  if (SS_Phase == 1 && green >= 0)
  {
    //in first part of sunset (reduce green only)

    green = green_daynight - (int)LED_phase; //work out 0 - 255 for sunset.  4 Because in 2 phases (/2) and green starts at 128 (/4)

    if (green < 0)
    {
      green = 0;
    }
  }

  if (SS_Phase == 1 && green == 0)
  {
    //in second part of sunset (reduce red, increase blue)

    red = 255 - abs(255 - (LED_phase * change * 2)); //work out 0 - 255 for sunset
    blue = abs(255 - (LED_phase * change * 2));      //work out 0 - 255 for sunset.

    if (red < 0)
    {
      red = 0;
    }

    if (blue > 255)
    {
      blue = 255;
    }
  }

  //It's night and not in sunrise or sunset mode
  if (SR_Phase == 0 && SS_Phase == 0 && night == 1)
  {
    blue = 255;
    green = 0;
    red = 0;
  }

  //It's day and not in sunrise or sunset mode
  if (SR_Phase == 0 && SS_Phase == 0 && night == 0)
  {
    blue = 0;
    green = 128;
    red = 255;
  }
}

void nightlight()
{
  //sunset:  Start with blue reducing to zero, and red increasing then green increases

  if (SS_Phase == 1 && blue >= 0)
  {

    blue = 255 - (LED_phase * change); //work out 0 - 255 for sunset.  4 Because in 2 phases (/2) and green starts at 128 (/4)

    if (blue < 0)
    {
      blue = 0;
    }
  }

  red = 0 + (LED_phase * change); //work out 0 - 255 for sunset.

  if (red >= 255)
  {
    red = 255;
  }

  if (SS_Phase == 1 && blue == 0)
  {
    //in second part of sunrise (increase green)

    green = 255 - (LED_phase * change); //work out 0 - 255 for sunset.

    if (green > 128)
    {
      green = 128;
    }
  }

  //sunrise:  Start with green reducing to zero, only then red reduces @ same rate that blue increases

  if (SR_Phase == 1 && green >= 0)
  {
    //in first part of sunset (reduce green only)

    green = 128 - (LED_phase * change); //work out 0 - 255 for sunset.  4 Because in 2 phases (/2) and green starts at 128 (/4)
    red = 255 - (LED_phase * change);

    if (green < 0)
    {
      green = 0;
    }
  }

  if (SR_Phase == 1 && green == 0)
  {
    //in second part of sunrise (reduce red, increase blue)

    red = 255 - (LED_phase * change);
    blue = LED_phase * change; //work out 0 - 255 for sunset.

    if (red < 0)
    {
      red = 0;
    }

    if (blue > 255)
    {
      blue = 255;
    }
  }

  //It's day and not in sunrise or sunset mode  or  if it's in lightmode 2 (ignore sunrise/set changes) then fore into Day or Night LED colours
  if (((SR_Phase == 0 && SS_Phase == 0) || lightmode == 2) && night == 0)
  {
    blue = 0;
    green = 0;
    red = 0;
  }

  //It's night and not in sunrise or sunset mode  or  if it's in lightmode 2 (ignore sunrise/set changes) then fore into Day or Night LED colours
  if (((SR_Phase == 0 && SS_Phase == 0) || lightmode == 2) && night == 1)
  {
    blue = blue_nightlight; 
    green = green_nightlight;
    red = red_nightlight;
  }
}

