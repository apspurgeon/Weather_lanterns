#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ESP8266HTTPClient.h>
#include <FastLED.h>
#include <ArduinoJson.h>


//Put into platformio.ini
//build_flags =
//    -DSSID_NAME="SSID"
//    -DPASSWORD_NAME="WIFI_Password"


//*** Things to change  ***
#ifndef SSID_NAME
#define SSID_NAME "WIFI_SSID"     //Default SSID if not build flag from PlatformOI doesn't work
#endif

#ifndef PASSWORD_NAME
#define PASSWORD_NAME "WIFI_PASSWORD"     //Default SSID if not build flag from PlatformOI doesn't work
#endif

#define NUM_LEDS_PER_STRIP 15 //LEDs per strip
#define PIN_LED D7           //I.O pin on device going to LEDs
#define COLOR_ORDER GRB      // if colors are mismatched; change this  //RBG //GRB

const IPAddress timeServer(203, 118, 151, 32);                                                    // Your local NTP server
const int nzutc = 12;                                                                             //Country UTC offset, needed for UTC for day/night calc  (+12)  don't need to change for daylight saving as no needed for day/night
const char *sunrise_api_request = "http://api.sunrise-sunset.org/json?lat=-41.2865&lng=174.7762"; //API with your Long / Lat

const int green_nightlight = 255;   //Night RGB LED settings for night lightmode
const int blue_nightlight = 255;    //Night RGB LED settings for night light mode
const int red_nightlight = 255;     //Night RGB LED settings for night light mode

const int howbright = 255;          //0-255 LED Brightness level
const int lightmode = 2;           //0 = day/night    1 = night light mode with sunrise/set colour changes    2 = night light mode without sunrise/set changes  (binary on/off)

const int NTPSecondstowait = 600;   //Wait between NTP pulls (sec)
const int APISecondstowait = 3600; //Wait between Sunrise API pulls (sec)

const int LEDSecondstowait = 10;   //Wait between LED updates (sec)
const int minswithin = 60;         //Minutes within sunrise / sunset to begin the LED colour change sequence  (60 = phase starts 30mins before sunrise/set and end 30mins after)
const int change = 1;              //Speed of LED change in tones

const int testUTC = 0;          //*TESTING* Normal condition =0.    Force a UTC time (entered as minutes from midnight) for testing purposes  (making sure LEDs do as expected)
const int testDayNight = 1;      //*TESTING* If testUTC !=0 then this gets used for testing purposes
//*** Things to change  ***



//Gets SSID/PASSWORD from Platform.ini build flags
const char ssid[] = xstr(SSID_NAME);          //  your network SSID (name)
const char pass[] = xstr(PASSWORD_NAME);      // your network password

//LED Variables
int green = 0; 
int blue = 0;
int red = 0;

const int green_daynight = 128;  //Day RGB LED settings for day/night mode (Yellow)
const int blue_daynight = 0;     //Day RGB LED settings for day/night mode (Yellow)
const int red_daynight = 255;    //Day RGB LED settings for day/night mode (Yellow)

//Wifi and internet variables
const unsigned int localPort = 2390; // local port to listen for UDP packets
WiFiUDP udp;                         // A UDP instance to let us send and receive packets over UDP

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
int SS_Phase = 0;  //1 = in Sunset phase (30 mins either side if minwithin = 60mins)
int hourtomin = 0; //Used to convert hours into total minutes
float LED_phase;   //0-255 in the phase of sunrise/set   0=begining 255=end
char SR_AMPM[1], SS_AMPM[1];
String AMPM, sunAPIresponse;
struct CRGB leds[NUM_LEDS_PER_STRIP]; //initiate FastLED with number of LEDs

void nightlight ();
void daynight ();
String JSON_Extract(String);
void API_Request ();
void DoTheLEDs ();
bool Check_Time ();
void DecodeEpoch (unsigned long);
void sendNTPpacket (const IPAddress &address);
void ConnectToAP ();
void Request_Time ();

void setup()
{
  Serial.begin(9600);

  FastLED.addLeds<WS2811, PIN_LED, COLOR_ORDER>(leds, NUM_LEDS_PER_STRIP);

  ConnectToAP(); //Connect to Wifi

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

//Do the main execution
void loop()
{

  //Get epoch from millis count.  May get over writtem by NTP pull
  epoch = epochstart + (millis() - startmillis) / 1000;

  //Check if it's time to display to get NTP time
  int SecondsSinceLastNTP = (millis() - LastNTP) / 1000; //How many seconds since LastNTP pull
  if (SecondsSinceLastNTP > NTPSeconds_to_wait)
  {
    Request_Time(); //Get the time
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
    DecodeEpoch(epoch + SecondsSinceLastNTP); //Turn epoch time into Hours Minutes Seconds.  Work out timing for LEDs.  Must go after API request
    NTPSeconds_to_wait = NTPSecondstowait;    //Over write the initial wait period (1 sec) to the ongoing period (120 sec)
  }

  //Check if it's time to get Sunrise/Set times
  int SecondsSinceLastAPI = (millis() - LastAPI) / 1000; //How many seconds since Last API pull
  if (SecondsSinceLastAPI > APISecondstowait)
  {
    LastAPI = millis();
    API_Request(); //get sunrise/sunset data
    yield();
  }

  //Check if it's time to display LEDs
  int SecondsSinceLastLED = (millis() - LastLED) / 1000; //How many seconds since Last LED update
  if (SecondsSinceLastLED > LEDSecondstowait)
  {
    LastLED = millis();
    DoTheLEDs(); //Set the LED colours based on the Time and the Sun position
    yield();
  }
}

void DoTheLEDs()
{

  //Set the LED lights based on the NTP time and the Sunrise/Sunset time

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
  Serial.println("");

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

  if (lightmode == 0)
  {
    leds[NUM_LEDS_PER_STRIP - 1].setRGB(0, 0, 255); //Light on top of TARDIS but only if in day/night mode
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

//Get Time
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

  //*******************

  //Work out Hours/min into minutes from midnight
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

void sendNTPpacket(const IPAddress &address) // send an NTP request to the time server at the given address
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

void API_Request()
// Start api sunrise-sunset
{
  HTTPClient http;
  char buff[400];

  Serial.print("Getting Sunrise API data");
  Serial.println();
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