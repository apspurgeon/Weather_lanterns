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
// 1-8:  5-40seconds for flash period (5 sec blocks)
// 9: Flash the number of the hour (12hr not 24hr clock)

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

#include <ESP8266HTTPClient.h>
#include <FastLED.h>
#include <ArduinoJson.h>
#include <WiFiManager.h>
#include <FS.h> //For SPIFFs
#include <CapacitiveSensor.h>
#include <SoftwareSerial.h>    //Library for serial comms to TF Sound module
#include <DFPlayer_Mini_Mp3.h> //Library for TF Sound module
//#include <DNSServer.h>
//#include <Arduino.h>
//#include <BlynkSimpleEsp8266.h>
//#define BLYNK_PRINT Serial
//#include <string.h>
//#include <ESP8266WiFi.h>
//#include <WiFiUdp.h>

//Functions declared
void nightlight();                            //function if in nightlight mode
void daynight();                              //function if in day night mode
void API_Request();                           //Gets sunrise/set times from API
void DoTheLEDs();                             //Update LED colours based on time (minutes from midnight UTC)
void checkflash();                            //Check if flash is needed
void Request_Time();                          //Get time from NTP time server
bool Check_Time();                            //Check time is correct and ok
void decode_epoch(unsigned long);             //Turn Unix epoch time into hours:minutes
void sendNTPpacket(const IPAddress &address); //Get data
void ConnectToAP();                           //connect to WiFi Access point
void WiFi_and_Credentials();                  //Get WiFi credentials if using WiFi manager option (also connects to access point)
void checkreset(int);                         //Check if reset button has been pressed
void sunrise_sunset();                        //Calculate sunrise/sunset LED colours
void SpeakClock();                            //Speak the time based on local time
void touchsensor_check();                     //Check if sensor touched and action
void alarm_check();                           //Checks alarm setting against current local time
void update_epoch_time();                     //Updates the epock time by millis or by new NTP request
void API_check();                             //Check if it's time to get Sunrise/Set times
void LED_check();                             //check if it's time to update the LEDs
void test_LEDs();                             //Basic Red Blue Green Test
void show_chime_array();                      //Print the chime array, which Hours are on/off for chime sound.
void initiate_time();                         //Get NTP and time set up for the first time

//if using build flags for WiFi credentials
//platformio.ini
//build_flags =
//    -DSSID_NAME="SSID"
//    -DPASSWORD_NAME="password"
//    -DBLYNKCERT_NAME="1234567890"

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

//*************************
//*** Things to change  ***
//*************************

//LED details
#define NUM_LEDS_PER_STRIP 13 //Number of LEDs per strip
#define PIN_LED D7            //I.O pin on ESP2866 device going to LEDs
#define COLOR_ORDER GRB       // LED stips aren't all in the same RGB order.  If colours are wrong change this  e.g  RBG > GRB.   :RBG=TARDIS

//Flash at top of hour
int flash_delay = 60;    //delay between LED flashes (ms)
int flash_length = 2000; //for flash = 1-8, how many seconds per (e.g 1 = 2s)

//Max values for LEDs
const int green_max = 128; //green (128 for yellow / 255 for orange)
const int blue_max = 255;
const int red_max = 255;

//What lamp looks like for day time (sunshine)
const int green_day = 128; //green (128 for yellow / 255 for orange)
const int blue_day = 0;
const int red_day = 255;

//What lamp looks like for night
const int green_night = 0;
const int blue_night = 255;
const int red_night = 0;

//Lamp brightness
int howbright = 255; //0-255 LED Brightness level.  Ooverwritten by user input

//SPIFFS Filenames
String HTTPfilename = "APIaddress.txt"; //Filename for storing Sunrise API HTTP address in SPIFFS
String Modefilename = "Mode.txt";       //Filename for storing Sunrise Mode in SPIFFS (3 digits:  Mode, Top lamp, Flash)
String UTCfilename = "UTC.txt";         //Filename for storing Sunrise UTC in SPIFFS
String chimefilename = "chime.txt";     //Filename for storing Sunrise UTC in SPIFFS
String alarmfilename = "alarm.txt";     //Filename for storing Sunrise UTC in SPIFFS

//Lightmode, TARDIS, Longitude/Latitude and UTC are stated here but overwritten when webpage credentials are entered (if using WiFi Manager)
const char *NTPServerName = "0.nz.pool.ntp.org"; //local NTP server
int localUTC = 12;                               //Country UTC offset, needed for UTC for day/night calc  (+12 for NZ)  don't need to change for daylight saving as no needed for day/night
int UTCoffset = 0;                               //Set my user with touch button +1, -1, 0
int lightmode = 0;                               //0 = day/night (day = Yellow / night = Blue   e.g TARDIS Lamp)    1 = night light mode with sunrise/set colour changes (but off during daytime)    2 = night light mode without sunrise/set changes  (binary on (day) /off (night))
int TARDIS = 1;                                  //Used for my TARDIS lamp.  All LEDs work as per day/night lightmode, except 1 LED (last in strip) at the top of the TADIS which is forced Blue.

int NTPSecondstowait = 1 * 60 * 60; //Wait between NTP pulls (sec)
int APISecondstowait = 6 * 60 * 60; //Wait between Sunrise API pulls (sec)
int SecondsSinceLastAPI = 0;
int timefactor = 1; //Used for testing to accelerate time

const int LEDSecondstowait = 5; //Wait between LED updates (sec)
const int minswithin = 60;      //Minutes within sunrise / sunset to begin the LED colour change sequence  (60 = phase starts 30mins before sunrise/set and end 30mins after)
const int change = 1;           //Speed of LED change in tones.  Recommend = 1

int mp3vol = 0;                //Volume for DF card player.  Keep at 0, used as a flag to skip functions if not wifimanager credentials (no sound option)
int mp3_selected = 1;          //Default mp3 to play ("mp3/0001.mp3" on SDcard)
SoftwareSerial mySerial(4, 5); // Declare pin RX & TX pins for TF Sound module.  Using D1 (GPIO 5) and D2 (GPIO 4)

int touchthreshold = 300; //Min value from touch sensor to trigger
int touchstopmin = 300;   //Min touch to trigger a mp3_stop()
int touchtimemin = 1000;  //Min touch to trigger spoken clock_minutes
int touchalarmmin = 3000; //Max value to trigger spoken clock, greater than go to Alarm change
int touchUTC = 10000;     //this value or more to go into UTC change

int verbose_output = 0; //Flag to enable/disable printing on informations

//*************************
//*** Things to change  ***
//*************************

//Gets SSID/PASSWORD from platformio.ini build flags.
//Comment one or the other out in the following  (e.g Build flags vs Wifi Manager)
//const char ssid[] = xstr(SSID_NAME);          //gets ssid from build flags
const char ssid[] = ""; //define ssid or the ConnectAP function errors (even though not used when using WifiManager)
//const char pass[] = xstr(PASSWORD_NAME);      //gets pass from build flags
const char pass[] = ""; //define pass or the ConnectAP function errors (even though not used when using WifiManager)
//const char auth[] = xstr(BLYNKCERT_NAME);       // your BLYNK Cert from build flags

//Wifi and internet variables
const unsigned int localPort = 2390; // local port to listen for UDP packets
WiFiUDP udp;                         // A UDP instance to let us send and receive packets over UDP
IPAddress timeServer;

//Sunrise - Sunset API variables
char sunrise_api_request[100]; //It should end up containing an adress like this "http://api.sunrise-sunset.org/json?lat=-41.2865&lng=174.7762";
int h_sunrise, hour_sunrise, minute_sunrise, sunrise_minutes_from_midnight, local_sunrise_minutes_from_midnight;
int SR_Phase = 0; //1 = in Sunrise phase (30 mins either side if minwithin = 60mins)
int h_sunset, hour_sunset, minute_sunset, sunset_minutes_from_midnight, local_sunset_minutes_from_midnight;
int SS_Phase = 0;            //1 = in Sunset phase (30 mins either side if minwithin = 60mins)
int working_hourtomin = 0;   //Used to convert hours into total minutes
float LED_phase;             //0-255 in the phase of sunrise/set   0=begining 255=end
char SR_AMPM[1], SS_AMPM[1]; //Sunrise/set AMPM
String AMPM, sunAPIresponse;
struct CRGB leds[NUM_LEDS_PER_STRIP]; //initiate FastLED with number of LEDs
String JSON_Extract(String);
char mode[4];      //Used to get input from webpage
int SRSS_Flip = 0; //Used to manipulate SR and SS varible if in nightlight mode

//NTP and Time variables
char UTC[3];
int RequestedTime = 0, TimeCheckLoop = 0, NTPdelaycounter = 0;
int hour_actual = 200, dia_actual = 0, anyo = 0;
int timeout = 0, timeout_prev = 0;
int vera = 0, night = 0;                                                                                //1 = Night, 0 = Day
const int NTP_PACKET_SIZE = 48;                                                                         // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[NTP_PACKET_SIZE];                                                                     //buffer to hold incoming and outgoing packets
unsigned long epoch = 0, lastepoch = 0, Last_NTP_millis = 0, LastAPI, LastLED, epochstart, startmillis; //Unix time in seconds
int lastepochcount = 1, totalfailepoch = 0;
int hour, minute, second;                                           //UTC time
int clock_minutes_from_midnight, local_clock_minutes_from_midnight; //Minutes from midnight
int NTP_Seconds_to_wait = 1;                                        //Initial wait time between NTP/Sunrise pulls (1 sec)
String clock_AMPM;                                                  //AM/PM from NTP Server
int printNTP = 0;                                                   //Set to 1 when a NTP is pulled.  The decode_epoch function used for both NTP epoch and millis epoch.  printNTP=1 in this fucnction only print new NTP results (time).
int Seconds_SinceLast_NTP_millis;                                   //Counts seconds since last NTP pull
int retryNTP = 0;                                                   //Counts the number of times the NTP Server request has had to retry
int UTC_Cycle = 53;

//LED Flash variables
int flash_phase = 0;   //If x minutes within top of hour flash the LEDs
int flash = 0;         //flash = 0 (no flash)  flash = 1 (flash) set by user
int LastFlashmillis;   //Used for tracking delay
int flash_working = 0; //working var for sinearray (ranges 0-31)
int flash_start_millis;
int flash_phase_complete = 0; //Flag to ensure only 1 flash cycle per 0 minutes
int completed_flashes = 0;    //Counts the number of completed flashes if Flash=9 (flash the hour)

//LED Variables. Hold the value (0-255) of each primary colour
int green = 0;
int blue = 0;
int red = 0;

//Create an array with 0-255 sine wave with array 0-31
char sinetable[] = {127, 152, 176, 198, 217, 233, 245, 252, 254, 252, 245, 233, 217, 198, 176, 152, 128, 103, 79, 57, 38, 22, 38, 57, 79, 103};

//Array for the chime hours to go into.  24 digits starting with midnight.  0=no chime, 1=chime.  e.g 0000001111111111111111100
char chime[25];

//Specified alarm time by user, in format HHMMAM/HHMMPM
char alarm[6];
int alarmstate = 55;                          //55=Alarm ON 56=Alarm OFF  (0055.mp3 / 0056.mp3)
int alarm_local_minutes_from_midnight = 2000; //2000 minutes will never happen (e.g wont trigger - acceptable 0-1440)
int alarmdone = -1;                           //Flag to make sure alarm runs only once, -1 is an invalid time (mins from midnight) so won't run
int alarmmp3 = 0;                             //Which mp3 to play on alarm

//Touch sensors
long touchmillis;
long press_period;
const int cap1 = 2;  // D4 470k resistor between pins with 22pf cap in parallel
const int cap2 = 12; // D6 Capacitive Sensor
CapacitiveSensor csensy = CapacitiveSensor(cap1, cap2);
long touchsensor = csensy.capacitiveSensor(30);

int total_spoken_clocks = 0;

void setup()
{
  Serial.begin(9600);

  csensy.set_CS_AutocaL_Millis(0xFFFFFFFF); //Touch sensor Initialisation

  pinMode(0, INPUT); //Reset button initialisation.  GPIO0 (D3) to GND to reset ESP2866 Credentials

  FastLED.addLeds<WS2811, PIN_LED, COLOR_ORDER>(leds, NUM_LEDS_PER_STRIP); //Initialise the LEDs

  //DF Sound player setup
  mySerial.begin(9600);     //Initiate comms to TF Sound module
  mp3_set_serial(mySerial); //set softwareSerial for DFPlayer-mini mp3 module
  mp3_set_volume(mp3vol);   //Set default volume
  mp3_stop();               //soft-Reset module DFPlayer.  Make sure nothing is playing on start up

  WiFi_and_Credentials(); //Calls WiFi function to initiate.  either uses WifiManager to get Wifi and Longitude/Latitude data (And store API URL as SPIFFS file.)  // Can also use standard WiFi connection with build flags.
  test_LEDs();            //Red Blue Green test
  show_chime_array();     //Print the chime array, which Hours are on/off for chime sound.
  initiate_time();        //Get NTP and time set up for the first time
  //Blynk.begin(auth, ssid, pass);  //Blynk setup (if being used).

  //*** TESTING use only   .Set statement to true.  Overide setup from SPIFFs files
  verbose_output = 0; //*TESTING* Flag to enable/disable printing on informations

  if (1 == 0)
  {
    checkreset(0);          //Sending 1 = Clear SPIFFs  (0 = normal)
    howbright = 255;        //0-255 LED Brightness level
    flash = 1;              //Turn off flash if in testing mode
    NTPSecondstowait = 600; //Wait between NTP pulls (sec)
    APISecondstowait = 600; //Wait between Sunrise API pulls (sec)
    timefactor = 50;        //accellerate time by this factor
    lightmode = 1;          //overide lightmode
    TARDIS = 3;             //overide top light
    localUTC = 13;          //overide UTC
    sprintf(sunrise_api_request, "http://api.sunrise-sunset.org/json?lat=-41.2865&lng=174.7762");

    Serial.print("TEST sunrise_api_request = ");
    Serial.println(sunrise_api_request);
  }

  API_Request(); //Get sunrise/sunset times.  Do this after Test section in case API HTTP overridden

  if (verbose_output == 1)
  {
    Serial.print("millis = ");
    Serial.print(millis());
    Serial.print(",   start millis = ");
    Serial.println(startmillis);
    Serial.print("epochstart = ");
    Serial.print(epochstart);
    Serial.print(",   epoch = ");
    Serial.println(epoch);
  }
}

//Do the main execution loop
void loop()
{
  //Blynk.run();          //If Blynk being used
  checkreset(0);       //Has the GPIO (D3) been taken low to reset WiFiManager / clears SPIFFS?
  update_epoch_time(); //update epoch time by millis update ot NTP request
  decode_epoch(epoch); //epoch has been updated, Now turn this into UTC clock_minutes_from_midnight
  API_check();         //Check if it's time to get Sunrise/Set times and action
  LED_check();         //check if it's time to update the LEDs and action
  checkflash();        //Check if flash triggered and manipulate the brightness

  if (mp3vol > 0)
  {                      //Onyl check sensor and alarm if Volume set in Wifi manager (e.g only if sound option used)
    touchsensor_check(); //Check if the touchsensor has been triggered and action
    alarm_check();       //Check if the alarm has been triggered and action
  }
}

//Connect to the WiFi and manage credentials
void WiFi_and_Credentials()
{
  //2 ways to get WiFi.  ConnectToAP uses variables from build flags (platformio) or hard coded.
  //WiFiManager will check if WiFi credentials are known (in Flash memory).  If not it will stary a webserver to collect details from user

  //Use one of the follow lines depending on approach (build flags vs WiFiManager)
  //ConnectToAP();           //Connect to Wifi (if not using Wifi Manager approach)
  WiFiManager wifiManager;

  //SPIFFs section to Read and Write the saved credentials in Flash memory
  //Check if APIaddress.txt exists.  If not create it and store the http address for sunrise API, if yes read it.

  if (SPIFFS.begin())
  {
    Serial.println("SPIFFS Initialize....ok");
  }
  else
  {
    Serial.println("SPIFFS Initialization...failed");
  }

  if (SPIFFS.exists("\" & HTTPfilename") == true)
  {
    Serial.println("File already exisits.  Read stored data.");

    //Read HTTP addresss file data
    File f = SPIFFS.open("\" & HTTPfilename", "r");

    if (!f)
    {
      Serial.println("file open failed");
    }
    else
    {
      Serial.println("Reading Data from HTTP file:");
      //Data from file

      size_t size = f.size();

      f.readBytes(sunrise_api_request, size);

      f.close(); //Close file
      Serial.printf("Read http adress for Sunrise/set API = %s\n", sunrise_api_request);
      Serial.println("File Closed");

      //Read Mode file data
      File g = SPIFFS.open("\" & Modefilename", "r");

      if (!g)
      {
        Serial.println("file open failed");
      }
      else
      {
        Serial.println("Reading Data from Mode file:");
        //Data from file

        size_t size = g.size();

        g.readBytes(mode, size);

        g.close(); //Close file
        Serial.printf("Read Lamp Mode = %s\n", mode);
        Serial.println("File Closed");
      }

      //Read UTC file data
      File h = SPIFFS.open("\" & UTCfilename", "r");

      if (!h)
      {
        Serial.println("file open failed");
      }
      else
      {
        Serial.println("Reading Data from UTC file:");
        //Data from file

        size_t size = h.size();

        h.readBytes(UTC, size);

        h.close(); //Close file
        Serial.printf("Read UTC = %s\n", UTC);
        Serial.println("File Closed");
      }

      //Read chime file data
      File i = SPIFFS.open("\" & chimefilename", "r");

      if (!i)
      {
        Serial.println("file open failed");
      }
      else
      {
        Serial.println("Reading Data from chime file:");
        //Data from file

        size_t size = i.size();

        i.readBytes(chime, size);

        i.close(); //Close file
        Serial.printf("Read chime = %s\n", chime);
        Serial.println("File Closed");
      }

      //Read alarm file data
      File j = SPIFFS.open("\" & alarmfilename", "r");

      if (!j)
      {
        Serial.println("file open failed");
      }
      else
      {
        Serial.println("Reading Data from alarm file:");
        //Data from file

        size_t size = j.size();

        j.readBytes(alarm, size);

        j.close(); //Close file
        Serial.printf("Read alarm = %s\n", alarm);
        Serial.println("File Closed");
      }

      //WiFiManager will read stored WiFi Data, if it can't connect it will create a website to get new credentials.
      wifiManager.autoConnect("WiFi_Lamp");
    }
  }

  else
  {
    Serial.println("Filename DOESN'T exisit");

    //If file doesn't exist, get details from the user with wifimanager website
    //create http address and store in APIaddresst.txt file

    WiFiManagerParameter custom_longitude("Longitude", "Longitude", "", 10);
    WiFiManagerParameter custom_latitude("Latitude", "Latitude", "", 10);
    WiFiManagerParameter custom_mode("Mode", "Mode", "", 4);
    WiFiManagerParameter custom_UTC("UTC", "UTC", "", 3);
    WiFiManagerParameter custom_chime("chime", "chime", "", 25);
    WiFiManagerParameter custom_alarm("alarm", "alarm", "", 7);

    wifiManager.addParameter(&custom_longitude);
    wifiManager.addParameter(&custom_latitude);
    wifiManager.addParameter(&custom_mode);
    wifiManager.addParameter(&custom_UTC);
    wifiManager.addParameter(&custom_chime);
    wifiManager.addParameter(&custom_alarm);

    //WiFiManager will read stored WiFi Data, if it can't connect it will create a website to get new credentials
    wifiManager.autoConnect("WiFi_Lamp");

    //Check if new http address needed to be written to file.  If yes, create and write.
    //Example: sunrise_api_request = "http://api.sunrise-sunset.org/json?lat=-41.2865&lng=174.7762"
    //Mode of the Lamp (e.g 0, 1, 2) as separate files in SPIFFS
    sprintf(sunrise_api_request, "http://api.sunrise-sunset.org/json?lat=%s&lng=%s", +custom_latitude.getValue(), custom_longitude.getValue());
    sprintf(mode, "%s", +custom_mode.getValue());
    sprintf(UTC, "%s", +custom_UTC.getValue());
    sprintf(chime, "%s", +custom_chime.getValue());
    sprintf(alarm, "%s", +custom_alarm.getValue());

    //Create New HTTP File And Write Data to It
    //w=Write Open file for writing
    File f = SPIFFS.open("\" & HTTPfilename", "w");

    if (!f)
    {
      Serial.println("HTTP file open failed");
    }
    else
    {
      //Write data to file
      Serial.println("Writing Data to HTTP File");
      f.print(sunrise_api_request);
      Serial.println("New file written");
      f.close(); //Close file
    }

    //Create New Mode File And Write Data to It
    //w=Write Open file for writing
    File g = SPIFFS.open("\" & Modefilename", "w");

    if (!g)
    {
      Serial.println("Mode file open failed");
    }
    else
    {
      //Write data to file
      Serial.println("Writing Data to Mode File");
      g.print(mode);
      Serial.println("New file written");
      g.close(); //Close file
    }

    //Create New UTC File And Write Data to It
    //w=Write Open file for writing
    File h = SPIFFS.open("\" & UTCfilename", "w");

    if (!h)
    {
      Serial.println("UTC file open failed");
    }
    else
    {
      //Write data to file
      Serial.println("Writing Data to UTC File");
      h.print(UTC);
      Serial.println("New file written");
      h.close(); //Close file
    }

    //Create New chime File And Write Data to It
    //w=Write Open file for writing
    File i = SPIFFS.open("\" & chimefilename", "w");

    if (!i)
    {
      Serial.println("chime file open failed");
    }
    else
    {
      //Write data to file
      Serial.println("Writing Data to chime File");
      i.print(chime);
      Serial.println("New file written");
      i.close(); //Close file
    }

    //Create New alarm File And Write Data to It
    //w=Write Open file for writing
    File j = SPIFFS.open("\" & alarmfilename", "w");

    if (!j)
    {
      Serial.println("alarm file open failed");
    }
    else
    {
      //Write data to file
      Serial.println("Writing Data to alarm File");
      j.print(alarm);
      Serial.println("New file written");
      j.close(); //Close file
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
  if (lightmode < 0 || lightmode > 2)
  {
    Serial.println("Light mode incorrect - overriding");
    lightmode = 0;
  }
  Serial.print("Light mode used = ");
  Serial.println(lightmode);

  //Check Top light entry is valid
  if (TARDIS < 0 || TARDIS > 4)
  {
    Serial.println("Top light incorrect - overriding");
    TARDIS = 3;
  }
  Serial.print("Top light used = ");
  Serial.println(TARDIS);

  //Check flash mode is valid.
  if (flash < 0 || flash > 9)
  {
    Serial.println("Flash mode incorrect - overriding");
    flash = 0;
  }

  Serial.print("Flash mode used = ");
  Serial.println(flash);

  //Check brightness is valid.
  if (howbright_temp < 1 || howbright_temp > 5)
  {
    Serial.println("Brightness incorrect - overriding");
    howbright_temp = 5;
  }
  Serial.print("Brightness (1-5) = ");
  Serial.println(howbright_temp);

  //Entry x 5 +5 gives range of 55-255
  howbright = (howbright_temp * 50) + 5;

  //Get volume from alarm array
  char buffer0[1];
  buffer0[0] = chime[0];
  mp3vol = atoi(buffer0);

  //Set the mp3 volume form the chime array, this is used as a flag to run/no run sound functions in loop
  if (mp3vol < 0 || mp3vol > 9)
  {
    mp3vol = 0; //default to 0 in error state - no sound
    Serial.println("mp3 volume incorrect - overriding");
  }
  else
  {
    mp3vol = mp3vol * 3.3; //30 is max volume, 9 x 3.3 = 29.7

    if (mp3vol >= 29)
    { //29 is close enough to 30, make it 30
      mp3vol = 30;
    }
  }

  mp3_set_volume(mp3vol); //Set the mp3 volume
  Serial.print("mp3 Volume = ");
  Serial.println(mp3vol);

  //Get UTC from saved file and turn into an Int and check it's between 0-24
  char buffer4[3];
  buffer4[0] = UTC[0];
  buffer4[1] = UTC[1];
  buffer4[2] = UTC[2];
  localUTC = atoi(buffer4);

  if (localUTC < -13 || localUTC > 13)
  {
    Serial.println("UTC mode incorrect - overriding");
    localUTC = 12; //Default to NZ (winter)
  }
  Serial.print("UTC used = ");
  Serial.println(localUTC);

  //Get Alarm from saved file and turn into an mins from midnight > alarm_local_minutes_from_midnight
  char buffer5[2];

  buffer5[0] = alarm[0]; //H
  buffer5[1] = alarm[1]; //H
  int alarmhour = atoi(buffer5);

  buffer5[0] = alarm[2]; //M
  buffer5[1] = alarm[3]; //M
  int alarmminute = atoi(buffer5);

  buffer5[0] = alarm[6]; //1-8 (mp3)
  buffer5[1] = NULL;
  alarmmp3 = atoi(buffer5);

  buffer5[0] = alarm[4]; //A/P
  buffer5[1] = NULL;

  alarm_local_minutes_from_midnight = (alarmhour * 60) + (alarmminute);

  if ((0 == strcmp(buffer5, "p") || 0 == strcmp(buffer5, "P")) && (alarmhour >= 1 && alarmhour <= 11))
  {
    alarm_local_minutes_from_midnight += (12 * 60);
  }

  //Check and override alarm time
  if (alarmhour < 0 || alarmhour > 12 || alarmminute < 0 || alarmminute > 59)
  {
    alarm_local_minutes_from_midnight = 0;
    alarmhour = 0;
    alarmminute = 0;
    alarmmp3 = 0; //No sound
    Serial.println("Error in alarm time - overriding - nothing plays");
  }

  Serial.print("alarm_local_minutes_from_midnight = ");
  Serial.print(alarm_local_minutes_from_midnight);

  Serial.print(",  Alarm hour = ");
  Serial.print(alarmhour);

  Serial.print(",  Alarm minute = ");
  Serial.print(alarmminute);

  Serial.print(",  Alarm mp3 = ");
  Serial.println(alarmmp3);

  if (alarmmp3 <= 0 || alarmmp3 >= 9)
  {
    alarmmp3 = 1;
  }

  //Test_alarm
  //alarm_local_minutes_from_midnight = 1030;
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
  if (verbose_output == 1)
  {
    Serial.print("UTC Hour: ");
    Serial.print(hour);
    Serial.print(",   Minute: ");
    Serial.print(minute);
    Serial.print(",   Second: ");
    Serial.print(second);
    Serial.print(",   clock_AMPM: ");
    Serial.println(clock_AMPM);
    Serial.println();
  }
  //Check for sunrise.  clock_minutes_from_midnight is time in minutes from midnight.  Sunrise/set minutes and clock are both UTC
  //Only compare UTC with UTC as local time (using UTC offset can change with daylight savings).  Local only for figuring out if it's night or day

  //Corrected Sunrise/Set and time variables
  int sunrise_minutes_from_midnight_corrected = sunrise_minutes_from_midnight;
  int sunset_minutes_from_midnight_corrected = sunset_minutes_from_midnight;
  int clock_minutes_from_midnight_corrected = clock_minutes_from_midnight;

  //e.g SR 0020 means 30mins before and 30 after would be 1430:0050.  Different timelines are difficult to compare.  Make 0020 = 1460 (1440 + 0020) then 30mins before/after:  1430:1490
  //Need to correct time
  if (sunrise_minutes_from_midnight < (minswithin / 2))
  {
    sunrise_minutes_from_midnight_corrected = sunrise_minutes_from_midnight + 1440;

    //if Sunrise corrected then correct time is also in the same way
    if (clock_minutes_from_midnight < (minswithin / 2))
    {
      clock_minutes_from_midnight_corrected = clock_minutes_from_midnight + 1440;

      Serial.println("*sunrise period spans midnight UTC - corrected");
    }
  }

  if (sunset_minutes_from_midnight < (minswithin / 2))
  {
    sunset_minutes_from_midnight_corrected = sunset_minutes_from_midnight + 1440;

    //if Sunrise corrected then correct time is also in the same way
    if (clock_minutes_from_midnight < (minswithin / 2))
    {
      clock_minutes_from_midnight_corrected = clock_minutes_from_midnight + 1440;

      Serial.println("*sunset period spans midnight UTC - corrected");
    }
  }

  //Check for Sunrise phase
  if (clock_minutes_from_midnight_corrected >= (sunrise_minutes_from_midnight_corrected - (minswithin / 2)) && clock_minutes_from_midnight_corrected <= (sunrise_minutes_from_midnight_corrected + (minswithin / 2)))
  {
    SR_Phase = 1;
    LED_phase = ((clock_minutes_from_midnight_corrected - sunrise_minutes_from_midnight_corrected) + (minswithin / 2)) / (float)minswithin * 255;
  }
  else
  {
    SR_Phase = 0;
  }

  //Check for sunset.  clock_minutes_from_midnight is time in minutes from midnight.  Sunrise/set minutes is LOCAL time from API
  if (clock_minutes_from_midnight_corrected >= (sunset_minutes_from_midnight_corrected - (minswithin / 2)) && clock_minutes_from_midnight_corrected <= (sunset_minutes_from_midnight_corrected + (minswithin / 2)))
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

  if (verbose_output == 1)
  {

    Serial.print("clock_minutes_from_midnight (UTC) = ");
    Serial.print(clock_minutes_from_midnight);
    Serial.print(",   local_clock_minutes_from_midnight = ");
    Serial.println(local_clock_minutes_from_midnight);
    Serial.print("sunrise minutes_from_midnight (UTC) = ");
    Serial.print(sunrise_minutes_from_midnight);
    Serial.print(",   sunset_minutes_from_midnight (UTC) = ");
    Serial.println(sunset_minutes_from_midnight);
    Serial.print("Seconds_SinceLast_NTP_millis: ");
    Serial.print(Seconds_SinceLast_NTP_millis);
    Serial.print(",   Startmillis: ");
    Serial.print(startmillis);
    Serial.print(",   epochstart: ");
    Serial.print("SecondsSinceLastAPI: ");
    Serial.print(SecondsSinceLastAPI);
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
    Serial.print(sunrise_minutes_from_midnight - clock_minutes_from_midnight - int(minswithin / 2));
    Serial.print(", retryNTP = ");
    Serial.println(retryNTP);
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
  if (TARDIS == 1)
  {
    leds[NUM_LEDS_PER_STRIP - 1].setRGB(255, 0, 0); //Light on top of TARDIS Red
  }

  if (TARDIS == 2)
  {
    leds[NUM_LEDS_PER_STRIP - 1].setRGB(0, 255, 0); //Light on top of TARDIS Green
  }

  if (TARDIS == 3)
  {
    leds[NUM_LEDS_PER_STRIP - 1].setRGB(0, 0, 255); //Light on top of TARDIS Blue
  }

  if (TARDIS == 4)
  {
    leds[NUM_LEDS_PER_STRIP - 1].setRGB(255, 255, 255); //Light on top of TARDIS White
  }

  FastLED.setBrightness(howbright);
  FastLED.show();

  if (verbose_output == 1)
  {
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
  SRSS_Flip = 0; //0=Normal sunrise/sunset, 1=Inverted sunrise/sunset colour change order (only used in nightlight mode 1 & 2)
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
  SRSS_Flip = 1;    //1=Inverted sunrise/sunset colour change order
  sunrise_sunset(); //calculate LEDs

  //This section overrides the LED colours calculated from Sunrise/set phase.

  //Mode 0:  This nightlight function should not be called in daynight mode (mode 0)
  //Mode 1
  if (SR_Phase == 0 && SS_Phase == 0 && lightmode == 1 && night == 1) //Not in Sunrise/set phase, I'm in mode 1 and it's night time - LEDs yellow
  {
    blue = blue_day;
    green = green_day;
    red = red_day;
  }

  if (SR_Phase == 0 && SS_Phase == 0 && lightmode == 1 && night == 0) //Not in Sunrise/set phase, I'm in mode 1 and it's daytime - LEDs off
  {
    blue = 0;
    green = 0;
    red = 0;
  }

  //Mode 2
  if (lightmode == 2 && night == 0) //Don't care about in Sunrise/set phase, I'm in mode 2 and it's daytime - LEDs off
  {
    blue = 0;
    green = 0;
    red = 0;
  }

  if (lightmode == 2 && night == 1) //Don't care about Sunrise/set phase, I'm in mode 2 and it's night time - LEDs yellow
  {
    blue = blue_day;
    green = green_day;
    red = red_day;
  }
}

//Calculate sunrise/sunset LED colours
void sunrise_sunset()
{

  //This piece flips SR and SS phases if requested by use of SRSS_Flip (for nightlight)
  int SR_Phase_use;
  int SS_Phase_use;

  if (SR_Phase == 1 && SRSS_Flip == 0)
  {
    SR_Phase_use = 1;
    SS_Phase_use = 0;
  }

  if (SR_Phase == 1 && SRSS_Flip == 1)
  {
    SR_Phase_use = 0;
    SS_Phase_use = 1;
  }

  if (SS_Phase == 1 && SRSS_Flip == 0)
  {
    SR_Phase_use = 0;
    SS_Phase_use = 1;
  }

  if (SS_Phase == 1 && SRSS_Flip == 1)
  {
    SR_Phase_use = 1;
    SS_Phase_use = 0;
  }

  if (verbose_output == 1)
  {
    Serial.print("SR_Phase_use = ");
    Serial.print(SR_Phase_use);
    Serial.print(",   SS_Phase_use = ");
    Serial.println(SS_Phase_use);
    Serial.println();
  }

  //sunrise:  Start with blue reducing to zero, and red increasing, when blue 0 increase green
  if (SR_Phase_use == 1 && blue >= 0)
  {
    //in first part of sunrise (reduce blue / increase red)
    blue = blue_max - (LED_phase * 2); //LED Phase 0-128,  *2 as in 1st phase we need 0-255 of LED movement for blue & red
    red = 0 + (LED_phase * 2);

    if (blue < 0)
    {
      blue = 0;
    }
    if (red > red_max)
    {
      red = red_max;
    }
    green = 0; //force in case lamp started during phase
  }

  if (SR_Phase_use == 1 && blue == 0)
  {
    //in second part of sunrise (increase green)
    green = 0 + abs(LED_phase - 128); //LED Phase 128-255, need to be 0 to start green LED movement (0-128)

    if (green > green_max)
    {
      green = green_max;
    }
  }

  //sunset:  Start with green reducing to zero, then reducing red to 0 and increasing blue
  if (SS_Phase_use == 1 && green > 0)
  {
    green = green_max - (LED_phase); //LED Phase 0-128, reduce green from 128 > 0

    if (green < 0)
    {
      green = 0;
    }

    blue = 0; //force in case lamp started during phase
  }

  if (SS_Phase_use == 1 && green == 0)
  {
    //in second part of sunset increase blue and rdecrease red
    blue = 0 + (abs(LED_phase - 128) * 2);      //LED Phase 128-255, subtract 128 (gives range 0-128) then x2 (range 0-255)
    red = red_max - (abs(LED_phase - 128) * 2); //LED Phase 128-255, subtract 128 (gives range 0-128) then x2 (range 0-255)

    if (blue > blue_max)
    {
      blue = blue_max;
    }

    if (red < 0)
    {
      red = 0;
    }
  }
}

void touchsensor_check()
{
  touchsensor = csensy.capacitiveSensor(30);

  if (touchsensor > 0)
  {
    touchmillis = millis();
    //delay(200);
    touchsensor = csensy.capacitiveSensor(30);

    // Serial.print(touchsensor);
    // Serial.print(" - ");
    // Serial.println(UTC_Cycle);

    if (touchsensor > touchthreshold)
    {
      //How long has it been held down
      while (csensy.capacitiveSensor(30) > touchthreshold)
      {
        if (millis() - touchmillis > touchUTC)
        { //More than 10s then escape the while
          break;
        }
        yield(); //Tight loop, yield to avoid WTC crashes
      }

      press_period = millis() - touchmillis;

      //Between min time and the spoken time min
      if (press_period >= touchstopmin && press_period < touchtimemin)
      {
        mp3_stop();
      }

      //Between spoken time min and the alarm min
      if (press_period >= touchtimemin && press_period <= touchalarmmin)
      {
        SpeakClock();
      }

      if (press_period > touchalarmmin && press_period < touchUTC)
      {
        if (alarmstate == 55)
        {
          alarmstate = 56;
          Serial.println();
          Serial.println("****************");
          Serial.println("Alarm OFF");
          Serial.println("****************");
          Serial.println();
          mp3_play(alarmstate);
        }
        else
        {
          alarmstate = 55;
          Serial.println("****************");
          Serial.println("Alarm ON");
          Serial.println("****************");
          Serial.println();
          mp3_play(alarmstate);
        }
      }

      //Greater than UTC min
      if (press_period >= touchUTC)
      {
        UTC_Cycle++;

        if (UTC_Cycle > 54)
        {
          UTC_Cycle = 52;
        }

        if (UTC_Cycle == 52)
        {
          UTCoffset = -1;
        }

        if (UTC_Cycle == 53)
        {
          UTCoffset = 0;
        }

        if (UTC_Cycle == 54)
        {
          UTCoffset = 1;
        }

        mp3_play(UTC_Cycle); //Play selected mp3 in folder mp3
        delay(1000);
      }
    }
  }
}

void alarm_check()
{

  if (local_clock_minutes_from_midnight == alarm_local_minutes_from_midnight && alarmdone != local_clock_minutes_from_midnight)
  {
    alarmdone = alarm_local_minutes_from_midnight; //set flag so if statement above fails (plays once)

    Serial.println();
    Serial.println("****************");
    Serial.print("Alarm!");

    if (alarmstate == 55)
    {
      Serial.print(", Alarm ON,  mp3= ");
      Serial.println(alarmmp3);
      mp3_play(alarmmp3);
    }
    else
    {
      Serial.println(", Alarm OFF, No sound");
    }

    Serial.println("****************");
    Serial.println();
  }

  //Reset alarmdone to -1 (to enable alarm to play again)
  if (local_clock_minutes_from_midnight != alarm_local_minutes_from_midnight)
  {
    alarmdone = -1;
  }
}

//Check if flash is required and manipulate brightness
void checkflash()
{
  //if flash >= 1 (enabled) and minutes = 0 (top of hour) and LEDs are on (e.g not off during day) then set flash_phase = 1
  if (minute == 0 && flash >= 1 && flash_phase_complete == 0 && (red + blue + green) > 0)
  {
    flash_phase = 1;

    //if flash_start_millis = 0 then it's the first time, set start millis
    if (flash_start_millis == 0)
    {
      flash_start_millis = millis();

      Serial.println("Start flash phase...");

      int local_hour = local_clock_minutes_from_midnight / 60; //Turn mines into the hour, needed for chime check
      int chimehour;
      char buffer8[0]; //Buffer from chime hour

      //Get chime on/off for hour
      buffer8[0] = chime[local_hour + 1];
      chimehour = atoi(buffer8);

      //Check for chime
      Serial.print("local clock_minutes from midnight = ");
      Serial.print(local_clock_minutes_from_midnight);
      Serial.print(",  Local hour = ");
      Serial.print(local_hour);
      Serial.print(",  chime array for hour = ");
      Serial.print(chimehour);

      if (chimehour > 0)
      {
        Serial.print(",  chime ON, Volume = "); //chime is on, the chime array didn't have 0 for this hour
        Serial.print(mp3vol);
        Serial.print(", ");

        if (flash == 9)
        {
          Serial.println(" Flashing hours");
        }
        else
        {
          Serial.println(" Flashing defined seconds");
        }
        Serial.print("Completed flashes: ");

        if (chimehour >= 1 && chimehour <= 8)
        {
          mp3_play(chimehour); //Play selected mp3 in folder mp3  (e.g if the chime sequence is 1-8, play 0001.mp3 > 0008.mp3 )
        }

        if (chimehour == 9)
        { //if chime sequence = 9 then don't play 0009.mp3, speak the clock time instead
          SpeakClock();
        }
      }
      else
      {
        Serial.print(",  No chime, "); //The chime array had 0, no chime for this hour
        if (flash == 9)
        {
          Serial.println(" Flashing hours");
        }
        else
        {
          Serial.println(" Flashing defined seconds");
        }
        Serial.print("Completed flashes: ");
      }
    }
  }

  //Check for flash_start_miillis + flash_end_millis, if exceeded then flash_phase = 0 (off)
  //flash ranges from 1-8, x2000 for 2-16 seconds or if flash = 9  then flash the hour.
  if (flash_phase == 1 && flash <= 8 && millis() > (flash_start_millis + (flash * flash_length)))
  {

    flash_phase = 0;          //turn off flash_phase
    flash_start_millis = 0;   //reset start_millis
    flash_phase_complete = 1; //Flag to confirm a completed a flash cycle
    completed_flashes = 0;    //reset how many flashes flag
    //mp3_stop();               //Stop the sounds (if they are playing)

    //Stop mp3 with flashing use this, else comment out and let the mp3 play until finished
    //mp3_stop();               //Stop the sounds (if they are playing)

    Serial.println("End flash phase...");
    Serial.println();
    Serial.println("****************");
  }

  //Check for flash_start_miillis + flash_end_millis, if exceeded then flash_phase = 0 (off)
  //flash ranges from 1-6, x10,000 for 10-60 seconds or if flash = 9  then flash the hour.

  //Convert minutes from midnight to hours, and make 1-12
  //Local_hour2 is used for the number of flashes, e.g 13:00  > 1:00
  int local_hour2 = local_clock_minutes_from_midnight / 60;
  if (local_hour2 > 12)
  {
    local_hour2 = local_hour2 - 12;
  }

  //if 0 (midnight, then make 12)
  if (local_hour2 == 0)
  {
    local_hour2 = 12;
  }

  if (flash_phase == 1 && flash == 9 && completed_flashes == local_hour2)
  { //Flash==9 means flash the number of hours

    flash_phase = 0;          //turn off flash_phase
    flash_start_millis = 0;   //reset start_millis
    flash_phase_complete = 1; //Flag to confirm a completed a flash cycle
    completed_flashes = 0;    //reset how many flashes flag
    //mp3_stop();               //Stop the sounds (if they are playing)

    Serial.println();
    Serial.println("End flash phase...");
    Serial.println();
    Serial.println("****************");
  }

  //Use a flag flash_phase_complete which is only set back to 0 when in minute 30.  Using 30 in case in demo mode (time factor=big number)
  //This stops more than 1 flash cycle during a minute=0 scenario
  if (minute >= 30)
  {
    flash_phase_complete = 0; //Reset flag to allow new flash cycle.
  }

  //If in flash mode then do the flash routine
  if (flash_phase == 1)
  {

    //Check if the required time has passed to flash
    if (millis() - LastFlashmillis >= flash_delay)
    {
      LastFlashmillis = millis();

      FastLED.setBrightness(howbright * sinetable[flash_working]);
      FastLED.show();

      flash_working = flash_working + 1;
      //return to begining of sequence
      if (flash_working == 26)
      {
        flash_working = 0;
        completed_flashes = completed_flashes + 1;
        Serial.print(completed_flashes);
        Serial.print(", ");
      }
    }
  }
}

//Check if reset button pressed.  D3 / GPIO0 held to ground.
void checkreset(int ClearSPIFFS)
{
  if (digitalRead(0) == 0 || ClearSPIFFS == 1)
  {
    delay(500); //500ms for button bounce
    if (digitalRead(0) == 0 || ClearSPIFFS == 1)
    {

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
      SPIFFS.remove("\" & chimefilename");
      SPIFFS.remove("\" & alarmfilename");
      SPIFFS.format();
      WiFi.disconnect();

      delay(2500);
      ESP.restart();
    }
  }
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

void SpeakClock()
{

  int hour_mp3 = 0;
  int minute_mp3 = 0;
  int minute_mp3b = 999;
  int AMPMmp3 = 50;
  int local_hour = local_clock_minutes_from_midnight / 60; //Turn minutes into the hour, needed for chime check

  //Set to PM is 12pm or later
  if (local_hour >= 12)
  {
    AMPMmp3 = 51;
  }

  //Convert 24hr into 12hr clock
  if (local_hour > 12)
  {
    local_hour -= 12;
  }

  //Hours mp3.  Ranges from 30 (00 midnight) to 42 (Twelve)
  hour_mp3 = 30 + local_hour;

  //Minute mp3.  Specific words for 00 (OClock), 01, 02, 03, 04, 05, 06, 07, 08, 09, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 30, 40, 50
  if (minute <= 19)
  {
    minute_mp3 = 100 + minute;
  }
  else
  {
    if (minute % 10 == 0) //If it's MOD 0, this means it's at the top of the hour (O'Clock)
    {
      minute_mp3 = 100 + minute;
    }
    else
    {
      minute_mp3 = 100 + (minute - (minute % 10));
      minute_mp3b = 30 + (minute % 10);
    }
  }

  Serial.println();
  Serial.println("****************");
  Serial.print("Local time: ");
  Serial.print(local_hour);
  Serial.print(":");
  Serial.println(minute);
  Serial.print("Speaking mp3  Hour: ");
  Serial.print(hour_mp3);
  mp3_play(hour_mp3); //Play selected mp3 in folder mp3
  delay(1000);
  Serial.print(",  Minute: ");
  Serial.print(minute_mp3);
  mp3_play(minute_mp3); //Play selected mp3 in folder mp3
  delay(1000);
  Serial.print(" / ");
  Serial.print(minute_mp3b);
  Serial.print(" AMPM: ");

  if (minute_mp3b != 999)
  {
    mp3_play(minute_mp3b);
    delay(1000);
  }

  Serial.print(AMPMmp3);
  total_spoken_clocks++;
  Serial.print(",   Total spoken clocks = ");
  Serial.println(total_spoken_clocks);

  Serial.println("****************");
  Serial.println();
  mp3_play(AMPMmp3); //Play selected mp3 in folder mp3

  delay(1000);
  Serial.println();
}

void API_check()
{
  //Check if it's time to get Sunrise/Set times
  SecondsSinceLastAPI = (millis() - LastAPI) / 1000;              //How many seconds since Last API pull
  if (SecondsSinceLastAPI > APISecondstowait && flash_phase == 0) //Don't go to sunride API during flash phase as it causes flicker)
  {
    LastAPI = millis();
    API_Request(); //get sunrise/sunset data
    yield();
  }
}

void LED_check()
{
  //Check if it's time to display LEDs
  int SecondsSinceLastLED = (millis() - LastLED) / 1000; //How many seconds since Last LED update
  if (SecondsSinceLastLED > LEDSecondstowait)
  {
    LastLED = millis();

    //Set the LED colours based on the Time and the Sun position.  Don't update LED colours when in flash mode (causes flicker)
    if (flash_phase == 0)
    {
      DoTheLEDs();
    }

    yield();
  }
}

void test_LEDs()
{
  //Test the LEDs in RGB order
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
}

void show_chime_array()
{
  //Print the chime array, which Hours are on/off for chime sound.
  int chimehour;
  char buffer7[0];

  buffer7[0] = chime[0];
  chimehour = atoi(buffer7);
  Serial.print("Vol:");
  Serial.print(atoi(buffer7));
  Serial.print(", ");

  for (int x = 1; x <= 24; x++)
  {
    buffer7[0] = chime[x];
    chimehour = atoi(buffer7);

    Serial.print(x - 1);
    Serial.print(":");
    Serial.print(chimehour);

    if (chimehour > 9)
    {
      Serial.print(" correcting to 0 ");
      chime[x] = 0;
    }

    if (x == 24)
    {
      Serial.println();
      Serial.println();
    }
    else
    {
      Serial.print(", ");
    }
  }
}

//***********************************************************
//***          All the time functions below here          ***
//***********************************************************

//Get time from NTP Server
void Request_Time()
{
  Serial.println("Getting Time");
  WiFi.hostByName(NTPServerName, timeServer);
  sendNTPpacket(timeServer); // send an NTP packet to a time server
}

//This returns a bool value based on UDP time being received and placed in epoch variable
bool Check_Time()
{
  bool ret_val = false;

  // //Test only:  Simulate NTP packets not Rxd
  // srand (millis());

  // int blah=rand() % 100;
  // Serial.print("RAND = ");
  // Serial.println(blah);

  // if (blah >= 80){
  //   return ret_val;
  // }

  //Clear the buffer, keep looping and pulling packets until no more NTP packets are available.
  //Put here in scenario that if multiple retires to NTP then multiple packets eventually Rxd, this clears them all and takes the latest
  for (int cb = udp.parsePacket(); cb > 0; cb = udp.parsePacket())
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

    //Add seconds to factor for delay(2000)
    epoch = epoch + (2 * NTPdelaycounter);

    //Check if there are lost packets (epoch is wildly different from last time).  If yes, use last epoch
    //if first epoch time is wrong this is constantly fail.

    Serial.print("secsSince1900: ");
    Serial.print(secsSince1900);
    Serial.print(",  epoch: ");
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
        Serial.println(",  Making internal clock = new NTP time");
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
    Serial.println(lastepoch);

    Last_NTP_millis = millis(); //Set the last millis time the NTP time was attempted
    RequestedTime = 0;
    TimeCheckLoop = 0;

    ret_val = true;
  }

  if (!ret_val)
  {
    Serial.println("no packet yet");
  }

  return ret_val;
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

//Update the time
void decode_epoch(unsigned long currentTime)
{
  // print the raw epoch time from NTP server
  if (printNTP == 1 && verbose_output == 1)
  {
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

  clock_AMPM = "AM"; //Default to AM

  //If it's 12 or greater (e.g 12 > 23) go to PM
  if (hour >= 12)
  {
    clock_AMPM = "PM";
  }

  //If it's greater than 12 (e.g 13 > 23) deduct 12 to make 1 > 11
  if (hour > 12)
  {
    hour = hour - 12;
  }

  if (printNTP == 1 && verbose_output == 1)
  {
    Serial.print("UTC Hour: ");
    Serial.print(hour);
    Serial.print(",   Minute: ");
    Serial.print(minute);
    Serial.print(",   Second: ");
    Serial.print(second);
    Serial.print(",   clock_AMPM: ");
    Serial.println(clock_AMPM);
    Serial.println();
  }

  //Work out Hours/min into minutes from midnight to Calculate if it's AM or PM time
  working_hourtomin = hour;

  //PM add 12.  e,g 1PM = 13:00
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
  local_clock_minutes_from_midnight = clock_minutes_from_midnight + ((localUTC + UTCoffset) * 60);

  //If local_minutes is negative (e.g Negative UTC)
  if (local_clock_minutes_from_midnight > 1440)
  {
    local_clock_minutes_from_midnight = local_clock_minutes_from_midnight - 1440;
  }

  //If local_minutes is negative (e.g Negative UTC)
  if (local_clock_minutes_from_midnight < 0)
  {
    local_clock_minutes_from_midnight = local_clock_minutes_from_midnight + 1440;
  }

  if (printNTP == 1 && verbose_output == 1)
  {
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
  local_sunrise_minutes_from_midnight = sunrise_minutes_from_midnight + ((localUTC + UTCoffset) * 60);

  //If local_minutes is greater than 1 day (e.g large postive UTC)
  if (local_sunrise_minutes_from_midnight > 1440)
  {
    local_sunrise_minutes_from_midnight = local_sunrise_minutes_from_midnight - 1440;
  }

  //If local_minutes is negative (e.g Negative UTC)
  if (local_sunrise_minutes_from_midnight < 0)
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
    working_hourtomin = 12;
    Serial.println();
  }

  sunset_minutes_from_midnight = ((working_hourtomin * 60) + minute_sunset);

  //Convert UTC sunrise_minutes_from_midnight into local_sunrise_minutes_from_midnight with UTC
  local_sunset_minutes_from_midnight = sunset_minutes_from_midnight + ((localUTC + UTCoffset) * 60);

  //If local_minutes is greater than 1 day (e.g large postive UTC)
  if (local_sunset_minutes_from_midnight > 1440)
  {
    local_sunset_minutes_from_midnight = local_sunset_minutes_from_midnight - 1440;
  }

  //If local_minutes is negative (e.g Negative UTC)
  if (local_sunset_minutes_from_midnight < 0)
  {
    local_sunset_minutes_from_midnight = local_sunset_minutes_from_midnight + 1440;
  }

  if (verbose_output == 1 && printNTP == 1)
  {
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

void update_epoch_time()
{

  epoch = epochstart + (((millis() - startmillis) / 1000) * timefactor); //Get epoch from millis count.  May get over writtem by NTP pull.  timefactor is for testing to accellerate time for testing
  printNTP = 0;                                                          //Flag to state time was not from an NTP request.

  //Update the time.  NTP pull is only done periodically based on NTP_Seconds_to_wait, we count millis (pretty accurate) when not getting NTP time
  Seconds_SinceLast_NTP_millis = (millis() - Last_NTP_millis) / 1000; //How many seconds since Last_NTP_millis pull

  if (Seconds_SinceLast_NTP_millis > NTP_Seconds_to_wait && flash_phase == 0) //Don't go to NTP during flash phase as it causes flicker
  {
    if (verbose_output == 1)
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
    }

    Request_Time(); //Get the timedata
    printNTP = 1;   //1 is a flag to serialprint the time (only used for NTP pull not for millis updates)
    delay(2000);
    NTPdelaycounter++; //Count how many delay functions after request time

    while (!Check_Time()) //Converts to Epoch, returns a False if not data Rxd
    {                     //If no time recieved then do this
      delay(2000);
      Serial.println("No packets, NTP Wait...");
      NTPdelaycounter++; //+1 for another delay
      TimeCheckLoop++;

      //If after 5 tries, Give up and exit the NTP function, reset the loop counter.  epoch already updated from Millis()
      if (TimeCheckLoop >= 5)
      {
        TimeCheckLoop = 0;
        NTPdelaycounter = 0; //Reset counter on exit
        break;
      }
      else if (TimeCheckLoop > 2)
      {
        //If after 2 tried then try re-requesting the time
        retryNTP += 1; //Update the counter for informational only, not used in the program
        Request_Time();
        TimeCheckLoop = 0; //Reset the counter back to 0 after a request (we only use the latest packets from requests)
      }
    }

    NTPdelaycounter = 0; //Time recieved, reset counter

    //Time confirmed received and more than wait period to pull NTP / Sunrise time
    Last_NTP_millis = millis(); //Set the Last_NTP_millis time to now - resets the wait time

    Serial.println();
    Serial.println("****************");
    Serial.println();

    yield();
    NTP_Seconds_to_wait = NTPSecondstowait; //Over write the initial wait period (1 sec) to the ongoing period (e.g 600 sec)
  }
}

void initiate_time()
{
  //Initiate time
  udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(udp.localPort());

  NTPdelaycounter = 0; //Reset counter on exit
  Request_Time();      //Get the time
  delay(2000);
  NTPdelaycounter++; //Add 1 to delay counter

  while (!Check_Time())
  { //If no time recieved then do this
    delay(2000);
    TimeCheckLoop++;
    NTPdelaycounter++;

    if (TimeCheckLoop > 5)
    {                      //If not time received even after 5x 2sec delays, then try re-getting time
      Request_Time();      //Get the time
      NTPdelaycounter = 0; //Reset delay counter on new request
    }
  }

  NTPdelaycounter = 0;    //Reset delay counter on exit
  epochstart = epoch;     //epoch pulled from NTP server, use initial epoch to set starting point for epochmillis
  startmillis = millis(); //get starting point for millis
}