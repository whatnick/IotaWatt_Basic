/*
 *  This sketch sends ads1115 current sensor data via HTTP POST request to thingspeak server.
 *  It needs the following libraries to work (besides the esp8266 standard libraries supplied with the IDE):
 *
 *  - https://github.com/adafruit/Adafruit_ADS1X15
 *
 *  designed to run directly on esp8266-01 module, to where it can be uploaded using this marvelous piece of software:
 *
 *  https://github.com/esp8266/Arduino
 *
 *  2015 Tisham Dhar
 *  licensed under GNU GPL
 */
#include <FS.h> //this needs to be first, or it all crashes and burns..

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <Mcp3208.h>

#include <SPI.h>

#ifdef ESP8266
#include <pgmspace.h>
#else
#include <avr/pgmspace.h>
#endif


#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic

#include <ArduinoJson.h> //https://github.com/bblanchon/ArduinoJson

//flag for saving data
bool shouldSaveConfig = false;

#define SPI_CS_A      0        // Phase A CS
#define SPI_CS_B      16       // Phase B CS
#define SPI_CS_C      2        // Phase C CS
#define ADC_VREF    3300     // 3.3V Vref
#define ADC_CLK     1600000  // SPI clock 1.6MHz
#define calibrationMultiplier 4 // Current value multiplied by this value to get Watts

MCP3208 adcA(ADC_VREF, SPI_CS_A);
MCP3208 adcB(ADC_VREF, SPI_CS_B);
MCP3208 adcC(ADC_VREF, SPI_CS_C);

//Maximum value of ADS
#define ADC_COUNTS 4096
#define PHASECAL 1.0
#define VCAL 0.773392
#define ICAL -0.25674048
#define PCAL -1.14
#define POWEROFF -33.0 //Power offset to deal with phase issues

const char* server = "api.thingspeak.com";
// Sign up on thingspeak and get WRITE API KEY.
char auth[36] = "THINGSPEAK_API_KEY";

WiFiClient client;

float phases[3];
float currents[6];
float realPowers[6];
float pfs[6];

//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void readTSConfig()
{
  //clean FS, for testing
  //SPIFFS.format();

  //read configuration from FS json
  //Serial.println("mounting FS...");

  if (SPIFFS.begin()) {
    //Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      //Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        //Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          //Serial.println("\nparsed json");
          strcpy(auth, json["auth"]);

        } else {
          //Serial.println("failed to load json config");
        }
      }
    }
  } else {
    //Serial.println("failed to mount FS");
  }
  //end read
}

void saveTSConfig()
{
  //save the custom parameters to FS
  if (shouldSaveConfig) {
    //Serial.println("saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["auth"] = auth;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      //Serial.println("failed to open config file for writing");
    }

    json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();
    //end save
  }
}

double squareRoot(double fg)  
{
  double n = fg / 2.0;
  double lstX = 0.0;
  while (n != lstX)
  {
    lstX = n;
    n = (n + fg / n) / 2.0;
  }
  return n;
}

void calcVI(unsigned int crossings, unsigned int timeout,MCP3208 adc,MCP3208::Channel current,unsigned int pow_index)
{
  uint32_t t1;
  uint32_t t2;

  uint16_t rawVarr[crossings];
  uint16_t rawIarr[crossings];

  t1 = micros();
  for(int i=0;i<crossings;i++)
  {
    rawVarr[i] = adc.read(MCP3208::SINGLE_0);
    delay(1);
    rawIarr[i] = adc.read(current);
    yield();
  }

  double sumsqV = 0.0;
  double sumsqI = 0.0;
  double sumIV = 0.0;
  double prevalI = 0.0;
  for(int i=0;i<crossings;i++)
  {
    // get analog value
    double valV = (adc.toAnalog(rawVarr[i])-1650);
    double valI = (adc.toAnalog(rawIarr[i])-1651);
    sumsqV = sumsqV + (valV * valV);
    sumsqI = sumsqI + (valI * valI);
    //Interpolate current to account for phase shift due to measurement delay 
    sumIV = sumIV + (valV * valI);
    
    yield();
  }
  phases[pow_index/2] = sqrt(sumsqV);
  currents[pow_index] = sqrt(sumsqI) * calibrationMultiplier;
  realPowers[pow_index]=sumIV;
  t2 = micros();
}

void setup() {
  Serial.begin(115200);
  delay(10);

  //Read previous config
  readTSConfig();
  
  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  WiFiManagerParameter custom_ts_token("ts", "Thingspeak Key", auth, 33);

  //Use wifi manager to get config
  WiFiManager wifiManager;
  wifiManager.setDebugOutput(false);

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //add all your parameters here
  wifiManager.addParameter(&custom_ts_token);

  //first parameter is name of access point, second is the password
  wifiManager.autoConnect("EnergyMonitor", "whatnick");

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");

  //read updated parameters
  strcpy(auth, custom_ts_token.getValue());

  saveTSConfig();

  //Serial.println("");
  //Serial.println("WiFi connected");  
  //Serial.println("IP address: ");
  //Serial.println(WiFi.localIP());
  Wire.begin();
  
  // init done

  pinMode(SPI_CS_A, OUTPUT);
  pinMode(SPI_CS_B, OUTPUT);
  pinMode(SPI_CS_C, OUTPUT);

  // set initial PIN state
  digitalWrite(SPI_CS_A, HIGH);
  digitalWrite(SPI_CS_B, HIGH);
  digitalWrite(SPI_CS_C, HIGH);
  
  // initialize SPI interface for MCP3208
  SPISettings settings(ADC_CLK, MSBFIRST, SPI_MODE0);
  SPI.begin();
  SPI.beginTransaction(settings);

  ArduinoOTA.setPassword((const char *)"123");

  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  
  ESP.wdtEnable(WDTO_8S); // Enabling Watchdog
}

long curMillis=0,prevMillis=0;

void loop() {
  ArduinoOTA.handle();
  ESP.wdtFeed();
  curMillis = millis();
  if((curMillis-prevMillis) > 15000)
  {
    calcVI(20,2000,adcA,MCP3208::SINGLE_1,0); 
    calcVI(20,2000,adcA,MCP3208::SINGLE_2,1);
    calcVI(20,2000,adcB,MCP3208::SINGLE_1,2); 
    calcVI(20,2000,adcB,MCP3208::SINGLE_2,3);
    calcVI(20,2000,adcC,MCP3208::SINGLE_1,4); 
    calcVI(20,2000,adcC,MCP3208::SINGLE_2,5);
  
    for(int k=0;k<6;k++)
    {
      Serial.print(currents[k]);
      Serial.print(",");
    }
    Serial.println();
  
    if (client.connect(server,80)) {  //   "184.106.153.149" or api.thingspeak.com
      String postStr = String(auth);
             postStr +="&field1=";
             postStr += String(currents[0]);
             postStr +="&field2=";
             postStr += String(currents[1]);
             postStr +="&field3=";
             postStr += String(currents[2]);
             postStr +="&field4=";
             postStr += String(currents[3]);
             postStr +="&field5=";
             postStr += String(currents[4]);
             postStr +="&field6=";
             postStr += String(currents[5]);
             postStr += "\r\n\r\n";
   
       client.print("POST /update HTTP/1.1\n"); 
       client.print("Host: api.thingspeak.com\n"); 
       client.print("Connection: close\n"); 
       client.print("X-THINGSPEAKAPIKEY: "+String(auth)+"\n"); 
       client.print("Content-Type: application/x-www-form-urlencoded\n"); 
       client.print("Content-Length: "); 
       client.print(postStr.length()); 
       client.print("\n\n"); 
       client.print(postStr);  
    }
    client.stop();

    prevMillis = curMillis;
  }
}

