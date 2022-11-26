/*
// CAPTURE AND SEND IMAGE OVER ESP NOW
// WIFIAP and RSSI modifications by Blaise Lapierre!
// 
// Original Code by: Tal Ofer
// https://github.com/talofer99/ESP32CAM-Capture-and-send-image-over-esp-now
// This is the screen portion of the code for more information
// https://www.youtube.com/watch?v=0s4Bm9Ar42U
// 
// - The receiver connect to an access point so data will be accesible on it via
// webnavigator (it need that sender also get the same wifi channel). Shamelessly
// stolen and adapted from here https://randomnerdtutorials.com/esp32-esp-now-wi-fi-web-server/.
// - RSSI from ESP NOW sender is available once after each file transmission completed,
// shamelessly stolen and adapted from here https://github.com/TenoTrash/ESP32_ESPNOW_RSSI/issues/1.
//
// The access point must be switched on first the receiver second and therefore the camera and in range of it(<=50m)!
// Once the camera is started with correct parameters according to them, AP & receiver can
// be switched on/off as you need without effect on the camera (if the AP don't change it's wifi channel)
//
// Updated for TTGO T-JOURNAL board (much more easy to use the 22/11/2022.
// Don't forget to not only uplaod the sketch to the board but also sketch data in the spiffs memory
// and to adjust in the sketch : 
//  - wifi SSID of the access point and it's password
//  - local_IP which is the IP address you'll have to type in your browser (pay attention what range give
//    your ap usually it's 192.168.1.xx
//  - SenderMacID (MAC address of the camera board)
//  - ANTgain, antenna gain to reduce the power if you use a directionnal antenna
*/


//libs & define's-----------------------------------------------------------------
#include "esp_now.h"
#include "esp_wifi.h"
#include "WiFi.h"
#include "SPIFFS.h"
#include "ESPAsyncWebServer.h"//You also need Async-tcp lib with that one!
#define CHANNEL 1

//USER SETTINGS! Adjust as you need.----------------------------------------------
const char* ssid = "Wp5pro";// your box/router/AP network nameB4
const char* password = "1234567890";//it's password
IPAddress local_IP(192, 168, 43, 160);// Set a Static IP address on which the webpage will be accessible.
uint8_t SenderMacID[6] = {0x4C, 0xEB, 0xD6, 0x43, 0x3C, 0x54}; /*{0x30, 0xC6, 0xF7, 0x05, 0xC8, 0x44}; {0x10, 0x52, 0x1C, 0x63, 0x3F, 0xB4}; 0x78, 0x21, 0x84, 0x80, 0x8C, 0x30 TJURNAL0x4C, 0xEB, 0xD6, 0x43, 0x3C, 0x54 */
//Modify this according to the sender board MAC adress (example here for MAC address 10:52:1C:63:3F:B4)
int ANTgain = 6;// gain of your antenna, in France according to the law you can stay at 100 mW EIRP (20 dBm) if <= 6dBi. The antenna manufacturer may lie about the gain! This is needed because the receiver only receive on ESP-NOW but emit in classic WiFi!

//Sketch variable's---------------------------------------------------------------
int rssi_display;// value to store signal strenght
int RSSIpercent;//value to store last signal percent
boolean allowRSSI = 1;
int currentTransmitCurrentPosition = 0;
int currentTransmitTotalPackages = 0;
byte showImage = 0;
String pictureswitchSTR = "/picture.jpg";// path of working picture
String pictureRevSTR = "/picturetwo.jpg";// path of saved picture
unsigned long lastRSSIpercentCalculated;//store last time rssipercent value was calculated
unsigned long lastphotoreceived = 999999;//store last time rssipercent value was calculated
int RSSIpercentDelay = 20000;//to reset rssipercent value to zero if timed out (disconnected)
unsigned long Sago = 0;
unsigned long lastSago = 0;

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  int id;
  float picture;
  unsigned int readingId;
} struct_message;

//Webserver-------------------------------------------------------------------------
AsyncWebServer server(80);//Create an Async Web Server on port 80.
AsyncEventSource events("/events");//To automatically display the information on the web server when a new reading arrives, we’ll use Server-Sent Events (SSE).

//RSSI measure----------------------------------------------------------------------
struct_message paquete_datos;

// Estructuras para calcular los paquetes, el RSSI, etc
typedef struct {
  unsigned frame_ctrl: 16;
  unsigned duration_id: 16;
  uint8_t addr1[6]; /* receiver address */
  uint8_t addr2[6]; /* sender address */
  uint8_t addr3[6]; /* filtering address */
  unsigned sequence_ctrl: 16;
  uint8_t addr4[6]; /* optional */
} wifi_ieee80211_mac_hdr_t;

typedef struct {
  wifi_ieee80211_mac_hdr_t hdr;
  uint8_t payload[0]; /* network data ended with 4 bytes csum (CRC32) */
} wifi_ieee80211_packet_t;


//Static IP settings----------------------------------------------------------------
// Set your Gateway IP address, device IP was set in USER SETTINGS at the begining of this file
IPAddress gateway(192, 168, 1, 1);

IPAddress subnet(255, 255, 0, 0);
IPAddress primaryDNS(8, 8, 8, 8);   //optional
IPAddress secondaryDNS(8, 8, 4, 4); //optional

//For T-JOURNAL
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define I2C_SDA 14
#define I2C_SCL 13

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);


// Setup function-------------------------------------------------------------------
void setup() {
  SPIFFS.begin(true);//start the SPIFFS
  Serial.begin(115200);
  Serial.println("ESP32CAM, Esp-Now receiver/slave");

  //start spiffs
  if (!SPIFFS.begin()){Serial.println(F("ERROR: File System Mount Failed!"));}
  else{Serial.println(F("success init spifss"));}
  
  // init T-JOURNAL Display
  Wire.begin(I2C_SDA, I2C_SCL);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C, false, false)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);}  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.print("EspNow Receiver!");
  display.display();

  //Set TX power according to antenna gain
  float maxPWR = 20 - (ANTgain - 6);//cable loss allow 6dBi
  Serial.print("Max power to stay legal is ");Serial.print(maxPWR);Serial.println("dBm");
  Serial.print("Power set to ");
  if(maxPWR <= 1){WiFi.setTxPower(WIFI_POWER_MINUS_1dBm);Serial.println("1dBm");}
  if(maxPWR <= 2){WiFi.setTxPower(WIFI_POWER_2dBm);Serial.println("2dBm");}
  if(maxPWR <= 5){WiFi.setTxPower(WIFI_POWER_5dBm);Serial.println("5dBm");}
  if(maxPWR <= 7){WiFi.setTxPower(WIFI_POWER_7dBm);Serial.println("7dBm");}
  if(maxPWR <= 8.5){WiFi.setTxPower(WIFI_POWER_8_5dBm);Serial.println("8.5dBm");}
  if(maxPWR <= 11){WiFi.setTxPower(WIFI_POWER_11dBm);Serial.println("11dBm");}
  if(maxPWR <= 13){WiFi.setTxPower(WIFI_POWER_13dBm);Serial.println("13dBm");}
  if(maxPWR <= 15){WiFi.setTxPower(WIFI_POWER_15dBm);Serial.println("15dBm");}
  if(maxPWR <= 17){WiFi.setTxPower(WIFI_POWER_17dBm);Serial.println("17dBm");}
  if(maxPWR <= 18.5){WiFi.setTxPower(WIFI_POWER_18_5dBm);Serial.println("18.5dBm");}
  if(maxPWR <= 19){WiFi.setTxPower(WIFI_POWER_19dBm);Serial.println("19dBm");}
  if(maxPWR > 19){WiFi.setTxPower(WIFI_POWER_19_5dBm);Serial.println("19.5dBm");}
  
  
  //Set device in AP mode to begin with
  WiFi.mode(WIFI_AP_STA);

  // Configures static IP address
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("STA Failed to configure");
  }

  // Set device as a Wi-Fi Station
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Setting as a Wi-Fi Station..");
  }
Serial.print("Station IP Address: ");
Serial.println(WiFi.localIP());
Serial.print("Wi-Fi Channel: ");
Serial.println(WiFi.channel());
  //display IP on sreen
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 16);
  display.print("IP: "); display.print(local_IP);
  display.display();
  // configure device AP mode
  configDeviceAP();
  // This is the mac address of the Slave in AP Mode
  Serial.print("AP MAC: "); Serial.println(WiFi.softAPmacAddress());
  // Init ESPNow with a fallback logic
  InitESPNow();// function to init esp-now
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info.
  esp_now_register_recv_cb(OnDataRecv);//function to receive data
  
  //RSSI
  esp_wifi_set_promiscuous(true);//set promiscuous mode
  esp_wifi_set_promiscuous_rx_cb(&promiscuous_rx_cb);//call function to calculate RSSI

  //Webserver
  //Get main page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", "text/html");
  });
  //Get CSS library file
  server.on("/w3.css", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/w3.css", "text/css");
  });
  //Get javascript file
  server.on("/script.js", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/script.js", "text/javascript");
  });
    
  //Get the photo
  server.on("/img", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, pictureRevSTR, "image/jpeg", false);
  });

  //Get the RSSI percent value
  server.on("/readRSSI", HTTP_GET, [](AsyncWebServerRequest *request){
    String strRSSIpercent = String(RSSIpercent);
    request->send(200, "text/plain", strRSSIpercent);
  });
  server.begin();
  Serial.println("Server active!");
  
}//end setup


// Loop function--------------------------------------------------------------------
void loop(){
  yield();//empty function, do yield (perform background task).
  if(millis() >= lastRSSIpercentCalculated+RSSIpercentDelay){
    RSSIpercent = 0;//if delay is exceeded (disconnected) set RSSIpercent to zero
    allowRSSI = 1;//allow to calculate RSSI again
  }

    Sago = (millis()-lastphotoreceived)/1000;
    Serial.println("Sago: ");Serial.println(Sago);
    if(Sago != lastSago){// Erase photo received frm sreen after delay  
      display.clearDisplay();  
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(0, 0);
      display.print("EspNow Receiver!");
      display.setCursor(0, 12);
      display.print("Last pict. "); display.print(Sago); display.print("s ago");
      display.setCursor(0, 24);
      display.print("RSSI: " + String(rssi_display) + " / " + String(RSSIpercent) + "%");
      display.display();
    }
}






// callback when data is recv from Master-------------------------------------------
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {

  switch (*data++)
  {
    case 0x01:
      Serial.println("Start of new file transmit");
      allowRSSI = 0;//disable RSSI request during transfert
      currentTransmitCurrentPosition = 0;
      currentTransmitTotalPackages = (*data++) << 8 | *data;
      Serial.println("currentTransmitTotalPackages = " + String(currentTransmitTotalPackages));
      SPIFFS.remove(pictureswitchSTR);
      break;
    case 0x02:
      //Serial.println("chunk of file transmit");
      currentTransmitCurrentPosition = (*data++) << 8 | *data++;
      //Serial.println("chunk NUMBER = " + String(currentTransmitCurrentPosition));
      File file = SPIFFS.open(pictureswitchSTR,FILE_APPEND);
      if (!file)
        Serial.println("Error opening file ...");
        
      for (int i=0; i < (data_len-3); i++)
      {
        //byte dat = *data++;
        //Serial.println(dat);
        file.write(*data++);
      }
      file.close();

      if (currentTransmitCurrentPosition == currentTransmitTotalPackages)
      {
        showImage = 1;
        lastphotoreceived = millis();
        Serial.println("done file transfer");
        SPIFFS.open(pictureswitchSTR);
        Serial.println(file.size());
        file.close();
        Serial.println("File saved and closed.");
        allowRSSI = 1;//allow to get rssi
        if(pictureswitchSTR == "/picture.jpg"){pictureswitchSTR = "/picturetwo.jpg"; pictureRevSTR = "/picture.jpg";}
          else{ pictureswitchSTR = "/picture.jpg"; pictureRevSTR = "/picturetwo.jpg";}
      }      
      break;
  } //end case 
} //end of OnDataRecv function


//La callback que hace la magia (to get RSSI)---------------------------------------
void promiscuous_rx_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
 if(allowRSSI == 1){
  // All espnow traffic uses action frames which are a subtype of the mgmnt frames so filter out everything else.
  if (type != WIFI_PKT_MGMT)
    return;

  const wifi_promiscuous_pkt_t *ppkt = (wifi_promiscuous_pkt_t *)buf;
  const wifi_ieee80211_packet_t *ipkt = (wifi_ieee80211_packet_t *)ppkt->payload;
  const wifi_ieee80211_mac_hdr_t *hdr = &ipkt->hdr;
  
  for (size_t i = 0; i < 6; i++)
  {
    if (hdr->addr2[1] != SenderMacID[1]){return;}
    rssi_display = ppkt->rx_ctrl.rssi;
    RSSIpercent = (-90-rssi_display)/-0.6;//RSSI -30 = 100%, -90 = 0%
    lastRSSIpercentCalculated = millis();
   }
 if(RSSIpercent > 100){RSSIpercent = 100;}// if >100 = 100
 if(RSSIpercent < 0){RSSIpercent = 0;}// if < 0 = 0
 String rxsignal ="RSSI: ";
 String separator =" / ";
 String sign ="%";
 Serial.println(rxsignal+ rssi_display + separator + RSSIpercent + sign);//display on serial
 Serial.println();
 
 //events.send(String(RSSIpercent).c_str(), "readRSSI", millis());//send to client via javascript RSSIpercent variable
 allowRSSI = 0;//disable next request till ,next file transfert is done
 }
}//end of promiscuous_rx_cb function



// Init ESP Now with fallback------------------------------------------------------
void InitESPNow() {
  //WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
  else {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}//end of esp now init function

// config AP SSID------------------------------------------------------------------
void configDeviceAP() {
  const char *SSID = "Slave_1";
  bool result = WiFi.softAP(SSID, "Slave_1_Password", CHANNEL, 0);
  if (!result) {
    Serial.println("AP Config failed.");
  } else {
    Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID));
  }
}//end of esp now slave init function
