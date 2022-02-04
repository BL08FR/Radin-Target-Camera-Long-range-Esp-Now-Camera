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
// shamelessly stolen and adapted from here https://github.com/TenoTrash/ESP32_ESPNOW_RSSI/issues/1. ip fixe
*/


//libs & define's-----------------------------------------------------------------
#include "esp_now.h"
#include "esp_wifi.h"
#include "WiFi.h"
#include "SPIFFS.h"
#include "ESPAsyncWebServer.h"//You also need Async-tcp lib with that one!
#define CHANNEL 1

//USER SETTINGS! Adjust as you need.----------------------------------------------
const char* ssid = "Your SSID here!";// your box/router/AP network name
const char* password = "Your password here!";//it's password

//Sketch variable's---------------------------------------------------------------
int REDLED = 33;// gpio pin of red led (esp32cam/Ai thinker/HW818
int rssi_display;// value to store signal strenght
int RSSIpercent;//value to store last signal percent
boolean allowRSSI = 1;
int currentTransmitCurrentPosition = 0;
int currentTransmitTotalPackages = 0;
byte showImage = 0;
String pictureswitchSTR = "/picture.jpg";// path of working picture
String pictureRevSTR = "/picturetwo.jpg";// path of saved picture

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

uint8_t SenderMacID[6] = {0x10, 0x52, 0x1C, 0x63, 0x3F, 0xB4};//sender adress 10:52:1C:63:3F:B4

typedef struct {
  wifi_ieee80211_mac_hdr_t hdr;
  uint8_t payload[0]; /* network data ended with 4 bytes csum (CRC32) */
} wifi_ieee80211_packet_t;





// Setup function-------------------------------------------------------------------
void setup() {
  pinMode(REDLED, OUTPUT);//red led
  Serial.begin(115200);
  Serial.println("ESP32CAM, Esp-Now receiver/slave");

  //start spiffs
  if (!SPIFFS.begin()){Serial.println(F("ERROR: File System Mount Failed!"));}
  else{Serial.println(F("success init spifss"));}


  //Set device in AP mode to begin with
  WiFi.mode(WIFI_AP_STA);

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
        Serial.println("done file transfer");
        SPIFFS.open(pictureswitchSTR);
        Serial.println(file.size());
        file.close();
        Serial.println("File saved and closed.");
        allowRSSI = 1;//allow to get rssi
        if(digitalRead(REDLED) == HIGH){digitalWrite(REDLED, LOW);}//change led status after file received
          else{digitalWrite(REDLED, HIGH);}
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
