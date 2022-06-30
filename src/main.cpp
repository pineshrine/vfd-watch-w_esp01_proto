#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include "RTClib.h"
#include "NTPClient.h"
#include <WiFiUdp.h>
#include "Adafruit_MCP23X17.h"

//io expander
#define IOEXP1 0x20
Adafruit_MCP23X17 mcp;
#define VFD_A 0
#define VFD_B 1
#define VFD_C 2
#define VFD_D 3
#define VFD_E 4
#define VFD_F 5
#define VFD_G 6
#define VFD_COL 7
#define VFD_G1 8
#define VFD_G2 9
#define VFD_G3 10
#define VFD_G4 11
#define VFD_GC 12
#define MD_1 13
#define MD_2 14
int md_state = 0;

//timer0
volatile unsigned long next;

// rtc
RTC_DS3231 rtc;

//wifi temp
const char* ssid = "matsu_11"; 
const char* password = "password";

//ntp
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP,32400);

//vfd mat
unsigned int mat[17] =
{
  0b00111111,
  0b00000110,
  0b01011011,
  0b01001111,
  0b01100110,
  0b01101101,
  0b01111100,
  0b00000111,
  0b01111111,
  0b01100111,
  0b01110111,
  0b01111101,
  0b00111001,
  0b01011110,
  0b01110001,
  0b01110001
};
//Y 0,1,2,3,4,5,6,7,8,9,a,b,c,d,e,f
//X a,b,c,d,e,f,g,:
//: is always 0

//vfd digit
unsigned int dig[4] =
{
  0b00010001,
  0b00010010,
  0b00010100,
  0b00011000,
};
//: is always 1

void vfd_dr(unsigned int cha, unsigned int chb){
  //unsigned int loc_cha = cha;
  //unsigned int loc_chb;
  mcp.writeGPIOA(0b00000000); //ghost workaround
  mcp.writeGPIOB(chb & 0b01111111); //ghost workaround
  if (md_state <= 15)
  {
    //loc_chb = (chb | 0b00100000) & 0b00111111;
    chb = (chb | 0b00100000) & 0b00111111;
    md_state++;
  }else if (md_state < 30)
  {
    //loc_chb = (chb | 0b01000000) & 0b01011111;
    chb = (chb | 0b01000000) & 0b01011111;
    md_state++;
  }else{
    //loc_chb = (chb | 0b01000000) & 0b01011111;
    chb = (chb | 0b01000000) & 0b01011111;
    md_state = 0;
  }
  //mcp.writeGPIOA(loc_cha);
  //mcp.writeGPIOB(loc_chb);
  mcp.writeGPIOA(cha);
  mcp.writeGPIOB(chb);
  delay(1);
}

void vfd_test(void){
  for (size_t i = 1; i < 10000; i++)
  {
    int v = i;
    int usb[4];
    usb[3] = (v % 10); v /= 10;
    usb[2] = (v % 10); v /= 10;
    usb[1] = (v % 10); v /= 10;
    usb[0] = (v % 10); v /= 10;
    for (size_t j = 0; j < 100; j++)
    {
      for (size_t k = 0; k < 4; k++)
      {
        vfd_dr(mat[usb[k]],dig[k]);
        delay(1);
      }
    }
    Serial.print(i);
    Serial.print("\r");
  }
  Serial.println("");
  Serial.println("testing done.");
}

void vfd_test2(void){

  int usb[4];
  usb[3] = 4;
  usb[2] = 3;
  usb[1] = 2;
  usb[0] = 1;
  for (size_t i = 0; i < 100; i++)
  {
    for (size_t j = 0; j < 4; j++)
    {

      vfd_dr(mat[usb[j]],dig[j]);
      delay(1);
    }
    Serial.print(i);
    Serial.print("\r");
  }
  Serial.println("");
  Serial.println("testing done.");
}

void disp_clock(void){

  DateTime time = rtc.now();
  int hh = time.hour();
  int mm = time.minute();
  int blink = time.second() % 2;
  
  int usb[4];
  usb[0] = hh/10;
  usb[1] = hh%10;
  usb[2]= mm/10;
  usb[3] = mm%10;
  //Serial.println(hh);Serial.println(mm);Serial.println(hss.toInt());
  Serial.print(time.timestamp()+"\r");
  for (size_t i = 0; i < 4; i++)
  {
    if(blink)
    {
      vfd_dr((mat[usb[i]] | 0b10000000) & 0b11111111,dig[i]);
    }else{
      vfd_dr(mat[usb[i]] & 0b01111111,dig[i]);
    }
    delay(1);
  }
}

void setup() {
  system_update_cpu_freq(20);
  Wire.begin(0,2);
  Serial.begin(115200);
  Serial.println("Started.");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    for (int i = 0; i < 30; i++)
    {
      delay(500);
      Serial.print(".");
    }
    break;
  }
  Serial.println("");
  Serial.print("SSID: ");
  Serial.println(ssid);
  Serial.println("IP address: ");
  delay(500);

  if (! rtc.begin(&Wire)) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
  }else{
    Serial.println("rtc running");
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println(WiFi.localIP());
    timeClient.begin();
    timeClient.update();
    delay(500);
    if(timeClient.isTimeSet())
    {
      delay(500);
      Serial.println(timeClient.getFormattedTime());
      rtc.adjust(DateTime(timeClient.getEpochTime()));
    }else{
      Serial.println("unable to get ntp date.");
    }
  }else{
    timeClient.end();
  }
  
  if (!mcp.begin_I2C(IOEXP1,&Wire)) {
    Serial.println("MCP Error.");
    while (1);
  }else{
    mcp.pinMode(VFD_A,OUTPUT);
    mcp.pinMode(VFD_B,OUTPUT);
    mcp.pinMode(VFD_C,OUTPUT);
    mcp.pinMode(VFD_D,OUTPUT);
    mcp.pinMode(VFD_E,OUTPUT);
    mcp.pinMode(VFD_F,OUTPUT);
    mcp.pinMode(VFD_G,OUTPUT);
    mcp.pinMode(VFD_COL,OUTPUT);
    mcp.pinMode(VFD_G1,OUTPUT);
    mcp.pinMode(VFD_G2,OUTPUT);
    mcp.pinMode(VFD_G3,OUTPUT);
    mcp.pinMode(VFD_G4,OUTPUT);
    mcp.pinMode(VFD_GC,OUTPUT);
    mcp.pinMode(MD_1,OUTPUT);
    mcp.pinMode(MD_2,OUTPUT);
    mcp.digitalWrite(MD_1,LOW);
    mcp.digitalWrite(MD_2,LOW);
    Serial.println("MCP Setup done.");

    vfd_test2();
  }
  Serial.println("setup done.");
  Serial.println("commencing WiFi turning off.");
  timeClient.end();
  ntpUDP.stopAll();
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
}

void loop(void){
  //vfd_test();
  disp_clock();
}
