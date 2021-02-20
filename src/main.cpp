#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_SGP30.h"
#include <ESP_EEPROM.h>
#include "SHTSensor.h"
#include <U8g2lib.h>
#include <TaskScheduler.h>
#include "ESP8266WiFiMulti.h"
#include <WiFiClientSecure.h>
#include <ESP8266HTTPClient.h>
#include "ArduinoJson.h"

const char trustRoot[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFFjCCAv6gAwIBAgIRAJErCErPDBinU/bWLiWnX1owDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMjAwOTA0MDAwMDAw
WhcNMjUwOTE1MTYwMDAwWjAyMQswCQYDVQQGEwJVUzEWMBQGA1UEChMNTGV0J3Mg
RW5jcnlwdDELMAkGA1UEAxMCUjMwggEiMA0GCSqGSIb3DQEBAQUAA4IBDwAwggEK
AoIBAQC7AhUozPaglNMPEuyNVZLD+ILxmaZ6QoinXSaqtSu5xUyxr45r+XXIo9cP
R5QUVTVXjJ6oojkZ9YI8QqlObvU7wy7bjcCwXPNZOOftz2nwWgsbvsCUJCWH+jdx
sxPnHKzhm+/b5DtFUkWWqcFTzjTIUu61ru2P3mBw4qVUq7ZtDpelQDRrK9O8Zutm
NHz6a4uPVymZ+DAXXbpyb/uBxa3Shlg9F8fnCbvxK/eG3MHacV3URuPMrSXBiLxg
Z3Vms/EY96Jc5lP/Ooi2R6X/ExjqmAl3P51T+c8B5fWmcBcUr2Ok/5mzk53cU6cG
/kiFHaFpriV1uxPMUgP17VGhi9sVAgMBAAGjggEIMIIBBDAOBgNVHQ8BAf8EBAMC
AYYwHQYDVR0lBBYwFAYIKwYBBQUHAwIGCCsGAQUFBwMBMBIGA1UdEwEB/wQIMAYB
Af8CAQAwHQYDVR0OBBYEFBQusxe3WFbLrlAJQOYfr52LFMLGMB8GA1UdIwQYMBaA
FHm0WeZ7tuXkAXOACIjIGlj26ZtuMDIGCCsGAQUFBwEBBCYwJDAiBggrBgEFBQcw
AoYWaHR0cDovL3gxLmkubGVuY3Iub3JnLzAnBgNVHR8EIDAeMBygGqAYhhZodHRw
Oi8veDEuYy5sZW5jci5vcmcvMCIGA1UdIAQbMBkwCAYGZ4EMAQIBMA0GCysGAQQB
gt8TAQEBMA0GCSqGSIb3DQEBCwUAA4ICAQCFyk5HPqP3hUSFvNVneLKYY611TR6W
PTNlclQtgaDqw+34IL9fzLdwALduO/ZelN7kIJ+m74uyA+eitRY8kc607TkC53wl
ikfmZW4/RvTZ8M6UK+5UzhK8jCdLuMGYL6KvzXGRSgi3yLgjewQtCPkIVz6D2QQz
CkcheAmCJ8MqyJu5zlzyZMjAvnnAT45tRAxekrsu94sQ4egdRCnbWSDtY7kh+BIm
lJNXoB1lBMEKIq4QDUOXoRgffuDghje1WrG9ML+Hbisq/yFOGwXD9RiX8F6sw6W4
avAuvDszue5L3sz85K+EC4Y/wFVDNvZo4TYXao6Z0f+lQKc0t8DQYzk1OXVu8rp2
yJMC6alLbBfODALZvYH7n7do1AZls4I9d1P4jnkDrQoxB3UqQ9hVl3LEKQ73xF1O
yK5GhDDX8oVfGKF5u+decIsH4YaTw7mP3GFxJSqv3+0lUFJoi5Lc5da149p90Ids
hCExroL1+7mryIkXPeFM5TgO9r0rvZaBFOvV2z0gp35Z0+L4WPlbuEjN/lxPFin+
HlUjr8gRsI3qfJOQFy/9rKIJR0Y/8Omwt/8oTWgy1mdeHmmjk7j1nYsvC9JSQ6Zv
MldlTTKB3zhThV1+XWYp6rjd5JW1zbVWEkLNxE7GJThEUG3szgBVGP7pSWTUTsqX
nLRbwHOoq7hHwg==
-----END CERTIFICATE-----
)EOF";
X509List cert(trustRoot);

Scheduler runner;
Adafruit_SGP30 sgp;
SHTSensor sht;
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);
ESP8266WiFiMulti wifiMulti;

float humidity;
float temperature;
uint16_t eCO2;
uint16_t TVOC;
uint16_t eCO2_baseline = 0;
uint16_t TVOC_baseline = 0;

uint32_t getAbsoluteHumidity(float temperature, float humidity)
{
  // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
  const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
  const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity);                                                                // [mg/m^3]
  return absoluteHumidityScaled;
}

void sht_task_callback()
{
  if (sht.readSample())
  {
    humidity = sht.getHumidity();
    temperature = sht.getTemperature();
    sgp.setHumidity(getAbsoluteHumidity(temperature, humidity));
    Serial.print("SHT:\n");
    Serial.print("  RH: ");
    Serial.print(sht.getHumidity(), 1);
    Serial.print("\n");
    Serial.print("  T:  ");
    Serial.print(sht.getTemperature(), 1);
    Serial.print("\n");
  }
  else
  {
    Serial.print("[SHT30]Error in readSample()\n");
  }
}

void sgp_task_callback()
{

  if (sgp.IAQmeasure())
  {
    Serial.print("TVOC ");
    Serial.print(sgp.TVOC);
    Serial.print(" ppb\t");
    Serial.print("eCO2 ");
    Serial.print(sgp.eCO2);
    Serial.println(" ppm");

    eCO2 = sgp.eCO2;
    TVOC = sgp.TVOC;
  }
  else
  {
    Serial.print("[SGP30]Error in IAQmeasure()\n");
  }
}

void ssd1306_task_callback()
{
  u8g2.clearBuffer();
  u8g2.drawUTF8(0, 16, ("温度: " + String(temperature) + " °C").c_str());
  u8g2.drawUTF8(0, 32, ("湿度: " + String(humidity) + " %").c_str());
  u8g2.drawUTF8(0, 48, ("eCO2: " + String(eCO2) + " ppm").c_str());
  u8g2.drawUTF8(0, 64, ("TVOC: " + String(TVOC) + " ppb").c_str());
  u8g2.sendBuffer();
}

void save_baseline_callback()
{
  if (sgp.getIAQBaseline(&eCO2_baseline, &TVOC_baseline))
  {
    EEPROM.put(0, eCO2_baseline);
    EEPROM.put(2, TVOC_baseline);

    Serial.println(eCO2_baseline, HEX);
    Serial.println(TVOC_baseline, HEX);

    boolean ok = EEPROM.commit();
    Serial.println((ok) ? "EEPROM commit OK" : "Commit failed");
  }
}
WiFiClientSecure wificlient;
HTTPClient httpclient;
StaticJsonDocument<64> doc;
uint8_t json[64];

bool connectHTTP()
{
  configTime(8 * 3600, 0, "ntp2.aliyun.com", "ntp3.aliyun.com");

  wificlient.setTrustAnchors(&cert);
  // wificlient.setInsecure();
  httpclient.setReuse(true);
  if (httpclient.begin(wificlient, "https://iot.swz1994.xyz"))
  {
    return true;
  }
  else
  {
    return false;
  }
}

void ota_task_callback()
{
  if (WiFi.isConnected())
  {
    Serial.println(ESP.getFreeHeap());

    if (!httpclient.connected())
    {
      if (!connectHTTP())
      {
        return;
      }
    }
    httpclient.setURL("/api/v1/AE9nt8TXo1YlCG8XsP3o/attributes?&sharedKeys=ota,ota_version,ota_url");

    Serial.println(httpclient.GET());
    Serial.println(httpclient.getString());
    Serial.println(ESP.getFreeHeap());
  }
}

void http_task_callback()
{
  if (WiFi.isConnected())
  {
    Serial.println(ESP.getFreeHeap());

    if (!httpclient.connected())
    {
      if (!connectHTTP())
      {
        return;
      }
    }
    httpclient.setURL("/api/v1/AE9nt8TXo1YlCG8XsP3o/telemetry");

    httpclient.addHeader("Content-Type", "application/json");
    Serial.println("post data");

    doc["TVOC"] = TVOC;
    doc["eCO2"] = eCO2;
    doc["Temp"] = temperature;
    doc["Hum"] = humidity;

    size_t size = serializeJson(doc, json);
    httpclient.addHeader("Content-Length", String(size));
    Serial.println(httpclient.POST(json, size));
    Serial.println(ESP.getFreeHeap());
  }
  else
  {
    Serial.println("WiFi not connected");
    if (wifiMulti.run() == WL_CONNECTED)
    {
      Serial.println("");
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
    }
  }
}

Task sht_task(10000, TASK_FOREVER, &sht_task_callback);
Task sgp_task(1000, TASK_FOREVER, &sgp_task_callback);
Task ssd1306_task(5000, TASK_FOREVER, &ssd1306_task_callback);
Task save_baseline(60 * 60 * 1000, TASK_FOREVER, &save_baseline_callback);
Task http_task(10 * 1000, TASK_FOREVER, &http_task_callback);
Task ota_task(15 * 1000, TASK_FOREVER, &ota_task_callback);

void setup()
{
  Serial.begin(9600);
  Wire.begin(D1, D2);

  wifiMulti.addAP("HOME", "12345679");
  wifiMulti.addAP("OnePlus7Pro", "12345679");
  wifiMulti.addAP("OpenWrt", "12345679");

  if (sgp.begin() && u8g2.begin() && sht.init())
  {
  }
  else
  {
    ESP.restart();
  }

  EEPROM.begin(4);
  EEPROM.get(0, eCO2_baseline);
  EEPROM.get(2, TVOC_baseline);
  if (eCO2_baseline != 0 && TVOC_baseline != 0)
  {
    sgp.setIAQBaseline(eCO2_baseline, TVOC_baseline);
  }

  u8g2.setFont(u8g2_font_wqy15_t_gb2312);

  sht.setAccuracy(SHTSensor::SHT_ACCURACY_HIGH);

  runner.init();
  runner.addTask(sht_task);
  runner.addTask(sgp_task);
  runner.addTask(ssd1306_task);
  runner.addTask(save_baseline);
  runner.addTask(http_task);
  runner.addTask(ota_task);
  runner.enableAll();
}

void loop()
{
  runner.execute();
}