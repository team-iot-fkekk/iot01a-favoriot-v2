/**
 ******************************************************************************
 * @file   DISCO_IOT_HTS221_DataLog_Terminal.ino
 * @author  WI6LABS from AST
 * @version V1.0.0
 * @date    7 September 2017
 * @brief   Arduino test application for the STMicrolectronics STM32 DISCO_IOT
 *          MEMS Inertial and Environmental sensor expansion board.
 *          This application makes use of C++ classes obtained from the C
 *          components' drivers.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

// Includes.
#include <Arduino.h>
#include <HTS221Sensor.h>
#include <SPI.h>
#include <WiFiST.h>

/*
  The following configuration is dedicated to the DISCO L475VG IoT board.
  You should adapt it to your board.

Configure SPI3:
 * MOSI: PC12
 * MISO: PC11
 * SCLK: PC10

Configure WiFi:
 * SPI         : SPI3
 * Cs          : PE0
 * Data_Ready  : PE1
 * reset       : PE8
 * wakeup      : PB13
 */

SPIClass SPI_3(PC12, PC11, PC10);
WiFiClass WiFi(&SPI_3, PE0, PE1, PE8, PB13);

#define I2C2_SCL PB10
#define I2C2_SDA PB11

char ssid[] = "";             // change it!
char pass[] = "";             // change it!
const String yourDevice = ""; // change it!
int status = WL_IDLE_STATUS;
String apikey = ""; // **replace with your api key from the FAVORIOT platform account setting
char server[] = "apiv2.favoriot.com";
WiFiClient client;

// Components.
HTS221Sensor *HumTemp;
TwoWire *dev_i2c;
void datastream(float humidity, float temperature);
void setup()
{
  // Led.
  pinMode(LED_BUILTIN, OUTPUT);
  // Initialize serial for output.
  Serial.begin(9600);

  // Initialize I2C bus.
  dev_i2c = new TwoWire(I2C2_SDA, I2C2_SCL);
  dev_i2c->begin();

  // Initlialize components.
  HumTemp = new HTS221Sensor(dev_i2c);
  HumTemp->Enable();

  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED)
  {
    Serial.print("Attempting to connect to WiFi network ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
    // wait 10 seconds for connection:
    delay(10000);
  }
  Serial.println("Connected.\n");
}

void loop()
{
  // Led blinking.
  digitalWrite(LED_BUILTIN, HIGH);
  delay(250);
  digitalWrite(LED_BUILTIN, LOW);
  delay(250);

  // Read humidity and temperature.
  float humidity, temperature;
  HumTemp->GetHumidity(&humidity);
  HumTemp->GetTemperature(&temperature);

  // Output data.
  Serial.print("Hum[%]: ");
  Serial.print(humidity, 2);
  Serial.print(" | Temp[C]: ");
  Serial.println(temperature, 2);

  datastream(humidity, temperature);
  delay(20000);
}

void datastream(float humidity, float temperature)
{
  String json = "{\"device_developer_id\":\"" + yourDevice + "\",\"data\":{\"Temperature\":\"" + temperature + "\",\"Humidity\":\"" + humidity + "\"}}";
  //String json = "{\"device_developer_id\":\""+yourDevice+"\",\"data\":{\"Potentio\":\""+pot+"\",\"Temperature\":\""+temp+"\",\"Humidity\":\""+humid+"\"}}";
  Serial.println(json);
  if (client.connect(server, 80))
  {
    // Make a HTTP request:
    Serial.println("        STATUS : Sending data..."); //Display sending status
    client.println("POST /v2/streams HTTP/1.1");
    client.println("Host: apiv2.favoriot.com");
    client.print(String("apikey: "));
    client.println(apikey);
    client.println("Content-Type: application/json");
    client.println("cache-control: no-cache");
    client.print("Content-Length: ");
    int thisLength = json.length();
    client.println(thisLength);
    client.println("Connection: close");

    client.println();
    client.println(json);
    Serial.println("        STATUS : Data sent!"); //display sent status
  }
  client.stop();

  Serial.println("Waiting...");
}