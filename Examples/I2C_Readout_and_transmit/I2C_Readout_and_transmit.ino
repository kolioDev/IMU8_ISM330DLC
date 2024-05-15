#include <Wire.h>
#include "./c6dof_lib/c6dof_lib.c"
#include "./read_data/read_data.c"

// #define CONNECTION_TYPE_WIFI_TCP
#define CONNECTION_TYPE_WIFI_UDP

// Both should be off of wifi to work
// #define CONNECTION_TYPE_SERIAL
// #define SERIAL_DEBUG

  
#if  defined(CONNECTION_TYPE_WIFI_TCP) || defined(CONNECTION_TYPE_WIFI_UDP)
  #include <WiFi.h>
  const char *ssid = "ESP32-AP";
  const char *password = "password_esp32";
  unsigned int localPort = 12345;
#endif

#ifdef CONNECTION_TYPE_WIFI_TCP
  #include <WiFiClient.h>
  WiFiServer server(localPort); // Use TCP server
  WiFiClient client;
#endif

#ifdef CONNECTION_TYPE_WIFI_UDP
  #include <WiFiUdp.h>
  WiFiUDP udp;
  IPAddress pythonIP(192, 168, 4, 2); // IP address of the Python client
#endif


#define DEVICE_ADR  0x6B // the I2C address of the Microelectronica board

char SETUP_COMPLETE = 0;
c6dofimu8_t board_ctx;
uint32_t c = 0;


void setup() {
  pinMode(13, OUTPUT); // LED pin  - for debug purposes
  digitalWrite(13, LOW); // Turn off the LED

  Wire.begin();        // join i2c bus (address optional for master)
  Wire.setClock(400000); // Set the I2C clock frequency to 400 kHz
  // Wire.setClock(1000000);
  // Wire.setClock(50000); //kHz

  #if defined(SERIAL_DEBUG) || defined(CONNECTION_TYPE_SERIAL)
  Serial.begin(250000);  // start serial for output
  while (!Serial); // Wait for serial monitor to open
  #endif

  #ifdef CONNECTION_TYPE_WIFI_TCP
    WiFi.softAP(ssid, password);
    server.begin(); // Start TCP server
  #endif
  #ifdef CONNECTION_TYPE_WIFI_UDP
    WiFi.softAP(ssid, password);
    udp.begin(localPort);
  #endif
  

  delay(1000);
  println("Reset the board..");
  
  SETUP_COMPLETE = connect2Device(DEVICE_ADR)==0;

  uint8_t err = c6dofimu8_default_cfg(DEVICE_ADR, &board_ctx, C6DOFIMU8_ODR_1660_HZ,  C6DOFIMU8_FS_G_500DPS, C6DOFIMU8_FS_XL_8G);
  if(err != C6DOFIMU8_OK){
   print("Error - ");
    println(err);
  }
  delay(1000);
  println("Setup complete");
  digitalWrite(13, HIGH); // Turn on the LED
}





void loop(){
  // Check if the connection to the Microelectronica device is complete 
  if(!SETUP_COMPLETE){
    SETUP_COMPLETE = connect2Device(DEVICE_ADR)==0;
    return;
  }

 u_int8_t data[ 14 ];
 u_int8_t err =  read_row_data(&board_ctx, data);
 if(err!=0){
    if(err != NO_DATA_AVAILABLE){
       print("Sensor data obtaining error:  ");
       println(err);
    }
    return;;
 }

  
  #ifdef CONNECTION_TYPE_SERIAL
    transmit_raw_data_serial(data, sizeof(data), c);
  #endif

  #ifdef CONNECTION_TYPE_WIFI_TCP
    
    if(!client){
      client = server.available();
    }
    
    if(client){
      if(c%1000!=0 || client.connected()){ //check for client only every 1000th cycles
        transmit_raw_data_wifi_tcp(data, sizeof(data), c, client);
      }else{
        client.stop();
      }
    }
  
  #endif

  #ifdef CONNECTION_TYPE_WIFI_UDP
    uint32_t currentTime = micros();
    udp.beginPacket(pythonIP, localPort); // Send to Python client's IP
    udp.write((uint8_t *)&c, sizeof(c));
    udp.write(data,sizeof(data));
    udp.write((uint8_t *)&currentTime, sizeof(currentTime));
    udp.endPacket();
  #endif

  // Debug purposes;
  // Keep the light for 1500 cycles, then off for 1500, etc
  if(c%3000<1500){
    digitalWrite(13, HIGH);
  }else{
    digitalWrite(13, LOW);
  }


  c++;
  // loop_read_data();
}

int connect2Device(byte address){
   println("Try connecting");
   
   Wire.beginTransmission(address);
   byte error = Wire.endTransmission();

   if (error == 0) {
    print("I2C device found at address 0x");
     println(address, HEX);
     
   } else if (error == 4) {
     println("Unknown error connecting with device");
     return 1;
   }

   return 0;
}


void discover_i2c_device() {
 byte error, address;
 int deviceCount = 0;

 println("Scanning...");

 for (address = 1; address < 127; address++) {
   Wire.beginTransmission(address);
   error = Wire.endTransmission();

   if (error == 0) {
    print("I2C device found at address 0x");
     if (address < 16)
      print("0");
    print(address, HEX);
     println(" !");
     deviceCount++;
   } else if (error == 4) {
    print("Unknown error at address 0x");
     if (address < 16)
      print("0");
     println(address, HEX);
   }
 }

 if (deviceCount == 0)
   println("No I2C devices found\n");
 else
   println("Done\n");
}



// Wrapper forprintln() with multiple parameters
template<typename... Args>
void println(const Args&... args) {
  #ifdef SERIAL_DEBUG
    (Serial.println(args), ...);
  #endif
}

// Wrapper forprint() with multiple parameters
template<typename... Args>
void print(const Args&... args) {
  #ifdef SERIAL_DEBUG
    (Serial.print(args), ...);
  #endif
}