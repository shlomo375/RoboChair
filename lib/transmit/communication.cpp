#include <Arduino.h>
#include <main.cpp>
#include <WiFi.h>
#include <HardwareSerial.h>

//Check if there is a connection request, if there is already a connection to the client,
//  in case it does not exist then connect to the new client.
bool check_for_TCP_connection(WiFiServer *server, WiFiClient *remote_client)
{
  if(server->hasClient())
  {
    if(remote_client->connected())
    {
      server->available().stop();
      Serial.println("connection rejected");
    }
    else
    {
      *remote_client = server->available();
      Serial.println("connction accepted");
    }
    return true;
  }
  return false;
}

//increasement counter at TCP_timer overflow
// void IRAM_ATTR TCP_connect_interrupt()
// {
//   portENTER_CRITICAL_ISR(&mux);
//   tmr_TCP_counter++;
//   portEXIT_CRITICAL_ISR(&mux);
// }


//increasement counter at transmission_timer overflow
// void IRAM_ATTR receive_transmission_interrupt(){
//   portENTER_CRITICAL_ISR(&mux);
//   tmr_transmission_counter ++;
//   portEXIT_CRITICAL_ISR(&mux);
// }


String translateEncryptionType(wifi_auth_mode_t encryptionType) {
 
  switch (encryptionType) {
    case (WIFI_AUTH_OPEN):
      return "Open";
    case (WIFI_AUTH_WEP):
      return "WEP";
    case (WIFI_AUTH_WPA_PSK):
      return "WPA_PSK";
    case (WIFI_AUTH_WPA2_PSK):
      return "WPA2_PSK";
    case (WIFI_AUTH_WPA_WPA2_PSK):
      return "WPA_WPA2_PSK";
    case (WIFI_AUTH_WPA2_ENTERPRISE):
      return "WPA2_ENTERPRISE";
    case (WIFI_AUTH_MAX):
      return "WIFI_AUTH_MAX";
    default:
      return "Unknown";
  }
}

void scanNetworks() {
 
  int numberOfNetworks = WiFi.scanNetworks();
 
  Serial.print("Number of networks found: ");
  Serial.println(numberOfNetworks);
 
  for (int i = 0; i < numberOfNetworks; i++) {
 
    Serial.print("Network name: ");
    Serial.println(WiFi.SSID(i));
 
    Serial.print("Signal strength: ");
    Serial.println(WiFi.RSSI(i));
 
    Serial.print("MAC address: ");
    Serial.println(WiFi.BSSIDstr(i));
 
    Serial.print("Encryption type: ");
    String encryptionTypeDescription = translateEncryptionType(WiFi.encryptionType(i));
    Serial.println(encryptionTypeDescription);
    Serial.println("-----------------------");
 
  }
}


void connectToNetwork(const char* ssid, const char* password) {
  WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Establishing connection to WiFi..");
  }
 
  Serial.println("Connected to network");
 
}

void send_float_data(WiFiClient *remote_client, float* float_data, uint8_t data_length) {
  if(remote_client->connected()) {
    uint8_t byte_legth = data_length * sizeof(float);
    uint8_t data[byte_legth];
    
    memcpy(data, float_data, byte_legth);
   
    remote_client->write(data, byte_legth);
  }
  else {
    Serial.println("There's no connection.");
  }
}

uint8_t get_float_data(WiFiClient *remote_client, float* float_data,uint8_t data_length) {
  uint8_t packege = 0;
  while(remote_client->available() && remote_client->connected()) {
    uint8_t byte_legth = data_length * sizeof(float);
    uint8_t data[byte_legth];
    
    packege = remote_client->read(data, sizeof(data));

    memcpy(float_data, data, byte_legth);
    
  }
  return packege;
}


bool get_order(WiFiClient *remote_client, uint8_t *order, float *data, size_t order_buf_size){
  // uint8_t o[5];
  // size_t s=5;
  if(remote_client->available() && remote_client->connected()){
    // remote_client->printf("check point 2");
    // remote_client->print("order buf size: ");
    // remote_client->print("end: ");
    remote_client->read(order,order_buf_size);
    // remote_client->read(o,s);
    // remote_client->writ("check point 3");
    if(order[0]==1){
      // remote_client->print("check point 4");
      uint8_t packege = get_float_data(remote_client, data, order[2]);
      // remote_client->print("check point 5");
      if(packege == order[2]){
        // remote_client->print("check point 6");
        // remote_client->print("controler get all the data.");
        return true;
      }
      else{
        // remote_client->print("check point 7");
        return false;
      }
      // remote_client->print("check point 8");
    }
    else
    {
      // remote_client->print("check point 9");
      // Serial.printf("After executing the command the data will be sent.");
      return true;
    }
  }
  else{
    return false;
  }
}