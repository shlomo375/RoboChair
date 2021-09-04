// #ifndef communication_h
// #define communication_h

#include <Arduino.h>


//Check if there is a connection request, if there is already a connection to the client,
//  in case it does not exist then connect to the new client.
bool check_for_TCP_connection(WiFiServer *server, WiFiClient *remote_client);

//increasement counter at TCP_timer overflow
// void IRAM_ATTR TCP_connect_interrupt();

//increasement counter at transmission_timer overflow
// void IRAM_ATTR receive_transmission_interrupt();


String translateEncryptionType(wifi_auth_mode_t encryptionType);

void scanNetworks();

void connectToNetwork(const char* ssid, const char* password);

void send_float_data(WiFiClient *remote_client, float* data, uint8_t data_size);
uint8_t get_float_data(WiFiClient *remote_client, float* float_data, uint8_t data_length);
bool get_order(WiFiClient *remote_client, uint8_t *order, float *data, size_t order_buf_size);
// #elif
// };