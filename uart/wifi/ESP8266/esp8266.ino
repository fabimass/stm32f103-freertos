
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <FirebaseESP8266.h>
#include <addons/RTDBHelper.h>

//#define TEST_MODE

// ESP8266 is not compatible with 5.8GHz networks, must be 2.4GHz
#define WIFI_SSID "network name" 
#define WIFI_PASSWORD "network password"
#define DATABASE_URL "<databaseName>.firebaseio.com"

/**
     Set the database rules to allow public read and write.

       {
          "rules": {
              ".read": true,
              ".write": true
          }
        }

  */

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

String command="", value="";

void setup()
{
    Serial.begin(115200);

    // Wait for the connect command
    do {
      while (Serial.available() == 0) {} 
      command = Serial.readString();
    } while (command != "connect");

#if defined(TEST_MODE)
    Serial.print("Connecting to Wi-Fi");
    Serial.println();
    Serial.print(WIFI_SSID);
    Serial.println();

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print(".");
        delay(300);
    }

    Serial.println();
    Serial.print("Connected with IP: ");
    Serial.println(WiFi.localIP());
    Serial.println();
    Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);
#else
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print("0");
        delay(1000);
    }
    Serial.print("1");
#endif   
    
    config.database_url = DATABASE_URL;

    config.signer.test_mode = true;

    // Comment or pass false value when WiFi reconnection will control by your code or third party library e.g. WiFiManager
    Firebase.reconnectNetwork(true);

    // Since v4.4.x, BearSSL engine was used, the SSL buffer need to be set.
    // Large data transmission may require larger RX buffer, otherwise connection issue or data read time out can be occurred.
    fbdo.setBSSLBufferSize(4096 /* Rx buffer size in bytes from 512 - 16384 */, 1024 /* Tx buffer size in bytes from 512 - 16384 */);

    /* Initialize the library with the Firebase authen and config */
    Firebase.begin(&config, &auth);
}

void loop()
{ 
    // Wait for data available
    while (Serial.available() == 0) {} 
    
    // Read the command
    command = Serial.readString();

    if (Firebase.ready()) {
        
        if (command == "get-led"){
        
            // Query the db for the led status and print it through the serial port
            if(Firebase.getInt(fbdo, F("/led"))){
                value = String(fbdo.to<int>()).c_str();
                Serial.printf("%s", value);         
            }
            
        }

        if (command == "set-count"){

            // Wait for data available
            while (Serial.available() == 0) {}
            
            // Read the value
            value = Serial.readString();
            
            // Update the db with the new count value
            Firebase.setInt(fbdo, F("/count"), String(value).toInt());

        }
         
    }
}
