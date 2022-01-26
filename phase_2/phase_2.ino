
#include <WiFi.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <string.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>


#define rele 23

WiFiClient espClient;
WebSocketsServer webSocket = WebSocketsServer(81);

PubSubClient client(espClient);


// Esto debe guardarse en la memoria EEPROM para restaurar internet en caso de corte de luz
const char* ssid = "";
const char* password = "";

// MQTT Broker
const char *mqtt_broker = "192.168.0.16";
const char *topic = "";
const char *mqtt_username = "esp32";
const char *mqtt_password = "public";
const char *will_topic = "";
const int mqtt_port = 1883;

// Esto debe tener un id Unico para identificar como red unica
const char* APssid = "Access Point Esp32";
const char* APpassword = "12345678";

// IDENTIDAD DEL DISPOSITIVO, Esto se va a devolver cada vez que la app se sincronice por WS llamando al evento /device_information
// TODO:esto debe guardarse en la memoria EEPROMM 
//const char* roomName = "";
//const char* deviceId = "";

const char* topicMqttDevice = "";

// Esto vendrá hardcodeado dependiendo del dispositivo
const char* typeOfDevice = "light";

#define MQTTpubQos          2

void decodeMessage(uint8_t num, char* json){
  // Leer Json
  DynamicJsonDocument doc(1024);
  deserializeJson(doc, json);

  // Responder Json
  DynamicJsonDocument response(1024);
  char responseBuffer[100];
              
  String eventName = doc["event_name"];  
  Serial.print(eventName);

  // Si el tipo de dispositivo es "luz"
  if(eventName == "/change_state"){
     String state = doc["message"]["state"];
     
     if(state == "on"){
        digitalWrite(2, HIGH);          
      } else if(state == "off"){    
        digitalWrite(2, LOW);
      }   
  }

  if(eventName == "/set_information"){
      // Debo generar conexión a servidor MQTT y suscribirme a un tópico

      //roomName = doc["message"]["room_name"];
      //deviceId = ssid = doc["message"]["device_id"];
      
      ssid = doc["message"]["ssid"];
      password = doc["message"]["password"];
      
      topic = doc["message"]["topic_mqtt_device"];
      
      // Este es el ultimo evento de la configuracion de dispositivo desde la APP, para este entonces, ya tendré cargado los datos del ssid y password en la placa y solo debo conectarme
      
      // TODO: ARREGLAR EL POR QUE ME ESTA MOSTRANDO PARTE DE OTRAS VARIABLES CUANDO QUIERO PASARLE EL SSID Y PASSWORD PARA CONFIGURAR WIFI
       setupWifi(ssid, password);

      // TODO: UNA VEZ QUE ME CONECTÉ A INTERNET, NECESITO SUSCRIBIRME AL TOPICO QUE ME DA LA APP Y GUARDAR ESE VALOR EN LA MEMORIA EEPROM IGUAL QUE LA PASSWORD Y SSID DEL WIFI

      // TODO: ESTE FEEDBACK NO LO ESTOY DANDO YA QUE AL CONECTARME A INTERNET, ME FUI DE LA CONEXION SOCKET Y LA APP NUNCA RECIBE EL EVENTO
      // TODO: Necesito darle Feedback a la app de que se recibió este evento
      
      response["event_name"] = "/set_information_response";
      //response["type_of_device"] = typeOfDevice;

      //TODO: enviar por WS este response
      
    }

  
    if(eventName == "/set_credentials_wifi"){
      // TODO: Debo generar conexión a servidor MQTT y suscribirme a un tópico

      //ssid = doc["message"]["ssid"];
      //password = doc["message"]["password"];

      // Una vez recibimos la ssid y password desde App, intentamos conectarnos a internet
      //setupWifi(ssid, password);

      // TODO: Necesito darle Feedback a la app de que se recibió este evento
      response["event_name"] = "/set_credentials_wifi_response";
      response["type_of_device"] = typeOfDevice;

      //TODO: enviar por WS este response
    }
    

    // TODO: Este evento lo pide la app para tener los datos del dispositivo en ella
    if(eventName == "/device_information"){
      
      // TODO: Necesito darle Feedback a la app de que se recibió este evento
      response["event_name"] = "/device_information_response";
      response["on"] = true;
      //response["room_name"] = roomName;
      response["type_of_device"] = typeOfDevice;
      //TODO: enviar por WS este response

    }

    serializeJson(response, responseBuffer);
    // ENVIAMOS RESPONSE GENERADO A LA APP
    webSocket.sendTXT(num, responseBuffer);
 }


 void callback(char *topic, byte *payload, unsigned int length) {
 Serial.print("Message arrived in topic: ");
 Serial.println(topic);
 Serial.print("Message:");
 for (int i = 0; i < length; i++) {
     Serial.print((char) payload[i]);
 }
 Serial.println();
 Serial.println("-----------------------");
}
  

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            Serial.printf("[%u] Disconnected!\n", num);
            break;
        case WStype_CONNECTED: {
            IPAddress ip = webSocket.remoteIP(num);
            Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);

            // send message to client
            webSocket.sendTXT(num, "Connected");
        }
            break;
        case WStype_TEXT:
            Serial.printf("[%u] get Text: %s\n", num, payload);
            char* json = (char*)payload;
            
            // Decodificamos el mensaje desde WS
            decodeMessage(num, json);
      break;
    }
}

void setupWifi(const char* ssid,const char* password){
    int count = 0;
    bool accessPointMode = false;
  
    WiFi.mode(WIFI_STA); //para que no inicie el SoftAP en el modo normal  
    Serial.printf("Intentando SSID %s y password %s", ssid, password);

    // We start by connecting to a WiFi network
    WiFi.begin(ssid, password);
    
    while(WiFi.status() != WL_CONNECTED && !accessPointMode) {
        Serial.printf("Waiting for WiFi... %d segundos", count);
        Serial.println();
        delay(1000);
        
        count++;

      if(count == 10) {
        accessPointMode = true;
      }
    } 

    
   if(!accessPointMode){
      Serial.println("");
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());

      delay(500);

      // Una vez que sabemos que hay internet, intentamos conectarnos
      connectToMqtt();
    }else {
      
    setupAccessPoint();
      
    }

}

void setupAccessPoint(){
  // Connect to Wi-Fi network with SSID and password
  Serial.print("Configurando (Access Point)…");
  // Remove the password parameter, if you want the AP (Access Point) to be open
  WiFi.softAP(APssid, APpassword);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
}


void connectToMqtt(){
  
 //connecting to a mqtt broker
 client.setServer(mqtt_broker, mqtt_port);
 client.setCallback(callback);
 int retryConnections = 0;
 
 while (!client.connected() && retryConnections != 4) {
     String client_id = "esp32-client-";
     client_id += String(WiFi.macAddress());
     Serial.printf("The client %s connects to the public mqtt broker\n", client_id.c_str());
     if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
         Serial.println("Public emqx mqtt broker connected");
     } else {
         retryConnections++;
         Serial.print("failed with state ");
         Serial.print(client.state());
         delay(2000);
     }
 }

 if(retryConnections != 4) {
   Serial.print("Suscribiendome y publicando en topico ");
   Serial.print(topic);
 
 // publish and subscribe
 client.publish(topic, "Hola desde ESP32 !!", MQTTpubQos);
 client.subscribe(topic);
 }
  
}

void setup()
{
    Serial.begin(115200);
    delay(10);

    // Los pines a los que se va a apuntar deben guardarse en la memoria eeprom 
    pinMode(rele, OUTPUT);
    pinMode(2,OUTPUT);

    setupWifi(ssid, password);

    // Vamos a intentar conectarnos en un inicio
    // connectToMqtt();
    
    // start webSocket server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}


void loop()
{
   client.loop();
   webSocket.loop();
}
