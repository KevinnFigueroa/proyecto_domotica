/*

<--- ESP SOLO PARA CONTROL DE LUCES --->

*/

#include <WiFi.h>
#include <WebSocketsServer.h>
#include <string.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>

#define MQTTpubQos 2 

/* --- RELES --- */

#define releSimple 13

#define releSimple2 27

#define releSimple3 32

#define releSimple4 26

#define releCombinadaDoble 25

/* --- FIN RELES --- */


/* --- TECLAS --- */

// TECLA SIMPLE
#define teclaSimple 14

// TECLA SIMPLE 2
#define teclaSimple2 17

// TECLA SIMPLE 3
#define teclaSimple3 19

// TECLA SIMPLE 4
#define teclaSimple4 21

// TECLA COMBINADA DOBLE
#define teclaCombinadaDoble 16
#define teclaCombinadaDoble2 18

/* --- FIN TECLAS --- */


/* --- ESTADOS ANTERIORES DE LAS TECLAS  --- */

// ESTADOS ANTERIORES DE LAS TECLAS COMBINADAS DOBLES
int beforeStateTeclaDobleValue1 = 0;
int beforeStateTeclaDobleValue2 = 0;

/* --- FIN ESTADOS ANTERIORES DE LAS TECLAS  --- */

WiFiClient espClient;
PubSubClient client(espClient);
WebSocketsServer webSocket = WebSocketsServer(81);


// Esto debe guardarse en la memoria EEPROM para restaurar internet en caso de corte de luz
const char* ssid = "";
const char* password = "";

// MQTT Broker
const char *mqtt_broker = "192.168.0.16";
const char *topic = "";
const char *mqtt_username = "esp32/test";
const char *mqtt_password = "public";
const char *will_topic = "";
const int mqtt_port = 1883;

// Esto debe tener un id Unico para identificar como red unica
const char* APssid = "Access Point Esp32";
const char* APpassword = "12345678";

// IDENTIDAD DEL DISPOSITIVO, Esto se va a devolver cada vez que la app se sincronice por WS llamando al evento /device_information
// TODO:esto debe guardarse en la memoria EEPROMM 
const char* user = "";
const char* room = "";

const char* topicMqttDevice = "";

// Esto vendrá hardcodeado dependiendo del dispositivo
const char* typeOfDevice = "light";

char* configurationDevice = "configuration_device/";

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
          
      ssid = doc["message"]["ssid"];
      password = doc["message"]["password"];
      
      topic = doc["message"]["topic_mqtt_device"];
      user = doc["message"]["user"];
      room = doc["message"]["room"];
      
      // Este es el ultimo evento de la configuracion de dispositivo desde la APP, para este entonces, ya tendré cargado los datos del ssid y password en la placa y solo debo conectarme
      
      // TODO: ARREGLAR EL POR QUE ME ESTA MOSTRANDO PARTE DE OTRAS VARIABLES CUANDO QUIERO PASARLE EL SSID Y PASSWORD PARA CONFIGURAR WIFI
       setupWifi(ssid, password);

      // TODO: UNA VEZ QUE ME CONECTÉ A INTERNET, NECESITO SUSCRIBIRME AL TOPICO QUE ME DA LA APP Y GUARDAR ESE VALOR EN LA MEMORIA EEPROM IGUAL QUE LA PASSWORD Y SSID DEL WIFI
      response["event_name"] = "/set_information_response";
      
    }
 
    if(eventName == "/device_information"){
      
      // TODO: Necesito darle Feedback a la app de que se recibió este evento
      response["event_name"] = "/device_information_response";
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
  
  String topicStr = topic; 
  String stringJson = String(( char *) payload);
 
  // Leer Json
  DynamicJsonDocument doc(1024);
  deserializeJson(doc, stringJson);

  // Responder Json
  DynamicJsonDocument response(1024);
  char responseBuffer[100];

 for (int i = 0; i < length; i++) {
     Serial.print((char) payload[i]);
 }

 String eventName = doc["event_name"];  

 if (eventName == "/change_state"){
  int state = doc["message"]["state"];

  if(state == 1){
    digitalWrite(2, HIGH);          
   } else if(state == 0 ){    
    digitalWrite(2, LOW);
   }

    response["event_name"] = "/change_state_response";
    response["topic_mqtt_device"] = topicStr;
    response["state"] = digitalRead(2); 
 
 }

 if (eventName == "/finish_configuration"){
  // Este evento no nos interesa
  return;
 }

 serializeJson(response, responseBuffer);
 client.publish(topic, responseBuffer, MQTTpubQos);

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

     // Responder Json
    DynamicJsonDocument response(1024);
    char responseBuffer[250];
  
    response["user"] = user;
    response["device"]["room"] = room;
    response["device"]["type_of_device"] = typeOfDevice;
    response["device"]["topic_mqtt_device"] = topic;

   serializeJson(response, responseBuffer); 

   // TODO: será que no puedo enviar el mensaje desde el esp porque es muy largo el topico??

   // Este evento lo va a recibir el backend y va a generar una bandera de este dispositivo para luego avisar a la app
   client.publish(configurationDevice, responseBuffer, MQTTpubQos);
   client.subscribe(topic);
 }
  
}

void setup()
{

    Serial.begin(115200);
    delay(10);

    pinMode(teclaSimple, INPUT_PULLUP);

    pinMode(teclaSimple2, INPUT_PULLUP);

    pinMode(teclaSimple3, INPUT_PULLUP);

    pinMode(teclaSimple4, INPUT_PULLUP);

    
    pinMode(teclaCombinadaDoble, INPUT_PULLUP);
    pinMode(teclaCombinadaDoble2, INPUT_PULLUP);
    
    // Los pines a los que se va a apuntar deben guardarse en la memoria eeprom 
    pinMode(releSimple, OUTPUT);

    pinMode(releSimple2, OUTPUT);

    pinMode(releSimple3, OUTPUT);

    pinMode(releSimple4, OUTPUT);

    pinMode(releCombinadaDoble, OUTPUT);
    
  
    // TOMO TODOS LOS VALORES QUE TIENEN LAS TECLAS AL ENCENDER EL ESP32

    // ESTADOS ANTERIORES DE LAS TECLAS COMBINADAS DOBLES
    beforeStateTeclaDobleValue1 = digitalRead(teclaCombinadaDoble);
    beforeStateTeclaDobleValue2 = digitalRead(teclaCombinadaDoble2);
    
    //setupWifi(ssid, password);

    // Vamos a intentar conectarnos en un inicio
    // connectToMqtt();
    
    // start webSocket server
    //webSocket.begin();
    //webSocket.onEvent(webSocketEvent);
}

void readLightKeys(){
  // Agregar MQTT A TODOS ESTOS CAMBIOS DE ESTADO, ESCRIBIENDO A LOS TOPICOS

  // Tecla simple 
  
  int teclaSimpleValue = digitalRead(teclaSimple);
  
  if(teclaSimpleValue == 1){
      digitalWrite(releSimple, LOW);  
   } else {
      digitalWrite(releSimple, HIGH);
   }  

  // Tecla simple 2
  
  int teclaSimpleValue2 = digitalRead(teclaSimple2);
  
  if(teclaSimpleValue2 == 1){
      digitalWrite(releSimple2, LOW);  
   } else {
      digitalWrite(releSimple2, HIGH);
   }  

  // Tecla simple 3
  
  int teclaSimpleValue3 = digitalRead(teclaSimple3);
  
  if(teclaSimpleValue3 == 1){
      digitalWrite(releSimple3, LOW);  
   } else {
      digitalWrite(releSimple3, HIGH);
   }  

  // Tecla simple 
  
  int teclaSimpleValue4 = digitalRead(teclaSimple4);
  
  if(teclaSimpleValue4 == 1){
      digitalWrite(releSimple4, LOW);  
   } else {
      digitalWrite(releSimple4, HIGH);
   }  
  
   
  // Tecla combinada doble

  int teclaDoble1Value = digitalRead(teclaCombinadaDoble);
  int teclaDoble2Value = digitalRead(teclaCombinadaDoble2);

  if(teclaDoble1Value != beforeStateTeclaDobleValue1 || teclaDoble2Value != beforeStateTeclaDobleValue2){
    beforeStateTeclaDobleValue1 = digitalRead(teclaCombinadaDoble);
    beforeStateTeclaDobleValue2 = digitalRead(teclaCombinadaDoble2);

    
    // Leo en qué estado está la luz
    int val = digitalRead(releCombinadaDoble);
    
    // Cambio al estado contrario al que estaba
    if(val == 1){
      digitalWrite(releCombinadaDoble, LOW);  
    }else{
      digitalWrite(releCombinadaDoble, HIGH);
     }          
    
   }

}


void loop()
{
   // Control Manual de luces
   readLightKeys();
   
   // TODO: aqui deberia ir preguntando en un futuro el estado de la conexion a internet, si está conectado entonces prendemos led VERDE y sino EN ROJO.
   
   //client.loop();
   //webSocket.loop();
}
