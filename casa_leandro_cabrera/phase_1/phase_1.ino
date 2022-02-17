#define tecla1 19
#define tecla2 18
#define rele 23

void setup() {
  Serial.begin(115200);
  pinMode(tecla1, INPUT_PULLUP);
  pinMode(tecla2, INPUT);
  pinMode(rele, OUTPUT);
}

void loop() {
  int teclaValue = digitalRead(tecla1);   
  //TODO: Hacer la combinacion con las 2 teclas para una luz
  //TODO: Si esta encendido el foco, al apretar cualquiera de las dos teclas, se apaga y viceversa
  
  if( teclaValue == 1){
    digitalWrite(rele, HIGH);
    digitalWrite(2, HIGH);
  }else{
   digitalWrite(rele, LOW);
   digitalWrite(2, LOW); 
  }
  
  Serial.println("Imprimimos valor de la tecla");  
  Serial.println(teclaValue);
  delay(500);
}
