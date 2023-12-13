//Variables para el serial
String inputString = ""; // variable para almacenar la entrada serial
bool validInput = false; // variable para verificar si se ingresó un valor válido

//Variables para la balanza
#include "HX711.h"        // Modulo HX711 - Celda de carga
#define DOUT  A1
#define CLK  A0
float pesoRecipiente = 0;
HX711 balanza(DOUT, CLK);
float peso;

//Variables para la mini bomba de agua
bool pumpRunning = false;
unsigned long previousMillis = 0;
int Input1 = 9;    // Control pin 1 for motor 1
int Input2 = 10;     // Control pin 2 for motor 1
int Densidad = 1;// en g/mL | MODIFIQUE PREVIAMENTE CON EL VALOR DE DENSIDAD CORRESPONDIENTE!!!
int Volumen_deseado = 0;
int OnOff = 1; // Switch On/Off para la bomba (1 = On | 0 = Off)
int Adelante; // Variable PWM 1
int Atras; // Variable PWM 2
int Detener = map(0, 0, 1023, 0, 255);
int Var = 600;
int x = 1;

// Puertos y Variables para Motor paso a paso
#define StepPin 4
#define ms1 5
#define ms2 6
#define ms3 7

unsigned long int t = 980; // Rango: 980 - 9800 que corresponde a 150 - 15 RPMs aprox.

// Puertos y Variables para Encoder
#define Encoder_pin 2      // The pin the encoder is connected 
#define PulsosRotacion 20  // Value of pluses per rotation

int Encoder = 0;           // Value of the Output of the encoder 
int Estado = 1;            // Current state of the encoder to recognize the pulse changes
int Pulsos = 0;            // Counter of pulses
unsigned long InitialTime = 0;
unsigned long FinalTime; 
float RPMs;

void setup() {
  Serial.begin(9600);
  CalibracionBalanza();
  Serial.println(" ");
  Serial.println("¡¡¡LISTO PARA PESAR!!!");
  Serial.println(" ");
  Serial.println(" ");

  pinMode( ms1 , OUTPUT);
  pinMode( ms2 , OUTPUT);
  pinMode( ms3 , OUTPUT);
  pinMode( StepPin , OUTPUT);
  pinMode( Encoder_pin , INPUT);

  ////// Motor paso a paso //////
  
  ControlMotorPasoaPaso();
}

void loop() {

  if (OnOff == 1) {
    Serial.println(" ");
    Serial.println("¡¡¡LISTO PARA DOSIFICAR!!!");
    Serial.println(" ");
    Serial.println("Eche el frutiño");
    Serial.println(" ");
    Serial.println(" ");
    delay(1000);

    DosificarB(Volumen_deseado); //Dosificación de volumen según Balanza
   
  }

  // leer entrada serial y actualizar Volumen_deseado
  if (Serial.available() > 0) {
    char inChar = (char)Serial.read();
  if (inChar == '\n') {
    // se ha ingresado una línea completa
    if (inputString.length() > 0 && inputString.startsWith("V=")) {
      // se ha ingresado un valor válido
      Volumen_deseado = inputString.toInt();
      Serial.print("Volumen deseado actualizado a ");
      Serial.println(Volumen_deseado);
      validInput = true;
    }
    inputString = "";
  }
  else if (isDigit(inChar)) {
    inputString += inChar;
  }
  }

  // dosificar si se ingresó un valor válido
  if (validInput) {
    DosificarB(Volumen_deseado); //Dosificación de volumen según Balanza
    validInput = false;
  }

 motorPasoPaso();
  
}


//Función de Medición de Balanza: Permite obtener la medida actual en peso (g) de la balanza
float MedidaBalanza() {
  float peso = balanza.get_units(1)*1000;
  peso = -peso/1.0004 - 0.2183/1.0004;
  if (peso < 0) {
    peso = 0;
  }
  return peso;
}

void motorPasoPaso(){
  // Check if there is data available in the serial buffer  
  if (Serial.available() > 0) {
    String inputString = Serial.readStringUntil('\n');
    if (inputString.startsWith("rpm=")){
      int newInputValue = inputString.substring(4).toInt();
      int rpm = newInputValue;
      float exponent = -1/1.005;
      float base = rpm / 278137.0;
      t = static_cast<unsigned long>(pow(base, exponent));
      Serial.print("El valor se cambió a: ");
      Serial.println(rpm);
    }
  }
  
  ////// Motor paso a paso //////
    
  digitalWrite(StepPin, HIGH); 
  delayMicroseconds(t);           
  digitalWrite(StepPin, LOW);  
  delayMicroseconds(t);

  ////// Encoder //////
  Encoder = digitalRead(Encoder_pin);  

  if(Encoder == LOW && Estado == 1) {
     Estado = 0;             
  }

  if(Encoder == HIGH && Estado == 0) {
     Pulsos++; 
     Estado = 1;     
  }

  if(Pulsos == PulsosRotacion) {
    FinalTime = millis();
    RPMs = 60000/(FinalTime - InitialTime);
    Pulsos = 0; 
    InitialTime = FinalTime;
    peso = MedidaBalanza(); //Medición actual de la Balanza
    Serial.print("RPM = ");
    Serial.println(RPMs); 
    Serial.print("peso = ");
    Serial.println(peso); 
  }
}

  
void CalibracionBalanza(void){
  Serial.begin(9600);
  balanza.begin(DOUT, CLK);
  Serial.println("No ponga ningun  objeto sobre la balanza");
  Serial.println("Destarando...");
  Serial.println("...");
  balanza.set_scale(1116125); // Establecemos la escala
  balanza.tare(20);  //El peso actual es considerado Tara.
  
  Serial.println("Listo para pesar");   
  
}

 
//Función de DosificarB: Permite entregar un volumen deseado(mL) según la medida actual de peso (g) de la balanza
void DosificarB(int Volumen_deseado) {

  int Tiempo_deseado = (Volumen_deseado - 2.24)/0.0142;
  const long interval = Tiempo_deseado;

  while (x == 1) {    
    motorPasoPaso();
    if (!pumpRunning) {
      pumpRunning = true;
      OnOff = 0;
      Adelante = map(Var, 0, 1023, 0, 255);
      analogWrite(Input1, Adelante);
      previousMillis = millis();
    }

    if (pumpRunning && millis() - previousMillis >= interval) {
      pumpRunning = false;
      analogWrite(Input1, Detener);
      previousMillis = millis(); // Actualizar previousMillis al apagar la bomba
      x = 0;
    }
    else if (!pumpRunning && millis() - previousMillis >= interval) {
      previousMillis = millis(); // Actualizar previousMillis al encender la bomba
      pumpRunning = true;
      OnOff = 0;
      Adelante = map(Var, 0, 1023, 0, 255);
      analogWrite(Input1, Adelante);
    }
  }
}

//Función para Controlar el Motor Paso a Paso: Permite controlar el motor y los pasos de este
void ControlMotorPasoaPaso(void) {
  Serial.println("Motor Control");

  digitalWrite(ms1, LOW);
  digitalWrite(ms2, LOW);
  digitalWrite(ms3, LOW);
//  digitalWrite(DirPin, LOW);
}