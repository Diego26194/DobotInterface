// ROS
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>


static bool rosConnected = false;

//#include <msg_estados.h>

//********Definiciones del Servo ************************************
#include <Servo.h>
Servo Pinza;
#define ABIERTO_MAX 10
#define CERRADO_MAX 120


//Botornera
// Definiciones de pines
//Botonera de la base
const int pullME = 11;//  Pull modo Escritura
const int pullML = 12;//  Pull modo Lectura
const int pullR = 10;//  Pull Correr Rutina
const int pullT_P =9;//   Puññ Modos Trayrcyoia-Punto 

const int ledME = 7;// led modo Escritura 
const int ledML = 6;//  Led modo lectura 
const int ledR = 5;//pin 8 esta roto  9 Led correr rutina 
const int ledSMP = 4;//50  Led modo Punto 
const int ledSMT = 3;//52 Led modo trayectoria 

//Botonera sobre el robot
const int pullD =40;//    Pull Desenclavar 
const int pullGD = 42;//   Pull Guardar Dato 
const int ledGD = 44;// Led guardar dato 
const int ledD = 46;//  led Desacnclar motores 

// Estados iniciales
bool estadoModo = true;  //modo lectura(false)- modo control(true) 
bool estadoRutina = false; //bandera,corriendo rutina 
bool estadoSub_Modo = true;  //punto a punto-tomar trayectoria completa(modo lectura)
bool estadoParada = true;  //estado para parada de emergencia

bool pulso = false;
unsigned long lastDebounceTime = 0; // Última vez que el pin cambió
unsigned long debounceDelay = 50; // Tiempo de debounce en milisegundos
bool lastPull4State = false; // Estado anterior del botón
bool pulso_ant = false;
bool enclavar =true;
bool estado_ant3 = true;  //modo lectura(false)- modo control(true)
bool estado_ant5 = true;  //punto a punto-tomar trayectoria completa(modo lectura)

bool valAnterior=false;

unsigned long t6 = 0;
const unsigned long tpull6 = 50;  //tiempo en ms

unsigned long tpublespera= 0;
const unsigned long tpublic_escritura = 50;  //tiempo en ms


unsigned long tiempoInicio = 0; // Variable para almacenar el tiempo de inicio
const int tiempoEspera = 2000; // Tiempo de espera en milisegundos (2 segundos)



/////7
//************ Definiciones para uso de libreria Dynamixel **********************************
#include <Dynamixel2Arduino.h>

#define DXL_SERIAL   Serial1
const uint8_t DXL_DIR_PIN = 2;   
const uint8_t DXL_ID1 = 1;
const uint8_t DXL_ID2 = 2;
const uint8_t DXL_ID3 = 3;
const uint8_t DXL_ID4 = 4;
const uint8_t DXL_ID5 = 5;
const uint8_t DXL_ID6 = 6;


const uint8_t DXL_ID7 = 7;

int a=0;
int a2=0;
int b=0;
int c=0;
int positionMotor5=500;
int positionMotor6=500;

unsigned long te1=0;
unsigned long te2=0;
unsigned long te3=0;
unsigned long te4=0;
unsigned long te5=0;
unsigned long te6=0;
unsigned long te7=0;

unsigned long timeref=0;
unsigned long timeout=0;

int16_t de1=0;
int16_t de2=0;
int16_t de3=0;
int16_t de4=0;
int16_t de5=0;
int16_t de6=0;
int16_t det=0;
int contadorErrores1=0;
int contadorAciertos1=0;
int contadorAciertos2=0;
int msgResibidos=0;
int contadorPublicaciones=0;
float prueba=0;

float conversionDynamixelsCord= 0.0088*0.29;


Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
//This namespace is required to use Control table item names
using namespace ControlTableItem;

//********** configuraciones de ROS *******************************
ros::NodeHandle nh;
std_msgs::Int16MultiArray dato;
std_msgs::Int16MultiArray dato2;
// ROS logging
//char g_logStr[200] = {0};
//void LogInfo (const char* str) { nh.loginfo(str); }
//void LogInfo1(const char* str) { strcat(g_logStr, str); }

#define POSITIONS_PUBLISH_FREQUENCY    2   //hz

//************ Definicion de publicadores y subscriptores **************************
void llama(const std_msgs::Int16MultiArray& msg) ;

ros::Subscriber<std_msgs::Int16MultiArray> sub("cord_dy", &llama);


void modo_funcionamiento(const std_msgs::Bool& msg) ;

ros::Subscriber<std_msgs::Bool> sub_modo_funcionamiento("modo_actuar", &modo_funcionamiento);


void cambiar_modo_lectura(const std_msgs::Bool& msg) ;

ros::Subscriber<std_msgs::Bool> sub_tipo_modo_lectura("tipo_modo_lectura", &cambiar_modo_lectura);


void modificar_pid(const std_msgs::Int16MultiArray& msg) ;

ros::Subscriber<std_msgs::Int16MultiArray> sub_mod_pid("mod_pid", &modificar_pid);




//std_msgs::String str_msg;
//ros::Publisher chatter("chatter", &str_msg);
std_msgs::Int16MultiArray poscion;
ros::Publisher pos_dy("pos_dy", &poscion);

std_msgs::Int16MultiArray cErrores;
ros::Publisher erroresA("erroresA", &cErrores);


std_msgs::Int16MultiArray punto;
ros::Publisher p_dy("p_dy", &punto);

std_msgs::Bool bool_msg;
ros::Publisher rutina("rutina", &bool_msg);

std_msgs::Bool modo_msg;
ros::Publisher modo_actuar("modo_actuar", &modo_msg);

std_msgs::Bool m_lectura_msg;
ros::Publisher tipo_modo_lectura("tipo_modo_lectura", &m_lectura_msg);

std_msgs::Bool guardarT_msg;
ros::Publisher guardar_trayectoria("guardar_trayectoria", &m_lectura_msg);

std_msgs::Bool stop_msg;
ros::Publisher parada_emergencia("parada_emergencia", &stop_msg);

std_msgs::Bool clean_msg;
ros::Publisher limpieza_rutina("limpieza_rutina", &clean_msg);

//ros::Publisher estado_pub("estado", &estado_msg);


//void estadoleer(const std_msgs::BoolMultiArray& estado);
//ros::Subscriber<std_msgs::BoolMultiArray> estado_sub("estado", &estadoleer);

//*********** Funciones ROS ****************************************
boolean g_motorsEnabled = false;
/*
int rad_bit(float rad){
  return int((rad +3.1416) * (4095.0 / (2 * 3.1416)));
}
int bit_rad(int bit){
  return float (bit) * (2 * 3.1416) / 4095.0  -3.1416;
}
*/
void modo_funcionamiento(const std_msgs::Bool& msg) {
  estadoModo = msg.data;  // Accede al valor booleano dentro de msg
}

void cambiar_modo_lectura(const std_msgs::Bool& msg) {
  estadoSub_Modo = msg.data;  // Accede al valor booleano dentro de msg
}

void llama(const std_msgs::Int16MultiArray& msg) {
  if (estadoModo && !estadoParada){
    //contadorAciertos2=contadorAciertos2 + 1;
    if (msg.data_length != 6)
    {
        nh.loginfo("Tamaño de dato publicado incorrecto");
        //contadorErrores1=contadorErrores1 + 1;
        return; 
    }
    
    //dato = msg;
    //a = a + 1;
    //prueba= (dato.data[0]);
  /*
    dxl.setGoalPosition(DXL_ID1, dato.data[0] ,UNIT_RAW);
    dxl.setGoalPosition(DXL_ID2, dato.data[1]-1023,UNIT_RAW);
    dxl.setGoalPosition(DXL_ID3, dato.data[2],UNIT_RAW);
    dxl.setGoalPosition(DXL_ID4, dato.data[3],UNIT_RAW);
    dxl.setGoalPosition(DXL_ID5, dato.data[4]/4,UNIT_RAW);
    dxl.setGoalPosition(DXL_ID6, dato.data[5]/4,UNIT_RAW);
*/
    //te1 = millis();
    dxl.setGoalPosition(DXL_ID1, msg.data[0],UNIT_RAW);
    //te2 = millis();
    dxl.setGoalPosition( DXL_ID7, 4095 -msg.data[1],UNIT_RAW);
    //te2 = millis();
    dxl.setGoalPosition(DXL_ID2, msg.data[1],UNIT_RAW);
    //te3 = millis();
    dxl.setGoalPosition(DXL_ID3, msg.data[2],UNIT_RAW);
    //te4 = millis();
    dxl.setGoalPosition(DXL_ID4, msg.data[3],UNIT_RAW);

    
    //a = dxl.getPresentPosition(DXL_ID3, UNIT_RAW);
    //b = dxl.getPresentPosition(DXL_ID2, UNIT_RAW);


    //te5 = millis();
    //dxl.setGoalPosition(DXL_ID5, msg.data[4],UNIT_RAW);
    positionMotor5=msg.data[4];
    //te6 = millis();
    //dxl.setGoalPosition(DXL_ID6, msg.data[5],UNIT_RAW);
    positionMotor6=msg.data[5];
    //te7 = millis();

    //contadorAciertos1=contadorAciertos1 + 1;
  }
}


void modificar_pid(const std_msgs::Int16MultiArray& msg) {
  if (msg.data[0]==1){
    dxl.torqueOff(DXL_ID1);
    dxl.writeControlTableItem(POSITION_P_GAIN, DXL_ID1, msg.data[1]);
    dxl.writeControlTableItem(POSITION_I_GAIN, DXL_ID1, msg.data[2]);
    dxl.writeControlTableItem(POSITION_D_GAIN, DXL_ID1, msg.data[3]);
    dxl.writeControlTableItem(FEEDFORWARD_1ST_GAIN, DXL_ID1, msg.data[4]);
    dxl.writeControlTableItem(FEEDFORWARD_2ND_GAIN, DXL_ID1, msg.data[5]);
    dxl.torqueOn(DXL_ID1);
  }
  else if(msg.data[0]==2){
    dxl.torqueOff(DXL_ID2);
    dxl.writeControlTableItem(POSITION_P_GAIN, DXL_ID2, msg.data[1]);
    dxl.writeControlTableItem(POSITION_I_GAIN, DXL_ID2, msg.data[2]);
    dxl.writeControlTableItem(POSITION_D_GAIN, DXL_ID2, msg.data[3]);
    dxl.writeControlTableItem(FEEDFORWARD_1ST_GAIN, DXL_ID2, msg.data[4]);
    dxl.writeControlTableItem(FEEDFORWARD_2ND_GAIN, DXL_ID2, msg.data[5]);
    dxl.torqueOn(DXL_ID2);

    dxl.torqueOff(DXL_ID7);
    dxl.writeControlTableItem(POSITION_P_GAIN, DXL_ID7, msg.data[1]);
    dxl.writeControlTableItem(POSITION_I_GAIN, DXL_ID7, msg.data[2]);
    dxl.writeControlTableItem(POSITION_D_GAIN, DXL_ID7, msg.data[3]);
    dxl.writeControlTableItem(FEEDFORWARD_1ST_GAIN, DXL_ID7, msg.data[4]);
    dxl.writeControlTableItem(FEEDFORWARD_2ND_GAIN, DXL_ID7, msg.data[5]);
    dxl.torqueOn(DXL_ID7);
  }
  else if(msg.data[0]==3){
    dxl.torqueOff(DXL_ID3);
    dxl.writeControlTableItem(POSITION_P_GAIN, DXL_ID3, msg.data[1]);
    dxl.writeControlTableItem(POSITION_I_GAIN, DXL_ID3, msg.data[2]);
    dxl.writeControlTableItem(POSITION_D_GAIN, DXL_ID3, msg.data[3]);
    dxl.writeControlTableItem(FEEDFORWARD_1ST_GAIN, DXL_ID3, msg.data[4]);
    dxl.writeControlTableItem(FEEDFORWARD_2ND_GAIN, DXL_ID3, msg.data[5]);
    dxl.torqueOn(DXL_ID3);
  }
}


/*
void estadoleer(const std_msgs::BoolMultiArray& estado) {
  if (estado.data_length >= 5) {
        estadoModo = estado.data[2];  // El tercer elemento del vector (índice 2)
        estadoRutina = estado.data[3];  // El cuarto elemento del vector (índice 3)
    }
}
*/


/*
void posCallback(const std_msgs::Int16MultiArray& msg)
{

    dato2 = msg;
    //c = c + 1;
}
*/
void publishPositions(void)
{
  /*String message =// String(dato.position[0], 2) +
                   /*", b=" + String(dato.position[1], 2) +
                   ", c=" + String(dato.position[2], 2) +
                   ", d=" + String(dato.position[3], 2) +
                   ", e=" + String(dato.position[4], 2) +
                   ", f=" + String(dato.position[5], 2) +
                   ", " + String(a) +
                   ", " + String(b) +
                   ", " + String(c);

  str_msg.data = message.c_str();
  chatter.publish(&str_msg);
  */
  //if (dato.data_length >= 6) {
  poscion.data_length = 6; // Definir la longitud del array de datos
  int16_t poscionData[6];

  
  poscionData[0] = dxl.getPresentPosition(DXL_ID1, UNIT_RAW);
  poscionData[1] = dxl.getPresentPosition(DXL_ID2, UNIT_RAW);
  poscionData[2] = dxl.getPresentPosition(DXL_ID3, UNIT_RAW);
  poscionData[3] = dxl.getPresentPosition(DXL_ID4, UNIT_RAW);
  //poscionData[4] = dxl.getPresentPosition(DXL_ID5, UNIT_RAW);
  poscionData[4]=positionMotor5;
  //poscionData[5] = dxl.getPresentPosition(DXL_ID6, UNIT_RAW);
  poscionData[5]=positionMotor6;
   
  poscion.data = poscionData; // Asignar memoria para el array de datos

  pos_dy.publish(&poscion); // Publicar el mensaje


  //Comprobacion de msjs BORRAR
  /*
  contadorPublicaciones=contadorPublicaciones+1;
  if(contadorPublicaciones>20){
    contadorPublicaciones=0;
    //cErrores.data_length = 3;
    /*int16_t erroresData[3];
    erroresData[0] = contadorErrores1;
    erroresData[1] = contadorAciertos1;
    erroresData[2] = contadorErrores1+contadorAciertos1;
    cErrores.data = erroresData;
    cErrores.data_length = 3;

    erroresA.publish(&cErrores);
    */

    /*
    de6=(int16_t)(te7-te6);
    de5=(int16_t)(te6-te5);
    de4=(int16_t)(te5-te4);
    de3=(int16_t)(te4-te3);
    de2=(int16_t)(te3-te2);
    de1=(int16_t)(te2-te1);
    det=(int16_t)(te7-te1);
    int16_t erroresData[] = {contadorErrores1, contadorAciertos1, de1,de2,de3,de4,de5,de6,det};
    cErrores.data = erroresData;
    cErrores.data_length = sizeof(erroresData)/sizeof(erroresData[0]);
    erroresA.publish(&cErrores);
  }
*/

}

void publishPosdy(void)
{
  poscion.data_length = 6; // Definir la longitud del array de datos
  int16_t poscionData[6];

  
  poscionData[0] = dxl.getPresentPosition(DXL_ID1, UNIT_RAW);
  poscionData[1] = dxl.getPresentPosition(DXL_ID2, UNIT_RAW);
  poscionData[2] = dxl.getPresentPosition(DXL_ID3, UNIT_RAW);
  poscionData[3] = dxl.getPresentPosition(DXL_ID4, UNIT_RAW);
  //poscionData[4] = dxl.getPresentPosition(DXL_ID5, UNIT_RAW);
  poscionData[4]=positionMotor5;
  //poscionData[5] = dxl.getPresentPosition(DXL_ID6, UNIT_RAW);
  poscionData[5]=positionMotor6;
   
  poscion.data = poscionData;

  p_dy.publish(&poscion); // Publicar el mensaje
}
  
void setup()
{

  //Botonera

  // Inicializar pines de entrada para los pulsadores
  pinMode(pullME, INPUT_PULLUP);
  pinMode(pullML, INPUT_PULLUP);
  pinMode(pullR, INPUT_PULLUP);
  pinMode(pullGD, INPUT_PULLUP);
  pinMode(pullD, INPUT_PULLUP);
  pinMode(pullT_P, INPUT_PULLUP);
  
  // Inicializar pines de salida para los LEDs
  pinMode(ledME, OUTPUT);
  pinMode(ledML, OUTPUT);
  pinMode(ledR, OUTPUT);
  pinMode(ledGD, OUTPUT);
  pinMode(ledD, OUTPUT);
  pinMode(ledSMT, OUTPUT);
  pinMode(ledSMP, OUTPUT);
  

  //


    //******* Nodos publicadores y subscriptores *************
    //g_logStr[0] = 0;

    nh.getHardware()->setBaud(115200);
    nh.initNode();

  // Suscriptores
  nh.subscribe(sub);
  nh.subscribe(sub_modo_funcionamiento);
  nh.subscribe(sub_tipo_modo_lectura);
  nh.subscribe(sub_mod_pid);

  // Publicadores
  
  nh.advertise(erroresA);
  nh.advertise(pos_dy);
  nh.advertise(p_dy);
  nh.advertise(rutina);
  nh.advertise(modo_actuar);
  nh.advertise(tipo_modo_lectura);
  nh.advertise(guardar_trayectoria);
  nh.advertise(parada_emergencia);  
  nh.advertise(limpieza_rutina);  
  //nh.advertise(estado_pub);
  //nh.subscribe(estado_sub);

    //Inicializamos configuracion de Dynamixel
    
    DXL_SERIAL.begin(1000000);
    dxl.begin(1000000);
    //DXL_SERIAL.begin(57600);
    //dxl.begin(57600);
    dxl.setPortProtocolVersion(2.0);

    //XL430 - Eje 4
    dxl.torqueOff(DXL_ID1);
    dxl.setOperatingMode(DXL_ID1, OP_POSITION);
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID1, 120);//265);   41 por cada rad/seg
    /*
    dxl.writeControlTableItem(POSITION_P_GAIN, DXL_ID1, 640);
    dxl.writeControlTableItem(POSITION_I_GAIN, DXL_ID1, 0);
    */
    dxl.torqueOn(DXL_ID1);

    
    //XL430 - Eje 2
    dxl.torqueOff(DXL_ID2);
    dxl.setOperatingMode(DXL_ID2, OP_POSITION);
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID2, 120);//265);
    /*
    dxl.writeControlTableItem(POSITION_P_GAIN, DXL_ID2, 800);
    dxl.writeControlTableItem(POSITION_I_GAIN, DXL_ID2, 700);
    dxl.writeControlTableItem(POSITION_D_GAIN, DXL_ID2, 15000);
    dxl.writeControlTableItem(FEEDFORWARD_1ST_GAIN, DXL_ID2, 950);
    dxl.writeControlTableItem(FEEDFORWARD_2ND_GAIN, DXL_ID2, 950);
    */
    /*
    dxl.writeControlTableItem(POSITION_P_GAIN, DXL_ID2, 640);
    dxl.writeControlTableItem(POSITION_I_GAIN, DXL_ID2, 0);
    dxl.writeControlTableItem(POSITION_D_GAIN, DXL_ID2, 4000);
    dxl.writeControlTableItem(FEEDFORWARD_1ST_GAIN, DXL_ID2, 0);
    dxl.writeControlTableItem(FEEDFORWARD_2ND_GAIN, DXL_ID2, 0);
    */
    dxl.torqueOn(DXL_ID2);

    //XL320 - Eje 3
    dxl.torqueOff(DXL_ID3);
    dxl.setOperatingMode(DXL_ID3, OP_POSITION);
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID3, 120);//265);
    /*
    dxl.writeControlTableItem(POSITION_P_GAIN, DXL_ID3, 800);
    dxl.writeControlTableItem(POSITION_I_GAIN, DXL_ID3, 700);
    dxl.writeControlTableItem(POSITION_D_GAIN, DXL_ID3, 15000);
    dxl.writeControlTableItem(FEEDFORWARD_1ST_GAIN, DXL_ID3, 550);
    dxl.writeControlTableItem(FEEDFORWARD_2ND_GAIN, DXL_ID3, 550);
    */
    /*
    dxl.writeControlTableItem(POSITION_P_GAIN, DXL_ID3, 640);
    dxl.writeControlTableItem(POSITION_I_GAIN, DXL_ID3, 0);
    dxl.writeControlTableItem(POSITION_D_GAIN, DXL_ID3, 4000);
    dxl.writeControlTableItem(FEEDFORWARD_1ST_GAIN, DXL_ID3, 0);
    dxl.writeControlTableItem(FEEDFORWARD_2ND_GAIN, DXL_ID3, 0);
    */
    dxl.torqueOn(DXL_ID3);
    
    //XL430 - Eje 4
    dxl.torqueOff(DXL_ID4);
    dxl.setOperatingMode(DXL_ID4, OP_POSITION);
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID4, 200);//265);
    dxl.torqueOn(DXL_ID4);

    //XL320 - Eje 5
    dxl.torqueOff(DXL_ID5);
    dxl.setOperatingMode(DXL_ID5, OP_POSITION);
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID5, 200);//265);
    dxl.torqueOn(DXL_ID5);
    
    //XL430 - Eje 6
    dxl.torqueOff(DXL_ID6);
    dxl.setOperatingMode(DXL_ID6, OP_POSITION);
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID6, 200);//265); 175
    dxl.torqueOn(DXL_ID6);

    
    dxl.torqueOff(DXL_ID7);
    dxl.setOperatingMode(DXL_ID7, OP_POSITION);
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID7, 120);//265);   41 por cada rad/seg  86
    /*
    dxl.writeControlTableItem(POSITION_P_GAIN, DXL_ID7, 800);
    dxl.writeControlTableItem(POSITION_I_GAIN, DXL_ID7, 700);
    dxl.writeControlTableItem(POSITION_D_GAIN, DXL_ID7, 15000);
    dxl.writeControlTableItem(FEEDFORWARD_1ST_GAIN, DXL_ID7, 950);
    dxl.writeControlTableItem(FEEDFORWARD_2ND_GAIN, DXL_ID7, 950);
    */
    /*
    dxl.writeControlTableItem(POSITION_P_GAIN, DXL_ID7, 640);
    dxl.writeControlTableItem(POSITION_I_GAIN, DXL_ID7, 0);
    dxl.writeControlTableItem(POSITION_D_GAIN, DXL_ID7, 4000);
    dxl.writeControlTableItem(FEEDFORWARD_1ST_GAIN, DXL_ID7, 0);
    dxl.writeControlTableItem(FEEDFORWARD_2ND_GAIN, DXL_ID7, 0);
    */
    dxl.torqueOn(DXL_ID7);
               
    delay(15); 
    
    digitalWrite(ledME, HIGH); 

    //Mandar a topicos estados iniciales
    //Estado del Modo
    modo_msg.data = estadoModo; 
    modo_actuar.publish(&modo_msg);

    //Estado de Parada de emergencia
    stop_msg.data = estadoParada; 
    parada_emergencia.publish(&stop_msg);
}

//***************************************************************************************
void loop()
{    //f_present_position = dxl.getPresentPosition(DXL_ID);
  // Leer estados de los pulsadores
  bool pull1 = digitalRead(pullME);
  bool pull2 = digitalRead(pullML);
  bool pull3 = digitalRead(pullR);
  bool pull4 = digitalRead(pullGD);
  bool pull5 = digitalRead(pullD);
  bool pull6 = digitalRead(pullT_P);

  if (estadoModo && !estado_ant3) {
    
    digitalWrite(ledME, HIGH);   // Prender LED verde
    digitalWrite(ledML, LOW);    // Apagar LED rojo
    digitalWrite(ledSMT, LOW);     // Apagar LED azul 2
    digitalWrite(ledSMP, LOW);      // Apagar LED rojo 2
    digitalWrite(ledD, LOW);     // Apagar LED amarillo 2 

    if (estadoParada){
      digitalWrite(ledSMT, HIGH);
      digitalWrite(ledSMP, LOW);
    } else{
        digitalWrite(ledSMP, HIGH);
        digitalWrite(ledSMT, LOW);
      }
    estado_ant3=estadoModo;
    
  } else if (!estadoModo && estado_ant3){
    digitalWrite(ledME, LOW);   // Apagar LED verde
    digitalWrite(ledML, HIGH);   // Prender LED rojo 
    if (estadoSub_Modo){
      digitalWrite(ledSMT, HIGH);
    } else{
        digitalWrite(ledSMP, HIGH);
      }
    estado_ant3=estadoModo;
    }
    


  // Actualizar estadoModo y controlar LEDs
  if (pull1 && !pull2 && !estadoModo) {
    estadoModo = true;
    modo_msg.data = estadoModo; 
    modo_actuar.publish(&modo_msg);

  } else if (!pull1 && pull2 && estadoModo){
    estadoModo = false;
    modo_msg.data = estadoModo; 
    modo_actuar.publish(&modo_msg);

    }

    //digitalWrite(ledGD, LOW);   // Apagar LED verde
    //digitalWrite(ledR, HIGH);   // Prender LED rojo
  

  // Actualizar estadoRutina y controlar LED amarillo
  if (pull3 && !estadoRutina) {
    // Activar estado2 y encender LED amarillo
    estadoRutina = true; 
    digitalWrite(ledR, HIGH);
    b=0;
    if (estadoModo){
      bool_msg.data = true; // Cambia esto según tu lógica
      rutina.publish(&bool_msg);
    }
    else{
      clean_msg.data = true; // Cambia esto según tu lógica
      limpieza_rutina.publish(&clean_msg);
    }
    

    // Registrar el tiempo de inicio
    tiempoInicio = millis();
  }

  // Verificar si han pasado 2 segundos desde que se activó el pulsador
  if (estadoRutina && (millis() - tiempoInicio >= tiempoEspera)) {
    // Apagar LED amarillo y restablecer estadoRutina
    digitalWrite(ledR, LOW);
    estadoRutina = false;
  }

  if (!estadoModo){

  
    if(pull6 && !valAnterior && (millis() - t6 > tpull6)) { 
      estadoSub_Modo = !estadoSub_Modo; 
      m_lectura_msg.data = estadoSub_Modo; // Cambia esto según tu lógica
      tipo_modo_lectura.publish(&m_lectura_msg);
      


      t6 = millis();         
    }
    valAnterior=pull6;
    //////////////////////////////////////
    //Guardado de punto y trayectoria
    //////////////////////////////////////
    if (pull4 != lastPull4State) {
        // Reinicia el temporizador de debounce
        lastDebounceTime = millis();
        //digitalWrite(ledR, HIGH);
      }

      // Si ha pasado el tiempo de debounce, actualiza el estado del pulsador
    if ((millis() - lastDebounceTime) > debounceDelay) {
      // Si el estado ha cambiado, actúa en consecuencia
      if (pull4 && !pulso) {
        pulso = true;
        digitalWrite(ledGD, HIGH); // Prender LED azul
        if (estadoSub_Modo){
          guardarT_msg.data = true; // Cambia esto según tu lógica
          guardar_trayectoria.publish(&guardarT_msg);
        }
        else{ publishPosdy(); }// Ejecutar la función publishPosdy         
      } else if (!pull4 && pulso) {
        pulso = false;
        digitalWrite(ledGD, LOW); // Apagar LED azul
      }
    }
    lastPull4State=pull4;
    //////////////////////////////////////
    //////////////////////////////////////




    

      // ENCLAVAR-DESENCLAVAR
    if (pull5 == HIGH && enclavar) {      
      digitalWrite(ledD, HIGH);       // Prender LED

      //noInterrupts(); // Deshabilitar interrupciones

      dxl.torqueOff(DXL_ID1);              // Desactivar torque
      dxl.torqueOff(DXL_ID2); 
      dxl.torqueOff(DXL_ID3); 
      dxl.torqueOff(DXL_ID4); 
      dxl.torqueOff(DXL_ID5);
      dxl.torqueOff(DXL_ID6);      
      dxl.torqueOff(DXL_ID7);  

      //interrupts();   // Habilitar interrupciones

      enclavar = false;                        // Cambiar estado de enclavar
                              
      } else if (pull5 == LOW && !enclavar ) {               
        digitalWrite(ledD, LOW); // Apagar LED

        //noInterrupts(); // Deshabilitar interrupciones

        dxl.torqueOn(DXL_ID1);        // Activar torque
        dxl.torqueOn(DXL_ID2);
        dxl.torqueOn(DXL_ID3);
        dxl.torqueOn(DXL_ID4);
        dxl.torqueOn(DXL_ID5);
        dxl.torqueOn(DXL_ID6);       
        dxl.torqueOn(DXL_ID7);

        //interrupts();   // Habilitar interrupcioness

        enclavar = true;                 // Cambiar estado de enclavar
        tpublespera=millis();
        
       }


   if (estadoSub_Modo){

      digitalWrite(ledSMT, HIGH);
      digitalWrite(ledSMP, LOW);

      if(!enclavar){
        if(millis() - tpublespera > tpublic_escritura){
          tpublespera =millis();
          publishPosdy();
        }
      }   

    }else{
      digitalWrite(ledSMP, HIGH);
      digitalWrite(ledSMT, LOW);      
      
    }
  }
  else{;
    if(pull6 && !valAnterior && (millis() - t6 > tpull6)) { 
      if(estadoParada){
        dxl.torqueOn(DXL_ID1);
        dxl.torqueOn(DXL_ID2);
        dxl.torqueOn(DXL_ID3);
        dxl.torqueOn(DXL_ID4);
        dxl.torqueOn(DXL_ID5);
        dxl.torqueOn(DXL_ID6);
        dxl.torqueOn(DXL_ID7);
      }
      estadoParada = !estadoParada; 
      stop_msg.data = estadoParada; 
      parada_emergencia.publish(&stop_msg);
      


      t6 = millis();         
    }
    valAnterior=pull6;

    if (estadoParada){

      digitalWrite(ledSMT, HIGH);
      digitalWrite(ledSMP, LOW);

    }else{
      digitalWrite(ledSMP, HIGH);
      digitalWrite(ledSMT, LOW);            
    }
  }



  
    
    static uint32_t pubTime = 0;

    uint32_t t = millis();
    if ((t-pubTime) >= (1000 / POSITIONS_PUBLISH_FREQUENCY)) {
        publishPositions();
        pubTime = t;
    }

    
    nh.spinOnce();



    
    if (nh.connected()) 
    {
        if (false == rosConnected) 
        {
            rosConnected = true;
            //delay(2);
            nh.loginfo("ARDUINO - DYNAMIXEL CONECTADO");
        }
    }
    else {
        if (rosConnected)
          nh.initNode();  // Try to reinit node in case of failure
          
        rosConnected = false;
    }


  
      //dxl.setGoalPosition(DXL_ID1, b);
}
