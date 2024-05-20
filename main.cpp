
#include <Arduino.h>

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <vector>
#include <EEPROM.h>
#include <Metro.h>


using namespace std;

#define ADDRES = 0x40
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates



struct bufferTempStruct
{
    int serv;
    float value;
};

vector<bufferTempStruct> Buffer;
    

class MemMotor{


    private:

    public:

        float arrayValues[100];

        void begin(){

            EEPROM.begin(512); // Iniciar la EEPROM (512 bytes disponibles)

            //inicializamos bufffer term
            
            for(int i = 0; i <= 512;i++){

                EEPROM.put(i, 0); // Escribir el valor en la dirección 0
            
                EEPROM.commit(); // Guardar los cambios


                arrayValues[i];
            }




        }

        void MotorWrite(int servo,float value){

            //EEPROM.put(servo, value); // Escribir el valor en la dirección 0
            
            //EEPROM.commit(); // Guardar los cambios

            arrayValues[servo] =  value;

        }

        float MotorRead(int servo){

            float valorLeido;
            
            //EEPROM.get(servo, valorLeido);
            
            valorLeido =  arrayValues[servo];
            
            return valorLeido;
        }


};

class UtilMk{

    
    private:
        Adafruit_PWMServoDriver pwm;
        using CallbackFunction = void (*)(int, int);


        

    public:
    
        UtilMk( Adafruit_PWMServoDriver pwmIn){
            pwm = pwmIn;
        }

        void start(){

            pwm.begin();
            pwm.setOscillatorFrequency(27000000);
            pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

        }

        void angleToPulse(uint8_t servoNum, float angle) {
            if (angle < 0) angle = 0;
            if (angle > 180) angle = 180;
            float pulseWidth = map(angle, 0, 180, SERVOMIN, SERVOMAX);

            pwm.setPWM(servoNum, 0, pulseWidth);
        }

            
        void parseCommand(String command,CallbackFunction cb) {

            int servoNum;
            int angle;


            // Busca la coma para separar el número de servo y el ángulo
            int commaIndex = command.indexOf(',');
            
            if (commaIndex != -1 && command.length() > commaIndex + 1) {
                // Extrae el número de servo y el ángulo de la cadena de comando
                servoNum = command.substring(0, commaIndex).toInt();
                angle = command.substring(commaIndex + 1).toInt();

                    // Asegúrate de que el número del servo esté dentro del rango válido
                    if (servoNum >= 0 && servoNum <= 15) {
                    // Establece el ángulo para el servo especificado
                    

                        if (cb) {

                            cb(servoNum, angle );

                        }
                    }
            }

        }
};

// our servo # counter
uint8_t servonum = 10;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

UtilMk Serv =  UtilMk(pwm);

MemMotor Mem;


// Configuración del servo
uint8_t servoNum = 0; // Número del servo en el controlador PWM
float posicionInicial = 90.0; // Posición inicial del servo
float posicionFinal = 30.0; // Posición final deseada
float aceleracion = 1.0; // Valor de aceleración
float velocidadMaxima = 200.0; // Velocidad máxima

Metro tiempoMovimiento = Metro(1); // Duración total del movimiento (en milisegundos)
Metro Actividades = Metro(100);//actualizacion de actividades

struct servoInfo
{
    int servo;
    float value;
    bool state;
};


vector<servoInfo> instruciones;



void setup() {
  Serial.begin(115200);
  Serial.println("8 channel Servo test!");

  Serv.start();
  Mem.begin();

    


    //inicializa buffer
    for(int i = 0; i <= 16;i++){

        bufferTempStruct s;

        s.serv = 0;
        s.value = 0;

        Buffer.push_back(s);

    }   
    
    
    Buffer[15].serv = 15;
    Buffer[15].value =  10;

    Buffer[14].serv = 14;
    Buffer[14].value =  90;



    Buffer[13].serv = 13;
    Buffer[13].value =  90;

    Buffer[12].serv = 12;
    Buffer[12].value =  10;

    Buffer[11].serv = 11;
    Buffer[11].value =  80;

    Buffer[10].serv = 10;
    Buffer[10].value =  90;

    Buffer[9].serv = 9;
    Buffer[9].value =  90;

    for(int i = 9; i <= 15;i++){
    

        Serv.angleToPulse(i,Buffer[i].value);

    }



    delay(10);
}



void loop() {

  
    if (Serial.available() > 0) {

        // Lee el comando desde Serial
        Serial.setTimeout(5);
        String command = Serial.readStringUntil('\n');
        Serial.println(command);

        Serv.parseCommand(command, [](int servoNum, int angle) {

       
            servoInfo StartInfo;
            StartInfo.servo = servoNum;
            StartInfo.value =  angle;
            StartInfo.state =  false;
            instruciones.push_back(StartInfo);

            //motorRead state
            //si es true agregar comando al buffer de tiempo 

            //si es no ejecutar

            //SE LANZA ACTIVADAD CON
            //Buffer[servoNum].serv = servoNum;
            //Buffer[servoNum].value =  angle;
            //Buffer[servoNum].state =  true; activo


        });

    }

    if(Actividades.check()){

    for (size_t i = 0; i < instruciones.size(); ++i) {

        servoInfo elemento = instruciones[i];

        if(elemento.state == true){
            
        }
   
    }
    
   /*
    if (tiempoMovimiento.check()) {

    
        //Buffer[15] > Mem.MotorRead(15)
        for(int i = 0; i <= 15;i++){

            //Buffer[i].serv;
            //Buffer[i].value;
            
            //Serial.println("Servo:" + String(i) + ",Objetivo:" + String(  Buffer[i].value )  + ",Actual:" + String(  Mem.MotorRead(i) )  );


            if( Buffer[i].value  > Mem.MotorRead(Buffer[i].serv)){
                
                float inc = Mem.MotorRead(i) + 0.3;

                if(inc >= 180){
                    
                    inc = 180;
                    
                }   

                Serial.println(inc);

                Serv.angleToPulse(i,inc);

                Mem.MotorWrite(i,inc);



            }else if(Buffer[i].value  < Mem.MotorRead(Buffer[i].serv)){

                float inc = Mem.MotorRead(i)  - 0.3;
             
                if(inc < 0){
                    
                    inc  = 0;

                }   

                Serv.angleToPulse(i,inc);

                Mem.MotorWrite(i,inc);

                Serial.println(inc);


            }else if(Buffer[i].value == Mem.MotorRead(Buffer[i].serv)){

                 

            }

        }


  
    }

    */
}