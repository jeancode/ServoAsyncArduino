
#include <Arduino.h>

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <vector>
#include <EEPROM.h>



using namespace std;

#define ADDRES = 0x40
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

float map_custom(float x, float in_min, float in_max, float out_min, float out_max) {
    if (in_min == in_max) {
        return out_min; // O devuelve out_max según la lógica de tu aplicación
    } else {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
}

class UtilMk{

    
    private:
        Adafruit_PWMServoDriver pwm;
        using CallbackFunction = void (*)(int, int);

        using CallbackAcelerationTime = void (*)(int,int);

        

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


        void Acelerate(int id,float aceleracion,  int maximaAceleracion,int minAceleration,int posicionActual,float destino,float interval,CallbackAcelerationTime cb){


                    //cuantos devemos avanzar
                    float cuantos  =  abs(destino - posicionActual);

                    int direccion = ((destino - posicionActual) > 0) ? 1 : -1;

                    Serial.println("Cuantos:" +String(cuantos));
                    
                    // 30% de incremto
                    float saltos =  roundf((cuantos / 100) * 35.00);

                    Serial.println("Saltos" + String(saltos));

                    float ms = 0;

                    float msMaxim = (aceleracion * saltos) * 2;

                    Serial.println("MS:" + String(msMaxim));


                    if(direccion>0){

                        Serial.println("Inical:" + String(posicionActual) + "-" + String(posicionActual+saltos));

                        //Aceleracion
                        for(float i = posicionActual; i <= posicionActual+saltos;i += interval){
                            if(msMaxim != ms){
                                ms = ms + aceleracion;
                                float Mms =  map_custom(ms,0,msMaxim,maximaAceleracion,minAceleration);
                                //Serv.angleToPulse(motor.id, i);
                                cb(id,i);
                                if(ms < 0){
                                    ms = 0;
                                }
                                delay(Mms);
                            }
                        }
                        
       

                        //Constante
                        float limite2 = destino - saltos - 1;

                        Serial.println("Constante:" + String( posicionActual+saltos) + "-" + String(limite2));
                    
                        //Serial.println("Constant Spped:" +String(limite2));
                        for(float i = posicionActual+saltos+1; i <=  limite2;i += interval){

                            if(msMaxim != ms){
                                //Serv.angleToPulse(motor.id, i);
                                cb(id,i);
                                float Mms =  map_custom(ms,0,msMaxim,maximaAceleracion,minAceleration);
                                if(ms < 0){
                                    ms = 0;
                                }
                                delay(Mms);
                            }

                        }


                        Serial.println("Desaseleracion:" + String(limite2) + "-" + String( destino));

                        //desaceleracion
                        for(float i = limite2+1; i <=  destino;i += interval){


                            if(msMaxim != ms){

                                ms = ms - aceleracion;

                                cb(id,i);
                                //Serv.angleToPulse(motor.id, i);

                                float Mms =  map_custom(ms,0,msMaxim,maximaAceleracion,minAceleration);

                                if(ms < 0){

                                    ms = 0;
                                }

                                delay(Mms);
                            }

                        }

                    }else if (direccion < 0) {
                        
                        Serial.println("Inical:" + String(posicionActual) + "-" + String(posicionActual - saltos));

                        // Aceleración en dirección opuesta
                        for (float i = posicionActual; i >= posicionActual - saltos; i -= interval) {
                            if (msMaxim != ms) {
                                ms = ms + aceleracion;
                                float Mms = map_custom(ms, 0, msMaxim, maximaAceleracion,minAceleration);
                                //Serv.angleToPulse(motor.id, i);
                                cb(id,i);
                                if (ms < 0) {
                                    ms = 0;
                                }
                                delay(Mms);
                            }
                        }

                        // Constante en dirección opuesta
                        float limite2 = destino + saltos + 1;

                        Serial.println("Constante:" + String(posicionActual - saltos) + "-" + String(limite2));

                        for (float i = posicionActual - saltos - 1; i >= limite2; i -= interval) {
                            if (msMaxim != ms) {
                                //Serv.angleToPulse(motor.id, i);
                                cb(id,i);
                                float Mms = map_custom(ms, 0, msMaxim, maximaAceleracion,minAceleration);
                                if (ms < 0) {
                                    ms = 0;
                                }
                                delay(Mms);
                            }
                        }

                        Serial.println("Desaceleracion:" + String(limite2) + "-" + String(destino));

                        // Desaceleración en dirección opuesta
                        for (float i = limite2 - 1; i >= destino; i -= interval) {
                            if (msMaxim != ms) {
                                ms = ms - aceleracion;
                                //Serv.angleToPulse(motor.id, i);
                                cb(id,i);
                                float Mms = map_custom(ms, 0, msMaxim, maximaAceleracion, minAceleration);
                                if (ms < 0) {
                                    ms = 0;
                                }
                                delay(Mms);
                            }
                        }
                    }


        }

};


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
UtilMk Serv =  UtilMk(pwm);
TaskHandle_t Principal_task, Motor1_task,Motor2_task,Motor3_task,Motor4_task,Motor5_task,Motor6_task;


//estructura de datos que controlara el motor
struct Motor_Struc
{
    int id;
    float value;
    bool state =  false;
};


vector<Motor_Struc> Motor_Hardware;
vector<Motor_Struc> Memory;


// Función para la tarea del primer núcleo
void Principal(void *parameter) {

  while (true) {

      
    if (Serial.available() > 0) {

        // Lee el comando desde Serial
        Serial.setTimeout(5);
        String command = Serial.readStringUntil('\n');
       

        Serv.parseCommand(command, [](int servoNum, int angle) {

            Serial.println("Command:" + String(servoNum)  +"," + angle);
            
            Motor_Hardware[servoNum].id = servoNum;
            Motor_Hardware[servoNum].state =  true;
            Motor_Hardware[servoNum].value = angle;



        });

    }

  

    vTaskDelay(pdMS_TO_TICKS(1)); // Espera 1000ms

  }

}




//funcion Motor1
void Moro1(void *parameter) {



  while (true) {

    //analizamos esclusivamente el motot
    int id = 10;

    //consultamos State  en Motor_hardware
    if(Motor_Hardware[id].state == true){

        // Encontrar el elemento con id igual a 15
        for (const auto& motor : Motor_Hardware) {

            if (motor.id == id) {

                    Serial.print("ID: ");
                    Serial.print(motor.id);
                    Serial.print(", Value: ");
                    Serial.print(motor.value);
                    Serial.print(", State: ");
                    Serial.println(motor.state ? "ON" : "OFF");


                    //consultamos memori
                    Serial.print("ID_Momory: ");
                    Serial.print(Memory[motor.id] .id);
                    Serial.print(", Value: ");
                    Serial.print(Memory[motor.id] .value);
                    Serial.print(", State: ");
                    Serial.println(Memory[motor.id] .state ? "ON" : "OFF");


                    //Serv.angleToPulse(motor.id,motor.value);
                    //Memory[motor.id] posicion anterior
          

                    float aceleracion = 0.1; // Aceleración del movimiento
                    int maximaAceleracion = 10; // Máxima aceleración permitida
                    int minAceleration = 5;
                    int posicionActual = Memory[motor.id].value; // Posición actual
                    float interval = 0.5; // divicion de pasos

                    //funcion de ejecution
                    Serv.Acelerate(motor.id,aceleracion,maximaAceleracion,minAceleration,posicionActual,motor.value,interval,[](int id,int value){
                        
                        Serv.angleToPulse(id, value);

                    });

                    // Mover al valor final
                    Serv.angleToPulse(motor.id, motor.value);
                

                    // Actualizar la memoria con la nueva posición, estado y ID del motor al final del movimiento
                    Memory[motor.id].id = motor.id;
                    Memory[motor.id].value = motor.value;
                    Memory[motor.id].state = motor.state;

                break; // Detener la búsqueda después de imprimir el elemento
            }
        }

    }else if(Motor_Hardware[id].state == false){


    }

    //terminamos y cambiamos el valor
    Motor_Hardware[id].state  = false;

    vTaskDelay(pdMS_TO_TICKS(1)); // Espera 1000ms

  }

}


//funcion Motor2
void Moro2(void *parameter) {

  while (true) {

 
    //analizamos esclusivamente el motot
    int id = 11;

    //consultamos State  en Motor_hardware
    if(Motor_Hardware[id].state == true){

        // Encontrar el elemento con id igual a 15
        for (const auto& motor : Motor_Hardware) {

            if (motor.id == id) {

                    Serial.print("ID: ");
                    Serial.print(motor.id);
                    Serial.print(", Value: ");
                    Serial.print(motor.value);
                    Serial.print(", State: ");
                    Serial.println(motor.state ? "ON" : "OFF");


                    //consultamos memori
                    Serial.print("ID_Momory: ");
                    Serial.print(Memory[motor.id] .id);
                    Serial.print(", Value: ");
                    Serial.print(Memory[motor.id] .value);
                    Serial.print(", State: ");
                    Serial.println(Memory[motor.id] .state ? "ON" : "OFF");



                    float aceleracion = 0.1; // Aceleración del movimiento
                    int maximaAceleracion = 10; // Máxima aceleración permitida
                    int minAceleration = 5;
                    int posicionActual = Memory[motor.id].value; // Posición actual
                    float interval = 0.5; // divicion de pasos

                    //funcion de ejecution
                    Serv.Acelerate(motor.id,aceleracion,maximaAceleracion,minAceleration,posicionActual,motor.value,interval,[](int id,int value){
                        
                        Serv.angleToPulse(id, value);

                    });

                    // Mover al valor final
                    Serv.angleToPulse(motor.id, motor.value);
                

                    // Actualizar la memoria con la nueva posición, estado y ID del motor al final del movimiento
                    Memory[motor.id].id = motor.id;
                    Memory[motor.id].value = motor.value;
                    Memory[motor.id].state = motor.state;

                    break; // Detener la búsqueda después de imprimir el elemento
            }
        }

    }else if(Motor_Hardware[id].state == false){


    }

    //terminamos y cambiamos el valor
    Motor_Hardware[id].state  = false;

    vTaskDelay(pdMS_TO_TICKS(1)); // Espera 1000ms

  }

}


//motor3
void Moro3(void *parameter) {

  while (true) {

 
    //analizamos esclusivamente el motot
    int id = 12;

    //consultamos State  en Motor_hardware
    if(Motor_Hardware[id].state == true){

        // Encontrar el elemento con id igual a 15
        for (const auto& motor : Motor_Hardware) {

            if (motor.id == id) {

                    Serial.print("ID: ");
                    Serial.print(motor.id);
                    Serial.print(", Value: ");
                    Serial.print(motor.value);
                    Serial.print(", State: ");
                    Serial.println(motor.state ? "ON" : "OFF");


                    //consultamos memori
                    Serial.print("ID_Momory: ");
                    Serial.print(Memory[motor.id] .id);
                    Serial.print(", Value: ");
                    Serial.print(Memory[motor.id] .value);
                    Serial.print(", State: ");
                    Serial.println(Memory[motor.id] .state ? "ON" : "OFF");


                    float aceleracion = 0.1; // Aceleración del movimiento
                    int maximaAceleracion = 10; // Máxima aceleración permitida
                    int minAceleration = 5;
                    int posicionActual = Memory[motor.id].value; // Posición actual
                    float interval = 0.5; // divicion de pasos

                    //funcion de ejecution
                    Serv.Acelerate(motor.id,aceleracion,maximaAceleracion,minAceleration,posicionActual,motor.value,interval,[](int id,int value){
                        
                        Serv.angleToPulse(id, value);

                    });

                    // Mover al valor final
                    Serv.angleToPulse(motor.id, motor.value);
                

                    // Actualizar la memoria con la nueva posición, estado y ID del motor al final del movimiento
                    Memory[motor.id].id = motor.id;
                    Memory[motor.id].value = motor.value;
                    Memory[motor.id].state = motor.state;


                break; // Detener la búsqueda después de imprimir el elemento
            }
        }

    }else if(Motor_Hardware[id].state == false){


    }

    //terminamos y cambiamos el valor
    Motor_Hardware[id].state  = false;

    vTaskDelay(pdMS_TO_TICKS(1)); // Espera 1000ms

  }

}


//motor4
void Moro4(void *parameter) {

  while (true) {

 
    //analizamos esclusivamente el motot
    int id = 13;

    //consultamos State  en Motor_hardware
    if(Motor_Hardware[id].state == true){

        // Encontrar el elemento con id igual a 15
        for (const auto& motor : Motor_Hardware) {

            if (motor.id == id) {

                    Serial.print("ID: ");
                    Serial.print(motor.id);
                    Serial.print(", Value: ");
                    Serial.print(motor.value);
                    Serial.print(", State: ");
                    Serial.println(motor.state ? "ON" : "OFF");


                    //consultamos memori
                    Serial.print("ID_Momory: ");
                    Serial.print(Memory[motor.id] .id);
                    Serial.print(", Value: ");
                    Serial.print(Memory[motor.id] .value);
                    Serial.print(", State: ");
                    Serial.println(Memory[motor.id] .state ? "ON" : "OFF");


                    float aceleracion = 0.1; // Aceleración del movimiento
                    int maximaAceleracion = 10; // Máxima aceleración permitida
                    int minAceleration = 5;
                    int posicionActual = Memory[motor.id].value; // Posición actual
                    float interval = 0.5; // divicion de pasos

                    //funcion de ejecution
                    Serv.Acelerate(motor.id,aceleracion,maximaAceleracion,minAceleration,posicionActual,motor.value,interval,[](int id,int value){
                        
                        Serv.angleToPulse(id, value);

                    });

                    // Mover al valor final
                    Serv.angleToPulse(motor.id, motor.value);
                

                    // Actualizar la memoria con la nueva posición, estado y ID del motor al final del movimiento
                    Memory[motor.id].id = motor.id;
                    Memory[motor.id].value = motor.value;
                    Memory[motor.id].state = motor.state;
                    

                break; // Detener la búsqueda después de imprimir el elemento
            }
        }

    }else if(Motor_Hardware[id].state == false){


    }

    //terminamos y cambiamos el valor
    Motor_Hardware[id].state  = false;

    vTaskDelay(pdMS_TO_TICKS(1)); // Espera 1000ms

  }

}


//motor5
void Moro5(void *parameter) {

  while (true) {

 
    //analizamos esclusivamente el motot
    int id = 14;

    //consultamos State  en Motor_hardware
    if(Motor_Hardware[id].state == true){

        // Encontrar el elemento con id igual a 15
        for (const auto& motor : Motor_Hardware) {

            if (motor.id == id) {

                    Serial.print("ID: ");
                    Serial.print(motor.id);
                    Serial.print(", Value: ");
                    Serial.print(motor.value);
                    Serial.print(", State: ");
                    Serial.println(motor.state ? "ON" : "OFF");


                    //consultamos memori
                    Serial.print("ID_Momory: ");
                    Serial.print(Memory[motor.id] .id);
                    Serial.print(", Value: ");
                    Serial.print(Memory[motor.id] .value);
                    Serial.print(", State: ");
                    Serial.println(Memory[motor.id] .state ? "ON" : "OFF");


                    float aceleracion = 0.1; // Aceleración del movimiento
                    int maximaAceleracion = 10; // Máxima aceleración permitida
                    int minAceleration = 5;
                    int posicionActual = Memory[motor.id].value; // Posición actual
                    float interval = 0.5; // divicion de pasos

                    //funcion de ejecution
                    Serv.Acelerate(motor.id,aceleracion,maximaAceleracion,minAceleration,posicionActual,motor.value,interval,[](int id,int value){
                        
                        Serv.angleToPulse(id, value);

                    });

                    // Mover al valor final
                    Serv.angleToPulse(motor.id, motor.value);
                

                    // Actualizar la memoria con la nueva posición, estado y ID del motor al final del movimiento
                    Memory[motor.id].id = motor.id;
                    Memory[motor.id].value = motor.value;
                    Memory[motor.id].state = motor.state;
                    

                break; // Detener la búsqueda después de imprimir el elemento
            }
        }

    }else if(Motor_Hardware[id].state == false){


    }

    //terminamos y cambiamos el valor
    Motor_Hardware[id].state  = false;

    vTaskDelay(pdMS_TO_TICKS(1)); // Espera 1000ms

  }

}


//motor6
void Moro6(void *parameter) {

  while (true) {

 
    //analizamos esclusivamente el motot
    int id = 15;

    //consultamos State  en Motor_hardware
    if(Motor_Hardware[id].state == true){

        // Encontrar el elemento con id igual a 15
        for (const auto& motor : Motor_Hardware) {

            if (motor.id == id) {

                    Serial.print("ID: ");
                    Serial.print(motor.id);
                    Serial.print(", Value: ");
                    Serial.print(motor.value);
                    Serial.print(", State: ");
                    Serial.println(motor.state ? "ON" : "OFF");


                    //consultamos memori
                    Serial.print("ID_Momory: ");
                    Serial.print(Memory[motor.id] .id);
                    Serial.print(", Value: ");
                    Serial.print(Memory[motor.id] .value);
                    Serial.print(", State: ");
                    Serial.println(Memory[motor.id] .state ? "ON" : "OFF");


                    float aceleracion = 0.1; // Aceleración del movimiento
                    int maximaAceleracion = 10; // Máxima aceleración permitida
                    int minAceleration = 5;
                    int posicionActual = Memory[motor.id].value; // Posición actual
                    float interval = 0.5; // divicion de pasos

                    //funcion de ejecution
                    Serv.Acelerate(motor.id,aceleracion,maximaAceleracion,minAceleration,posicionActual,motor.value,interval,[](int id,int value){
                        
                        Serv.angleToPulse(id, value);

                    });

                    // Mover al valor final
                    Serv.angleToPulse(motor.id, motor.value);
                

                    // Actualizar la memoria con la nueva posición, estado y ID del motor al final del movimiento
                    Memory[motor.id].id = motor.id;
                    Memory[motor.id].value = motor.value;
                    Memory[motor.id].state = motor.state;
                    

                break; // Detener la búsqueda después de imprimir el elemento
            }
        }

    }else if(Motor_Hardware[id].state == false){


    }

    //terminamos y cambiamos el valor
    Motor_Hardware[id].state  = false;

    vTaskDelay(pdMS_TO_TICKS(1)); // Espera 1000ms

  }

}




void setup() {
    
    Serial.begin(115200);
    Serial.println("8 channel Servo test!");

    Serv.start();
    //Serv.angleToPulse(i,Buffer[i].value);

    //inicializar Motor_Hardware


    for(int i = 0;i <= 16;i++){

        Motor_Struc a;
    
        a.id = i;
        a.state = false;
        a.value = 90;

        Motor_Hardware.push_back(a);
        Memory.push_back(a);
    }


    
    Motor_Hardware[10].id = 10;
    Motor_Hardware[10].state =  true;
    Motor_Hardware[10].value = 50;

    
    Motor_Hardware[11].id = 11;
    Motor_Hardware[11].state =  true;
    Motor_Hardware[11].value = 90;

    
    Motor_Hardware[12].id = 12;
    Motor_Hardware[12].state =  true;
    Motor_Hardware[12].value = 40;



    // Crear tareas para cada núcleo
    xTaskCreatePinnedToCore(
        Principal,          // Función de la tarea del primer núcleo
        "Principal",          // Nombre de la tarea
        10000,            // Tamaño de la pila de la tarea
        NULL,             // Parámetros de la tarea
        1,                // Prioridad de la tarea
        &Principal_task,           // Manejador de la tarea
        0                 // Núcleo 0
    );


    //Motor1
    xTaskCreatePinnedToCore(
        Moro1,          // Función de la tarea del primer núcleo
        "Moro1",          // Nombre de la tarea
        10000,            // Tamaño de la pila de la tarea
        NULL,             // Parámetros de la tarea
        1,                // Prioridad de la tarea
        &Motor1_task,           // Manejador de la tarea
        0                 // Núcleo 0
    );

    //Motor2
    xTaskCreatePinnedToCore(
        Moro2,          // Función de la tarea del primer núcleo
        "Moro2",          // Nombre de la tarea
        10000,            // Tamaño de la pila de la tarea
        NULL,             // Parámetros de la tarea
        1,                // Prioridad de la tarea
        &Motor2_task,           // Manejador de la tarea
        0                 // Núcleo 0
    );

    //Motor3
    xTaskCreatePinnedToCore(
        Moro3,          // Función de la tarea del primer núcleo
        "Moro3",          // Nombre de la tarea
        10000,            // Tamaño de la pila de la tarea
        NULL,             // Parámetros de la tarea
        1,                // Prioridad de la tarea
        &Motor3_task,           // Manejador de la tarea
        0                 // Núcleo 0
    );


    //Motor4
    xTaskCreatePinnedToCore(
        Moro4,          // Función de la tarea del primer núcleo
        "Moro4",          // Nombre de la tarea
        10000,            // Tamaño de la pila de la tarea
        NULL,             // Parámetros de la tarea
        1,                // Prioridad de la tarea
        &Motor4_task,           // Manejador de la tarea
        0                 // Núcleo 0
    );


    //Motor5
    xTaskCreatePinnedToCore(
        Moro5,          // Función de la tarea del primer núcleo
        "Moro5",          // Nombre de la tarea
        10000,            // Tamaño de la pila de la tarea
        NULL,             // Parámetros de la tarea
        1,                // Prioridad de la tarea
        &Motor5_task,           // Manejador de la tarea
        0                 // Núcleo 0
    );


    //Motor5
    xTaskCreatePinnedToCore(
        Moro6,          // Función de la tarea del primer núcleo
        "Moro6",          // Nombre de la tarea
        10000,            // Tamaño de la pila de la tarea
        NULL,             // Parámetros de la tarea
        1,                // Prioridad de la tarea
        &Motor6_task,           // Manejador de la tarea
        0                 // Núcleo 0
    );




    delay(10);
}



void loop() {



}