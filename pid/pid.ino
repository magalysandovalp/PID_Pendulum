/***************************************************************
****************************************************************
Laboratorio de control automatico 
Pendulo grua puente

programa para:
1. medir el setpoint
2. Ajustar la posicion del carro manualmente
3. Control de ajuste para estabilizar el angulo

****************************************************************
***************************************************************/
#include <PID_v1.h>  //biblioteca del PID
#include <PID_AutoTune_v0.h>


int motor1 = 13;              //Salida HIGH se aleja del motor
int motor2 = 12;              //Salida HIGH se acerca al motor
int lejos  = 11;              //Entrada HIGH se aleja del motor
int cerca  = 10;              //Entrada HIGH se acerca del motor

int a  = 3;
int b  = 4;

//velocidad motor enable del puente
int pwm_out = 6;              //salida hacia el enable en pwm
int entrada = 0;              // entrada analogica para medir el MA3
double ma3;                   //valor analogico medido del MA3

int boton = 7;                // Si es High es manual si es LOW control


double setpoint;
double Output;

int SI = 2;
int sd = 3;


double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=1, consKi=0.05, consKd=0.25;
boolean bsetpoint;

double p1;
double p2;
double p3;
double p4;
double p5;
double p6;
double p7;
double p8;
double p9;
double p10;




PID myPID(&ma3, &Output, &setpoint,consKp,consKi,consKd, REVERSE);


void setup()  {
  
  delay(1000);  // Para evitar el error inicial por el moviemiento sin sentido
 // delay(10000); 
  pinMode(motor1,OUTPUT);
  pinMode(motor2,OUTPUT);
  pinMode(boton,INPUT);
  pinMode(lejos,INPUT);
  pinMode(cerca,INPUT);
  pinMode(a,INPUT);
  pinMode(b,INPUT);
  

  bsetpoint = true;
  myPID.SetOutputLimits(-255, 255);   // cosas para analizar
  myPID.SetSampleTime(50);            // Disminuir el tiempo de muestreo
  myPID.SetMode(AUTOMATIC);
  Serial.begin(9600);
  
  pinMode(SI,INPUT);
  pinMode(sd,INPUT);
   

}

//////////////Funcion movimiento/////////////////////////

int movimiento(int direccion, double velocidad)  {
  
  analogWrite(pwm_out, velocidad);
 
  if (direccion == 0)  {
    digitalWrite(motor1,HIGH);
    digitalWrite(motor2,LOW);
    
  }
  else if(direccion == 1)  {
    digitalWrite(motor1,LOW);
    digitalWrite(motor2,HIGH);
  }
  else if(direccion == 2)  {
    digitalWrite(motor1,LOW);
    digitalWrite(motor2,LOW);
  }
  return 0;
}

//////////////Funcion movimiento/////////////////////////

void loop()  { 
  
  if(bsetpoint == true){
   delay(10000);
   
   
  p1=analogRead(entrada);
  delay(1000);
  p2=analogRead(entrada);
  delay(1000);
  p3=analogRead(entrada);
  delay(1000);
  p4=analogRead(entrada);
  delay(1000);
  p5=analogRead(entrada);
  delay(1000);
  p6=analogRead(entrada);
  delay(1000);
  p7=analogRead(entrada);
  delay(1000);
  p8=analogRead(entrada);
  delay(1000);
  p9=analogRead(entrada);
  delay(1000);
  p10=analogRead(entrada);
  
  setpoint = (p1+p2+p3+p4+p5+p6+p7+p8+p9+p10)/10;
   
   
   
   
 //  setpoint = analogRead(entrada);     // valor actual de MA3 estable setpoint 
   bsetpoint = false; 
  }
  
  
  
  if(digitalRead(SI)==HIGH){
    movimiento(0,255);
    delay(1000);
  }
   if(digitalRead(sd)==HIGH){
    movimiento(1,255);
    delay(1000);
  }
  
  ////////////Ajuste manual/////////////////////////////////
  if (digitalRead(boton) == HIGH)  {
     if((digitalRead(lejos) == HIGH) && (digitalRead(cerca) == LOW))  {
       movimiento(0,250);
     }
     else if((digitalRead(lejos) == LOW) && (digitalRead(cerca) == HIGH))  {
       movimiento(1,250);
     }
     else  {
       movimiento(2,250);
     }
  }
  
  ////////////Ajuste manual/////////////////////////////////
  ////////////Ciclo con control/////////////////////////////////
  else if (digitalRead(boton) == LOW)  {
    
 //   ma3 = analogRead(entrada);
   
     valorma3();
  
  double gap = abs(setpoint-ma3); //distance away from setpoint
  if(gap<5)
  {  //we're close to setpoint, use conservative tuning parameters
    myPID.SetTunings(consKp, consKi, consKd);
  }
  
    else
  {
     //we're far from setpoint, use aggressive tuning parameters
     myPID.SetTunings(aggKp, aggKi, aggKd);
  }
    
    
    myPID.Compute();
    
    if(Output > 0)  {
      movimiento(0,abs(Output));
    }
    else if(Output < 0)  {
      movimiento(1,abs(Output));
    }
    else  {
      movimiento(2,0);
    }
  }
  else  {
    movimiento(2,0);
  }
}


 void valorma3(){
   p10=p9;
   p9=p8;
   p8=p7;
   p7=p6;
   p6=p5;
   p5=p4;
   p4=p3;
   p3=p2;
   p2=p1;
   p1=analogRead(entrada);
   ma3 = (p1+p2+p3+p4+p5+p6+p7+p8+p9+p10)/10;
 }

