


#define BLYNK_TEMPLATE_ID           "TMPL6_6lV0vpG"
#define BLYNK_TEMPLATE_NAME         "Quickstart Template"
#define BLYNK_AUTH_TOKEN            "YAtbOMNsC4LQhbvxoJhGL7MrTcqp1RwZ"


#define BLYNK_PRINT Serial


#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>




//LiquidCrystal_I2C lcd(0x27, 16, 2);   //adress 0x27/0x3F
//---------SETUP IO-------------------------
//#define led1 23
//#define led2 19
#define FAN 18
#define LED1x 19
#define LED2x 2
#define LED3x 5

uint8_t PWMx = 4;
int LED1=0;
int LED2=0;
int LED3=0;
int TMM=0;
float CAL=0;
float PHx=0,TBx=0,TB=0;
float h=0,t=0;
float hx=0,tx=0;

#define vCalibration 83.3
#define currCalibration 0.50

#define ONE_WIRE_BUS 4
// Comment this out to disable prints and save space




// Potentiometer is connected to GPIO 34 (Analog ADC1_CH6) 
const int potPin = 34;
const int potPin2 = 35;
const int potPin3 = 32;
const int potPin4 = 33;
const int potPin5 = 25;

float ADC1,ADC2,ADC3,ADC4;

int Ready=0;
int Ml=0;
String MinS="00";
String HourS="00";
String SecS="00";
int DataIn=0;
String DATA="";
String Temp1x="";

String Temp2x="";
String Temp1y="";
String PHy="";
String Temp2y="";
String Temp3y="";
String Temp3x="";
String Temp4y="";
String Temp4x="";
String currentTime;
String currentDate;
String TimerGet="00:00:00";

int analogVal=0;
float analogVolt=0;
int analogVal2=0;
float analogVolt2=0;
int SLIDER=0;
int MODE=0;
int Hour=0;
int Min=0;
float PH=0;
float TDS=0;
float Tempx=0;
int Spo2=0;
int Sec=0;
float SOIL;
float LEVEL=0;
int ALM=0;
int Val=100;
int Index=0;
float CV=0;
int CKN=0;
float temperature;
//-----------------------------------------------------------------
int ALM1=0;
int ALM2=0;
int TDIS=0;
int Rly1=0;
int wait=0;
int Rly2=0;
int Rly3=0;
int Rly4=0;
int Rly5=0;
float ZAVG=0;
int TOTALSAMPLE=0;
float TZ=0;
float TS=0;
int Stat=0;
int MOTION=0;
int STARTUP=0;

int POSITION=0;
float SBP,DBP;
float TD=0;
float TDm=0;
int STEP=0;
float StepCount=0;
float OldIrValue=0;
int Beat=0;
int BeatCount=0;
float PPg=0;
float FLEX=0;
int Tcount=0;
int COUNT=0;
float PIZEZO=0;
float VOLT=0;
float Current=0;
int FSPEED=150;
long previousMillis = 0;    
long interval = 3000; 
long previousMillis1 = 0;    
long interval1 = 10000; 
float AQI=0;
long duration1x, distance1, duration2x, distance2, duration3x, distance3, duration4x, distance4, duration5x, distance5;
 
//---------------------------------------------
  long UpperThreshold = 518;
    long LowerThreshold = 490;
    long reading = 0;
    float Pulse = 0.0;
    bool IgnoreReading = false;
    bool FirstPulseDetected = false;
    unsigned long FirstPulseTime = 0;
    unsigned long SecondPulseTime = 0;
    unsigned long PulseInterval = 0;
    int MyTimer=0;
//------------------------------------------------

//-----------------------------------------------------------

char auth[] = BLYNK_AUTH_TOKEN;

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "AQI";
char pass[] = "12345678";

BlynkTimer timer;

// This function is called every time the Virtual Pin 0 state changes
BLYNK_WRITE(V1) //switch
{
  int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable

   if (pinValue==1){
// digitalWrite(LED1x,HIGH);
 //digitalWrite(LED2x,LOW);
 //digitalWrite(LED3x,LOW);

  }

  }
  BLYNK_WRITE(V2) //switch
{
  int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable

   if (pinValue==1){
 //digitalWrite(LED1x,LOW);
 //digitalWrite(LED2x,HIGH);
 //digitalWrite(LED3x,LOW);

  }

  }
  BLYNK_WRITE(V3) //switch
{
  int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable

   if (pinValue==1){
 //digitalWrite(LED1x,LOW);
 //digitalWrite(LED2x,LOW);
 //digitalWrite(LED3x,HIGH);

  }

  }

BLYNK_WRITE(V10) //switch
{
  int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable
  Rly1=pinValue;
   if (pinValue==1){
if (MODE==0){
  digitalWrite(FAN,HIGH);
  if (STARTUP==0){
    STARTUP=1;
    ledcWrite(1, 255);
  }
}
  }
   if (pinValue==0){
 digitalWrite(FAN,LOW);
 STARTUP=0;
  }

  }

  BLYNK_WRITE(V11) //switch
{
  int pin2Value = param.asInt(); // assigning incoming value from pin V1 to a variable
  Rly2=pin2Value;
   if (pin2Value==1){
if (MODE==0){
 
}   
  }
   if (pin2Value==0){

  }

  }

    BLYNK_WRITE(V12) //switch
{
  int pin3Value = param.asInt(); // assigning incoming value from pin V1 to a variable
  Rly3=pin3Value;
   if (pin3Value==1){
     MODE=1;   //AUTO;
  }
   if (pin3Value==0){
    MODE=0;
  }

  }

  BLYNK_WRITE(V4)
{
  FSPEED = param.asInt(); // assigning incoming value from pin V1 to a variable
if (MODE==0){

}
}






//----------------------------------------------------------------------------
//---------------------------------------------------------------------

// This function is called every time the device is connected to the Blynk.Cloud
BLYNK_CONNECTED()
{
 
}


void myTimerEvent()
{

         ADC1 = analogRead(potPin);
    ADC1= (5.0 * ADC1)/1024.0;  //convert the analog data to moisture level
    AQI=(ADC1)/5.0 * 100;
  
  if (AQI<100){
    MODE=0;

  }
    if (AQI>100){
    MODE=1;

  }


   //-------------------------------------------------------------------
static unsigned long timepoint = millis();
//Blynk.syncVirtual(V4);

  if (MODE==0){
    ledcWrite(1, FSPEED);
  }







  if (MODE==1){
  

    if (AQI<=100){
       digitalWrite(FAN,LOW);
      STARTUP=0;
      ledcWrite(1, 0);
     
      Blynk.virtualWrite(V4,0);
            digitalWrite(LED1x,LOW);
 digitalWrite(LED2x,HIGH);
 digitalWrite(LED3x,LOW);
      LED1=0; LED2=1;  LED3=0;
    }
       if (AQI>110 && AQI<=150){
      //digitalWrite(FAN,HIGH);
      if (STARTUP==0){
        STARTUP=1;
        ledcWrite(1, 255);
        delay(500);
      }
      ledcWrite(1, 255);
      Blynk.virtualWrite(V4,255);
      digitalWrite(LED1x,LOW);
 digitalWrite(LED2x,LOW);
 digitalWrite(LED3x,HIGH);
 LED1=0; LED2=0;  LED3=1;
    }
     if (AQI>150){
     // digitalWrite(FAN,HIGH);
      if (STARTUP==0){
        STARTUP=1;
        ledcWrite(1, 255);
        delay(500);
        digitalWrite(LED1x,HIGH);
 digitalWrite(LED2x,LOW);
 digitalWrite(LED3x,LOW);
 LED1=1; LED2=0;  LED3=0;
       
      }
      ledcWrite(1, 255);
      Blynk.virtualWrite(V4,255);

    }
  }
    if (MODE==0){
  



    
    
  }
 Serial.println(AQI);


      Blynk.virtualWrite(V0,AQI);
       Blynk.virtualWrite(V1,LED1);
       Blynk.virtualWrite(V2,LED2);
       Blynk.virtualWrite(V3,LED3);
       



/*
ADC1 = analogRead(potPin);
 ADC1= (3.3 * ADC1)/1024.0;  //
*/



//Blynk.virtualWrite(V5,FLEX);


/*
Blynk.virtualWrite(V0,PH);
Blynk.virtualWrite(V1,TB);
Blynk.virtualWrite(V2,temperature);
*/






  //-------------------------------------------------------------
}

void setup()
{

    ledcAttachPin(PWMx, 1);
    ledcSetup(1, 12000, 8);
pinMode(FAN,OUTPUT);
pinMode(LED1x,OUTPUT);
pinMode(LED2x,OUTPUT);
pinMode(LED3x,OUTPUT);







  int i,k;
  Serial.begin(9600);




  Blynk.begin(auth, ssid, pass);
delay(1000);











  // You can also specify server:
  //Blynk.begin(auth, ssid, pass, "blynk.cloud", 80);
  //Blynk.begin(auth, ssid, pass, IPAddress(192,168,1,100), 8080);

  // Setup a function to be called every second
  timer.setInterval(1500L, myTimerEvent);


  delay(2000);
   // Initialize sensor

}


void sendSensor()
{
 

}

void loop()
{






   
  Blynk.run();
  timer.run();

}
