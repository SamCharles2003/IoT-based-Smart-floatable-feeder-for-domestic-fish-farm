/*************************************************************
  WARNING!
    It's very tricky to get it working. Please read this article:
    http://help.blynk.cc/hardware-and-libraries/arduino/esp8266-with-at-firmware

  You’ll need:
   - Blynk IoT app (download from App Store or Google Play)
   - Arduino Uno board
   - Decide how to connect to Blynk
     (USB, Ethernet, Wi-Fi, Bluetooth, ...)

  There is a bunch of great example sketches included to show you how to get
  started. Think of them as LEGO bricks  and combine them as you wish.
  For example, take the Ethernet Shield sketch and combine it with the
  Servo example, or choose a USB sketch and add a code from SendData
  example.
 *************************************************************/

/* Fill-in information from Blynk Device Info here */
#define BLYNK_TEMPLATE_ID           "TMPL3-wGRTRDr"
#define BLYNK_TEMPLATE_NAME         "Smart Fishery"
#define BLYNK_AUTH_TOKEN            "o3ONqj8xHAlpiltnqGMII2vZGzwmz3iZ"

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial


#include <ESP8266_Lib.h>
#include <BlynkSimpleShieldEsp8266.h>
#include <NewPing.h>    
#include <Servo.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "Galaxy A14 5G";
char pass[] = "masleschar3002";

// Hardware Serial on Mega, Leonardo, Micro...
#define EspSerial Serial1

// or Software Serial on Uno, Nano...
//#include <SoftwareSerial.h>
//SoftwareSerial EspSerial(3,13); // RX, TX

// Your ESP8266 baud rate:
#define ESP8266_BAUD 115200

//Motor Driver
#define LeftMotorForward  7
#define LeftMotorBackward 6
#define RightMotorForward 4
#define RightMotorBackward 5
#define sensorPin A1
#define pHSense A0
#define ONE_WIRE_BUS 2

int forward_value;
int reverse_value;
int left_value;
int right_value;
int manual_value;
int feeder_value;
int skimmer_value;

      float ph_value;
      float temperature_value;
      int ntu;
      int volt;
      int samples = 10;
      float adc_resolution = 1024.0;      
      float tempC;       

//Newping
#define trig_pin 12 //analog input 1
#define echo_pin 11 //analog input 2
#define maximum_distance 200
boolean goesForward = false;
int distance = 100;


NewPing sonar(trig_pin, echo_pin, maximum_distance);
BlynkTimer timer;
ESP8266 wifi(&EspSerial);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
LiquidCrystal_I2C lcd(0x27, 20, 4);



Servo servo_motor; //our servo name
Servo feeder_motor;
Servo skimmer_motor;

void setup()
{
 
  // Debug console
  Serial.begin(9600);
  sensors.begin();
   lcd.init();
  lcd.backlight();

  lcd.setCursor(0,0);
  lcd.print("Smart Fishery");
  lcd.setCursor(0,1);
  lcd.print("Connecting to......");
  lcd.setCursor(0,2);
  lcd.print(ssid);
  


  // Set ESP8266 baud rate
  EspSerial.begin(ESP8266_BAUD);
  delay(10);
// timer.setInterval(1000L,myTimer);
  Blynk.begin(BLYNK_AUTH_TOKEN, wifi, ssid, pass);
  // You can also specify server:
  //Blynk.begin(BLYNK_AUTH_TOKEN, wifi, ssid, pass, "blynk.cloud", 80);
  //Blynk.begin(BLYNK_AUTH_TOKEN, wifi, ssid, pass, IPAddress(192,168,1,100), 8080);
  pinMode(RightMotorForward, OUTPUT);
  pinMode(LeftMotorForward, OUTPUT);
  pinMode(LeftMotorBackward, OUTPUT);
  pinMode(RightMotorBackward, OUTPUT);

   servo_motor.attach(8); //our servo pin
  feeder_motor.attach(9);//brown
  skimmer_motor.attach(10);
    timer.setInterval(1000L,myTimer);
lcd.clear();
}

void loop()
{
  Blynk.run();
  myTimer();
  // You can inject your own code or combine it with other sketches.
  // Check other examples on how to communicate with Blynk. Remember
  // to avoid delay() function!
}
void myTimer()
{  
  lcd.setCursor(0,0);
  lcd.print("Smart Fishery (WIFI)");
  temperature();
  read_ph();
  turbidity();

  
  distance = readPing();
Serial.println(distance);
  feeder_motor.write(feeder_value==1?90:180);
   skimmer_motor.write(skimmer_value);
   if(manual_value!=1){
 
   distance = readPing();
Serial.println(distance);   
  if (distance <= 20)
  {
    moveStop();
    delay(300);
    moveBackward();
    delay(400);
    moveStop();
    delay(300);
   int distanceRight = lookRight();
    delay(300);
   int distanceLeft = lookLeft();     
    delay(300);
    if (distance >= distanceLeft)
    {
      turnRight();
      moveStop();
    }
    else
    {
      turnLeft();
      moveStop();
    }
  }
  else
  { 
    moveForward(); 
  }
    distance = readPing();
}
else{manual_control(); distance = readPing();}
}




BLYNK_WRITE(V2) 
{
manual_value=param.asInt();
Serial.print("Manual:");
Serial.println(manual_value);

}

BLYNK_WRITE(V3) 
{
 forward_value=param.asInt();
Serial.print("Forward:");
Serial.println(forward_value);

}

BLYNK_WRITE(V4) 
{
 reverse_value=param.asInt();
Serial.print("Reverse:");
Serial.println(reverse_value);

}

BLYNK_WRITE(V5) 
{
 left_value=param.asInt();
Serial.print("Left:");
Serial.println(left_value);

}

BLYNK_WRITE(V6) 
{
right_value=param.asInt();
Serial.println("Right:");
Serial.print(right_value);

}

BLYNK_WRITE(V8) 
{
 feeder_value=param.asInt();
Serial.println("feeder:");
Serial.print(feeder_value);
//feeder_motor.write(digitalRead(feeder_value)==1?140:115);
 
}

BLYNK_WRITE(V9) 
{
 skimmer_value=param.asInt();
Serial.println("Skimmer:");
Serial.print(skimmer_value);
// skimmer_motor.write(skimmer_value==1?45:180));
//skimmer_motor.write(digitalRead(skimmer_value)==1?140:115);

}
int lookRight()
{  
  servo_motor.write(50);
  delay(500);
  int distance = readPing();
  delay(100);
  servo_motor.write(115);
  return distance;
}

int lookLeft()
{
  servo_motor.write(170);
  delay(500);
  int distance = readPing();
  delay(100);
  servo_motor.write(115);
  return distance;
  delay(100);
}

int readPing()
{
  delay(70);
  int cm = sonar.ping_cm();
  if (cm==0)
  {
    cm=250;
  }
  return cm;
}

void moveStop()
{
  digitalWrite(RightMotorForward, LOW);
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);
  digitalWrite(LeftMotorBackward, LOW);
}

void moveForward()
{
  if(!goesForward)
  {
    goesForward=true;   
    digitalWrite(LeftMotorForward, HIGH);
    digitalWrite(RightMotorForward, HIGH); 
    digitalWrite(LeftMotorBackward, LOW);
    digitalWrite(RightMotorBackward, LOW); 
  }
}

void moveBackward()
{
  goesForward=false;
  digitalWrite(LeftMotorBackward, HIGH);
  digitalWrite(RightMotorBackward, HIGH); 
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorForward, LOW);
}

void turnRight()
{
  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(RightMotorBackward, HIGH); 
  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorForward, LOW);
  delay(500);
  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(RightMotorForward, HIGH);
  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorBackward, LOW); 
}

void turnLeft()
{
  digitalWrite(LeftMotorBackward, HIGH);
  digitalWrite(RightMotorForward, HIGH);
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);
  delay(500);
  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(RightMotorForward, HIGH);
  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorBackward, LOW);
}

void manual_control()
{
    read_ph();
  turbidity();
   temperature();
   feeder_motor.write(feeder_value==1?90:180);
   skimmer_motor.write(skimmer_value);
  if(forward_value==1) {
    digitalWrite(LeftMotorBackward, HIGH);
  digitalWrite(RightMotorBackward, HIGH); 
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorForward, LOW); }
  if(reverse_value==1) {
    digitalWrite(LeftMotorBackward, HIGH);
  digitalWrite(RightMotorBackward, HIGH); 
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorForward, LOW);}
  if(left_value==1) 
  {digitalWrite(LeftMotorBackward, HIGH);
  digitalWrite(RightMotorForward, HIGH);
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);}
  if(right_value==1) {
  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(RightMotorBackward, HIGH); 
  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorForward, LOW);}
  else{moveStop();}
}

void alternate_back()
{digitalWrite(LeftMotorBackward, HIGH);
  digitalWrite(RightMotorBackward, HIGH); 
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorForward, LOW);
  }

   void alternate_forward()
  {
    digitalWrite(LeftMotorForward, HIGH);
    digitalWrite(RightMotorForward, HIGH); 
    digitalWrite(LeftMotorBackward, LOW);
    digitalWrite(RightMotorBackward, LOW); 
  } 

  int turbidity()
{
 
     volt = 0;
    for(int i=0; i<800; i++)
    {
        volt += ((float)analogRead(sensorPin)/1023)*5;
    }
    volt = volt/800;
    volt = round_to_dp(volt,2);
   if(volt < 2.5){
      ntu = 3000;
    }else{
      ntu = -1120.4 * (volt * volt) + 5742.3 * volt - 4353.8;
    }
 Blynk.virtualWrite(V0,ntu);
 lcd.setCursor(0,3);
  lcd.print("Tur:"+String(ntu));
return 1;
}
 
float round_to_dp( float in_value, int decimal_place )
{
  float multiplier = powf( 10.0f, decimal_place );
  in_value = roundf( in_value * multiplier ) / multiplier;
  return in_value;
}

int read_ph()
  {
int measurings=0;
    for (int i = 0; i < samples; i++)
    {
        measurings += analogRead(pHSense);
        delay(10);
    }
    float voltage = 5 / adc_resolution * measurings/samples;
    Serial.print("pH= ");
    Serial.println(ph(voltage));    
   
    if (ph(voltage)<5 || ph(voltage)>9)
    {
    Blynk.virtualWrite(V7,ph(voltage));
    lcd.setCursor(0,1);
    lcd.print("pH:"+String(ph(voltage),2));
    lcd.setCursor(11,1);
    lcd.print("Warning!");
    lcd.clear();
    Blynk.logEvent("ph","Current pH: "+String(ph(voltage)));      
    }
    else if(ph(voltage)>5 || ph(voltage)<9)  {
    Blynk.virtualWrite(V7,ph(voltage));
    lcd.setCursor(0,1);
    lcd.print("pH:"+String(ph(voltage),2));
    lcd.setCursor(11,1);
    lcd.print("Normal");

    }
  }
  float ph (float voltage) {
    return 7 + ((2.5 - voltage) / 0.18)-12;
}
int temperature()
{ 
  // call sensors.requestTemperatures() to issue a global temperature 
  // request to all devices on the bus
  Serial.print("Requesting temperatures...");
  sensors.requestTemperatures(); // Send the command to get temperatures
  Serial.println("DONE");
  // After we got the temperatures, we can print them here.
  // We use the function ByIndex, and as an example get the temperature from the first sensor only.
   tempC = sensors.getTempCByIndex(0);

  // Check if reading was successful
  if(tempC<35 || tempC>25)
  {
        Blynk.virtualWrite(V1,tempC);
    lcd.setCursor(0,2);
  lcd.print("Temp:"+String(tempC));   
    lcd.setCursor(11,2);
  lcd.print("Normal!");    }
  
        
    else  if(tempC>35 || tempC<25)
  {
    Blynk.logEvent("temperature","Current Temperature"+String(tempC));
    lcd.setCursor(0,2);
  lcd.print("Temp:"+String(tempC));   
    lcd.setCursor(11,2);
  lcd.print("WARNING!");    
  Blynk.virtualWrite(V1,tempC);
 
  }
  
  else
  {
    Serial.println("Error: Could not read temperature data");
  }    return tempC;
}