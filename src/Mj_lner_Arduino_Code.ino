#include <AceButton.h>



#include <EEPROM.h>


// Mjølner V0.0 Arduino Code//
//By Ben Sanford & Thor Hansen //
//For the nerf community //

//PHYSICAL PIN FUNCTIONS
//D10 ESC Stage 1 Signal
//(D9 ESC stage 2 Signal) Work-in-progress
//D8 Encoder SW
//D7 Encoder DT 
//D6 Encoder CLK
//D5 Solenoid mosfet
//D3 RevSwitchFront
//D2 FireSwitchBack
//A4 SDA
//A5 SCL
//A0 Battery monitor

//Libs included
#include <Servo.h>        // Servo for PMW signal to the ESC's
#include <SPI.h>        //Screen
#include <Wire.h>       //Screen
#include <Adafruit_GFX.h>       //Screen
#include <Adafruit_SSD1306.h>   //Screen
#include <AceMenu.h>        //Menu


//CONST INTS
const int TriggerBack = 2;              // the number of the trigger switch input for plunger
const int TriggerFront = 3;           // the number of the rev trigger switch input
const int Plunger = 5;              // the number of the plunger output
int ModeSelect = (1,2,3);           // The mode of operation selected
const int WaitPlungerOut = 500;     //Plunger out delay
const int WaitPlungerIn  = 500;     //Plunger in delay
const int ESCS1 = 10;               //ESC stage 1 pin
const int motorOnTime = 500;           // time motor will stay in maxthrottle after a shot before slowing down to min throttle
const int motorIdleTime = 1500;         // time motor will stay in minthrottle before going to nothrottle
const int spinUpTime = 500;           // time motor has to spin up before shot.


// Encoder CONST INTS
int counter = 0;                    //Use this variable to store "steps"
int previous_counter = 0;           //Use this variable to store previous "steps" value
int currentStateClock;              //Store the status of the clock pin (HIGH or LOW)
int StateData;                      //Store the status of the data pin (HIGH or LOW)
int lastStateClock;                 //Store the PREVIOUS status of the clock pin (HIGH or LOW)
String currentDir ="";              //Use this to print text 
unsigned long lastButtonPress = 0;  //Use this to store if the push button was pressed or not

//Speed ints
int currentSpeed = 1040;        //
int spindownRate = 10;          //units per 10ms
int ROF = (WaitPlungerOut+WaitPlungerIn)/1000; //Plunger cycles per 1 second
int getSpeed = currentSpeed; //
int newModeSelect = (1,2,3);

//Unsigned vars
unsigned long motorStartTime;        
unsigned long lastSpindownCheck = 0;
unsigned long elapsedTime = 0;
unsigned int timeOn=10;
unsigned int timeOff=90;

//Bools
bool TriggerStatus = LOW;            //button press status, 0 or 1
bool ModeStatus = LOW;            //button press status, 0 or 1  
bool RevTriggerStatus = LOW;                  //button press status, 0 or 1

//Define Speed values
#define NOTHROTTLE 1040                 
#define MINTHROTTLE 1200
#define MAXTHROTTLE 1960
int ThrottlePMW = (1200,1960);

//Define Screen values
#define SDA 5
#define SCL 4
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//Define Rotary stuff
#define Clock 9   //Clock pin connected to D9
#define Data 8    //Data pin connected to D8
#define Push 10   //Push button pin connected to D10

//Define Menu stuff
#define kMenuCount (sizeof(menuItems) / sizeof(menuItem))


//Servo
Servo throttles1;                   //Set throttle stage 1 to accept pmw signals 

//Battery setup
int voltage;
int getVoltage = voltage;           //Showing the current voltage
int printVolts(){
  int sensorValue = analogRead(A0); //read the A0 pin value
  float voltage = sensorValue * (2 / 1023.00) * 2; //convert the value to a true voltage.
  if (voltage < 9.2) //set the voltage considered low battery here
  {
    //ADD WHAT HAPPENS IF LOW VALUE VOLTAGE
  }
}
//Menu Setup
menuItem menuItems[] = {
    // name, getter, setter, format, min value , max value
    {"Fire Mode", ModeSelect, newModeSelect, Format::deciTemp, 1, 3},
    {"Battery", voltage, NULL, Format::boolean, 0, 1},
};



AceMenu menu = AceMenu(menuItems, kMenuCount,);

void setup()   // put your setup code here, to run once:
{  
  /*  Set encoder pins as inputs with pullups. If you use the Encoder Module, you don't need 
   *  pullups for Clock and Data, only for the push button.*/
  pinMode(Clock,INPUT_PULLUP);
  pinMode(Data,INPUT_PULLUP);
  pinMode(Push, INPUT_PULLUP);

  //Here we activate pin change interruptions on pin D8 and D9 with PCINT0 and PCINT1
  PCICR |= (1 << PCIE0);      //enable PCMSK0 scan                                                 
  PCMSK0 |= (1 << PCINT0);    //Pin 8 (Data) interrupt. Set pin D8 to trigger an interrupt on state change.  
  PCMSK0 |= (1 << PCINT1);    //Pin 9 (Clock) interrupt. Set pin D9 to trigger an interrupt on state change.  

    //Startup the serial port for monitoring
  Serial.begin(9600);
  

  // Read the initial state of Clock pin (it could be HIGH or LOW)
  lastStateClock = digitalRead(Clock);

  menu.begin();   // initialise the menu 

  pinMode(Plunger, OUTPUT);              //assign plunger output
  pinMode(TriggerBack, INPUT_PULLUP);               //assign trigger input
  digitalWrite(Plunger,LOW);             //write the plunger off


       pinMode(TriggerFront, INPUT);
  digitalWrite(TriggerFront, HIGH);                   //write the rev trigger on
  throttles1.attach(ESCS1);                           //Attach esc pin to throttle stage 1
  throttles2.attach(ESCS2);                           //Attach esc pin to throttle stage 2
  
  // ESC Arming Sequence
    throttles1.writeMicroseconds(NOTHROTTLE);      // Wait for ESC to arm / Exit safety mode
    delay(1000);   

  //Screen setup
display.clearDisplay(); // Clear the buffer
}  


void loop() 
{

 // put your main code here, to run repeatedly:
   
  menu.update(); // update the menu

 ModeStatus = ModeSelect;          //Read the button status
 delay(100);
 TriggerStatus = digitalRead(TriggerBack);          //Read the button status

 
 if(TriggerStatus == LOW)
 {

  if (ModeStatus == 1)
  {
    FirePlunger();                   //Call the fire plunger function
     Serial.println("SINGLE");
   }
 if (ModeStatus == 2)
  {
    FireBurst();
    Serial.println("BURST"); 
  }
 if (ModeStatus == 3)
  {
    FireAuto();                       //Call the Fire full auto plunger function
    Serial.println("AUTO"); 
  }
   
 }
  
  RevTriggerStatus = digitalRead(TriggerFront);            //Read the button status
  
  if (RevTriggerStatus == LOW){
    Serial.println("trigger pushed");
    Serial.println("Slow start");
    throttles1.writeMicroseconds(MAXTHROTTLE);
    motorStartTime = millis();
  }
  else if ((millis() - motorStartTime) < motorIdleTime){  // code for going to minthrottle for x seconds after maxthrottle
    unsigned long currentTime = millis();
    while ((millis() - currentTime) < motorIdleTime){
      Serial.println("Motors standig idle");
      throttles1.writeMicroseconds(MINTHROTTLE);
      elapsedTime = 0;
      lastSpindownCheck = millis();
      currentSpeed = MINTHROTTLE;
    }
  }
  else{
    Serial.println("Motors stopped");
    spinDown();
  }

 if(counter != previous_counter)
  {
    Serial.print("Counter: ");
    Serial.println(counter);
  }

               
}             

 /*In this case, the counter will automatically change its value in the interruption. 
  So all we need to do is to print its value in the void loop*/
ISR(PCINT0_vect){  
  cli(); //We pause interrupts happening before we read pin values
  currentStateClock =   (PINB & B00000010);       //Check pin D9 state? Clock
  StateData  =   (PINB & B00000001);              //Check pin D8 state? Data
  if (currentStateClock != lastStateClock){
    // If "clock" state is different "data" state, the encoder is rotating clockwise
    if (StateData != currentStateClock){ 
      counter  ++;                                // We increment   
      lastStateClock = currentStateClock;         // Updates the previous state of the clock with the current state
      sei(); //restart interrupts
    }
    //Else, the encoder is rotating counter-clockwise 
    else {
      counter --;                                 // We decrement
      lastStateClock = currentStateClock;         // Updates  previous state of the clock with the current state    
      sei(); //restart interrupts
    } 
  }  
} 

// setters functions, no setter for the temperature and heater read only
void setSetPoint(int v) { eeprom_write_word(kEepromSetpoint, (uint16_t)v); }
void setHysteresis(int v) { eeprom_write_word(kEepromHysteresis, (uint16_t)v); }

void FirePlunger(void)
{ 
  digitalWrite(Plunger,HIGH);             //write the plunger on
  delay(WaitPlungerOut);                  //wait for the plunger to come out
  digitalWrite(Plunger,LOW);             //write the plunger off
  delay(WaitPlungerIn);                  //wait for the plunger to come out
  while (digitalRead(TriggerBack)== LOW);    //until idiot releases button   
}
void FireBurst(void)
{
   digitalWrite(Plunger,HIGH);             //write the plunger on
  delay(WaitPlungerOut);                  //wait for the plunger to come out
  digitalWrite(Plunger,LOW);             //write the plunger off
  delay(WaitPlungerIn);                  //wait for the plunger to come out
    digitalWrite(Plunger,HIGH);             //write the plunger on
  delay(WaitPlungerOut);                  //wait for the plunger to come out
  digitalWrite(Plunger,LOW);             //write the plunger off
  delay(WaitPlungerIn);                  //wait for the plunger to come out
    digitalWrite(Plunger,HIGH);             //write the plunger on
  delay(WaitPlungerOut);                  //wait for the plunger to come out
  digitalWrite(Plunger,LOW);             //write the plunger off
  delay(WaitPlungerIn);                  //wait for the plunger to come out
  while (digitalRead(TriggerBack)== LOW);    //until idiot releases button    
}
void FireAuto(void)
{
  while (digitalRead(TriggerBack)== LOW)    //until idiot releases button
  {    
  digitalWrite(Plunger,HIGH);             //write the plunger on
  delay(WaitPlungerOut);                  //wait for the plunger to come out
  digitalWrite(Plunger,LOW);             //write the plunger off
  delay(WaitPlungerIn);                  //wait for the plunger to come out 
  }
   
  
}

void spinDown(){
  
  int flipSpindown = (16 - spindownRate) * 2;
  if (currentSpeed > 1040){
    if (spindownRate == 0){
      currentSpeed = 1040;
    }
    else{
      elapsedTime = lastSpindownCheck == 0 ? 0 : elapsedTime + millis() - lastSpindownCheck;
      lastSpindownCheck = millis();
      int spindown = elapsedTime / 10 * flipSpindown;
      elapsedTime %= 10;
      currentSpeed -= spindown;
    }
  }
  throttles1.writeMicroseconds(currentSpeed);
}
