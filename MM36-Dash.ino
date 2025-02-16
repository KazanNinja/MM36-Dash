#include <ESP32-TWAI-CAN.hpp>
#include <Adafruit_NeoPixel.h>

//Setting Neopixel data output pin
//Setting number of Neopixels in chain
#define DATA_PIN 11
#define NUMPIXELS 9
Adafruit_NeoPixel pixels(NUMPIXELS, DATA_PIN, NEO_GRB + NEO_KHZ800);

// CAN TX-RX Pins
#define CAN_RX 33
#define CAN_TX 34

// Button Pins
#define Button1Pin 13
#define Button2Pin 12
#define Button3Pin 35
#define Button4Pin 17

//Define 7-segment display segments and their pins on the ESP
#define GearA 38
#define GearB 10
#define GearC 5
#define GearD 7
#define GearE 6
#define GearF 9
#define GearG 8
#define GearDP 4

//CAN Recieving frame
//Sets Transmitting frame ID
CanFrame rxFrame;
int frameID = 0x7F6;

//Defining Tasks
TaskHandle_t CAN_Task;
TaskHandle_t Neopixel_Task;
TaskHandle_t Button_Task;
TaskHandle_t Gear_Task;

//Creates global rpm, clt, gear, etc integers for CAN data
int rpm;
int gear;
int neutral;
int pit;
int button1State;
int button2State;
int button3State;
int button4State;
int clt;

//Defines shfit light rpm switching points
int shiftRpm1 = 9500;
int shiftRpm2 = 10000;
int shiftRpm3 = 10500;
int shiftRpm4 = 11000;
int shiftRpm5 = 11500;
int shiftRpm6 = 12000;
int shiftRpm7 = 12500;
int flashingRPM = 13000;

//Define flashing light bools
bool rpmFlashState = false;
bool pitFlashState = false;
bool cltFlashState = false;

//Value at which the "Cold" and "Hot" lights are turned on, in Celsius
//Value at which light starts flashing
int coolantCold = 70;
int coolantHot = 105;
int coolantFlash = 115;

//Millis for flashing lights
//Intervals too
int previousMillis = 0;
int previousMillis2 = 0;
int previousMillis3 = 0;
const long overheatMillis = 200;
const long pitMillis = 350;
const long shiftMillis = 125;

void setup() {
  //Begin Serial comms
  Serial.begin(115200);

  //Testing Values
  rpm = 8000;
  clt=30;
  pit = 0;
  neutral = 0;

  //Dash button pinModes
  //Sets pull-up because buttons are pulldown when closed
  pinMode(Button1Pin, INPUT_PULLUP);
  pinMode(Button2Pin, INPUT_PULLUP);
  pinMode(Button3Pin, INPUT_PULLUP);
  pinMode(Button4Pin, INPUT_PULLUP);

  //Gear indicator pinModes
  pinMode(GearA, OUTPUT);
  pinMode(GearB, OUTPUT);
  pinMode(GearC, OUTPUT);
  pinMode(GearD, OUTPUT);
  pinMode(GearE, OUTPUT);
  pinMode(GearF, OUTPUT);
  pinMode(GearG, OUTPUT);
  pinMode(GearDP, OUTPUT);

  //Turns gear indicator off (INVERTED RIGHT NOW, USING COMMONG CATHODE RN)
  // digitalWrite(GearA,LOW);
  // digitalWrite(GearB,LOW);
  // digitalWrite(GearC, LOW);
  // digitalWrite(GearD, LOW);
  // digitalWrite(GearE, LOW);
  // digitalWrite(GearF, LOW);
  // digitalWrite(GearG, LOW);
  // digitalWrite(GearDP, LOW);

  //Setting up task for CAN bus stuffz, grabs data and stuff
  xTaskCreatePinnedToCore(
    CAN_Task_Code, //Function to implement the task
    "CAN Task",    //Name of the task
    10000,         //Stack size in words
    NULL,          //Task input parameter
    0,             //Priority of the task
    &CAN_Task,     //Task handle
    0              //Core where the task should run
  );            

  //Settings up task for Lighting 
  xTaskCreatePinnedToCore(
    Light_Task_Code, //Function to implement the task
    "Light Task",    //Name of the task
    10000,              //Stack size in words
    NULL,               //Task input parameter
    1,                  //Priority of the task
    &Neopixel_Task,     //Task handle
    1                   //Core where the task should run
  );

  //Settings up task for dash buttons 
  xTaskCreatePinnedToCore(
    Button_Task_Code,   //Function to implement the task
    "Button Task",      //Name of the task
    10000,              //Stack size in words
    NULL,               //Task input parameter
    1,                  //Priority of the task
    &Button_Task,       //Task handle
    1                   //Core where the task should run
  );

  //Settings up task for gear indicator stuff
  xTaskCreatePinnedToCore(
    Gear_Indicator_Code, //Function to implement the task
    "Gear Indicator Task",    //Name of the task
    10000,              //Stack size in words
    NULL,               //Task input parameter
    1,                  //Priority of the task
    &Gear_Task,         //Task handle
    1                   //Core where the task should run
  );

  //CAN setup
  ESP32Can.setPins(CAN_TX, CAN_RX);
  ESP32Can.setRxQueueSize(5);
  ESP32Can.setTxQueueSize(5);
  ESP32Can.setSpeed(ESP32Can.convertSpeed(1000));

  // You can also just use .begin()..
  if(ESP32Can.begin()) {
      Serial.println("CAN bus started!");
  } else {
      Serial.println("CAN bus failed!");
  }
}

void loop(){
  //Do nothing :)
}

void CAN_Task_Code(void *parameter) {

  while(true){

    //Shitasses
    static uint32_t lastStamp = 0;
    uint32_t currentStamp = millis();

    //CAN TX-ing, sends dash button states
    if(currentStamp - lastStamp > 50) {
      lastStamp = currentStamp;
      SendButtonCAN();
    }

    //Checks if there are any frames to read
    if(ESP32Can.readFrame(rxFrame, 1000)) {

      //Engine Speed CAN Frame
      if(rxFrame.identifier == 0x640) {  
        byte rpmLow = rxFrame.data[0];
        byte rpmHigh = rxFrame.data[1];
        rpm = (rpmLow << 8) + rpmHigh;
      }
    
      //Goolant CAN Frame
      if(rxFrame.identifier == 0x649){
        clt = rxFrame.data[0] - 40;
      }

      //Gear position CAN Frame
      if(rxFrame.identifier == 0x64D){
        gear = rxFrame.data[6] & 0b00001111;
      }

      //Pit Switch CAN Frame
      if(rxFrame.identifier == 0x64E){
        neutral = rxFrame.data[3] & 0b01000000;
      }

      //Neutral Switch CAN Frame
      if(rxFrame.identifier == 0x64E){
        pit = rxFrame.data[3] & 0b00000001;
      }
    }
  }
}
    
void Light_Task_Code(void *parameter) {

  while(true){

    unsigned long currentMillis = millis();

    //Coolant lighting Neopixel
    //First three "simple" coolant states
    //Cold, operating temp, hot
    if(clt < coolantCold){
      pixels.setPixelColor(0, pixels.Color(0,0,255));
    }
    else if(clt >= coolantCold && clt <= coolantHot){
      pixels.setPixelColor(0, pixels.Color(0,0,0));
    }
    else if(clt > coolantHot && clt < coolantFlash){
      pixels.setPixelColor(0, pixels.Color(255,0,0));
    }

    //Flashing coolant light for when coolant temp is exceeding coolantFlash threshold
    //uses currentMillis, previousMillis, overheatMillis to see how much time has 
    //passed since conditions met, turns coolant light on or off based on that interval
    else if(clt > coolantFlash && cltFlashState && (currentMillis-previousMillis >= overheatMillis)){
        previousMillis = currentMillis;
        cltFlashState = false;
        pixels.setPixelColor(0, pixels.Color(0,0,0));
    }
    else if(clt > coolantFlash && !cltFlashState && (currentMillis - previousMillis >= overheatMillis)){
        previousMillis = currentMillis;
        cltFlashState = true;
        pixels.setPixelColor(0, pixels.Color(255,0,0));
    }

    //Shift lights When Pit Switch is Off
    //First Light
    if(pit == 0){

      if(rpm >= shiftRpm1 && rpm < flashingRPM){
        pixels.setPixelColor(1, pixels.Color(0,100,0));
      }
      else if(rpm <= shiftRpm1){
        pixels.setPixelColor(1, pixels.Color(0,0,0));
      }

      //Second Light
      if(rpm >= shiftRpm2 && rpm < flashingRPM){
        pixels.setPixelColor(2, pixels.Color(0,100,0));
      }
      else if(rpm <= shiftRpm2){
        pixels.setPixelColor(2, pixels.Color(0,0,0));
      }

      //Third Light
      if(rpm >= shiftRpm3 && rpm < flashingRPM){
        pixels.setPixelColor(3, pixels.Color(0,100,0));
      }
      else if(rpm <= shiftRpm3){
        pixels.setPixelColor(3, pixels.Color(0,0,0));
      }

      //Fourth Light
      if(rpm >= shiftRpm4 && rpm < flashingRPM){
        pixels.setPixelColor(4, pixels.Color(100,100,0));
      }
      else if(rpm <= shiftRpm4){
        pixels.setPixelColor(4, pixels.Color(0,0,0));
      }

      //Fifth Light
      if(rpm >= shiftRpm5 && rpm < flashingRPM){
        pixels.setPixelColor(5, pixels.Color(100,100,0));
      }
      else if(rpm <= shiftRpm5){
        pixels.setPixelColor(5, pixels.Color(0,0,0));
      }

      //Sixth Light
      if(rpm >= shiftRpm6 && rpm < flashingRPM){
        pixels.setPixelColor(6, pixels.Color(100,0,0));
      }
      else if(rpm <= shiftRpm6){
        pixels.setPixelColor(6, pixels.Color(0,0,0));
      }

      //Seventh Light
      if(rpm >= shiftRpm7 && rpm < flashingRPM){
        pixels.setPixelColor(7, pixels.Color(100,0,0));
      }
      else if(rpm <= shiftRpm7){
        pixels.setPixelColor(7, pixels.Color(0,0,0));
      }

      //Flashing RPM Light Stuffz
      //Similar to flashing coolant light
      //Uses millis and stuffs to find interval of time to turn on or off the
      //rpm sequence lights
      if(rpm > flashingRPM){
        if(rpmFlashState && (currentMillis-previousMillis3 >= shiftMillis)){
          previousMillis3 = currentMillis;
          rpmFlashState = false;
          pixels.setPixelColor(1, pixels.Color(0,0,0));
          pixels.setPixelColor(2, pixels.Color(0,0,0));
          pixels.setPixelColor(3, pixels.Color(0,0,0));
          pixels.setPixelColor(4, pixels.Color(0,0,0));
          pixels.setPixelColor(5, pixels.Color(0,0,0));
          pixels.setPixelColor(6, pixels.Color(0,0,0));
          pixels.setPixelColor(7, pixels.Color(0,0,0));
        }
        else if(!rpmFlashState && (currentMillis - previousMillis3 >= shiftMillis)){
          previousMillis3 = currentMillis;
          rpmFlashState = true;
          pixels.setPixelColor(1, pixels.Color(100,0,100));
          pixels.setPixelColor(2, pixels.Color(100,0,100));
          pixels.setPixelColor(3, pixels.Color(100,0,100));
          pixels.setPixelColor(4, pixels.Color(100,0,100));
          pixels.setPixelColor(5, pixels.Color(100,0,100));
          pixels.setPixelColor(6, pixels.Color(100,0,100));
          pixels.setPixelColor(7, pixels.Color(100,0,100));
          //Porple drank
        }
      }
    }

    //Pit Limiter Lights
    //Alternates between center thingies if pit switch is on
    //Same millis stuff going on up above
    if(pit == 1){
      if(pitFlashState && (currentMillis-previousMillis2 >= pitMillis)){
        previousMillis2 = currentMillis;
        pitFlashState = false;
        pixels.setPixelColor(1, pixels.Color(0,0,0));
        pixels.setPixelColor(2, pixels.Color(125,100,0));
        pixels.setPixelColor(3, pixels.Color(0,0,0));
        pixels.setPixelColor(4, pixels.Color(125,100,0));
        pixels.setPixelColor(5, pixels.Color(0,0,0));
        pixels.setPixelColor(6, pixels.Color(125,100,0));
        pixels.setPixelColor(7, pixels.Color(0,0,0));
      }
      if(!pitFlashState && (currentMillis - previousMillis2 >= pitMillis)){
        previousMillis2 = currentMillis;
        pitFlashState = true;
        pixels.setPixelColor(1, pixels.Color(125,100,0));
        pixels.setPixelColor(2, pixels.Color(0,0,0));
        pixels.setPixelColor(3, pixels.Color(125,100,0));
        pixels.setPixelColor(4, pixels.Color(0,0,0));
        pixels.setPixelColor(5, pixels.Color(125,100,0));
        pixels.setPixelColor(6, pixels.Color(0,0,0));
        pixels.setPixelColor(7, pixels.Color(125,100,0));
      }
    }

    //Neutral switch
    if(neutral == 1){
      pixels.setPixelColor(8, pixels.Color(0,255,0));
    }
    else{
      pixels.setPixelColor(8, pixels.Color(0,0,0));
    }
      
    //Testing stuff
    if(rpm < 14000){
      rpm = rpm +2;
    }
    else{
      rpm = 8000;
    }

    if(clt < 240){
      clt = clt + 0.05;
    }
    else{
      clt = 20;
    }

    rpm = 10000;
    clt =  70;
    //Set neopixels to set values
    //Delay to yield to other tasks
    pixels.show();
    vTaskDelay(0.1);
  }
}

void Button_Task_Code(void *parameter){
  while(true){
    button1State = digitalRead(Button1Pin);
    button2State = digitalRead(Button2Pin);
    button3State = digitalRead(Button3Pin);
    button4State = digitalRead(Button4Pin);

    vTaskDelay(0.1);
  }
}

void Gear_Indicator_Code(void *parameter){
  while(true){

    //Neutral
    if(gear == 0){
      digitalWrite(GearA, LOW);
      digitalWrite(GearB, LOW);
      digitalWrite(GearC, LOW);
      digitalWrite(GearE, LOW);
      digitalWrite(GearF, LOW);
    }
    else if(gear == 1){
      digitalWrite(GearB, LOW);
      digitalWrite(GearC, LOW);
    }
    else if(gear == 2){
      digitalWrite(GearA, LOW);
      digitalWrite(GearB, LOW);
      digitalWrite(GearD, LOW);
      digitalWrite(GearE, LOW);
      digitalWrite(GearG, LOW);
    }
    else if(gear == 3){
      digitalWrite(GearA, LOW);
      digitalWrite(GearB, LOW);
      digitalWrite(GearC, LOW);
      digitalWrite(GearD, LOW);
      digitalWrite(GearG, LOW);
    }
    else if(gear == 4){
      digitalWrite(GearB, LOW);
      digitalWrite(GearC, LOW);
      digitalWrite(GearF, LOW);
      digitalWrite(GearG, LOW);
    }
    else if(gear == 5){
      digitalWrite(GearA, LOW);
      digitalWrite(GearC, LOW);
      digitalWrite(GearD, LOW);
      digitalWrite(GearF, LOW);
      digitalWrite(GearG, LOW);
    }
    else if(gear == 6){
      digitalWrite(GearA, LOW);
      digitalWrite(GearC, LOW);
      digitalWrite(GearD, LOW);
      digitalWrite(GearE, LOW);
      digitalWrite(GearF, LOW);
      digitalWrite(GearG, LOW);
    }
    else{
      digitalWrite(GearA, LOW);
      digitalWrite(GearB, LOW);
      digitalWrite(GearC, LOW);
      digitalWrite(GearD, LOW);
      digitalWrite(GearE, LOW);
      digitalWrite(GearF, LOW);
      digitalWrite(GearDP, LOW);
    }

    vTaskDelay(0.1);
  }
}


void SendButtonCAN() {
	CanFrame obdFrame = { 0 };
	obdFrame.identifier = frameID;
	obdFrame.extd = 0;
	obdFrame.data_length_code = 8;
	obdFrame.data[0] = 0;
	obdFrame.data[2] = 0;
	obdFrame.data[4] = 0; 
	obdFrame.data[6] = 0;

  //Sets the second byte of the byte pair to 255 depending on button state
  //Does this for all 4 buttons
  //Theres probably a better way to do this
  //Figure the rest out in M1 Tune
  if(button1State == 0){
	  obdFrame.data[1] = 0b11111111;
  }
  else{
    obdFrame.data[1] = 0b00000000;
  }

  if(button2State == 0){
    obdFrame.data[3] = 0b11111111;
  }
  else{
    obdFrame.data[3] = 0b00000000;
  }
	
  if(button3State == 0){
	  obdFrame.data[5] = 0b11111111;
  }
  else{
    obdFrame.data[5] = 0b00000000;
  }

  if(button4State == 0){
	  obdFrame.data[7] = 0b11111111;
  }
  else{
    obdFrame.data[7] = 0b00000000;
  }
  ESP32Can.writeFrame(obdFrame);  // timeout defaults to 1 ms
}
