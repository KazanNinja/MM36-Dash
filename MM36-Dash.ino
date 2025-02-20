#include <ESP32-TWAI-CAN.hpp>
#include <Adafruit_NeoPixel.h>

//Setting Neopixel data output pin
//Setting number of Neopixels in chain
#define DATA_PIN 11
#define NUMPIXELS 9
Adafruit_NeoPixel pixels(NUMPIXELS, DATA_PIN, NEO_GRB + NEO_KHZ800);

//CAN TX-RX Pins
#define CAN_RX 33
#define CAN_TX 34

//Button Pins
#define Button1Pin 13
#define Button2Pin 12
#define Button3Pin 35
#define Button4Pin 17

//Define 7-segment display segments and their pins on the ESP
#define GearA 7
#define GearB 6
#define GearC 9
#define GearD 38
#define GearE 10
#define GearF 5
#define GearG 4
#define GearDP 8

//Status light pin
#define StatusLED 47

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
const long shiftMillis = 100;

//Gear "debounce" thing to ignore the -2 Motec value in between shifts
//Sets last gear position var and ignore duration
const long gearIgnoreMillis = 700;
long gear14Timer = 0;

void setup() {
  //Begin Serial comms
  Serial.begin(115200);

  //Sets clt to -69 to prevent light coming on prior to connection with Motec
  //Sets gear position to 14 prior to connection with Motec
  clt = -69;
  gear = 14;

  //Sets StatusLED pin to output
  pinMode(StatusLED, OUTPUT);

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

  //Clears gear indicator
  digitalWrite(GearA, HIGH);
  digitalWrite(GearB, HIGH);
  digitalWrite(GearC, HIGH);
  digitalWrite(GearD, HIGH);      
  digitalWrite(GearE, HIGH);
  digitalWrite(GearF, HIGH);
  digitalWrite(GearG, HIGH);
  digitalWrite(GearDP, HIGH);

  //Slight Delay for stuff
  delay(200);

  //Setting up task for CAN bus stuffz, grabs data and stuff
  xTaskCreatePinnedToCore(
    CAN_Task_Code, //Function to implement the task
    "CAN Task",    //Name of the task
    4096,         //Stack size in words
    NULL,          //Task input parameter
    0,             //Priority of the task
    &CAN_Task,     //Task handle
    1              //Core where the task should run
  );            

  //Settings up task for Lighting 
  xTaskCreatePinnedToCore(
    Light_Task_Code, //Function to implement the task
    "Light Task",    //Name of the task
    4096,              //Stack size in words
    NULL,               //Task input parameter
    1,                  //Priority of the task
    &Neopixel_Task,     //Task handle
    1                   //Core where the task should run
  );

  //Settings up task for dash buttons 
  xTaskCreate(
    Button_Task_Code,   //Function to implement the task
    "Button Task",      //Name of the task
    2048,              //Stack size in words
    NULL,               //Task input parameter
    4,                  //Priority of the task
    &Button_Task       //Task handle
    //1,                   //Core where the task should run
  );

  //Settings up task for gear indicator stuff
  xTaskCreatePinnedToCore(
    Gear_Indicator_Code, //Function to implement the task
    "Gear Indicator Task",    //Name of the task
    4096,              //Stack size in words
    NULL,               //Task input parameter
    2,                  //Priority of the task
    &Gear_Task,         //Task handle
    1                   //Core where the task should run
  );

  //CAN setup
  ESP32Can.setPins(CAN_TX, CAN_RX);
  ESP32Can.setRxQueueSize(64);
  ESP32Can.setTxQueueSize(64);
  ESP32Can.setSpeed(ESP32Can.convertSpeed(1000));

  // You can also just use .begin()..
  if(ESP32Can.begin()) {
      Serial.println("CAN bus started!");
      digitalWrite(StatusLED, HIGH);
      pixels.setPixelColor(8, pixels.Color(255,255,255));
      pixels.setPixelColor(0, pixels.Color(255,255,255));
      pixels.show();
      delay(75);
      digitalWrite(StatusLED, LOW);
      delay(75);
      digitalWrite(StatusLED, HIGH);
      delay(75);
      digitalWrite(StatusLED, LOW);
      pixels.setPixelColor(8, pixels.Color(0,0,0));
      pixels.setPixelColor(0, pixels.Color(0,0,0));
      pixels.show();
  } 
  else {
      Serial.println("CAN bus failed!");
      digitalWrite(StatusLED, LOW);
  }

  //Clears pixels
  pixels.clear();
}

void loop(){
  vTaskDelete(NULL);
  //Do nothing :)
}

void CAN_Task_Code(void *parameter) {
  Serial.println("Running CAN Task");

  while(true){

    //Sets StatusLED off until CAN talk begins
    digitalWrite(StatusLED, LOW);

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

      //Turns statusLED on during rx'ing
      digitalWrite(StatusLED, HIGH);

      //Engine Speed CAN Frame
      if(rxFrame.identifier == 0x640) {  
        byte rpmLow = rxFrame.data[0];
        byte rpmHigh = rxFrame.data[1];
        rpm = (rpmLow << 8) + rpmHigh;
        //Serial.println(rpm);
      }
    
      //Goolant CAN Frame
      if(rxFrame.identifier == 0x649){
        clt = rxFrame.data[0] - 40;
        //Serial.println(clt);
      }

      //Gear position CAN Frame
      if(rxFrame.identifier == 0x64D){
        gear = rxFrame.data[6] & 0b00001111;
        //Serial.println(gear);
      }

      //Pit Switch CAN Frame
      if(rxFrame.identifier == 0x64E){
        pit = rxFrame.data[3] & 0b01000000;
        Serial.println(pit);
      }

      //Neutral Switch CAN Frame
      if(rxFrame.identifier == 0x64E){
        neutral = rxFrame.data[3] & 0b00000001;
        //Serial.println(neutral);
      }
    }

    //Delay to yield to other tasks
    //vTaskDelay(1);
  }
}
    
void Light_Task_Code(void *parameter) {
  //Serial.println("Running Light Task");

  while(true){

    unsigned long currentMillis = millis();

    //Coolant lighting Neopixel
    //First three "simple" coolant states
    //Cold, operating temp, hot
    if(clt < coolantCold && clt != -69){
      pixels.setPixelColor(0, pixels.Color(0,0,255));
    }
    else if(clt >= coolantCold && clt <= coolantHot){
      pixels.setPixelColor(0, pixels.Color(0,64,0));
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
      if(rpm < shiftRpm1 && rpm < flashingRPM){
        pixels.setPixelColor(1, pixels.Color(0,0,0));
        pixels.setPixelColor(2, pixels.Color(0,0,0));
        pixels.setPixelColor(3, pixels.Color(0,0,0));
        pixels.setPixelColor(4, pixels.Color(0,0,0));
        pixels.setPixelColor(5, pixels.Color(0,0,0));
        pixels.setPixelColor(6, pixels.Color(0,0,0));
        pixels.setPixelColor(7, pixels.Color(0,0,0));
      }

      //First Light
      if(rpm >= shiftRpm1 && rpm < flashingRPM){
        pixels.setPixelColor(1, pixels.Color(0,255,0));
        pixels.setPixelColor(7, pixels.Color(0,255,0));
      }
      else if(rpm <= shiftRpm1){
        pixels.setPixelColor(1, pixels.Color(0,0,0));
        pixels.setPixelColor(7, pixels.Color(0,0,0));
      }

      //Second Light
      if(rpm >= shiftRpm3 && rpm < flashingRPM){
        pixels.setPixelColor(2, pixels.Color(255,255,0));
        pixels.setPixelColor(6, pixels.Color(255,255,0));
      }
      else if(rpm <= shiftRpm3){
        pixels.setPixelColor(2, pixels.Color(0,0,0));
        pixels.setPixelColor(6, pixels.Color(0,0,0));
      }

      //Third Light
      if(rpm >= shiftRpm5 && rpm < flashingRPM){
        pixels.setPixelColor(3, pixels.Color(255,0,0));
        pixels.setPixelColor(5, pixels.Color(255,0,0));
      }
      else if(rpm <= shiftRpm5){
        pixels.setPixelColor(3, pixels.Color(0,0,0));
        pixels.setPixelColor(3, pixels.Color(0,0,0));
      }

      //Fourth Light
      if(rpm >= shiftRpm7 && rpm < flashingRPM){
        pixels.setPixelColor(4, pixels.Color(255,0,0));
      }
      else if(rpm <= shiftRpm7){
        pixels.setPixelColor(4, pixels.Color(0,0,0));
      }

      //Fifth Light
      // if(rpm >= shiftRpm5 && rpm < flashingRPM){
      //   pixels.setPixelColor(5, pixels.Color(255,255,0));
      // }
      // else if(rpm <= shiftRpm5){
      //   pixels.setPixelColor(5, pixels.Color(0,0,0));
      // }

      // //Sixth Light
      // if(rpm >= shiftRpm6 && rpm < flashingRPM){
      //   pixels.setPixelColor(6, pixels.Color(255,0,0));
      // }
      // else if(rpm <= shiftRpm6){
      //   pixels.setPixelColor(6, pixels.Color(0,0,0));
      // }

      // //Seventh Light
      // if(rpm >= shiftRpm7 && rpm < flashingRPM){
      //   pixels.setPixelColor(7, pixels.Color(255,0,0));
      // }
      // else if(rpm <= shiftRpm7){
      //   pixels.setPixelColor(7, pixels.Color(0,0,0));
      // }

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
          pixels.setPixelColor(1, pixels.Color(255,0,255));
          pixels.setPixelColor(2, pixels.Color(255,0,255));
          pixels.setPixelColor(3, pixels.Color(255,0,255));
          pixels.setPixelColor(4, pixels.Color(255,0,255));
          pixels.setPixelColor(5, pixels.Color(255,0,255));
          pixels.setPixelColor(6, pixels.Color(255,0,255));
          pixels.setPixelColor(7, pixels.Color(255,0,255));
          //Porple drank
        }
      }
    }

    //Pit Limiter Lights
    //Alternates between center thingies if pit switch is on
    //Same millis stuff going on up above
    if(pit == 64){
      if(pitFlashState && (currentMillis-previousMillis2 >= pitMillis)){
        previousMillis2 = currentMillis;
        pitFlashState = false;
        pixels.setPixelColor(1, pixels.Color(0,0,0));
        pixels.setPixelColor(2, pixels.Color(255,180,0));
        pixels.setPixelColor(3, pixels.Color(0,0,0));
        pixels.setPixelColor(4, pixels.Color(255,180,0));
        pixels.setPixelColor(5, pixels.Color(0,0,0));
        pixels.setPixelColor(6, pixels.Color(255,180,0));
        pixels.setPixelColor(7, pixels.Color(0,0,0));
      }
      if(!pitFlashState && (currentMillis - previousMillis2 >= pitMillis)){
        previousMillis2 = currentMillis;
        pitFlashState = true;
        pixels.setPixelColor(1, pixels.Color(255,180,0));
        pixels.setPixelColor(2, pixels.Color(0,0,0));
        pixels.setPixelColor(3, pixels.Color(255,180,0));
        pixels.setPixelColor(4, pixels.Color(0,0,0));
        pixels.setPixelColor(5, pixels.Color(255,180,0));
        pixels.setPixelColor(6, pixels.Color(0,0,0));
        pixels.setPixelColor(7, pixels.Color(255,180,0));
      }
    }

    //Neutral switch
    if(neutral == 1){
      pixels.setPixelColor(8, pixels.Color(0,255,0));
    }
    else{
      pixels.setPixelColor(8, pixels.Color(0,0,0));
    }
      
    //Set neopixels to set values
    //Delay to yield to other tasks
    pixels.show();
    vTaskDelay(1);
  }
}

void Button_Task_Code(void *parameter){
  //Serial.println("Running Button Task");

  while(true){

    //Grabs digital states of the 4 expansion buttons
    button1State = digitalRead(Button1Pin);
    button2State = digitalRead(Button2Pin);
    button3State = digitalRead(Button3Pin);
    button4State = digitalRead(Button4Pin);

    //Delay to yield to other tasks
    vTaskDelay(50);
  }
}

void Gear_Indicator_Code(void *parameter){
  
  while(true){
    
    unsigned long currentMillis = millis();

    //Neutral
    if(gear == 0 || neutral == 1){
      digitalWrite(GearA, LOW);
      digitalWrite(GearB, LOW);
      digitalWrite(GearC, LOW);
      digitalWrite(GearD, HIGH);
      digitalWrite(GearE, LOW);
      digitalWrite(GearF, LOW);
      digitalWrite(GearG, HIGH);
      digitalWrite(GearDP, HIGH);
    }
    //First Gear
    else if(gear == 1){
      digitalWrite(GearA, HIGH);
      digitalWrite(GearB, LOW);
      digitalWrite(GearC, LOW);
      digitalWrite(GearD, HIGH);
      digitalWrite(GearE, HIGH);
      digitalWrite(GearF, HIGH);
      digitalWrite(GearG, HIGH);
      digitalWrite(GearDP, HIGH);
    }
    //Second Gear
    else if(gear == 2){
      digitalWrite(GearA, LOW);
      digitalWrite(GearB, LOW);
      digitalWrite(GearC, HIGH);
      digitalWrite(GearD, LOW);
      digitalWrite(GearE, LOW);
      digitalWrite(GearG, LOW);
      digitalWrite(GearDP, HIGH);

    }
    //Third Gear
    else if(gear == 3){
      digitalWrite(GearA, LOW);
      digitalWrite(GearB, LOW);
      digitalWrite(GearC, LOW);
      digitalWrite(GearD, LOW);
      digitalWrite(GearE, HIGH);
      digitalWrite(GearF, HIGH);
      digitalWrite(GearG, LOW);
      digitalWrite(GearDP, HIGH);
    }
    //Fourth Gear
    else if(gear == 4){
      digitalWrite(GearA, HIGH);
      digitalWrite(GearB, LOW);
      digitalWrite(GearC, LOW);
      digitalWrite(GearD, HIGH);
      digitalWrite(GearE, HIGH);
      digitalWrite(GearF, LOW);
      digitalWrite(GearG, LOW);
      digitalWrite(GearDP, HIGH);
    }
    //Fifth Gear
    else if(gear == 5){
      digitalWrite(GearA, LOW);
      digitalWrite(GearB, HIGH);
      digitalWrite(GearC, LOW);
      digitalWrite(GearD, LOW);
      digitalWrite(GearE, HIGH);
      digitalWrite(GearF, LOW);
      digitalWrite(GearG, LOW);
      digitalWrite(GearDP, HIGH);
    }
    //Sixth Gear
    else if(gear == 6){
      digitalWrite(GearA, LOW);
      digitalWrite(GearB, HIGH);
      digitalWrite(GearC, LOW);
      digitalWrite(GearD, LOW);
      digitalWrite(GearE, LOW);
      digitalWrite(GearF, LOW);
      digitalWrite(GearG, LOW);
      digitalWrite(GearDP, HIGH);
    }

    //If no known gears are seen, display an E with the DP
    //Uses millis and stuff to ignore the time period in between gear shifts
    //As the Motec estimates the gear position based on wheel speed, rpm, gear ratio, etc
    //If window of time is exceeded, segment will display the E and DP
    if(gear == 14){

      //Starts timer thing if gear position is 14
      if(gear14Timer == 0){
        gear14Timer = currentMillis;
      }
    
      //If timer and currentmillis exceed the timer interval gearIgnoreMillis
      //Display the E and DP
      else if(currentMillis - gear14Timer >= gearIgnoreMillis){    
        digitalWrite(GearA, LOW);
        digitalWrite(GearB, HIGH);
        digitalWrite(GearC, HIGH);
        digitalWrite(GearD, LOW);
        digitalWrite(GearE, LOW);
        digitalWrite(GearF, LOW);
        digitalWrite(GearG, LOW);
        digitalWrite(GearDP, LOW);
      }
    }
    else{
      //Resets gear 14 timer when its not in gear 14
      gear14Timer = 0;
    }

    //Serial.println(gear);
    //Delay to yield to other tasks
    vTaskDelay(2);
  }
}

//CAN Sending stuff for the dash buttons
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
