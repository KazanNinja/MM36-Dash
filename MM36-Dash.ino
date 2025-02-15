#include <ESP32-TWAI-CAN.hpp>
#include <Adafruit_NeoPixel.h>

//Setting Neopixel data output pin
//Setting number of Neopixels in chain
#define DATA_PIN 11
#define NUMPIXELS 9
Adafruit_NeoPixel pixels(NUMPIXELS, DATA_PIN, NEO_GRB + NEO_KHZ800);

// CAN TX-RX Pins
#define CAN_TX 34
#define CAN_RX 33

//CAN Recieving frame object
CanFrame rxFrame;

//Defining Tasks
TaskHandle_t CAN_Task;
TaskHandle_t Neopixel_Task;
TaskHandle_t Blink;

//Define 7-segment display segments and their pins


//Defines global rpm, clt, gear, etc integers for CAN data
int rpm;
int clt;
int gear;
int neutral;
int pit;
int driverSwitch3;
int driverSwitch4;

//Defines shfit light rpm switching points
int shiftRpm1 = 9500;
int shiftRpm2 = 10000;
int shiftRpm3 = 10500;
int shiftRpm4 = 11000;
int shiftRpm5 = 11500;
int shiftRpm6 = 12000;
int shiftRpm7 = 12500;

//Define flashing light bools
int flashingRPM = 13000;
bool SHIFTNOWDINGDONG = false;

//Value at which the "Cold" and "Hot" lights are turned on, in Celsius
//Value at which light starts flashing
int coolantCold = 70;
int coolantHot = 105;
int coolantFlash = 120;
bool cltFlash = false;

//Millis for flashing lights
int previousMillis = 0;
const long overheatMillis = 500;
const long pitMillis = 750;

//OBD TX frame setup
void sendObdFrame(uint8_t obdId) {
	CanFrame obdFrame = { 0 };
	obdFrame.identifier = 0x0F3; // Default OBD2 address;
	obdFrame.extd = 0;
	obdFrame.data_length_code = 8;
	obdFrame.data[0] = 2;
	obdFrame.data[1] = 1;
	obdFrame.data[2] = obdId;
	obdFrame.data[3] = 0xAA;    // Best to use 0xAA (0b10101010) instead of 0
	obdFrame.data[4] = 0xAA;    // CAN works better this way as it needs
	obdFrame.data[5] = 0xAA;    // to avoid bit-stuffing
	obdFrame.data[6] = 0xAA;
	obdFrame.data[7] = 0xAA;
  // Accepts both pointers and references 
  //ESP32Can.writeFrame(obdFrame);  // timeout defaults to 1 ms
}

void setup() {
  //Begin Serial comms
  Serial.begin(115200);


  rpm = 8000;
  clt=125;
  //Dash button pinModes
  pinMode(12, INPUT_PULLUP);
  pinMode(13, INPUT_PULLUP);

  //Gear indicator pinModes
  // pinMode(4, OUTPUT);
  // pinMode(5, OUTPUT);
  // pinMode(6, OUTPUT);
  // pinMode(7, OUTPUT);
  // pinMode(8, OUTPUT);
  // pinMode(9, OUTPUT);
  // pinMode(10, OUTPUT);
  // pinMode(38, OUTPUT);

  //Turns gear indicator off
  // digitalWrite(4,HIGH);
  // digitalWrite(5,HIGH);
  // digitalWrite(6,HIGH);
  // digitalWrite(7,HIGH);
  // digitalWrite(8,HIGH);
  // digitalWrite(9,HIGH);
  // digitalWrite(10,HIGH);
  // digitalWrite(38,HIGH);

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

  //Settings up task for Shift Light lighting
  xTaskCreatePinnedToCore(
    Light_Task_Code, //Function to implement the task
    "Light Task",    //Name of the task
    10000,              //Stack size in words
    NULL,               //Task input parameter
    1,                  //Priority of the task
    &Neopixel_Task,     //Task handle
    1                   //Core where the task should run
  );

  //Settings up task for Shift Light lighting
  // xTaskCreatePinnedToCore(
  //   Gear_Indicator_Code, //Function to implement the task
  //   "Gear Indicator Task",    //Name of the task
  //   10000,              //Stack size in words
  //   NULL,               //Task input parameter
  //   1,                  //Priority of the task
  //   &Neopixel_Task,     //Task handle
  //   1                   //Core where the task should run
  // );

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

    //Onboard button
    //int buttonState = digitalRead(35);

    //CAN TX-ing
    // if(currentStamp - lastStamp > 50) {   // sends OBD2 request every second
    //   lastStamp = currentStamp;
    //   sendObdFrame(5); // For coolant temperature
    // }
    
    //Onboard button LED driving, button pressed (0, pulled up) turn the LED on
    // if(buttonState == 0){
    //   digitalWrite(14, HIGH);
    // }
    // else
    // {
    //   digitalWrite(14, LOW);
    // }

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
    
void Light_Task_Code(void *parameter2) {

  while(true){

    unsigned long currentMillis = millis();
    
    //Clears any existing pixels 
    pixels.clear();

    //Coolant lighting Neopixel
    if(clt < coolantCold){
      pixels.setPixelColor(0, pixels.Color(0,0,255));
    }
    else if(clt >= coolantCold && clt <= coolantHot && clt ){
      pixels.setPixelColor(0, pixels.Color(0,0,0));
    }
    else if(clt > coolantHot && clt < coolantFlash){
      pixels.setPixelColor(0, pixels.Color(255,0,0));
    }
    if(clt > coolantFlash){
      Serial.println("reaching0");
      Serial.println(currentMillis);
      Serial.println(previousMillis);
      if((currentMillis - previousMillis) >= overheatMillis){
        Serial.println("reaching1");
        previousMillis = currentMillis;
        if(cltFlash == false){
          Serial.println("reaching2");
          pixels.setPixelColor(0, pixels.Color(255,0,0));
          cltFlash = true;
        }
        else{
          Serial.println("reaching3");
          pixels.setPixelColor(0, pixels.Color(0,0,0));
          cltFlash = false;
        }
      }
    }
    else{
      pixels.setPixelColor(0, pixels.Color(255,255,255));
    }

    //Shift lights When Pit Limiter is Off
    //First Light
    if(pit == 0){
      if(rpm >= shiftRpm1){
        pixels.setPixelColor(1, pixels.Color(0,255,0));
      }
      else{
        pixels.setPixelColor(1, pixels.Color(0,0,0));
      }

      //Second Light
      if(rpm >= shiftRpm2){
        pixels.setPixelColor(2, pixels.Color(0,255,0));
      }
      else{
        pixels.setPixelColor(2, pixels.Color(0,0,0));
      }

      //Third Light
      if(rpm >= shiftRpm3){
        pixels.setPixelColor(3, pixels.Color(0,255,0));
      }
      else{
        pixels.setPixelColor(3, pixels.Color(0,0,0));
      }

      //Fourth Light
      if(rpm >= shiftRpm4){
        pixels.setPixelColor(4, pixels.Color(255,0,0));
      }
      else{
        pixels.setPixelColor(4, pixels.Color(0,0,0));
      }

      //Fifth Light
      if(rpm >= shiftRpm5){
        pixels.setPixelColor(5, pixels.Color(255,0,0));
      }
      else{
        pixels.setPixelColor(5, pixels.Color(0,0,0));
      }

      //Sixth Light
      if(rpm >= shiftRpm6){
        pixels.setPixelColor(6, pixels.Color(255,0,255));
      }
      else{
        pixels.setPixelColor(6, pixels.Color(0,0,0));
      }

      //Seventh Light
      if(rpm >= shiftRpm7){
        pixels.setPixelColor(7, pixels.Color(255,0,255));
      }
      else{
        pixels.setPixelColor(7, pixels.Color(0,0,0));
      }

      //Sets shfit flashing bool to true when rpm passes flashingRPM variable
      if(rpm > flashingRPM){
        SHIFTNOWDINGDONG = true;
      }
      else{
        SHIFTNOWDINGDONG = false;
      }
    }

    // if(rpm < 14000){
    //   rpm++;
    // }
    // else{
    //   rpm = 8000;
    // }
    //Sets pixel output, 1ms delay
    pixels.show();
    //Serial.println(rpm);
    //Serial.println(clt);
    vTaskDelay(0.1);

  }
}
