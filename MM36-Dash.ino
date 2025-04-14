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
int battery;
int button1State;
int button2State;
int button3State;
int button4State;
int clt;
bool button1LastState = HIGH;
bool button2LastState = HIGH;

//Defines shfit light rpm switching points
int shiftRpm1 = 6000;
int shiftRpm3 = 7000;
int shiftRpm5 = 8000;
int shiftRpm7 = 9000;
int flashingRPM = 9500;

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

//Timestamp stuff
unsigned long last;
unsigned long lastButton;

//Gear "debounce" thing to ignore the -2 Motec value in between shifts
//Sets last gear position var and ignore duration
const long gearIgnoreMillis = 700;
long gear14Timer = 0;

//Boot wave
int frame = 0;
bool waveDone = false;
uint8_t brightness[NUMPIXELS] = { 0 };
#define FADE_STEP 2
bool fading[NUMPIXELS] = { false };


void setup() {
  // Setup serial for debbuging.
  Serial.begin(115200);

  //Sets clt to -69 to prevent light coming on prior to connection with Motec
  //Sets gear position to 14 prior to connection with Motec
  clt = -69;
  gear = 14;

  //Gear indicator pinModes
  pinMode(GearA, OUTPUT);
  pinMode(GearB, OUTPUT);
  pinMode(GearC, OUTPUT);
  pinMode(GearD, OUTPUT);
  pinMode(GearE, OUTPUT);
  pinMode(GearF, OUTPUT);
  pinMode(GearG, OUTPUT);
  pinMode(GearDP, OUTPUT);

  //Dash button pinModes
  //Sets pull-up because buttons are pulldown when closed
  pinMode(Button1Pin, INPUT_PULLUP);
  pinMode(Button2Pin, INPUT_PULLUP);
  pinMode(Button3Pin, INPUT_PULLUP);
  pinMode(Button4Pin, INPUT_PULLUP);

  //Clears gear indicator
  digitalWrite(GearA, HIGH);
  digitalWrite(GearB, HIGH);
  digitalWrite(GearC, HIGH);
  digitalWrite(GearD, HIGH);
  digitalWrite(GearE, HIGH);
  digitalWrite(GearF, HIGH);
  digitalWrite(GearG, HIGH);
  digitalWrite(GearDP, HIGH);

  // Set pins
  ESP32Can.setPins(CAN_TX, CAN_RX);

  // You can set custom size for the queues - those are default
  ESP32Can.setRxQueueSize(5);
  ESP32Can.setTxQueueSize(5);

  //Delay for USB comms
  delay(500);

  // or override everything in one command;
  // It is also safe to use .begin() without .end() as it calls it internally
  if (ESP32Can.begin(ESP32Can.convertSpeed(1000), CAN_TX, CAN_RX, 10, 10)) {
    int center = NUMPIXELS / 2;

    // Loop through each outward pair from center
    for (int frame = 0; frame <= center; frame++) {
      int left = center - frame;
      int right = center + frame;

      // Start new LEDs
      if (left >= 0) {
        brightness[left] = 255;
        fading[left] = true;
      }
      if (right < NUMPIXELS) {
        brightness[right] = 255;
        fading[right] = true;
      }

      // Do a few fade steps between each frame
      for (int step = 0; step < (255 / FADE_STEP) / 2; step++) {
        for (int i = 0; i < NUMPIXELS; i++) {
          if (fading[i] && brightness[i] > 0) {
            brightness[i] = (brightness[i] > FADE_STEP) ? brightness[i] - FADE_STEP : 0;
          }

          pixels.setPixelColor(i, pixels.Color(brightness[i], 0, 0));
        }

        pixels.show();
        delay(2);
      }
    }

    // Continue fading any remaining brightness until fully off
    bool anyOn = true;
    while (anyOn) {
      anyOn = false;

      for (int i = 0; i < NUMPIXELS; i++) {
        if (brightness[i] > 0) {
          brightness[i] = (brightness[i] > FADE_STEP) ? brightness[i] - FADE_STEP : 0;
          anyOn = true;
        }
        pixels.setPixelColor(i, pixels.Color(brightness[i], 0, 0));
      }

      pixels.show();
      delay(2);
    }

  } else {
    Serial.println("CAN bus failed!");
  }
}

void loop() {
  uint32_t currentStamp = millis();
  unsigned long currentMillis = millis();


  if (currentStamp - lastButton > 50) {  // sends OBD2 request every 20 ms
    lastButton = currentStamp;
    sendObdFrame();  // For coolant temperature
  }

  //Checks if there are any frames to read
  if (ESP32Can.readFrame(rxFrame, 50)) {
    //Serial.println(rxFrame.identifier, HEX);
    //Turns statusLED on during rx'ing
    // digitalWrite(StatusLED, HIGH);

    // //Engine Speed CAN Frame
    if (rxFrame.identifier == 0x640) {
      byte rpmLow = rxFrame.data[0];
      byte rpmHigh = rxFrame.data[1];
      rpm = (rpmLow << 8) + rpmHigh;
      //Serial.println(rpm);
    }

    //Goolant CAN Frame
    if (rxFrame.identifier == 0x649) {
      clt = rxFrame.data[0] - 40;
      battery = rxFrame.data[5];
      //Serial.println(battery);
    }

    // //Gear position CAN Frame
    if (rxFrame.identifier == 0x64D) {
      gear = rxFrame.data[6] & 0b00001111;
      //Serial.println(gear);
    }

    // //Pit Switch CAN Frame
    //Neutral Switch Too
    if (rxFrame.identifier == 0x64E) {
      pit = rxFrame.data[3] & 0b01000000;
      neutral = rxFrame.data[3] & 0b00000001;
      //Serial.println(neutral);
    }
  }

  //Neutral
  if (neutral == 1) {
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
  else if (gear == 1) {
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
  else if (gear == 2) {
    digitalWrite(GearA, LOW);
    digitalWrite(GearB, LOW);
    digitalWrite(GearC, HIGH);
    digitalWrite(GearD, LOW);
    digitalWrite(GearE, LOW);
    digitalWrite(GearG, LOW);
    digitalWrite(GearDP, HIGH);

  }
  //Third Gear
  else if (gear == 3) {
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
  else if (gear == 4) {
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
  else if (gear == 5) {
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
  else if (gear == 6) {
    digitalWrite(GearA, LOW);
    digitalWrite(GearB, HIGH);
    digitalWrite(GearC, LOW);
    digitalWrite(GearD, LOW);
    digitalWrite(GearE, LOW);
    digitalWrite(GearF, LOW);
    digitalWrite(GearG, LOW);
    digitalWrite(GearDP, HIGH);
  }

  //If no known gears are seen, display an 8 with the DP
  //Uses millis and stuff to ignore the time period in between gear shifts
  //As the Motec estimates the gear position based on wheel speed, rpm, gear ratio, etc
  //If window of time is exceeded, segment will display the E and DP
  if (gear == 14 && neutral != 1) {

    //Starts timer thing if gear position is 14
    if (gear14Timer == 0) {
      gear14Timer = currentMillis;
    }

    //If timer and currentmillis exceed the timer interval gearIgnoreMillis
    //Display the E and DP
    else if (currentMillis - gear14Timer >= gearIgnoreMillis) {
      digitalWrite(GearA, LOW);
      digitalWrite(GearB, LOW);
      digitalWrite(GearC, LOW);
      digitalWrite(GearD, LOW);
      digitalWrite(GearE, LOW);
      digitalWrite(GearF, LOW);
      digitalWrite(GearG, LOW);
      digitalWrite(GearDP, LOW);
    }
  } else {
    //Resets gear 14 timer when its not in gear 14
    gear14Timer = 0;
  }

  //Coolant lighting Neopixel
  //First three "simple" coolant states
  //Cold, operating temp, hot
  if (clt < coolantCold && clt != -69) {
    pixels.setPixelColor(0, pixels.Color(0, 0, 255));
  } else if (clt >= coolantCold && clt <= coolantHot) {
    pixels.setPixelColor(0, pixels.Color(0, 64, 0));
  } else if (clt > coolantHot && clt < coolantFlash) {
    pixels.setPixelColor(0, pixels.Color(255, 0, 0));
  }

  //Flashing coolant light for when coolant temp is exceeding coolantFlash threshold
  //uses currentMillis, previousMillis, overheatMillis to see how much time has
  //passed since conditions met, turns coolant light on or off based on that interval
  else if (clt > coolantFlash && cltFlashState && (currentMillis - previousMillis >= overheatMillis)) {
    previousMillis = currentMillis;
    cltFlashState = false;
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
  } else if (clt > coolantFlash && !cltFlashState && (currentMillis - previousMillis >= overheatMillis)) {
    previousMillis = currentMillis;
    cltFlashState = true;
    pixels.setPixelColor(0, pixels.Color(255, 0, 0));
  }

  //Button toggle logic and stuff
  bool button1CurrentState = digitalRead(Button1Pin);
  bool button2CurrentState = digitalRead(Button2Pin);

  if (button1CurrentState == LOW && button1LastState == HIGH) {
    button1State = !button1State;
  }

  if (button2CurrentState == LOW && button2LastState == HIGH) {
    button2State = !button2State;
  }

  button1LastState = button1CurrentState;
  button2LastState = button2CurrentState;

  //Shift lights When Pit Switch is Off
  //First Light
  if (pit == 0) {

    //Turn all shift lights off if below threasholds
    //Was having issues with PIT lights sticking when switch was turned off
    if (rpm < shiftRpm1 && rpm < flashingRPM) {
      pixels.setPixelColor(1, pixels.Color(0, 0, 0));
      pixels.setPixelColor(2, pixels.Color(0, 0, 0));
      pixels.setPixelColor(3, pixels.Color(0, 0, 0));
      pixels.setPixelColor(4, pixels.Color(0, 0, 0));
      pixels.setPixelColor(5, pixels.Color(0, 0, 0));
      pixels.setPixelColor(6, pixels.Color(0, 0, 0));
      pixels.setPixelColor(7, pixels.Color(0, 0, 0));
    }

    //First Light
    if (rpm >= shiftRpm1 && rpm < flashingRPM) {
      pixels.setPixelColor(1, pixels.Color(0, 255, 0));
      pixels.setPixelColor(7, pixels.Color(0, 255, 0));
    } else if (rpm <= shiftRpm1) {
      pixels.setPixelColor(1, pixels.Color(0, 0, 0));
      pixels.setPixelColor(7, pixels.Color(0, 0, 0));
    }

    //Second Light
    if (rpm >= shiftRpm3 && rpm < flashingRPM) {
      pixels.setPixelColor(2, pixels.Color(255, 255, 0));
      pixels.setPixelColor(6, pixels.Color(255, 255, 0));
    } else if (rpm <= shiftRpm3) {
      pixels.setPixelColor(2, pixels.Color(0, 0, 0));
      pixels.setPixelColor(6, pixels.Color(0, 0, 0));
    }

    //Third Light
    if (rpm >= shiftRpm5 && rpm < flashingRPM) {
      pixels.setPixelColor(3, pixels.Color(255, 0, 0));
      pixels.setPixelColor(5, pixels.Color(255, 0, 0));
    } else if (rpm <= shiftRpm5) {
      pixels.setPixelColor(3, pixels.Color(0, 0, 0));
      pixels.setPixelColor(5, pixels.Color(0, 0, 0));
    }

    //Fourth Light
    if (rpm >= shiftRpm7 && rpm < flashingRPM) {
      pixels.setPixelColor(4, pixels.Color(255, 0, 0));
    } else if (rpm <= shiftRpm7) {
      pixels.setPixelColor(4, pixels.Color(0, 0, 0));
    }

    //Flashing RPM Light Stuffz
    //Similar to flashing coolant light
    //Uses millis and stuffs to find interval of time to turn on or off the
    //rpm sequence lights
    if (rpm > flashingRPM) {
      if (rpmFlashState && (currentMillis - previousMillis3 >= shiftMillis)) {
        previousMillis3 = currentMillis;
        rpmFlashState = false;
        pixels.setPixelColor(1, pixels.Color(0, 0, 0));
        pixels.setPixelColor(2, pixels.Color(0, 0, 0));
        pixels.setPixelColor(3, pixels.Color(0, 0, 0));
        pixels.setPixelColor(4, pixels.Color(0, 0, 0));
        pixels.setPixelColor(5, pixels.Color(0, 0, 0));
        pixels.setPixelColor(6, pixels.Color(0, 0, 0));
        pixels.setPixelColor(7, pixels.Color(0, 0, 0));
      } else if (!rpmFlashState && (currentMillis - previousMillis3 >= shiftMillis)) {
        previousMillis3 = currentMillis;
        rpmFlashState = true;
        pixels.setPixelColor(1, pixels.Color(255, 0, 255));
        pixels.setPixelColor(2, pixels.Color(255, 0, 255));
        pixels.setPixelColor(3, pixels.Color(255, 0, 255));
        pixels.setPixelColor(4, pixels.Color(255, 0, 255));
        pixels.setPixelColor(5, pixels.Color(255, 0, 255));
        pixels.setPixelColor(6, pixels.Color(255, 0, 255));
        pixels.setPixelColor(7, pixels.Color(255, 0, 255));
        //Porple drank
      }
    }
  }

  //Pit Limiter Lights
  //Alternates between center thingies if pit switch is on
  //Same millis stuff going on up above
  if (pit == 64) {
    if (pitFlashState && (currentMillis - previousMillis2 >= pitMillis)) {
      previousMillis2 = currentMillis;
      pitFlashState = false;
      pixels.setPixelColor(1, pixels.Color(0, 0, 0));
      pixels.setPixelColor(2, pixels.Color(255, 180, 0));
      pixels.setPixelColor(3, pixels.Color(0, 0, 0));
      pixels.setPixelColor(4, pixels.Color(255, 180, 0));
      pixels.setPixelColor(5, pixels.Color(0, 0, 0));
      pixels.setPixelColor(6, pixels.Color(255, 180, 0));
      pixels.setPixelColor(7, pixels.Color(0, 0, 0));
    }
    if (!pitFlashState && (currentMillis - previousMillis2 >= pitMillis)) {
      previousMillis2 = currentMillis;
      pitFlashState = true;
      pixels.setPixelColor(1, pixels.Color(255, 180, 0));
      pixels.setPixelColor(2, pixels.Color(0, 0, 0));
      pixels.setPixelColor(3, pixels.Color(255, 180, 0));
      pixels.setPixelColor(4, pixels.Color(0, 0, 0));
      pixels.setPixelColor(5, pixels.Color(255, 180, 0));
      pixels.setPixelColor(6, pixels.Color(0, 0, 0));
      pixels.setPixelColor(7, pixels.Color(255, 180, 0));
    }
  }

  //Battery Light
  if (battery >= 131) {
    pixels.setPixelColor(8, pixels.Color(0, 255, 0));
  } else if (battery == 130) {
    pixels.setPixelColor(8, pixels.Color(255, 255, 0));
  } else {
    pixels.setPixelColor(8, pixels.Color(255, 0, 0));
  }

  //Set neopixels to set values
  //Delay to yield to other tasks

  //delay(10);
  if (currentStamp - last >= 50) {
    pixels.show();
    last = currentStamp;
  }
}


void sendObdFrame() {
  CanFrame obdFrame = { 0 };
  obdFrame.identifier = 0x7F6;  // Default OBD2 address;
  obdFrame.extd = 0;
  obdFrame.data_length_code = 8;

  if (button1State == 1) {
    obdFrame.data[0] = 4096 >> 8 & 0xFF;
    obdFrame.data[1] = 4096 >> 0 & 0xFF;
  } else {
    obdFrame.data[0] = 0;
    obdFrame.data[1] = 0;
  }

  if (button2State == 1) {
    obdFrame.data[2] = 4096 >> 8 & 0xFF;
    obdFrame.data[3] = 4096 >> 0 & 0xFF;
  } else {
    obdFrame.data[2] = 0;
    obdFrame.data[3] = 0;
  }

  obdFrame.data[4] = 0xAA;  // CAN works better this way as it needs
  obdFrame.data[5] = 0xAA;  // to avoid bit-stuffing
  obdFrame.data[6] = 0xAA;
  obdFrame.data[7] = 0xAA;
  // Accepts both pointers and references
  ESP32Can.writeFrame(obdFrame);  // timeout defaults to 1 ms
}
