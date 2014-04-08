/*
  RGB LED lamp connected to Teensy 3.0 micro (arduino compatable) controlled via serial commands over bluetooth with custom Tasker app
 This version: Test 11, rewrite using LED library!!! (has has built in fading capabilities)
 Works?: yes, but with a fading bug where half the time it will fade then the lamp will go to red : (
 */

#include <ColorLamp.h>
#include <SerialCommand.h>
#include <Metro.h> //Timer library to run things only at certain intervals

Metro donttouchMetro = Metro(700);
Metro touchbrightMetro = Metro(100);
Metro fadeMetro = Metro(0);

SerialCommand sCmd;     //Instantiate the SerialCommand object
ColorLamp * lamp  =  new ColorLamp( 3, 4, 5 ); //Instantiate the rgb lamp object

#define RedLED 3              // red led connected to PWM pin
#define GreenLED 4           // green led connected to PWM pin
#define BlueLED 5            // blue led connected to PWM pin
#define WHITELED 6            // white led connected to PWM pin
#define touchPin 23          //Pin used for capsense (touch sensor)
#define DEBUG true
#define MAX_FADE_QUEUE 10

int8_t CycleArrayIndex = 6;
uint32_t CycleArray[MAX_FADE_QUEUE][4]={
  {
    0,4096,5,500  }
  ,
  {
    0,4096,20,500  }
  ,
  {
    0,4096,500,500  }
  ,
  {
    0,4096,2000,500  }
  ,
  {
    0,4096,4095,500  }
  ,
  {
    0,4096,0,500  }
};

uint32_t Rainbow_Speed = 15000;

void setup() {
  analogWriteResolution(12); //set PWM to 12 bit resolution  
  Serial1.begin(115200);
  Serial.begin(9600);

  pinMode(WHITELED, OUTPUT);          // set pin as output
  analogWrite(WHITELED, 0);

  /* Setup callbacks for SerialCommand commands
   All Commands have to start with a dash "-" since I was unable to send the one letter "KEY" prefix when using tasker to send an intent to Amarino android app.
   When no "key" prefix is sent, Amarino defaults to sending a - as the key "prefix"
   */
  sCmd.addCommand("-RED", RED_led);          //Directly control brightness LEDs 12bit 0-4096 value
  sCmd.addCommand("-GREEN", GREEN_led);
  sCmd.addCommand("-BLUE", BLUE_led);
  sCmd.addCommand("-WHITE", WHITE_led);
  sCmd.addCommand("-HSBN", Set_Hsbn);        
  sCmd.addCommand("-ALLOFF", LEDS_off);      // Turns all LEDs off
  sCmd.addCommand("-TEST", Test_reply);      // Sends test repy test over BT

  sCmd.addCommand("-RHSV", RainbowHSV);      //Rainbow by cycling through hues, given sat & bri values input
  sCmd.addCommand("-RSpeed", RainbowSpeed);  //Set rainbow speed (total time for full hue revolution) milliseconds
  sCmd.addCommand("-CANDLE", Candle);        //Start candle effect
  sCmd.addCommand("-SENDTEST", SendTest);    //Send string from teensy to Amarino on Android
  sCmd.addCommand("-TOUCHBRIGHT", TouchBright); //Put into touchbright mode. Capsense value determines brightness
  sCmd.addCommand("-DONTTOUCH", DontTouch); //Put into "don't touch me" mode. Send message to tasker to play don't touchme sound if capsense
  sCmd.addCommand("-FADEADD", FadeAdd); // Add [H,S,B,Speed] to fade sequence array
  sCmd.addCommand("-FADECLEAR", FadeClear); //Clear fade sequence array
  sCmd.addCommand("-FADEGO", FadeGo); //Start the fade sequence
  sCmd.addCommand("-FADEDEBUG", FadeDebug); //Start the fade sequence
  sCmd.setDefaultHandler(unrecognized);      // Handler for command that isn't matched  (says "What?")
}

void loop() {
  sCmd.readSerial();
  lamp->update();
}   

void RED_led() {
  int aNumber;
  char *arg;
  arg = sCmd.next();
  if (arg != NULL) {
    aNumber = atoi(arg);
    analogWrite(RedLED, aNumber);
  }
}

void GREEN_led() {
  int aNumber;
  char *arg;
  arg = sCmd.next();
  if (arg != NULL) {
    aNumber = atoi(arg);
    analogWrite(GreenLED, aNumber);
  }
}

void BLUE_led() {
  int aNumber;
  char *arg;
  arg = sCmd.next();
  if (arg != NULL) {
    aNumber = atoi(arg);
    analogWrite(BlueLED, aNumber);
  }
}

void WHITE_led() {
  int aNumber;
  char *arg;
  arg = sCmd.next();
  if (arg != NULL) {
    aNumber = atoi(arg);
    analogWrite(WHITELED, aNumber);
  }
}

void LEDS_off() {
  lamp->intensityTo( 0, 100 );
  analogWrite(WHITELED, 0);  
}

void FadeAdd() {
  uint32_t fade_hue; //0 to 12284
  uint32_t fade_sat; //0 to 4096 (0 to 4095 with small inaccuracy)
  uint32_t fade_bri; //0 to 4095
  uint32_t fade_speed;

  char *arg;
  arg = sCmd.next();
  if (arg != NULL) {
    fade_hue = atoi(arg);
  }

  arg = sCmd.next();
  if (arg != NULL) {
    fade_sat = atoi(arg);
  }

  arg = sCmd.next();
  if (arg != NULL) {
    fade_bri = atoi(arg);
  }

  arg = sCmd.next();
  if (arg != NULL) {
    fade_speed = atoi(arg);
  }

  CycleArray[CycleArrayIndex][0]=fade_hue;
  CycleArray[CycleArrayIndex][1]=fade_sat;
  CycleArray[CycleArrayIndex][2]=fade_bri;
  CycleArray[CycleArrayIndex][3]=fade_speed;
  if(CycleArrayIndex < MAX_FADE_QUEUE){
    CycleArrayIndex++;
  }
}

void FadeGo() {
  uint32_t hue, sat, bri, fade_time;

  if(DEBUG){
    Serial1.println("Fade Go called..");
  }

  for (int i=0; i < CycleArrayIndex;){
    if (fadeMetro.check() == 1) {
      Serial1.print("i=");
      Serial1.println(i);
      hue=CycleArray[i][0];
      sat=CycleArray[i][1];
      bri=CycleArray[i][2];
      fade_time=CycleArray[i][3];
      if(DEBUG){
        Serial1.print("[");
        Serial1.print(hue);
        Serial1.print(",");
        Serial1.print(sat);
        Serial1.print(",");
        Serial1.print(bri);
        Serial1.print(",");
        Serial1.print(fade_time);
        Serial1.println("]");
      }
      fadeMetro.interval(fade_time);
      //delay(fade_time);
      //HSBtoRGB(hue, sat, bri); 
      //RGBanalogWrite();
      i++;    
    }
  }
  fadeMetro.interval(0);
}

void fadeHSBtoHSB(uint32_t fadeFrom[], uint32_t fadeTo[]){
  uint32_t hue_diff, sat_diff, bri_diff;

  boolean hue_increase;
  boolean fade_hue, fade_sat, fade_bri;

  if(fadeFrom[0] == fadeTo[0]){
    hue_increase = true;
  }
  else if(fadeFrom[0] < fadeTo[0]){
    hue_increase = true;
  }
  else if(fadeFrom[0] > fadeTo[0]){
    hue_increase = false;
  }
}


void FadeClear() {
  CycleArrayIndex=0;
}

void FadeDebug() {
  int8_t array_index;

  char *arg;
  arg = sCmd.next();
  if (arg != NULL) {
    array_index = atoi(arg);
  }
  Serial1.print("index=");
  Serial1.println(array_index);
  Serial1.print("[");
  Serial1.print(CycleArray[array_index][0]);
  Serial1.print(",");
  Serial1.print(CycleArray[array_index][1]);
  Serial1.print(",");
  Serial1.print(CycleArray[array_index][2]);
  Serial1.print(",");
  Serial1.print(CycleArray[array_index][3]);
  Serial1.println("]");
}

void Set_Hsbn(){
  int32_t hue; //0 to 12284
  int32_t sat; //0 to 4096 (0 to 4095 with small inaccuracy)
  int32_t bri; //0 to 4095

  char *arg;
  arg = sCmd.next();
  if (arg != NULL) {
    hue = atoi(arg);
  }

  arg = sCmd.next();
  if (arg != NULL) {
    sat = atoi(arg);
  }

  arg = sCmd.next();
  if (arg != NULL) {
    bri = atoi(arg);
  }

  lamp->hsbTo( hue, sat, bri, 500 );

}


void RainbowHSV(){
  int sat;
  int bri;

  if (DEBUG){
    Serial.println("Rainbow started");
  }

  char *arg;
  arg = sCmd.next();
  if (arg != NULL) {
    sat = atoi(arg);
    if (DEBUG){
      Serial.print("sat=");
      Serial.println(sat);
    }
  }

  arg = sCmd.next();
  if (arg != NULL) {
    bri = atoi(arg);
    if (DEBUG){
      Serial.print("bri=");
      Serial.println(bri);
    }
  }

  lamp->setAnimationType(SINUS, true , false);

  while(!Serial1.available()){
    int i;
    if( !lamp->isAnimating()){
      i++;
      if(i==1){
        lamp->hsbTo( 0, 4094, 4094, Rainbow_Speed, false );
      }
      if(1==2){
        lamp->hsbTo( 4095, 4094, 4094, Rainbow_Speed, false );
      }
      if(i>1){
        i=0;
      }
    }
  }
}

void Candle(){
  Serial.println("Candle started");
  int32_t hue; //0 to 12284
  int32_t sat; //0 to 4096 (0 to 4095 with small inaccuracy)
  int32_t bri; //0 to 4095
  // the start of the flicker (low)
  const int flicker_low_min = 1400;
  const int flicker_low_max = 1800;

  // the end value of the flicker (high)
  const int flicker_high_min = 2200;
  const int flicker_high_max = 4000;

  // delay between each low-high-low cycle
  // low->high |flicker_hold| high->low
  const int flicker_hold_min = 40; // milliseconds
  const int flicker_hold_max = 80; // milliseconds

  // delay after each low-high-low cycle
  // low->high->low |flicker_pause| low->high...
  const int flicker_pause_min = 100; // milliseconds
  const int flicker_pause_max = 200;  // milliseconds

  // delay low to high and high to low cycle
  const int flicker_speed_min = 100; // microseconds
  const int flicker_speed_max = 800; // microseconds

  int flicker_random_low_start = 0;
  int flicker_random_low_end = 0;
  int flicker_random_high = 0;
  int flicker_random_speed_start = 0;
  int flicker_random_speed_end = 0;

  char *arg;
  arg = sCmd.next();
  if (arg != NULL) {
    hue = atoi(arg);
    Serial.print("hue=");
    Serial.println(hue);
  }

  arg = sCmd.next();
  if (arg != NULL) {
    sat = atoi(arg);
    Serial.print("sat=");
    Serial.println(sat);
  }
  while(!Serial1.available()){
    // random time for low
    flicker_random_low_start = random(flicker_low_min, flicker_low_max);
    flicker_random_low_end = random(flicker_low_min, flicker_low_max);

    // random time for high
    flicker_random_high = random(flicker_high_min, flicker_high_max);

    // random time for speed
    flicker_random_speed_start = random(flicker_speed_min, flicker_speed_max);
    flicker_random_speed_end = random(flicker_speed_min, flicker_speed_max);

    // low -> high
    for (int i = flicker_random_low_start; i<flicker_random_high; i++) {
      //analogWrite(FLICKER_LED_PIN, i);
      //HSBtoRGB(hue, sat, i);
      lamp->setHSB(hue, sat, i, true);
      //RGBanalogWrite();
      delayMicroseconds(flicker_random_speed_start);
    }

    // hold
    delay(random(flicker_hold_min, flicker_hold_max));

    // high -> low
    for (int i = flicker_random_high; i>=flicker_random_low_end; i--) {
      //analogWrite(FLICKER_LED_PIN, i);
      //HSBtoRGB(hue, sat, i);
      lamp->setHSB(hue, sat, i, true);
      //RGBanalogWrite();
      delayMicroseconds(flicker_random_speed_end);
    }

    // pause
    delay(random(flicker_pause_min, flicker_pause_max));
  }
}

void RainbowSpeed() {
  int aNumber;
  char *arg;
  arg = sCmd.next();
  if (arg != NULL) {
    aNumber = atoi(arg);
    Rainbow_Speed = aNumber;
  }
}


void Test_reply() {
  Serial1.println("Testing, testing 1, 2, 3");
  Serial.println("Testing, testing 1, 2, 3");
}

void SendTest() {
  char startFlag = 18;
  char ack = 19;
  char delimiter = 59; //';'

  Serial1.print(startFlag);
  Serial1.print("TestString");
  Serial1.print(ack);
}
void DontTouch() {
  while(!Serial1.available()){
    if (donttouchMetro.check() == 1) {
      int touchReading = 0;
      touchReading = touchRead(touchPin);
      if (touchReading > 2500) {
        char startFlag = 18;
        char ack = 19;
        char delimiter = 59; //';'

        Serial1.print(startFlag);
        Serial1.print("CapsenseTrue");
        Serial1.print(ack);
      }
    }
  }
}

void TouchBright() {
  while(!Serial1.available()){
    if (touchbrightMetro.check() == 1) {
      int touchReading = 0;
      touchReading = touchRead(touchPin);
      analogWrite(WHITELED, touchReading);
    }  
  }
}

void unrecognized(const char *command) {
  Serial1.println("What?");    
}
