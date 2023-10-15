/********************************
 *   Capacitive touch sensors   *

Goal of the project: 
Create 4 Capacitor Touch Sensors to use as inputs for our Acrade Rhythem Game

The capacitor touch sensors should be able to detect physical human input which will help add interactiveness to the game.
After detecting this input, it should be able to immediately update the touch sensor state
Using SerialPort Write and read, it should be able to send info to Unity and Unity should be able to read the changes in state in real time
Finally, there should be no delay from touching the touchsensors and updating the info in unity

 ********************************/
#define THRES 150
#define SAMPLES 20
#define LED1 3
#define LED2 4
#define LED3 5
#define LED4 6

typedef struct touchPad{
  int pin;
  int unpValue;
  int value;
  char state=0, prevState=0;
  char toggleState=0;
};

touchPad touchPad1, touchPad2, touchPad3, touchPad4;

void setup() {
  /* Set A0 pull-up resistor, used to charge internal capacitor */
  Serial.begin(9600);
  pinMode(A0, INPUT_PULLUP);
  analogRead(A0);
  
  //Sets up for 4 capacitor Touch Sensors
  pinMode(LED1, OUTPUT);
  digitalWrite(LED1, 0);
  pinMode(LED2, OUTPUT);
  digitalWrite(LED2, 0);
  pinMode(LED3, OUTPUT);
  digitalWrite(LED3, 0);
  pinMode(LED4, OUTPUT);
  digitalWrite(LED4, 0);

  //initialize sensors
  touchPadInit(&touchPad1, A1);
  touchPadInit(&touchPad2, A2);
  touchPadInit(&touchPad3, A3);
  touchPadInit(&touchPad4, A4);
}

void loop() {
  //scan Sensors
  touchPadScan(&touchPad1);
  touchPadScan(&touchPad2);
  touchPadScan(&touchPad3);
  touchPadScan(&touchPad4);

  //Change LED state to ON/OFF LED's Touch Sensor State
  //Visual Test to see if it working
  digitalWrite(LED1, touchPad1.toggleState);
  digitalWrite(LED2, touchPad2.toggleState);
  digitalWrite(LED3, touchPad3.toggleState);
  digitalWrite(LED4, touchPad4.toggleState);


  //For Every Touch Sensor, We will write their state to the serial port 
  //We will use SERIALPORT Read in UNITY to determine whether the TOUCH_SENSORS are activated
  //write 1 to serial port if TOUCH_SENSOR_1 is off
  if (touchPad1.toggleState == 0){
    Serial.write("1");
    Serial.flush();
    delay(20);
  }
  //write 2 to serial port if TOUCH_SENSOR_1 is on
  else if (touchPad1.toggleState == 1){ 
    Serial.write("2");
    Serial.flush();
    delay(20);
  }
  //write one 3 to serial port if TOUCH_SENSOR_2 is off
  if (touchPad2.toggleState == 0){
    Serial.write("3");
    Serial.flush();
    delay(20);
  }
  //write one 4 to serial port if TOUCH_SENSOR_2 is on
  else if (touchPad2.toggleState == 1){ 
    Serial.write("4");
    Serial.flush();
    delay(20);
  }
  //write one 5 to serial port if TOUCH_SENSOR_3 is off
  if (touchPad3.toggleState == 0){
    Serial.write("5");
    Serial.flush();
    delay(20);
  }
  //write one 6 to serial port if TOUCH_SENSOR_3 is on
  else if (touchPad3.toggleState == 1){ 
    Serial.write("6");
    Serial.flush();
    delay(20);
  }
  //write one 7 to serial port if TOUCH_SENSOR_4 is off
  if (touchPad4.toggleState == 0){
    Serial.write("7\n");
    Serial.flush();
    delay(20);
  }
  //write one 8 to serial port if TOUCH_SENSOR_4 is on
  else if (touchPad4.toggleState == 1){ 
    Serial.write("8\n");
    Serial.flush();
    delay(20);
  }
}

//Sets up Touch_Sensors
void touchPadInit(touchPad *pad, int pin){
  pad->pin=pin;
  pad->unpValue = (sampleB(pin) - sampleA(pin));
  DIDR0 |= 1;
  DIDR0 |= 1<<(pin-A0);
}

int sampleA(int sensePin){
  //Sample capacitor is charged to VCC via A0 pull-up resistor, 
  //touch pad is discharged by pulling pin low
  ADMUX = 0b01000000;
  pinMode(sensePin, OUTPUT);
  digitalWrite(sensePin, 0);
  
  pinMode(sensePin, INPUT);
  ADMUX = 0b01000000 | sensePin-A0;

  ADCSRA |= 1<<ADSC;
  while((ADCSRA & (1<<ADSC)) != 0){
    return ADC;
  }
}

int sampleB(int sensePin){
  //Sample capacitor is discharged by selecting GND as ADC input
  //touch pad is charged to VCC via pin pull-up resistor
  ADMUX = 0b01001111;
  pinMode(sensePin, INPUT_PULLUP);
  
  pinMode(sensePin, INPUT);
  ADMUX = 0b01000000 | sensePin-A0;

  ADCSRA |= 1<<ADSC;
  while((ADCSRA & (1<<ADSC)) != 0){
    return ADC;
  } 
}

//Updates Readings from Capacitor Touch Sensors
void touchPadScan(touchPad *pad){
  static float A=0;
  static float B=0;
  // Get readings from sensor and gets average
  for(int i=0; i<SAMPLES; i++){
    A += sampleA(pad->pin);
    B += sampleB(pad->pin);
  }
  A /= SAMPLES;
  B /= SAMPLES;
  pad->value = (B - A);

  //Checks if there is a sudden Change in unpValue
  //A sudden Change implies someone has touched the capacitor touch sensor
  if(pad->value > (pad->unpValue + THRES)){
     pad->toggleState=1;
  }
  //touch pad is not being pressed and should turn off the signal
  else{
    pad->toggleState=0;
    pad->unpValue=((float)pad->unpValue*0.9)+((float)pad->value*0.1);
  }
}