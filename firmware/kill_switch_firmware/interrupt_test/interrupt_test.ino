
//Duty Cycle range values, if outside range: do nothing
const int top_lim_DC_HIGH  = 2200;   //gives HIGH output, measured in [us]
const int bttm_lim_DC_HIGH = 1800;   //gives HIGH output, measured in [us]
const int top_lim_DC_LOW  = 1200;    //gives LOW output, measured in [us]
const int bttm_lim_DC_LOW = 800;     //gives LOW output, measured in [us]

//PWM inp pins
const byte interrupt_pin = 2;

//Bool output pins
const byte w_pin = A0;
const byte led_pin = 13;

//rising Duty cycle counting variables
unsigned long risingPin2;
unsigned long fallingPin2;
unsigned long maxP2 = 0;

void setup() {
  Serial.begin(9600);
  pinMode(led_pin, OUTPUT);
  pinMode(interrupt_pin, INPUT_PULLUP);
  pinMode(w_pin, OUTPUT);
  pinMode(13, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(interrupt_pin), p2, CHANGE);
  
}

void loop() {
  //digitalWrite(w_pin, HIGH);   // turn the output on        
}

void p2() {
  Serial.print("up: ");
  //Rising edge
  if (digitalRead(interrupt_pin) == HIGH){
    risingPin2 = micros();
    //Serial.print("up: ");
    //Serial.println(risingPin2);
  }
  else{
    //Pin low --> Falling edge
    unsigned long duration = micros() - risingPin2;
    if (bttm_lim_DC_HIGH < duration){
      if ( duration < top_lim_DC_HIGH){
        digitalWrite(w_pin, HIGH);   // turn the output on        
      }
      else{
        digitalWrite(led_pin, HIGH);    // turn internal LED on to signify error
        Serial.println(duration);
      }
    }
    else if (bttm_lim_DC_LOW<duration){
      if(duration<top_lim_DC_LOW){
        digitalWrite(w_pin, LOW);    // turn the output off          
      }
      else{
        digitalWrite(led_pin, HIGH);    // turn internal LED on to signify error
        Serial.println(duration);
      }
    }
    else{
      digitalWrite(led_pin, HIGH);    // turn internal LED on to signify error
      Serial.println(duration);
    }
    
  }
}
