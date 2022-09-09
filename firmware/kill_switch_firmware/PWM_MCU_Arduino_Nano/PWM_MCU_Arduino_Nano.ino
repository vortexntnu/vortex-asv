//This firmware is used on an Arduino Nano, to interpret 50Hz RC PWM signals from a FrSky Radio Reciever and relay those to the Kill Switch MCU.

//ATTENTION: Do not upload while sending PWM signals to pins, this causes errors and results in an aborted upload. (This is presumably caused by harware interrupts halting upload while triggered.)


//duty cycle range values, if outside range: do nothing
const int top_lim_DC_HIGH  = 2200;   //gives HIGH output, measured in [us]
const int bttm_lim_DC_HIGH = 1800;   //gives HIGH output, measured in [us]
const int top_lim_DC_LOW  = 1200;    //gives LOW output, measured in [us]
const int bttm_lim_DC_LOW = 800;     //gives LOW output, measured in [us]

//PWM input pins
const byte inpD2 = 2;   //OtA Kill Switch - 5VAC input
const byte inpD3 = 3;   //Arm system - 5VAC input
const byte inpD8 = 8;   //ESC selector for MUX - 5VAC input

//bool output pins
const byte outA0 = 14;    //OtA Kill Switch 1 - 5VDC out
const byte outA1 = 15;    //OtA Kill Switch 2 - 5VDC out
const byte outA2 = 16;    //OtA Kill Switch LED - 5VDC out
const byte outA3 = 17;    //Arm system - 5VDC out
const byte outA4 = A4;    //ESC PWM selector DC

//rising edge - duty cycle - start time variables
unsigned long rise_inpD2;
unsigned long rise_inpD3;
unsigned long rise_inpD8;

void setup() {
  //Serial.begin(9600);
  
  //setup input pins
  pinMode(inpD2, INPUT_PULLUP);
  pinMode(inpD3, INPUT_PULLUP);
  pinMode(inpD8, INPUT_PULLUP);

  //setup output pins
  pinMode(outA0, OUTPUT);
  pinMode(outA1, OUTPUT);
  pinMode(outA2, OUTPUT);
  pinMode(outA3, OUTPUT);
  pinMode(outA4, OUTPUT);
  
  //setup interrupts
    //Harware interrupts
      attachInterrupt(digitalPinToInterrupt(inpD2), interrupt_inpD2, CHANGE);
      attachInterrupt(digitalPinToInterrupt(inpD3), interrupt_inpD3, CHANGE);
    //Software interrupt
      //Enable PCIE0 Bit 0 = 1(Port B)
      PCICR |= B00000001;
      //Select PCINT0 Bit0 = 1(Pin D8)
      PCMSK0 |= B00000001;
}

//required empty function
void loop() {}

void interrupt_inpD2() {
  
  //Rising edge
  if (digitalRead(inpD2) == HIGH){
    rise_inpD2 = micros();
  }
  else{
    //Pin low --> Falling edge
    unsigned long duration = micros() - rise_inpD2;
    if (bttm_lim_DC_HIGH < duration){
      if ( duration < top_lim_DC_HIGH){
        digitalWrite(outA0, HIGH);   // turn the output HIGH
        digitalWrite(outA1, HIGH);   // turn the output HIGH
        digitalWrite(outA2, HIGH);   // turn the output HIGH
        
      }
    }
    else if (bttm_lim_DC_LOW<duration){
      if(duration<top_lim_DC_LOW){
        digitalWrite(outA0, LOW);   // turn the output LOW
        digitalWrite(outA1, LOW);   // turn the output LOW
        digitalWrite(outA2, LOW);   // turn the output LOW
      }
      
    }    
  }
}

void interrupt_inpD3() {

  //Rising edge
  if (digitalRead(inpD3) == HIGH){
    rise_inpD3 = micros();
  }
  else{
    //Pin low --> Falling edge
    unsigned long duration = micros() - rise_inpD3;
    if (bttm_lim_DC_HIGH < duration){
      if ( duration < top_lim_DC_HIGH){
        digitalWrite(outA3, HIGH);   // turn the output HIGH
      }
    }
    else if (bttm_lim_DC_LOW<duration){
      if(duration<top_lim_DC_LOW){
        digitalWrite(outA3, LOW);   // turn the output LOW
      }
    }
  }
}

ISR (PCINT0_vect){
  // Interrupt for Port B

  //Rising edge
  if (digitalRead(inpD8) == HIGH){
    rise_inpD8 = micros();
    
  }
  else{
    //Pin low --> Falling edge
    unsigned long duration = micros() - rise_inpD8;
    if (bttm_lim_DC_HIGH < duration){
      if ( duration < top_lim_DC_HIGH){
        digitalWrite(outA4, HIGH);   // turn the output HIGH
      }
    }
    else if (bttm_lim_DC_LOW<duration){
      
      if(duration<top_lim_DC_LOW){
        
        digitalWrite(outA4, LOW);   // turn the output LOW
      } 
    }   
  }
}
