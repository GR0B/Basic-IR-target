/*
Copyright (c) 2015 Bearocratic Designs (Robert Sturzbecher)

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.


[Basic target] 
Designed to be low cost, low powered, small, as minisistic as possible and as easy to assemble as possible.  

To be sold as basic kits for anyone to assemble. 
Cheap so someone can wear a heap of them or make a target wall from them. 
Should work with a large number of systems MilesTag(V1 & v2)/Fragtag and any other data packet > 14bits ontop of the Sony IR protocol/ 

[Inputs]
-TSOP 3pin 56Mhz 

[Outputs]  
-LED  (optional)
-viration motor (30mA@3.3v)(optional)
-beeper (optional)

[Power]
-3v-5v, 
  Peak power 60mA with motor 
  Idle 1.4mA
  Sleep 0.4mA (Goes to sleep after idle for 10seconds, wakes on IR data) 
  Current testing shows battery life for over a week using 3xLR44 watch batteries.

[MPU] 
-Attiny85 internal clock @1Mhz (does not need to be any faster)

                      RST-[1*    8]-VCC
     LED               D3-[2     7]-D2 INT0, SCK,           i2c-SCL/Speaker
     Motor             D4-[3     6]-D1(PWM), MISO,          IR  
                      GND-[4     5]-D0(PWM), MOSI,          i2c-SDA
                       
Future plan is to include i2c support using this http://playground.arduino.cc/Code/USIi2c or software serial. 

[Programming note]
-To program the attiny85 onboard using the ISP connector the Speaker needs to be disconnected.
-To run code on a standard Atmega/Arduino you need to remove the sleep code.

*/


//includes
#include <avr/power.h>                      //needed for disabling/enabling chip parts during sleep/wakeup 
#include <avr/sleep.h>
//#include <avr/wdt.h>                      //watch dog timer, makes it easier to wake processor after time, not used currently
//#include <avr/interrupt.h>

//Pins
uint8_t LED_pin     = 3;
uint8_t SPK_pin     = 2;
uint8_t Motor_pin   = 4;
uint8_t IR_pin      = 1; 
unsigned long sleeptimer;
uint16_t beeptime   = 250;                  //how long to beep for after a hit
uint16_t feeltime   = 2000;                 //how long for the motor and led to stay on after a hit
uint16_t sleeptime  = 10000;                //how long to stay awake for before going to sleep



void setup() {
//        Serial.begin(9600);                  //only works on Arduino boards
        pinMode(LED_pin, OUTPUT);
        pinMode(SPK_pin, OUTPUT);
        pinMode(Motor_pin, OUTPUT);
        pinMode(IR_pin, INPUT);
        ADCSRA &= ~(1<<ADEN);                  //Disable ADC, saves ~230uA on attiny
        Hit();                                 //use this as a boot seq for now, shows that Motor/LED/buzzer works
        sleeptimer = millis();                 //reset the sleeptimer
}


void loop() {
        IR_receive();                          //check for IR signal
        if(millis() - sleeptimer > beeptime){  
            analogWrite(SPK_pin, 0);           //stop the buzzer
        }
        if(millis() - sleeptimer > feeltime){  //stop the LED/Motor stays on longer then buzzer
             digitalWrite(LED_pin,LOW);        //LED off
             digitalWrite(Motor_pin,LOW);      //Motor off
        }
        if(millis() - sleeptimer > sleeptime){  
            sleep();                           //sleep time to save battery
        }
}

void Hit() {//This is what we do when we detect a hit
        digitalWrite(LED_pin,HIGH);
        digitalWrite(Motor_pin,HIGH);
        analogWrite(SPK_pin, 255);             //using PWM to generate our tone
        sleeptimer = millis();                 //reset the sleeptimer
//        Serial.println(F("HIT!"));
}


void sleep() {
        GIMSK |= _BV(PCIE);             // Enable Pin Change Interrupts
        PCMSK |= _BV(PCINT1);           // Use PB1 as interrupt pin  (IR)
        ADCSRA &= ~_BV(ADEN);           // ADC off, should already be off anyway
        MCUCR |= _BV(SM1); MCUCR &= ~_BV(SM0); // Select "Power-down" sleep mode
        sleep_enable();                 // Sets the Sleep Enable bit in the MCUCR Register (SE BIT)
        sei();                          // Enable interrupts
         sleep_cpu();                   // SLEEP
                                        //CPU will wake up here 
        cli();                          // Disable interrupts
        PCMSK &= ~_BV(PCINT1);          // Turn off PB1 as interrupt pin (IR)
        sleep_disable();                // Clear SE bit
        //ADCSRA |= _BV(ADEN);          // Turn ADC back on, But we don't need it so leave it off to save power
        sei();                          // Enable interrupts, 
        sleeptimer = millis();          // Reset the sleeptimer, we should be comming out of sleep now 
} 


ISR(PCINT0_vect) {
        // This is called when the interrupt occurs, but I don't need to do anything just return to loop.
}



void IR_receive() {//we only need to check if we get more then 14 bits
        uint16_t IR_MAXPULSE                = 2600;                 // Sony (Miles) header is 2400us, 1= 1200us, 0= 600us  
        uint16_t IR_MAXPULSEGAP             = 750;                  // this is the timeout for the gaps between the bits and at the end, needs to be 600+tolerance  
        uint8_t  IR_RESOLUTION              = 32;                   // needs to be a multiple of 4 if 16mhz. 8 for 8Mhz, 32 for 1Mhz  
        uint8_t  IR_received_len            = 0;
        uint16_t highpulse, lowpulse;                               // temporary storage timing
        highpulse = lowpulse                = 0;                    // start out with no pulse length
        uint8_t currentpulse                = 0;                    // index for pulses we're storing   
        uint8_t timedout                    = 0; 
        uint16_t pulsetimeout               = IR_MAXPULSE / IR_RESOLUTION;
        uint16_t gaptimeout                 = IR_MAXPULSEGAP / IR_RESOLUTION;
        
        if(digitalRead(IR_pin) == LOW){                                          // If the receive pin is low a signal is being received.
            while (timedout == 0 ){
                 highpulse = lowpulse = 0;                                       // reset pulse timers 
                 //while (PIND & (1 << IRRX_pin)) {                              // pin is still HIGH, this is faster but less compatible
                 while (digitalRead(IR_pin) == HIGH) {                           // digitalRead is slower but more compatible if porting to attiny             
                       highpulse++;
                       delayMicroseconds(IR_RESOLUTION);
                       if (highpulse >= gaptimeout) {                            // If the pulse is too long, we 'timed out' - either nothing
                           timedout = 1;                                         // timedout so exit
                           //Serial.println(F(" Pulse timeout"));
                           //Serial.print(" Pulses: ");
                           //Serial.println(currentpulse);        
                           IR_received_len = currentpulse;  
                           if (currentpulse > 12){                               // Only process if enough data, should be header + 16 data bits for Milestag1 + parity, and about 14+ for MilesTag2 
                               Hit();                                            // call our hit function (Lights and buzzer)  
                           }
                           return;                                               // we've grabbed so far, and then reset
                       }
                }
                while (digitalRead(IR_pin) == LOW) {                             // pin is still LOW
                       lowpulse++;
                       delayMicroseconds(IR_RESOLUTION);
                       if (lowpulse >= pulsetimeout) {                           // If we get carrier that is too long to even be a header
                           //Serial.println(F("Pulse timeout. Signal too long, someone jamming/spamming us?"));
                           return;
                       }
                }
                currentpulse++;   
            }
        }
}






