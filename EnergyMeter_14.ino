#define VER "14"
#define PROGRAM_NAME "Energy Meter"

#define LINE_FREQUENCY 60
#define ADC_COUNTS_TO_VOLTAGE 0.835
  // this factor is (Rin/Rout)*(Vref/COUNTmax)
  // 132K/510 * 3.3/1023 = 0.835
#define ADC_COUNTS_TO_CURRENT 0.0644
  // 2000/1 Current Transformer ratio
  // this factor is (Ratio/Rb)*(Vref/COUNTmax)
  // 2000/100 * 3.3/1023 = 0.0644
//#define NUMBER_OF_SAMPLES 50
#define NUMBER_OF_SAMPLES 65
// number of samples cannot be more than 65 because this would overflow the calcuations for ADC0
//  which uses 16 bit variables.  ADC readings are 10 bit and they are averaged to get the count
//  that is the DC level of the signal  (trying to use minimal memory)
#define SYSTEM_CLOCK 16000000

// timer for sampling
//#define TimerPreloadValue 60203;                   // Use this number for 50 samples 2^16-60203=5333 
//#define TimerPreloadValue 61980;                   // Use this number for 75 samples 2^16-61980=3556 
uint16_t TimerPreloadValue = pow(2,16) - SYSTEM_CLOCK / NUMBER_OF_SAMPLES / LINE_FREQUENCY ;
  // need to sample over exactly a full cycle (1/LINE_FREQUENCY), 16666μSec
  // Timer1 is a 16 bit counter. it will overflow and interupt when the count reaches 65536 (2^16)
  //
  // READINGS_PER_SAMPLE 2 //one volt, one current
  // COUNTS_BETWEEN_IRQ = SYSTEM_CLOCK / ( (NUMBER_OF_SAMPLES * LINE_FREQUENCY)
  //  50 samples: 16,000,000 / (50*60) = 5333
  //  75 samples: 16,000,000 / (75*60) = 3556
  // COUNTS_OF_PROCESSING_TIME = 69 ?
  // TimerPreloadValue = 2^16 - (COUNTS_BETWEEN_IRQ - COUNTS_OF_PROCESSING_TIME)
  // 
  // ADC_CLOCK_CYCLES_PER_READING 13
  // ADC_CLOCK_FREQUENCY 125000 //Hz
  // READING_TIME = ADC_CLOCK_CYCLES_PER_READING / ADC_CLOCK_FREQUENCY = 104μSec
  // SAMPLE_TIME = READINGS_PER_SAMPLE * READING_TIME = 208μSec
  // SAMPLE_TIME has to be complete in under 1/(NUMBER_OF_SAMPLES * LINE_FREQUENCY)) 333μSec
 

/*
  ver 14
    13 working well
    Add in calculations for voltage and current then send to MySensors gateway
    What to do about total kWh?
  ver 13
    12 is working as expected and hoped
    Calculate TimerPreloadValue from system clock, line frequency and number of samples
    Some experimentation indicates that 65 samples per cycle (60Hz) 
      will be the most samples per cycle that can be take
    This version is adding more documentation
  ver 12
    removing print statements and sending power to MQTT broker via the gateway
  ver 11
    This appears not to crash
    Again, with timer registers being backed up/restored integrating MySensors
      another possibility could be that MySensors uses dynamic memory allocation 
        but there's no memory left to allocate
  ver 10
    added sampling of current on A1
    changed number of samples to 50 with readings for Volts and Current
    removed calculation for ADC0 from setup
    remove wait for zero crossing from loop
    Backup timer registers before sampling and restore after
  ver 09
    starting at ver 06
      skipped 7 & 8 because crashed when MySensors was added.
    added a digital output, pin 6, to be able to flag both sampling and total sample time
    moved the ISR to before sample one sycle
    Saved and restored timer registers
  ver 06
    don't know what I did here
  ver 05
    calculating ADC0 by taken the average of a readings from one cycle
  ver 04
    using timer to trigger when to take reading
    determined TimerPreloadValue by adjusting "TimerPreloadValue" until it was 60Hz for 100 samples
      This makes the assumption that the 16MHz clock is accurate.
  ver 03
    not getting rock solid voltage reading
    only getting 75 readings, why not time readings and calculate after?
    is waiting for zero crossing necessary?
    
  ver 02
    determin ADC count that is the zero crossing point
      average of max and min
    wait for zero crossing routine
    Calculate RMS voltage
    Determine the number of acquistions that can be done in 1/(60Hz) seconds
      166 acquistions can be done in 37.19mSec
      74 acquistions
  ver 01
    Demostrated basic read ADC
    cannot read and store one cycle because of insufficient RAM
*/

//--------------------------------------------------MySensors parameters
#define DEVELOPMENT // MySensors Gateway on channel 86 otherwise 121
#define MY_DEBUG

#include <EEPROM.h>
unsigned short MY_NODE_ID = EEPROM.read(0);  // get NODE ID

// radio type, radio control pins and channel 
#define MY_RADIO_RF24
#define MY_RF24_CS_PIN 9
#define MY_RF24_CE_PIN 10
#ifdef DEVELOPMENT
#define MY_RF24_CHANNEL 86
#else
#define MY_RF24_CHANNEL 121
#endif
#define MY_RF24_PA_LEVEL (RF24_PA_MAX)

#include <MySensors.h>
// these wait periods seem to be necessary 
#define MESSAGEWAIT 200              // ms to wait after send message
#define WAIT_AFTER_PRESENTATION 2000 // ms to wait after presentation message

// configure communication protocol to MQTT broker
#define CHILD_ID_POWER 0   // Id of the sensor child
MyMessage msgPOWER(CHILD_ID_POWER, V_WATT);

//------------------------------------------------------before (starting MySensors)
void before(){ // this happens before MySensors starts
  analogReference(EXTERNAL);  // this needs to be done very near the start so ADC doesn't get buggered
  // usual program information at very start
  Serial.begin(115200);
  Serial.print(PROGRAM_NAME);Serial.print(" version ");Serial.println(VER);
}
//-------------------------------------------------------presentation
void presentation(){
  
	sendSketchInfo(PROGRAM_NAME, VER);

	present(CHILD_ID_POWER, S_POWER);
  wait(WAIT_AFTER_PRESENTATION);
}

//--------------------------------------------------Energy Meter parameters
#define TRIGGER_START_SAMPLE_PERIOD_PIN 5
#define TRIGGER_START_SAMPLE_PIN 6
#define VOLTS_IN_PIN A0
#define CURRENT_IN_PIN A1

volatile uint8_t sample = 0;  // must be volatile to allow the wait outside the IRQ to see this value
int16_t samplesVolts[NUMBER_OF_SAMPLES];
int16_t samplesCurrent[NUMBER_OF_SAMPLES];

// used to measure the time it took to make all the samples.
//   An oscilloscope was used to measure this time.
//   This method included a lot of overhead time
unsigned long samplingStart;
unsigned long samplingEnd;

//------------------------------------------------------ISR
ISR(TIMER1_OVF_vect){                    // interrupt service routine for overflow
  TCNT1 = TimerPreloadValue;  // must be first line!  starts the timer counting again
  digitalWrite(TRIGGER_START_SAMPLE_PIN,HIGH);  // this is used to measure the time with an oscilloscope
                                                //   while no longer needed, it probably needs to remain so as to not mess up the timing
                                                //   on second thought, I don't think so
  samplesVolts[--sample]=analogRead(VOLTS_IN_PIN); // decrement before capturing because arrays start at zero and sample starts at number of samples
  samplesCurrent[sample]=analogRead(CURRENT_IN_PIN);
  digitalWrite(TRIGGER_START_SAMPLE_PIN,LOW);  // again, used with oscilloscope
  if (sample) return;  // count down to zero.  This is a trick because when the count decrements to zero, sampling is done
                       // because it is "volatile" variable sample is in RAM and not cache
                       // This makes the code fast

  // terminate sampling by turnning off interupt
  digitalWrite(TRIGGER_START_SAMPLE_PERIOD_PIN,LOW); // indicate that sampling is complete
                                                     // no longer necessary
  samplingEnd = micros();  // see above, this is probably no longer necessary
  TCCR1B &= 248; // turns off timer 
}

//------------------------------------------------------sampleOneCycle
void sampleOneCycle(){
  // back up timer registers
  uint8_t TCNT1_b  = TCNT1;
  uint8_t TCCR1B_b = TCCR1B;
  uint8_t TCCR1A_b = TCCR1A;
  uint8_t TIMSK1_b = TIMSK1;
  

  // configure timer which starts the sampling
  noInterrupts();                       // disable all interrupts
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = TimerPreloadValue;                        // preload timer
    //TCCR1B |= (1 << CS10)|(1 << CS12);    // 1024 prescaler 
    TCCR1B &= 248; // turns off timer?
    TIMSK1 |= (1 << TOIE1);               // enable timer overflow interrupt ISR

    // demark sampling
    sample = NUMBER_OF_SAMPLES; // count down to zero (8 bit number)
    digitalWrite(TRIGGER_START_SAMPLE_PERIOD_PIN,HIGH);  // used for oscilloscope, no longer necessary
    samplingStart = micros();  // used to measure time of taking all samples, no longer necessary

    TCNT1 = 65535; // This number, 2^16-1 will cause the trigger to happen asap!
    TCCR1B |= 1; // turns on timer 
  interrupts();                         // enable all interrupts

  // wait for sampling to be complete
  while(sample); // This should be a very fast wait loop which will allow the IRQ to respond quicker
  samplingEnd = micros();
  
  // restore timer registers
  TCNT1  = TCNT1_b;
  TCCR1B = TCCR1B_b;
  TCCR1A = TCCR1A_b;
  TIMSK1 = TIMSK1_b;
}
 
//------------------------------------------------------setup
void setup() {
  #ifdef DEVELOPMENT
  MY_SERIALDEVICE.println("---------------------------------- DEVELOPMENT ----------------------------------");
  #endif

  //pinMode(LED_BUILTIN, OUTPUT);
  
  // configure oscilloscope pins, no longer needed
  pinMode(TRIGGER_START_SAMPLE_PERIOD_PIN,OUTPUT);
  digitalWrite(TRIGGER_START_SAMPLE_PERIOD_PIN,LOW);
  pinMode(TRIGGER_START_SAMPLE_PIN,OUTPUT);
  digitalWrite(TRIGGER_START_SAMPLE_PIN,LOW);
  
}

// These variables are used to average the results of many samples taken during the AVERAGING_PERIOD
// They are global because they need to be initially initiallized
//   then subsequently get reset when data is sent
#define AVERAGING_PERIOD 5000 // milliseconds
unsigned long averagingStart = millis();
float VrmsADC = 0.0;
float IrmsADC = 0.0;
float averageWattCycle = 0.0;
uint32_t averageSamplingPeriod = 0;
uint16_t averagingCount = 0;
//------------------------------------------------------loop
void loop() {

  sampleOneCycle();

  // immediated ADC0 calculations
  //   determine ADC count that is the DC level of the signal
  uint16_t  ADC0Volts = 0;  // the ADC value that represents the DC level of the signal volts. 
  uint16_t  ADC0Current = 0;  // the ADC value that represents the DC level of the signal amps. 
  for (uint8_t i=0;i<NUMBER_OF_SAMPLES;i++){
    ADC0Volts += samplesVolts[i];
    ADC0Current += samplesCurrent[i];
  }
  ADC0Volts /= NUMBER_OF_SAMPLES;
  ADC0Current /= NUMBER_OF_SAMPLES;
  
  // calculate rms of this cycle.  This is in ADC counts and will be later converted to Volts and Amps
  float WattCycle = 0.0;
  uint32_t sumOfSquaresVolts = 0;
  uint32_t sumOfSquaresCurrent = 0;
  for (uint8_t i=0;i<NUMBER_OF_SAMPLES;i++){
    sumOfSquaresVolts += sq(samplesVolts[i]-ADC0Volts);
    sumOfSquaresCurrent += sq(samplesCurrent[i]-ADC0Current);
    WattCycle += float(int(samplesVolts[i])-int(ADC0Volts)) * float(int(samplesCurrent[i])-int(ADC0Current));
  }
    
  // add this cycle to the sum of for the averaging period  
  averageSamplingPeriod += (samplingEnd - samplingStart);
  VrmsADC += sqrt(float(sumOfSquaresVolts)/float(NUMBER_OF_SAMPLES));  // final part of RMS value
  IrmsADC += sqrt(float(sumOfSquaresCurrent)/float(NUMBER_OF_SAMPLES));  // final part of RMS value
  averageWattCycle += WattCycle;
  averagingCount++;  //increment the number of Vrms'
  
  if (millis()-averagingStart > AVERAGING_PERIOD){
    averagingStart = millis();  // reset the timer that will indicate when data will be sent
    
    // Calculate Watts, RMS voltage, RMS current
    float Watts = ADC_COUNTS_TO_VOLTAGE*ADC_COUNTS_TO_CURRENT/float(NUMBER_OF_SAMPLES) * averageWattCycle/float(averagingCount);
  
    send(msgPOWER.set(Watts,2));
    wait(MESSAGEWAIT);

    /*
    Serial.print(ADC0Volts);
    Serial.print(F("/"));
    Serial.print(ADC0Current);
    Serial.print(F(" "));
    Serial.print(averageSamplingPeriod/averagingCount);
    Serial.print(F("uSec "));
    Serial.print(VrmsADC*ADC_COUNTS_TO_VOLTAGE/float(averagingCount));
    Serial.print(F("V "));
    Serial.print(IrmsADC*ADC_COUNTS_TO_CURRENT/float(averagingCount));
    Serial.print(F("A "));
    Serial.print(averageWattCycle*ADC_COUNTS_TO_VOLTAGE*ADC_COUNTS_TO_CURRENT*60.0/float(averagingCount)/float(NUMBER_OF_SAMPLES)/float(LINE_FREQUENCY));
    Serial.println(F("W"));
    // in "));
    // Serial.print(timerEnd-timerStart);
    // Serial.println(F("uSec"));
    */

    // reset counters
    VrmsADC = 0.0;
    IrmsADC = 0.0;
    averageWattCycle = 0.0;
    averageSamplingPeriod = 0;
    averagingCount = 0;

  }

}
