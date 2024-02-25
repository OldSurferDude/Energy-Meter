#define VER "22"
#define PROGRAM_NAME "Energy Meter"

// IMPORTANT: disconnect the +5 from Arduino before plugging in USB cable!


#define LINE_FREQUENCY 60
#define VOLTS_per_COUNT 0.835
  // this factor is (Rin/Rout)*(Vref/COUNTmax)
  // 132K/510 * 3.3/1023 = 0.835 volt/count
#define AMPS_per_COUNT 0.0644
  // 2000/1 Current Transformer ratio
  // this factor is (Ratio/Rb)*(Vref/COUNTmax)  = volts/count
  // 2000/100 * 3.3/1023 = 0.0644 = A/count
//#define SAMPLES_per_CYCLES 50
#define SAMPLES_per_CYCLES 65
// number of samples cannot be more than 65 because this would overflow the calcuations for ADC0
//  which uses 16 bit variables.  ADC readings are 10 bit and they are averaged to get the count
//  that is the DC level of the signal  (trying to use minimal memory)
#define SYSTEM_CLOCK 16000000

// timer for sampling
//#define TimerPreloadValue 60203;                   // Use this number for 50 samples 2^16-60203=5333 
//#define TimerPreloadValue 61980;                   // Use this number for 75 samples 2^16-61980=3556 
uint16_t TimerPreloadValue = pow(2,16) - SYSTEM_CLOCK / SAMPLES_per_CYCLES / LINE_FREQUENCY ;
/*
  // need to sample over exactly a full cycle (1/LINE_FREQUENCY), 16666μSec
  // Timer1 is a 16 bit counter. it will overflow and interupt when the count reaches 65536 (2^16)
  //
  // READINGS_PER_SAMPLE 2 //one volt, one current
  // COUNTS_BETWEEN_IRQ = SYSTEM_CLOCK / ( (SAMPLES_per_CYCLES * LINE_FREQUENCY)
  //  50 samples: 16,000,000 / (50*60) = 5333
  //  75 samples: 16,000,000 / (75*60) = 3556
  // COUNTS_OF_PROCESSING_TIME = 69 ?
  // TimerPreloadValue = 2^16 - (COUNTS_BETWEEN_IRQ - COUNTS_OF_PROCESSING_TIME)
  // 
  // ADC_CLOCK_CYCLES_PER_READING 13
  // ADC_CLOCK_FREQUENCY 125000 //Hz
  // READING_TIME = ADC_CLOCK_CYCLES_PER_READING / ADC_CLOCK_FREQUENCY = 104μSec
  // SAMPLE_TIME = READINGS_PER_SAMPLE * READING_TIME = 208μSec
  // SAMPLE_TIME has to be complete in under 1/(SAMPLES_per_CYCLES * LINE_FREQUENCY)) 333μSec
*/

/* Energy calculation
The Voltage and Current measurements are assumed to be constant for the time between sampling occurs
Thus, voltage * current is the same as energy.  
The time period for this energy is 1/LINE_FREQUENCY divided by SAMPLES_per_CYCLES 
  or, assuming 60Hz and 65 samples per cycle, ~256μSeconds
If the Voltage is 260Vrms and the current is 20Arms, the energy for one sample is
  260 * 20 * 0.000000256 = 1.333 Watt-seconds
The energy for 1 hour would be 260*20= 5200 W-hour or 5.2kWh
  divided by 1000 Watts per kiloWats and divided by 3600 seconds per hour converts the number kWh
   = 0.000000370 kWh is transferred in 1/65th of 1/60th of a second or 256μSec
   or 0.00002407 kWh in 1/60th of a second
   or 0.00144444 kWh in 1 second
   or 5.200 kWh in 1 hr
   
How many counts would be 1Whr (not kWhr) in the averaging period of 10 seconds?  this is the accuracy desired.
There are 10*60*65 samples in 10 seconds (39000)
260Vrms means a peak voltage of 368V,  divided by 0.835 yields 440 in units of counts (uoC).  
  Around an average count 512, min would 72 and max would be 952
20Arms means a peak current of 28A, divided by  0.0644 yields 439 uoC
  73 min and 941 max
440*439 = 193160 uoC^2.   multiply by factors 0.835 (volts per uoC) * 0.0644 (Amps per uoC) = 10387 W peak
  divide 10387 by 2 (square root of 2 times the square root of 2) = 5193 Watts
Multipyling 193160 uoC^2 by 1/5200 yeilds 37 uoC^2  which would reporesent 1 Watt
  using 4 fewer bits in the counts2_cumulator_in_AveragingPeriod would still achieve the desired accuracy

counts2_cumulator_in_AveragingPeriod could be negative, which means a sign bit is required
  The sign of which would indicate the direction of energy flow.  
The maximum counts2_cumulator_in_AveragingPeriod in one cycle would be +/-6283895 which can be represented by 24 bits.
  (sum of the maximum value of each sample (volts times Amps) in the cycle)
The number of sampling cycles in 10 seconds is 600, 600 requires 10 bits.
  Multiply counts2_cumulator_in_AveragingPeriod by 600 would then require 34 bits.
if the two least significant bit were not used, then the counts2_cumulator_in_AveragingPeriod, can be a signed 4 byte number 
  divide counts2_cumulator_in_cycle by 4 before adding to counts2_cumulator_in_AveragingPeriod
    (not shift, because the sign bit is not shifted)
   As a reminder, only if the averaging cycle is 10 seconds or less.
   
When determining the total Watts in an Averging Cycle, this factor of 4 must be multiplied in

*/

/*
  ver 21
    20 doesn't let the cumulative kWh get sent
    added sending fractional kWh
  ver 20
    19 is not getted expected numbers  Volts. Amps and VA look right
      but Watts, kWh and PF are FUBAR
  ver 19
    Looks like the Watt calculation is wrong as it is more than the VA
    Also, changing the precision of the Watt value being sent to 0 fractional part
    There seems to be a problem with the description of energy in the presentation routine,
      it is not getting to Home Assistant. 
      changing name to Cumulative_kWh
      oops, this was a bad line of code.  I'm surprised it compiled.
    add a request for an echo for each send to the MySensors Gateway
      Then wait some amount of time or until echo comes
      This seems to speed things up instead of waiting for an arbitrary amount of time.
  ver 18
    Home Assistant ignores energy readings less than 100W
     This program sends readings below that
    This version will sum kWh until it reaches 1, then increments a 4 byte unsigned integer
      and then removes the integer part of the summing variable.
  ver 17
    It appears the Watt cacluation is not correct
    16 was had the MySensor taken out and many print statements were added for diagnostics
    It turned out to be a problem with summing ADC0Volts and ADC0Current (the DC level in counts)
      overflowed into the sign bit.  Changed the types to long int
    Removed the debug print statements with compiler directive
  ver 16
    Assume that the Home Assistant integration "Utility Meter (helper)" is installed
    change the number of decimal points on the kWh to 3
    store kWh as Watts.  Send number so it appears as floating point kWh 
    Changed interval between sends to 10 seconds (AVERAGING_PERIOD)
  ver 15
    Set kWh as a 4 byte unsigned integer
    On boot, read this number from EEPROM
    Each time the float kWh changes flips to another whole kWh
      increment the integer kWh
      store it in EEPROm
      send the integer kWh
      set the float kWh to the fractional part of the float.
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
#include <LibPrintf.h>
#include <EEPROM.h>

//--------------------------------------------------MySensors parameters
#define DEVELOPMENT // MySensors Gateway on channel 86 otherwise 121
#define MY_DEBUG

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
#define WAIT_AFTER_SEND_MESSAGE 300,0              // ms to wait after send message
#define WAIT_AFTER_PRESENTATION 5000,0 // ms to wait after presentation message or echo was returned

// configure communication protocol to Home Assistant for power, energy, voltage, current, VA, and powerfactor
#define CHILD_ID_POWER 0   // Id of the sensor child
MyMessage msgPOWER(CHILD_ID_POWER, V_WATT);
#define CHILD_ID_ENERGY 1
MyMessage msgENERGY(CHILD_ID_ENERGY, V_KWH);
#define CHILD_ID_VOLTAGE 2   // Id of the sensor child
MyMessage msgVOLTAGE(CHILD_ID_VOLTAGE, V_VOLTAGE);
#define CHILD_ID_CURRENT  3  // Id of the sensor child
MyMessage msgCURRENT(CHILD_ID_CURRENT, V_CURRENT);
#define CHILD_ID_VA 4  // Id of the sensor child
MyMessage msgVA(CHILD_ID_VA, V_VA);
#define CHILD_ID_POWER_FACTOR  5  // Id of the sensor child
MyMessage msgPOWER_FACTOR(CHILD_ID_POWER_FACTOR, V_POWER_FACTOR);
#define CHILD_ID_ENERGY_FRACTIONAL 6
MyMessage msgENERGYfractional(CHILD_ID_ENERGY_FRACTIONAL, V_KWH);

//------------------------------------------------------before (starting MySensors)
void before(){ // this happens before MySensors starts
  analogReference(EXTERNAL);  // this needs to be done very near the start so ADC doesn't get buggered
  // usual program information at very start
  Serial.begin(115200);
  Serial.print(PROGRAM_NAME);Serial.print(" version ");Serial.println(VER);
}
//-------------------------------------------------------presentation
void presentation(){
  //Serial.println(F("preseting program name"));
	sendSketchInfo(PROGRAM_NAME, VER,true);
  wait(WAIT_AFTER_PRESENTATION);
  
  //Serial.println(F("preseting Power"));
  present(CHILD_ID_POWER, S_POWER,"Power",true);
  wait(WAIT_AFTER_PRESENTATION);
  
  //Serial.println(F("preseting Cumulative_kWh"));
	present(CHILD_ID_ENERGY, S_POWER, "Energy",true);
  wait(WAIT_AFTER_PRESENTATION);
  
  //Serial.println(F("preseting voltage"));
	present(CHILD_ID_VOLTAGE, S_MULTIMETER, "Voltage",true);
  wait(WAIT_AFTER_PRESENTATION);
  
  //Serial.println(F("preseting Current"));
	present(CHILD_ID_CURRENT, S_MULTIMETER, "Current", true);
  wait(WAIT_AFTER_PRESENTATION);
  
  //Serial.println(F("preseting VA"));
	present(CHILD_ID_VA, S_POWER, "VA",true);
  wait(WAIT_AFTER_PRESENTATION);
  
  //Serial.println(F("preseting Power Factor"));
	present(CHILD_ID_POWER_FACTOR, S_POWER, "Power Factor", true);
  wait(WAIT_AFTER_PRESENTATION);
  
  //Serial.println(F("preseting Cumulative_kWh"));
	present(CHILD_ID_ENERGY_FRACTIONAL, S_POWER, "EnergyFractional",true);
  wait(WAIT_AFTER_PRESENTATION);
}

//--------------------------------------------------Energy Meter parameters
#define TRIGGER_START_SAMPLE_PERIOD_PIN 5
#define TRIGGER_START_SAMPLE_PIN 6
#define VOLTS_IN_PIN A0
#define CURRENT_IN_PIN A1

volatile uint8_t measurementCount = 0;  // must be volatile to allow the wait outside the IRQ to see this value
int16_t samplesVolts[SAMPLES_per_CYCLES];
int16_t samplesCurrent[SAMPLES_per_CYCLES];

// used to measure the time it took to make all the samples.
//   An oscilloscope was used to measure this time.
//   This method included a lot of overhead time

//------------------------------------------------------ISR
ISR(TIMER1_OVF_vect){                    // interrupt service routine for overflow
  TCNT1 = TimerPreloadValue;  // must be first line!  starts the timer counting again
  digitalWrite(TRIGGER_START_SAMPLE_PIN,HIGH);  // this is used to measure the time with an oscilloscope
                                                //   while no longer needed, it probably needs to remain so as to not mess up the timing
                                                //   on second thought, I don't think so
  samplesVolts[--measurementCount]=analogRead(VOLTS_IN_PIN); // decrement before capturing because arrays start at zero and measurementCount starts at number of samples
  samplesCurrent[measurementCount]=analogRead(CURRENT_IN_PIN);
  digitalWrite(TRIGGER_START_SAMPLE_PIN,LOW);  // again, used with oscilloscope
  if (measurementCount) return;  // count down to zero.  This is a trick because when the count decrements to zero, sampling is done
                       // because it is "volatile" variable measurementCount is in RAM and not cache
                       // This makes the code fast

  // terminate sampling by turnning off interupt
  digitalWrite(TRIGGER_START_SAMPLE_PERIOD_PIN,LOW); // indicate that sampling is complete
                                                     // no longer necessary
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
    measurementCount = SAMPLES_per_CYCLES; // count down to zero (8 bit number)
    digitalWrite(TRIGGER_START_SAMPLE_PERIOD_PIN,HIGH);  // used for oscilloscope, no longer necessary

    TCNT1 = 65535; // This number, 2^16-1 will cause the trigger to happen asap!
    TCCR1B |= 1; // turns on timer 
  interrupts();                         // enable all interrupts

  // wait for sampling to be complete
  while(measurementCount); // This should be a very fast wait loop which will allow the IRQ to respond quicker
  
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

// 20 amps/(.0644A/count) = 311 counts,  240V/(0.835V/count) = 287 counts, 4.8kW = 89,389 counts^2, 
// 10 seconds of *  60 cycles / second = 600 cycles ... * 65 samples per cycle = 39000 
// 89389 * 39000 = 3486171000 which is less than 2^32 (4294967296)
//                                                     3486171000
// Thus a 32 bit unsigned integer can handle a 10 second period
#define AVERAGING_PERIOD 10000 // milliseconds
bool firstLoop = true;
unsigned long averagingStart = millis();

float sumOfVrmsADC = 0.0;
float sumOfIrmsADC = 0.0;
int32_t counts2_cumulator_in_AveragingPeriod = 0; // this number could be negative
uint16_t averagingCount = 0;

uint32_t kWh_cumulator = 0;  // total kWh since boot
float kWh_cumulator_fractional = 0.0;
//------------------------------------------------------loop
void loop() {
  if (firstLoop){
    firstLoop = false;
    send(msgENERGY.set(kWh_cumulator)); // make sure HA will get a message for kWh right away
  }
  sampleOneCycle();
  // immediated ADC0 calculations
  //   determine ADC count that is the DC level of the signal
  int32_t  ADC0Volts = 0;  // the ADC value that represents the DC level of the signal volts. 
  int32_t  ADC0Current = 0;  // the ADC value that represents the DC level of the signal amps. 
  for (uint8_t i=0;i<SAMPLES_per_CYCLES;i++){
    ADC0Volts += samplesVolts[i];
    ADC0Current += samplesCurrent[i];
  }
  ADC0Volts /= SAMPLES_per_CYCLES;  // DC level for volts
  ADC0Current /= SAMPLES_per_CYCLES; // DC level for current

  // calculate rms of this cycle.  This is in ADC counts and will be later converted to Volts and Amps
  int32_t counts2_cumulator_in_cycle = 0;  // this number could be negative depending on the direction of the flow of energy!
  uint32_t sumOfSquaresVolts = 0;
  uint32_t sumOfSquaresCurrent = 0;
  for (uint8_t i=0;i<SAMPLES_per_CYCLES;i++){
    sumOfSquaresVolts += sq(samplesVolts[i]-ADC0Volts);
    sumOfSquaresCurrent += sq(samplesCurrent[i]-ADC0Current);
    counts2_cumulator_in_cycle += int32_t(samplesVolts[i]-ADC0Volts) * int32_t(samplesCurrent[i]-ADC0Current);  //
  }
    
  // add this cycle to the sum of for the averaging period  
  sumOfVrmsADC += sqrt(float(sumOfSquaresVolts)/float(SAMPLES_per_CYCLES));  // final part of RMS value
  sumOfIrmsADC += sqrt(float(sumOfSquaresCurrent)/float(SAMPLES_per_CYCLES));  // final part of RMS value
  
  counts2_cumulator_in_AveragingPeriod += counts2_cumulator_in_cycle / 4;  // see documentation above 
  averagingCount++;  //increment the number of cycles sampled during Averaging Cycle
  
  if (millis()-averagingStart > AVERAGING_PERIOD){
    averagingStart = millis();  // reset the timer that will indicate when data will be sent

    // power, energy, voltage, current, VA, and powerfactor
   
    // Calculate Watts,
    // Not every cycle in the AVERAGING_PERIOD was measured
    // The assumption is made that the energy was the same in the cycles missed
    // Thus the sum of Watts is increased by the percentage of cycles missed
    // There are LINE_FREQUENCY * AVERAGING_PERIOD / 1000 (Sec/mSec) number of cycles in the 
    //   AVERAGING_PERIOD is in milliseconds
    // There were averagingCount cycles sampled
    // Thus Watts are increased by LINE_FREQUENCY * AVERAGING_PERIOD / 1000 / averagingCount
    // 
    // VOLTS_per_COUNT                       V/count
    // AMPS_per_COUNT                        A/count
    // counts2_cumulator_in_AveragingPeriod  count^2   This a form of watts
    // LINE_FREQUENCY                        Cycles/second
    // SAMPLES_per_CYCLES                    Samples/Cycle
    // AVERAGING_PERIOD                      milliseconds
    
    // Total Samples in AVERAGING_PERIOD  = averagingCount * SAMPLES_per_CYCLES


    // 
    //   |<-- W-s/s               |<--   Watts-AVERAGING_PERIOD       ()                                                               c/s                  S/c                -->|<-- seconds per AVERAGING_PERIOD-->|
    float Watt_seconds_per_second = ( (VOLTS_per_COUNT*AMPS_per_COUNT * (float(counts2_cumulator_in_AveragingPeriod) * 4.0)) /(float(LINE_FREQUENCY)*float(SAMPLES_per_CYCLES)) ) / (float(AVERAGING_PERIOD)/1000.0);  // this is W-S for 1 second
    float Watt_seconds_per_second_adjusted =  Watt_seconds_per_second * float(LINE_FREQUENCY) * (float(AVERAGING_PERIOD)/1000.0) / float(averagingCount);  // adjusted for cycles not measured
    kWh_cumulator_fractional += (Watt_seconds_per_second_adjusted * (float(AVERAGING_PERIOD)/1000.0) / 3600.0) / 1000.0;  // this is the Wh for the sampling period adjusted for cycles not measured
    //    |<-- W
    //float Average_Watts_during_Averaging_Period = Watt_seconds_per_second_adjusted / (float(AVERAGING_PERIOD)/1000.0);  // W-S per seconds

    // Calculate Votage during averaging cycle
    float voltage = VOLTS_per_COUNT * sumOfVrmsADC / float(averagingCount);
    // Calculate Current during averaging cycle
    float current = AMPS_per_COUNT * sumOfIrmsADC / float(averagingCount);
    // Calculat VA during averaging cycle
    float VA = voltage * current;
    // Calculate power factor during averaging cycle
    float PowerFactor = Watt_seconds_per_second_adjusted/VA;

    send(msgPOWER.set(Watt_seconds_per_second_adjusted,0),true);
    wait(WAIT_AFTER_SEND_MESSAGE);
    // only send energy when integer part changes
    if (kWh_cumulator_fractional > 1.0) {
      uint32_t integer_kWh = uint32_t(kWh_cumulator_fractional);
      kWh_cumulator_fractional -= float(integer_kWh); // strip of the interger part and keep accumulating
      kWh_cumulator += integer_kWh; // accumulate
      send(msgENERGY.set(kWh_cumulator),true);
      wait(WAIT_AFTER_SEND_MESSAGE);
    }
    send(msgENERGYfractional.set(kWh_cumulator_fractional,3),true);
    wait(WAIT_AFTER_SEND_MESSAGE);
    
    send(msgVOLTAGE.set(voltage,1),true);
    wait(WAIT_AFTER_SEND_MESSAGE);
    send(msgCURRENT.set(current,2),true);
    wait(WAIT_AFTER_SEND_MESSAGE);
    send(msgVA.set(VA,0),true);
    wait(WAIT_AFTER_SEND_MESSAGE);
    send(msgPOWER_FACTOR.set(PowerFactor,2),true);
    wait(WAIT_AFTER_SEND_MESSAGE);

    // reset counters
    sumOfVrmsADC = 0.0;
    sumOfIrmsADC = 0.0;
    counts2_cumulator_in_AveragingPeriod = 0.0;
    averagingCount = 0;

  }

}

//------------------------------------------------------receive
/*
void receive(const MyMessage &message) {
  printf("Received Message --- command: %d Type: %d\r\n",message.getCommand(),message.getPayloadType());
}
*/