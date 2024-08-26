/*
  Emon.cpp - Library for openenergymonitor
  Created by Trystan Lea, April 27 2010
  GNU GPL
  modified to use up to 12 bits ADC resolution (ex. Arduino Due)
  by boredman@boredomprojects.net 26.12.2013
  Low Pass filter for offset removal replaces HP filter 1/1/2015 - RW
*/

// Proboscide99 10/08/2016 - Added ADMUX settings for ATmega1284 e 1284P (644 / 644P also, but not tested) in readVcc function

//#include "WProgram.h" un-comment for use on older versions of Arduino IDE
#include "EmonLib.h"

#if defined(ARDUINO) && ARDUINO >= 100
# include "Arduino.h"
#else
# ifdef RPI_PICO
#  include <stdio.h>
#  include <sys/time.h>
#  include "pico/stdlib.h"
#  include "RPiPicoEmul.h"
# else
#  include "WProgram.h"
# endif
#endif

//--------------------------------------------------------------------------------------
// Sets the pins to be used for voltage and current sensors
//--------------------------------------------------------------------------------------
void EnergyMonitor::voltage(unsigned int _inPinV, double _VCAL, double _PHASECAL)
{
  inPinV = _inPinV;
  VCAL = _VCAL;
  PHASECAL = _PHASECAL;
  offsetV = ADC_COUNTS>>1;
}

void EnergyMonitor::current(unsigned int _inPinI, double _ICAL)
{
  inPinI = _inPinI;
  ICAL = _ICAL;
  offsetI = ADC_COUNTS>>1;
}

//--------------------------------------------------------------------------------------
// Sets the pins to be used for voltage and current sensors based on emontx pin map
//--------------------------------------------------------------------------------------
void EnergyMonitor::voltageTX(double _VCAL, double _PHASECAL)
{
  inPinV = 2;
  VCAL = _VCAL;
  PHASECAL = _PHASECAL;
  offsetV = ADC_COUNTS>>1;
}

void EnergyMonitor::currentTX(unsigned int _channel, double _ICAL)
{
  if (_channel == 1) inPinI = 3;
  if (_channel == 2) inPinI = 0;
  if (_channel == 3) inPinI = 1;
  ICAL = _ICAL;
  offsetI = ADC_COUNTS>>1;
}

//--------------------------------------------------------------------------------------
// emon_calc procedure
// Calculates realPower,apparentPower,powerFactor,Vrms,Irms,kWh increment
// From a sample window of the mains AC voltage and current.
// The Sample window length is defined by the number of half wavelengths or crossings we choose to measure.
//--------------------------------------------------------------------------------------
void EnergyMonitor::calcVI(unsigned int crossings, unsigned int timeout, unsigned int loopDelay)
{
  #if defined emonTxV3
  int SupplyVoltage=3300;
  #else
  int SupplyVoltage = readVcc();
  #endif

  unsigned int crossCount = 0;                             //Used to measure number of times threshold is crossed.
  unsigned int numberOfSamples = 0;                        //This is now incremented

  //-------------------------------------------------------------------------------------------------------------------------
  // 1) Waits for the waveform to be close to 'zero' (mid-scale adc) part in sin curve.
  //-------------------------------------------------------------------------------------------------------------------------
  unsigned long start = millis();    //millis()-start makes sure it doesnt get stuck in the loop if there is an error.

  while(1)                                   //the while loop...
  {
    startV = analogRead(inPinV);                    //using the voltage waveform
    if ((startV < (ADC_COUNTS*0.55)) && (startV > (ADC_COUNTS*0.45))) break;  //check its within range
    if ((millis()-start)>timeout) break;
  }

#ifdef CALIBRATION

  // ------------------------------------------------------------------------
  // General info and current values at start of this call
  // Especially needed for calibration simulator is "Start Offsets"
  // ------------------------------------------------------------------------

  printf("PIN I: %d, U: %d\n", inPinI, inPinV);
  printf("ADC range: %d\n", ADC_COUNTS);
  printf("Supply: %dmV, vCal: %.2f, phaseCal: %.2f\n", SupplyVoltage, VCAL, PHASECAL);
  unsigned long now = millis();
  printf("Start %ld Timeout: %ld, now: %ld -> %ldms\n", start, timeout, now, (now - start));
  printf("Start Offsets: I: %.2f U: %.2f\n", offsetI, offsetV);
  struct timeval t1;
  struct timeval tLast;

  uint samplesI[CALIBRATION_NUM_SAMPLES];
  uint samplesV[CALIBRATION_NUM_SAMPLES];
  unsigned long samplesT[CALIBRATION_NUM_SAMPLES];

  gettimeofday(&t1, NULL);

#endif // CALIBRATION_NUM_SAMPLES

  uint phaseCalInt = (uint) PHASECAL; // as int type
  double phaseShiftedI;
  double pcSamples[PHASECAL_BUFFER_SIZE]; // PHASECAL cannot be more than this

  //-------------------------------------------------------------------------------------------------------------------------
  // 2) Main measurement loop
  //-------------------------------------------------------------------------------------------------------------------------
  start = millis();

  while ((crossCount < crossings) && ((millis()-start)<timeout))
  {
    numberOfSamples++;                       //Count number of times looped.
    lastFilteredV = filteredV;               //Used for delay/phase compensation

    //-----------------------------------------------------------------------------
    // A) Read in raw voltage and current samples
    //-----------------------------------------------------------------------------

#ifdef CALIBRATION
    // Determine the time needed for this loop cycle
    tLast = t1;
    gettimeofday(&t1, NULL);
#endif // CALIBRATION

    sampleV = analogRead(inPinV);                 //Read in raw voltage signal
    sampleI = analogRead(inPinI);                 //Read in raw current signal

    //-----------------------------------------------------------------------------
    // B) Apply digital low pass filters to extract the 2.5 V or 1.65 V dc offset,
    //     then subtract this - signal is now centred on 0 counts.
    //-----------------------------------------------------------------------------
    offsetV = offsetV + ((sampleV-offsetV)/1024);
    filteredV = sampleV - offsetV;
    offsetI = offsetI + ((sampleI-offsetI)/1024);
    filteredI = sampleI - offsetI;

    //-----------------------------------------------------------------------------
    // C) Root-mean-square method voltage
    //-----------------------------------------------------------------------------
    sqV= filteredV * filteredV;                 //1) square voltage values
    sumV += sqV;                                //2) sum

    //-----------------------------------------------------------------------------
    // D) Root-mean-square method current
    //-----------------------------------------------------------------------------
    sqI = filteredI * filteredI;                //1) square current values
    sumI += sqI;                                //2) sum

    //-----------------------------------------------------------------------------
    // E) Phase calibration
    //-----------------------------------------------------------------------------
    if(phaseCalInt > 2)
    {
      // -------------------------------------------------------------- 
      // phaseCal > 2 means that V is ahead of I more than the time needed for one loop cycle.
      // Especially needed with high sampling rates (here the deltaT is so small, that the classic
      // phase calibration has no sufficient effect anymore).
      // The actual phaseCal value can be determined using the simulation spreadsheet.
      // Idea is to use an I sample value <phaseCal> samples in the past.
      // See for example (phaseCal = 26):
      //  https://community.openenergymonitor.org/t/sct013-value-consistently-too-low/26480/20
      // -------------------------------------------------------------- 

      // Store PHASECAL_BUFFER_SIZE samples in a circular buffer
      pcSamples[numberOfSamples & (PHASECAL_BUFFER_SIZE - 1)] = filteredI;

      // We want the sample index "phaseCalInt" in the past
      uint refSample = numberOfSamples - phaseCalInt;

      // Chose val depending on whether we already have that val, if so determine right index in circular buffer
      phaseShiftedI = (refSample > 0 ? pcSamples[refSample & (PHASECAL_BUFFER_SIZE - 1)] : 0);
      phaseShiftedV = filteredV;
    }
    else if(phaseCalInt < 0)
    {
      // Same thing with I being ahead of V
      pcSamples[numberOfSamples & (PHASECAL_BUFFER_SIZE - 1)] = filteredV;
      uint refSample = numberOfSamples - phaseCalInt;
      phaseShiftedV = (refSample > 0 ? pcSamples[refSample & (PHASECAL_BUFFER_SIZE - 1)] : 0);
      phaseShiftedI = filteredI;
    }
    else
    {
      // "Classic" phase calibration, tune V value with previous sample
      phaseShiftedV = lastFilteredV + PHASECAL * (filteredV - lastFilteredV);
      phaseShiftedI = filteredI;
    }

    //-----------------------------------------------------------------------------
    // F) Instantaneous power calc
    //-----------------------------------------------------------------------------
    instP = phaseShiftedV * phaseShiftedI;      //Instantaneous Power
    sumP +=instP;                               //Sum

    //-----------------------------------------------------------------------------
    // G) Find the number of times the voltage has crossed the initial voltage
    //    - every 2 crosses we will have sampled 1 wavelength
    //    - so this method allows us to sample an integer number of half wavelengths which increases accuracy
    //-----------------------------------------------------------------------------
    lastVCross = checkVCross;
    if (sampleV > startV) checkVCross = true;
                     else checkVCross = false;
    if (numberOfSamples==1) lastVCross = checkVCross;

    if (lastVCross != checkVCross) crossCount++;

    if(loopDelay > 0)
    {
      sleep_us(loopDelay);
    }

#ifdef CALIBRATION
    // Store sample values of this cycle
    if(numberOfSamples <= CALIBRATION_NUM_SAMPLES)
    {
      samplesI[numberOfSamples - 1] = sampleI;
      samplesV[numberOfSamples - 1] = sampleV;
      samplesT[numberOfSamples - 1] = tLast.tv_usec;
    }
#endif // CALIBRATION
  }

#ifdef CALIBRATION
  unsigned long now2 = millis();
#endif // CALIBRATION

  //-------------------------------------------------------------------------------------------------------------------------
  // 3) Post loop calculations
  //-------------------------------------------------------------------------------------------------------------------------
  //Calculation of the root of the mean of the voltage and current squared (rms)
  //Calibration coefficients applied.

  double V_RATIO = VCAL *((SupplyVoltage/1000.0) / (ADC_COUNTS));
  Vrms = V_RATIO * sqrt(sumV / numberOfSamples);

  double I_RATIO = ICAL *((SupplyVoltage/1000.0) / (ADC_COUNTS));
  Irms = I_RATIO * sqrt(sumI / numberOfSamples);

#ifdef CALIBRATION
  // Print collected sample values in CSV format - ready to load into
  // calibration simulator spreadsheet
  printf("End meas loop: %ldms NumSamples: %d (%d)\n", (now2 - now), numberOfSamples, phaseCalInt);
  printf("t;ADC V;ADC I\n");
  for(int i = 0; (i < CALIBRATION_NUM_SAMPLES && i < numberOfSamples); i++)
  {
    printf("%ld;%d;%d\n", samplesT[i], samplesV[i], samplesI[i]);
  }
#endif // CALIBRATION

  //Calculation power values
  realPower = V_RATIO * I_RATIO * sumP / (phaseCalInt > 2 ? (numberOfSamples - phaseCalInt) : numberOfSamples);
  apparentPower = Vrms * Irms;
  powerFactor=realPower / apparentPower;

  //Reset accumulators
  sumV = 0;
  sumI = 0;
  sumP = 0;
//--------------------------------------------------------------------------------------
}

//--------------------------------------------------------------------------------------
double EnergyMonitor::calcIrms(unsigned int Number_of_Samples)
{

  #if defined emonTxV3
    int SupplyVoltage=3300;
  #else
    int SupplyVoltage = readVcc();
  #endif


  for (unsigned int n = 0; n < Number_of_Samples; n++)
  {
    sampleI = analogRead(inPinI);

    // Digital low pass filter extracts the 2.5 V or 1.65 V dc offset,
    //  then subtract this - signal is now centered on 0 counts.
    offsetI = (offsetI + (sampleI-offsetI)/1024);
    filteredI = sampleI - offsetI;

    // Root-mean-square method current
    // 1) square current values
    sqI = filteredI * filteredI;
    // 2) sum
    sumI += sqI;
  }

  double I_RATIO = ICAL *((SupplyVoltage/1000.0) / (ADC_COUNTS));
  Irms = I_RATIO * sqrt(sumI / Number_of_Samples);

  //Reset accumulators
  sumI = 0;
  //--------------------------------------------------------------------------------------

  return Irms;
}

void EnergyMonitor::serialprint()
{
  Serial.print(realPower);
  Serial.print(' ');
  Serial.print(apparentPower);
  Serial.print(' ');
  Serial.print(Vrms);
  Serial.print(' ');
  Serial.print(Irms);
  Serial.print(' ');
  Serial.print(powerFactor);
  Serial.println(' ');
  delay(100);
}

//thanks to http://hacking.majenko.co.uk/making-accurate-adc-readings-on-arduino
//and Jérôme who alerted us to http://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/

long EnergyMonitor::readVcc() {
  long result;

  //not used on emonTx V3 - as Vcc is always 3.3V - eliminates bandgap error and need for calibration http://harizanov.com/2013/09/thoughts-on-avr-adc-accuracy/

  #if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328__) || defined (__AVR_ATmega328P__)
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined(__AVR_ATmega644__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_AT90USB1286__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  ADCSRB &= ~_BV(MUX5);   // Without this the function always returns -1 on the ATmega2560 http://openenergymonitor.org/emon/node/2253#comment-11432
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);

  #endif


  #if defined(__AVR__)
  delay(2);                                        // Wait for Vref to settle
  ADCSRA |= _BV(ADSC);                             // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = READVCC_CALIBRATION_CONST / result;  //1100mV*1024 ADC steps http://openenergymonitor.org/emon/node/1186
  return result;
  #elif defined(__arm__)
  return (3300);                                  //Arduino Due
  #else
  return (3300);                                  //Guess that other un-supported architectures will be running a 3.3V!
  #endif
}

