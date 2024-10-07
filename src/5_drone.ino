// 5 drones with fixed LFOs. Much reduced and simplified from the Helios Drone code.
// Includes frequency-based volume correction to even out energy across the frequency range.
// - Brandon Istenes, 2024

 // A BlogHoskins Monstrosity @ 2019 / 2020
// https://bloghoskins.blogspot.com/
/*    v5.0a (Added by CallPhysical, March 2022)
 *    NOTE: This is a 6-voice drone that will run on the standard Helios build
 *    Toggle between Helios or Drone mode at power on using LFO switch
 *    If LFO switch is HIGH, unit will enter Drone mode, otherwise, it will enter Helios mode*    
 *    Drone mode is based on  Control_Oscill_Wash using 6 pots to change pitch
 *    and LFO switches 2 and 3 to change oscillator type
 *     
 *    v4.6 
 *    Time to fix distortion - Fixed
 *    changed update audio from 8 -> 9
 *    Change Filter Res and Cut-off from >>2 to mozzieMap -> done
 *    change lfo amount pot - check if all are still in spec with multimeter - done
 *    add resistors? - yep, done & done
 *    reduce resonance total in res pot - done
 *    
 *    v4.5
 *    LFO On/off switch added - using arduino digital pin 3
 *     
 *    v4.4 
 *    Applied advice given from Mozzi Forums - working much better now - thanks Staffan
 *    Still need to apply values to stop distortion + add on/off switch for LFO
 *     
 *    v4.3 LFO implementation
 *    LFO is working(?), but filter is interfering.  Figure it out!
 *    *replaced pin assignments 'const int' with #define -> now reassign values to stop distortion
 *    
 *    
 *    V3.1 - Minor Updates
 *   *IImproved Filter Resonance (?).  Apparantly After setting resonance, you need to call setCuttoffFreq() to hear the change.
 *   Swapped order of cut-off and resonance in code.  Filter Sounds Better now?
 *   *Increased note sustain from 10 seconds to 60 seconds
 *   *OSC OUTPUT made louder on audio update>>9; // changed down from 10 to 9 because was louder
 *   
 *   
 *   V3
 *   In this version we add a low pass filter.  A cut off pot and resonance pot are added.
 *   For this you'll need two B10k pots, and a 220ohm resistor.
 *   A3: for Resonance (center pot).  Other lugs go to ground and +5v
 *   A2: for Cut-off (center pot).   Other lugs go to ground (with 220ohm resistor) and +5v
 *   
 *   
 *   v2
 *   https://bloghoskins.blogspot.com/2020/06/helios-one-arduino-synth-part-2.html
 *   This version adds Attack & Release on the analog inputs. You'll need 2 pots. 
 *   Connect center pot pin to;
 *   A5: for Atk
 *   A4: for Release
 *   connect the other pot pins to ground and 5v. 
 *   To stop mis-triggering on the atk I have x2 1k resistors in parallel (amounting 
 *   to 500 ohms resistance) on the ground input of the atk pot. you could put two 
 *   200(ish)ohm resisters in series instead, or play with the code...  maybe set the
 *   int val of the atkVal to something over 20 instead of 0?
 *   
 *   
 *   v1
 *   http://bloghoskins.blogspot.com/2019/07/helios-one-arduino-synth-part-1.html
 *   This vesrion of code lets you set between SQR and SAW wave with a 
 *   switch (input 2 on arduino)
 *   MIDI is working.
 *   You'll need to install the MIDI & Mozzi arduino libraries to get it to work.
*/


#include <MozziGuts.h>
#include <Oscil.h> // oscillator template
#include <ADSR.h>
#include <LowPassFilter.h> // You need this for the LPF
#include <AutoMap.h> // to track and reassign the pot values

#include <tables/saw2048_int8.h> // saw table for oscillator  -- Shared by Helios and Drone
#include <tables/square_no_alias_2048_int8.h> // square table for oscillator
#include <tables/sin2048_int8.h>                // for Drone
#include <tables/triangle_valve_2_2048_int8.h>  // for Drone

//*****************LFO******************************************************************************************
#include <tables/cos2048_int8.h> // for filter modulation
#include <StateVariable.h>
#include <mozzi_rand.h> // for rand()
//***************END********************************************************************************************

//********ADD LFO FILTER MODULATION OSC*************************************************************************
Oscil<COS2048_NUM_CELLS, CONTROL_RATE> kFilterMod(COS2048_DATA);
StateVariable <NOTCH> svf; // can be LOWPASS, BANDPASS, HIGHPASS or NOTCH
//***************END********************************************************************************************

//// Set up LFO Rate Pot****************************************************************************************
#define LFOratePot A1    // select the input pin for the potentiometer
AutoMap LFOratePotMap(0, 1023, 40, 750);  // LFO Rate mapped 25 to 1300 (up from 5 - 1023)
//***************END********************************************************************************************

//// ****************Set up LFO Res Pot*************************************************************************
#define LFOresPot A0    // select the input pin for the potentiometer
AutoMap LFOresPotMap(0, 1023, 2, 212);  // 0-255 val distorted, 2-212 within range - find better value? Was 40-180
//***************END********************************************************************************************

//Create an instance of a low pass filter
LowPassFilter lpf; 

#define WAVE_SWITCH 2 // switch for switching waveforms
#define LFO_ON_OFF_SWITCH 3 // switch for LFO ON/OFF

//*******CUT-OFF POT***********
#define cutoffPot A2    // cut-off pot will be on A2
AutoMap cutOffPotMap(0, 1023, 20, 250);  // 0 - 255

//*******RESONANCE POT***********
#define resPot A3    // resonance pot will be on A2
AutoMap resFreqPotMap(0, 1023, 0, 255);  // 0 - 255

// use #define for CONTROL_RATE, not a constant
#define CONTROL_RATE 128 // powers of 2 please

////*********************** Defaults for Drone oscillators ***************************************************
// harmonics
Oscil<SIN2048_NUM_CELLS, AUDIO_RATE> aOsc1;
Oscil<SIN2048_NUM_CELLS, AUDIO_RATE> aOsc2;
Oscil<SIN2048_NUM_CELLS, AUDIO_RATE> aOsc3;
Oscil<SIN2048_NUM_CELLS, AUDIO_RATE> aOsc4;
Oscil<SIN2048_NUM_CELLS, AUDIO_RATE> aOsc5;

// volume controls, use sin regardless of the harmonic oscillator
Oscil<SIN2048_NUM_CELLS, CONTROL_RATE> kVol1(SIN2048_DATA);
Oscil<SIN2048_NUM_CELLS, CONTROL_RATE> kVol2(SIN2048_DATA);
Oscil<SIN2048_NUM_CELLS, CONTROL_RATE> kVol3(SIN2048_DATA);
Oscil<SIN2048_NUM_CELLS, CONTROL_RATE> kVol4(SIN2048_DATA);
Oscil<SIN2048_NUM_CELLS, CONTROL_RATE> kVol5(SIN2048_DATA);

//***************END********************************************************************************************

// ******** variables for Drone *******************************************************************************
char v1,v2,v3,v4,v5;  //char is more efficient than int, for some reason
int f1,f2,f3,f4,f5;  // store the values from the analog pins to set frequency
int wav = 0;      // store the value of the two wave switches
int lfo = 0;      // store the value of the LFO switch
int oldWav = 0;   // remember the previous value of the wave switches, so we know if it's changed
int upCycle = 1;  // count the update cycle.  We only read a few of the inputs on each cycle, to reduce clicking souds
//***************END********************************************************************************************

void setup() {
  Serial.begin(9600); // see the output

//****************Set toggle switches***************************************************************************** 
  pinMode(2, INPUT_PULLUP); // Pin two to switch, high switch goes to 5v, low to ground. Selects WAVE in Helios and Drone mode
  pinMode(3, INPUT_PULLUP); // Pin three to switch, high switch goes to 5v, low to ground. Selects LFO in Helios and WAVE in Drone
//****************End*********************************************************************************************

// ****************Set default frequencies for Drone**************************************************************

// set harmonic frequencies
  aOsc1.setFreq(100);
  aOsc2.setFreq(100);
  aOsc3.setFreq(100);
  aOsc4.setFreq(100);
  aOsc5.setFreq(100);

//// set volume change freq
  kVol1.setFreq(6.0f);
  kVol2.setFreq(0.28f);
  kVol3.setFreq(0.28f);
  kVol4.setFreq(0.188f);
  kVol5.setFreq(0.36f);

  v1=v2=v3=v4=v5=128;
  
//****************End*********************************************************************************************

  startMozzi(CONTROL_RATE); 
  setupFastAnalogRead(FASTER_ADC);  // doesn't seem to help.  Remove?
  
}

// the following four voids switch the harmonic oscillator waveforms
// you can try other waveform types, but keep them all to the same # of samples, eg 2048

// ******************************** oscillator type 1 for Drone ************************************************
void setTable1() { 
      aOsc1.setTable(SIN2048_DATA);
      aOsc2.setTable(SIN2048_DATA);
      aOsc3.setTable(SIN2048_DATA);
      aOsc4.setTable(SIN2048_DATA);
      aOsc5.setTable(SIN2048_DATA);
      }
// ******************************** oscillator type 2 for Drone ************************************************      
void setTable2() {
      aOsc1.setTable(SQUARE_NO_ALIAS_2048_DATA);
      aOsc2.setTable(SQUARE_NO_ALIAS_2048_DATA);
      aOsc3.setTable(SQUARE_NO_ALIAS_2048_DATA);
      aOsc4.setTable(SQUARE_NO_ALIAS_2048_DATA);
      aOsc5.setTable(SQUARE_NO_ALIAS_2048_DATA);
       }
// ******************************** oscillator type 3 for Drone ************************************************    
void setTable3() {
      aOsc1.setTable(TRIANGLE_VALVE_2_2048_DATA);
      aOsc2.setTable(TRIANGLE_VALVE_2_2048_DATA);
      aOsc3.setTable(TRIANGLE_VALVE_2_2048_DATA);
      aOsc4.setTable(TRIANGLE_VALVE_2_2048_DATA);
      aOsc5.setTable(TRIANGLE_VALVE_2_2048_DATA);
      }
// ******************************** oscillator type 4 for Drone ************************************************
void setTable4() {
      aOsc1.setTable(SAW2048_DATA);
      aOsc2.setTable(SAW2048_DATA);
      aOsc3.setTable(SAW2048_DATA);
      aOsc4.setTable(SAW2048_DATA);
      aOsc5.setTable(SAW2048_DATA);
    } 
// ******************************** end of oscillator setTable voids ********************************************

void loop() {
  audioHook(); // required here
} 

float frequencyBasedCompression(int f) {
  if (f < 25) {
    return 0.0f;
  }
  float compression = 1.0f / pow(f, 0.7) * 9;
  return compression;
}

void updateControl()
{
  //********************************* START OF THE DRONE CONTROLS ********************************************************
  // To reduce regular clicking sounds caused by performance problems, 
  // the control updates are split into 5 cycles, as follows:
  // In cycles 1 and 3, four of the eight pots are read
  // In cycles 2 and 4, the values read from the pots are set to the oscillators
  // Also, two oscillator volume changes are progressed in each cycle
  // In cycle 5, the digital inputs are read, and if changed, the oscillator types are switched

  switch (upCycle) {
    case 1:  // read three analog pins 
      f1=(mozziAnalogRead(A0));
      f2=(mozziAnalogRead(A1));
      f3=(mozziAnalogRead(A2));
    break;
    case 2:  // set three oscillator frequencies
      aOsc1.setFreq(f1);
      aOsc2.setFreq(f2);
      aOsc3.setFreq(f3);
      v1 = kVol1.next() * frequencyBasedCompression(f1);  // update three oscillator volumes
      v2 = kVol2.next() * frequencyBasedCompression(f2);
      v3 = kVol3.next() * frequencyBasedCompression(f3);
   
    break;
    case 3:     // read three analog pins 
      f4=(mozziAnalogRead(A3)); 
      f5=(mozziAnalogRead(A4));
      break;
     case 4: // update the three upper oscillators
      aOsc4.setFreq(f4 << 1);      
      aOsc5.setFreq(f5 << 1);
      v4 = kVol4.next() * frequencyBasedCompression(f4);     // update the three upper oscillator volumes
      v5 = kVol5.next() * frequencyBasedCompression(f5);
    break; 
    case 5:   // read two digital pins
      wav = digitalRead(2) + (digitalRead(3)<<1); 
      if (wav!=oldWav) // swap the oscillator tables, but only if the switch position has changed
        {
        Serial.print("changing wav! ");
        Serial.println(wav);
        switch (wav) {
            case 1:
            setTable1();
            break;
            case 2:
            setTable2();
            break;
            case 3:
            setTable3();
            break;
            default: // case 0
            setTable4();
            break;      
            } // end of switch wav
       
        oldWav = wav;   //remember the old switch positions
      }  
    } 
    upCycle++;  // ready for the next update cycle
    if (upCycle > 5) {
      upCycle = 1;
    };

}

int updateAudio(){
  long asigDRN = (long)
    aOsc1.next()*v1 +
    aOsc2.next()*v2 +
    aOsc3.next()*v3 +
    aOsc4.next()*v4 +
    aOsc5.next()*v5;
  asigDRN >>= 8;
  return (int) asigDRN;
}
