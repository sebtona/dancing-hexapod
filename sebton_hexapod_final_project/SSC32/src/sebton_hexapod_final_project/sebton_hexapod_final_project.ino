// Dancing Hexapod by Aharon Sebton


#include "SSC32.h"        // Allows SSC-32U to control motors independently of Arduino
#include "arduinoFFT.h"   // Library to perform FFT on time sampled audio signal
SSC32 mySSC32;            // Instantiate an SSC32 object
bool mybool = true;

// Using Hexapod Number 5
// Define Motor Pin Numbers
#define LFlift 30     // Pin 30 on the SSC-32U controls the left front lift motor
#define LFstride 31   // Pin 31 on the SSC-32U controls the left front stride motor
#define LClift 28     // Pin 28 on the SSC-32U controls the left center lift motor
#define LCstride 29   // Pin 29 on the SSC-32U controls the left center stride motor
#define LBlift 16     // Pin 16 on the SSC-32U controls the left back lift motor
#define LBstride 17   // Pin 17 on the SSC-32U controls the left back stride motor
#define RFlift 14     // Pin 14 on the SSC-32U controls the right front lift motor
#define RFstride 15   // Pin 15 on the SSC-32U controls the right front stride motor
#define RClift 10     // Pin 10 on the SSC-32U controls the right center lift motor
#define RCstride 11   // Pin 11 on the SSC-32U controls the right center stride motor
#define RBlift 0      // Pin 0 on the SSC-32U controls the right back lift motor
#define RBstride 1    // Pin 1 on the SSC-32U controls the right back stride motor

int channels[] = {RBlift,RBstride,RClift,RCstride,RFlift,RFstride,LBlift,LBstride,LClift,LCstride,LFlift,LFstride};   // Define motor channels that will need to be enabled
int homeFrame[] = {1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500};    // Define robot starting frame (standing upright)

arduinoFFT FFT = arduinoFFT();    // Instantiate an FFT object
#define CHANNEL1 A0               // Pin A0 on the Arduino provides the data collected by the front microphone
#define CHANNEL2 A1               // Pin A1 on the Arduino provides the data collected by the back microphone
const uint16_t samples = 128;     // Number of audio samples taken per frame. This value should be a power of 2
const double samplingFrequency = 16; // Sampling frequency in hertz

unsigned int sampling_period_us;  // Instantiate sampling period variable
unsigned long microseconds;       // Instantiate variable to capture starting time with
double x;                         // Instantiate dominant frequency variable

double vReal[samples];            // Instantiate input and output vectors for FFT related operations
double vImag[samples];

void setup() {
  mySSC32.begin(9600);              // Begin communication with SSC-32U
  mySSC32.enableServos(channels);   // Enable servo motors
  sethome();                        // Set robot to home position
  sampling_period_us = round(1000000*(1.0/samplingFrequency));  // Calculate and store audio sampling period in microseconds
}

void loop() {
  getBPM();           // Acquire audio samples, process them, and update the current BPM
  up_down();          // Bob up and down to the beat
  getBPM();           // Acquire audio samples, process them, and update the current BPM
  hind_legs();        // Stand on hind legs and wave pinchers to the beat
  getBPM();           // Acquire audio samples, process them, and update the current BPM
  ripple_rotate();    // Turn to the beat
}

void getBPM() {
  // Acquire samples
  microseconds = micros();    // Store starting time
  for(int i=0; i<samples; i++)
  {
      vReal[i] = analogRead(CHANNEL1);     // Read audio signal from front microphone (amplitude is purely real)
      vImag[i] = 0;                        // Zero out imaginary vector
      while(micros() - microseconds < sampling_period_us){    // Wait until the sampling period has passed
        
      }
      microseconds += sampling_period_us;   // Update current/starting time
  }

  FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  // Weigh audio signal appropriately using Hamming window

  FFT.Compute(vReal, vImag, samples, FFT_FORWARD); // Compute the FFT

  FFT.ComplexToMagnitude(vReal, vImag, samples); // Find the magnitude of the FFT

  vReal[0]=0;               // Remove the first FFT data point (Frequency will never be zero)
  vReal[samples/2-1]=0;     // Remove the last FFT data point (Frequency will never be 16 Hz)
  
  x = FFT.MajorPeak(vReal, samples, samplingFrequency);   // Find maximum of FFT magnitude values
  while(x<1 || x>3) {     // If peak frequency is not between 1 and 3 Hz, we have a harmonic of the true frequency
    if (x<1) {
      x=x*2;              // If below 1 Hz, multiply by 2
    }
    if (x>3) {
      x=x/2;              // If above 3 Hz, divide by 2
    }
  }
}

void sethome()    // Set robot to home position
{
  mySSC32.setFrame(homeFrame);
  mySSC32.setFrame(channels,homeFrame);
}

void hind_legs() {
  int tframe1[] = {homeFrame[0]-400,homeFrame[1],homeFrame[2]+400,homeFrame[3],homeFrame[4],homeFrame[5],homeFrame[6]+400,homeFrame[7],homeFrame[8]-400,homeFrame[9],homeFrame[10],homeFrame[11]};  // Up On Hind Legs
  int tframe2[] = {homeFrame[0]-400,homeFrame[1],homeFrame[2]+400,homeFrame[3],homeFrame[4],homeFrame[5]+400,homeFrame[6]+400,homeFrame[7],homeFrame[8]-400,homeFrame[9],homeFrame[10],homeFrame[11]-400};  // Pinchers Forward
  int tframe3[] = {homeFrame[0]-400,homeFrame[1],homeFrame[2]+400,homeFrame[3],homeFrame[4]-400,homeFrame[5]+400,homeFrame[6]+400,homeFrame[7],homeFrame[8]-400,homeFrame[9],homeFrame[10]-400,homeFrame[11]-400};  // Right Pincher Up, Left Pincher Down
  int tframe4[] = {homeFrame[0]-400,homeFrame[1],homeFrame[2]+400,homeFrame[3],homeFrame[4]+400,homeFrame[5]+400,homeFrame[6]+400,homeFrame[7],homeFrame[8]-400,homeFrame[9],homeFrame[10]+400,homeFrame[11]-400};  // Right Pincher Down, Left Pincher Up

  mySSC32.setFrame(tframe1);    // Transition robot to frame 1 defined above
  delay(1000/x);                // Wait 1000/x milliseconds before transitioning to the next frame
  mySSC32.setFrame(tframe2);
  delay(1000/x);
  mySSC32.setFrame(tframe3);
  delay(1000/x);
  mySSC32.setFrame(tframe4);
  delay(1000/x);
  mySSC32.setFrame(tframe3);
  delay(1000/x);
  mySSC32.setFrame(homeFrame);
  delay(1000/x);
  mySSC32.setFrame(tframe1);
  delay(1000/x);
  mySSC32.setFrame(tframe2);
  delay(1000/x);
  mySSC32.setFrame(tframe3);
  delay(1000/x);
  mySSC32.setFrame(tframe4);
  delay(1000/x);
  mySSC32.setFrame(tframe3);
  delay(1000/x);
  mySSC32.setFrame(homeFrame);
  delay(1000/x);
}

void up_down() {
  int tframe1[] = {homeFrame[0]+400,homeFrame[1],homeFrame[2]+400,homeFrame[3],homeFrame[4]+400,homeFrame[5],homeFrame[6]-400,homeFrame[7],homeFrame[8]-400,homeFrame[9],homeFrame[10]-400,homeFrame[11]};  // Stand Up
  int tframe2[] = {homeFrame[0]-400,homeFrame[1],homeFrame[2]-400,homeFrame[3],homeFrame[4]-400,homeFrame[5],homeFrame[6]+400,homeFrame[7],homeFrame[8]+400,homeFrame[9],homeFrame[10]+400,homeFrame[11]};  // Crouch

  mySSC32.setFrame(tframe1);
  delay(1000/x);
  mySSC32.setFrame(homeFrame);
  delay(1000/x);
  mySSC32.setFrame(tframe2);
  delay(1000/x);
  mySSC32.setFrame(homeFrame);
  delay(1000/x);
  mySSC32.setFrame(tframe1);
  delay(1000/x);
  mySSC32.setFrame(homeFrame);
  delay(1000/x);
  mySSC32.setFrame(tframe2);
  delay(1000/x);
  mySSC32.setFrame(homeFrame);
  delay(1000/x);
}

void ripple_rotate() {
  int tframe1[] = {homeFrame[0],homeFrame[1],homeFrame[2],homeFrame[3],homeFrame[4]-400,homeFrame[5]+200,homeFrame[6],homeFrame[7],homeFrame[8]+400,homeFrame[9]+200,homeFrame[10],homeFrame[11]};  // RF, LC Up+Fwd
  int tframe2[] = {homeFrame[0],homeFrame[1],homeFrame[2],homeFrame[3],homeFrame[4],homeFrame[5]+200,homeFrame[6],homeFrame[7],homeFrame[8],homeFrame[9]+200,homeFrame[10],homeFrame[11]};  // RF, LC Down
  int tframe3[] = {homeFrame[0],homeFrame[1],homeFrame[2]-400,homeFrame[3]+200,homeFrame[4],homeFrame[5],homeFrame[6]+400,homeFrame[7]+200,homeFrame[8],homeFrame[9],homeFrame[10],homeFrame[11]};  // RC, LB Up+Fwd
  int tframe4[] = {homeFrame[0],homeFrame[1],homeFrame[2],homeFrame[3]+200,homeFrame[4],homeFrame[5],homeFrame[6],homeFrame[7]+200,homeFrame[8],homeFrame[9],homeFrame[10],homeFrame[11]};  // RC, LB Down
  int tframe5[] = {homeFrame[0]-400,homeFrame[1]+200,homeFrame[2],homeFrame[3],homeFrame[4],homeFrame[5],homeFrame[6],homeFrame[7],homeFrame[8],homeFrame[9],homeFrame[10]+400,homeFrame[11]+200};  // RB, LF Up+Fwd
  int tframe6[] = {homeFrame[0],homeFrame[1]+200,homeFrame[2],homeFrame[3],homeFrame[4],homeFrame[5],homeFrame[6],homeFrame[7],homeFrame[8],homeFrame[9],homeFrame[10],homeFrame[11]+200};  // RB, LF Down

  mySSC32.setFrame(tframe1);
  delay(1000/x);
  mySSC32.setFrame(tframe2);
  delay(1000/x);
  mySSC32.setFrame(homeFrame);
  delay(1000/x);
  mySSC32.setFrame(tframe3);
  delay(1000/x);
  mySSC32.setFrame(tframe4);
  delay(1000/x);
  mySSC32.setFrame(homeFrame);
  delay(1000/x);
  mySSC32.setFrame(tframe5);
  delay(1000/x);
  mySSC32.setFrame(tframe6);
  delay(1000/x);
  mySSC32.setFrame(homeFrame);
  delay(1000/x);
}
