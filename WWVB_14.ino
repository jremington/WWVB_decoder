// inspired by and large sections taken from
// https://github.com/bhall66/WWVB-clock
// https://github.com/ahooper/WWVBClock/blob/master/WWVB7ino

// major revisions of both by SJR 12/2021
// This version assumes WWVB radio output = HIGH for modulated signal (carrier reduced)
// Uses unweighted cross-correlation of bit samples to identify two successive sync pulses => start of new frame
// Bit types classified by unweighted cross-correlation

// sample rate reduced to 50 Hz
// clock drift problem fixed by properly initializing Timer1
//


#include <TimeLib.h>                               // time functions - install within IDE 

#define RADIO_OUT 7                               //input pin for radio output
#define RADIO_POWER 6

#define NOBIT  4                                   // value for no bit
#define ERRBIT 3                                   // value for an error bit
#define MARKER 2                                   // value for a marker bit
#define HIBIT  1                                   // value for bit '1'
#define LOBIT  0                                   // value for bit '0'

#define SAMPLE_HZ  50                            // must be a factor of 62500: 2, 4, 5, 10, 20, 25, 50, 100, 125
#define SYNCTIMEOUT 300                           // seconds until sync gives up
#define SYNCFREQUENCY 4                          // minutes since last successful decode
#define MIN_SYNC_CORR1 (SAMPLE_HZ-4)              // minimum successful correlation of template with sync pulse
#define MIN_SYNC_CORR2 (SAMPLE_HZ-8)             //  more leeway for second sync pulse
#define UTC  0                                     // Coordinated Universal Time
#define EST -5                                     // Eastern Standard Time
#define CST -6                                     // Central Standard Time
#define MST -7                                     // Mountain Standard Time
#define PST -8                                     // Pacific Standard Time
#define LOCALTIMEZONE PST                          // Set to your own time zone!

char useLocalTime = 0; //UTC for now

// ============ GLOBAL VARIABLES =====================================================

// interrupt shared
volatile byte sampleCounter = 0;                   // samples in current second
volatile byte newBit = NOBIT;                      // current bit
volatile byte val = 0;                   // global sample value for sync routine
volatile byte newval = 0;                // global flag for sync routine
volatile signed char bitcorr[3] = {0};   // bitwise cross correlation values (LOW, HIGH, MARKER)

//WWVB data frame
byte oldBit = NOBIT;                               // previous bit
byte frame[60];                                    // space to hold full minute of bits
int frameIndex = 0;                                // current bit position in frame (0..59)

// crosscorrelation variables
byte samples[SAMPLE_HZ] = {0};           //circular sample buffer for sync routine
byte psample = 0;                        //circular sample buffer start pointer

time_t t = 0;                                      // time that display last updated
time_t goodTime = 0;                               // time of last receiver decode
time_t syncTime = 0;                               // time of last successful sync
byte dst = 0;                                      // daylight saving time flag

// frame statistics
unsigned long F_invalid = 0;  //count frames invalid
unsigned long F_checked = 0;  //count frames analyzed
unsigned long F_total = 0;

// ============ TIMER ROUTINES =====================================================

void initTimer() {                                 // set up 100Hz interrupt
  // Start timer1 for periodic interrupt
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS11);    // 8 prescaler
  OCR1A = (F_CPU / 8 / SAMPLE_HZ) - 1; //Tweak TOP= (40000-1) for more accurate timing?
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  interrupts();
}

void stopTimer() {  // Stop TIMER1
  TCCR1B = 0;
  TIMSK1 &= ~(1 << OCIE1A);  // disable timer compare interrupt
}

// generate perfect 1 second sync pulse for tests. Assumes 50 Hz sample rate
byte getval(void) {
  static byte valno = 0;  //keep track of number of bits returned
  byte retval = 1;
  if (valno > 39) retval = 0;  //>800 ms
  valno++;
  if (valno > 49) valno = 0;  //50 Hz
  return retval;
}

// ISR updates global byte variables newBit, sampleCounter, val, newval
// bitcorr[3] > lo, hi, marker, sums of unweighted cross correlation function.

ISR(TIMER1_COMPA_vect) {  // called every 20mS @50 Hz
  val = digitalRead(RADIO_OUT);                   // new bit from radio
  //  val = getval();                             //for testing sync recognition
  newval = 1;                                      //set flag for cross correlation sync search
  if (sampleCounter == 0) {                       //sampleCounter can be reset by main program
    bitcorr[0] = 0;
    bitcorr[1] = 0;
    bitcorr[2] = 0;
  };
  byte i = sampleCounter;
  bool V0 = (val == 0);                 // shortcut
  if ( i < 10 ) {
    if (val) bitcorr[0]++;               //cross correlation to LOW template
    else bitcorr[0]--;                   //correct for bit in the wrong plac
  }
  if ( i > 9 ) {                       //ideally, zero from now on
    if (V0) bitcorr[0]++;
    else bitcorr[0]--;
  }

  if ( i < 25 ) {
    if (val) bitcorr[1]++;               //cross correlation to HIGH template
    else bitcorr[1]--;                   //correct for bit in the wrong plac
  }
  if ( i > 24 ) {                       //ideally, zero from now on
    if (V0) bitcorr[1]++;
    else bitcorr[1]--;
  }

  if ( i < 40 ) {
    if (val) bitcorr[2]++;               //cross correlation to MARKER template
    else bitcorr[2]--;                   //correct for bit in the wrong plac
  }
  if ( i > 39 ) {                       //ideally, zero from now on
    if (V0) bitcorr[2]++;
    else bitcorr[2]--;
  }

  sampleCounter++;                             // count samples
  if (sampleCounter >= SAMPLE_HZ)                // full second of sampling?
  {
    signed char max = -127, type = 0;
    // find max correlation to bit templates
    for (i = 0; i < 3; i++) {
      if (max < bitcorr[i]) {
        max = bitcorr[i];
        type = i;
      }
    }

    byte retval = ERRBIT;
    if (max > 0) retval = type;   // move this threshold up? (SJR)
    sampleCounter = 0;  // reset sampleCounter
    newBit = retval;     // return the bit identified
  }
}

// ============ WWVB ROUTINES =====================================================
// WWVB reference https://www.nist.gov/sites/default/files/documents/2017/04/28/SP-432-NIST-Time-and-Frequency-Services-2012-02-13.pdf

// symbols below taken from https://github.com/ahooper/WWVBClock/blob/master/WWVB7ino
// Indices for parts of WWVB frame
/*
  enum {
    FPRM, // Frame reference marker: .8L+.2H
    FPUU, // Unweighted: .2L+.8H
    // radio data 1 = .5L+.5H / 0 = .2L+.8H
    FPM1, // 10 minutes
    FPM2, //  1 minutes
    FPH1, // 10 hours
    FPH2, //  1 hours
    FPD1, //100 days
    FPD2, // 10 days
    FPD3, //  1 days
    FPUS, // UTC sign
    FPUC, // UTC correction
    FPY1, // 10 years
    FPY2, //  1 years
    FPLY, // Leap year
    FPLS, // Leap second
    FPDS, // Daylight saving time
    FPEF}; // End of frame same as FPRM
  // Order of received frame, one per second
  const byte FramePattern[] =
    // .0   .1   .2   .3   .4   .5   .6   .7   .8   .9  segment#
  {FPRM,FPM1,FPM1,FPM1,FPUU,FPM2,FPM2,FPM2,FPM2,FPRM, //0
  FPUU,FPUU,FPH1,FPH1,FPUU,FPH2,FPH2,FPH2,FPH2,FPRM, //1
  FPUU,FPUU,FPD1,FPD1,FPUU,FPD2,FPD2,FPD2,FPD2,FPRM, //2
  FPD3,FPD3,FPD3,FPD3,FPUU,FPUU,FPUS,FPUS,FPUS,FPRM, //3
  FPUC,FPUC,FPUC,FPUC,FPUU,FPY1,FPY1,FPY1,FPY1,FPRM, //4
  FPY2,FPY2,FPY2,FPY2,FPUU,FPLY,FPLS,FPDS,FPDS,FPEF}; //5
*/

// now, for frame validation, just classify fixed marker, fixed zeros and variable value bit fields

enum {FPRM, FPUU, FPBV}; //bit data types: MARKER, fixed LOBIT, variable BITVALUE
const byte FramePattern[] =
  // .0   .1   .2   .3   .4   .5   .6   .7   .8   .9
{ FPRM, FPBV, FPBV, FPBV, FPUU, FPBV, FPBV, FPBV, FPBV, FPRM, //0
  FPUU, FPUU, FPBV, FPBV, FPUU, FPBV, FPBV, FPBV, FPBV, FPRM, //1
  FPUU, FPUU, FPBV, FPBV, FPUU, FPBV, FPBV, FPBV, FPBV, FPRM, //2
  FPBV, FPBV, FPBV, FPBV, FPUU, FPUU, FPBV, FPBV, FPBV, FPRM, //3
  FPBV, FPBV, FPBV, FPBV, FPUU, FPBV, FPBV, FPBV, FPBV, FPRM, //4
  FPBV, FPBV, FPBV, FPBV, FPUU, FPBV, FPBV, FPBV, FPBV, FPRM  //5
};

int minutesSinceDecode() {                            // return minutes since time successfully decoded
  return (now() - goodTime) / 60;
}

int minutesSinceSync() {                            // return minutes since time successfully synced
  return (now() - syncTime) / 60;
}

void getRadioTime()                                // decode time from current frame
{
  const byte daysInMonth[] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  //JanFebMarAprMayJunJulAugSepOctNovDec
  const int century = 2000;
  int yr, mo, dy, hr, mn, leap;
  dy = hr = dst = leap = mn = 0;
  yr = century;

  if (frame[1] == HIBIT) mn += 40;                // decode minutes
  if (frame[2] == HIBIT) mn += 20;
  if (frame[3] == HIBIT) mn += 10;
  if (frame[5] == HIBIT) mn += 8;
  if (frame[6] == HIBIT) mn += 4;
  if (frame[7] == HIBIT) mn += 2;
  if (frame[8] == HIBIT) mn += 1;

  if (frame[12] == HIBIT) hr += 20;               // decode hours
  if (frame[13] == HIBIT) hr += 10;
  if (frame[15] == HIBIT) hr += 8;
  if (frame[16] == HIBIT) hr += 4;
  if (frame[17] == HIBIT) hr += 2;
  if (frame[18] == HIBIT) hr += 1;

  if (frame[22] == HIBIT) dy += 200;              // decode days
  if (frame[23] == HIBIT) dy += 100;
  if (frame[25] == HIBIT) dy += 80;
  if (frame[26] == HIBIT) dy += 40;
  if (frame[27] == HIBIT) dy += 20;
  if (frame[28] == HIBIT) dy += 10;
  if (frame[30] == HIBIT) dy += 8;
  if (frame[31] == HIBIT) dy += 4;
  if (frame[32] == HIBIT) dy += 2;
  if (frame[33] == HIBIT) dy += 1;

  if (frame[45] == HIBIT) yr += 80;               // decode years
  if (frame[46] == HIBIT) yr += 40;
  if (frame[47] == HIBIT) yr += 20;
  if (frame[48] == HIBIT) yr += 10;
  if (frame[50] == HIBIT) yr += 8;
  if (frame[51] == HIBIT) yr += 4;
  if (frame[52] == HIBIT) yr += 2;
  if (frame[53] == HIBIT) yr += 1;
  if (frame[55] == HIBIT) leap = 1;              // get leapyear indicator
  if (frame[58] == HIBIT) dst = 1;               // get DST indicator

  mo = 1;                                         // convert day of year to month/day
  while (1) {                                     // for each mon, starting with Jan
    byte dim = daysInMonth[mo];                  // get # of days in this month
    if (mo == 2 && leap == 1) dim += 1;          // adjust for leap year, if necessary
    if (dy <= dim) break;                        // have we reached right month yet?
    dy -= dim;  mo += 1;                         // no, subtract all days in this month
  }

  // data to serial, NOR corrected for WWVB "missing minute"

  Serial.print("UTC ");
  if (mo < 10) Serial.print("0"); Serial.print(mo); // mm/dd/yyyy format
  Serial.print("/");
  if (dy < 10) Serial.print("0"); Serial.print(dy);
  Serial.print("/");  Serial.print(yr); Serial.print(" ");
  Serial.print(hr);
  Serial.print(":"); if (mn < 10) Serial.print("0"); Serial.print(mn);
  Serial.print(" L "); Serial.print(leap); Serial.print(" DST "); Serial.println(dst);

  //  setTime(hr, mn, 0, dy, mo, yr);                 // set the arduino time
  //  adjustTime(61);                                 // adjust for 61 seconds of delay
  //  if (useLocalTime)                               // if using local time:
  //    adjustTime((LOCALTIMEZONE + dst) * 3600L);     // adjust for zone & daylight savings
  goodTime = now();                               // remember when time was decoded.
}

byte checkFrame() {                                // check data validity in current frame
  F_checked++;
  byte errorCount = 0;
  for (int i = 0; i < 60; i++)                      // examine bit type, detects fixed value bit errors only
  {
    if (FramePattern[i] == FPRM && frame[i] != MARKER) errorCount++;  //must be a marker
    if (FramePattern[i] == FPUU && frame[i] != LOBIT)  errorCount++;  //must be a zero bit
    if (FramePattern[i] == FPBV && (frame[i] == MARKER || frame[i] == ERRBIT)) errorCount++; //should be a 1 or 0
  }
  if (errorCount > 0) F_invalid++;
  return errorCount;
}

void startNewFrame()
{
  frameIndex = 0;                                  // start with first bit (bit 0);
}

void checkRadioData() {
  if (newBit != NOBIT) {                           // do nothing until bit received
    if (frameIndex > 59) {
      byte errorCount = checkFrame();
      if (errorCount > 8) doSync();
      if (errorCount == 0) getRadioTime();          // decode time from previous frame
      startNewFrame();
    }
    frame[frameIndex] = newBit;                    // save this bit
    //    showBit(frameIndex);                       // and show it
    //    Serial.write(newBit + '0');                 //ASCII

    frameIndex++;                                  // advance to next bit position
    oldBit = newBit;                               // remember current bit
    newBit = NOBIT;                                // wait until next bit received
  }
}

// new version of sync finds two successive marker pulses by cross correlation
// template is 800 ms "1" and 200 ms "0"
//  constants for 50 Hz sample rate
bool sync() {                                      // return true if sync successful
  unsigned long start_ms = millis();
  bool timedOut = false;

  // fill circular sample buffer with incoming samples
  byte i = 0, j = 0;
  int corr = 0;

  while (1) {
    if (newval) {
      samples[i] = val;
      newval = 0;
      i++;
      if (i >= SAMPLE_HZ) break;
    }
  }

  psample = 0;  //initialize circular buffer pointer
  while (1) {  //loop over successive samples until high correlation

    // calculate cross correlation with template
    corr = 0;
    for (i = 0; i < SAMPLE_HZ; i++) {
      j = i + psample;
      if (j > (SAMPLE_HZ - 1)) j -= SAMPLE_HZ;   //align sample array with sync pulse template
      bool S1 = (samples[j] == 1);
      bool S0 = (samples[j] == 0);          //shorthand
      if ( i < 40 ) {      //1 in template
        if (S1) corr++;    //unweighted cross correlation function
        else corr--;      //subtract bits in the wrong places
      }
      if ( i > 39 ) {       //0 in template
        if (S0) corr++;    //unweighted cross correlation function
        else corr--;      //subtract bits in the wrong places
      }
    }
    if (corr > MIN_SYNC_CORR1) break;  // found one

    timedOut = (millis() - start_ms > (SYNCTIMEOUT * 1000UL));              // give up yet?
    if (timedOut) break;

    //  get new sample
    while (!newval);
    samples[psample] = val; //replace oldest sample stored in circular buffer
    newval = 0;
    psample++;   //start of next frame to test
    if (psample >= SAMPLE_HZ) psample = 0;
  }

  sampleCounter = 0;                             // reset ISR sampleCounter for 1 second
  if (timedOut) return !timedOut;

  Serial.print(corr); Serial.print(",");

  // found a sync pulse
  // get next 50 samples and check for second sync pulse

  for (i = 0; i < SAMPLE_HZ; i++) {
    j = i + psample;
    if (j > (SAMPLE_HZ - 1)) j -= SAMPLE_HZ;   //align sample array with template
    while (!newval); //wait for next sample
    samples[psample] = val; //replace oldest sample stored in circular buffer
    newval = 0;
    psample++;  //start of most recent data frame
    if (psample >= SAMPLE_HZ) psample = 0;
  }

  //Note: measured time to correlate 50 samples ~150 microseconds on 16 MHz Arduino

  corr = 0;
  for (i = 0; i < SAMPLE_HZ; i++) {
    // calculate cross correlation with template
    j = i + psample;
    if (j > (SAMPLE_HZ - 1)) j -= SAMPLE_HZ;   //align sample array with template
    bool S1 = (samples[j] == 1);
    bool S0 = (samples[j] == 0);          //shorthand
    if ( i < 40 ) {
      if (S1) corr++;    //unweighted cross correlation function
      else corr--;      //subtract bits in the wrong places
    }
    if ( i > 39 ) {
      if (S0) corr++;    //unweighted cross correlation function
      else corr--;      //subtract bits in the wrong places
    }
  }
  Serial.print(corr); Serial.print("|");
  if (corr < MIN_SYNC_CORR2) timedOut = true;    //not a second sync pulse

  sampleCounter = 0;                             // reset ISR sampleCounter for 1 second
  return !timedOut;                                // true = sync successful
}

// check if time to sync (minutes since last successful decode)

bool needSync() {
  bool flag = (minutesSinceSync() >= SYNCFREQUENCY);
  return flag;
}

void doSync() {// sync with visual status update
  //  Serial.println();
  Serial.print("SYNCING: valid frames ");
  Serial.print((F_checked - F_invalid));
  Serial.print("/");
  Serial.print(F_checked);
  Serial.print(": |");

  while (!sync());
  Serial.println("OK");
  newBit = MARKER;
  oldBit = MARKER;
  startNewFrame();
  syncTime = now();                               //got two presumed sync pulses, off and running!
}

void updateTimeDisplay(void) {
}

// ============ MAIN PROGRAM ===================================================

void setup() {
  pinMode(RADIO_POWER, OUTPUT);
  digitalWrite(RADIO_POWER, HIGH);
  pinMode(RADIO_OUT, INPUT);
  Serial.begin(115200);
  while (!Serial);
  Serial.println();
  Serial.println("WWVB_14 50 Hz double sync detection, template cross correlation, prescaler 8");

  initTimer();                                     // start the counter
  doSync();
}

void loop() {
  //  if(newval) {Serial.print(val); newval=0;}
  checkRadioData();                               // collect data & update time
  updateTimeDisplay();                            // keep display current
  //  if (needSync()) doSync();                       // re-synchronize if data is stale
}
