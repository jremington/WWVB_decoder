# WWVB_decoder
New, more accurate WWVB time signal decoder for Arduino, based on cross correlation with bit templates

Inspired by extracting a WWVB receiver with loop antenna from an Oregon Scientific "radio controlled" weather station display, I looked for Arduino receiver examples.  The following two Github repositories had useful code, so I based this project on those offerings.

// https://github.com/bhall66/WWVB-clock

// https://github.com/ahooper/WWVBClock/blob/master/WWVB7ino

However, I was not happy with the high error rate observed with the primitive procedures used to decode the bit values, so I rewrote that to use cross correlation techniques. The resulting code is significantly more accurate, and under good reception conditions can decode WWVB time signals without single bit errors for hours on end.

Note: WWVB radios differ in the logic level used to represent a "modulated" bit (binary one) and may be either HIGH or LOW. The code in this repository assumes HIGH receiver output = modulated = binary one.  Easy to change in the Interrupt Service Routine for Timer1.

I also added a routine to unambiguously detect the double sync pulse signifying the start of the frame. 

No code for a graphic display is offered, as those change so rapidly that code for any particular display quickly becomes obsolete. And, graphic displays are always a matter of personal taste. 

Example output on Serial: (bit values shown below, marker bit = 2). 

These data were collected about 2000 km from WWVB in Colorado, USA.

WWVB_13 50 Hz new double sync, template cross correlation
SYNCING: valid frames 0/0/0: |49,20|48,21|49,23|49,50|OK

201000110200100000120011001102000100010200010001020001000002UTC 12/27/2021 21:26 L 0 DST 0
201000111200100000120011001102000100010200010001020001000002UTC 12/27/2021 21:27 L 0 DST 0
201001000200100000120011001102000100010200010001020001000002UTC 12/27/2021 21:28 L 0 DST 0
201001001200100000120011001102000100010200010001020001000002UTC 12/27/2021 21:29 L 0 DST 0
201100000200100000120011001102000100010200010001020001000002UTC 12/27/2021 21:30 L 0 DST 0
201100001200100000120011001102000100010200010001020001000002UTC 12/27/2021 21:31 L 0 DST 0
201100010200100000120011001102000100010200010001020001000002UTC 12/27/2021 21:32 L 0 DST 0
201100011200100000120011001102000100010200010001020001000002UTC 12/27/2021 21:33 L 0 DST 0
201100100200100000120011001102000100010200010001020001000002UTC 12/27/2021 21:34 L 0 DST 0
201100101200100000120011001102000100010200010001020001000002UTC 12/27/2021 21:35 L 0 DST 0
201100110200100000120011001102000100010200010001020001000002UTC 12/27/2021 21:36 L 0 DST 0
201100111200100000120011001102000100010200010001020001000002UTC 12/27/2021 21:37 L 0 DST 0
201101000200100000120011001102000100010200010001020001000002UTC 12/27/2021 21:38 L 0 DST 0
201101001200100000120011001102000100010200010001020001000002UTC 12/27/2021 21:39 L 0 DST 0
210000000200100000120011001102000100010200010001020001000002UTC 12/27/2021 21:40 L 0 DST 0
210000001200100000120011001102000100010200010001020001000002UTC 12/27/2021 21:41 L 0 DST 0
210000010200100000120011001102000100010200010001020001000002UTC 12/27/2021 21:42 L 0 DST 0
210000011200100000120011001102000100010200010001020001000002UTC 12/27/2021 21:43 L 0 DST 0
210000100200100000120011001102000100010200010001020001000002UTC 12/27/2021 21:44 L 0 DST 0
210000101200100000120011001102000100010200010001020001000002UTC 12/27/2021 21:45 L 0 DST 0
210000110200100000120011001102000100010200010001020001000002UTC 12/27/2021 21:46 L 0 DST 0
210000111200100000120011001102000100010200010001020001000002UTC 12/27/2021 21:47 L 0 DST 0
210001000200100000120011001102000100010200010001020001000002UTC 12/27/2021 21:48 L 0 DST 0
210001001200100000120011001102000100010200010001020001000002UTC 12/27/2021 21:49 L 0 DST 0
210100000200100000120011001102000100010200010001020001000002UTC 12/27/2021 21:50 L 0 DST 0
210100001200100000120011001102000100010200010001020001000002UTC 12/27/2021 21:51 L 0 DST 0
210100010200100000120011001102000100010200010001020001000002UTC 12/27/2021 21:52 L 0 DST 0
210100011200100000120011001102000100010200010001020001000002UTC 12/27/2021 21:53 L 0 DST 0
210100100200100000120011001102000100010200010001020001000002UTC 12/27/2021 21:54 L 0 DST 0
210100101200100000120011001102000100010200010001020001000002UTC 12/27/2021 21:55 L 0 DST 0
210100110200100000120011001102000100010200010001020001000002UTC 12/27/2021 21:56 L 0 DST 0
210100111200100000120011001102000100010200010001020001000002UTC 12/27/2021 21:57 L 0 DST 0
210101000200100000120011001102000100010200010001020001000002UTC 12/27/2021 21:58 L 0 DST 0
210101001200100000120011001102000100010200010001020001000002UTC 12/27/2021 21:59 L 0 DST 0
200000000200100001020011001102000100010200010001020001000002UTC 12/27/2021 22:00 L 0 DST 0
200000001200100001020011001102000100010200010001020001000002UTC 12/27/2021 22:01 L 0 DST 0
200000010200100001020011001102000100010200010001020001000002UTC 12/27/2021 22:02 L 0 DST 0
200000011200100001020011001102000100010200010001020001000002UTC 12/27/2021 22:03 L 0 DST 0
200000100200100001020011001102000100010200010001020001000002UTC 12/27/2021 22:04 L 0 DST 0
200000101200100001020011001102000100010200010001020001000002UTC 12/27/2021 22:05 L 0 DST 0
200000110200100001020011001102000100010200010001020001000002UTC 12/27/2021 22:06 L 0 DST 0
200000111200100001020011001102000100010200010001020001000002UTC 12/27/2021 22:07 L 0 DST 0
200001000200100001020011001102000100010200010001020001000002UTC 12/27/2021 22:08 L 0 DST 0
200001001200100001020011001102000100010200010001020001000002UTC 12/27/2021 22:09 L 0 DST 0
200100000200100001020011001102000100010200010001020001000002UTC 12/27/2021 22:10 L 0 DST 0
200100001200100001020011001102000100010200010001020001000002UTC 12/27/2021 22:11 L 0 DST 0
200100010200100001020011001102000100010200010001020001000002UTC 12/27/2021 22:12 L 0 DST 0
200100011200100001020011001102000100010200010001020001000002UTC 12/27/2021 22:13 L 0 DST 0
200100100200100001020011001102000100010200010001021001000002UTC 12/27/2029 22:14 L 0 DST 0
200100101200100001020011001102000100010200010001020001000002UTC 12/27/2021 22:15 L 0 DST 0
200100110200100001020011001102000100010200010001020001000002UTC 12/27/2021 22:16 L 0 DST 0
200100111200100001020011001102000100010200010001020001000002UTC 12/27/2021 22:17 L 0 DST 0
200101000200100001020011001102000100010200010001020001000002UTC 12/27/2021 22:18 L 0 DST 0
200101001200100001020011001102000100010200010001020001000002UTC 12/27/2021 22:19 L 0 DST 0
201000000200100001020011001102000100010200010001020001000002UTC 12/27/2021 22:20 L 0 DST 0
201000001200100001020011001102000100010200010001020001000002UTC 12/27/2021 22:21 L 0 DST 0
201000010200100001020011001102000100010200010001020001000002UTC 12/27/2021 22:22 L 0 DST 0
201000011200100001020011001102000100010200010001020001000002UTC 12/27/2021 22:23 L 0 DST 0
201000100200100001020011001102000100010200010001020001000002UTC 12/27/2021 22:24 L 0 DST 0
201000101200100001020011001102000100010200010001020001000002UTC 12/27/2021 22:25 L 0 DST 0
201000110200100001020011001102000100010200010001020001000002UTC 12/27/2021 22:26 L 0 DST 0
201000111200100001020011001102000100010200010001020001000002UTC 12/27/2021 22:27 L 0 DST 0
201001000200100001020011001102000100010200010001020001000002UTC 12/27/2021 22:28 L 0 DST 0
201001001200100001020011001102000100010200010001020001000002UTC 12/27/2021 22:29 L 0 DST 0
201100000200100001020011001102000100010200010001020001000002UTC 12/27/2021 22:30 L 0 DST 0
201100001200100001020011001102000100010200010001020001000002UTC 12/27/2021 22:31 L 0 DST 0
201100010200100001020011001102000100010200010001020001000002UTC 12/27/2021 22:32 L 0 DST 0
201100011200100001020011001102000100010200010001020001000002UTC 12/27/2021 22:33 L 0 DST 0
201100100200100001020011001102000100010200010001020001000002UTC 12/27/2021 22:34 L 0 DST 0
201100101200100001020011001102000100010200010001020001000002UTC 12/27/2021 22:35 L 0 DST 0
201100110200100001020011001102000100010200010001020001000002UTC 12/27/2021 22:36 L 0 DST 0
201100111200100001020011001102000100010200010001020001000002UTC 12/27/2021 22:37 L 0 DST 0
201101000200100001020011001102000100010200010001020001000002UTC 12/27/2021 22:38 L 0 DST 0
201101001200100001020011001102000100010200010001020001000002UTC 12/27/2021 22:39 L 0 DST 0
210000000200100001020011001102000100010200010001020001000002UTC 12/27/2021 22:40 L 0 DST 0
210000001200100001020011001102000100010200010001020001000002UTC 12/27/2021 22:41 L 0 DST 0
210000010200100001020011001102000100010200010001020001000002UTC 12/27/2021 22:42 L 0 DST 0
210000011200100001020011001102000100010200010001020001000002UTC 12/27/2021 22:43 L 0 DST 0
210000100200100001020011001102000100010200010001020001000002UTC 12/27/2021 22:44 L 0 DST 0
210000101200100001020011001102000100010200010001020001000002UTC 12/27/2021 22:45 L 0 DST 0
210000110200100001020011001102000100010200010001020001000002UTC 12/27/2021 22:46 L 0 DST 0
210000111200100001020011001102000100010200010001020001000002UTC 12/27/2021 22:47 L 0 DST 0
210001000200100001020011001102000100010200010001020001000002UTC 12/27/2021 22:48 L 0 DST 0

TODO:  WWVB frames carry no checksum, and when single bit errors do occur, they can be disastrous. No single frame can be trusted without some external or internal verification scheme. Comparing with an RTC reading approximate time is an obvious approach, but and internal verification scheme is needed to automatically set the RTC when required. 

So, code is needed to compare successive frames to identify erroneous bits. A "voting" scheme for relatively slow-changing value fields should work. Ideas welcome!


Below: photo of WWVB receiver module extracted from Oregon Scientific RMR112A weather station indoor module. I constructed a single-transistor level shifting interface to invert the 3V LOW signal to 5V HIGH for input to an Arduino.

![Photo of Oregon Scientific RMR112A WWVB receiver module](https://github.com/jremington/WWVB_decoder/blob/d06a4f027d7323ffc2fceca21fdb95d49aa655cf/RMR112A_WWVB.jpg?raw=true)


Note on unweighted cross correlation for binary bit streams:

The unweighted cross correlation is defined as 
SUM( (x - xave) * (y - yave))
where xave and yave are the average values of the bit streams. Assuming the average value to be 0.5 and scaling up by a factor of 2, the sum of products becomes as simple as counting 1 bits and 0 bits in the X stream that coincide with 1 bits and 0 bits of the Y stream, and subtracting those 1's and 0's that are in the wrong place.

The WWVB bits are defined as "1" for 0.2, 0.5 or 0.8 seconds, followed by "0" for the rest of the one second bit frame. The bit stream templates can be calculated on the fly by simply examining the time index, and making the 1/0 choice dependent on the progression of time.

Thus, the inner loop for cross correlating the received bit stream X with, for example, the MARKER pulse template Y (800 ms "1" followed by 200 ms of "0" becomes simply the following, and is very fast.
```

  val = digitalRead(input);  //get latest sample from radio
  
  bool V0 = (val == 0);   //shortcut for writing the following code
  
  // i is sample index of incoming bitstream, 100 samples per second
  
  if ( i < 80 && val) bitcorr[2]++;               //cross correlation to MARKER template = "1" for samples 0 to 79
  if ( i > 79 && V0)  bitcorr[2]++;              // template = "0" for samples 80 to 99
  if ( i < 80 && V0)  bitcorr[2]--;               //correct for bits in the wrong place
  if ( i > 79 && val) bitcorr[2]--;
  ```
  
  The values of the correlation function range 100 for a perfect match, to -100 for a perfect mismatch.
