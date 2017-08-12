/**
 * @file ArduboyPlaytune.cpp
 * \brief An Arduino library that plays a one or two part musical score and
 * generates tones. Intended for the Arduboy game system.
 */

/*****************************************************************************
* ArduboyPlaytune
*
* Plays a one or two part musical score and generates tones.
*
* Derived from:
* Playtune: An Arduino tune player library
* https://github.com/LenShustek/arduino-playtune
*
* Modified to work well with the Arduboy game system
* https://www.arduboy.com/
*
* The MIT License (MIT)
*
* (C) Copyright 2016, Chris J. Martinez, Kevin Bates, Josh Goebel, Scott Allen
* Based on work (C) Copyright 2011, 2015, Len Shustek
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* This was inspired by and adapted from "A Tone Generator Library",
* written by Brett Hagman, http://www.roguerobotics.com/
*
*****************************************************************************/

#include "Gamebuino-ArduboyPlaytune.h"


static const byte tune_pin_to_timer[] = { 3, 1 };
static volatile byte *_tunes_timer1_pin_port;
static volatile byte _tunes_timer1_pin_mask;
static volatile int32_t timer1_toggle_count;
static volatile byte *_tunes_timer3_pin_port;
static volatile byte _tunes_timer3_pin_mask;
static byte _tune_pins[AVAILABLE_TIMERS];
static byte _tune_num_chans = 0;
static volatile boolean tune_playing = false; // is the score still playing?
static volatile unsigned wait_timer_frequency2;       /* its current frequency */
static volatile boolean wait_timer_playing = false;   /* is it currently playing a note? */
static volatile unsigned long wait_toggle_count;      /* countdown score waits */
static volatile boolean all_muted = false; // indicates all sound is muted
static volatile boolean tone_playing = false;
static volatile boolean tone_mutes_score = false;
static volatile boolean tone_only = false; // indicates don't play score on tone channel
static volatile boolean mute_score = false; // indicates tone playing so mute other channels

// pointer to a function that indicates if sound is enabled
static boolean (*outputEnabled)();

// pointers to your musical score and your position in said score
static volatile const byte *score_start = 0;
static volatile const byte *score_cursor = 0;

// Table of midi note frequencies * 2
//   They are times 2 for greater accuracy, yet still fits in a word.
//   Generated from Excel by =ROUND(2*440/32*(2^((x-9)/12)),0) for 0<x<128
// The lowest notes might not work, depending on the Arduino clock frequency
// Ref: http://www.phy.mtu.edu/~suits/notefreqs.html
const uint8_t _midi_byte_note_frequencies[48] PROGMEM = {
16,17,18,19,21,22,23,24,26,28,29,31,33,35,37,39,41,44,46,49,52,55,58,62,65,
69,73,78,82,87,92,98,104,110,117,123,131,139,147,156,165,175,185,196,208,220,
233,247
};
const unsigned int _midi_word_note_frequencies[80] PROGMEM = {
262,277,294,311,330,349,370,392,415,440,466,494,523,554,587,622,659,
698,740,784,831,880,932,988,1047,1109,1175,1245,1319,1397,1480,1568,1661,1760,
1865,1976,2093,2217,2349,2489,2637,2794,2960,3136,3322,3520,3729,3951,4186,
4435,4699,4978,5274,5588,5920,6272,6645,7040,7459,7902,8372,8870,9397,9956,
10548,11175,11840,12544,13290,14080,14917,15804,16744,17740,18795,19912,21096,
22351,23680,25088
};

ArduboyPlaytune::ArduboyPlaytune(boolean (*outEn)())
{
  outputEnabled = outEn;
}

void ArduboyPlaytune::initChannel(byte pin)
{
  // trash
}

void ArduboyPlaytune::playNote(byte chan, byte note)
{
  // trash
}

void ArduboyPlaytune::stopNote(byte chan)
{
  // trash
}

void ArduboyPlaytune::playScore(const byte *score)
{
  // trash
}

void ArduboyPlaytune::stopScore()
{
  // trash
}

boolean ArduboyPlaytune::playing()
{
  return false; // trash
}

/* Do score commands until a "wait" is found, or the score is stopped.
This is called initially from playScore(), but then is called
from the interrupt routine when waits expire.

If CMD < 0x80, then the other 7 bits and the next byte are a
15-bit big-endian number of msec to wait
*/
void ArduboyPlaytune::step()
{
  // trash
}

void ArduboyPlaytune::closeChannels()
{
  // trash
}

void ArduboyPlaytune::tone(unsigned int frequency, unsigned long duration)
{
  // trash
}

void ArduboyPlaytune::toneMutesScore(boolean mute)
{
  // trash
}
