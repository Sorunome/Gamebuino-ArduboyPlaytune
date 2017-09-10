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
#include <Gamebuino-Meta.h>


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

void gb_updateTones();

class GB_Sound_Handler_Playtune;

Gamebuino_Meta::Sound_Channel* gb_channel[2];
GB_Sound_Handler_Playtune* gb_handler[2] = {0, 0};

class GB_Sound_Handler_Playtune : public Gamebuino_Meta::Sound_Handler {
public:
  int8_t i = -1;
  GB_Sound_Handler_Playtune(uint8_t _i) : Gamebuino_Meta::Sound_Handler(0) {
    i = _i;
  };
  ~GB_Sound_Handler_Playtune() {
    gb_channel[i] = 0;
    gb_handler[i] = 0;
  };
  void update() {
    gb_updateTones();
  }
  void rewind() {
    // do nothing, this is a dummy
  }
  void escapeChannel() {
    gb_channel[i] = channel;
  }
};

ArduboyPlaytune::ArduboyPlaytune(boolean (*outEn)())
{
  outputEnabled = outEn;
}

void ArduboyPlaytune::initChannel(byte pin)
{
  // trash, we already initialized stuff in gb.begin()
  _tune_num_chans = 2; // we only need to do this, lol
  tone_only = true; // so we don't do scores on channel 1, it seems
}

void ArduboyPlaytune::playNote(byte chan, byte note)
{
  byte timer_num;
  byte prescalar_bits;
  unsigned int frequency2; /* frequency times 2 */
  unsigned long ocr;

  // we can't play on a channel that does not exist
  if (chan >= _tune_num_chans) {
    return;
  }

  // if channel 1 is for tones only
  if ((chan == 1) && tone_only) {
    return;
  }

  // we only have frequencies for 128 notes
  if (note > 127) {
    return;
  }
  
  if (note < 48) {
    frequency2 = pgm_read_byte(_midi_byte_note_frequencies + note);
  } else {
    frequency2 = pgm_read_word(_midi_word_note_frequencies + note - 48);
  }
  
  if (!gb_channel[chan]) {
    gb_handler[chan] = new GB_Sound_Handler_Playtune(chan);
    gb.sound.play(gb_handler[chan], true);
    gb_handler[chan]->escapeChannel();
  }
  
  if (!gb_channel[chan]) {
    delete gb_handler[chan];
    gb_handler[chan] = 0;
    return;
  }
  
  gb_channel[chan]->type = Gamebuino_Meta::Sound_Channel_Type::pattern;
  gb_channel[chan]->total = 44100 / frequency2;
  gb_channel[chan]->index = 0;
  gb_channel[chan]->amplitude = 0x30;
  
  gb_channel[chan]->use = true;
}

void ArduboyPlaytune::stopNote(byte chan)
{
  if (gb_channel[chan]) {
    gb_channel[chan]->use = false;
  }
}

void ArduboyPlaytune::playScore(const byte *score)
{
  score_start = score;
  score_cursor = score_start;
  step();  /* execute initial commands */
  tune_playing = true; /* release the interrupt routine */
}

void ArduboyPlaytune::stopScore()
{
  for (uint8_t i = 0; i < _tune_num_chans; i++)
    stopNote(i);
  tune_playing = false;
}

boolean ArduboyPlaytune::playing()
{
  return tune_playing;
}

/* Do score commands until a "wait" is found, or the score is stopped.
This is called initially from playScore(), but then is called
from the interrupt routine when waits expire.

If CMD < 0x80, then the other 7 bits and the next byte are a
15-bit big-endian number of msec to wait
*/
int32_t gb_score_duration = 0;
void ArduboyPlaytune::step()
{
  byte command, opcode, chan;
  unsigned duration;
  
  while (1) {
    command = pgm_read_byte(score_cursor++);
    opcode = command & 0xf0;
    chan = command & 0x0f;
    if (opcode == TUNE_OP_STOPNOTE) { /* stop note */
      stopNote(chan);
    }
    else if (opcode == TUNE_OP_PLAYNOTE) { /* play note */
      all_muted = !outputEnabled();
      playNote(chan, pgm_read_byte(score_cursor++));
    }
    else if (opcode < 0x80) { /* wait count in msec. */
      duration = ((unsigned)command << 8) | (pgm_read_byte(score_cursor++));
      wait_toggle_count = ((unsigned long) wait_timer_frequency2 * duration + 500) / 1000;
      if (wait_toggle_count == 0) wait_toggle_count = 1;
      gb_score_duration = wait_toggle_count;
      break;
    }
    else if (opcode == TUNE_OP_RESTART) { /* restart score */
      score_cursor = score_start;
    }
    else if (opcode == TUNE_OP_STOP) { /* stop score */
      tune_playing = false;
      break;
    }
  }
}

void ArduboyPlaytune::closeChannels()
{
  for (uint8_t i = 0; i < _tune_num_chans; i++)
    stopNote(i);
  _tune_num_chans = 0;
  tune_playing = tone_playing = tone_only = mute_score = false;
}

int32_t gb_tone_duration;
void ArduboyPlaytune::tone(unsigned int frequency, unsigned long duration)
{
  if (!outputEnabled() || _tune_num_chans < 2) {
    return;
  }
  
  tone_playing = true;
  mute_score = tone_mutes_score;
  
  if (!gb_channel[1]) {
    gb_handler[1] = new GB_Sound_Handler_Playtune(1);
    gb.sound.play(gb_handler[1], true);
    gb_handler[1]->escapeChannel();
  }
  
  if (!gb_channel[1]) {
    delete gb_handler[1];
    gb_handler[1] = 0;
    return;
  }
  
  gb_channel[1]->type = Gamebuino_Meta::Sound_Channel_Type::pattern;
  gb_channel[1]->total = 22050 / frequency;
  gb_channel[1]->index = 0;
  gb_channel[1]->amplitude = 0x30;
  
  gb_channel[1]->use = true;
  gb_tone_duration = duration;
}

void ArduboyPlaytune::toneMutesScore(boolean mute)
{
  tone_mutes_score = mute;
}

uint32_t lastFrameUpdate = 0;

void gb_updateTones() {
  if (lastFrameUpdate == gb.frameCount) {
    return;
  }
  lastFrameUpdate = gb.frameCount;
  
  for (uint8_t i = 0; i < _tune_num_chans; i++) {
    if (!gb_channel[i]) {
      ArduboyPlaytune::stopNote(i);
      continue;
    }
  }
  
  // tone update
  if (tone_playing) {
    gb_tone_duration -= gb.getTimePerFrame();
    if (gb_tone_duration <= 0) {
      if (gb_channel[1]) {
        gb_channel[1]->use = false;
      }
      tone_playing = mute_score = false;
    }
  }
  
  // score update
  if (tune_playing) {
    gb_score_duration -= gb.getTimePerFrame();
    if (gb_score_duration <= 0) {
      ArduboyPlaytune::step();
    }
  }
  
  if (mute_score || all_muted) {
    ArduboyPlaytune::stopNote(0);
  }
  if (all_muted) {
    ArduboyPlaytune::stopNote(1);
  }
}
