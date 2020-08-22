/*
 * Copyright (c) 2015 by
 Arturo Guadalupi <a.guadalupi@arduino.cc>
 Angelo Scialabba <a.scialabba@arduino.cc>
 Claudio Indellicati <c.indellicati@arduino.cc> <bitron.it@gmail.com>

 * Audio library for Arduino Zero.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#ifndef AUDIOZERO_H
#define AUDIOZERO_H

#include "Arduino.h"
#include <SPIMemory.h>

#define MAX_CLIPS 4

class AudioZeroClass{
public:

	AudioZeroClass() {

		for (uint8_t i; i<MAX_CLIPS; i++) {
			audioClips[i].data = NULL;
			audioClips[i].len = 0;
			audioClips[i].index = 0;
		}
		currentClip = 0; // No clips loaded yet
	};

	void begin(uint32_t sampleRate);
	void addClip(void);
	void removeClip(void);
	void play(File myFile);
	void end();

private:

	void eepromWrite(void);
	void eepromRead(void);

	typedef enum {WAIT, READ_FROM_SERIAL, WRITE_TO_EEPROM} state_t;

	enum {
	  WRITE_EEPROM_SUCCESS,
	  WRITE_EEPROM_ERROR
	};

	void dacConfigure(void);

	void tcConfigure(uint32_t sampleRate);
	bool tcIsSyncing(void);
	void tcStartCounter(void);
	void tcReset(void);
	void tcEnable(void);
	void tcDisable(void);

	typedef struct {
		uint8_t *data;
		uint8_t len;
		uint8_t index;
	} clip_t;

	clip_t *audioClips[MAX_CLIPS];
	uint8_t currentClip;
};

extern AudioZeroClass AudioZero;
#endif
