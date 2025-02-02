#include <Arduino.h>
#include "config.h"
#include "audio.h"

bool IsMuted = false;
static void audio_tone(int freqHz, int milliseconds);

static void audio_tone(int freqHz, int milliseconds) {
	uint8_t channel = 0;
#ifdef AUDIO_ENABLE_PIN
	digitalWrite(pinAudioEn, 0);
#endif
	ledcAttachPin(pinAudio, channel);
	ledcWriteTone(channel, freqHz);
	delay(milliseconds);
	audio_off();
	}


void audio_off() {
	uint8_t channel = 0;
#ifdef AUDIO_ENABLE_PIN
	digitalWrite(pinAudioEn, 1);
#endif
	ledcWrite(channel, 0);
	ledcDetachPin(pinAudio);
	pinMode(pinAudio, OUTPUT);
	digitalWrite(pinAudio, 0);
	}


void audio_beep(int freqHz, int onMs, int offMs, int numBeeps) {
	while(numBeeps--) {
		audio_tone(freqHz, onMs);
		delay(offMs);
		}
	}


void audio_set_frequency(int freqHz) {
	if (!IsMuted) {
		uint8_t channel = 0;
		ledcAttachPin(pinAudio, channel);
		if (freqHz > 0) {
#ifdef AUDIO_ENABLE_PIN
			digitalWrite(pinAudioEn, 0);
#endif
			ledcWriteTone(channel, freqHz);	
			}
		else {
			audio_off();
			}
		}
	}


void audio_generate_tone(int freqHz, int ms) {
#ifdef AUDIO_ENABLE_PIN
	digitalWrite(pinAudioEn, 0);
#endif
	audio_tone(freqHz, ms);
#ifdef AUDIO_ENABLE_PIN
	digitalWrite(pinAudioEn, 1);
#endif
	}
