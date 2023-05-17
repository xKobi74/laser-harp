#include "audio.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define MAX_OCTAVE_NUMBER 5
#define MIN_OCTAVE_NUMBER 1
//If you want to change this defines change some others in "stm32f1xx_it.c"
#define MAX_VOLUME 100
#define MIN_VOLUME 0
#define VOLUME_JUMP 10

const float pseudo_cos[256] = {1, 0.999849409, 0.999397728, 0.998645228, 0.997592363, 0.996239767, 0.994588255, 0.992638821, 0.99039264, 0.987851065, 0.985015627, 0.981888033, 0.978470168, 0.97476409, 0.970772033, 0.966496399, 0.961939766, 0.957104878, 0.951994647, 0.946612151, 0.940960632, 0.935043496, 0.928864305, 0.922426783, 0.915734806, 0.908792407, 0.901603766, 0.894173214, 0.886505227, 0.878604423, 0.870475563, 0.862123541, 0.853553391, 0.844770272, 0.835779477, 0.826586421, 0.817196642, 0.807615795, 0.797849652, 0.787904096, 0.777785117, 0.76749881, 0.757051372, 0.746449096, 0.735698368, 0.724805665, 0.713777547, 0.702620657, 0.691341716, 0.679947518, 0.668444927, 0.65684087, 0.645142339, 0.633356379, 0.62149009, 0.60955062, 0.597545161, 0.585480944, 0.573365237, 0.561205338, 0.54900857, 0.536782282, 0.524533837, 0.512270614, 0.5, 0.487729386, 0.475466163, 0.463217718, 0.45099143, 0.438794662, 0.426634763, 0.414519056, 0.402454839, 0.39044938, 0.37850991, 0.366643621, 0.354857661, 0.34315913, 0.331555073, 0.320052482, 0.308658284, 0.297379343, 0.286222453, 0.275194335, 0.264301632, 0.253550904, 0.242948628, 0.23250119, 0.222214883, 0.212095904, 0.202150348, 0.192384205, 0.182803358, 0.173413579, 0.164220523, 0.155229728, 0.146446609, 0.137876459, 0.129524437, 0.121395577, 0.113494773, 0.105826786, 0.098396234, 0.091207593, 0.084265194, 0.077573217, 0.071135695, 0.064956504, 0.059039368, 0.053387849, 0.048005353, 0.042895122, 0.038060234, 0.033503601, 0.029227967, 0.02523591, 0.021529832, 0.018111967, 0.014984373, 0.012148935, 0.00960736, 0.007361179, 0.005411745, 0.003760233, 0.002407637, 0.001354772, 0.000602272, 0.000150591, 0, 0.000150591, 0.000602272, 0.001354772, 0.002407637, 0.003760233, 0.005411745, 0.007361179, 0.00960736, 0.012148935, 0.014984373, 0.018111967, 0.021529832, 0.02523591, 0.029227967, 0.033503601, 0.038060234, 0.042895122, 0.048005353, 0.053387849, 0.059039368, 0.064956504, 0.071135695, 0.077573217, 0.084265194, 0.091207593, 0.098396234, 0.105826786, 0.113494773, 0.121395577, 0.129524437, 0.137876459, 0.146446609, 0.155229728, 0.164220523, 0.173413579, 0.182803358, 0.192384205, 0.202150348, 0.212095904, 0.222214883, 0.23250119, 0.242948628, 0.253550904, 0.264301632, 0.275194335, 0.286222453, 0.297379343, 0.308658284, 0.320052482, 0.331555073, 0.34315913, 0.354857661, 0.366643621, 0.37850991, 0.39044938, 0.402454839, 0.414519056, 0.426634763, 0.438794662, 0.45099143, 0.463217718, 0.475466163, 0.487729386, 0.5, 0.512270614, 0.524533837, 0.536782282, 0.54900857, 0.561205338, 0.573365237, 0.585480944, 0.597545161, 0.60955062, 0.62149009, 0.633356379, 0.645142339, 0.65684087, 0.668444927, 0.679947518, 0.691341716, 0.702620657, 0.713777547, 0.724805665, 0.735698368, 0.746449096, 0.757051372, 0.76749881, 0.777785117, 0.787904096, 0.797849652, 0.807615795, 0.817196642, 0.826586421, 0.835779477, 0.844770272, 0.853553391, 0.862123541, 0.870475563, 0.878604423, 0.886505227, 0.894173214, 0.901603766, 0.908792407, 0.915734806, 0.922426783, 0.928864305, 0.935043496, 0.940960632, 0.946612151, 0.951994647, 0.957104878, 0.961939766, 0.966496399, 0.970772033, 0.97476409, 0.978470168, 0.981888033, 0.985015627, 0.987851065, 0.99039264, 0.992638821, 0.994588255, 0.996239767, 0.997592363, 0.998645228, 0.999397728, 0.999849409};
const char number_of_notes = 7;

unsigned int freq_notes[7] = {260, 294, 330, 350, 392, 440, 494};

unsigned char octave_number = 1;
unsigned char volume = 100;

void change_octave(const unsigned char change_parameter) {
		if (change_parameter == CHANGE_PARAMETER_UP && octave_number < MAX_OCTAVE_NUMBER) {
			++octave_number;
			for (unsigned char i = 0; i < number_of_notes; i++) {
				freq_notes[i] <<= 1;
			}
		}
		else if (change_parameter == CHANGE_PARAMETER_DOWN && octave_number > MIN_OCTAVE_NUMBER) {
			--octave_number;
			for (unsigned char i = 0; i < number_of_notes; i++) {
				freq_notes[i] >>= 1;
			}
		}
}

void change_volume(const unsigned char change_parameter) {
		if (change_parameter == CHANGE_PARAMETER_UP && volume < MAX_VOLUME) {
			volume += VOLUME_JUMP;
		}
		else if (change_parameter == CHANGE_PARAMETER_DOWN && volume > MIN_VOLUME) {
			volume -= VOLUME_JUMP;
		}
}

sound* create_sound() {
    sound* N_A = (sound*)malloc(sizeof(sound));
    *N_A = (sound){{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}};
    return N_A;
}

void wait(sound* N_A, const float time, const float d_time) {
    float attenuation = powf(500.0f, d_time);
    for (int i = 0; i < number_of_notes; i++) {
        float* ptr_ampl = ((float*)(&(N_A->ampl)) + i);
        *ptr_ampl /= attenuation;

        float* ptr_in_time = ((float*)(&(N_A->in_time)) + i);
        *ptr_in_time = (*ptr_ampl) * pseudo_cos[(int)(256.0f * (float)freq_notes[i] * fmodf(time, 1.0f / (float)freq_notes[i]))];
    }
}

void signal(sound* N_A, unsigned char* signals) {
    for (int i = 0; i < number_of_notes; i++) {
        if (signals[i] != 0) {
            float *ptr_in_time = ((float*)(&(N_A->in_time)) + i);
            float *ptr_ampl = ((float*)(&(N_A->ampl)) + i);
            *ptr_in_time /= *ptr_ampl;
            if (*ptr_in_time > 1.0f) {*ptr_in_time = 1.0f;}
            *ptr_ampl = 1.0f;
        }
				signals[i] = 0;
    }
}

void delete_sound(sound* N_A) {
    free(N_A);
}

float summ_in_time(const sound* N_A) {
    float summ_ = 0.0f;
    for (int i = 0; i < number_of_notes; i++) {
        summ_ += *((float*)(&(N_A->in_time)) + i);
    }
    return (((float)volume/100.0f)*summ_)/(float)number_of_notes;
}

float summ_ampl(const sound* N_A) {
    float summ_ = 0.0f;
    for (int i = 0; i < number_of_notes; i++) {
        summ_ += *((float*)(&(N_A->ampl)) + i);
    }
    return summ_/(float)number_of_notes;
}