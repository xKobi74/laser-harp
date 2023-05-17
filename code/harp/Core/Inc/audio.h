#pragma once

#define CHANGE_PARAMETER_UP 1
#define CHANGE_PARAMETER_DOWN 0

typedef struct {
    float C;
    float D;
    float E;
    float F;
    float G;
    float A;
    float B;
} notes;

typedef struct {
    notes ampl;
    notes in_time;
} sound;

void change_octave(const unsigned char change_parameter);

void change_volume(const unsigned char change_parameter);

sound* create_sound();

void wait(sound* N_A, const float time, const float d_time);

void signal(sound* N_A, unsigned char* signals);

void delete_sound(sound* N_A);

float summ_in_time(const sound* N_A);

float summ_ampl(const sound* N_A);