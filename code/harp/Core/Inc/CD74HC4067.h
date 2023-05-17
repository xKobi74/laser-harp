#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "main.h"

typedef enum MultiplexerChannel {
    MUX_CHANNEL_0,
    MUX_CHANNEL_1,
    MUX_CHANNEL_2,
    MUX_CHANNEL_3,
    MUX_CHANNEL_4,
    MUX_CHANNEL_5,
    MUX_CHANNEL_6,
    MUX_CHANNEL_7,
    MUX_CHANNEL_8,
    MUX_CHANNEL_9,
    MUX_CHANNEL_10,
    MUX_CHANNEL_11,
    MUX_CHANNEL_12,
    MUX_CHANNEL_13,
    MUX_CHANNEL_14,
    MUX_CHANNEL_15,
} MultiplexerChannel;

typedef enum MultiplexerValue {
    MUX_HIGH,
    MUX_LOW
} MultiplexerValue;

typedef struct Multiplexer {
    GPIO_TypeDef *GPIOx;
    uint16_t dataS0;
    uint16_t dataS1;
    uint16_t dataS2;
    uint16_t dataS3;
    uint16_t enablePin;
    uint16_t signalPin;
    MultiplexerChannel selectedChannel;
} Multiplexer;


Multiplexer *initMultiplexer(GPIO_TypeDef *GPIOx, uint16_t dataS0, uint16_t dataS1, uint16_t dataS2, uint16_t dataS3, uint16_t enablePin, uint16_t signalPin);

void setEnableMultiplexer(Multiplexer *mux, bool isEnabled);
void setMultiplexerChannel(Multiplexer *mux, MultiplexerChannel channel);
void deleteMultiplexer(Multiplexer *mux);
