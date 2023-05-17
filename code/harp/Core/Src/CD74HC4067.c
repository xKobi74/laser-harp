#include "CD74HC4067.h"

Multiplexer *initMultiplexer(GPIO_TypeDef *GPIOx, uint16_t dataS0, uint16_t dataS1, uint16_t dataS2, uint16_t dataS3, uint16_t enablePin, uint16_t signalPin) {
    Multiplexer *mux = malloc(sizeof(struct Multiplexer));
    if (mux == NULL) return NULL;
    mux->GPIOx = GPIOx;
    mux->dataS0 = dataS0;
    mux->dataS1 = dataS1;
    mux->dataS2 = dataS2;
    mux->dataS3 = dataS3;
    mux->enablePin = enablePin;
    mux->signalPin = signalPin;
    mux->selectedChannel = -1;
		
		HAL_GPIO_WritePin(GPIOC, enablePin, 0);
    return mux;
}

void setEnableMultiplexer(Multiplexer *mux, bool isEnabled) {
    if (mux != NULL) {
        if (isEnabled) {
						HAL_GPIO_WritePin(mux->GPIOx, mux->enablePin, GPIO_PIN_RESET);
        } else {
						HAL_GPIO_WritePin(mux->GPIOx, mux->enablePin, GPIO_PIN_SET);
        }
    }
}

void setMultiplexerChannel(Multiplexer *mux, MultiplexerChannel channel) {
    if (mux != NULL && mux->selectedChannel != channel) {
				HAL_GPIO_WritePin(mux->GPIOx, mux->dataS0, ((channel & (uint16_t) 1) == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
				channel >>= 1;
				HAL_GPIO_WritePin(mux->GPIOx, mux->dataS1, ((channel & (uint16_t) 1) == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
				channel >>= 1;
				HAL_GPIO_WritePin(mux->GPIOx, mux->dataS2, ((channel & (uint16_t) 1) == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
				channel >>= 1;
				HAL_GPIO_WritePin(mux->GPIOx, mux->dataS3, ((channel & (uint16_t) 1) == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        mux->selectedChannel = channel;
    }
}

void deleteMultiplexer(Multiplexer *mux) {
    free(mux);
}