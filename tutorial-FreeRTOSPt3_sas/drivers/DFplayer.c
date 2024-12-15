
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom_map.h"
#include "driverlib/rom.h"
#include "drivers/DFPlayer.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"


void DFPlayer_SendCommand(uint8_t command, uint8_t param1, uint8_t param2) {
    uint8_t packet[10] = {0x7E, 0xFF, 0x06, command, 0x00, param1, param2, 0xEF};
    uint16_t checksum = 0xFFFF - (command + param1 + param2) + 1;
    int i;
    packet[7] = (checksum >> 8) & 0xFF;
    packet[8] = checksum & 0xFF;

    for (i = 0; i < 10; i++) {
        UARTCharPut(UART3_BASE, packet[i]);
    }
}

void DFPlayer_Init(void) {
    DFPlayer_SendCommand(0x06, 0x00, 0x1E); // Configurar volumen al 50%
}

void DFPlayer_PlayTrack(uint16_t track) {
    DFPlayer_SendCommand(0x03, (track >> 8) & 0xFF, track & 0xFF);
}

void DFPlayer_Stop(void) {
    DFPlayer_SendCommand(0x16, 0x00, 0x00);
}

void DFPlayer_SetVolume(uint8_t volume) {
    if (volume > 30) volume = 30; // Límite máximo
    DFPlayer_SendCommand(0x06, 0x00, volume);
}

