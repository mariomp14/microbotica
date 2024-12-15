#ifndef DFPLAYER_H
#define DFPLAYER_H

#include <stdint.h>

void DFPlayer_SendCommand(uint8_t command, uint8_t param1, uint8_t param2);
// Funci�n para inicializar el DFPlayer
void DFPlayer_Init(void);

// Funci�n para reproducir una pista espec�fica
void DFPlayer_PlayTrack(uint16_t track);

// Funci�n para detener la reproducci�n
void DFPlayer_Stop(void);

// Funci�n para ajustar el volumen
void DFPlayer_SetVolume(uint8_t volume);

#endif
