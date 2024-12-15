#ifndef DFPLAYER_H
#define DFPLAYER_H

#include <stdint.h>

void DFPlayer_SendCommand(uint8_t command, uint8_t param1, uint8_t param2);
// Función para inicializar el DFPlayer
void DFPlayer_Init(void);

// Función para reproducir una pista específica
void DFPlayer_PlayTrack(uint16_t track);

// Función para detener la reproducción
void DFPlayer_Stop(void);

// Función para ajustar el volumen
void DFPlayer_SetVolume(uint8_t volume);

#endif
