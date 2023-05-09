#include "fir.h"
#include <stdlib.h>

bool fir_init(fir_t *fir, const float *h, uint32_t taps) {
  // Al menos debe de haber un retraso
  if(taps<2) return false;

  // Reservar memoria para los filtros
  fir->order = taps-1;
  fir->h = (float *)malloc(sizeof(float)*taps);
  fir->xbuf = (float *)calloc(taps-1, sizeof(float)); // Calloc hace que los elementos sean inicializados en 0
  fir->order = taps-1;

  // Comprobar si se reservo correctamente la memoria
  if(fir->h == NULL || fir->xbuf == NULL) {
    fir_deinit(fir);
    return false;
  }
  
  // Copiar la respuesta al impulso
  memcpy(fir->h, h, taps);

  return true;
}

float fir_step(fir_t *fir, float x) {
  // Aplicar convolución
  // y(n) = h(0)*x(n)
  float y = fir->h[0]*x;
  for(int k = 0; k < fir->order; k++) {
    // y(n) += h(k+1)*x(n-k)
    y += fir->h[k+1]*fir->xbuf[k];
  }

  // Actualizar los valores pasados de x(n)
  for(int k = fir->order-1; k > 0; k--) {
    fir->xbuf[k] = fir->xbuf[k-1];
  }
  fir->xbuf[0] = x;

  return y;
}

void fir_deinit(fir_t *fir) {
  if(fir->h) free(fir->h);
  if(fir->xbuf) free(fir->xbuf);
  fir->order = 0;
}