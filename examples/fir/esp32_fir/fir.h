#ifndef __FIR_H
#define	__FIR_H

/* Exported includes -------------------*/
#include <Arduino.h>

/* Exported defines --------------------*/

/* Exported macros ---------------------*/

/* Exported typedefs -------------------*/
typedef struct {
  float *h;         //<! Respuesta al impulso
  uint32_t order;   //<! Orden del filtro
  float *xbuf;      //<! Buffer de valores pasados de la señal x(n)
} fir_t;

/* Exported classes --------------------*/

/* Exported functions ------------------*/
/* 
 * @name    fir_init
 * @brief   This function inititialize a fir filter structure
 *
 * @param   fir, structure that contains the definition of fir filter
 * @param   h, array containing the impulse response coefficients
 * @param   order, number of taps of the filter (how many coefficients)
 *
 * @return  If the fir structure was succesfully created
*/
bool fir_init(fir_t *fir, const float *h, uint32_t taps);

/* 
 * @name    fir_step
 * @brief   This function returns the output of a fir filter
 *
 * @param   fir, structure that contains the definition of fir filter
 * @param   x, current sample of the input signal x(n)
 *
 * @return  None
*/
float fir_step(fir_t *fir, float x);

/* 
 * @name    fir_deinit
 * @brief   This function deinitialize the fir structure (free memory)
 *
 * @param   fir, structure that contains the definition of fir filter
 *
 * @return  None
*/
void fir_deinit(fir_t *fir);


#endif /* __FIR_H */
