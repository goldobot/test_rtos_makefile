/*
 * test_input.c
 *
 *  Created on: 11/02/2018
 *      Author: Goldo
 */

#include "main.h"
#include "stm32f3xx_hal.h"
#include "cmsis_os.h"
#include <string.h>
#include "test_input.h"

extern int __io_getchar(void);

/* StartTestInputTask function */
void StartTestInputTask(void const * argument)
{
  // Decodage des arguments de tache
  /* FIXME : TODO */

  printf ("StartTestInputTask()\n");

  // Boucle infinie
  for(;;) {
    char c = getchar();
    if (c!=0) {
      putchar(c);
      if (c==0x0d) {
        c=0x0a;
        putchar(c);
      }
    }
    osDelay(1);                      // Delai de 1 ms
  }
}
