#include "goldo_odometry.h"

#if 1 /* FIXME : TODO : portage RTOS */
#include "main.h"
#include "stm32f3xx_hal.h"
#include "cmsis_os.h"
#endif /* FIXME : TODO : portage RTOS */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>

#include <math.h>




#if 0 /* FIXME : TODO : portage RTOS */
static int fd_qe_r,fd_qe_l;
#else /* FIXME : TODO : portage RTOS */
extern TIM_HandleTypeDef htim1; /* DROITE */
extern TIM_HandleTypeDef htim4; /* GAUCHE */
#endif /* FIXME : TODO : portage RTOS */


int goldo_odometry_hal_init(void)
{
#if 0 /* FIXME : TODO : portage RTOS */
	/* Open encoder devices */
   fd_qe_r = open("/dev/qe1", O_RDONLY);
  if (fd_qe_r < 0) 
    {
      printf("init_devices: open %s failed: %d\n", "/dev/qe1", errno);
      goto errout;
    }

  fd_qe_l = open("/dev/qe0", O_RDONLY);
  if (fd_qe_l < 0) 
    {
      printf("init_devices: open %s failed: %d\n", "/dev/qe0", errno);
      goto errout;
    }

 	/* Read initial encoder values */   
    int32_t qe_r;
    int32_t qe_l;
    ioctl(fd_qe_r, QEIOC_POSITION, (unsigned long)((uintptr_t)&qe_r));
    ioctl(fd_qe_l, QEIOC_POSITION, (unsigned long)((uintptr_t)&qe_l));
    g_odometry_state.counts_left = qe_l;
    g_odometry_state.counts_right = qe_r;

    return OK;

    errout:
    	return GOLDO_ERROR;
#else /* FIXME : TODO : portage RTOS */
    /* Read initial encoder values */   
    g_odometry_state.counts_left  = htim4.Instance->CNT;
    g_odometry_state.counts_right = htim1.Instance->CNT;

    return OK;
#endif /* FIXME : TODO : portage RTOS */
}

int goldo_odometry_hal_release(void)
{
#if 0 /* FIXME : TODO : portage RTOS */
  close(fd_qe_l);
  close(fd_qe_r);
#endif /* FIXME : TODO : portage RTOS */
  return OK;
}


int goldo_odometry_hal_read_encoders(int32_t* left, int32_t* right)
{
#if 0 /* FIXME : TODO : portage RTOS */
  ioctl(fd_qe_l, QEIOC_POSITION, (unsigned long)((uintptr_t)left));
  ioctl(fd_qe_r, QEIOC_POSITION, (unsigned long)((uintptr_t)right));
#else /* FIXME : TODO : portage RTOS */
  *right = htim1.Instance->CNT;
  *left  = htim4.Instance->CNT;
#endif /* FIXME : TODO : portage RTOS */
  return OK;
}
