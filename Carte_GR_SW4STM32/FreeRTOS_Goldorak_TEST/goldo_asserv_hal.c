#include "goldo_asserv_hal.h"

#include <sys/types.h>
//#include <sys/ioctl.h>

#if 1 /* FIXME : TODO : portage RTOS */
#include "main.h"
#include "stm32f3xx_hal.h"
#include "cmsis_os.h"
#endif /* FIXME : TODO : portage RTOS */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
//#include <debug.h>
#include <string.h>



#if 1 /* FIXME : TODO : portage RTOS */
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
#endif /* FIXME : TODO : portage RTOS */

extern void goldo_maxon2_dir_p(void);
extern void goldo_maxon2_dir_n(void);
extern void goldo_maxon2_en(void);
extern void goldo_maxon2_dis(void);
extern void goldo_maxon2_speed(int32_t s);
extern void goldo_maxon1_dir_p(void);
extern void goldo_maxon1_dir_n(void);
extern void goldo_maxon1_en(void);
extern void goldo_maxon1_dis(void);
extern void goldo_maxon1_speed(int32_t s);


typedef struct goldo_asserv_hal_s
{
  bool initialized;
  int fd_left;
  int fd_right;
  int pwm_left;
  int pwm_right;

} goldo_asserv_hal_s;

goldo_asserv_hal_s g_asserv_hal = {false,-1,-1,0,0};

#if 0 /* FIXME : TODO : portage RTOS */
static int fdL, fdR;
#endif /* FIXME : TODO : portage RTOS */

void goldo_asserv_hal_reset()
{
  g_asserv_hal.initialized = false;
  g_asserv_hal.fd_left = -1;
  g_asserv_hal.fd_right = -1;
  g_asserv_hal.pwm_left = 0;
  g_asserv_hal.pwm_right = 0;
}

static int init_devices(void)
{
#if 0 /* FIXME : TODO : portage RTOS */
  struct pwm_info_s info;
  int ret;
  printf("init goldo_asserv_hal devices\n");
  fdL = open("/dev/pwm1", O_RDONLY);
  if (fdL < 0)
    {
      printf("init_devices: open /dev/pwm1 failed: %d\n", errno);
      goto errout;
    }

  fdR = open("/dev/pwm0", O_RDONLY);
  if (fdR < 0)
    {
      printf("init_devices: open /dev/pwm0 failed: %d\n", errno);
      goto errout;
    }

  info.frequency = 10000;
  info.duty = 0;

  ret = ioctl(fdL,PWMIOC_SETCHARACTERISTICS,(unsigned long)((uintptr_t)&info));
  if (ret < 0)
    {
      printf("init_devices: LEFT : ioctl(PWMIOC_SETCHARACTERISTICS) failed: %d\n", errno);
      goto errout;
    }

  ret = ioctl(fdR,PWMIOC_SETCHARACTERISTICS,(unsigned long)((uintptr_t)&info));
  if (ret < 0)
    {
      printf("init_devices: RIGHT : ioctl(PWMIOC_SETCHARACTERISTICS) failed: %d\n", errno);
      goto errout;
    }

  ret = ioctl(fdL, PWMIOC_START, 0);
  if (ret < 0)
    {
      printf("init_devices: LEFT : ioctl(PWMIOC_START) failed: %d\n", errno);
      goto errout;
    }

  ret = ioctl(fdR, PWMIOC_START, 0);
  if (ret < 0)
    {
      printf("init_devices: RIGHT : ioctl(PWMIOC_START) failed: %d\n", errno);
      goto errout;
    }
   printf("enable left\n");
  return OK;

 errout:
  return GOLDO_ERROR;
#else /* FIXME : TODO : portage RTOS */
  return OK;
#endif /* FIXME : TODO : portage RTOS */
}

static void stop_devices(void)
{
#if 0 /* FIXME : TODO : portage RTOS */
  (void)ioctl(fdL, PWMIOC_STOP, 0);
  (void)ioctl(fdR, PWMIOC_STOP, 0);
  close(fdL);
  close(fdR);
#endif /* FIXME : TODO : portage RTOS */
}

int goldo_asserv_hal_init(void)
{
  if(g_asserv_hal.initialized)
  {
    return OK;
  }
  int ret = init_devices();
  if(ret == OK)
  {
    printf("finish init motors\n");
    g_asserv_hal.initialized = true;
    return OK;
  } else
  {
    g_asserv_hal.initialized = false;
    return GOLDO_ERROR;
  }
}

int goldo_asserv_hal_quit(void)
{
  if(!g_asserv_hal.initialized)
  {
    return GOLDO_ERROR;
  }
  stop_devices();
  return OK;
}

int goldo_asserv_hal_set_motors_enable(bool left, bool right)
{
  if(left)
  {
    goldo_maxon2_en();
  } else
  {
    goldo_maxon2_dis();
  }
  if(right)
  {
    goldo_maxon1_en();
  } else
  {
    goldo_maxon1_dis();
  }
  return OK;
}

int goldo_asserv_hal_set_motors_pwm(int left, int right)
{
  goldo_maxon2_speed(left);
  goldo_maxon1_speed(right);
  return OK;
}


#if 1 /* FIXME : TODO : portage RTOS */

void goldo_maxon2_dir_p(void)
{
  HAL_GPIO_WritePin(MAXON2_DIR_GPIO_Port, MAXON2_DIR_Pin, GPIO_PIN_RESET);
}

void goldo_maxon2_dir_n(void)
{
  HAL_GPIO_WritePin(MAXON2_DIR_GPIO_Port, MAXON2_DIR_Pin, GPIO_PIN_SET);
}

void goldo_maxon2_en(void)
{
  HAL_GPIO_WritePin(MAXON_EN_GPIO_Port, MAXON_EN_Pin, GPIO_PIN_SET);
}

void goldo_maxon2_dis(void)
{
  HAL_GPIO_WritePin(MAXON_EN_GPIO_Port, MAXON_EN_Pin, GPIO_PIN_RESET);
}

void goldo_maxon2_speed(int32_t s)
{
  int32_t abs_s;

  if (s>=0) {
    goldo_maxon2_dir_n();
    abs_s = s;
  } else {
    goldo_maxon2_dir_p();
    abs_s = -s;
  }
  if (abs_s>0xffff) abs_s=0xffff;

  /* set maxon2 duty cycle */
  {
    TIM_OC_InitTypeDef sConfigOC;

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = abs_s;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  }

}

void goldo_maxon1_dir_p(void)
{
  HAL_GPIO_WritePin(MAXON1_DIR_GPIO_Port, MAXON1_DIR_Pin, GPIO_PIN_RESET);
}

void goldo_maxon1_dir_n(void)
{
  HAL_GPIO_WritePin(MAXON1_DIR_GPIO_Port, MAXON1_DIR_Pin, GPIO_PIN_SET);
}

void goldo_maxon1_en(void)
{
  HAL_GPIO_WritePin(MAXON_EN_GPIO_Port, MAXON_EN_Pin, GPIO_PIN_SET);
}

void goldo_maxon1_dis(void)
{
  HAL_GPIO_WritePin(MAXON_EN_GPIO_Port, MAXON_EN_Pin, GPIO_PIN_RESET);
}

void goldo_maxon1_speed(int32_t s)
{
  int32_t abs_s;

  if (s>=0) {
    goldo_maxon1_dir_n();
    abs_s = s;
  } else {
    goldo_maxon1_dir_p();
    abs_s = -s;
  }
  if (abs_s>0xffff) abs_s=0xffff;

  /* set maxon1 duty cycle */
  {
    TIM_OC_InitTypeDef sConfigOC;

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = abs_s;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  }

}


#endif /* FIXME : TODO : portage RTOS */

