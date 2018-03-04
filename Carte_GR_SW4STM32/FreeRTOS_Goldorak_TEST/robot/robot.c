#include "goldo_robot.h"
#include "../goldo_odometry.h"
#include "../goldo_asserv.h"
#include "../goldo_asserv_hal.h"
#include "../goldo_match_timer.h"

#if 0 /* FIXME : TODO : portage RTOS */
#include <nuttx/config.h>
#include <nuttx/drivers/pwm.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#else /* FIXME : TODO : portage RTOS */
#include "main.h"
#include "stm32f3xx_hal.h"
#include "cmsis_os.h"
#endif /* FIXME : TODO : portage RTOS */


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>


/* Hack */
//Start : PB8
//obstacle PB9


extern int goldo_get_start_gpio_state(void);
extern int goldo_get_obstacle_gpio_state(void);

int goldo_dynamixels_init(void);

int goldo_robot_init(void)
{
  goldo_odometry_config_s odometry_config;

  /* Petit robot */
  #if 0
  odometry_config.dist_per_count_left = -0.4025e-3f ;
  odometry_config.dist_per_count_right = 0.3797e-3f;
  odometry_config.wheel_spacing = 0.313213;
  odometry_config.update_period = 10e-3f;
  #endif
  #if 1 //calib goldo
  odometry_config.dist_per_count_left = -0.383204e-3f ;
  odometry_config.dist_per_count_right = 0.379943e-3f;
  odometry_config.wheel_spacing = 0.307566;
  odometry_config.update_period = 10e-3f;
  #endif

#if 0 /* FIXME : TODO : portage RTOS */
  goldo_match_timer_init();
  goldo_odometry_init();
  goldo_odometry_set_config(&odometry_config); 
  goldo_asserv_init();
  goldo_dynamixels_init();
  goldo_arms_init();
  goldo_adversary_detection_init();
#else /* FIXME : TODO : temporaire */
  goldo_odometry_init();
  goldo_asserv_hal_init();
  goldo_asserv_hal_set_motors_enable(false, false);
  goldo_asserv_hal_set_motors_pwm(0, 0);
#endif /* FIXME : TODO : portage RTOS */


  return OK;
}

int goldo_robot_release(void)
{
#if 0 /* FIXME : TODO : portage RTOS */
  goldo_adversary_detection_release();
  goldo_arms_release();
  goldo_asserv_quit();
  goldo_odometry_quit();
  goldo_match_timer_release();
#endif /* FIXME : TODO : portage RTOS */
  return OK;
  
};

int goldo_robot_wait_for_match_begin(void)
{
#if 0 /* FIXME : TODO : portage RTOS */
  printf("goldo_robot: wait for start of match\n");
  while(!goldo_get_start_gpio_state())
  {
    usleep(10000);
  }
#endif /* FIXME : TODO : portage RTOS */
  return OK;
}

static int s_funny_action_ramp[] = {40000,45000,50000,52500,55000,57500,60000,62500,65000,55000,40000,0,-1};

int goldo_robot_do_funny_action(void)
{
#if 0 /* FIXME : TODO : portage RTOS */
  int ret;
  printf("goldo_robot: do funny action\n");
  /* Open PWM driver */
  struct pwm_info_s info;
  int fd;
  fd = open("/dev/pwm8", O_RDONLY);
  if (fd < 0)
  {
    printf("pwm_main: open %s failed: %d\n", "/dev/pwm8", errno);
    return GOLDO_ERROR;
  }
 
  info.duty = 30000;
  info.frequency = 10000;
  ret = ioctl(fd, PWMIOC_SETCHARACTERISTICS, (unsigned long)((uintptr_t)&info));
  if (ret < 0)
  {
    printf("goldorak_go_ioctl: LEFT : ioctl(PWMIOC_SETCHARACTERISTICS) failed: %d\n", errno);
  }
  ret = ioctl(fd, PWMIOC_START, 0);
  int i=0;
  while(s_funny_action_ramp[i] >0)
  {
    info.duty = s_funny_action_ramp[i];
    ret = ioctl(fd, PWMIOC_SETCHARACTERISTICS, (unsigned long)((uintptr_t)&info));
    usleep(500000);
    i++;
  }
  ret = ioctl(fd, PWMIOC_STOP, 0);
  close(fd);
#endif /* FIXME : TODO : portage RTOS */
  return OK;
}

