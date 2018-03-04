/****************************************************************************
 *  Porte depuis $(NUTTX_APPS)/examples/goldorak_go/goldorak_go_main.c
 *
 ****************************************************************************/

/*

Cote carte : 1-Vert 2-Noir 3-Jaun

V N J : KO+ OK-
N V J : KO!
J N V : KO!
N J V : OK+ OK-
J V N : OK+ KO-
V J N : KO!

Cote moteur : 1-Gris 2-Maro 3-Jaun

Ce qui marche : (carte-moteur)
 2-Noir-Gris-1 3-Jaun-Maro-2 1-Vert-Jaun-3
 
#define SEUIL_FRICTION_BRUSHLESS_1P 14600 // 15720 16260 15780 16020 // 15720
#define SEUIL_FRICTION_BRUSHLESS_1N 14500 // 15720 16440 16080 17340 // 16395
#define SEUIL_FRICTION_BRUSHLESS_2P 17000 // 14700 15540 14940 15660 // 15210
#define SEUIL_FRICTION_BRUSHLESS_2N 14500 // 15000 15660 16020 16080 // 15690
#define SEUIL_PROTECT_BRUSHLESS 60000

 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "main.h"
#include "stm32f3xx_hal.h"
#include "cmsis_os.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

//typedef unsigned int wint_t;
//#include <math.h>

#include "robot/goldo_robot.h"
#include "goldo_odometry.h"
#include "goldo_main_loop.h"
#include "goldo_asserv_hal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int run_mode=0;
/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * thread_asserv
 ****************************************************************************/

/* FIXME : TODO : mettre tout ce bordel dans asserv.c */


void my_exit()
{
  while(1) {
    osDelay(1);
  }
}


/****************************************************************************
 * Name: goldorak_go_help
 ****************************************************************************/

static void goldorak_go_help(void)
{
  printf("Usage: \n");
  printf("       h\n");
  printf("          shows this message and exits\n");
  printf("       match\n");
  printf("          begins a true match (90 sec)\n");
  printf("       homologation\n");
  printf("          does the homologation sequence\n");
  printf("       test_odometry\n");
  printf("          displays the output of the quadrature encoders\n");
  printf("       test_motors\n");
  printf("          does a simple open loop motor test (w/o PID regulation)\n");
  printf("       test_asserv\n");
  printf("          does a true motricity test (with PID regulation)\n");
  printf("       test_dynamixels\n");
  printf("          does a dynamixel servo test\n");
  printf("       test_arms\n");
  printf("          tests the arms (?)\n");
  printf("       test_match\n");
  printf("          ?\n");
  printf("       utest_start_match\n");
  printf("          ?\n");
  printf("       utest_adversary_detection\n");
  printf("          ?\n");
  printf("       utest_match_timer\n");
  printf("          ?\n");
  printf("       utest_funny_action\n");
  printf("          ?\n");
}

/****************************************************************************
 * Name: parse_args
 ****************************************************************************/

static void parse_args(int argc, char **argv)
{
  char *ptr;

  if (argc!=1) {
    goldorak_go_help();
    my_exit();
  }

  ptr = argv[0];

  printf("parse_args(%d,%s)\n", argc, ptr);

  if ((ptr[0]=='h')) {
    goldorak_go_help();
    my_exit();
  }
  if (strcmp(ptr,"match")==0)
  {
    run_mode = GOLDO_MODE_MATCH;
    return;
  }
  if (strcmp(ptr,"homologation")==0)
  {
    run_mode = GOLDO_MODE_HOMOLOGATION;
    return;
  }
  if (strcmp(ptr,"test_odometry")==0)
  {
    run_mode = GOLDO_MODE_TEST_ODOMETRY;
    return;
  }
  if (strcmp(ptr,"test_motors")==0)
  {
    run_mode = GOLDO_MODE_TEST_MOTORS;
    return;
  }
  if (strcmp(ptr,"test_asserv")==0)
  {
    run_mode = GOLDO_MODE_TEST_ASSERV;
    return;
  }
  if (strcmp(ptr,"test_dynamixels")==0)
  {
    run_mode = GOLDO_MODE_TEST_DYNAMIXELS;
    return;
  }
  if (strcmp(ptr,"test_arms")==0)
  {
    run_mode = GOLDO_MODE_TEST_ARMS;
    return;
  }
   if (strcmp(ptr,"test_match")==0)
  {
    run_mode = GOLDO_MODE_TEST_MATCH;
    return;
  }
  if (strcmp(ptr,"utest_start_match")==0)
  {
    run_mode = GOLDO_MODE_UTEST_START_MATCH;
    return;
  }
  if (strcmp(ptr,"utest_adversary_detection")==0)
  {
    run_mode = GOLDO_MODE_UTEST_ADVERSARY_DETECTION;
    return;
  }
  if (strcmp(ptr,"utest_match_timer")==0)
  {
    run_mode = GOLDO_MODE_UTEST_MATCH_TIMER;
    return;
  }
  if (strcmp(ptr,"utest_funny_action")==0)
  {
    run_mode = GOLDO_MODE_UTEST_FUNNY_ACTION;
    return;
  }
}


/****************************************************************************
 * Public Functions
 ****************************************************************************/
extern int __io_getchar(void);
#if 1 /* FIXME : TODO : portage RTOS */
#define MY_READLINE_BUF_SIZE 128
char my_readline_buf[MY_READLINE_BUF_SIZE];
void my_readline(char *input_buf, int len)
{
  int i;
  char *buf_p;
  //putchar('D');
  for (i=0;i<16;i++) { /* FIXME : TODO : quelle taille la FIFO RX des STM? */
    __io_getchar();
  }
  if (len>MY_READLINE_BUF_SIZE) len=MY_READLINE_BUF_SIZE;
  buf_p = input_buf;
  for(;;) {
    char c = __io_getchar();
    if (c!=0) {
      putchar(c);
      if (c==0x0d) {
        c=0x0a;
        putchar(c);
        *buf_p++ = 0;
        len--;
        break;
      } else {
        *buf_p++ = c;
        len--;
        if (len==0) break;
      }
    }
    osDelay(10);                      // Delai de 1 ms
  }
}
#endif /* FIXME : TODO : portage RTOS */

/****************************************************************************
 * Name: goldorak_go_main
 ****************************************************************************/
void goldorak_go_main(void const * argument)
{
  // int ret;
  char arg1_buf[128];
  char *arg1_buf_p;

  printf("goldorak_go_main()\n");

  goldo_asserv_hal_reset();

  /* Parse the command line */
  printf("Command:");
#if 0 /* FIXME : TODO : portage RTOS */
  gets(arg1_buf);
#else /* FIXME : TODO : portage RTOS */
  my_readline(arg1_buf, 64);
#endif /* FIXME : TODO : portage RTOS */
  printf("%s\n",arg1_buf);
  arg1_buf_p = arg1_buf;
  parse_args(1, &arg1_buf_p);

  goldo_robot_init();
  switch(run_mode)
  {
    case GOLDO_MODE_MATCH:
      printf(" main_loop_match()\n");
      //main_loop_match();
      break;
    case GOLDO_MODE_HOMOLOGATION:
      printf(" main_loop_homologation()\n");
      //main_loop_homologation();
      break;
    case GOLDO_MODE_TEST_ODOMETRY:
      printf(" main_loop_test_odometry()\n");
      main_loop_test_odometry();
      break;
    case GOLDO_MODE_TEST_ASSERV:
      printf(" main_loop_test_asserv()\n");
      //main_loop_test_asserv();
      break;
    case GOLDO_MODE_TEST_MOTORS:
      printf(" main_loop_test_motors()\n");
      main_loop_test_motors();
      break;
    case GOLDO_MODE_TEST_DYNAMIXELS:
      printf(" main_loop_test_dynamixels()\n");
      //main_loop_test_dynamixels();
      break;
    case GOLDO_MODE_TEST_ARMS:
      printf(" main_loop_test_arms()\n");
      //main_loop_test_arms();
      break;
     case GOLDO_MODE_TEST_MATCH:
      printf(" main_loop_test_match()\n");
      //main_loop_test_match();
      break;
    case GOLDO_MODE_UTEST_START_MATCH:
      printf(" main_loop_utest_start_match()\n");
      //main_loop_utest_start_match();
      break;
    case GOLDO_MODE_UTEST_ADVERSARY_DETECTION:
      printf(" main_loop_utest_adversary_detection()\n");
      //main_loop_utest_adversary_detection();
      break;
    case GOLDO_MODE_UTEST_MATCH_TIMER:
      printf(" main_loop_utest_match_timer()\n");
      //main_loop_utest_match_timer();
      break;
    case GOLDO_MODE_UTEST_FUNNY_ACTION:
      printf(" main_loop_utest_funny_action()\n");
      //main_loop_utest_funny_action();
      break;
  }
  goldo_robot_release();
  printf("End of main\n");

#if 1 /* FIXME : TODO : portage RTOS */
  my_exit();
#endif /* FIXME : TODO : portage RTOS */

}
