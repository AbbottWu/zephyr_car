#include "Motor.h"
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

static void motor_test_cmd_handler(const struct shell *sh, size_t argc,
                                   char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    ARG_UNUSED(sh);
    set_disp_pulse(1);
    set_speed(0, 0, 0 ,0);
    k_sleep(K_SECONDS(2));
    set_speed(500, 500, 500 ,500);
    k_sleep(K_SECONDS(2));
    set_speed(1000, 1000, 1000 ,1000);
    k_sleep(K_SECONDS(2));
    set_speed(2000, 2000, 2000 ,2000);
    k_sleep(K_SECONDS(2));
    set_speed(0, 0, 0 ,0);
    k_sleep(K_SECONDS(2));
    set_speed(-500, -500, -500 ,-500);
    k_sleep(K_SECONDS(2));
    set_speed(-1000, -1000, -1000 ,-1000);
    k_sleep(K_SECONDS(2));
    set_speed(-2000, -2000, -2000 ,-2000);
    k_sleep(K_SECONDS(2));
    set_speed(0, 0, 0 ,0);
    set_disp_pulse(0);
}

SHELL_CMD_REGISTER(motor_test, NULL, "Motor test", motor_test_cmd_handler);
