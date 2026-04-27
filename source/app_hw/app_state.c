#include "app_state.h"
#include "task_servo_ctrl.h"
#include "task_dc_motor.h"
#include "task_console.h"

static volatile bool g_system_active = false;

void app_state_set_active(bool active)
{
    g_system_active = active;

    task_servo_set_motion_enabled(active);

    if (!active)
    {
        /* Coast motor immediately when system goes OFF */
        dc_motor_message_t msg = { .direction = DC_MOTOR_COAST, .speed_percent = 0 };
        xQueueOverwrite(q_dc_motor, &msg);
    }

    task_print_info("System: %s", active ? "ON" : "OFF");
}

bool app_state_is_active(void)
{
    return g_system_active;
}
