#include "pyro_bsp_pwm.h"
#include "FreeRTOS.h"
#include "task.h"

namespace pyro {

extern void buzzer_demo_start();

extern "C" {


    void pyro_init_thread(void *argument) {
        bsp_pwm::init_all();

        buzzer_demo_start();

        vTaskDelete(nullptr);
    }
}
} // namespace pyro