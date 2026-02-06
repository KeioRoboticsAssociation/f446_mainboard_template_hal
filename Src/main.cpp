#include "main.hpp"
#include "microros_core.h"
#include "microros_app.h"

// ---- HALのオブジェクト ----
extern UART_HandleTypeDef huart2;

// GPIO割り込み
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    
}


void setup() {
    microros_core_setup(&huart2);

}

void loop() {
    microros_core_loop();
    microros_app_loop(microros_core_get());

    // Throttle printf to avoid saturating the micro-ROS serial transport.
    static uint32_t last_print_ms = 0;
    uint32_t now = HAL_GetTick();
    if ((now - last_print_ms) >= 1000U) {
        last_print_ms = now;
        printf("Hello micro-ROS!\n");
    }

    HAL_Delay(1);
}
