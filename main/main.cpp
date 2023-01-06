#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern void setup();
extern void loop();

static void loop_task(void *)
{
    for (;;)
    {
      loop();
      vTaskDelay(10);
    }
}

extern "C" void app_main()
{
    setup();
    xTaskCreate(&loop_task, "loop_task", 3 * 1024, NULL, 2, NULL);
}
