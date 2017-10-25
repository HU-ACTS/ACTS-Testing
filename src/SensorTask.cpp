#include "SensorTask.hpp"
#define Timer_100hz        pdMS_TO_TICKS( 10 )
SensorTask::SensorTask(unsigned int task_priority) : BaseTask(task_priority)  {}

void sensor_handle_task(void *args)  {
    while(1)    {
        EventBits_t uxBits;
        uxBits = xEventGroupWaitBits(egh, SensorMeasurementFlag, pdTRUE, pdFALSE, 0);
        if(uxBits & SensorMeasurementFlag)  {
            // SensorMeasurementFlag has been set
        }
    }
}
void set_sensor_measurement_bit( TimerHandle_t xTimer )  {
    //Todo set bit 4
    xEventGroupSetBits(egh, SensorMeasurementFlag);
}

void SensorTask::main_task() {
    TimerHandle_t wifi_poll_timer = NULL;
    wifi_poll_timer = xTimerCreate("sensor_poll_clock", Timer_100hz, pdTRUE, 0,
    set_sensor_measurement_bit
    );
    xTimerStart( wifi_poll_timer, 0 );
    if(wifi_poll_timer == NULL) {
      // Something has failed creating the timer
    }
    TaskHandle_t xHandle = NULL;
    BaseType_t xReturned = xTaskCreatePinnedToCore(sensor_handle_task, "sensor_task", 2048, NULL, 2, &xHandle, 0);

    if(xHandle != NULL) {
      // xHandle works
    } else {
      // Handle assignment has failed
    }

    if(xReturned == pdPASS) {
      // Task creation succesful
    }
    else  {
      // xReturned false (something went wrong!)
    }
}
