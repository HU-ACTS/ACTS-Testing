#include "WifiTask.hpp"
#define mainSOFTWARE_TIMER_PERIOD_MS        pdMS_TO_TICKS( 1000 )
void run_wifi_task(void *args)  {
  while(1);
}
void system_timers_callback( TimerHandle_t xTimer )  {
  //Todo set bit 4
}
WifiTask::WifiTask(unsigned int task_priority, EventGroupHandle_t& egh) : BaseTask(task_priority, egh)  {}

void WifiTask::main_task() {
  TimerHandle_t wifi_poll_timer = NULL;
  wifi_poll_timer = xTimerCreate("wifi_poll_clock", mainSOFTWARE_TIMER_PERIOD_MS * 60 * 10 /* 10 minutes */, pdTRUE, 0, system_timers_callback);
  xTimerStart( wifi_poll_timer, 0 );
  if(wifi_poll_timer == NULL) {
    // Something has failed creating the timer
  }
  TaskHandle_t xHandle = NULL;
  BaseType_t xReturned = xTaskCreatePinnedToCore(run_wifi_task, "wifi_task", 2048, NULL, 2, &xHandle, 0);

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
