#include "SdWriterTask.hpp"
//typedef  void (SdWriterTask::*run_sd_task)(void *args);

void run_sd_task(void *args) {

  while(1)  {
    EventBits_t uxBits;
    uxBits = xEventGroupSetBits(xEventGroup, BIT_4);
    if(BIT_4) {
      // Write buffer to sd card
    }

  }
}

SdWriterTask::SdWriterTask(unsigned int task_priority, EventGroupHandle_t& egh) : BaseTask(task_priority, egh)  {}

void SdWriterTask::main_task() {
  TaskHandle_t xHandle = NULL;
  BaseType_t xReturned = xTaskCreatePinnedToCore(run_sd_task, "run_sd_task", 2048, NULL, 1, &xHandle, 0);

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
