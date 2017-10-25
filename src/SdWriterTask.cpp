#include "SdWriterTask.hpp"
//typedef  void (SdWriterTask::*run_sd_task)(void *args);

void run_sd_task(void *args) {
    EventGroupHandle_t& egh = (EventGroupHandle_t& )args;
    while(1)  {
        EventBits_t uxBits;
        uxBits = xEventGroupWaitBits(egh, SensorBufferSdReady, pdTRUE, pdFALSE, 0);

        if(uxBits & SensorBufferSdReady){
            // Todo handle writing of buffer to sd card...
            // [&] (simple_test_buffer)    {
            //
            // }
            // for(std::array<int>::iterator it = simple_test_buffer.begin(); it != simple_test_buffer.end(); ++it)    {
            //     printf("%i", *it);
            // }
        }
        else {
            // Should not occur (only SensorBufferSdReady bit has been set)
        }
    }
}

SdWriterTask::SdWriterTask(unsigned int task_priority) : BaseTask(task_priority)  {}

void SdWriterTask::main_task() {
  TaskHandle_t xHandle = NULL;
  void* args[1];
  args[0] = (void *)&egh;
  BaseType_t xReturned = xTaskCreatePinnedToCore(run_sd_task, "run_sd_task", 2048, args, 1, &xHandle, 0);

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
