#ifndef BASE_TASK_HPP
#define BASE_TASK_HPP

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#define BIT_0	( 1 << 0 )
#define BIT_4	( 1 << 4 )

#define WIFI_PRIO 1 // priority is lower than sd writer because sd writer could possibly otherwise not write correctly
#define SD_PRIO 0
#define SENSOR_PRIO // sensor task is prio 0

/// @brief The base class that contains the requirements that all tasks require to operate
class BaseTask{
public:
  BaseTask(unsigned int task_priority, EventGroupHandle_t& egh) : task_priority{task_priority}, egh{egh} {} ;
private:

protected:
  ~BaseTask();
  unsigned int task_priority;
  EventGroupHandle_t& egh;
  virtual void main_task() = 0;
};
#endif //BASE_TASK_HPP
