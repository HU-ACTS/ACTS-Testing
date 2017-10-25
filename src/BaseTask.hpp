#ifndef BASE_TASK_HPP
#define BASE_TASK_HPP

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "SystemVariables.hpp"
#define BIT_0	( 1 << 0 )
#define BIT_4	( 1 << 4 )

// -------Flags for Tasks----------------------------
// Flag for SensorTask to act upon
const EventBits_t SensorMeasurementFlag = ( 1 << 0 );

// Flag for SdWriterTask to act upon
const EventBits_t SensorBufferSdReady   = ( 1 << 4 );

// Flags for WifiTask to act upon
const EventBits_t WifiActivateFlag      = (1 << 1);
const EventBits_t WifiReadyFlag         = (1 << 2);
//-------------------------------------------------

#define WIFI_PRIO 1 // priority is lower than sd writer because sd writer could possibly otherwise not write correctly
#define SD_PRIO 0
#define SENSOR_PRIO // sensor task is prio 0

/// @brief The base class that contains the requirements that all tasks require to operate
class BaseTask{
public:
  BaseTask(unsigned int task_priority) : task_priority{task_priority} {} ;
private:
protected:
  ~BaseTask();
  unsigned int task_priority;
  virtual void main_task() = 0;
};
#endif //BASE_TASK_HPP
