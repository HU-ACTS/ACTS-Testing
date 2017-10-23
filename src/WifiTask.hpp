#ifndef WIFI_TASK_HPP
#define WIFI_TASK_HPP

#include "BaseTask.hpp"
class WifiTask : BaseTask {
public:
  WifiTask(unsigned int task_priority, EventGroupHandle_t& egh);
private:
protected:
  void WifiTask::main_task();
};
#endif //WIFI_TASK_HPP
