#ifndef SENSOR_TASK_HPP
#define SENSOR_TASK_HPP

#include "BaseTask.hpp"
class SensorTask : BaseTask {
public:
    SensorTask(unsigned int task_priority);
private:
protected:
    void main_task();
};
#endif //SENSOR_TASK_HPP
