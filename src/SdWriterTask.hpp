#ifndef SD_WRITER_TASK_HPP
#define SD_WRITER_TASK_HPP
#include "BaseTask.hpp"
/// @brief SdWriterTask that handles the writing to the sd.
class SdWriterTask : BaseTask  {
  // ToDo Create two buffers of 50.000 bytes each (total is ~20% of system memory)
  // Buffer system should be created and ready to be used
public:
  SdWriterTask(unsigned int task_priority, EventGroupHandle_t& egh);
  ~SdWriterTask() = delete; //Should be deleted?
private:

protected:
  void main_task();
};
#endif //SD_WRITER_TASK_HPP
