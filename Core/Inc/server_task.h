#ifndef __SERVER_TASK_H__
#define __SERVER_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis_os.h"  // For osThreadAttr_t etc., if used in declaration

// Allow caller to create the task from outside
void ServerTask_Start(void);

// Directly exposing the task function for osThreadNew():
void ServerTask(void const *argument);

#ifdef __cplusplus
}
#endif

#endif // __SERVER_TASK_H__
