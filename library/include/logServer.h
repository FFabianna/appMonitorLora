
#ifndef LOGSERVER_H_
#define LOGSERVER_H_

void* log_open(void *taskHandle, void *xMutex, char *buff, int *length, const int buffSize);
void log_close(void *handle);

#endif // LOGSERVER_H_