#ifndef __LOGGING_H__
#define __LOGGING_H__

#define _GNU_SOURCE

#include <time.h>

#define NSEC_PER_SEC (1000000000)
#define DELAY_TICKS (1)
#define ERROR (-1)
#define OK (0)


double getTimeMsec(void);

#endif /* logging.h */