// Author: Victor Kronberg
// Date: 8/3/20
//
// Provides basic logging functionality using syslog and timestamps
// Timestamps adapted from Sam Siewert's implementation
//

#include "logging.h"


/**
 * [getTimeMsec]
 * @description:  Get current monotonic time of system in nsec and return in msec
 * @return        [double precision monotonic system time in msec]
 */
double getTimeMsec(void)
{
  struct timespec event_ts = {0, 0};

  clock_gettime(CLOCK_MONOTONIC, &event_ts);
  return ((event_ts.tv_sec)*1000.0) + ((event_ts.tv_nsec)/1000000.0);
}
