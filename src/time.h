#ifndef NAV_TIME_H
#define NAV_TIME_H

#include <ctime>
#include <sys/types.h>

#include "./communication_LVI_SAM/msgs_ros2struct.h"

namespace system_time
{

// Calculate time difference, (e.g. difftime = a - b, timespec_diff(a, b, difftime))
inline void timespec_diff(const struct timespec& a, const struct timespec& b, struct timespec& result) {
   result.tv_sec  = a.tv_sec  - b.tv_sec;
   result.tv_nsec = a.tv_nsec - b.tv_nsec;
   if (result.tv_nsec < 0) {
      --result.tv_sec;
      result.tv_nsec += 1000000000L;
   }
}

// convert seconds(double) to timespec.
inline void fromSec(struct timespec& t, double d)
{
    t.tv_sec = d>=0.0 ? d : d+1;
    t.tv_nsec = (long) ((d - (double)t.tv_sec)*1e9L);
}

// convert timespec to seconeds(double).
inline double toSec(const struct timespec& t){return t.tv_sec + t.tv_nsec/1e9L;}

inline double getNowTimestamp()
{
   struct timespec t;
   clock_gettime(CLOCK_REALTIME, &t);
   return (static_cast<double>(t.tv_sec) + 1e-9*static_cast<double>(t.tv_nsec));
}

msgs_Time toTimeFromSec(double sec)
{
   double temp = sec;
   uint64_t t_sec = (uint64_t) temp;
   temp = temp - (double) t_sec;
   uint64_t t_nsec = (uint64_t) (temp*1e+9);
   msgs_Time time;
   time.sec = t_sec;
   time.nsec = t_nsec;
   return time;
}

}
#endif