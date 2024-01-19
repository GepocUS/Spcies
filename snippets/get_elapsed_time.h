// get_elapsed_time() - Returns the elapsed time between 'now' and 'before'

#if WIN32
void get_elapsed_time(double *elapsed_time, LARGE_INTEGER *now, LARGE_INTEGER *before);
#else // If Linux
void get_elapsed_time(double *elapsed_time, struct timespec *now, struct timespec *before);
#endif
