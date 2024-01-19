// read_time() - Reads the current time

#if WIN32
void read_time(LARGE_INTEGER *current_time);
#else // If Linux
void read_time(struct timespec *current_time);
#endif
