// read_time() - Reads the current time

#if WIN32
void read_time(LARGE_INTEGER *current_time){
    QueryPerformanceCounter(&current_time); // Get current time
}
#else // If Linux
void read_time(struct timespec *current_time){
    clock_gettime(CLOCK_MONOTONIC_RAW, current_time); // Get current time
}
#endif
