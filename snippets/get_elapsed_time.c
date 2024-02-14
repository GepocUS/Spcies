// get_elapsed_time() - Returns the elapsed time between 'now' and 'before'

#if WIN32
void get_elapsed_time(double *elapsed_time, LARGE_INTEGER *now, LARGE_INTEGER *before){
    __int64 measured_time;
    LARGE_INTEGER frequency;
    QueryPerformanceFrequency(&frequency);
    measured_time = 1000000000ULL * (now->QuadPart - before->QuadPart) / frequency.QuadPart;
    *elapsed_time = measured_time/(double)1e+9;
}
#else // If Linux
void get_elapsed_time(double *elapsed_time, struct timespec *now, struct timespec *before){
    *elapsed_time = (double) ( (now->tv_sec - before->tv_sec) * 1000.0 ) + (double) ( (now->tv_nsec - before->tv_nsec) / 1000000.0 );
}
#endif
