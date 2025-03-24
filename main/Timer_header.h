#include <esp_sntp.h>

struct timeval tv_now;

int64_t get_time_us()
{
	gettimeofday(&tv_now, NULL);
	int64_t time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
	return time_us;
}
