#ifndef CTIMER_H
#define CTIMER_H

/**
@author Tom Krajnik
*/
#include <sys/time.h>
#include <stdlib.h>

#define TIMEOUT_INTERVAL 40000

class CTimer
{
	public:
		CTimer(int timeOut = TIMEOUT_INTERVAL);
		~CTimer();

		void reset(int timeOut = TIMEOUT_INTERVAL);
		bool paused();

		int pause();
		int start();
		int getTime();
		bool timeOut();

	private:
		int getRealTime();
		int startTime;
		int pauseTime;
		bool running;
		int timeoutInterval;
};

#endif
