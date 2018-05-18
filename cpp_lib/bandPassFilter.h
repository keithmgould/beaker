#ifndef __BEAKER_BPF__
#define __BEAKER_BPF__

#include "./averager.h"

/*
		This is a ghetto band-pass filter, as possibly found here:
		https://en.wikipedia.org/wiki/Inverted_pendulum#Essentials_of_stabilization

		I'm probably doing it totally wrong. :/

		Given Beaker's dimensions, the values are like so:

		square root of (g/l) = sq(9.8/0.181) ~ 7.358Hz ~ period of 136ms
*/

class BandPassFilter{
	unsigned int historySize = 3;

	Averager periods = Averager(historySize);

	float maxResonance = 136; // see notes above.
	float radPerSec = 0;
	bool isPreviousRadPerSecPositive = true;
	long lastReversal = 0;
	float multiplier = 0;
	float avgPeriod = 0;

	bool isReversed(){
		return (radPerSec < 0 && isPreviousRadPerSecPositive) || (radPerSec > 0 && !isPreviousRadPerSecPositive);
	}

	// store how long its been since last reversal.
	void storePeriod(){
		long nowish = millis();
		if(lastReversal != 0){
			periods.push(nowish - lastReversal);
		}
		lastReversal = nowish;
	}

	// minimum passed through is 20%
	// maximum passed through is 100%
	float applyFilter(){
		avgPeriod = periods.computeAverage();

		if(avgPeriod > maxResonance){
			multiplier = 0.0000594 * avgPeriod - 0.00808;
		}else{
			multiplier = -0.00594 * avgPeriod + 0.808;
		}

		multiplier = constrain(multiplier, 0, 0.8);
		multiplier += 0.2;

		return multiplier * radPerSec;
	}

	public:

	float getMultiplier(){
		return multiplier;
	}

	unsigned int size(){
		return periods.size();
	}

	float getAverage(){
		return avgPeriod;
	}

	float filter(float rps){
		// housekeeping
		radPerSec = rps;
		if(isReversed()){ storePeriod(); }
		isPreviousRadPerSecPositive = radPerSec > 0;

		// if we have no history, don't filter		
		if(periods.size() < historySize){ return radPerSec; }

		// finally the actual filter
		return applyFilter();
	}
};

#endif