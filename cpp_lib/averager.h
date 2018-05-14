#ifndef __BEAKER_AVERAGER__
#define __BEAKER_AVERAGER__

#include <deque>

/*
		Tiny helper class to hold queues of floats and compute their average,
		which is needed in a few places in the app
*/

class Averager{
	std::deque<float> values;
	unsigned int maxSize = 0;

	float sumValues(){
    float sum = 0;
    
    std::for_each(values.begin(), values.end(), [&] (float n) {
      sum += n;
    });

    return sum;
	}

	public:

	Averager(int mSize){
		// ensure good small size for averager
		if(mSize <= 0 || mSize > 100) { 
			Outputs::turnBuzzerOn();
			while(true){ delay(1); }
		}

		maxSize  = mSize;
	}

	float computeAverage(){
		float size = values.size();
		if(size == 0) { return 0;}
    return sumValues() / size;
  }

	void push(float newVal){
		values.push_front(newVal);
		if(values.size() > maxSize){
			values.pop_back();
		}
	}
};

#endif