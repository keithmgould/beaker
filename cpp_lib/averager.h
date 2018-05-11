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

	bool isTimeToClear(){
		return maxSize > 0 && values.size() >= maxSize;
	}

	public:

	Averager(int mSize){
		maxSize  = mSize;
	}


	float computeAverage(){
		float sum = sumValues();
    float avg = sum / (float) values.size();
    if(isTimeToClear()) { values.clear(); }
    return avg;
  }

	void push(float newVal){
		values.push_front(newVal);
	}
};

#endif