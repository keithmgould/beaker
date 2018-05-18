
#include <chrono>
#include <thread>
using namespace std::chrono;


// commence Ghetto Mocking
long millis() { 
	milliseconds ms = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
  long duration = ms.count();
  return duration;
}

float constrain(float a, float b, float c){
	if(a<b){return b;}
	if(a>c){return c;}
	return a;
}

void snooze(int ms){
	std::this_thread::sleep_for (std::chrono::milliseconds(ms));
}

// cease with Ghetto Mocking

#include "../bandPassFilter.h"
#include "gtest/gtest.h"

namespace {

  //-------------------------------------------------------
	// #size()
	//-------------------------------------------------------

	TEST(BandPassFilterTest, TestSize) {
		BandPassFilter bpf;
		unsigned int results = bpf.size();
		EXPECT_FLOAT_EQ(results, 0);
		bpf.filter(-6.28);
		snooze(10);
		bpf.filter(6.28);
		results = bpf.size();
		EXPECT_FLOAT_EQ(results, 1);
	}

	//-------------------------------------------------------
	// #getMultiplier()
	//-------------------------------------------------------

	TEST(BandPassFilterTest, GetMultiplierWhenEmpty) {
		BandPassFilter bpf;
		float results = bpf.getMultiplier();
	  EXPECT_FLOAT_EQ(results,0);
	}

	TEST(BandPassFilterTest, GetMultiplierAfterSinglePositiveValue) {
		BandPassFilter bpf;
		bpf.filter(6.28);
		float results = bpf.getMultiplier();
	  EXPECT_FLOAT_EQ(results,0);
	}

	TEST(BandPassFilterTest, GetMultiplierAfterSingleNegativeValue) {
		BandPassFilter bpf;
		bpf.filter(-6.28);
		float results = bpf.getMultiplier();
	  EXPECT_FLOAT_EQ(results,0);
	}

	// pass in enough values that we start to get a real multiplier
	TEST(BandPassFilterTest, GetMultiplierAfterSingleReversalPositiveToNegative) {
		BandPassFilter bpf;
		bpf.filter(-6.28);
		float results = bpf.getMultiplier();
		EXPECT_FLOAT_EQ(results,0);
		snooze(10);
		bpf.filter(6.28);
		results = bpf.getMultiplier();
		EXPECT_FLOAT_EQ(results,0);
		snooze(10);
		bpf.filter(-6.28);
		results = bpf.getMultiplier();
		EXPECT_FLOAT_EQ(results,0);
		snooze(10);
		bpf.filter(6.28);
		results = bpf.getMultiplier();
	  EXPECT_GT(results,0); // greater than zero
	}

  //-------------------------------------------------------
	// #filter()
	//-------------------------------------------------------

	TEST(BandPassFilterTest, SingleFilter) {
		BandPassFilter bpf;
		float results = bpf.filter(6.28);
		EXPECT_FLOAT_EQ(results, 6.28);
	}

	TEST(BandPassFilterTest, MultiFilterCloseToMax) {
		BandPassFilter bpf;
		float results = bpf.filter(-6.28);
		EXPECT_FLOAT_EQ(results,-6.28);
		
		snooze(136);
		results = bpf.filter(6.28);
		EXPECT_FLOAT_EQ(results,6.28);
		
		snooze(136);
		results = bpf.filter(-6.28);
		EXPECT_FLOAT_EQ(results,-6.28);
		
		snooze(136);
		results = bpf.filter(6.28);
		EXPECT_GT(bpf.getMultiplier(), 0.2);
		EXPECT_LT(bpf.getMultiplier(), 0.201);
		EXPECT_GT(bpf.getAverage(), 136);
		EXPECT_LT(bpf.getAverage(), 150);
		EXPECT_GT(results, 1.25);
		EXPECT_LT(results, 1.26);
	}

	TEST(BandPassFilterTest, MultiFilterSmall) {
		BandPassFilter bpf;
		float results = bpf.filter(-6.28);
		EXPECT_FLOAT_EQ(results,-6.28);
		
		snooze(10);
		results = bpf.filter(6.28);
		EXPECT_FLOAT_EQ(results,6.28);
		
		snooze(10);
		results = bpf.filter(-6.28);
		EXPECT_FLOAT_EQ(results,-6.28);
		
		snooze(10);
		results = bpf.filter(6.28);
		EXPECT_GT(bpf.getMultiplier(), 0.930);
		EXPECT_LT(bpf.getMultiplier(), 0.95);
		EXPECT_GT(bpf.getAverage(), 10);
		EXPECT_LT(bpf.getAverage(), 15);
		EXPECT_GT(results, 5.8);
		EXPECT_LT(results, 6);
	}

	TEST(BandPassFilterTest, MultiFilterLarge) {
		BandPassFilter bpf;
		float results = bpf.filter(-6.28);
		EXPECT_FLOAT_EQ(results,-6.28);
		
		snooze(1000);
		results = bpf.filter(6.28);
		EXPECT_FLOAT_EQ(results,6.28);
		
		snooze(1000);
		results = bpf.filter(-6.28);
		EXPECT_FLOAT_EQ(results,-6.28);
		
		snooze(1000);
		results = bpf.filter(6.28);
		// EXPECT_GT(bpf.getMultiplier(), 0.930);
		// EXPECT_LT(bpf.getMultiplier(), 0.95);
		// EXPECT_GT(bpf.getAverage(), 10);
		// EXPECT_LT(bpf.getAverage(), 15);
		// EXPECT_GT(results, 5.8);
		// EXPECT_LT(results, 6);
		std::cout << "results: " << results << ", avg: " << bpf.getAverage() << ", multiplier: " << bpf.getMultiplier() << "\n";
	}


}