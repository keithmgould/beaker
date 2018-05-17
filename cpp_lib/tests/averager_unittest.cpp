#include <limits.h>
#include "../averager.h"
#include "gtest/gtest.h"
namespace {

	TEST(AveragerTest, EmptyAverager) {
		Averager averager = Averager(2);
	  EXPECT_EQ(averager.computeAverage(), 0);
	}

	TEST(AveragerTest, FilledAverager) {
		Averager averager = Averager(2);
		averager.push(1);
		averager.push(2);
	  EXPECT_EQ(averager.computeAverage(), 1.5);
	}

	TEST(AveragerTest, DequePops) {
		Averager averager = Averager(2);
		averager.push(1);
		averager.push(2);
		averager.push(4);
	  EXPECT_EQ(averager.computeAverage(), 3);
	}

	TEST(AveragerTest, SizeCheck) {
		Averager averager = Averager(2);
		EXPECT_EQ(averager.size(), 0);
		averager.push(1.5);
		EXPECT_EQ(averager.size(), 1);
		averager.push(1.5);
		EXPECT_EQ(averager.size(), 2);
		averager.push(1.5);
		EXPECT_EQ(averager.size(), 2);
	}
}