#pragma once
#include <stdlib.h> 
//#include <glm\glm.hpp>

namespace Utils
{
	float Randomize(int smallestNum, int greatestNum)
	{
		int range = greatestNum - smallestNum;
		float decimals = rand() % range;
		decimals += (float)(rand() % 10000) / 10000;
		if (decimals > range)
			decimals = range;
		return decimals + smallestNum;
	}
}