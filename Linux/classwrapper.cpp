#include "MotionCombine.h"
#include <stdio.h>

extern "C" void* createInstance()
{
	static hsc3::algo::MotionCombine* mInstance = NULL;
	if(mInstance == NULL)
	{
		printf("/--------------createInstance-->MotionCombine-------------/\n");
		mInstance = new hsc3::algo::MotionCombine();
	}
	return mInstance;
}

extern "C" int execPlan(void* obj, struct GroupConfigPara *cfgpara)
{
	static_cast<hsc3::algo::MotionCombine*>(obj)->execPlan(cfgpara);
	return 0;
}

extern "C" int setRatio(void* obj, double radio)
{
	static_cast<hsc3::algo::MotionCombine*>(obj)->setRatio(radio);
	return 0;
}

extern "C" double getRatio(void* obj)
{
	return static_cast<hsc3::algo::MotionCombine*>(obj)->getRatio();
}

extern "C" void destroy_instance(void* obj)
{
	delete static_cast<hsc3::algo::MotionCombine*>(obj);
}