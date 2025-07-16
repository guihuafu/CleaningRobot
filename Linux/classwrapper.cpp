#include "MotionCombine.h"

extern "C" void* createInstance()
{
	return new hsc3::algo::MotionCombine();
}

extern "C" int execMotion(void* obj, int en)
{
	static_cast<hsc3::algo::MotionCombine*>(obj)->execMotion(en);
}

extern "C" int setRatio(void* obj, double radio)
{
	static_cast<hsc3::algo::MotionCombine*>(obj)->setRatio(radio);
}

extern "C" double getRatio(void* obj)
{
	return static_cast<hsc3::algo::MotionCombine*>(obj)->getRatio();
}

extern "C" void destroy_instance(void* obj)
{
	delete static_cast<hsc3::algo::MotionCombine*>(obj);
}