#pragma once
#include "calibrate.h"
#include "automove.h"
#include "baseautomove.h"
#include "basemanualmove.h"

#define CYCLE 0.004

namespace hsc3
{
	namespace algo
	{
		class MotionCombine
		{
		public:
			MotionCombine();
			~MotionCombine();
			void initPara();
			void planJoint();
			hsc3::algo::HS_MStatus execJointIntMove(double *jointpos, double *jointvel, double *spacepos);
			void planSpace();
			hsc3::algo::HS_MStatus execSpaceIntMove(double *jointpos, double *jointvel, double *spacepos);
			void planManual(double *nowpos);
			hsc3::algo::HS_MStatus execManualIntMove(double *jointpos);
			void stopPlanManual();
		public:
			hsc3::algo::GroupStaticPara mGroupStaticPara[4];

		private:
			hsc3::algo::MotionPara *mMotionPara;
			hsc3::algo::AutoMove *mAutoMove;
			hsc3::algo::BaseManualMove *mBaseManualMove;
			hsc3::algo::Calibrate *mCalibrate;
		};
	}
}