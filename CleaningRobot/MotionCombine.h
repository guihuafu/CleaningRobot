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
			void setRatio(double ratio);
			void initPara();
			void planJoint(double *endpos);
			hsc3::algo::HS_MStatus execJointIntMove(double *jointpos, double *jointvel, double *jointacc, double *spacepos);
			void planSpace();
			hsc3::algo::HS_MStatus execSpaceIntMove(double *jointpos, double *jointvel, double *jointacc, double *spacepos);
			void planManual(int axisnum, bool dir, bool isjoint, double *nowpos);
			hsc3::algo::HS_MStatus execManualIntMove(double *jointpos, double *jointvel, double *jointacc, double *spacepos);
			void stopPlanManual();
		public:
			hsc3::algo::GroupStaticPara mGroupStaticPara[4];

		private:
			double mRatio;
			hsc3::algo::MotionPara *mMotionPara;
			hsc3::algo::AutoMove *mAutoMove;
			hsc3::algo::BaseManualMove *mBaseManualMove;
			hsc3::algo::Calibrate *mCalibrate;
		};
	}
}