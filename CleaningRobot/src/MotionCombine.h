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
		private:
			void initPara();
			hsc3::algo::GroupMotionData dealElemt(bool firstmove, int linenum, double *startpos, double *endpos);
			
			void planJoint(double *endpos);
			void planSpace();
			void planManual(int axisnum, bool dir, bool isjoint, double *nowpos);			
			void stopPlanManual();

			hsc3::algo::HS_MStatus execJointIntMove(double *jointpos, double *jointvel, double *jointacc, double *spacepos);
			hsc3::algo::HS_MStatus execSpaceIntMove(double *jointpos, double *jointvel, double *jointacc, double *spacepos);
			hsc3::algo::HS_MStatus execManualIntMove(double *jointpos, double *jointvel, double *jointacc, double *spacepos);
			
		public:
			MotionCombine();
			~MotionCombine();
			void setRatio(double ratio);
			double getRatio();
			void resetMotion();

			int execPlan(GroupConfigPara *config);
			int execMove(GroupCommandPara *cmddata, GroupFeedbackPara *fbdata);

		public:
			hsc3::algo::GroupStaticPara mGroupStaticPara[4];
			
		private:
			int mDataNum;
			double mRatio;
			double *mJointPos;
			double *mLastJointPos;
			double *mLastVel;
			hsc3::algo::MotionPara *mMotionPara;
			hsc3::algo::AutoMove *mAutoMove;
			hsc3::algo::BaseManualMove *mBaseManualMove;
			hsc3::algo::Calibrate *mCalibrate;
			hsc3::algo::GroupTrajData *mGroupTrajout;
			hsc3::algo::GroupConfigPara mGroupConfigPara;
			hsc3::algo::GroupFeedbackPara mGroupFeedbackPara;
		};
	}
}