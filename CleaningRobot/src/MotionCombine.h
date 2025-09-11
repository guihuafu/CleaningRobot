#pragma once
#include "calibrate.h"
#include "automove.h"
#include "baseautomove.h"
#include "basemanualmove.h"
#include "MotionParaDef.h"

#define CYCLE 0.004

namespace hsc3
{
	namespace algo
	{
		class MotionCombine
		{
		private:
			void initPara();
			hsc3::algo::GroupMotionData dealElemt(bool firstmove, int linenum, bool isjoint, double *startpos, double *endpos);
			
			int planAutoRun();
			int planMoveTo(double *endpos, bool isjoint);
			int planManual(int axisnum, bool dir, bool isjoint, double *nowpos);
			int stopPlanAuto();
			int stopPlanManual();

			hsc3::algo::HS_MStatus execMoveToIntMove(double *jointpos, double *jointvel, double *jointacc, double *spacepos, int &errid);
			hsc3::algo::HS_MStatus execAutoRunIntMove(double *jointpos, double *jointvel, double *jointacc, double *spacepos, int &errid);
			hsc3::algo::HS_MStatus execManualIntMove(double *jointpos, double *jointvel, double *jointacc, double *spacepos, int &errid);
			
		public:
			MotionCombine();
			~MotionCombine();
			void setRatio(double ratio);
			double getRatio();
			int syncPos(GroupCommandPara *cmddata, GroupFeedbackPara *fbdata);
			int resetMotion();

			int execPlan(GroupConfigPara *config);
			int execMove(GroupCommandPara *cmddata, GroupFeedbackPara *fbdata);

		public:
			hsc3::algo::GroupStaticPara mGroupStaticPara[4];
			
		private:
			bool mIsMoving;
			bool mFirstMove;
			int mDataNum;
			int mRunDataNum;
			double mAxis2DiffPos;
			double mRatio;
			double *mJointPos;
			double *mLastJointPos;
			double *mLastVel;
			hsc3::algo::MotionPara *mMotionPara;
			hsc3::algo::AutoMove *mAutoMove;
			hsc3::algo::BaseManualMove *mBaseManualMove;
			hsc3::algo::Calibrate *mCalibrate;
			hsc3::algo::GroupTrajData *mGroupTrajout;
			GroupConfigPara mGroupConfigPara;
			GroupCommandPara mGroupCommandPara;	// ¡˘÷·µ„Œª
			GroupFeedbackPara mGroupFeedbackPara;
			hsc3::algo::GroupMotionData *mGroupMotionData;
		};
	}
}