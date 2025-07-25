#include "stdafx.h"
#include "MotionCombine.h"
#include "HS3_MotionAlertDef.h"
#include <stdio.h>
#include <cstring>

namespace hsc3
{
	namespace algo
	{
		MotionCombine::MotionCombine()
		{
			this->mDataNum = 0;
			this->mRatio = 0.3;
			this->mJointPos = new double[MaxAxisNum];
			this->mLastJointPos = new double[MaxAxisNum];
			this->mLastVel = new double[MaxAxisNum];
			mJointPos[0] = 0.0; mJointPos[1] = -90.0; mJointPos[2] = 180.0;
			mJointPos[3] = 0.0; mJointPos[4] = 90.0; mJointPos[5] = 0.0;
			mJointPos[6] = 0.0; mJointPos[7] = 0.0; mJointPos[8] = 0.0;
			this->mGroupTrajout = new hsc3::algo::GroupTrajData[40];
			memset(this->mLastJointPos, 0.0, sizeof(double)*MaxAxisNum);
			memset(this->mLastVel, 0.0, sizeof(double)*MaxAxisNum);
			memset(this->mGroupTrajout, 0, sizeof(hsc3::algo::GroupTrajData) * 40);
			this->initPara();
			this->mMotionPara = new hsc3::algo::MotionPara();
			this->mMotionPara->setGroupStaticPara(mGroupStaticPara);
			this->mAutoMove = new hsc3::algo::BaseAutoMove(mMotionPara, CYCLE, 2);
			this->mBaseManualMove = new hsc3::algo::BaseManualMove(mMotionPara, CYCLE);
			this->mCalibrate = new hsc3::algo::Calibrate(mMotionPara, CYCLE, 0);
			
			this->mGroupConfigPara.ePlanMode = Plan_None;
			this->mGroupConfigPara.iAxisNum = 0;
			this->mGroupConfigPara.iIsJoint = true;
			this->mGroupConfigPara.iDir = true;
			memset(this->mGroupConfigPara.dPos, 0.0, sizeof(double)*MaxAxisNum);

			this->mGroupFeedbackPara.iStatus = 0;
			this->mGroupFeedbackPara.iServoErr = 0;
			memset(this->mGroupFeedbackPara.dFbAxisPos, 0.0, sizeof(double)*MaxAxisNum);
			memset(this->mGroupFeedbackPara.dFbAxisVel, 0.0, sizeof(double)*MaxAxisNum);
			memset(this->mGroupFeedbackPara.dFbAxisAcc, 0.0, sizeof(double)*MaxAxisNum);
		}

		MotionCombine::~MotionCombine()
		{
			delete []mJointPos;
			delete []mLastJointPos;
			delete []mLastVel;
		}

		void MotionCombine::setRatio(double ratio)
		{
			this->mRatio = ratio / 100.0;
			printf("MotionCombine::setRatio--%f \n", this->mRatio);
		}

		double MotionCombine::getRatio()
		{
			return this->mRatio;
		}

		void MotionCombine::initPara()
		{
			mGroupStaticPara[0].tGroupModelPara.eRobtype = hsc3::algo::HSROB_PUMA;
			mGroupStaticPara[0].tGroupVelocityPara.dVtran = 1700.0;
			mGroupStaticPara[0].tGroupVelocityPara.dVrot = 50.0;
			mGroupStaticPara[0].tGroupVelocityPara.dVtranacc = 100.0;
			mGroupStaticPara[0].tGroupVelocityPara.dVrotacc = 100.0;
			mGroupStaticPara[0].tGroupVelocityPara.dJerkrat = 9.0;
			mGroupStaticPara[0].tGroupVelocityPara.dTFreMin = 0.05;
			mGroupStaticPara[0].tGroupVelocityPara.dTFreMax = 0.4;

			mGroupStaticPara[0].tGroupModelPara.DHPara[0][0] = 0.0;    mGroupStaticPara[0].tGroupModelPara.DHPara[0][1] = 0.0;  mGroupStaticPara[0].tGroupModelPara.DHPara[0][2] = -90.0; mGroupStaticPara[0].tGroupModelPara.DHPara[0][3] = 0.0;
			mGroupStaticPara[0].tGroupModelPara.DHPara[1][0] = 1700.0; mGroupStaticPara[0].tGroupModelPara.DHPara[1][1] = 0.0;  mGroupStaticPara[0].tGroupModelPara.DHPara[1][2] = 0.0; mGroupStaticPara[0].tGroupModelPara.DHPara[1][3] = 0.0;
			mGroupStaticPara[0].tGroupModelPara.DHPara[2][0] = 0.0;    mGroupStaticPara[0].tGroupModelPara.DHPara[2][1] = 0.0;  mGroupStaticPara[0].tGroupModelPara.DHPara[2][2] = 90.0; mGroupStaticPara[0].tGroupModelPara.DHPara[2][3] = 0.0;
			mGroupStaticPara[0].tGroupModelPara.DHPara[3][0] = 0.0;    mGroupStaticPara[0].tGroupModelPara.DHPara[3][1] = 1700.0; mGroupStaticPara[0].tGroupModelPara.DHPara[3][2] = -90.0; mGroupStaticPara[0].tGroupModelPara.DHPara[3][3] = 0.0;
			mGroupStaticPara[0].tGroupModelPara.DHPara[4][0] = 0.0;    mGroupStaticPara[0].tGroupModelPara.DHPara[4][1] = 0.0;   mGroupStaticPara[0].tGroupModelPara.DHPara[4][2] = 90.0; mGroupStaticPara[0].tGroupModelPara.DHPara[4][3] = 0.0;
			mGroupStaticPara[0].tGroupModelPara.DHPara[5][0] = 0.0;    mGroupStaticPara[0].tGroupModelPara.DHPara[5][1] = 140.0; mGroupStaticPara[0].tGroupModelPara.DHPara[5][2] = 0.0; mGroupStaticPara[0].tGroupModelPara.DHPara[5][3] = 0.0;

			//mGroupStaticPara[0].tGroupModelPara.DHPara[0][0] = 0.0; mGroupStaticPara[0].tGroupModelPara.DHPara[0][1] = 0.0; mGroupStaticPara[0].tGroupModelPara.DHPara[0][2] = -90.0; mGroupStaticPara[0].tGroupModelPara.DHPara[0][3] = 0.0;
			//mGroupStaticPara[0].tGroupModelPara.DHPara[1][0] = 360.0; mGroupStaticPara[0].tGroupModelPara.DHPara[1][1] = 0.0; mGroupStaticPara[0].tGroupModelPara.DHPara[1][2] = 0.0; mGroupStaticPara[0].tGroupModelPara.DHPara[1][3] = 0.0;
			//mGroupStaticPara[0].tGroupModelPara.DHPara[2][0] = -90.0; mGroupStaticPara[0].tGroupModelPara.DHPara[2][1] = 0.0; mGroupStaticPara[0].tGroupModelPara.DHPara[2][2] = 90.0; mGroupStaticPara[0].tGroupModelPara.DHPara[2][3] = 0.0;
			//mGroupStaticPara[0].tGroupModelPara.DHPara[3][0] = 0.0; mGroupStaticPara[0].tGroupModelPara.DHPara[3][1] = 376.5; mGroupStaticPara[0].tGroupModelPara.DHPara[3][2] = -90.0; mGroupStaticPara[0].tGroupModelPara.DHPara[3][3] = 0.0;
			//mGroupStaticPara[0].tGroupModelPara.DHPara[4][0] = 0.0; mGroupStaticPara[0].tGroupModelPara.DHPara[4][1] = 0.0; mGroupStaticPara[0].tGroupModelPara.DHPara[4][2] = 90.0; mGroupStaticPara[0].tGroupModelPara.DHPara[4][3] = 0.0;
			//mGroupStaticPara[0].tGroupModelPara.DHPara[5][0] = 0.0; mGroupStaticPara[0].tGroupModelPara.DHPara[5][1] = 119.0; mGroupStaticPara[0].tGroupModelPara.DHPara[5][2] = 0.0; mGroupStaticPara[0].tGroupModelPara.DHPara[5][3] = 0.0;
			memset(mGroupStaticPara[0].dWorldCoord, 0, sizeof(double) * 6);

			for(int i=0; i<MAXCOORDNUM; i++)
			{
				memset(mGroupStaticPara[0].dToolCoord[i], 0, sizeof(double) * 6);
				memset(mGroupStaticPara[0].dWorkCoord[i], 0, sizeof(double) * 6);
			}

			for(int i=0; i<MaxAxisNum; i++)
			{
				mGroupStaticPara[0].tAxisVelocityPara.dVmax[i] = 250.0;
				mGroupStaticPara[0].tAxisVelocityPara.dVcruise[i] = 200.0;
				mGroupStaticPara[0].tAxisVelocityPara.dAccelerate[i] = 2000.0;
				mGroupStaticPara[0].tAxisVelocityPara.dJerkrat[i] = 400.0;
				mGroupStaticPara[0].tLimitPara.dPmax[i] = 360.0;
				mGroupStaticPara[0].tLimitPara.dPmin[i] = -360.0;
				mGroupStaticPara[0].tLimitPara.bOpen[i] = true;
			}
		}

		void MotionCombine::planJoint(double *endpos)
		{
			int mMotionDataNum = 0;
			hsc3::algo::GroupMotionData groupdata = {0};
			memset(this->mLastJointPos, 0.0, sizeof(double)*MaxAxisNum);
			memset(this->mLastVel, 0.0, sizeof(double)*MaxAxisNum);

			groupdata.iLineNum = 0;
			groupdata.tHS_GroupRel.eGroupRelType[0] = hsc3::algo::GRT_Independent;
			groupdata.tHS_GroupRel.eGroupRelType[1] = hsc3::algo::GRT_NoUse;
			groupdata.tHS_GroupRel.eGroupRelType[2] = hsc3::algo::GRT_NoUse;
			groupdata.tHS_GroupRel.eGroupRelType[3] = hsc3::algo::GRT_NoUse;
			groupdata.dCnt = 0.0;
			groupdata.dCR = 0.0;
			groupdata.iSmooth = 0;
			groupdata.bStartMove = true;
			groupdata.bWristQYFlag = false;
			groupdata.tFilterControl.bFilterOpenFlag = false;
			groupdata.tBaseMoveData[0].eTrajType = hsc3::algo::MP_Joint;
			groupdata.tBaseMoveData[0].sCurCoordinate.iCoordinate = hsc3::algo::JOINT_COORD_SYSTEM;
			groupdata.tBaseMoveData[0].sCurCoordinate.iToolNum = -1;
			groupdata.tBaseMoveData[0].sCurCoordinate.iWorkNum = -1;
			groupdata.tBaseMoveData[0].sStartPos.dPos[0] = 0.0; groupdata.tBaseMoveData[0].sStartPos.dPos[1] = -90.0; groupdata.tBaseMoveData[0].sStartPos.dPos[2] = 180.0;
			groupdata.tBaseMoveData[0].sStartPos.dPos[3] = 0.0; groupdata.tBaseMoveData[0].sStartPos.dPos[4] = 90.0; groupdata.tBaseMoveData[0].sStartPos.dPos[5] = 0.0;
			groupdata.tBaseMoveData[0].sStartPos.dPos[6] = 0.0; groupdata.tBaseMoveData[0].sStartPos.dPos[7] = 0.0; groupdata.tBaseMoveData[0].sStartPos.dPos[8] = 0.0;
			groupdata.tBaseMoveData[0].sStartPos.iPose = 0;
			groupdata.tBaseMoveData[0].sStartPos.hs_coordinate.iCoordinate = hsc3::algo::JOINT_COORD_SYSTEM;
			groupdata.tBaseMoveData[0].sStartPos.hs_coordinate.iToolNum = -1;
			groupdata.tBaseMoveData[0].sStartPos.hs_coordinate.iWorkNum = -1;
			memset(groupdata.tBaseMoveData[0].sMidPos.dPos, 0, sizeof(double) * 9);
			groupdata.tBaseMoveData[0].sEndPos.dPos[0] = endpos[0]; groupdata.tBaseMoveData[0].sEndPos.dPos[1] = endpos[1]; groupdata.tBaseMoveData[0].sEndPos.dPos[2] = endpos[2];
			groupdata.tBaseMoveData[0].sEndPos.dPos[3] = endpos[3]; groupdata.tBaseMoveData[0].sEndPos.dPos[4] = endpos[4]; groupdata.tBaseMoveData[0].sEndPos.dPos[5] = endpos[5];
			groupdata.tBaseMoveData[0].sEndPos.dPos[6] = endpos[6]; groupdata.tBaseMoveData[0].sEndPos.dPos[7] = 0; groupdata.tBaseMoveData[0].sEndPos.dPos[8] = 0;
			groupdata.tBaseMoveData[0].sEndPos.iPose = 0;
			groupdata.tBaseMoveData[0].sEndPos.hs_coordinate.iCoordinate = hsc3::algo::JOINT_COORD_SYSTEM;
			groupdata.tBaseMoveData[0].sEndPos.hs_coordinate.iToolNum = -1;
			groupdata.tBaseMoveData[0].sEndPos.hs_coordinate.iWorkNum = -1;
			groupdata.tBaseMoveData[0].b2mid = false;
			groupdata.tBaseMoveData[0].dVel = 100.0;
			groupdata.tBaseMoveData[0].dVort = 100.0;
			groupdata.tBaseMoveData[0].dAcc = 100.0;
			groupdata.tBaseMoveData[0].dDec = 100.0;
			groupdata.tBaseMoveData[0].iCntType = 0;
			groupdata.tBaseMoveData[0].tRevolve.iTurn = 0;

			this->mAutoMove->execPrehandle(groupdata, this->mGroupTrajout, mMotionDataNum);
			hsc3::algo::HS_GroupJPos groupjpos = {0};
			memcpy(groupjpos.dJPos[0], groupdata.tBaseMoveData[0].sStartPos.dPos, sizeof(double) * MaxAxisNum);

			this->mAutoMove->execPlanMove(this->mGroupTrajout, 0, this->mRatio, groupjpos);
		}

		hsc3::algo::HS_MStatus MotionCombine::execJointIntMove(double *jointpos, double *jointvel, double *jointacc, double *spacepos)
		{
			int errorID = 0;
			hsc3::algo::IntData intdata = {0.0};
			hsc3::algo::HS_MStatus status = hsc3::algo::M_UnInit;
			status = this->mAutoMove->execIntMove(intdata, errorID);					// 获取周期关节插补点
			memcpy(jointpos, intdata.tGJPos[0].dJPos[0], sizeof(double)*MaxAxisNum);
			this->mCalibrate->calcJPosToCPos(jointpos, -1, -1, spacepos);				// 获取空间位置

			for(int i=0; i<MaxAxisNum; i++)
			{
				jointvel[i] = (jointpos[i] - this->mLastJointPos[i]) / CYCLE;			// 获取关节速度
				if((jointvel[i] > 500) || (jointvel[i] < -500))
					jointvel[i] = 0.0;

				jointacc[i] = (jointvel[i] - this->mLastVel[i]) / CYCLE;				// 获取关节加速度
				//if((jointacc[i] > 400) || (jointacc[i] < -400))
				//	jointacc[i] = 0.0;
			}

			memcpy(this->mLastJointPos, jointpos, sizeof(double)*MaxAxisNum);
			memcpy(this->mLastVel, jointvel, sizeof(double)*MaxAxisNum);

			return status;
		}

		hsc3::algo::GroupMotionData MotionCombine::dealElemt(bool firstmove, int linenum, double *startpos, double *endpos)
		{
			hsc3::algo::GroupMotionData groupdata = {0};
			groupdata.iLineNum = linenum;
			groupdata.tHS_GroupRel.eGroupRelType[0] = hsc3::algo::GRT_Independent;
			groupdata.tHS_GroupRel.eGroupRelType[1] = hsc3::algo::GRT_NoUse;
			groupdata.tHS_GroupRel.eGroupRelType[2] = hsc3::algo::GRT_NoUse;
			groupdata.tHS_GroupRel.eGroupRelType[3] = hsc3::algo::GRT_NoUse;
			groupdata.dCnt = 0.0;
			groupdata.dCR = 0.0;
			groupdata.iSmooth = 0;
			groupdata.bStartMove = firstmove;
			groupdata.bWristQYFlag = false;
			groupdata.tFilterControl.bFilterOpenFlag = false;
			groupdata.tBaseMoveData[0].eTrajType = hsc3::algo::MP_Line;
			groupdata.tBaseMoveData[0].sCurCoordinate.iCoordinate = hsc3::algo::BASE_COORD_SYSTEM;
			groupdata.tBaseMoveData[0].sCurCoordinate.iToolNum = -1;
			groupdata.tBaseMoveData[0].sCurCoordinate.iWorkNum = -1;
			groupdata.tBaseMoveData[0].sStartPos.iPose = 0;
			groupdata.tBaseMoveData[0].sStartPos.hs_coordinate.iToolNum = -1;
			groupdata.tBaseMoveData[0].sStartPos.hs_coordinate.iWorkNum = -1;

			if(firstmove)
			{
				groupdata.tBaseMoveData[0].sStartPos.hs_coordinate.iCoordinate = hsc3::algo::JOINT_COORD_SYSTEM;
				groupdata.tBaseMoveData[0].sStartPos.dPos[0] = 0.0; groupdata.tBaseMoveData[0].sStartPos.dPos[1] = -90.0; groupdata.tBaseMoveData[0].sStartPos.dPos[2] = 180.0;
				groupdata.tBaseMoveData[0].sStartPos.dPos[3] = 0.0; groupdata.tBaseMoveData[0].sStartPos.dPos[4] = 90.0; groupdata.tBaseMoveData[0].sStartPos.dPos[5] = 0.0;
				groupdata.tBaseMoveData[0].sStartPos.dPos[6] = 0; groupdata.tBaseMoveData[0].sStartPos.dPos[7] = 0; groupdata.tBaseMoveData[0].sStartPos.dPos[8] = 0;
			}
			else
			{
				groupdata.tBaseMoveData[0].sStartPos.hs_coordinate.iCoordinate = hsc3::algo::BASE_COORD_SYSTEM;
				groupdata.tBaseMoveData[0].sStartPos.dPos[0] = startpos[0]; groupdata.tBaseMoveData[0].sStartPos.dPos[1] = startpos[1]; groupdata.tBaseMoveData[0].sStartPos.dPos[2] = startpos[2];
				groupdata.tBaseMoveData[0].sStartPos.dPos[3] = startpos[3]; groupdata.tBaseMoveData[0].sStartPos.dPos[4] = startpos[4]; groupdata.tBaseMoveData[0].sStartPos.dPos[5] = startpos[5];
				groupdata.tBaseMoveData[0].sStartPos.dPos[6] = 0; groupdata.tBaseMoveData[0].sStartPos.dPos[7] = 0; groupdata.tBaseMoveData[0].sStartPos.dPos[8] = 0;
			}

			memset(groupdata.tBaseMoveData[0].sMidPos.dPos, 0, sizeof(double) * 9);
			groupdata.tBaseMoveData[0].sEndPos.dPos[0] = endpos[0]; groupdata.tBaseMoveData[0].sEndPos.dPos[1] = endpos[1]; groupdata.tBaseMoveData[0].sEndPos.dPos[2] = endpos[2];
			groupdata.tBaseMoveData[0].sEndPos.dPos[3] = endpos[3]; groupdata.tBaseMoveData[0].sEndPos.dPos[4] = endpos[4]; groupdata.tBaseMoveData[0].sEndPos.dPos[5] = endpos[5];
			groupdata.tBaseMoveData[0].sEndPos.dPos[6] = endpos[6]; groupdata.tBaseMoveData[0].sEndPos.dPos[7] = 0; groupdata.tBaseMoveData[0].sEndPos.dPos[8] = 0;
			groupdata.tBaseMoveData[0].sEndPos.iPose = 0;
			groupdata.tBaseMoveData[0].sEndPos.hs_coordinate.iCoordinate = hsc3::algo::BASE_COORD_SYSTEM;
			groupdata.tBaseMoveData[0].sEndPos.hs_coordinate.iToolNum = -1;
			groupdata.tBaseMoveData[0].sEndPos.hs_coordinate.iWorkNum = -1;
			groupdata.tBaseMoveData[0].b2mid = false;
			groupdata.tBaseMoveData[0].dVel = 1500.0;
			groupdata.tBaseMoveData[0].dVort = 100.0;
			groupdata.tBaseMoveData[0].dAcc = 100.0;
			groupdata.tBaseMoveData[0].dDec = 100.0;
			groupdata.tBaseMoveData[0].iCntType = 0;
			groupdata.tBaseMoveData[0].tRevolve.iTurn = 0;

			return groupdata;
		}

		void MotionCombine::planSpace()
		{
			memset(this->mLastJointPos, 0.0, sizeof(double)*MaxAxisNum);
			memset(this->mLastVel, 0.0, sizeof(double)*MaxAxisNum);
			hsc3::algo::GroupMotionData groupdata = {0};
			double dStartPos1[MaxAxisNum] = {0.0, -90.0, 180.0, 0.0, 90.0, 0.0, 0.0, 0.0, 0.0};
			double dEndPos1[MaxAxisNum] = {1700.0, 0.0, 1400.0, 180.0, 0.0, 180.0, 60.0, 0.0, 0.0};
			double dEndPos2[MaxAxisNum] = {1700.0, 100.0, 1400.0, 180.0, 0.0, 180.0, -30.0, 0.0, 0.0};
			double dEndPos3[MaxAxisNum] = {1500, 100.0, 1400.0, 180.0, 0.0, 180.0, 60.0, 0.0, 0.0};
			double dEndPos4[MaxAxisNum] = {1500, 0.0, 1400.0, 180.0, 0.0, 180.0, 30.0, 0.0, 0.0};

			memcpy(this->mJointPos, dStartPos1, sizeof(double)*MaxAxisNum);

			this->mDataNum = 0;
			groupdata = this->dealElemt(true, this->mDataNum, dStartPos1, dEndPos1);
			this->mAutoMove->execPrehandle(groupdata, this->mGroupTrajout, this->mDataNum);

			this->mDataNum = 1;
			groupdata = this->dealElemt(false, this->mDataNum, dEndPos1, dEndPos2);
			this->mAutoMove->execPrehandle(groupdata, this->mGroupTrajout, this->mDataNum);

			this->mDataNum = 2;
			groupdata = this->dealElemt(false, this->mDataNum, dEndPos2, dEndPos3);
			this->mAutoMove->execPrehandle(groupdata, this->mGroupTrajout, this->mDataNum);

			this->mDataNum = 3;
			groupdata = this->dealElemt(false, this->mDataNum, dEndPos3, dEndPos4);
			this->mAutoMove->execPrehandle(groupdata, this->mGroupTrajout, this->mDataNum);
		}

		hsc3::algo::HS_MStatus MotionCombine::execSpaceIntMove(double *jointpos, double *jointvel, double *jointacc, double *spacepos)
		{
			int errorID = 0;
			static int iID = 0;
			bool bCalcOut = false;
			hsc3::algo::IntData intdata = {0.0};
			hsc3::algo::HS_GroupJPos groupjpos = {0.0};
			hsc3::algo::HS_MStatus status = hsc3::algo::M_UnInit;
			status = this->mAutoMove->execIntMove(intdata, errorID);					// 获取周期关节插补点

			switch(status)
			{
				case hsc3::algo::M_UnInit:
					if(iID <= this->mDataNum)
					{
						bCalcOut = true;
						memcpy(groupjpos.dJPos[0], this->mJointPos, sizeof(double) * MaxAxisNum);
						errorID = this->mAutoMove->execPlanMove(this->mGroupTrajout, iID, this->mRatio, groupjpos);
						if((errorID != 0) && (errorID < Waring))
							status = hsc3::algo::M_Error;
						printf("MotionCombine::execSpaceIntMove--M_UnInit--Plan Next Position\n");
					}
					break;
				case hsc3::algo::M_Busy:
					bCalcOut = true;
					memcpy(this->mJointPos, intdata.tGJPos[0].dJPos[0], sizeof(double)*MaxAxisNum);
					break;
				case hsc3::algo::M_Done:
					if(iID <= this->mDataNum)
					{
						bCalcOut = true;
						iID = iID + 1;
						memcpy(groupjpos.dJPos[0], intdata.tGJPos[0].dJPos[0], sizeof(double) * MaxAxisNum);
						errorID = this->mAutoMove->execPlanMove(this->mGroupTrajout, iID, this->mRatio, groupjpos);
						printf("MotionCombine::execSpaceIntMove--Plan Next Position--errorID=%d \n", errorID);
						if((errorID != 0) && (errorID < Waring))
							status = hsc3::algo::M_Error;
						status = hsc3::algo::M_UnInit;
						memcpy(this->mJointPos, intdata.tGJPos[0].dJPos[0], sizeof(double)*MaxAxisNum);
						goto outres;
					}
					iID = 0;
					printf("MotionCombine::execSpaceIntMove--Plan Finish M_Done\n");
					return hsc3::algo::M_Done;
					break;
				case hsc3::algo::M_Error:
					printf("MotionCombine::execSpaceIntMove--IntMove Error\n");
					status = hsc3::algo::M_Error;
					break;
			}
			outres:
			if(bCalcOut)
			{
				memcpy(jointpos, this->mJointPos, sizeof(double)*MaxAxisNum);
				this->mCalibrate->calcJPosToCPos(jointpos, -1, -1, spacepos);				// 获取空间位置

				for(int i=0; i<MaxAxisNum; i++)
				{
					jointvel[i] = (jointpos[i] - this->mLastJointPos[i]) / CYCLE;			// 获取关节速度
					if((jointvel[i] > 500) || (jointvel[i] < -500))
						jointvel[i] = 0.0;

					jointacc[i] = (jointvel[i] - this->mLastVel[i]) / CYCLE;				// 获取关节加速度
					//if((jointacc[i] > 10) || (jointacc[i] < -10))
					//	jointacc[i] = 0.0;
				}

				memcpy(this->mLastJointPos, jointpos, sizeof(double)*MaxAxisNum);
				memcpy(this->mLastVel, jointvel, sizeof(double)*MaxAxisNum);
			}

			return status;
		}

		void MotionCombine::planManual(int axisnum, bool dir, bool isjoint, double *nowpos)
		{
			printf("planManual nowpos %f %f %f %f %f %f \n",nowpos[0],nowpos[1],nowpos[2],nowpos[3],nowpos[4],nowpos[5]);
			//memset(this->mLastJointPos, 0.0, sizeof(double)*MaxAxisNum);
			//memset(this->mLastVel, 0.0, sizeof(double)*MaxAxisNum);
			hsc3::algo::ManualPara mManualPara;
			mManualPara.iAxisNum = axisnum;
			mManualPara.iGroupNum = 0;
			mManualPara.bDir = dir;
			mManualPara.dHandVelRatio = this->mRatio;
			mManualPara.iSmooth = 5;
			mManualPara.bWristQYOpen = true;
			mManualPara.dVtran = 250.0;
			mManualPara.dVrot = 120.0;
			memset(mManualPara.iToolNum, 0, sizeof(int)*MAXGROUPNUM);
			mManualPara.tHS_GroupRel.eGroupRelType[0] = hsc3::algo::GRT_Independent;
			mManualPara.tHS_GroupRel.eGroupRelType[1] = hsc3::algo::GRT_NoUse;
			mManualPara.tHS_GroupRel.eGroupRelType[2] = hsc3::algo::GRT_NoUse;
			mManualPara.tHS_GroupRel.eGroupRelType[3] = hsc3::algo::GRT_NoUse;
			mManualPara.hs_coordinate.iToolNum = -1;
			mManualPara.hs_coordinate.iWorkNum = -1;

			if(isjoint)
				mManualPara.hs_coordinate.iCoordinate = hsc3::algo::JOINT_COORD_SYSTEM;
			else
				mManualPara.hs_coordinate.iCoordinate = hsc3::algo::BASE_COORD_SYSTEM;

			int iErrorId = 0;
			hsc3::algo::HS_GroupJPos groupjpos = {0.0};
			memcpy(groupjpos.dJPos[0], nowpos, sizeof(double)*MaxAxisNum);
			this->mBaseManualMove->Plan(groupjpos, mManualPara);
		}

		hsc3::algo::HS_MStatus MotionCombine::execManualIntMove(double *jointpos, double *jointvel, double *jointacc, double *spacepos)
		{
			int iErrorId = 0;
			hsc3::algo::HS_MStatus status = hsc3::algo::M_UnInit;
			hsc3::algo::HS_GroupJPos groupjpos = {0.0};
			status = this->mBaseManualMove->Move(iErrorId, groupjpos);					// 获取周期关节插补点
			memcpy(jointpos, groupjpos.dJPos[0], sizeof(double)*6);
			this->mCalibrate->calcJPosToCPos(jointpos, -1, -1, spacepos);				// 获取空间位置

			for(int i=0; i<MaxAxisNum; i++)
			{
				jointvel[i] = (jointpos[i] - this->mLastJointPos[i]) / CYCLE;			// 获取关节速度
				if((jointvel[i] > 300) || (jointvel[i] < -300))
					jointvel[i] = 0.0;

				jointacc[i] = (jointvel[i] - this->mLastVel[i]) / CYCLE;				// 获取关节加速度
				if((jointacc[i] > 1000) || (jointacc[i] < -1000))
					jointacc[i] = 0.0;
			}

			memcpy(this->mLastJointPos, jointpos, sizeof(double)*MaxAxisNum);
			memcpy(this->mLastVel, jointvel, sizeof(double)*MaxAxisNum);

			return status;
		}

		void MotionCombine::stopPlanManual()
		{
			this->mBaseManualMove->StopPlan();
		}

		void MotionCombine::resetMotion()
		{
			this->mAutoMove->execReset();
		}

		int MotionCombine::execPlan(GroupConfigPara *config)
		{
			printf("/------------MotionCombine::execPlan-->ePlanMode=%d, iAxisNum=%d, iIsJoint=%d, iDir=%d \n", config->ePlanMode, config->iAxisNum, config->iIsJoint, config->iDir);
			this->mGroupConfigPara.ePlanMode = config->ePlanMode;
			this->mGroupConfigPara.iAxisNum = config->iAxisNum;
			this->mGroupConfigPara.iIsJoint = config->iIsJoint;
			this->mGroupConfigPara.iDir = config->iDir;
			memcpy(this->mGroupConfigPara.dPos, config->dPos, sizeof(double)*MaxAxisNum);

			switch(config->ePlanMode)
			{
			case Plan_Manual:
				this->planManual(config->iAxisNum, (bool)config->iDir, (bool)config->iIsJoint, this->mGroupFeedbackPara.dFbAxisPos);
				break;
			case Plan_Auto:
				if((bool)config->iIsJoint)
					this->planJoint(config->dPos);
				else
					this->planSpace();
				break;
			case Plan_Stop:
				this->stopPlanManual();
				break;
			default:
				break;
			}

			return 0;
		}

		int MotionCombine::execMove(GroupCommandPara *cmddata, GroupFeedbackPara *fbdata)
		{
			hsc3::algo::HS_MStatus status = hsc3::algo::M_UnInit;

			memcpy(this->mGroupFeedbackPara.dFbAxisPos, fbdata->dFbAxisPos, sizeof(double)*MaxAxisNum);
			memcpy(this->mGroupFeedbackPara.dFbAxisVel, fbdata->dFbAxisVel, sizeof(double)*MaxAxisNum);
			memcpy(this->mGroupFeedbackPara.dFbAxisAcc, fbdata->dFbAxisAcc, sizeof(double)*MaxAxisNum);

			if(this->mGroupConfigPara.ePlanMode == Plan_Manual)
			{
				status = this->execManualIntMove(cmddata->dCmdAxisPos, cmddata->dCmdAxisVel, cmddata->dCmdAxisAcc, cmddata->dCmdSpacePos);
			}
			else if(this->mGroupConfigPara.ePlanMode == Plan_Auto)
			{
				if((bool)this->mGroupConfigPara.iIsJoint)
				{
					status = this->execJointIntMove(cmddata->dCmdAxisPos, cmddata->dCmdAxisVel, cmddata->dCmdAxisAcc, cmddata->dCmdSpacePos);
				}
				else
				{
					status = this->execSpaceIntMove(cmddata->dCmdAxisPos, cmddata->dCmdAxisVel, cmddata->dCmdAxisAcc, cmddata->dCmdSpacePos);
				}
			}
			else if(this->mGroupConfigPara.ePlanMode == Plan_Stop)
			{
				status = this->execManualIntMove(cmddata->dCmdAxisPos, cmddata->dCmdAxisVel, cmddata->dCmdAxisAcc, cmddata->dCmdSpacePos);
			}

			return (int)status;
		}
	}
}