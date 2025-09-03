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
			this->mRunDataNum = 0;
			this->mRatio = 0.4;
			this->mAxis2DiffPos = 0.0;
			this->mAxisDir = new int[MaxAxisNum];
			this->mJointPos = new double[MaxAxisNum];
			this->mLastJointPos = new double[MaxAxisNum];
			this->mLastVel = new double[MaxAxisNum];
			this->mGroupMotionData = new hsc3::algo::GroupMotionData[6];
			memset(this->mGroupMotionData, 0.0, sizeof(double)*6);
			this->mGroupTrajout = new hsc3::algo::GroupTrajData[40];
			memset(this->mAxisDir, 0.0, sizeof(int)*MaxAxisNum);
			memset(this->mJointPos, 0.0, sizeof(double)*MaxAxisNum);
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
			this->mGroupConfigPara.dRatio = 30;
			memset(this->mGroupConfigPara.dPos, 0.0, sizeof(double)*MaxAxisNum);

			this->mGroupCommandPara.iCmdWord = 0;
			this->mGroupCommandPara.iCmdErr = 0;
			memset(this->mGroupCommandPara.dCmdAxisPos, 0.0, sizeof(double)*MaxAxisNum);
			memset(this->mGroupCommandPara.dCmdAxisVel, 0.0, sizeof(double)*MaxAxisNum);
			memset(this->mGroupCommandPara.dCmdAxisAcc, 0.0, sizeof(double)*MaxAxisNum);
			memset(this->mGroupCommandPara.dCmdSpacePos, 0.0, sizeof(double)*MaxAxisNum);

			this->mGroupFeedbackPara.iStatus = 0;
			this->mGroupFeedbackPara.iServoErr = 0;
			memset(this->mGroupFeedbackPara.dFbAxisPos, 0.0, sizeof(double)*MaxAxisNum);
			memset(this->mGroupFeedbackPara.dFbAxisVel, 0.0, sizeof(double)*MaxAxisNum);
			memset(this->mGroupFeedbackPara.dFbAxisAcc, 0.0, sizeof(double)*MaxAxisNum);
			memset(this->mGroupFeedbackPara.dFbSpace, 0.0, sizeof(double)*MaxAxisNum);
		}

		MotionCombine::~MotionCombine()
		{
			delete []mAxisDir;
			delete []mJointPos;
			delete []mLastJointPos;
			delete []mLastVel;
			// delete []mGroupTrajout;
			// delete mMotionPara;
			// delete mAutoMove;
			// delete mBaseManualMove;
			// delete mCalibrate;
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
			mGroupStaticPara[0].tGroupVelocityPara.dVtran = 170.0;
			mGroupStaticPara[0].tGroupVelocityPara.dVrot = 10.0;
			mGroupStaticPara[0].tGroupVelocityPara.dVtranacc = 5.0;
			mGroupStaticPara[0].tGroupVelocityPara.dVrotacc = 5.0;
			mGroupStaticPara[0].tGroupVelocityPara.dJerkrat = 9.0;
			mGroupStaticPara[0].tGroupVelocityPara.dTFreMin = 0.05;
			mGroupStaticPara[0].tGroupVelocityPara.dTFreMax = 0.4;

			mGroupStaticPara[0].tGroupModelPara.DHPara[0][0] = 0.0;    mGroupStaticPara[0].tGroupModelPara.DHPara[0][1] = 0.0;  mGroupStaticPara[0].tGroupModelPara.DHPara[0][2] = -90.0; mGroupStaticPara[0].tGroupModelPara.DHPara[0][3] = 0.0;
			mGroupStaticPara[0].tGroupModelPara.DHPara[1][0] = 1700.0; mGroupStaticPara[0].tGroupModelPara.DHPara[1][1] = 0.0;  mGroupStaticPara[0].tGroupModelPara.DHPara[1][2] = 0.0; mGroupStaticPara[0].tGroupModelPara.DHPara[1][3] = 0.0;
			mGroupStaticPara[0].tGroupModelPara.DHPara[2][0] = 0.0;    mGroupStaticPara[0].tGroupModelPara.DHPara[2][1] = 0.0;  mGroupStaticPara[0].tGroupModelPara.DHPara[2][2] = 90.0; mGroupStaticPara[0].tGroupModelPara.DHPara[2][3] = 0.0;
			mGroupStaticPara[0].tGroupModelPara.DHPara[3][0] = 0.0;    mGroupStaticPara[0].tGroupModelPara.DHPara[3][1] = 1700.0; mGroupStaticPara[0].tGroupModelPara.DHPara[3][2] = -90.0; mGroupStaticPara[0].tGroupModelPara.DHPara[3][3] = 0.0;
			mGroupStaticPara[0].tGroupModelPara.DHPara[4][0] = 0.0;    mGroupStaticPara[0].tGroupModelPara.DHPara[4][1] = 0.0;   mGroupStaticPara[0].tGroupModelPara.DHPara[4][2] = 90.0; mGroupStaticPara[0].tGroupModelPara.DHPara[4][3] = 0.0;
			mGroupStaticPara[0].tGroupModelPara.DHPara[5][0] = 0.0;    mGroupStaticPara[0].tGroupModelPara.DHPara[5][1] = 140.0; mGroupStaticPara[0].tGroupModelPara.DHPara[5][2] = 0.0; mGroupStaticPara[0].tGroupModelPara.DHPara[5][3] = 0.0;

			memset(mGroupStaticPara[0].dWorldCoord, 0, sizeof(double) * 6);

			this->mAxisDir[0] = 1; this->mAxisDir[1] = 1;
			this->mAxisDir[2] = 1; this->mAxisDir[3] = 1;

			for(int i=0; i<MAXCOORDNUM; i++)
			{
				memset(mGroupStaticPara[0].dToolCoord[i], 0, sizeof(double) * 6);
				memset(mGroupStaticPara[0].dWorkCoord[i], 0, sizeof(double) * 6);
			}

			for(int i=0; i<MaxAxisNum; i++)
			{
				mGroupStaticPara[0].tAxisVelocityPara.dVmax[i] = 25.0;
				mGroupStaticPara[0].tAxisVelocityPara.dVcruise[i] = 20.0;
				mGroupStaticPara[0].tAxisVelocityPara.dAccelerate[i] = 200.0;
				mGroupStaticPara[0].tAxisVelocityPara.dJerkrat[i] = 40.0;
				mGroupStaticPara[0].tLimitPara.dPmax[i] = 360.0;
				mGroupStaticPara[0].tLimitPara.dPmin[i] = -360.0;
				mGroupStaticPara[0].tLimitPara.bOpen[i] = true;
			}
		}

		int MotionCombine::planMoveTo(double *endpos, bool isjoint)
		{
			int iErrorID = 0;
			int mMotionDataNum = 0;
			hsc3::algo::GroupMotionData groupdata = {0};
			//memset(this->mLastJointPos, 0.0, sizeof(double)*MaxAxisNum);
			//memset(this->mLastVel, 0.0, sizeof(double)*MaxAxisNum);

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
			
			groupdata.tBaseMoveData[0].sCurCoordinate.iToolNum = -1;
			groupdata.tBaseMoveData[0].sCurCoordinate.iWorkNum = -1;
			memcpy(groupdata.tBaseMoveData[0].sStartPos.dPos, this->mGroupFeedbackPara.dFbAxisPos, sizeof(double) * MaxAxisNum);	// 六轴点位
			groupdata.tBaseMoveData[0].sStartPos.iPose = 0;
			groupdata.tBaseMoveData[0].sStartPos.hs_coordinate.iCoordinate = hsc3::algo::JOINT_COORD_SYSTEM;
			groupdata.tBaseMoveData[0].sStartPos.hs_coordinate.iToolNum = -1;
			groupdata.tBaseMoveData[0].sStartPos.hs_coordinate.iWorkNum = -1;
			memset(groupdata.tBaseMoveData[0].sMidPos.dPos, 0, sizeof(double) * 9);
			groupdata.tBaseMoveData[0].sEndPos.iPose = 0;
			groupdata.tBaseMoveData[0].sEndPos.hs_coordinate.iToolNum = -1;
			groupdata.tBaseMoveData[0].sEndPos.hs_coordinate.iWorkNum = -1;
			groupdata.tBaseMoveData[0].b2mid = false;
			groupdata.tBaseMoveData[0].dVort = 100.0;
			groupdata.tBaseMoveData[0].dAcc = 100.0;
			groupdata.tBaseMoveData[0].dDec = 100.0;
			groupdata.tBaseMoveData[0].iCntType = 0;
			groupdata.tBaseMoveData[0].tRevolve.iTurn = 0;

			if(isjoint)
			{
				groupdata.tBaseMoveData[0].eTrajType = hsc3::algo::MP_Joint;
				groupdata.tBaseMoveData[0].sCurCoordinate.iCoordinate = hsc3::algo::JOINT_COORD_SYSTEM;
				groupdata.tBaseMoveData[0].sEndPos.hs_coordinate.iCoordinate = hsc3::algo::JOINT_COORD_SYSTEM;
				groupdata.tBaseMoveData[0].dVel = 100.0;
				groupdata.tBaseMoveData[0].sEndPos.dPos[0] = endpos[0]; groupdata.tBaseMoveData[0].sEndPos.dPos[1] = endpos[1]; groupdata.tBaseMoveData[0].sEndPos.dPos[2] = endpos[2]/* - this->mAxis2DiffPos*/;
				groupdata.tBaseMoveData[0].sEndPos.dPos[3] = 0.0; groupdata.tBaseMoveData[0].sEndPos.dPos[4] = endpos[3]; groupdata.tBaseMoveData[0].sEndPos.dPos[5] = 0.0;
				groupdata.tBaseMoveData[0].sEndPos.dPos[6] = 0.0; groupdata.tBaseMoveData[0].sEndPos.dPos[7] = 0.0; groupdata.tBaseMoveData[0].sEndPos.dPos[8] = 0.0;
			}
			else
			{
				groupdata.tBaseMoveData[0].eTrajType = hsc3::algo::MP_Line;
				groupdata.tBaseMoveData[0].sCurCoordinate.iCoordinate = hsc3::algo::BASE_COORD_SYSTEM;
				groupdata.tBaseMoveData[0].sEndPos.hs_coordinate.iCoordinate = hsc3::algo::BASE_COORD_SYSTEM;
				groupdata.tBaseMoveData[0].dVel = 1500.0;
				groupdata.tBaseMoveData[0].sEndPos.dPos[0] = endpos[0]; groupdata.tBaseMoveData[0].sEndPos.dPos[1] = endpos[1]; groupdata.tBaseMoveData[0].sEndPos.dPos[2] = endpos[2];
				groupdata.tBaseMoveData[0].sEndPos.dPos[3] = 180.0; groupdata.tBaseMoveData[0].sEndPos.dPos[4] = endpos[3]; groupdata.tBaseMoveData[0].sEndPos.dPos[5] = 180.0;
				groupdata.tBaseMoveData[0].sEndPos.dPos[6] = 0.0; groupdata.tBaseMoveData[0].sEndPos.dPos[7] = 0.0; groupdata.tBaseMoveData[0].sEndPos.dPos[8] = 0.0;
			}

			iErrorID = this->mAutoMove->execPrehandle(groupdata, this->mGroupTrajout, mMotionDataNum);
			if(iErrorID != 0)
				return iErrorID;

			hsc3::algo::HS_GroupJPos groupjpos = {0};
			memcpy(groupjpos.dJPos[0], groupdata.tBaseMoveData[0].sStartPos.dPos, sizeof(double) * MaxAxisNum);

			return this->mAutoMove->execPlanMove(this->mGroupTrajout, 0, this->mRatio, groupjpos);
		}

		hsc3::algo::HS_MStatus MotionCombine::execMoveToIntMove(double *jointpos, double *jointvel, double *jointacc, double *spacepos)
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

			//memcpy(this->mLastJointPos, jointpos, sizeof(double)*MaxAxisNum);
			//memcpy(this->mLastVel, jointvel, sizeof(double)*MaxAxisNum);

			return status;
		}

		hsc3::algo::GroupMotionData MotionCombine::dealElemt(bool firstmove, int linenum, bool isjoint, double *startpos, double *endpos)
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

			groupdata.tBaseMoveData[0].sCurCoordinate.iToolNum = -1;
			groupdata.tBaseMoveData[0].sCurCoordinate.iWorkNum = -1;
			groupdata.tBaseMoveData[0].sStartPos.iPose = 0;
			groupdata.tBaseMoveData[0].sStartPos.hs_coordinate.iToolNum = -1;
			groupdata.tBaseMoveData[0].sStartPos.hs_coordinate.iWorkNum = -1;

			if(isjoint)
			{
				groupdata.tBaseMoveData[0].eTrajType = hsc3::algo::MP_Joint;
				groupdata.tBaseMoveData[0].sCurCoordinate.iCoordinate = hsc3::algo::JOINT_COORD_SYSTEM;
				groupdata.tBaseMoveData[0].sStartPos.hs_coordinate.iCoordinate = hsc3::algo::JOINT_COORD_SYSTEM;
				groupdata.tBaseMoveData[0].sEndPos.hs_coordinate.iCoordinate = hsc3::algo::JOINT_COORD_SYSTEM;
				groupdata.tBaseMoveData[0].dVel = 100.0;
			} 
			else
			{
				groupdata.tBaseMoveData[0].eTrajType = hsc3::algo::MP_Line;
				groupdata.tBaseMoveData[0].sCurCoordinate.iCoordinate = hsc3::algo::BASE_COORD_SYSTEM;
				groupdata.tBaseMoveData[0].sStartPos.hs_coordinate.iCoordinate = hsc3::algo::BASE_COORD_SYSTEM;
				groupdata.tBaseMoveData[0].sEndPos.hs_coordinate.iCoordinate = hsc3::algo::BASE_COORD_SYSTEM;
				groupdata.tBaseMoveData[0].dVel = 1500.0;
			}

			if(firstmove)
			{
				groupdata.tBaseMoveData[0].sStartPos.hs_coordinate.iCoordinate = hsc3::algo::JOINT_COORD_SYSTEM;
				memcpy(groupdata.tBaseMoveData[0].sStartPos.dPos, this->mGroupCommandPara.dCmdAxisPos, sizeof(double) * MaxAxisNum);	// 六轴点位
			}
			else
			{
				groupdata.tBaseMoveData[0].sStartPos.hs_coordinate.iCoordinate = hsc3::algo::BASE_COORD_SYSTEM;
				memcpy(groupdata.tBaseMoveData[0].sStartPos.dPos, startpos, sizeof(double)*MaxAxisNum);
			}

			memset(groupdata.tBaseMoveData[0].sMidPos.dPos, 0, sizeof(double) * 9);
			memcpy(groupdata.tBaseMoveData[0].sEndPos.dPos, endpos, sizeof(double)*MaxAxisNum);
			groupdata.tBaseMoveData[0].sEndPos.iPose = 0;
			groupdata.tBaseMoveData[0].sEndPos.hs_coordinate.iToolNum = -1;
			groupdata.tBaseMoveData[0].sEndPos.hs_coordinate.iWorkNum = -1;
			groupdata.tBaseMoveData[0].b2mid = false;
			groupdata.tBaseMoveData[0].dVort = 100.0;
			groupdata.tBaseMoveData[0].dAcc = 100.0;
			groupdata.tBaseMoveData[0].dDec = 100.0;
			groupdata.tBaseMoveData[0].iCntType = 0;
			groupdata.tBaseMoveData[0].tRevolve.iTurn = 0;

			return groupdata;
		}

		int MotionCombine::planAutoRun()
		{
			bool bJoint = true;
			//memset(this->mLastJointPos, 0.0, sizeof(double)*MaxAxisNum);
			//memset(this->mLastVel, 0.0, sizeof(double)*MaxAxisNum);
			hsc3::algo::GroupMotionData groupdata = {0};

			//bJoint = false;
			//double dEndPos1[MaxAxisNum] = {1200.0, 500.0, 1300.0, 180.0, 0.0, 180.0, 60.0, 0.0, 0.0};
			//double dEndPos2[MaxAxisNum] = {1500.0, 1500.0, 1100.0, 180.0, 0.0, 180.0, -30.0, 0.0, 0.0};
			//double dEndPos3[MaxAxisNum] = {1100, -1200.0, 1700.0, 180.0, 0.0, 180.0, 60.0, 0.0, 0.0};
			//double dEndPos4[MaxAxisNum] = {1700, 0.0, 1560.0, 180.0, 0.0, 180.0, 30.0, 0.0, 0.0};

			//bJoint = false;
			//double dEndPos1[MaxAxisNum] = {1700.0, 0.0, 1400.0, 180.0, 0.0, 180.0, 60.0, 0.0, 0.0};
			//double dEndPos2[MaxAxisNum] = {1700.0, 100.0, 1400.0, 180.0, 0.0, 180.0, -30.0, 0.0, 0.0};
			//double dEndPos3[MaxAxisNum] = {1500, 100.0, 1400.0, 180.0, 0.0, 180.0, 60.0, 0.0, 0.0};
			//double dEndPos4[MaxAxisNum] = {1500, 0.0, 1400.0, 180.0, 0.0, 180.0, 30.0, 0.0, 0.0};

			bJoint = true;
			double dEndPos1[MaxAxisNum] = {-30.0, -104.0, 195.0, 0.0, 60.0, 0.0, 0.0, 0.0, 0.0};
			double dEndPos2[MaxAxisNum] = {60.0, -80.0, 152.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
			double dEndPos3[MaxAxisNum] = {30.0, -74.0, 182.0, 0.0, -40.0, 0.0, 0.0, 0.0, 0.0};
			double dEndPos4[MaxAxisNum] = {0.0, -90.0, 180.0, 0.0, -90.0, 0.0, 0.0, 0.0, 0.0};

			memcpy(this->mJointPos, this->mGroupCommandPara.dCmdAxisPos, sizeof(double)*MaxAxisNum);

			this->mDataNum = 0;
			this->mGroupMotionData[0] = this->dealElemt(true, this->mDataNum, bJoint, dEndPos1, dEndPos1); // 首次启动起点为当前点

			this->mDataNum = 1;
			this->mGroupMotionData[1] = this->dealElemt(false, this->mDataNum, bJoint, dEndPos1, dEndPos2);

			this->mDataNum = 2;
			this->mGroupMotionData[2] = this->dealElemt(false, this->mDataNum, bJoint, dEndPos2, dEndPos3);

			this->mDataNum = 3;
			this->mGroupMotionData[3] = this->dealElemt(false, this->mDataNum, bJoint, dEndPos3, dEndPos4);
			return 0;
		}

		hsc3::algo::HS_MStatus MotionCombine::execAutoRunIntMove(double *jointpos, double *jointvel, double *jointacc, double *spacepos)
		{
			int errorID = 0;
			bool bCalcOut = false;
			hsc3::algo::IntData intdata = {0.0};
			hsc3::algo::HS_GroupJPos groupjpos = {0.0};
			hsc3::algo::HS_MStatus status = hsc3::algo::M_UnInit;
			status = this->mAutoMove->execIntMove(intdata, errorID);					// 获取周期关节插补点
			
			switch(status)
			{
				case hsc3::algo::M_UnInit:
					if(this->mRunDataNum <= this->mDataNum)
					{
						bCalcOut = true;
						memcpy(groupjpos.dJPos[0], this->mJointPos, sizeof(double) * MaxAxisNum);
						this->mAutoMove->execPrehandle(this->mGroupMotionData[this->mRunDataNum], this->mGroupTrajout, this->mRunDataNum);
						errorID = this->mAutoMove->execPlanMove(this->mGroupTrajout, this->mRunDataNum, this->mRatio, groupjpos);
						if((errorID != 0) && (errorID < Waring))
							status = hsc3::algo::M_Error;
						printf("MotionCombine-->execAutoRunIntMove-->M_UnInit-->Plan Next Position\n");
					}
					break;
				case hsc3::algo::M_Busy:
					bCalcOut = true;
					memcpy(this->mJointPos, intdata.tGJPos[0].dJPos[0], sizeof(double)*MaxAxisNum);
					break;
				case hsc3::algo::M_Done:
					this->mRunDataNum = this->mRunDataNum + 1;		// 已在M_UnInit做首次规划
					if(this->mRunDataNum <= this->mDataNum)
					{
						bCalcOut = true;
						memcpy(groupjpos.dJPos[0], intdata.tGJPos[0].dJPos[0], sizeof(double) * MaxAxisNum);
						this->mAutoMove->execPrehandle(this->mGroupMotionData[this->mRunDataNum], this->mGroupTrajout, this->mRunDataNum);
						errorID = this->mAutoMove->execPlanMove(this->mGroupTrajout, this->mRunDataNum, this->mRatio, groupjpos);
						
						printf("MotionCombine-->execAutoRunIntMove-->Plan Next Position-->DataNum=%d, errorID=%d \n", this->mRunDataNum, errorID);
						if((errorID != 0) && (errorID < Waring))
							status = hsc3::algo::M_Error;
						status = hsc3::algo::M_UnInit;
						memcpy(this->mJointPos, intdata.tGJPos[0].dJPos[0], sizeof(double)*MaxAxisNum);
						
						goto outres;
					}
					this->mRunDataNum = 0;
					printf("MotionCombine-->execAutoRunIntMove->Plan Finish M_Done\n");
					return hsc3::algo::M_Done;
					break;
				case hsc3::algo::M_Error:
					printf("MotionCombine-->execAutoRunIntMove->IntMove Error\n");
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

				//memcpy(this->mLastJointPos, jointpos, sizeof(double)*MaxAxisNum);
				//memcpy(this->mLastVel, jointvel, sizeof(double)*MaxAxisNum);
			}

			return status;
		}

		int MotionCombine::planManual(int axisnum, bool dir, bool isjoint, double *nowpos)
		{
			int iErrorID = 0;
			printf("MotionCombine-->planManual nowpos %f %f %f %f %f %f \n",nowpos[0],nowpos[1],nowpos[2],nowpos[3],nowpos[4],nowpos[5]);
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
			iErrorId = this->mBaseManualMove->Plan(groupjpos, mManualPara);
			printf("MotionCombine-->planManual iErrorId = %d \n",iErrorId);
			return iErrorId;
		}

		hsc3::algo::HS_MStatus MotionCombine::execManualIntMove(double *jointpos, double *jointvel, double *jointacc, double *spacepos, int &errid)
		{
			int iErrorId = 0;
			hsc3::algo::HS_MStatus status = hsc3::algo::M_UnInit;
			hsc3::algo::HS_GroupJPos groupjpos = {0.0};
			status = this->mBaseManualMove->Move(iErrorId, groupjpos);					// 获取周期关节插补点
			memcpy(jointpos, groupjpos.dJPos[0], sizeof(double)*6);
			this->mCalibrate->calcJPosToCPos(jointpos, -1, -1, spacepos);				// 获取空间位置
			if(iErrorId != 0)
			{
				errid = iErrorId;
				printf("MotionCombine::execManualIntMove %d \n", iErrorId);
			}

			for(int i=0; i<MaxAxisNum; i++)
			{
				jointvel[i] = (jointpos[i] - this->mLastJointPos[i]) / CYCLE;			// 获取关节速度
				if((jointvel[i] > 300) || (jointvel[i] < -300))
					jointvel[i] = 0.0;

				jointacc[i] = (jointvel[i] - this->mLastVel[i]) / CYCLE;				// 获取关节加速度
				if((jointacc[i] > 1000) || (jointacc[i] < -1000))
					jointacc[i] = 0.0;
			}

			//memcpy(this->mLastJointPos, jointpos, sizeof(double)*MaxAxisNum);
			//memcpy(this->mLastVel, jointvel, sizeof(double)*MaxAxisNum);

			return status;
		}

		int MotionCombine::stopPlanManual()
		{
			return this->mBaseManualMove->StopPlan();
		}

		int MotionCombine::stopPlanAuto()
		{
			printf("MotionCombine-->stopPlanAuto\n");
			return this->mAutoMove->execStopPlan();
		}

		void MotionCombine::resetMotion()
		{
			printf("MotionCombine-->resetMotion\n");
			this->mRunDataNum = 0;
			this->mAutoMove->execReset();
			memset(this->mGroupMotionData, 0.0, sizeof(double)*6);
			memset(this->mGroupTrajout, 0.0, sizeof(hsc3::algo::GroupTrajData) * 40);
		}

		int MotionCombine::execPlan(GroupConfigPara *config)
		{
			printf("MotionCombine-->execPlan-->ePlanMode=%d, iAxisNum=%d, iIsJoint=%d, dRatio=%f, iDir=%d \n", config->ePlanMode, config->iAxisNum, config->iIsJoint, config->dRatio,config->iDir);
			int iErrorNum = 0;
			this->mGroupConfigPara.ePlanMode = config->ePlanMode;
			this->mGroupConfigPara.iAxisNum = config->iAxisNum;
			this->mGroupConfigPara.iIsJoint =(bool) config->iIsJoint;
			this->mGroupConfigPara.iDir = (bool)config->iDir;
			this->mGroupConfigPara.dRatio = config->dRatio;
			this->setRatio(config->dRatio);
			memcpy(this->mGroupConfigPara.dPos, config->dPos, sizeof(double)*MaxAxisNum);

			switch(config->ePlanMode)
			{
			case Plan_Manual:
				//this->mAxis2DiffPos = 0.0;
				if(config->iAxisNum > 3)
				{
					printf("MotionCombine-->execPlan-->AxisNum Out Limit!!! \n");
					return 1;
				}
				if(config->iAxisNum == 3)
				{
					this->mGroupConfigPara.iAxisNum = 4;
					this->mGroupConfigPara.iIsJoint = true;
				}
				iErrorNum = this->planManual(this->mGroupConfigPara.iAxisNum, this->mGroupConfigPara.iDir, this->mGroupConfigPara.iIsJoint, this->mGroupFeedbackPara.dFbAxisPos);
				break;
			case Plan_Auto:
				iErrorNum = this->planAutoRun();
				break;
			case Plan_Stop:
				iErrorNum = this->stopPlanManual();
				break;
			case Plan_MoveTo:
				//this->mAxis2DiffPos = 0.0;
				iErrorNum = this->planMoveTo(config->dPos, (bool)config->iIsJoint);
				break;
			case Plan_StopAuto:
				iErrorNum = this->stopPlanAuto();
				break;
			default:
				break;
			}

			return iErrorNum;
		}

		int MotionCombine::execMove(GroupCommandPara *cmddata, GroupFeedbackPara *fbdata)
		{
			int iErrorID = 0;
			double dAxis2CmdPos = 0.0;
			static bool bFirstMove = true;
			GroupCommandPara strCmdData = {0.0};
			GroupFeedbackPara strFbData = {0.0};
			hsc3::algo::HS_MStatus status = hsc3::algo::M_UnInit;

			memset(this->mGroupFeedbackPara.dFbAxisPos, 0.0, sizeof(double)*MaxAxisNum);
			memset(this->mGroupFeedbackPara.dFbAxisVel, 0.0, sizeof(double)*MaxAxisNum);
			memset(this->mGroupFeedbackPara.dFbAxisAcc, 0.0, sizeof(double)*MaxAxisNum);

			fbdata->dFbAxisPos[2] = fbdata->dFbAxisPos[2] + this->mAxis2DiffPos;

			// 转换为六轴点位
			for(int i=0; i<3; i++)
			{
				this->mGroupFeedbackPara.dFbAxisPos[i] = fbdata->dFbAxisPos[i] * this->mAxisDir[i];
				this->mGroupFeedbackPara.dFbAxisVel[i] = fbdata->dFbAxisVel[i] * this->mAxisDir[i];
				this->mGroupFeedbackPara.dFbAxisAcc[i] = fbdata->dFbAxisAcc[i] * this->mAxisDir[i];
			}
			this->mGroupFeedbackPara.dFbAxisPos[4] = fbdata->dFbAxisPos[3] * this->mAxisDir[3];
			this->mGroupFeedbackPara.dFbAxisVel[4] = fbdata->dFbAxisVel[3] * this->mAxisDir[3];
			this->mGroupFeedbackPara.dFbAxisAcc[4] = fbdata->dFbAxisAcc[3] * this->mAxisDir[3];

			this->mCalibrate->calcJPosToCPos(this->mGroupFeedbackPara.dFbAxisPos, -1, -1, fbdata->dFbSpace);				// 更新反馈空间位置

			switch(this->mGroupConfigPara.ePlanMode)
			{
			case Plan_Manual:
				status = this->execManualIntMove(strCmdData.dCmdAxisPos, strCmdData.dCmdAxisVel, strCmdData.dCmdAxisAcc, strCmdData.dCmdSpacePos, iErrorID);
				break;
			case Plan_MoveTo:
				status = this->execMoveToIntMove(strCmdData.dCmdAxisPos, strCmdData.dCmdAxisVel, strCmdData.dCmdAxisAcc, strCmdData.dCmdSpacePos);
				break;
			case Plan_Auto:
				status = this->execAutoRunIntMove(strCmdData.dCmdAxisPos, strCmdData.dCmdAxisVel, strCmdData.dCmdAxisAcc, strCmdData.dCmdSpacePos);
				break;
			case Plan_Stop:
				status = this->execManualIntMove(strCmdData.dCmdAxisPos, strCmdData.dCmdAxisVel, strCmdData.dCmdAxisAcc, strCmdData.dCmdSpacePos, iErrorID);
				break;
			case Plan_StopAuto:
				status = this->execMoveToIntMove(strCmdData.dCmdAxisPos, strCmdData.dCmdAxisVel, strCmdData.dCmdAxisAcc, strCmdData.dCmdSpacePos);
				break;
			}

			if((status != hsc3::algo::M_UnInit) && (status != hsc3::algo::M_Done))
			{
				cmddata->iCmdErr = iErrorID;
				if(iErrorID != 0)
					printf("MotionCombine-->execMove-->status=%d, iErrorID=%d \n", status, iErrorID);

				memset(cmddata->dCmdAxisPos, 0.0, sizeof(double)*MaxAxisNum);
				memset(cmddata->dCmdAxisVel, 0.0, sizeof(double)*MaxAxisNum);
				memset(cmddata->dCmdAxisAcc, 0.0, sizeof(double)*MaxAxisNum);
				memcpy(cmddata->dCmdSpacePos, strCmdData.dCmdSpacePos, sizeof(double)*MaxAxisNum);
				for(int i=0; i<3; i++)
				{
					cmddata->dCmdAxisPos[i] = strCmdData.dCmdAxisPos[i] * this->mAxisDir[i];
					cmddata->dCmdAxisVel[i] = strCmdData.dCmdAxisVel[i] * this->mAxisDir[i];
					cmddata->dCmdAxisAcc[i] = strCmdData.dCmdAxisAcc[i] * this->mAxisDir[i];
				}
				cmddata->dCmdAxisPos[3] = strCmdData.dCmdAxisPos[4] * this->mAxisDir[3];
				cmddata->dCmdAxisVel[3] = strCmdData.dCmdAxisVel[4] * this->mAxisDir[3];
				cmddata->dCmdAxisAcc[3] = strCmdData.dCmdAxisAcc[4] * this->mAxisDir[3];
				if(bFirstMove == true)
				{
					this->mLastJointPos[1] = strCmdData.dCmdAxisPos[1];
					bFirstMove = false;
					printf("/-----MotionCombine::execMove first mAxis2DiffPos %f %f %f\n", this->mAxis2DiffPos, strCmdData.dCmdAxisPos[1], this->mLastJointPos[1]);
				}
				this->mAxis2DiffPos = this->mAxis2DiffPos - (strCmdData.dCmdAxisPos[1] - this->mLastJointPos[1]);
				cmddata->dCmdAxisPos[2] = cmddata->dCmdAxisPos[2] - this->mAxis2DiffPos;
				printf("/-----MotionCombine::execMove mAxis2DiffPos %f %f %f\n", this->mAxis2DiffPos, strCmdData.dCmdAxisPos[1], this->mLastJointPos[1]);
				memcpy(this->mGroupCommandPara.dCmdAxisPos, strCmdData.dCmdAxisPos, sizeof(double)*MaxAxisNum); // 缓存六轴位置
				memcpy(this->mGroupCommandPara.dCmdAxisVel, strCmdData.dCmdAxisVel, sizeof(double)*MaxAxisNum);
				memcpy(this->mGroupCommandPara.dCmdAxisAcc, strCmdData.dCmdAxisAcc, sizeof(double)*MaxAxisNum);
				memcpy(this->mGroupCommandPara.dCmdSpacePos, strCmdData.dCmdSpacePos, sizeof(double)*MaxAxisNum);


			}
			// 触发运动停止完成
			if(status == M_StopDone)
			{
				this->resetMotion();
				printf("MotionCombine-->execMove-->M_StopDone \n");
			}

			memcpy(this->mLastJointPos, cmddata->dCmdAxisPos, sizeof(double)*MaxAxisNum);
			memcpy(this->mLastVel, cmddata->dCmdAxisPos, sizeof(double)*MaxAxisNum);
			//printf("/-----MotionCombine::execMove mAxis2DiffPos---- %f %f %f\n",strCmdData.dCmdAxisPos[0] , strCmdData.dCmdAxisPos[1], strCmdData.dCmdAxisPos[2]);
			return (int)status;
		}

		void MotionCombine::syncPos()
		{
			memcpy(this->mGroupCommandPara.dCmdAxisPos, this->mGroupFeedbackPara.dFbAxisPos, sizeof(double)*MaxAxisNum);
			memcpy(this->mGroupCommandPara.dCmdAxisVel, this->mGroupFeedbackPara.dFbAxisVel, sizeof(double)*MaxAxisNum);
			memcpy(this->mGroupCommandPara.dCmdAxisAcc, this->mGroupFeedbackPara.dFbAxisAcc, sizeof(double)*MaxAxisNum);
		}
	}
}