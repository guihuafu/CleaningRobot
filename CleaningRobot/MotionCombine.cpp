#include "stdafx.h"
#include "MotionCombine.h"

namespace hsc3
{
	namespace algo
	{
		MotionCombine::MotionCombine()
		{
			this->initPara();
			this->mMotionPara = new hsc3::algo::MotionPara();
			this->mMotionPara->setGroupStaticPara(mGroupStaticPara);
			this->mAutoMove = new hsc3::algo::BaseAutoMove(mMotionPara, CYCLE, 2);
			this->mBaseManualMove = new hsc3::algo::BaseManualMove(mMotionPara, CYCLE);
			this->mCalibrate = new hsc3::algo::Calibrate(mMotionPara, CYCLE, 0);
			
		}

		MotionCombine::~MotionCombine(){}

		void MotionCombine::initPara()
		{
			mGroupStaticPara[0].tGroupModelPara.eRobtype = hsc3::algo::HS_RobotType::HSROB_PUMA;
			mGroupStaticPara[0].tGroupModelPara.eRobtype_sub = hsc3::algo::HS_RobotType_sub::HSROB_SUB_NONE;
			mGroupStaticPara[0].tGroupVelocityPara.dVtran = 1700.0;
			mGroupStaticPara[0].tGroupVelocityPara.dVrot = 50.0;
			mGroupStaticPara[0].tGroupVelocityPara.dVtranacc = 100.0;
			mGroupStaticPara[0].tGroupVelocityPara.dVrotacc = 100.0;
			mGroupStaticPara[0].tGroupVelocityPara.dJerkrat = 9.0;
			mGroupStaticPara[0].tGroupVelocityPara.dTFreMin = 0.05;
			mGroupStaticPara[0].tGroupVelocityPara.dTFreMax = 0.4;
			mGroupStaticPara[0].tGroupModelPara.DHPara[0][0] = 0.0; mGroupStaticPara[0].tGroupModelPara.DHPara[0][1] = 0.0; mGroupStaticPara[0].tGroupModelPara.DHPara[0][2] = -90.0; mGroupStaticPara[0].tGroupModelPara.DHPara[0][3] = 0.0;
			mGroupStaticPara[0].tGroupModelPara.DHPara[1][0] = 360.0; mGroupStaticPara[0].tGroupModelPara.DHPara[1][1] = 0.0; mGroupStaticPara[0].tGroupModelPara.DHPara[1][2] = 0.0; mGroupStaticPara[0].tGroupModelPara.DHPara[1][3] = 0.0;
			mGroupStaticPara[0].tGroupModelPara.DHPara[2][0] = -90.0; mGroupStaticPara[0].tGroupModelPara.DHPara[2][1] = 0.0; mGroupStaticPara[0].tGroupModelPara.DHPara[2][2] = 90.0; mGroupStaticPara[0].tGroupModelPara.DHPara[2][3] = 0.0;
			mGroupStaticPara[0].tGroupModelPara.DHPara[3][0] = 0.0; mGroupStaticPara[0].tGroupModelPara.DHPara[3][1] = 376.5; mGroupStaticPara[0].tGroupModelPara.DHPara[3][2] = -90.0; mGroupStaticPara[0].tGroupModelPara.DHPara[3][3] = 0.0;
			mGroupStaticPara[0].tGroupModelPara.DHPara[4][0] = 0.0; mGroupStaticPara[0].tGroupModelPara.DHPara[4][1] = 0.0; mGroupStaticPara[0].tGroupModelPara.DHPara[4][2] = 90.0; mGroupStaticPara[0].tGroupModelPara.DHPara[4][3] = 0.0;
			mGroupStaticPara[0].tGroupModelPara.DHPara[5][0] = 0.0; mGroupStaticPara[0].tGroupModelPara.DHPara[5][1] = 119.0; mGroupStaticPara[0].tGroupModelPara.DHPara[5][2] = 0.0; mGroupStaticPara[0].tGroupModelPara.DHPara[5][3] = 0.0;
			memset(mGroupStaticPara[0].dWorldCoord, 0, sizeof(double) * 6);
			memset(&mGroupStaticPara[0].dFlTrackCoord, 0, sizeof(hsc3::algo::HS_Coord));
			memset(&mGroupStaticPara[0].dPoCoord, 0, sizeof(hsc3::algo::HS_Coord));

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

		void MotionCombine::planJoint()
		{
			double dRatio = 0.3;
			int mMotionDataNum = 0;
			hsc3::algo::GroupMotionData groupdata = {0};
			hsc3::algo::GroupTrajData *mGroupTrajout = new hsc3::algo::GroupTrajData[40];
			memset(mGroupTrajout, 0, sizeof(hsc3::algo::GroupTrajData) * 40);

			groupdata.iLineNum = 0;
			groupdata.tHS_GroupRel.eGroupRelType[0] = hsc3::algo::GroupRelType::GRT_Independent;
			groupdata.tHS_GroupRel.eGroupRelType[1] = hsc3::algo::GroupRelType::GRT_NoUse;
			groupdata.tHS_GroupRel.eGroupRelType[2] = hsc3::algo::GroupRelType::GRT_NoUse;
			groupdata.tHS_GroupRel.eGroupRelType[3] = hsc3::algo::GroupRelType::GRT_NoUse;
			groupdata.dCnt = 0.0;
			groupdata.dCR = 0.0;
			groupdata.iSmooth = 0;
			groupdata.bExtTool = false;
			groupdata.bStartMove = true;
			groupdata.bWristQYFlag = false;
			groupdata.tFilterControl.bFilterOpenFlag = false;
			groupdata.tBaseMoveData[0].eTrajType = hsc3::algo::HS_MOVETYPE::MP_Joint;
			groupdata.tBaseMoveData[0].sCurCoordinate.iCoordinate = hsc3::algo::COORD_SYSTEM::JOINT_COORD_SYSTEM;
			groupdata.tBaseMoveData[0].sCurCoordinate.iToolNum = -1;
			groupdata.tBaseMoveData[0].sCurCoordinate.iWorkNum = -1;
			groupdata.tBaseMoveData[0].sCurCoordinate.bExtTool = -1;
			groupdata.tBaseMoveData[0].sCurCoordinate.bExtCoorper = -1;
			groupdata.tBaseMoveData[0].sStartPos.dPos[0] = 0; groupdata.tBaseMoveData[0].sStartPos.dPos[1] = -90; groupdata.tBaseMoveData[0].sStartPos.dPos[2] = 180;
			groupdata.tBaseMoveData[0].sStartPos.dPos[3] = 0; groupdata.tBaseMoveData[0].sStartPos.dPos[4] = 90; groupdata.tBaseMoveData[0].sStartPos.dPos[5] = 0;
			groupdata.tBaseMoveData[0].sStartPos.dPos[6] = 0; groupdata.tBaseMoveData[0].sStartPos.dPos[7] = 0; groupdata.tBaseMoveData[0].sStartPos.dPos[8] = 0;
			groupdata.tBaseMoveData[0].sStartPos.iPose = 0;
			groupdata.tBaseMoveData[0].sStartPos.hs_coordinate.iCoordinate = hsc3::algo::COORD_SYSTEM::JOINT_COORD_SYSTEM;
			groupdata.tBaseMoveData[0].sStartPos.hs_coordinate.iToolNum = -1;
			groupdata.tBaseMoveData[0].sStartPos.hs_coordinate.iWorkNum = -1;
			groupdata.tBaseMoveData[0].sStartPos.hs_coordinate.bExtTool = false;
			groupdata.tBaseMoveData[0].sStartPos.hs_coordinate.bExtCoorper = false;
			memset(groupdata.tBaseMoveData[0].sMidPos.dPos, 0, sizeof(double) * 9);
			groupdata.tBaseMoveData[0].sEndPos.dPos[0] = 70; groupdata.tBaseMoveData[0].sEndPos.dPos[1] = -30; groupdata.tBaseMoveData[0].sEndPos.dPos[2] = 140;
			groupdata.tBaseMoveData[0].sEndPos.dPos[3] = 30; groupdata.tBaseMoveData[0].sEndPos.dPos[4] = 90; groupdata.tBaseMoveData[0].sEndPos.dPos[5] = 0;
			groupdata.tBaseMoveData[0].sEndPos.dPos[6] = 0; groupdata.tBaseMoveData[0].sEndPos.dPos[7] = 0; groupdata.tBaseMoveData[0].sEndPos.dPos[8] = 0;
			groupdata.tBaseMoveData[0].sEndPos.iPose = 0;
			groupdata.tBaseMoveData[0].sEndPos.hs_coordinate.iCoordinate = hsc3::algo::COORD_SYSTEM::JOINT_COORD_SYSTEM;
			groupdata.tBaseMoveData[0].sEndPos.hs_coordinate.iToolNum = -1;
			groupdata.tBaseMoveData[0].sEndPos.hs_coordinate.iWorkNum = -1;
			groupdata.tBaseMoveData[0].sEndPos.hs_coordinate.bExtTool = false;
			groupdata.tBaseMoveData[0].sEndPos.hs_coordinate.bExtCoorper = false;
			groupdata.tBaseMoveData[0].b2mid = false;
			groupdata.tBaseMoveData[0].dVel = 100.0;
			groupdata.tBaseMoveData[0].dVort = 100.0;
			groupdata.tBaseMoveData[0].dAcc = 100.0;
			groupdata.tBaseMoveData[0].dDec = 100.0;
			groupdata.tBaseMoveData[0].iCntType = 0;
			groupdata.tBaseMoveData[0].tRevolve.iTurn = 0;
			groupdata.tBaseMoveData[0].bCoorperMove = false;
			groupdata.tBaseMoveData[0].tWeavePara.bWeaveEn = false;

			this->mAutoMove->execPrehandle(groupdata, mGroupTrajout, mMotionDataNum);
			hsc3::algo::HS_GroupJPos groupjpos = {0};
			memcpy(groupjpos.dJPos[0], groupdata.tBaseMoveData[0].sStartPos.dPos, sizeof(double) * MaxAxisNum);

			this->mAutoMove->execPlanMove(mGroupTrajout, 0, dRatio, groupjpos);
		}

		hsc3::algo::HS_MStatus MotionCombine::execJointIntMove(double *jointpos, double *jointvel, double *spacepos)
		{
			int errorID = 0;
			static double dLastJointPos[MaxAxisNum] = {0.0, -90.0, 180.0, 0.0, 90.0, 0.0};
			hsc3::algo::IntData intdata = {0.0};
			hsc3::algo::HS_MStatus status = hsc3::algo::M_UnInit;
			status = this->mAutoMove->execIntMove(intdata, errorID);					// 获取周期关节插补点
			memcpy(jointpos, intdata.tGJPos[0].dJPos[0], sizeof(double)*MaxAxisNum);
			this->mCalibrate->calcJPosToCPos(jointpos, -1, -1, spacepos);				// 获取空间位置
			for(int i=0; i<MaxAxisNum; i++)
			{
				jointvel[i] = (jointpos[i] - dLastJointPos[i]) / CYCLE;					// 获取关节速度
			}
			memcpy(dLastJointPos, jointpos, sizeof(double)*MaxAxisNum);    
			return status;
		}

		void MotionCombine::planSpace()
		{
			double dRatio = 0.3;
			int mMotionDataNum = 0;
			hsc3::algo::GroupMotionData groupdata = {0};
			hsc3::algo::GroupTrajData *mGroupTrajout = new hsc3::algo::GroupTrajData[40];
			memset(mGroupTrajout, 0, sizeof(hsc3::algo::GroupTrajData) * 40);

			groupdata.iLineNum = 0;
			groupdata.tHS_GroupRel.eGroupRelType[0] = hsc3::algo::GroupRelType::GRT_Independent;
			groupdata.tHS_GroupRel.eGroupRelType[1] = hsc3::algo::GroupRelType::GRT_NoUse;
			groupdata.tHS_GroupRel.eGroupRelType[2] = hsc3::algo::GroupRelType::GRT_NoUse;
			groupdata.tHS_GroupRel.eGroupRelType[3] = hsc3::algo::GroupRelType::GRT_NoUse;
			groupdata.dCnt = 0.0;
			groupdata.dCR = 0.0;
			groupdata.iSmooth = 0;
			groupdata.bExtTool = false;
			groupdata.bStartMove = true;
			groupdata.bWristQYFlag = false;
			groupdata.tFilterControl.bFilterOpenFlag = false;
			groupdata.tBaseMoveData[0].eTrajType = hsc3::algo::HS_MOVETYPE::MP_Line;
			groupdata.tBaseMoveData[0].sCurCoordinate.iCoordinate = hsc3::algo::COORD_SYSTEM::BASE_COORD_SYSTEM;
			groupdata.tBaseMoveData[0].sCurCoordinate.iToolNum = -1;
			groupdata.tBaseMoveData[0].sCurCoordinate.iWorkNum = -1;
			groupdata.tBaseMoveData[0].sCurCoordinate.bExtTool = -1;
			groupdata.tBaseMoveData[0].sCurCoordinate.bExtCoorper = -1;
			groupdata.tBaseMoveData[0].sStartPos.dPos[0] = 0.0; groupdata.tBaseMoveData[0].sStartPos.dPos[1] = -90.0; groupdata.tBaseMoveData[0].sStartPos.dPos[2] = 180.0;
			groupdata.tBaseMoveData[0].sStartPos.dPos[3] = 0.0; groupdata.tBaseMoveData[0].sStartPos.dPos[4] = 90.0; groupdata.tBaseMoveData[0].sStartPos.dPos[5] = 0.0;
			groupdata.tBaseMoveData[0].sStartPos.dPos[6] = 0; groupdata.tBaseMoveData[0].sStartPos.dPos[7] = 0; groupdata.tBaseMoveData[0].sStartPos.dPos[8] = 0;
			groupdata.tBaseMoveData[0].sStartPos.iPose = 0;
			groupdata.tBaseMoveData[0].sStartPos.hs_coordinate.iCoordinate = hsc3::algo::COORD_SYSTEM::JOINT_COORD_SYSTEM;
			groupdata.tBaseMoveData[0].sStartPos.hs_coordinate.iToolNum = -1;
			groupdata.tBaseMoveData[0].sStartPos.hs_coordinate.iWorkNum = -1;
			groupdata.tBaseMoveData[0].sStartPos.hs_coordinate.bExtTool = false;
			groupdata.tBaseMoveData[0].sStartPos.hs_coordinate.bExtCoorper = false;

			memset(groupdata.tBaseMoveData[0].sMidPos.dPos, 0, sizeof(double) * 9);
			groupdata.tBaseMoveData[0].sEndPos.dPos[0] = 300.0; groupdata.tBaseMoveData[0].sEndPos.dPos[1] = 100.0; groupdata.tBaseMoveData[0].sEndPos.dPos[2] = 200.0;
			groupdata.tBaseMoveData[0].sEndPos.dPos[3] = 180.0; groupdata.tBaseMoveData[0].sEndPos.dPos[4] = 0.0; groupdata.tBaseMoveData[0].sEndPos.dPos[5] = 180.0;
			groupdata.tBaseMoveData[0].sEndPos.dPos[6] = 0; groupdata.tBaseMoveData[0].sEndPos.dPos[7] = 0; groupdata.tBaseMoveData[0].sEndPos.dPos[8] = 0;
			groupdata.tBaseMoveData[0].sEndPos.iPose = 0;
			groupdata.tBaseMoveData[0].sEndPos.hs_coordinate.iCoordinate = hsc3::algo::COORD_SYSTEM::BASE_COORD_SYSTEM;
			groupdata.tBaseMoveData[0].sEndPos.hs_coordinate.iToolNum = -1;
			groupdata.tBaseMoveData[0].sEndPos.hs_coordinate.iWorkNum = -1;
			groupdata.tBaseMoveData[0].sEndPos.hs_coordinate.bExtTool = false;
			groupdata.tBaseMoveData[0].sEndPos.hs_coordinate.bExtCoorper = false;
			groupdata.tBaseMoveData[0].b2mid = false;
			groupdata.tBaseMoveData[0].dVel = 100.0;
			groupdata.tBaseMoveData[0].dVort = 100.0;
			groupdata.tBaseMoveData[0].dAcc = 100.0;
			groupdata.tBaseMoveData[0].dDec = 100.0;
			groupdata.tBaseMoveData[0].iCntType = 0;
			groupdata.tBaseMoveData[0].tRevolve.iTurn = 0;
			groupdata.tBaseMoveData[0].bCoorperMove = false;
			groupdata.tBaseMoveData[0].tWeavePara.bWeaveEn = false;

			this->mAutoMove->execPrehandle(groupdata, mGroupTrajout, mMotionDataNum);
			hsc3::algo::HS_GroupJPos groupjpos = {0.0};
			memcpy(groupjpos.dJPos[0], groupdata.tBaseMoveData[0].sStartPos.dPos, sizeof(double) * MaxAxisNum);

			this->mAutoMove->execPlanMove(mGroupTrajout, 0, dRatio, groupjpos);
		}

		hsc3::algo::HS_MStatus MotionCombine::execSpaceIntMove(double *jointpos, double *jointvel, double *spacepos)
		{
			int errorID = 0;
			static double dLastJointPos[MaxAxisNum] = {0.0, -90.0, 180.0, 0.0, 90.0, 0.0};
			hsc3::algo::IntData intdata = {0.0};
			hsc3::algo::HS_MStatus status = hsc3::algo::M_UnInit;
			status = this->mAutoMove->execIntMove(intdata, errorID);					// 获取周期关节插补点
			memcpy(jointpos, intdata.tGJPos[0].dJPos[0], sizeof(double)*MaxAxisNum);    
			this->mCalibrate->calcJPosToCPos(jointpos, -1, -1, spacepos);				// 获取空间位置

			for(int i=0; i<MaxAxisNum; i++)
			{
				jointvel[i] = (jointpos[i] - dLastJointPos[i]) / CYCLE;					// 获取关节速度
			}
			memcpy(dLastJointPos, jointpos, sizeof(double)*MaxAxisNum);    
			return status;
		}

		void MotionCombine::planManual(int axisnum, bool dir, double *nowpos)
		{
			double dRatio = 0.06;
			hsc3::algo::ManualPara mManualPara;
			mManualPara.iAxisNum = axisnum;
			mManualPara.iGroupNum = 0;
			mManualPara.bDir = dir;
			mManualPara.dHandVelRatio = dRatio;
			mManualPara.dIncLen = 0.0;
			mManualPara.iSmooth = 5;
			mManualPara.bWristQYOpen = true;
			mManualPara.dVtran = 250.0;
			mManualPara.dVrot = 120.0;
			memset(mManualPara.iToolNum, 0, sizeof(int)*MAXGROUPNUM);
			mManualPara.tHS_GroupRel.eGroupRelType[0] = hsc3::algo::GroupRelType::GRT_Independent;
			mManualPara.tHS_GroupRel.eGroupRelType[1] = hsc3::algo::GroupRelType::GRT_NoUse;
			mManualPara.tHS_GroupRel.eGroupRelType[2] = hsc3::algo::GroupRelType::GRT_NoUse;
			mManualPara.tHS_GroupRel.eGroupRelType[3] = hsc3::algo::GroupRelType::GRT_NoUse;
			mManualPara.hs_coordinate.iCoordinate = hsc3::algo::COORD_SYSTEM::JOINT_COORD_SYSTEM;
			mManualPara.hs_coordinate.iToolNum = -1;
			mManualPara.hs_coordinate.iWorkNum = -1;
			mManualPara.hs_coordinate.bExtTool = false;
			mManualPara.hs_coordinate.bExtCoorper = false;

			int iErrorId = 0;
			hsc3::algo::HS_GroupJPos groupjpos = {0.0};
			memcpy(groupjpos.dJPos[0], nowpos, sizeof(double)*MaxAxisNum);
			this->mBaseManualMove->Plan(groupjpos, mManualPara);
		}

		hsc3::algo::HS_MStatus MotionCombine::execManualIntMove(double *jointpos, double *jointvel, double *spacepos)
		{
			int iErrorId = 0;
			static double dLastJointPos[MaxAxisNum] = {0.0, -90.0, 180.0, 0.0, 90.0, 0.0};
			hsc3::algo::HS_MStatus status = hsc3::algo::M_UnInit;
			hsc3::algo::HS_GroupJPos groupjpos = {0.0};
			status = this->mBaseManualMove->Move(iErrorId, groupjpos);					// 获取周期关节插补点
			memcpy(jointpos, groupjpos.dJPos[0], sizeof(double)*6);
			this->mCalibrate->calcJPosToCPos(jointpos, -1, -1, spacepos);				// 获取空间位置

			for(int i=0; i<MaxAxisNum; i++)
			{
				jointvel[i] = (jointpos[i] - dLastJointPos[i]) / CYCLE;					// 获取关节速度
			}
			memcpy(dLastJointPos, jointpos, sizeof(double)*MaxAxisNum);    

			return status;
		}

		void MotionCombine::stopPlanManual()
		{
			this->mBaseManualMove->StopPlan();
		}
	}
}