#ifndef __MOTIONPARA_H__
#define __MOTIONPARA_H__

enum PlanMode
{
	Plan_None		= 0,
	Plan_Manual		= 1,
	Plan_Auto		= 2,
	Plan_Stop		= 3,
};

typedef struct GroupConfigPara
{
	enum PlanMode ePlanMode;
	int iIsJoint;
	int iAxisNum;
	int iDir;
	double dPos[9];
}GroupConfigPara;

typedef struct GroupCommandPara
{
	int iCmdWord;			// 命令字
	int iCmdErr;			// 报警码
	double dCmdAxisPos[9];	// 指令关节位置
	double dCmdAxisVel[9];	// 指令关节速度
	double dCmdAxisAcc[9];	// 指令关节加速度
	double dCmdSpacePos[9];	// 指令空间位置
}GroupCommandPara;

typedef struct GroupFeedbackPara
{
	int iStatus;			// 伺服状态
	int iServoErr;			// 伺服报警码
	double dFbAxisPos[9];  	// 反馈关节位置
	double dFbAxisVel[9];  	// 反馈关节速度
	double dFbAxisAcc[9];  	// 反馈关节加速度
}GroupFeedbackPara;

#endif /*__MOTIONPARA_H__*/