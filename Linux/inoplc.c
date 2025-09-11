
#define __DEBUG__
#include <libinoplc.h>
#include "MotionParaDef.h"
#include "classwrapper.h"

__IOBLCOK_DEFINE MyIOBlock
{
	BYTE  *ptr;
	UINT  len;
	BYTE  name[126];
};

///< define the block of inputs
__IOBLCOK_DEFINE motion_inputs
{
	INT		iStatus;
	INT		iServoErr;
	LREAL	dFbPos[9];
	LREAL	dFbVel[9];
	LREAL	dFbAcc[9];
	LREAL	dFbSpace[9];
};

///< define the block of outpus
__IOBLCOK_DEFINE motion_outputs
{
	INT    iRobStatus;
	INT    iErrorNum;
	LREAL  dCmdPos[9];
	LREAL  dCmdVel[9];
	LREAL  dCmdAcc[9];
	LREAL  dCmdSpace[9];
};

__IOBLCOK_DEFINE motion_extra
{
	INT		iPlanMode;
	BOOL   	bIsJoint;
	INT  	iAxisNum;
	BOOL   	bDir;
	LREAL   dRatio;
	LREAL  	dPos[9];
};

/**
 * @brief initialize when booting up the IEC-runtime
 * @warning please don't change the function name and format
 *
 * @param [in] none
 *
 * @return handle result
 *	@retval -1 = failed
 *	@retval  0 = success
 */
int __CInit()
{
	libinoplc_log("init my library");
	return 0;
}

/**
 * @brief deinit when exiting the IEC-runtime
 * @warning please don't change the function name and format
 *
 * @param [in] none
 *
 * @return handle result
 *	@retval -1 = failed
 *	@retval  0 = success
 */
int __CExit()
{
	libinoplc_log("cleanup my library");
	return 0;
}

///< control the Application stop/run
/* 1 == run 
 * 2 == stop
 * 0 == do nothing
 */
static int __runstop = 0;

/**
 * @brief this function is used to run/stop the Application
 *
 * @param [in] runstop     setting the run/stop, 0==do noting, 1==run, 2==stop
 *
 * @return handle result
 *	@retval -1 = failed
 *	@retval  0 = success
 */
void __application_set_stat(int runstop)
{
	__runstop = runstop;
}

/**
 * @brief this function will call in IEC-runtime main thread every 10ms
 * @warning please don't change the function name and format
 *
 * @param [in] none
 *
 * @return handle result
 *	@retval -1 = failed
 *	@retval  0 = success
 */
int __CCycle(int *runstop)
{
	///< don't remove the belowing line
	*runstop = __runstop;

	///< do something else below, but pls be attention here
	///< this function is call by main thread of runtime, any trouble
	///< will leading crash or something dangrous.

	return 0;
}

/**
 * @brief called when application start up
 *
 * @param [in] pevent     event pointer
 *
 * @return handle result
 *	@retval -1 = failed
 *	@retval 0 = success
 */
int __CStartup(void *pevent)
{
	///< fill here some initial code for Application start up
	libinoplc_log("hello------");
	return 0;
}

/**
 * @brief called when application stop
 *
 * @param [in] pevent     event pointer
 *
 * @return handle result
 *	@retval -1 = failed
 *	@retval 0 = success
 */
int __CStopdown(void *pevent)
{
	///< fill here some stop code for Application stopdown
	libinoplc_log("goodbye------");
	return 0;
}

/**
 * @brief Initialize the resource of library via IOBlockCall
 *
 * @param [in] id                calling id
 * @param [in/out] pdata         values block
 *
 * @return handle result
 *	@retval -1 = failed
 *	@retval  0 = success
 */
int ExampleInitialize(int id, void *pdata)
{
	libinoplc_log("Example Init, id=%d", id);
	return 0;
}

/**
 * @brief release the resource of library
 *
 * @param [in] id                calling id
 * @param [in/out] pdata         input values block
 *
 * @return handle result
 *	@retval -1 = failed
 *	@retval  0 = success
 */
int ExampleRelease(int id, void *pdata)
{
	libinoplc_log("Example Rele, id=%d", id);
	return 0;
}

void* obj = NULL;
struct GroupConfigPara strConfigPara = {0};
struct GroupCommandPara strCommandPara = {0};
struct GroupFeedbackPara strFeedbackPara = {0};

int MotionSetting(int id, void *pinputs, void *poutputs, void *pextra)
{
	if(pinputs == NULL || poutputs == NULL || pextra == NULL)
		return -1;
	
	if(obj == NULL)
		obj = createInstance();
	
	struct MyIOBlock *piBlock = pinputs;
	struct MyIOBlock *poBlock = poutputs;
	struct motion_inputs *poin = (struct motion_inputs*)piBlock->ptr;
	struct motion_outputs *poout = (struct motion_outputs*)poBlock->ptr;
	struct motion_extra *pext = (struct motion_extra*)pextra;
	printf("InoPlc-->MotionPlan-->MotionSetting, ID=%d \n", id);
	if(id == 1001)
	{
		memcpy(strCommandPara.dCmdAxisPos, poin->dFbPos, sizeof(double)*9);
		syncPos(obj, &strCommandPara, &strFeedbackPara);
	}
	else if(id == 1002)
	{
		resetMotion(obj);
		strCommandPara.iCmdErr = 0;
	}
}

int MotionPlan(int id, void *pdata)
{
	if (pdata == NULL)
		return -1;

	int iErrorID = 0;
	struct motion_extra *pext = (struct motion_extra*)pdata;

	if(obj == NULL)
		obj = createInstance();

	strConfigPara.ePlanMode = pext->iPlanMode;
	strConfigPara.iAxisNum = pext->iAxisNum;
	strConfigPara.iIsJoint = pext->bIsJoint;
	strConfigPara.iDir = pext->bDir;
	strConfigPara.dRatio = pext->dRatio;
	memcpy(strConfigPara.dPos, pext->dPos, sizeof(double)*9);

	printf("InoPlc-->MotionPlan-->strConfigPara ePlanMode=%d, iAxisNum=%d, iIsJoint=%d, dRatio=%f, iDir=%d \n", strConfigPara.ePlanMode, strConfigPara.iAxisNum, strConfigPara.iIsJoint, strConfigPara.dRatio, strConfigPara.iDir);
	printf("InoPlc-->MotionPlan-->strConfigPara dPos %f %f %f %f %f %f \n", strConfigPara.dPos[0], strConfigPara.dPos[1], strConfigPara.dPos[2], strConfigPara.dPos[3], strConfigPara.dPos[4], strConfigPara.dPos[5]);
	iErrorID = execPlan(obj, &strConfigPara);

	return iErrorID;
}

int MotionMove(int id, void *pinputs, void *poutputs, void *pextra)
{
	int iStatus = 0;
	if(obj == NULL)
		obj = createInstance();

	if(pinputs == NULL || poutputs == NULL || pextra == NULL)
		return -1;

	struct MyIOBlock *piBlock = pinputs;
	struct MyIOBlock *poBlock = poutputs;
	struct motion_inputs *poin = (struct motion_inputs*)piBlock->ptr;
	struct motion_outputs *poout = (struct motion_outputs*)poBlock->ptr;
	struct motion_extra *pext = (struct motion_extra*)pextra;
	
	memcpy(strFeedbackPara.dFbAxisPos, poin->dFbPos, sizeof(double)*9);
	memcpy(strFeedbackPara.dFbAxisVel, poin->dFbVel, sizeof(double)*9);
	memcpy(strFeedbackPara.dFbAxisAcc, poin->dFbAcc, sizeof(double)*9);
	iStatus = execMove(obj, &strCommandPara, &strFeedbackPara);
	memcpy(poout->dCmdPos, strCommandPara.dCmdAxisPos, sizeof(double)*9);
	memcpy(poout->dCmdVel, strCommandPara.dCmdAxisVel, sizeof(double)*9);
	memcpy(poout->dCmdAcc, strCommandPara.dCmdAxisAcc, sizeof(double)*9);
	memcpy(poout->dCmdSpace, strCommandPara.dCmdSpacePos, sizeof(double)*9);
	memcpy(poin->dFbSpace, strFeedbackPara.dFbSpace, sizeof(double)*9);
	poout->iRobStatus = iStatus;
	poout->iErrorNum = strCommandPara.iCmdErr;

	// if(strConfigPara.ePlanMode != 0)
	// 	printf("InoPlc-->MotionMove-->iStatus=%d, dCmdAxisPos %f %f %f %f %f %f \n",iStatus, strCommandPara.dCmdAxisPos[0],strCommandPara.dCmdAxisPos[1],strCommandPara.dCmdAxisPos[2]
	// 		,strCommandPara.dCmdAxisPos[3],strCommandPara.dCmdAxisPos[4],strCommandPara.dCmdAxisPos[5]);

	pext->iPlanMode = strConfigPara.ePlanMode;
	pext->bIsJoint = strConfigPara.iIsJoint;
	pext->iAxisNum = strConfigPara.iAxisNum;
	pext->bDir = strConfigPara.iDir;

	return 0;
}
