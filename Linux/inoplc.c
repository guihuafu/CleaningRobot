
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

int MotionPlan(int id, void *pdata)
{
	if (pdata == NULL)
		return -1;

	static int bFirstStart = 1;
	struct motion_extra *pext = (struct motion_extra*)pdata;

	if(bFirstStart == 1)
	{
		obj = createInstance();
		bFirstStart = 0;
	}

	strConfigPara.ePlanMode = pext->iPlanMode;
	strConfigPara.iAxisNum = pext->iAxisNum;
	strConfigPara.iIsJoint = pext->bIsJoint;
	strConfigPara.iDir = pext->bDir;
	printf("/--MotionPlan-->strConfigPara ePlanMode=%d, iAxisNum=%d, iIsJoint=%d, iDir=%d \n", strConfigPara.ePlanMode, strConfigPara.iAxisNum, strConfigPara.iIsJoint, strConfigPara.iDir);
	execPlan(obj, &strConfigPara);

	pext->dPos[0] = 111.1;
	pext->dPos[1] = 122.2;
	pext->dPos[2] = 133.3;
	pext->dPos[3] = 144.4;
	return 0;
}

int MotionMove(int id, void *pinputs, void *poutputs, void *pextra)
{
	int iStatus = 0;
	static int bFirstStart = 1;
	if(bFirstStart == 1)
	{
		obj = createInstance();
		bFirstStart = 0;
	}

	if (pinputs == NULL || poutputs == NULL || pextra == NULL)
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
	poout->iRobStatus = iStatus;

	if(strConfigPara.ePlanMode != 0)
		printf("/--MotionMove-->dCmdAxisPos %f %f %f %f %f %f \n",strCommandPara.dCmdAxisPos[0],strCommandPara.dCmdAxisPos[1],strCommandPara.dCmdAxisPos[2]
			,strCommandPara.dCmdAxisPos[3],strCommandPara.dCmdAxisPos[4],strCommandPara.dCmdAxisPos[5]);
	double dPos[9] = {0.0};

	pext->iPlanMode = strConfigPara.ePlanMode;
	pext->bIsJoint = strConfigPara.iIsJoint;
	pext->iAxisNum = strConfigPara.iAxisNum;
	pext->bDir = strConfigPara.iDir;
	pext->dPos[0] = 11.1;
	pext->dPos[1] = 22.2;
	pext->dPos[2] = 33.3;
	pext->dPos[3] = 44.4;

	return 0;
}
