
#define __DEBUG__
#include <libinoplc.h>
#include "classwrapper.h"

__IOBLCOK_DEFINE MyIOBlock
{
	BYTE  *ptr;
	UINT  len;
	BYTE  name[126];
};

///< define the block of inputs
__IOBLCOK_DEFINE etc20_inputs
{
	UINT   lbus_status;
	UINT   fault_id;
	UDINT  error_slot1;
	UDINT  error_slot2;
	USINT  digital_input1;
	USINT  digital_input2;
};

///< define the block of outpus
__IOBLCOK_DEFINE etc20_outputs
{
	UINT   device_control;
	USINT  digital_output1;
	USINT  digital_output2;
	USINT  digital_output3;
};

__IOBLCOK_DEFINE etc20_extra
{
	DWORD  interval;
	WORD   errcode;
	LREAL  output1;
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

/**
 * @brief just one easy eample for IOBlockCall
 *
 * @param [in] id                calling id
 * @param [in/out] pinputs       input values block
 *
 * @return handle result
 *	@retval -1 = failed
 *	@retval  0 = success
 */
int ExampleCall(int id, void *pdata)
{
	libinoplc_log("Example Call, id=%d", id);
	return 0;
}

/**
 * @brief example routine call in IEC code
 *
 * @param [in] id                calling id
 * @param [in/out] pinputs       input values block
 * @param [in/out] poutpus       outputs values block
 * @param [in/out] pextra        extra values block
 *
 * @return handle result
 *	@retval -1 = failed
 *	@retval  0 = success
 */
int Etc20Example(int id, void *pinputs, void *poutputs, void *pextra)
{
	//void* obj = createInstance();
	void* obj;
	int iStatus = 0;
	static int bFirstStart = 1;
	if(bFirstStart == 1)
	{
		obj = createInstance();
		bFirstStart = 0;
	}

	static int a = 0, b = 0;
	if (pinputs == NULL || poutputs == NULL)
		return -1;

	struct MyIOBlock *piBlock = pinputs;
	struct MyIOBlock *poBlock = poutputs;
	struct etc20_inputs *poin = (struct etc20_inputs*)piBlock->ptr;
	struct etc20_outputs *poout = (struct etc20_outputs*)poBlock->ptr;
	struct etc20_extra *pext = (struct etc20_extra*)pextra;
	
	

	a = a + 1;
	b = b + 1;
	
	if(b > 100)
		b = 0;
	
	if(a > 200)
		a = 0;
	
	if(poin->digital_input1 == 1)
	{
		poout->digital_output3 = 2;
		iStatus = execMotion(obj, 1);
	}
	else if(poin->digital_input1 == 2)
	{
		poout->digital_output3 = 3;
		iStatus = execMotion(obj, 0);
		if(iStatus == 3)
			poout->digital_output1 = 3;
	}
	else
	{
		poout->digital_output3 = 1;
	}
	
	
	//poin->fault_id = 10;
	//poin->digital_input1 = 16;
	//poin->digital_input2 = 17;
	
	//poout->digital_output1 = a;
	poout->digital_output2 = b;
	//poout->digital_output3 = 33.67;
	
	//pext->interval = 55;
	//pext->errcode = 56;
	//pext->output1 = 66.67;
	
	//setRatio(obj, 33.0);
	//double dRatio = getRatio(obj);
	

	return iStatus;
}
