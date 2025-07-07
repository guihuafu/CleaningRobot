#pragma once

#include "motionpara.h"

#ifndef DLL_EXPORT
#ifndef _LINUX_
    #define DLL_EXPORT __declspec(dllexport)
#else
    #define DLL_EXPORT __attribute__((visibility("default")))
#endif
#endif

class HS_Kinematics;
class HS_Calibration;

namespace hsc3
{
namespace algo
{
// 算法标定接口
class DLL_EXPORT Calibrate
{
public:
    DLL_EXPORT Calibrate(MotionPara *para,double dCycle,int iGroupNum);
    DLL_EXPORT ~Calibrate();

    /******************************************************
    函数功能：打印系统内部关键参数
    参数：	
    ******************************************************/
    DLL_EXPORT int PrintKeyInfo();
	/**
	* @brief  关节转空间
	* @param  jpos  关节位置
	* @param  toolnum  工具坐标
	* @param  worknum  工件坐标
	* @param  cpos  空间位置
	* @return  是否转换成功
	*/
	DLL_EXPORT int calcJPosToCPos(double jpos[9], int toolnum, int worknum, double cpos[9]);

	/**
	* @brief  关节转空间
	* @param  jpos  关节角
	* @param  toolnum  工具坐标
	* @param  worknum  工件坐标
	* @param  mpos  位姿矩阵
	* @return  是否转换成功
	*/
	DLL_EXPORT int calcJPosToMPos(double jpos[9], double mpos[5][4]);

    /************************************************
    函数功能：逆解计算【临近点求解】
    参    数：dMPos-------空间齐次矩阵
		    iToolNum---点位工具号
            iWorkNum---点位工件号
            eCPType----点位对应的坐标类型
		    dLJPos-----临近关节位置
            dCJPos------求解得到的关节坐标
    返 回 值：错误码
    *************************************************/
    DLL_EXPORT int HS_MPosToJPos_LJ(double dMPos[5][4],int iToolNum,int iWorkNum,CPType eCPType,double dLJPos[6],double dCJPos[9]);
    /************************************************
    函数功能：空间位置转换为齐次矩阵
    参    数：dCPos---空间位置
		     dMPos---齐次矩阵
    返 回 值：错误ID
    *************************************************/
    DLL_EXPORT int HS_CPosToMPos(double dCPos[6],double dMPos[4][4]);
    /************************************************
    函数功能：将齐次矩阵坐标转换为空间坐标【欧拉角方式为ZYX】
    参    数：pMPos---齐次矩阵坐标	
		    pCPos---空间坐标,X,Y,Z,A,B,C
    返 回 值：错误Id
    *************************************************/
    DLL_EXPORT int HS_MPosToCPos(double dMPos[4][4],double dCPos[6]);
    /************************************************
    函数功能：空间点位逆解关节位置求解
    参    数：dCPos-----空间位置
		    iToolNum---点位的工具号
		    iWorkNum---点位的工件号
		    eState-----点位的形态【机器人形态有效】
		    dJPos------输出关节位置
    返 回 值： 0----求解成功
            -  1---求解失败
    *************************************************/
    DLL_EXPORT int HS_CPosToJPos_JXJ(double dCPos[MaxAxisNum],int iToolNum,int iWorkNum,unsigned char eState,int eCPType,double dJPos[MaxAxisNum]);
    /************************************************
    函数功能：计算雅克比矩阵
    参    数：dJPos---关节角度[6]
		    dJacobian---雅克比矩阵 六轴为[6][6] 四轴为[6][4]
            bTool-------是否附带工具【默认工具】【六轴有效】
    返 回 值：错误ID
    *************************************************/
    DLL_EXPORT int HS_Jacobian(double dJPos[6],double *dJacobian,bool bTool);
    /************************************************
    函数功能：空间点位偏移计算【姿态左乘】【位移相加】
    参    数：
            dCPosIn--------输入空间位置
            dCPosOffset----偏移空间位置
            dCPosOut-------输出空间位置
    返 回 值：无
    *************************************************/
    DLL_EXPORT void HS_CPosOffset_RotLeft(double dCPosIn[6],double dCPosOffset[6],double dCPosOut[6]);
    /************************************************
    函数功能：判断当前点位是否超过区域空间范围
    参    数：
             dPos---当前关节角度
    返 回 值：0--在正常范围之内
             其他--报警码
    *************************************************/
    DLL_EXPORT int HS_JPosLimitCheck(double *dPos);
    /************************************************
    函数功能：最近关节点位的求解与约束
    参    数：
            dPosSet---待优化关节点位
            dRefPos---参考点位
            iLimitAxis---进行限位约束的轴号【-1表示不进行限位约束】
    返 回 值： 0----求解成功
              -1---求解失败
    *************************************************/
    DLL_EXPORT int HS_NearestPoint(double &dPosSet,double dRefPos,int iLimitAxis);
    /************************************************
    函数功能：对关节点位进行奇异位置检测，用来检测点位是否可进行空间运动
    参    数：dJPos----需检测的关节位置
    返 回 值：错误ID	 
    *************************************************/
    DLL_EXPORT int HS_QYLimitCheck(double dJPos[6]);
    /************************************************
    函数功能：对关节点位进行奇异位置动态检测，是否逐渐进入奇异区间
    参    数：
            dCurJPos----当前关节位置
            dNexJPos----下一个关节位置
    返 回 值：错误ID	 
    *************************************************/
    DLL_EXPORT int HS_QYDynCheck(double dCurJPos[6],double dNexJPos[6]);
    /************************************************
    函数功能：根据当前关节角度求解位姿类型
    参    数：dJPos---当前关节角度	
    返 回 值：位姿形态	 
    *************************************************/
    DLL_EXPORT  unsigned char HS_JPosToAState(double dJPos[6]);
    /************************************************
    函数功能：工具坐标系六点标定位姿矩阵
    参    数：dError-------误差限定（大于0有效）
             dPos_In1~4---同TCP位置的不同法兰盘位置
             dPos_In5-----沿工具Z轴运行一段的位置点
             dPos_In6-----ZX平面上的一个位置点
             dToolOut-----输出位姿变换矩阵[6]
			 dMaxError----实际标定的最大误差值
    返 回 值：错误ID
    *************************************************/
    DLL_EXPORT int Tool_Six(double dError,double *dPosIn1,double *dPosIn2,double *dPosIn3,
        double *dPosIn4,double *dPosIn5,double *dPosIn6,double *dToolOut,double &dMaxError);

    /************************************************
    函数功能：工具坐标系四点标定位置变换矩阵
    参    数：dError-------误差限定（大于0有效）
		    dPos_In1~4---输入四点坐标（同TCP位置的四个基坐标位置）
		    dToolOut-----输出位置变换矩阵[6]
			dMaxError----实际标定的最大误差值
    返 回 值：错误ID
    *************************************************/
    DLL_EXPORT int Tool_Four(double dError,double *dPosIn1,double *dPosIn2,double *dPosIn3,
        double *dPosIn4,double *dToolOut,double &dMaxError);

    /************************************************
    函数功能：工件坐标系标定
    参    数：dPo------标定原点
	         dPx------工具坐标x轴上的点
		     dPxy-----工具坐标xy平面上的点
		     dToolOut-----输出位置变换矩阵[6]
    返 回 值：错误ID
    *************************************************/
    DLL_EXPORT int	Work_Three(double *dPo,double *dPx,double *dPxy,double *dWorkOut);
    /************************************************
    函数功能：外部工件坐标系三点标定位姿矩阵
    参    数：dPosIn1-----原点，外部TCP工具在法兰的位置（姿态不变取三点）
		     dPos_In2-----沿工件X轴运行一段的位置点
		     dPos_In3-----XY平面上的一个位置点
		     dToolIn------外部工具坐标系（实际存的是（工件）相对于基坐标的位置）
		     dWorkOut-----输出位姿变换矩阵[6]，外部工件坐标系（实际存的是（工具）相对于法兰的位置）
    返 回 值：错误ID
    *************************************************/
    DLL_EXPORT int Enternal_Work_Three(double *dPosIn1,double *dPosIn2,double *dPosIn3,double *dToolIn,double *dWorkOut);

    /************************************************
    函数功能：Scara左右手标定，标定2轴
    参    数：dLeftJPos------输入同一位置左手标定值（2轴）
	         dRightJPos-----输入同一位置右手标定值（2轴）
		     dZeroCorrect---输出对应的零点漂移补偿值（2轴）
						将当前零点值设为该值
    返 回 值：错误ID
    *************************************************/
    DLL_EXPORT int Scara_LeftRightZero(double dLeftJPos,double dRightJPos,double &dZeroCorrect);
    /************************************************
    函数功能：Scara两点标定算法
    参    数：dPosIn1------输入位置1（空间X/Y/Z/A/B/C）
	        dPosIn2------输入位置2（空间X/Y/Z/A/B/C）
		    dWorkOut-----工具矩阵（X/Y)
    返 回 值：错误ID
    *************************************************/
    DLL_EXPORT int Scara_Tool_Two(double *dPosIn1,double *dPosIn2,double *dWorkOut);
    /***********变位机标定***********************/
    DLL_EXPORT int GetPositionerAxisTrans(double *p1,double * p2,double * p3,double (*axisTrans)[4]);
    /************************************************
    函数功能：变位机标定算法
    参    数： dPosIn 输入0~7点标定数据
		      df(degree of freedom) 变位机自由度
		      caliRes 标定矩阵
    返 回 值：错误ID
    *************************************************/
    DLL_EXPORT int Positioner(double **dPosIn,int df,double **caliRes);
    /************************************************
    函数功能：外部轴标定
    参    数： dPos1~3 3个点位坐标值
		      dEncoder 3个点位对应的编码器坐标值
		      dExtAxisEncoder 外部轴每圈脉冲数
		      dRatio 减速比（度/编码器脉冲）
    返 回 值：错误ID
    *************************************************/
    DLL_EXPORT int CaliExtAxisRatio(double *dPos1,double *dPos2,double *dPos3,double *dEncoder,double &dExtAxisEncoder,double &dRatio);
    /************************************************
    函数功能：综合标定算法【20点】
    参    数：dDataIn---标定输入点位
            dOut1-----标定输出1
            dOut2-----标定输出2
    返 回 值：错误ID
    *************************************************/
    DLL_EXPORT int HS_IntegratedCalibration(double (*dDHPara)[3], double (*dDataIn)[6],double *dOut1,double *dOut2);
public:
    MotionPara *mMotionPara;	
    HS_Kinematics *m_HS_Kinematics;
    HS_Calibration *m_HS_Calibration;

};

}
}







