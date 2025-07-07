#include "stdafx.h"
#include "Calibrate.h"
#include <iostream>
#include "HS_BasicPara.h"
#include "HS_Kinematics.h"
#include "HS_Calibration.h"

namespace hsc3
{
namespace algo
{
    Calibrate::Calibrate(MotionPara *para,double dCycle,int iGroupNum)
    {
        HS_BasicPara  * pHS_BasicPara = HS_BasicPara::GetInstance();
        pHS_BasicPara->SetPara(para);
        pHS_BasicPara->SetCycle(dCycle,1);

        mMotionPara = para;

        m_HS_Kinematics = new HS_Kinematics(iGroupNum);
        m_HS_Calibration = new HS_Calibration(iGroupNum);
    }

    Calibrate::~Calibrate()
    {
        delete m_HS_Kinematics;
        delete m_HS_Calibration;
    }

    int Calibrate::PrintKeyInfo()
    {
        return m_HS_Kinematics->PrintKeyInfo();
    }

	int Calibrate::calcJPosToCPos(double jpos[9], int toolnum, int worknum, double cpos[9])
    {
        return m_HS_Kinematics->HS_JPosToCPos(jpos,toolnum,worknum,cpos);
    }

	int Calibrate::calcJPosToMPos(double jpos[9], double mpos[5][4])
    {
        return m_HS_Kinematics->HS_JPosToMPos(jpos,mpos);
    }

    int Calibrate::HS_MPosToJPos_LJ(double dMPos[5][4],int iToolNum,int iWorkNum,CPType eCPType,double dLJPos[6],double dCJPos[9])
    {
        return m_HS_Kinematics->HS_MPosToJPos_LJ(dMPos,iToolNum,iWorkNum,eCPType,dLJPos,dCJPos);
    }

    int Calibrate::HS_CPosToMPos(double dCPos[6],double dMPos[4][4])
    {
        return m_HS_Kinematics->HS_CPosToMPos(dCPos,dMPos);
    }

    int Calibrate::HS_MPosToCPos(double dMPos[4][4],double dCPos[6])
    {
        return m_HS_Kinematics->HS_MPosToCPos(dMPos,dCPos);
    }

    int Calibrate::HS_CPosToJPos_JXJ(double dCPos[MaxAxisNum],int iToolNum,int iWorkNum,unsigned char eState,int eCPType,double dJPos[MaxAxisNum])
    {
        return m_HS_Kinematics->HS_CPosToJPos_JXJ(dCPos,iToolNum,iWorkNum,eState,dJPos);
    }

    int Calibrate::HS_Jacobian(double dJPos[6],double *dJacobian,bool bTool)
    {
        return m_HS_Kinematics->HS_Jacobian(dJPos,dJacobian,bTool);
    }

    void Calibrate::HS_CPosOffset_RotLeft(double dCPosIn[6],double dCPosOffset[6],double dCPosOut[6])
    {
        m_HS_Kinematics->HS_CPosOffset_RotLeft(dCPosIn,dCPosOffset,dCPosOut);
    }

    int Calibrate::HS_JPosLimitCheck(double *dPos)
    {
        return m_HS_Kinematics->HS_JPosLimitCheck(dPos);
    }

    int Calibrate::HS_NearestPoint(double &dPosSet,double dRefPos,int iLimitAxis)
    {
        return m_HS_Kinematics->HS_NearestPoint(dPosSet,dRefPos,iLimitAxis);
    }

    int Calibrate::HS_QYLimitCheck(double dJPos[6])
    {
        return m_HS_Kinematics->HS_QYLimitCheck(dJPos);
    }

    int Calibrate::HS_QYDynCheck(double dCurJPos[6],double dNexJPos[6])
    {
        return m_HS_Kinematics->HS_QYDynCheck(dCurJPos,dNexJPos);
    }

    unsigned char Calibrate::HS_JPosToAState(double dJPos[6])
    {
        return m_HS_Kinematics->HS_JPosToAState(dJPos);
    }

    /***********工具坐标系标定***********************/
    int Calibrate::Tool_Six(double dError,double *dPosIn1,double *dPosIn2,double *dPosIn3,
        double *dPosIn4,double *dPosIn5,double *dPosIn6,double *dToolOut,double &dMaxError)
    {
        return m_HS_Calibration->Tool_Six(dError,dPosIn1,dPosIn2,dPosIn3,dPosIn4,dPosIn5,dPosIn6,dToolOut,dMaxError);
    }

    int Calibrate::Tool_Four(double dError,double *dPosIn1,double *dPosIn2,double *dPosIn3,
        double *dPosIn4,double *dToolOut,double &dMaxError)
    {
        return m_HS_Calibration->Tool_Four(dError,dPosIn1,dPosIn2,dPosIn3,dPosIn4,dToolOut,dMaxError);
    }
    /***********工件坐标系标定***********************/
    int Calibrate::Work_Three(double *dPo,double *dPx,double *dPxy,double *dWorkOut)
    {
        return m_HS_Calibration->Work_Three(dPo,dPx,dPxy,dWorkOut);
    }

    int Calibrate::Enternal_Work_Three(double *dPosIn1,double *dPosIn2,double *dPosIn3,double *dToolIn,double *dWorkOut)
    {
        return m_HS_Calibration->Enternal_Work_Three(dPosIn1,dPosIn2,dPosIn3,dToolIn,dWorkOut);
    }

    /***********Scara标定***********************/
    int Calibrate::Scara_LeftRightZero(double dLeftJPos,double dRightJPos,double &dZeroCorrect)
    {
        return m_HS_Calibration->Scara_LeftRightZero(dLeftJPos,dRightJPos,dZeroCorrect);
    }
    int Calibrate::Scara_Tool_Two(double *dPosIn1,double *dPosIn2,double *dWorkOut)
    {
        return m_HS_Calibration->Scara_Tool_Two(dPosIn1,dPosIn2,dWorkOut);
    }
    /***********变位机标定***********************/
    int Calibrate::GetPositionerAxisTrans(double *p1,double * p2,double * p3,double (*axisTrans)[4])
    {
        return m_HS_Calibration->GetPositionerAxisTrans(p1,p2,p3,axisTrans);
    }
    int Calibrate::Positioner(double **dPosIn,int df,double **caliRes)
    {
        return m_HS_Calibration->Positioner(dPosIn,df,caliRes);
    }
    int Calibrate::CaliExtAxisRatio(double *dPos1,double *dPos2,double *dPos3,double *dEncoder,double &dExtAxisEncoder,double &dRatio)
    {
        return m_HS_Calibration->CaliExtAxisRatio(dPos1,dPos2,dPos3,dEncoder,dExtAxisEncoder,dRatio);
    }
    /***********20点法标定***********************/
    int Calibrate::HS_IntegratedCalibration(double (*dDHPara)[3], double (*dDataIn)[6],double *dOut1,double *dOut2)
    {
        return m_HS_Calibration->HS_IntegratedCalibration(dDHPara,dDataIn,dOut1,dOut2);
    }
}
}