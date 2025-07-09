#include "stdafx.h"
#include "HS_GroupKin.h"

HS_GroupKin::HS_GroupKin()
{
	for(int i = 0;i < MAXGROUPNUM;i++)
	{
		m_HS_Kinematics[i] = new HS_Kinematics(i);
	}
	m_iGroupNum = 0;
	memset(m_iGroupToolNum,0,sizeof(m_iGroupToolNum));
}

HS_GroupKin::~HS_GroupKin()
{
	for(int i = 0;i < MAXGROUPNUM;i++)
	{
		delete m_HS_Kinematics[i];
		m_HS_Kinematics[i] = NULL;
	}
}

/************************************************
函数功能：获取对应轴组编号的算法对象
参    数：iGroupNum----轴组编号
返 回 值：算法计算对象
*************************************************/	
HS_Kinematics* HS_GroupKin::GetKinematicsByNum(int iGroupNum)
{
	if(iGroupNum < 0||iGroupNum >= MAXCOORDNUM)
		return m_HS_Kinematics[0];

	return m_HS_Kinematics[iGroupNum];
}
