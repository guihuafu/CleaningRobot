#include "stdafx.h"
#include "motionparamanager.h"


namespace hsc3
{
namespace algo
{

    MotionParaManager* MotionParaManager::GetInstance()       
    {
        static MotionParaManager single;
        return &single;
    }

    MotionParaManager::~MotionParaManager()
    {

    }

    void MotionParaManager::creatMotionPara(int id,MotionPara &motionP)
    {
        DB_MotionPara tDB_MotionPara;

        //ÒÑ´æÔÚ£¬ÐÞ¸Ä
        for(int i = 0;i < m_tDB_MotionPara.size();i++)
        {
            if(m_tDB_MotionPara[i].iId == id)
            {
                m_tDB_MotionPara[i].tMotionPara = motionP;
                return;
            }
        }
        tDB_MotionPara.iId = id;
        tDB_MotionPara.tMotionPara = motionP;
        m_tDB_MotionPara.push_back(tDB_MotionPara);
    }

	void MotionParaManager::deleteMotionPara(int id)
    {
        for(int i = 0;i < m_tDB_MotionPara.size();i++)
        {
            if(m_tDB_MotionPara[i].iId == id)
            {
                m_tDB_MotionPara.erase(m_tDB_MotionPara.begin() + i);
                return;
            }
        }
    }

	MotionPara* MotionParaManager::getMotionPara(int id)
    {
        for(int i = 0;i < m_tDB_MotionPara.size();i++)
        {
            if(m_tDB_MotionPara[i].iId == id)
            {                
                return &m_tDB_MotionPara[i].tMotionPara;
            }
        }
        return NULL;
    }

}
}