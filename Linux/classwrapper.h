#ifndef __CLASSWRAPPER_H__
#define __CLASSWRAPPER_H__

#ifdef __cplusplus 
extern "C" {
#endif 

  void* createInstance();  // 返回类实例指针 
  int execPlan(void* obj, struct GroupConfigPara *cfgpara);
  int execMove(void* obj, struct GroupCommandPara *cmdpara, struct GroupFeedbackPara *fbpara);
  void setRatio(void* obj, double ratio);
  double getRatio(void* obj);
  void destroy_instance(void* obj);
  
#ifdef __cplusplus 
}
#endif 

#endif /*__CLASSWRAPPER_H__*/