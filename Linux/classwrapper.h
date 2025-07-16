#ifdef __cplusplus 
extern "C" {
#endif 

  void* createInstance();  // 返回类实例指针 
  int execMotion(void* obj, int en);
  void setRatio(void* obj, double ratio);
  double getRatio(void* obj);
  void destroy_instance(void* obj);
  
#ifdef __cplusplus 
}
#endif 