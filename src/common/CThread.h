/** File: CThread.cpp
* Module: Common
* Description: Class for thread encapsualation
* Note:
* Author: Jan Faigl
* Created on: 23.06.2004
* History: 
*/
#ifndef CThread_h
#define CThread_h
#include "SDL/SDL_thread.h"

extern "C" int StartThread(void* arg);
class CThread {
      virtual int DoExecute(void) = 0; 
      friend int StartThread(void*);
   public:
      virtual ~CThread() = 0;
};
#endif
