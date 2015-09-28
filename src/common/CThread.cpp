/** File: CThread.cpp
* Module: Common
* Description: Class for thread encapsualation
* Note:
* Author: Jan Faigl
* Created on: 23.06.2004
* History: 
*/

#include "CThread.h"
extern "C" {
   int StartThread(void* arg) {
      CThread * thread = static_cast<CThread*>(arg);
      return thread->DoExecute();
   }
}

//----------------------------------------------------------------------------
//Class CThread
//----------------------------------------------------------------------------
CThread::~CThread() {
}
