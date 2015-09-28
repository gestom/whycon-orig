/** File: CDump.cpp
* Module: Utils
* Description: Class for dumping messages
* Note:
* Author: Jan Faigl
* Created on: 23.06.2004
* History: 
*/

#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "CDump.h"

CDump * dump=NULL; 

const char *StrBoolean[] = {
   "NO ",
   "YES"
};   

const char *StrLogLevel[] = {
   "(WW)",
   "(II)",
   "(EE)",
   "(DD)"
};

const char *StrLogModule[] = {
   "Main",
   "Segmentation",
   "Detector",
   "Server",
   "Client",
   "RawImage",
   "Camera",
   "Main",
   "Map",
   "RangeFinder",
   "Drive",
   "Rcm",
   "Formation",
   "number"
};

//-----------------------------------------------------------------------------
// Class CDump 
//-----------------------------------------------------------------------------
CDump::CDump(const char * filename, int max_filename_length, int period) : maxLogLineLength(512) {
   logOutputFile = NULL;
   logGeneratedFilename = NULL;
   maxLogFileNameLength = max_filename_length;
   logPeriod = period;
   dumpToFile = false;
   logToScreen = false;
   logFileName = new char[maxLogFileNameLength];
   tmpLine = new char[maxLogLineLength];
   if (logFileName == NULL || tmpLine == NULL) {
      fprintf(stderr, "allocation error line [%d] file [%s]", __LINE__, __FILE__);
      exit(-1);
   }
   strcpy(logFileName, "");
   if (filename != NULL) {
      dumpToFile = true;
      strncpy(logFileName, filename, maxLogFileNameLength);
      if (logPeriod <= 0) {
	 OpenLog(logFileName);
      } else {
	 logGeneratedFilename = new char[maxLogFileNameLength];
	 if (logGeneratedFilename == NULL) {
	    fprintf(stderr, "allocation error line [%d] file [%s]", __LINE__, __FILE__);
	    exit(-1);
	 }
	 OpenLog(GenerateLogFilename());
      }
   }
   EnableAll();
}

//-----------------------------------------------------------------------------
CDump::~CDump() {
   CloseLog();
   if (logGeneratedFilename != NULL)
      delete[] logGeneratedFilename;
   delete[] logFileName;
   delete[] tmpLine;
}

//-----------------------------------------------------------------------------
void CDump::OpenLog(const char * fileName) {
   if (fileName == NULL || strcmp(fileName, "") == 0) {
   } else {
      logOutputFile = fopen(fileName, "w");
      if (logOutputFile == NULL) {
	 fprintf(stderr, "Can not open log file %s use stderr instead line [%d] file [%s]", fileName, __LINE__, __FILE__);
	 logOutputFile = stderr;
      }
   }
   openLogTime = time(NULL);
}

//-----------------------------------------------------------------------------
void CDump::CloseLog(void) {
   if (dumpToFile) {
      fclose(logOutputFile);
   }
}

//-----------------------------------------------------------------------------
const char * CDump::GenerateLogFilename(void) {
   char timeStamp[40];
   time_t timeNow;

   time(&timeNow);
   strftime(timeStamp, sizeof(timeStamp), "%Y-%m-%d_%H:%M:%S",localtime(&timeNow));
   sprintf(logGeneratedFilename, "%s_%s.log", logFileName, timeStamp);
   return logGeneratedFilename;
}

//-----------------------------------------------------------------------------
void CDump::LogActualization(void) {
   time_t actTime;

   if (logPeriod > 0) {
      actTime = time(&actTime);
      if ((actTime - openLogTime) > logPeriod) {
	 CloseLog();
	 OpenLog(GenerateLogFilename());
      }
   }
}

//-----------------------------------------------------------------------------
//#include <sys/time.h> //TODO REMOVE
void CDump::AddMessage(TLogLevel logLevel, TLogModule logModule, const char * message) {
   char timeStamp[40];
   time_t timeNow;

   LogActualization();
   time(&timeNow);
   strftime(timeStamp, sizeof(timeStamp), "%Y-%m-%d %H:%M:%S",localtime(&timeNow));
/*   struct  timeval t;
   gettimeofday(&t, NULL);
   sprintf(timeStamp, "%ld:%ld", t.tv_sec, t.tv_usec);
 */  
   if (dumpToFile && logLevelToFile[logLevel] && logModuleToFile[logModule]) {
      fprintf(logOutputFile, "%s %s %s %s\n", StrLogLevel[logLevel], StrLogModule[logModule], timeStamp, message);
      fflush(logOutputFile);
   }
   if (logToScreen && logLevelFilter[logLevel] && logModuleFilter[logModule]) {
      fprintf(stderr, "%s %s %s %s\n", StrLogLevel[logLevel], StrLogModule[logModule], timeStamp, message);
      fflush(stderr);
   }
}

//-----------------------------------------------------------------------------
bool CDump::GetLogLevel(TLogLevel logLevel) {
   return logLevelFilter[logLevel];
}

//-----------------------------------------------------------------------------
bool CDump::GetLogModule(TLogModule logModule) {
   return logModuleFilter[logModule];
}

//-----------------------------------------------------------------------------
bool CDump::GetLogLevelToFile(TLogLevel logLevel) {
   return logLevelToFile[logLevel];
}

//-----------------------------------------------------------------------------
bool CDump::GetLogModuleToFile(TLogModule logModule) {
   return logModuleToFile[logModule];
}

//-----------------------------------------------------------------------------
void CDump::EnableLogLevel(TLogLevel logLevel) {
   logLevelFilter[logLevel] = true;
}

//-----------------------------------------------------------------------------
void CDump::EnableLogModule(TLogModule logModule) {
   logModuleFilter[logModule] = true;
}

//-----------------------------------------------------------------------------
void CDump::EnableLogLevelToFile(TLogLevel logLevel) {
   logLevelToFile[logLevel] = true;
}

//-----------------------------------------------------------------------------
void CDump::EnableLogModuleToFile(TLogModule logModule) {
   logModuleToFile[logModule] = true;
}

//-----------------------------------------------------------------------------
void CDump::EnableLogToScreen(void) {
   logToScreen = true;
}
//-----------------------------------------------------------------------------
void CDump::DisableLogLevel(TLogLevel logLevel) {
   logLevelFilter[logLevel] = false;
}

//-----------------------------------------------------------------------------
void CDump::DisableLogModule(TLogModule logModule) {
   logModuleFilter[logModule] = false;
}

//-----------------------------------------------------------------------------
void CDump::DisableLogLevelToFile(TLogLevel logLevel) {
   logLevelToFile[logLevel] = false;
}

//-----------------------------------------------------------------------------
void CDump::DisableLogModuleToFile(TLogModule logModule) {
   logModuleToFile[logModule] = false;
}

//-----------------------------------------------------------------------------
void CDump::DisableLogToScreen(void) {
   logToScreen = false;
}

//-----------------------------------------------------------------------------
void CDump::DisableAllLogLevel(void) {
   for (int i = 0; i < (int)LOG_LEVEL_NUMBER; i++) {
      logLevelFilter[i] = false;
   }
}

//-----------------------------------------------------------------------------
void CDump::DisableAllLogModule(void) {
   for (int i = 0; i < (int)LOG_MODULE_NUMBER; i++) {
      logModuleFilter[i] = false;
   }
}

//-----------------------------------------------------------------------------
void CDump::DisableAllLogLevelToFile(void) {
   for (int i = 0; i < (int)LOG_LEVEL_NUMBER; i++) {
      logLevelToFile[i] = false;
   }
}

//-----------------------------------------------------------------------------
void CDump::DisableAllLogModuleToFile(void) {
   for (int i = 0; i < (int)LOG_MODULE_NUMBER; i++) {
      logModuleToFile[i] = false;
   }
}

//-----------------------------------------------------------------------------
void CDump::DisableAll(void) {
   DisableAllLogLevel();
   DisableAllLogModule();
   DisableAllLogLevelToFile();
   DisableAllLogModuleToFile();
}

//-----------------------------------------------------------------------------
void CDump::EnableAllLogLevel(void) {
   for (int i = 0; i < (int)LOG_LEVEL_NUMBER; i++) {
      logLevelFilter[i] = true;
   }
} 

//-----------------------------------------------------------------------------
void CDump::EnableAllLogModule(void) {
   for (int i = 0; i < (int)LOG_MODULE_NUMBER; i++) {
      logModuleFilter[i] = true;
   }
}

//-----------------------------------------------------------------------------
void CDump::EnableAllLogLevelToFile(void) {
   for (int i = 0; i < (int)LOG_LEVEL_NUMBER; i++) {
      logLevelToFile[i] = true;
   }
} 

//-----------------------------------------------------------------------------
void CDump::EnableAllLogModuleToFile(void) {
   for (int i = 0; i < (int)LOG_MODULE_NUMBER; i++) {
      logModuleToFile[i] = true;
   }
}

//-----------------------------------------------------------------------------
void CDump::EnableAll(void) {
   EnableAllLogLevel();
   EnableAllLogModule();
   EnableAllLogLevelToFile();
   EnableAllLogModuleToFile();
   EnableLogToScreen();
}

//-----------------------------------------------------------------------------
int CDump::Inform(TLogModule logModule, const char * frm, ...) {
   va_list arg;
   if (logLevelFilter[LOG_LEVEL_I] && logModuleFilter[logModule]) {
      va_start(arg, frm);
      vsprintf(tmpLine, frm, arg);
      va_end(arg);
      AddMessage(LOG_LEVEL_I, logModule, tmpLine);
   }
   return 0;
}

//-----------------------------------------------------------------------------
int CDump::Warning(TLogModule logModule, const char * frm, ...) {
   va_list arg;

   if (logLevelFilter[LOG_LEVEL_W] && logModuleFilter[logModule]) {
      va_start(arg, frm);
      vsprintf(tmpLine, frm, arg);
      va_end(arg);
      AddMessage(LOG_LEVEL_W, logModule, tmpLine);
   }
   return 0;
}

//-----------------------------------------------------------------------------
int CDump::Error(TLogModule logModule, const char * frm, ...) {
   va_list arg;

   if (logLevelFilter[LOG_LEVEL_E] && logModuleFilter[logModule]) {
      va_start(arg, frm);
      vsprintf(tmpLine, frm, arg);
      va_end(arg);
      AddMessage(LOG_LEVEL_E, logModule, tmpLine);
   }
   return 0;
}

//-----------------------------------------------------------------------------
int CDump::Debug(TLogModule logModule, const char * frm, ...) {
   va_list arg;

   if (logLevelFilter[LOG_LEVEL_D] && logModuleFilter[logModule]) {
      va_start(arg, frm);
      vsprintf(tmpLine, frm, arg);
      va_end(arg);
      AddMessage(LOG_LEVEL_D, logModule, tmpLine);
   }
   return 0;
}

//-----------------------------------------------------------------------------
void CDump::MemCheck(void * pointer, TLogModule logModule, int line, char * filename) {
   if (pointer == NULL) {
      Error(logModule, "Couldn't allocate memory line[%d] file [%s]", line, filename);
      exit(-2); 
   }
}

//-----------------------------------------------------------------------------
void CDump::ThreadCheck(void * thread, TLogModule logModule, int line, char * filename) {
   if (thread == NULL) {
      Error(logModule, "Unable create thread line[%d] file [%s]", line, filename);
      exit(-3); 
   }
}
