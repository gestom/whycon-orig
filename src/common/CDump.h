/** File: CDump.h
* Module: Utils
* Description: Class for dumping messages
* Note:
* Author: Jan Faigl
* Created on: 23.06.2004
* History: 
*/

#ifndef CCDump_h
#define CCDump_h


//Nutne pro vizualizaci
#include <ncurses.h> //vizualizace pomoci curses
#include <time.h>

//! A enum
/*! Basic Boolean type*/
//typedef enum { False, True } Boolean;

//! A convert array
/*! convert array for translate Boolean to string*/
extern const char *StrBoolean[];

typedef enum  {
   LOG_LEVEL_W,
   LOG_LEVEL_I,
   LOG_LEVEL_E,
   LOG_LEVEL_D,
   LOG_LEVEL_NUMBER
} TLogLevel;


typedef enum {
	LOG_MAIN,
	LOG_MODULE_SEGMENTATION,
	LOG_MODULE_DETECTOR,
	LOG_MODULE_SERVER,
	LOG_MODULE_CLIENT,
	LOG_MODULE_RAWIMAGE,
	LOG_MODULE_CAMERA,
	LOG_MODULE_MAIN,
	LOG_MODULE_MAP,
	LOG_MODULE_RANGEFINDER,
	LOG_MODULE_DRIVE,
	LOG_MODULE_RCM,
	LOG_MODULE_FORMATION,
	LOG_MODULE_NUMBER
} TLogModule;

//! A convert array
extern const char *StrLogLevel[];

//! A convert array
extern const char *StrLogModule[];

//-----------------------------------------------------------------------------
//Class CDump
//-----------------------------------------------------------------------------
//! A CDump class
/*! 
   class for dumping message to screen
 */ 
class CDump {
   bool logLevelFilter[LOG_LEVEL_NUMBER];
   bool logModuleFilter[LOG_MODULE_NUMBER];
   bool logLevelToFile[LOG_LEVEL_NUMBER];
   bool logModuleToFile[LOG_MODULE_NUMBER];
   bool logToScreen;

   const int maxLogLineLength;
   int maxLogFileNameLength;
   time_t logPeriod;
   time_t openLogTime;
   FILE * logOutputFile;
   char * logGeneratedFilename;
   char * logFileName;
   char * tmpLine;
   bool dumpToFile;

   public:
   /**
     init log system
     Enable all logs
     if filename is specified and logperiod > 0 then create new log file 
     after period
    */ 
   CDump(const char * filename, int max_filename_length, int log_period);

   ~CDump();
   private:
   void OpenLog(const char * fileName);
   void CloseLog(void);
   const char * GenerateLogFilename(void);
   void LogActualization(void);
   void AddMessage(TLogLevel logLevel, TLogModule logModule, const char * message);

   public:

   /** get the setting of log level loglevel
    */ 
   bool GetLogLevel(TLogLevel logLevel);
   bool GetLogModule(TLogModule logModule);
   bool GetLogLevelToFile(TLogLevel logLevel);
   bool GetLogModuleToFile(TLogModule logModule);


   void EnableLogLevel(TLogLevel logLevel);
   void EnableLogModule(TLogModule logModule);
   void EnableLogLevelToFile(TLogLevel logLevel);
   void EnableLogModuleToFile(TLogModule logModule);
   void EnableLogToScreen(void);

   void DisableLogLevel(TLogLevel logLevel);
   void DisableLogModule(TLogModule logModule);
   void DisableLogLevelToFile(TLogLevel logLevel);
   void DisableLogModuleToFile(TLogModule logModule);
   void DisableLogToScreen(void);

   void DisableAllLogLevel(void);
   void DisableAllLogModule(void);
   void DisableAllLogLevelToFile(void);
   void DisableAllLogModuleToFile(void);
   void DisableAll(void);

   void EnableAllLogLevel(void);
   void EnableAllLogModule(void);
   void EnableAllLogLevelToFile(void);
   void EnableAllLogModuleToFile(void);
   void EnableAll(void);
   /**
     add msg to 
    */ 
   int Add_msg(TLogLevel loglvl, const char * frm, ...);

   /**\fn int Inform(TLogModule logModule, const char * frm, ...)
     \brief Add inform message to log
     
     \param logModule Log modul.
     \param frm String added to log.
   */  
   int Inform(TLogModule logModule, const char * frm, ...);
   int Warning(TLogModule logModule, const char * frm, ...);
   int Error(TLogModule logModule, const char * frm, ...);
   int Debug(TLogModule logModule, const char * frm, ...);

   /**
     \fn void MemCheck(void * pointer, TLogModule module, int line, char * filename);
     \brief Check if pointer == NULL print erorr message cannot allocate memory  and exit with code -2

     \param pointer pointer to new allocate memory
     \param logmodule name of calling module
     \param line line number of calling MemCheck
     \param filename name of file from MemCheck is calling
    */
   void MemCheck(void * pointer, TLogModule logModule, int line, char * filename);


   /**
     \fn void ThreadCheck(void * thread, TLogModule logModule, int line, char * filename);
     \brief Check if thread == NULL print erorr message cannot create thread  exit with code -3

     \param thread pointer to thread
     \param logmodule name of calling module
     \param line line number of calling MemCheck
     \param filename name of file from MemCheck is calling
    */
   void ThreadCheck(void * thread, TLogModule logModule, int line, char * filename);
};

extern CDump * dump;
#endif
