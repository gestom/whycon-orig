/*
 * File name: CMessage.h
 * Date:      2006/10/12 11:56
 * Author:    
 */

#ifndef __CMESSAGE_H__
#define __CMESSAGE_H__

#define MESSAGE_LENGTH 14 

typedef enum
{
	MSG_NONE = 0,
	MSG_START,
	MSG_ANGLES,
	MSG_HEIGHT,
	MSG_HEADING,
	MSG_POSITION,
	MSG_STIFF,
	MSG_ALL,
	MSG_EMPTY,
	MSG_RESET,
	MSG_STOP,
	MSG_QUIT,
	MSG_LEARNING,
	MSG_TRAVERSING,
	MSG_TURNING,
	MSG_SELECT_CAMERA,
	MSG_ABS_POS,
	MSG_NUMBER 
} TMessageType;

class CMessage
{
	public:
		CMessage();
		~CMessage();
		unsigned char buf[MESSAGE_LENGTH+1];
		void pack();
		void unpack();
		const char* getStrType();
		TMessageType type;
		int value[4];
};

#endif

/* end of CMessage.h */
