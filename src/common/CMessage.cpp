#include "CMessage.h"

const char* StrMessage[] ={
	"None",
	"Start",
	"Angles",
	"Height,roll",
	"Roll",
	"Stabilize at",
	"Stiff stabilize",
	"All",
	"Empty",
	"Reset",
	"Stop",
	"Quit",
	"Learn",
	"Traverse",
	"Turn",
	"Select camera",
	"ABS Position",
	"Number"
};

CMessage::CMessage()
{
	type = MSG_NONE;
	for (int i = 0;i<MESSAGE_LENGTH/3;i++)value[i]=0;
}

CMessage::~CMessage()
{
}

const char * CMessage::getStrType()
{
	return StrMessage[type];
}

void CMessage::pack()
{
	buf[0] = type;
	for (int i = 0;i<MESSAGE_LENGTH/3;i++){
		if (value[i] < 0) {
			value[i] = - value[i];
			buf[i*3+3] = 1;
		}else{
			buf[i*3+3] = 0;
		}
		buf[i*3+1] = value[i]%256; 
		buf[i*3+2] = value[i]/256;
	}
}

void CMessage::unpack()
{
	type = (TMessageType)buf[0];
	for (int i = 0;i<MESSAGE_LENGTH/3;i++){
		value[i] = buf[3*i+1] + buf[3*i+2]*256; 
		if (buf[3*i+3] > 0) value[i] = - value[i];
	}
}

