// The following ifdef block is the standard way of creating macros which make exporting 
// from a DLL simpler. All files within this DLL are compiled with the NOKOVSDKCLIENT_EXPORTS
// symbol defined on the command line. This symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see 
// NOKOVSDKCLIENT_API functions as being imported from a DLL, whereas this DLL sees symbols
// defined with this macro as being exported.

//#define _LINUX
//#define _WIN32
#pragma once

#if defined(_WIN32)
#define SYSTEM_WIN32
#elif defined(_LINUX)
#define SYSTEM_LINUX
#else
#error "undefined system!"
#endif

#ifdef SYSTEM_WIN32
#ifdef NOKOVSDKCLIENT_EXPORTS
#define NOKOVSDKCLIENT_API __declspec(dllexport)
#else
#define NOKOVSDKCLIENT_API __declspec(dllimport)
#endif
#elif defined(SYSTEM_LINUX)
#define NOKOVSDKCLIENT_API __attribute ((visibility("default")))
#endif

#include "NokovSDKTypes.h"

class ClientCore;

class NOKOVSDKCLIENT_API NokovSDKClient {
public:
	NokovSDKClient();
	~NokovSDKClient();

	int Initialize(char* szServerAddress);
	int Uninitialize();
	void NokovSDKVersion(unsigned char Version[4]);
	void SetVerbosityLevel(int level);

	int WaitForForcePlateInit(long time = 0);
	int SetForcePlateCallback(void (*CallbackFunction)(sForcePlates* pForcePlate, void* pUserData), void* pUserData = 0);

	int SetDataCallback(void (*CallbackFunction)(sFrameOfMocapData* pFrameOfData, void* pUserData), void* pUserData=0);
	int SetMessageCallback(void (*CallbackFunction)(int id, char *szTraceMessage));
	int SetNotifyMsgCallback(void (*CallbackFunc)(sNotifyMsg* pNotify, void* pUserData), void* pUserData=0);
	int SetAnalogChCallback(void (*CallbackFunc)(sFrameOfAnalogChannelData* pAnChData, void* pUserData), void* pUserData = 0);

	int GetServerDescription(sServerDescription *pServerDescription);

	int GetDataDescriptions(sDataDescriptions** pDataDescriptions);
	int GetDataDescriptionsEx(sDataDescriptions** pDataDescriptions);
	int GetTposeDataDescriptions(char* name, sDataDescriptions** pDataDescriptions);
	int FreeDataDescriptions(sDataDescriptions* pDataDescriptions);

	bool DecodeTimecode(unsigned int inTimecode, unsigned int inTimecodeSubframe, int* hour, int* minute, int* second, int* frame, int* subframe);
	bool TimecodeStringify(unsigned int inTimecode, unsigned int inTimecodeSubframe, char *Buffer, int BufferSize);

	int NokovCopyFrame(const sFrameOfMocapData* pSrc, sFrameOfMocapData* pDst);
	int NokovFreeFrame(sFrameOfMocapData* pDst);
	sFrameOfMocapData* GetLastFrameOfMocapData();
	//主动获取数据
	//type://1帧号，2markerset数量 ，3刚体数量，4骨骼数量，5命名点数量，6未命名点数量，7模拟通道数，8参数
	int GetLastFrameDataByType(int type);
	
	float GetLastFrameLatency();		// 延迟
	unsigned int GetLastFrameSubframe();// 帧数偏差
	unsigned int GetLastFrameTimecode();// 时间码
	long long GetLastFrameTimeStamp();	// 时间戳
	int GetLastFrameMarkerByName(const char* markersetName, const char* markerName, float** data);	//markerset
	int GetLastFrameRigidBodyByName(const char*name, sRigidBodyData** data);									//刚体
	int GetLastFrameSkeletonByName(const char* markerSetName, const char* skeName, sRigidBodyData** data);	//骨骼
	int GetLastFrameLabeledMarker(sMarker** marker);													//命名点
	int GetLastFrameUndefined(const int index, float** point);										//未命名点
	int GetLastFrameAnalogdata(const char* chName, float* data);									//模拟通道
private:
	ClientCore* m_pClientCore;
};
