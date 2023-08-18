#ifndef AIUIAGENTTESTER_H_
#define AIUIAGENTTESTER_H_

#include <aiui/AIUI.h>
#include <FileUtil.h>
#include <WriteAudioThread.h>
#include <AudioRecorder.h>
#include <TestListener.h>
#include <AudioPlayer.h>
#include <msp_cmn.h>


class AIUITester
{
private:
	IAIUIAgent* agent;//aiui代理(aiui aproxy)
	TestListener listener;
	AudioRecorder* audioRecorder;//与录音相关的(related to recording)
	AudioPlayer* audioPlayer;//与音频播放相关(related to audio playing)
public:
	AIUITester() ;
	~AIUITester();
private:
	void showIntroduction(bool detail);
	//创建AIUI 代理，通过AIUI代理与SDK发送消息通信(create AIUI proxy. Communicate with SDK proxy through AIUI)
	void createAgent();
	//唤醒接口(wake up interface)
	void wakeup();

	//开始AIUI，调用stop()之后需要调用此接口才可以与AIUI继续交互(start AIUI. Only if this interface is called after stop() is called, the interaction with AIUI can be continued)
	void start();
	//停止AIUI(stop AIUI)
	void stop();

	//usb麦克风设备创建(create usb microphone device)
    void recorder_creat();
	//麦克风录音开始(microphone starts recording)
	void recorder_start();
	//麦克风录音结束(microphone finishes recording)
	void recorder_stop();

	void stopWriteThread();

	void reset();

	void destory();

	void buildGrammar();

	void updateLocalLexicon();
	// void gTTS();

public:
	void readCmd();
	void test();
};

#endif /* AIUIAGENTTESTER_H_ */