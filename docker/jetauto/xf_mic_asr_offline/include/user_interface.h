/* user_interface_h */
#include <iostream>
#include <sstream>
#include <string>
#include "asr_offline_record_sample.h"
/***************************参数配置区域，用户可通过修改这些词来进行(parameter configuration area. You can modify the following words) ********************************/
#define whether_print_log 0 //是否打印log(whether to print log)
#define TIMEOUT 10 //在客户端获取服务端结果时，超时时间(timeout when acquiring the server result from client)

/******麦克风基础功能参数(microphone basic function parameter)******/
int PCM_MSG_LEN = 1024; //在录音时会发布音频流,单次发布大小为2048B(publish the audio when recording. The published size of audio is 2048B)
bool save_pcm_local = true; //保存音频到本地.(save the audio to local)
int max_pcm_size = 10240000; //最大为10M,超过10M后自动删除,以节省空间.(maximum size is 10M. File greater than 10M will be deleted to save space)
//录音文件保存的地址,最好设置为绝对地址(storage address for the audio file. It is better to set as absolute address)
char *ORIGINAL_SOUND_PATH = (char*)"/audio/vvui_ori.pcm";
char *DENOISE_SOUND_PATH = (char*)"/audio/vvui_deno.pcm";
//资源文件存储地址(storage address for resource files)
char *SYSTEM_PATH = (char*)"/tmp/system.tar";
char *SYSTEM_CONFIG_PATH = (char*)"/tmp/config.txt";

//int whether_finised;
//struct speech_rec iat;
//UserData asr_data;


/******与离线命令词识别相关参数(parameters related to offline voice commands recognition)******/
std::string source_path = "";
std::string appid="";
char *ASR_RES_PATH = (char*)"/config/msc/res/asr/common.jet"; //离线语法识别资源路径，重要，与麦克风及appid绑定(path for offline grammar recognition resource, which is important and bound with microphone and appid)
char *GRM_BUILD_PATH = (char*)"/config/msc/res/asr/GrmBuilld";   //构建离线语法识别网络生成数据保存路径(create the storage path for data generated by offline grammer recognition network)
char *GRM_FILE = (char*)"/config/call.bnf";					//构建离线识别语法网络所用的语法文件，用户自修改文件(create grammar files required by offline grammar recognition network. User can modify the file by themselves))
char *LEX_NAME = (char*)"contact";
char *APPID = (char*)" ";
//运行效果调试参数(run performance debugging parameters)
int confidence;
int time_per_order;//一次命令时长默认时长,如用户在接口中不进行设置,则默认为该值(default duration of voice command. If you don't the duration, the default value will be taken)

char awake_words[30] = "小幻小幻";//"hello jack";//"小幻小幻";
