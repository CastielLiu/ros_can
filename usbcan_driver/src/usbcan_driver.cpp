#include <vector>
#include <thread>
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <controlcan.h>
#include <can_msgs/FrameArray.h>

using namespace std;
#define _NODE_NAME_ "usb_can_driver"
#define CAN_CHANNEL1 0
#define CAN_CHANNEL2 1
#define DEVICE_INDEX 0
#define CHANNEL_CNT  2

#include <ctime>
#include <cassert>
class Log
{
/*
Log log;
log.init("log.txt", true);
log.addIntervalWriter("device_status", 0, 2.0);
log.addIntervalWriter("error_info", 1, 1.0);


log.write("匿名用户数据, 直接写入不进行判断");
log.write("非用户数据, 需要传递用户id", 0);

*/

public:
	Log()
	{
		enable = false;
		filename = "";
	}

	~Log()
	{
		if(of_handle.is_open())
			of_handle.close();
	}

	bool init(const std::string& _filename, bool _enable)
	{
		filename = _filename;
		enable = _enable;
		if(enable)
		{
			of_handle.open(filename.c_str());
			if(!of_handle.is_open())
				enable = false;
		}
	}

	/*@param 添加需要固定时间间隔写入的用户
	 *@param writername 写者名称，仅用于用户对照名称和id
	 *@param writer_id  写者ID，必须按顺序写0-1-2-...
	 *@param delay_s 间隔时间
	*/
	void addIntervalWriter(const std::string& writername, int writer_id, float delay_s)
	{
		if(writers_lasttime.size() != writer_id)
		{
			std::cerr << "Log class error: writer_id must be completed in order 0-1-2-3..." << std::endl;
			exit(0);
		}
		writers_lasttime.push_back(0);
		writers_interval.push_back(delay_s);
		writer_size = writers_lasttime.size();
	}

	/*@brief 写log到日志文件
     *@param info 数据
	 *@param writer_id 写者ID，默认为-1 匿名写者
	*/
	void write(const std::string& info, int writer_id=-1)
	{
		if(writer_id < -1 || writer_id >= writer_size)
			return;
		
		std::time_t now = std::time(0);
		if(writer_id == -1)
			goto write_log_info_to_file_by_log_class;
		
		if(now - writers_lasttime[writer_id] >= writers_interval[writer_id])
		{
			writers_lasttime[writer_id] = now;
			goto write_log_info_to_file_by_log_class;
		}

	write_log_info_to_file_by_log_class:
			
		std::tm *ltm = std::localtime(&now);
		of_handle << ltm->tm_year+1990 << "-" << ltm->tm_mon << "-" << ltm->tm_mday << " "
				<< ltm->tm_hour << ":" << ltm->tm_min << ":" << ltm->tm_sec << " "
				<<  info << std::endl;
	}


public:
	std::string filename;
	bool enable;
	
private:
	std::ofstream of_handle;

	std::vector<std::time_t> writers_lasttime;
	std::vector<float> writers_interval;
	size_t writer_size = 0;
};

class UsbCanDriver
{
public:
	UsbCanDriver();
	~UsbCanDriver();
	void run();
	
private:
	bool rosInit();
	bool deviceInit();
	void closeDevice();
	bool configCanChannel(int channel);
	void frameArray_callback(const can_msgs::FrameArray::ConstPtr& msgs);
	void receiveThread();
	
private:
	vector<int>    mFilterMode; //0.1 all frame 
								  //2   only std frame
								  //3   only exd frame
	vector<int>    mAccCode;
	vector<int>    mMaskCode;
	vector<int>    mBaudrate;
	vector<string> mFrameId;     //不同frame_id 对应不同的通道
	
	ros::Subscriber mSubFrameArray;
	ros::Publisher  mPub;
	
	// 日志文件
	std::string mLogFile;
	std::ofstream mOutLog;
	bool mIsLog;
};

UsbCanDriver::UsbCanDriver()
{
	mFilterMode.resize(CHANNEL_CNT);
	mAccCode.resize(CHANNEL_CNT);
	mMaskCode.resize(CHANNEL_CNT);
	mBaudrate.resize(CHANNEL_CNT);
	mFrameId.resize(CHANNEL_CNT);
}

UsbCanDriver::~UsbCanDriver()
{
	closeDevice();
}

void UsbCanDriver::closeDevice()
{
	if(1 == VCI_CloseDevice(VCI_USBCAN2,DEVICE_INDEX))
		printf("[%s] close device ok.\r\n", _NODE_NAME_);
	else
		printf("[%s] close device failed.\r\n", _NODE_NAME_);
}

bool UsbCanDriver::rosInit()
{
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	bool ok = false;
	
	ok = ros::param::get("~filter_mode",mFilterMode);
	if(!ok)
		for(int &mode:mFilterMode) mode = 0;
		
	ok = ros::param::get("~acc_code",mAccCode);
	if(!ok)
		for(int &acc:mAccCode) acc = 0x00000000;
	else
		for(int &acc:mAccCode) acc <<= 21;
		
	ok = ros::param::get("~mask_code",mMaskCode);
	if(!ok)
		for(int &mask:mMaskCode) mask = 0xFFFFFFFF;
	else
		for(int &mask:mMaskCode) mask <<= 21;
		
	ok = ros::param::get("~baudrate",mBaudrate);
	if(!ok)
	{
		for(int &baudrate:mBaudrate) 
		{
			baudrate = 500;
			ROS_INFO("[%s] baudrate set as default:%d",_NODE_NAME_,baudrate);
		}
	}
	
	ok = ros::param::get("~frame_id",mFrameId);
	if(!ok)
	{
		ROS_ERROR("[%s] no frame_id param!",_NODE_NAME_);
		return false;
	}
	
	mLogFile = nh_private.param<std::string>("log_file", "");
	if(mLogFile == "")
	    mIsLog = false;
    else
    {
        mOutLog.open(mLogFile.c_str());
		if(!mOutLog.is_open())
			mIsLog = false;
    }
	
	std::string to_can_topic   = nh_private.param<std::string>("to_can_topic","/to_usbcan");
	std::string from_can_topic = nh_private.param<std::string>("from_can_topic","/from_usbcan");
	
	mSubFrameArray = nh.subscribe(to_can_topic,100,&UsbCanDriver::frameArray_callback,this);
	mPub = nh.advertise<can_msgs::FrameArray>(from_can_topic, 100);
	return true;
}


void UsbCanDriver::frameArray_callback(const can_msgs::FrameArray::ConstPtr& msgs)
{
	for(int channel=0; channel<CHANNEL_CNT; ++channel)
	{
		if(mFrameId[channel] != msgs->header.frame_id)
			continue;

		//ROS_INFO("callback %d", msgs->frames.size());
		for(int i=0; i< msgs->frames.size(); ++i)
		{
			if(msgs->frames[i].len ==0) 
				continue;
				
			VCI_CAN_OBJ canObj;
			canObj.ID = msgs->frames[i].id;
			canObj.SendType = 0; //0 auto resend if error
								 //1 never resend
			canObj.RemoteFlag = msgs->frames[i].is_rtr;
			canObj.ExternFlag = msgs->frames[i].is_extended;
			canObj.DataLen =    msgs->frames[i].len;
			for(int j=0; j<canObj.DataLen; ++j)
				canObj.Data[j] =  msgs->frames[i].data[j];
			
			VCI_Transmit(VCI_USBCAN2, DEVICE_INDEX, channel, &canObj, 1);
		}
	}
	
	
}

// 配置设备通道
bool UsbCanDriver::configCanChannel(int channel)
{
	VCI_INIT_CONFIG config;
	config.AccCode = mAccCode[channel];
	config.AccMask = mMaskCode[channel];
	
	cout << "[" << _NODE_NAME_ << "] " << "ch" << channel << "\t"
		 << hex << config.AccCode << "\t" << config.AccMask << endl;
	
	config.Filter =  mFilterMode[channel];
	if(mBaudrate[channel] == 125)
	{
		config.Timing0=0x03;
		config.Timing1=0x1C;
	}
	else if(mBaudrate[channel] == 250)
	{
		config.Timing0=0x01;
		config.Timing1=0x1C;
	}
	else if(mBaudrate[channel] == 500)
	{
		config.Timing0=0x00;
		config.Timing1=0x1C;
	}
	else if(mBaudrate[channel] == 1000)
	{
		config.Timing0=0x00;
		config.Timing1=0x14;
	}
	else
	{
		ROS_ERROR("[%s] channel: %d : baudrate error, \
			please query manual and update source code!",_NODE_NAME_,channel);
		return false;
	}
	
	config.Mode=0;// 0: normal,  1:only receive,  2:cycle
	
	if(VCI_InitCAN(VCI_USBCAN2,DEVICE_INDEX,channel,&config)!=1)
	{
		ROS_ERROR("[%s] init channel %d error!", _NODE_NAME_,channel);
		VCI_CloseDevice(VCI_USBCAN2,DEVICE_INDEX);
		return false;
	}

	if(VCI_StartCAN(VCI_USBCAN2,DEVICE_INDEX,channel)!=1)
	{
		ROS_ERROR("[%s] start channel %d error!", _NODE_NAME_,channel);
		VCI_CloseDevice(VCI_USBCAN2,DEVICE_INDEX);
		return false;
	}
	return true;
}

bool UsbCanDriver::deviceInit()
{
	VCI_BOARD_INFO deviceInfos[5];
	int deviceCnt = VCI_FindUsbDevice2(deviceInfos);
	if(deviceCnt <1 )
	{
		ROS_ERROR("[%s] : No available can device.",_NODE_NAME_);
		return false;
	}
	
	if(VCI_OpenDevice(VCI_USBCAN2,DEVICE_INDEX,0)==1)
		ROS_INFO("[%s] : open usbcan deivce success!",_NODE_NAME_);
	else
	{
		ROS_ERROR("[%s] : open usbcan deivce failed!",_NODE_NAME_);
		return false;
	}
	
	for(int i=0; i<CHANNEL_CNT; ++i)
	{
		if(!configCanChannel(i))
			return false;
	}
	return true;
}

void UsbCanDriver::receiveThread()
{
	const int MaxLen = 500;
	VCI_CAN_OBJ msgs[MaxLen];
	can_msgs::FrameArray frames;
	int reclen;
	while(ros::ok())
	{
		for(int channel=0; channel<CHANNEL_CNT; ++channel)
		{
			reclen = VCI_Receive(VCI_USBCAN2,DEVICE_INDEX,channel,msgs,MaxLen,100);
			if(reclen <= 0) continue;
			//ROS_INFO("[%s] receive msgs len: %d",_NODE_NAME_,reclen);
			
			frames.frames.resize(reclen);
			frames.header.frame_id = mFrameId[channel];
			
			for(int i=0; i<reclen; ++i)
			{
				frames.frames[i].id          = msgs[i].ID;
				frames.frames[i].is_rtr      = msgs[i].RemoteFlag;
				frames.frames[i].is_extended = msgs[i].ExternFlag;
				frames.frames[i].len         = msgs[i].DataLen;
				for(int j=0; j<frames.frames[i].len; ++j)
					frames.frames[i].data[j] = msgs[i].Data[j];
			}
			mPub.publish(frames);
			//ROS_INFO("[%s] : published ",_NODE_NAME_);
		}
		
		usleep(10000);
		
	}
}


void UsbCanDriver::run()
{
	if(!rosInit())    return; //else ROS_INFO("rosInit ok");
	if(!deviceInit()) return; //else ROS_INFO("deviceInit ok");
	
	std::thread t(&UsbCanDriver::receiveThread,this);
	
	ros::spin();
	closeDevice();
}

int main(int argc,char** argv)
{
	ros::init(argc, argv, _NODE_NAME_);
	
	UsbCanDriver app;
	app.run();
	
	ros::Duration(0.5).sleep();
	
	return 0;
}

 
