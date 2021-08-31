// core

#include <iostream>
#include <fstream>
#include <ctime>
#include <cassert>

class Log
{
/*
Log log;
log.init("log.txt");
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

	bool init(const std::string& _log_dir)
	{
		if(_log_dir.empty())
		{
			enable = false;
			return true;
		}

		// std::ifstream if_handle(_log_dir.c_str());
		// if(if_handle.is_open())
		// {
		// 	size_t front = if_handle.tellg();
		// 	if_handle.seekg(0, std::ios::end);
		// 	size_t rear = if_handle.tellg();
		// 	size_t bytes = rear - front;
		// 	if(bytes > 1024*1024)
		// 	{
		// 	}
		// 	std::cout << bytes << std::endl;
		// }
		std::time_t now = std::time(0);
		std::tm *ltm = std::localtime(&now);
		std::stringstream ss;
		ss << ltm->tm_year+1900 << ltm->tm_mon +1  << ltm->tm_mday << "_"
						<< ltm->tm_hour << ltm->tm_min << ltm->tm_sec;
		filename = _log_dir + "/" + ss.str() + "_log.txt";

		of_handle.open(filename.c_str()); //std::ios::app
		if(!of_handle.is_open())
		{
			std::cerr << "Log class: open " << filename << " failed!" << std::endl;
			enable = false;	
			return false;
		}
		else
		{
			std::cout << "Log class: open " << filename << " ok!" << std::endl;
			enable = true;
		}

		return true;
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
	 *@param writer_id 写者ID，默认为0xff 匿名写者
	*/
	void write(const std::string& info, size_t writer_id=0xff)
	{
		// std::cout << info << "  " << writer_id << " " << (writer_id == 0xff) <<  std::endl;
		// printf("%d\r\n", writer_id);
		// std::cout << (!enable) << "  " << (writer_id < 0xff) << " " << (writer_id >= writer_size) << " " << writer_size << std::endl;
		if(!enable) return;
		
		std::time_t now = std::time(0);
		if(writer_id == 0xff)
			goto goto_write_log_info_to_file_by_log_class;
		else if(writer_id >= writer_size)
			return;
		

		if(now - writers_lasttime[writer_id] >= writers_interval[writer_id])
		{
			writers_lasttime[writer_id] = now;
			goto goto_write_log_info_to_file_by_log_class;
		}
		else
			return ;

	goto_write_log_info_to_file_by_log_class:	
		std::tm *ltm = std::localtime(&now);
		of_handle << ltm->tm_year+1900 << "-" << ltm->tm_mon +1 << "-" << ltm->tm_mday << " "
				<< ltm->tm_hour << ":" << ltm->tm_min << ":" << ltm->tm_sec << " "
				<<  info << std::endl;
	}

private:
	std::ofstream of_handle;

	std::vector<std::time_t> writers_lasttime;
	std::vector<float> writers_interval;
	size_t writer_size = 0;

	std::string filename;
	bool enable;
};