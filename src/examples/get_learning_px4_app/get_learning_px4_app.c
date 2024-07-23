#include <px4_platform_common/log.h>  //包含PX4_INFO等打印函数
#include <px4_platform_common/px4_config.h>  //包含PX4定义的一些宏
#include <px4_platform_common/posix.h>  //包含POSIX系统调用，如poll，open、close、read、write等
#include <px4_platform_common/tasks.h> //包含任务相关的函数

#include <unistd.h>  //包含POSIX系统调用，如fork、execve、sleep等
#include <stdio.h>  //包含标准输入输出函数
#include <poll.h>  //包含poll函数
#include <string.h>  //包含字符串处理函数
#include <math.h>  //包含数学函数

#include <uORB/uORB.h>  //包含uORB相关的头文件
#include <uORB/topics/sensor_combined.h>  //包含传感器数据的头文件
#include <uORB/topics/vehicle_attitude_setpoint.h>  //包含姿态数据的头文件

__EXPORT int get_learning_px4_app_main(int argc, char *argv[]);  //声明函数

int get_learning_px4_app_main(int argc, char *argv[])
{
	PX4_INFO("get_learning_px4_app_main Hello Sky!");  //打印信息

	//订阅姿态数据
	int sensor_sub_fd = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	//限制更新频率5hz
	orb_set_interval(sensor_sub_fd, 200);

	//设置poll监听
	px4_pollfd_struct_t fds[] = {
		{ .fd = sensor_sub_fd, .events = POLLIN },
	};

	for(int i = 0; i < 5; i++)
	{
		//等待数据更新
		int poll_ret = px4_poll(fds, 1, 1000);

		//判断是否有数据更新
		if (poll_ret == 0)
		{
			PX4_ERR("Got no data within a second");
		}
		else if (poll_ret < 0)
		{
			PX4_ERR("ERROR return value from poll(): %d", poll_ret);
		}
		else
		{
			if (fds[0].revents & POLLIN)
			{
				//获取姿态数据
				struct vehicle_attitude_setpoint_s raw;
				orb_copy(ORB_ID(vehicle_attitude_setpoint), sensor_sub_fd, &raw);
				PX4_INFO("Roll: %.4f, Pitch: %.4f, Yaw: %.4f", (double)raw.roll_body, (double)raw.pitch_body, (double)raw.yaw_body);
			}
		}
	}

	PX4_INFO("exiting");

	return 0;

}
