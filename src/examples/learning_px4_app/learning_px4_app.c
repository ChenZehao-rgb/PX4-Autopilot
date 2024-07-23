#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>

#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>

__EXPORT int learning_px4_app_main(int argc, char *argv[]);

int learning_px4_app_main(int argc, char *argv[])
{
	PX4_INFO("Hello Sky!");

	//订阅传感器数据
	int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
	//限制更新频率5hz
	orb_set_interval(sensor_sub_fd, 200);

	//设置poll监听
	px4_pollfd_struct_t fds[] = {
		{ .fd = sensor_sub_fd, .events = POLLIN },
	};

	//发布姿态数据
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	orb_advert_t att_pub = orb_advertise(ORB_ID(vehicle_attitude), &att);



	int error_counter = 0; //错误计数器

	for (int i = 0; i < 5; i++) {
		//等待数据更新
		int poll_ret = px4_poll(fds, 1, 1000);

		//判断是否有数据更新
		if (poll_ret == 0) {
			PX4_ERR("Got no data within a second");

		} else if (poll_ret < 0) {
			//严重错误
			if (error_counter < 10 || error_counter % 50 == 0) {
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}

			error_counter++;

		} else {
			if (fds[0].revents & POLLIN) {
				//获取传感器数据
				struct sensor_combined_s raw;
				orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);

				//打印数据
				PX4_INFO("Accelerometer:\t%8.4f\t%8.4f\t%8.4f",
					 (double)raw.accelerometer_m_s2[0],
					 (double)raw.accelerometer_m_s2[1],
					 (double)raw.accelerometer_m_s2[2]);


				orb_publish(ORB_ID(vehicle_attitude), att_pub, &att);


			}

		}

		PX4_INFO("fds: %d", fds[0].fd);

	}

	PX4_INFO("exiting");

	return 0;
}

