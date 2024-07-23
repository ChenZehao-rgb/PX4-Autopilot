#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>

#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>  //包含姿态数据的头文件

class MyModule : public ModuleBase<MyModule>
{
public:
    MyModule() = default;
    ~MyModule() override = default;

    static int task_spawn(int argc, char *argv[]);
    static MyModule *instantiate(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    void run() override;

private:
    int _vehicle_attitude_sub{-1};
};

int MyModule::task_spawn(int argc, char *argv[])
{
    _task_id = px4_task_spawn_cmd("my_module",
                                  SCHED_DEFAULT,
                                  SCHED_PRIORITY_DEFAULT,
                                  1024,
                                  (px4_main_t)&run_trampoline,
                                  (char *const *)argv);

    if (_task_id < 0) {
        _task_id = -1;
        return -1;
    }

    return 0;
}

MyModule *MyModule::instantiate(int argc, char *argv[])
{
    return new MyModule();
}

int MyModule::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}

int MyModule::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
Example module that subscribes to vehicle_attitude topic.

)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("my_module", "template");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

void MyModule::run()
{
    PX4_INFO("MyModule started");

    // Subscribe to vehicle_attitude topic
    _vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));

    // Main loop
    while (!should_exit()) {
        // Wait for new data
        px4_pollfd_struct_t fds[] = {
            { .fd = _vehicle_attitude_sub, .events = POLLIN },
        };

        int poll_ret = px4_poll(fds, 1, 1000);

        if (poll_ret == 0) {
            // Timeout, no data received
            PX4_INFO("No data within a second");

        } else if (poll_ret < 0) {
            // Poll error
            PX4_ERR("Poll error");

        } else {
            if (fds[0].revents & POLLIN) {
                // Data received, copy the data
                struct vehicle_attitude_setpoint_s att;
                orb_copy(ORB_ID(vehicle_attitude_setpoint), _vehicle_attitude_sub, &att);

                // Process the data
               PX4_INFO("Roll: %.4f, Pitch: %.4f, Yaw: %.4f", (double)att.roll_body, (double)att.pitch_body, (double)att.yaw_body);
            }
        }
    }

    // Unsubscribe
    orb_unsubscribe(_vehicle_attitude_sub);
}

extern "C" __EXPORT int my_module_main(int argc, char *argv[])
{
    return MyModule::main(argc, argv);
}
