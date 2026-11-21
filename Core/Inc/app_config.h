/*
 * 应用开关集中在这里。雷达和手动任务的宏在编译时选中相应分支，修改后需重新构建固件。
 * g_app_start 是运行时调试状态，可在 Keil Watch 查看选区、当前启动阶段和故障原因。
 */

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

/*
 * 由 StartDefaultTask() 中的 #if APP_RADAR_ENABLED 读取。
 * 1 选雷达握手、选区和扫描，等扫描结果后规划。
 * 0 选固定地图规划，新增障碍集合为空。
 * UART5 的硬件初始化仍由 main() 统一完成。
 */
#ifndef APP_RADAR_ENABLED
#define APP_RADAR_ENABLED 1
#endif

/* 任务数据来源：1=手动测试，不等待扫码；0=恢复实际扫码。
 * 手动模式仍会行驶到扫码区，再按下列四组任务继续完整流程。
 * 只跳过扫码等待，不跳过后续视觉定位、抓取和码垛动作。
 */
#ifndef APP_MANUAL_TASK_ENABLED
/* 0 在扫码工位等待实际二维码，1 取下方四组编译期数字。两种方式都要先按启停区键并走到扫码工位。 */
#define APP_MANUAL_TASK_ENABLED 0
#endif

/* 每组是三位十进制数，三位不能重复；修改后重新编译、烧录并复位。
 * 第1/3组：两圈取料颜色，各位范围1~6，两组必须是相同颜色的排列。
 * 第2/4组：两圈加工圆环，各位范围1~3；第一圈暂存沿用第2组，
 * 第二圈暂存自动按颜色匹配第一层，不直接使用第4组放置顺序。
 * 当前123+123+123+123：两圈均取颜色1/2/3，加工圆环均为1/2/3。
 */
#define APP_MANUAL_TASK_GROUP1 123U
#define APP_MANUAL_TASK_GROUP2 123U
#define APP_MANUAL_TASK_GROUP3 123U
#define APP_MANUAL_TASK_GROUP4 123U

/*
 * 主任务的状态枚举。enum 给每个状态定义了容易看懂的名字。
 * 在 Keil Watch 展开 g_app_start.status 就能顺着下面的顺序查卡点。
 */
typedef enum {
    APP_START_WAIT_KEY = 0,  /* 等 1 号区或 2 号区单键按下。 */
    APP_START_WAIT_FEEDBACK, /* 等四轮和 IMU 有效。 */
    APP_START_WAIT_POSE,     /* 等底盘确认起点校正命令。 */
    APP_START_PLANNING,      /* 等雷达扫描，以及八段路线产生。 */
    APP_START_RUNNING,       /* 正在走路或执行某个工位。 */
    APP_START_DONE,          /* 已完成最后一段和回库。 */
    APP_START_ERROR          /* 启动或路径出错时，故障原因看 fault。 */
} AppStart_Status_t;

typedef enum {
    APP_START_FAULT_NONE = 0,
    APP_START_FAULT_BAD_ZONE,
    APP_START_FAULT_FEEDBACK_TIMEOUT,
    APP_START_FAULT_CHASSIS,
    APP_START_FAULT_POSE_QUEUE,
    APP_START_FAULT_POSE_TIMEOUT,
    APP_START_FAULT_PLAN,
    APP_START_FAULT_FOLLOWER
} AppStart_Fault_t;

typedef struct {
    AppStart_Status_t status;
    AppStart_Fault_t fault;
    uint8_t selected_zone; /* 0=未按键；1=PD12；2=PD13，上电只选择一次。 */
    uint32_t pose_sequence; /* 必须等电机任务completed_sequence匹配才允许规划。 */
} AppStart_State_t;

extern AppStart_State_t g_app_start;

#endif /* APP_CONFIG_H */
