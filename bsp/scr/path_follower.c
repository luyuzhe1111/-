/*
 * 路线执行状态机。接收 A* 的八段结果，按顺序向底盘下发离站、转向、前进、对接和入库命令。
 * g_path_follower 中的 stage_index、edge_index、status 用来查看当前在第几段、哪条边和哪种动作。
 */

#include "path_follower.h"

#include <math.h>
#include <string.h>

#include "chassis_runtime.h"

PathFollower_State_t g_path_follower;

static path_plan_result_t s_plan;
static PathPlan_StartProfile_t s_start_profile;
static uint32_t s_exit_resume_cycle;
static uint32_t s_start_exit_cycle;
static uint8_t s_raw_exit_use_actual_start;

/*
 * 离站安全点：地图中的车辆中心坐标，单位mm。
 * 视觉定位贴近圆环后先回到这里，再原地转向。
 * 若修改地图中的加工/暂存节点，也需同步修改这些坐标。
 * 275x300车体自旋半径约203.5 mm；机构收回后再离站，实车核对圆环余量。
 */
/* 两圈原料共用：先保持-90度移到(1200,300)，到位后才转向下一段。
 * 当前A*原料节点为(1280,300)，不修改地图；后续转向从实际离站位置计算。
 */
#define PATH_EXIT_RAW_X_MM         1200.0f
#define PATH_EXIT_RAW_Y_MM          300.0f
#define PATH_EXIT_PROCESS_X_MM     1200.0f
#define PATH_EXIT_PROCESS_Y_MM     2000.0f
#define PATH_EXIT_STORAGE_X_MM     2000.0f
#define PATH_EXIT_STORAGE_Y_MM     1200.0f
#define PATH_EXIT_MAX_RPM            60.0f
#define PATH_EXIT_FEEDBACK_CYCLES       2U
#define PATH_EXIT_FEEDBACK_TIMEOUT     50U /* 约1秒未恢复有效回读则停止路径。 */
#define PATH_START_EXIT_MAX_RPM        80.0f
#define PATH_START_EXIT_TIMEOUT_CYCLES 400U /* 20ms周期约8秒，离站异常不能一直跑。 */
#define PATH_PARK_MAX_RPM               60.0f /* 2区安全节点到蓝块中心，严格入库。 */

static uint8_t PathFollower_GetExitTarget(uint8_t stage,
                                         float *x, float *y, float *yaw)
{
    if (stage == 1U || stage == 4U) {
        *x = PATH_EXIT_RAW_X_MM;
        *y = PATH_EXIT_RAW_Y_MM;
        *yaw = -90.0f;
        return 1U;
    }
    if (stage == 2U || stage == 5U) {
        *x = PATH_EXIT_PROCESS_X_MM;
        *y = PATH_EXIT_PROCESS_Y_MM;
        *yaw = 90.0f;
        return 1U;
    }
    if (stage == 3U || stage == 6U) {
        *x = PATH_EXIT_STORAGE_X_MM;
        *y = PATH_EXIT_STORAGE_Y_MM;
        *yaw = 0.0f;
        return 1U;
    }
    return 0U;
}

static void PathFollower_AdvanceStage(void)
{
    g_path_follower.stage_index++;
    g_path_follower.edge_index = 0U;
    g_path_follower.active_sequence = 0U;
    g_path_follower.status = PATH_FOLLOWER_WAIT_CHASSIS;
}

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static void PathFollower_SetFault(PathFollower_Fault_t fault)
{
    g_path_follower.fault = fault;
    g_path_follower.status = PATH_FOLLOWER_FAULT;
    (void)ChassisRuntime_Stop();
}

/*
 * 三个工位的视觉和机构都从车辆右侧工作，扫码模块位于相对的左侧。
 * 航向0度车头朝Y+，此时车辆右侧朝X+：
 * 原料区位于车辆Y-侧，所以航向-90度；
 * 加工区位于车辆Y+侧，所以航向+90度；
 * 暂存区位于车辆X+侧，所以航向0度。
 */
static uint8_t PathFollower_GetDockingYaw(uint8_t completed_stage,
                                          float *yaw_deg)
{
    if (yaw_deg == NULL) return 0U;

    /* 最终回库按Start时锁定的配置执行，不受之后误改全局选区影响。 */
    if (completed_stage == PP_MAX_STAGES - 1U) return 0U;

    /* 和A*共用朝向表，修改工位朝向后规划成本与执行保持一致。 */
    return (uint8_t)path_plan_stage_docking_yaw(completed_stage, yaw_deg);
}

static void PathFollower_FinishWaypoint(void)
{
    /* 收到对应运动完成后，将已完成道路数与本段道路下标各加一。走完本段最后一条道路才开始对接。 */
    g_path_follower.completed_waypoints++;
    g_path_follower.edge_index++;

    if (g_path_follower.edge_index >=
        (uint8_t)s_plan.length[g_path_follower.stage_index]) {
        float docking_yaw;
        uint32_t sequence;

        /* 先完成扫码侧/工作侧的对接朝向，再允许执行区域动作。 */
        g_path_follower.completed_stage = g_path_follower.stage_index;
        g_path_follower.signed_edge = 0;

        if (g_path_follower.completed_stage == PP_MAX_STAGES - 1U &&
            s_start_profile.final_parking_required != 0U) {
            ChassisRuntime_State_t chassis;
            ChassisRuntime_GetState(&chassis);
            /* 2区先回到A*终点(右下安全节点)，不能在蓝块中心自旋。
             * 严格到点完成后再转正；额外入库不计入规划边/路径点数量。
             */
            if (chassis.status != CHASSIS_RUNTIME_DONE ||
                chassis.nav_initialized == 0U || chassis.encoder_query_enabled == 0U ||
                chassis.encoder_valid_mask != CHASSIS_NAV_ENCODER_ALL_VALID ||
                !isfinite(chassis.nav.pose.x_mm) || !isfinite(chassis.nav.pose.y_mm) ||
                hypotf(chassis.nav.pose.x_mm - g_path_follower.target_x_mm,
                       chassis.nav.pose.y_mm - g_path_follower.target_y_mm) >
                    CHASSIS_NAV_TRANSIT_POSITION_MM) {
                PathFollower_SetFault(PATH_FOLLOWER_FAULT_CHASSIS);
                return;
            }
            sequence = 0U;
            if (!ChassisRuntime_TurnTo(s_start_profile.park_yaw_deg, &sequence)) {
                PathFollower_SetFault(PATH_FOLLOWER_FAULT_QUEUE);
                return;
            }
            g_path_follower.active_sequence = sequence;
            g_path_follower.target_yaw_deg = s_start_profile.park_yaw_deg;
            g_path_follower.status = PATH_FOLLOWER_PARKING_TURN;
            return;
        }

        /* 道路航向使车沿通道行驶，工位朝向使相机和抓手面向目标。这里通常要额外提交一次转向。 */
        if (PathFollower_GetDockingYaw(g_path_follower.completed_stage,
                                       &docking_yaw) != 0U) {
            sequence = 0U;
            if (!ChassisRuntime_TurnTo(docking_yaw, &sequence)) {
                PathFollower_SetFault(PATH_FOLLOWER_FAULT_QUEUE);
                return;
            }
            g_path_follower.active_sequence = sequence;
            g_path_follower.target_yaw_deg = docking_yaw;
            g_path_follower.status = PATH_FOLLOWER_DOCKING_TURN;
        } else {
            g_path_follower.status = PATH_FOLLOWER_WAIT_ACTION;
        }
    } else {
        g_path_follower.status = PATH_FOLLOWER_WAIT_CHASSIS;
    }
}

/* 接收规划结果并保存本次启停区配置。开始执行后用该配置处理出发和回库。 */
bool PathFollower_Start(const path_plan_result_t *plan)
{
    uint16_t total = 0U;
    uint8_t stage;

    memset(&g_path_follower, 0, sizeof(g_path_follower));
    if (plan == NULL) {
        PathFollower_SetFault(PATH_FOLLOWER_FAULT_BAD_PLAN);
        return false;
    }

    /* start_zone=0兼容旧工程手动构造的规划结果；非0必须和当前选区一致。 */
    /* 当前选区须与规划结果记录的选区一致，否则终点、离站点和地图入口可能对不上。 */
    if (plan->start_zone != 0U && plan->start_zone != path_plan_get_start_zone()) {
        PathFollower_SetFault(PATH_FOLLOWER_FAULT_BAD_PLAN);
        return false;
    }

    for (stage = 0U; stage < PP_MAX_STAGES; stage++) {
        if (plan->ok[stage] == 0 || plan->length[stage] < 0 ||
            plan->length[stage] > PP_MAX_PATH_LEN) {
            PathFollower_SetFault(PATH_FOLLOWER_FAULT_BAD_PLAN);
            return false;
        }
        total = (uint16_t)(total + (uint16_t)plan->length[stage]);
    }
    if (total == 0U) {
        PathFollower_SetFault(PATH_FOLLOWER_FAULT_BAD_PLAN);
        return false;
    }

    s_plan = *plan;
    s_start_profile = *path_plan_get_start_profile();
    s_exit_resume_cycle = 0U;
    s_start_exit_cycle = 0U;
    s_raw_exit_use_actual_start = 0U;
    g_path_follower.completed_stage = 0xFFU;
    g_path_follower.start_zone = s_start_profile.zone;
    g_path_follower.total_waypoints = total;
    g_path_follower.status = PATH_FOLLOWER_WAIT_CHASSIS;
    return true;
}

/* 只省去普通路径上的重复转向，不越过工位严格对接或离站避让。
 * 读取一致的反馈快照；转速仍大、回读关闭/无效时不允许直接前进。
 * 残余航向由GotoXY闭环在下一段行驶中继续修正。
 */
static uint8_t PathFollower_HeadingReady(float target_yaw)
{
    ChassisRuntime_State_t chassis;
    float error;
    ChassisRuntime_GetState(&chassis);
    if (chassis.nav_initialized == 0U || chassis.encoder_query_enabled == 0U ||
        chassis.encoder_valid_mask != CHASSIS_NAV_ENCODER_ALL_VALID ||
        (chassis.status != CHASSIS_RUNTIME_IDLE && chassis.status != CHASSIS_RUNTIME_DONE) ||
        chassis.turn_phase != CHASSIS_TURN_PHASE_NONE ||
        !isfinite(chassis.nav.pose.yaw_deg) ||
        !isfinite(chassis.nav.velocity.wz_deg_s) ||
        !isfinite(chassis.nav.velocity.vx_mm_s) ||
        !isfinite(chassis.nav.velocity.vy_mm_s)) return 0U;
    error = target_yaw - chassis.nav.pose.yaw_deg;
    while (error > 180.0f) error -= 360.0f;
    while (error <= -180.0f) error += 360.0f;
    return (fabsf(error) <= CHASSIS_NAV_TRANSIT_HEADING_DEG &&
            fabsf(chassis.nav.velocity.wz_deg_s) <= CHASSIS_NAV_TRANSIT_YAW_RATE_DEG_S &&
            hypotf(chassis.nav.velocity.vx_mm_s, chassis.nav.velocity.vy_mm_s) <=
                CHASSIS_NAV_TRANSIT_SPEED_MM_S) ? 1U : 0U;
}

static void PathFollower_StartForward(void)
{
    bool accepted;
    uint32_t sequence = 0U;
    /* 最后停车仍严格；普通路径残差在运动中继续校正。 */
    if (g_path_follower.stage_index == PP_MAX_STAGES - 1U &&
        g_path_follower.edge_index + 1U >=
            (uint8_t)s_plan.length[g_path_follower.stage_index]) {
        accepted = ChassisRuntime_GotoXY(g_path_follower.target_x_mm,
                                        g_path_follower.target_y_mm,
                                        g_path_follower.target_yaw_deg, &sequence);
    } else {
        accepted = ChassisRuntime_GotoXYTransit(g_path_follower.target_x_mm,
                                               g_path_follower.target_y_mm,
                                               g_path_follower.target_yaw_deg, &sequence);
    }
    if (!accepted) {
        PathFollower_SetFault(PATH_FOLLOWER_FAULT_QUEUE);
        return;
    }
    g_path_follower.active_sequence = sequence;
    g_path_follower.status = PATH_FOLLOWER_MOVING_FORWARD;
}

/* 每次推进当前状态，不在这里阻塞等待整段运动。进入下一个动作前会检查命令序号、反馈和完成状态。 */
void PathFollower_Process(void)
{
    uint32_t sequence;
    float start_x;
    float start_y;
    float target_x;
    float target_y;
    float delta_x;
    float delta_y;
    float target_yaw;
    int signed_edge;

    if (g_path_follower.status == PATH_FOLLOWER_IDLE ||                        //判断这些状态不推进路线
        g_path_follower.status == PATH_FOLLOWER_DONE ||
        g_path_follower.status == PATH_FOLLOWER_FAULT ||
        g_path_follower.status == PATH_FOLLOWER_WAIT_ACTION) {           
        return;
    }

    /* 底盘故障由运行时上报，路径执行器将其记录为自己的故障并退出本次推进。 */
    if (g_chassis_runtime.status == CHASSIS_RUNTIME_FAULT) {                //检查底盘有没有故障
        PathFollower_SetFault(PATH_FOLLOWER_FAULT_CHASSIS);
        return;
    }

    /* 出发安全动作先保持摆车航向平移。只有命令序号相符且反馈坐标落入安全点范围，才会继续处理第一条路。 */
    if (g_path_follower.status == PATH_FOLLOWER_START_EXIT_MOVING) {              //起步离站过程中：确认已经进入安全位置
        ChassisRuntime_State_t chassis;
        ChassisRuntime_GetState(&chassis);
        if (chassis.completed_sequence == g_path_follower.active_sequence &&
            g_path_follower.active_sequence != 0U &&
            chassis.status == CHASSIS_RUNTIME_DONE &&
            chassis.encoder_valid_mask == CHASSIS_NAV_ENCODER_ALL_VALID) {
            /* 不能仅靠序号放行；位置必须进入安全点容差圈，才允许转向。
             * 转向半径203.5mm，20mm位置容差仍保留距两侧边界约76.5/126.5mm余量。
             */
            if (!isfinite(chassis.nav.pose.x_mm) || !isfinite(chassis.nav.pose.y_mm) ||
                hypotf(chassis.nav.pose.x_mm - s_start_profile.exit_x_mm,
                       chassis.nav.pose.y_mm - s_start_profile.exit_y_mm) >
                    CHASSIS_NAV_TRANSIT_POSITION_MM) {
                PathFollower_SetFault(PATH_FOLLOWER_FAULT_CHASSIS);
                return;
            }
            g_path_follower.startup_exit_done = 1U;
            g_path_follower.active_sequence = 0U;
            g_path_follower.status = PATH_FOLLOWER_WAIT_CHASSIS;
            /* 同次调用继续第一条边，不加固定延时，也不增加已完成路径点数。 */
        } else {
            if (chassis.cycle_count - s_start_exit_cycle >= PATH_START_EXIT_TIMEOUT_CYCLES)
                PathFollower_SetFault(PATH_FOLLOWER_FAULT_CHASSIS);
            return;
        }
    }

    /* 回 2 号区时先到场内安全节点，原地转到停车航向后再保持该航向移入蓝块中心。 */
    if (g_path_follower.status == PATH_FOLLOWER_PARKING_TURN ||        //2号区回库：必须先转正，再移入停车中心
        g_path_follower.status == PATH_FOLLOWER_PARKING_MOVING) {
        ChassisRuntime_State_t chassis;
        ChassisRuntime_GetState(&chassis);
        if (g_path_follower.active_sequence == 0U ||
            chassis.completed_sequence != g_path_follower.active_sequence ||
            chassis.status != CHASSIS_RUNTIME_DONE ||
            chassis.nav_initialized == 0U || chassis.encoder_query_enabled == 0U ||
            chassis.encoder_valid_mask != CHASSIS_NAV_ENCODER_ALL_VALID) return;
        if (g_path_follower.status == PATH_FOLLOWER_PARKING_TURN) {
            /* 严格转正完成后才入库，当前航向在GotoXY中持续闭环保持。 */
            sequence = 0U;
            if (!ChassisRuntime_GotoXYLimited(s_start_profile.park_x_mm,
                                               s_start_profile.park_y_mm,
                                               s_start_profile.park_yaw_deg,
                                               PATH_PARK_MAX_RPM, &sequence)) {
                PathFollower_SetFault(PATH_FOLLOWER_FAULT_QUEUE);
                return;
            }
            g_path_follower.target_x_mm = s_start_profile.park_x_mm;
            g_path_follower.target_y_mm = s_start_profile.park_y_mm;
            g_path_follower.target_yaw_deg = s_start_profile.park_yaw_deg;
            g_path_follower.active_sequence = sequence;
            g_path_follower.status = PATH_FOLLOWER_PARKING_MOVING;
        } else {
            g_path_follower.status = PATH_FOLLOWER_WAIT_ACTION;
        }
        return;
    }

    /* 机构动作结束后会重新开启编码器查询。这里等新的底盘周期与四轮有效反馈，再发离站位移。 */
    if (g_path_follower.status == PATH_FOLLOWER_EXIT_WAIT_FEEDBACK) {                   //工位动作结束：先等新编码器反馈
        uint32_t cycles = g_chassis_runtime.cycle_count - s_exit_resume_cycle;
        /* 开关关闭期间视觉移动不会实时更新里程计，必须先取得新回读。 */
        if (cycles >= PATH_EXIT_FEEDBACK_CYCLES &&
            g_chassis_runtime.encoder_valid_mask == CHASSIS_NAV_ENCODER_ALL_VALID &&
            g_chassis_runtime.nav_initialized != 0U) {
            sequence = 0U;
            if (!ChassisRuntime_GotoXYLimited(g_path_follower.target_x_mm,
                                               g_path_follower.target_y_mm,
                                               g_path_follower.target_yaw_deg,
                                               PATH_EXIT_MAX_RPM, &sequence)) {
                PathFollower_SetFault(PATH_FOLLOWER_FAULT_QUEUE);
                return;
            }
            g_path_follower.active_sequence = sequence;
            g_path_follower.status = PATH_FOLLOWER_EXIT_MOVING;
        } else if (cycles >= PATH_EXIT_FEEDBACK_TIMEOUT) {
            PathFollower_SetFault(PATH_FOLLOWER_FAULT_CHASSIS);
        }
        return;
    }

    if (g_path_follower.status == PATH_FOLLOWER_EXIT_MOVING) {                     //几个“正在运动”的状态：等对应命令完成
        if (g_chassis_runtime.completed_sequence ==
                g_path_follower.active_sequence &&
            g_path_follower.active_sequence != 0U) {
            /* 同次调用准备下一条边，不再额外等一个主任务周期。 */
            if (g_path_follower.completed_stage == 1U ||
                g_path_follower.completed_stage == 4U) {
                s_raw_exit_use_actual_start = 1U;
            }
            PathFollower_AdvanceStage();
        } else return;
    }

    /* 道路转向完成后变成 WAIT_FORWARD，下一个分支提交位移。命令序号用于区分刚完成的动作和之前的动作。 */
    if (g_path_follower.status == PATH_FOLLOWER_TURNING) {                         
        if (g_chassis_runtime.completed_sequence ==
                g_path_follower.active_sequence &&
            g_path_follower.active_sequence != 0U) {
            g_path_follower.status = PATH_FOLLOWER_WAIT_FORWARD;
        } else return;
    }

    /* 每条道路的前进都在这里等待结束。结束时更新道路下标，走完该段则开始工位对接。 */
    if (g_path_follower.status == PATH_FOLLOWER_MOVING_FORWARD) {                    
        if (g_chassis_runtime.completed_sequence ==
                g_path_follower.active_sequence &&
            g_path_follower.active_sequence != 0U) {
            PathFollower_FinishWaypoint();
            /* 工位对接已经下发命令；只在普通中间点继续准备下一条边。 */
            if (g_path_follower.status != PATH_FOLLOWER_WAIT_CHASSIS) return;
        } else return;
    }

    /* 确认对接转向完成才进入 WAIT_ACTION。此时 freertos.c 读取 completed_stage 并调用该站动作。 */
    if (g_path_follower.status == PATH_FOLLOWER_DOCKING_TURN) {                  
        if (g_chassis_runtime.completed_sequence ==
                g_path_follower.active_sequence &&
            g_path_follower.active_sequence != 0U) {
            g_path_follower.status = PATH_FOLLOWER_WAIT_ACTION;
        }
        return;
    }

    if (g_chassis_runtime.status != CHASSIS_RUNTIME_IDLE &&                       //底盘忙着时，不提交下一条运动   
        g_chassis_runtime.status != CHASSIS_RUNTIME_DONE) {
        return;
    }

    if (g_path_follower.status == PATH_FOLLOWER_WAIT_FORWARD) {                   //转向完成了，就提交前进     
        PathFollower_StartForward();
        return;
    }

    /* 第一次起步先提交启停区离站目标。此动作保留摆放时的航向，然后才按第一条道路的航向转弯。 */
    if (g_path_follower.stage_index == 0U && g_path_follower.edge_index == 0U &&   //第一次起步：先离开启停区，不允许直接旋转
        g_path_follower.startup_exit_done == 0U) {
        ChassisRuntime_State_t chassis;
        ChassisRuntime_GetState(&chassis);
        if (chassis.nav_initialized == 0U || chassis.encoder_query_enabled == 0U ||
            chassis.encoder_valid_mask != CHASSIS_NAV_ENCODER_ALL_VALID) return;
        sequence = 0U;
        if (!ChassisRuntime_GotoXYLimitedTransit(s_start_profile.exit_x_mm,
                                                  s_start_profile.exit_y_mm,
                                                  s_start_profile.exit_yaw_deg,
                                                  PATH_START_EXIT_MAX_RPM, &sequence)) {
            PathFollower_SetFault(PATH_FOLLOWER_FAULT_QUEUE);
            return;
        }
        s_start_exit_cycle = chassis.cycle_count;
        g_path_follower.target_x_mm = s_start_profile.exit_x_mm;
        g_path_follower.target_y_mm = s_start_profile.exit_y_mm;
        g_path_follower.target_yaw_deg = s_start_profile.exit_yaw_deg;
        g_path_follower.active_sequence = sequence;
        g_path_follower.signed_edge = 0;
        g_path_follower.status = PATH_FOLLOWER_START_EXIT_MOVING;
        return;
    }

    signed_edge = s_plan.edge_path[g_path_follower.stage_index]                //解析道路并检查是否有效
                                    [g_path_follower.edge_index];
    /* 规划只给有符号道路号，执行前要展开为该方向的起终点坐标，再计算道路航向。 */
    if (path_plan_edge_points(signed_edge, &start_x, &start_y,
                              &target_x, &target_y) == 0) {
        PathFollower_SetFault(PATH_FOLLOWER_FAULT_BAD_EDGE);
        return;
    }

    if (g_path_follower.edge_index == 0U &&
        ((g_path_follower.stage_index == 0U &&
          g_path_follower.startup_exit_done != 0U) ||
         s_raw_exit_use_actual_start != 0U)) {
        /* 启停区/原料离站后的首段使用实际位置，不能沿用旧节点的转向角。 */
        ChassisRuntime_State_t chassis;
        ChassisRuntime_GetState(&chassis);
        start_x = chassis.nav.pose.x_mm;
        start_y = chassis.nav.pose.y_mm;
        s_raw_exit_use_actual_start = 0U;
    }

    delta_x = target_x - start_x;
    delta_y = target_y - start_y;
    if (fabsf(delta_x) < 0.1f && fabsf(delta_y) < 0.1f) {
        PathFollower_SetFault(PATH_FOLLOWER_FAULT_BAD_EDGE);
        return;
    }

    /*
     * 地图X+向左、Y+向下，车辆yaw=0朝Y+。
     * yaw = atan2(-dx, dy)：下0度、左-90度、上180度、右+90度。
     */
    target_yaw = atan2f(-delta_x, delta_y) *                         //最后判断：直接前进，还是先转向
                 (180.0f / (float)M_PI); 

    g_path_follower.signed_edge = (int16_t)signed_edge;
    g_path_follower.target_x_mm = target_x;
    g_path_follower.target_y_mm = target_y;
    g_path_follower.target_yaw_deg = target_yaw;

    g_path_follower.turn_skipped = 0U;
    /* 如果当前航向已经满足通行容差，可以直接发前进目标。其他情况先发转向目标。 */
    if (PathFollower_HeadingReady(target_yaw) != 0U) {
        PathFollower_StartForward();
        if (g_path_follower.status == PATH_FOLLOWER_MOVING_FORWARD) {
            g_path_follower.turn_skipped = 1U;
            g_path_follower.skipped_turns++;
        }
        return;
    }

    /* 大角度换向仍先原地转向，避免横向跑向下一条路。 */
    sequence = 0U;
    if (!ChassisRuntime_TurnToTransit(target_yaw, &sequence)) {
        PathFollower_SetFault(PATH_FOLLOWER_FAULT_QUEUE);
        return;
    }
    g_path_follower.active_sequence = sequence;
    g_path_follower.status = PATH_FOLLOWER_TURNING;
}

/* 主任务确认到站动作完成后调用。按工位安排离站步骤，最后一段完成后结束路线。 */
bool PathFollower_Continue(void)
{
    if (g_path_follower.status != PATH_FOLLOWER_WAIT_ACTION) return false;

    if (g_path_follower.completed_stage >= (PP_MAX_STAGES - 1U)) {
        g_path_follower.status = PATH_FOLLOWER_DONE;
        return true;
    }

    /* 有明确离站目标的工位会先以工位航向平移退出，回到安全位置后才开始下一段路径。 */
    if (PathFollower_GetExitTarget(g_path_follower.completed_stage,
                                    &g_path_follower.target_x_mm,
                                    &g_path_follower.target_y_mm,
                                    &g_path_follower.target_yaw_deg) != 0U) {
        /* 动作已返回：机构必须收回，并结束对USART3的访问。 */
        s_exit_resume_cycle = g_chassis_runtime.cycle_count;
        ChassisRuntime_SetEncoderQueryEnabled(true);
        g_path_follower.active_sequence = 0U;
        g_path_follower.status = PATH_FOLLOWER_EXIT_WAIT_FEEDBACK;
        return true;
    }

    PathFollower_AdvanceStage();
    return true;
}

void PathFollower_Stop(void)
{
    (void)ChassisRuntime_Stop();
    memset(&g_path_follower, 0, sizeof(g_path_follower));
}
