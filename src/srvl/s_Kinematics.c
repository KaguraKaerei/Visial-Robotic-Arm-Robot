#include "s_Kinematics.h"
#include "Servo_d.h"
#include <math.h>
#include <stdio.h>

/* ========================= 接 口 变 量 声 明 ========================= */



/* ========================= 私 有 变 量 声 明 ========================= */

#define PI                  3.14159265f
#define ANGLE_TO_RAD(angle) ((angle) * 3.14159265f / 180.0f)
#define RAD_TO_ANGLE(rad)   ((rad) * 180.0f / 3.14159265f)
#define LINK1_LENGTH        0.120f      // 单位：米
#define LINK2_LENGTH        0.10242f    // 单位：米
#define LINK3_LENGTH        0.06762f    // 单位：米
#define SERVO_TYPE          270.0f      // 270度舵机
#define CCR_MIN             500
#define CCR_MAX             2500

#define AIM_POINT_X         1052
#define AIM_POINT_Y         591
#define LASER_POINT_X       960
#define LASER_POINT_Y       520

#define PIXEL_TO_ANGLE_YAW    0.032f     // 1像素 : 0.05度底座旋转
#define PIXEL_TO_ANGLE_PITCH  0.018f    // 1像素 : 0.03度俯仰调整

// TODO: 角度范围表，确定上下限关节角和初始角
typedef struct{
    float chassis[3];
    float joint1[3];
    float joint2[3];
    float joint3[3];
    float gripper[3];
} AngleRange_t;

// 最小ccr对应角、最大ccr对应角、初始角、最小限制角、最大限制角
static AngleRange_t angle_range = {
    .chassis = {-135.0f, 135.0f, 0.0f},         // 500 ~ -135, 2500 ~ 135
    .joint1 = {225.0f, -45.0f, 165.0f},          // 500 ~ 225, 2500 ~ -45
    .joint2 = {45.0f, 315.0f, 60.0f},          // 500 ~ 45, 2500 ~ 315
    .joint3 = {315.0f, 45.0f, 100.0f},          // 500 ~ 315, 2500 ~ 45
    .gripper = {0.0f, 180.0f, 90.0f},           // 500 ~ 0, 2500 ~ 90
};

/* ========================= 私 有 函 数 声 明 ========================= */

static bool Arm_SetJointAngle(Servo_ID_t servo_id, float angle);
static float Arm_GetJointAngle(Servo_ID_t servo_id);
#define _constrain(val, min, max)   ( (val) < (min) ? (min) : ( (val) > (max) ? (max) : (val) ) )

/* ========================= 接 口 函 数 实 现 ========================= */

void Arm_Kinematics_Init(void)
{
    // 初始姿态
    Arm_SetJointAngle(SERVO_CHASSIS, angle_range.chassis[2]);
    Arm_SetJointAngle(SERVO_JOINT_1, angle_range.joint1[2]);
    Arm_SetJointAngle(SERVO_JOINT_2, angle_range.joint2[2]);
    Arm_SetJointAngle(SERVO_JOINT_3, angle_range.joint3[2]);
    Arm_SetJointAngle(SERVO_JOINT_GRIPPER, angle_range.gripper[2]);
}

/**
 * @brief 机械臂瞄准目标点(需外部闭环控制)
 * @param target 目标点在相机坐标系下的二维坐标
 * @param use_laser 是激光还是夹爪，true表示使用激光点，false表示使用夹爪点
 * @return 瞄准成功返回true，失败返回false
 */
bool Arm_AimAtTarget(Coord_2D_t target, bool use_laser)
{
    // 计算像素偏差
    float offset_x, offset_y;
    if(use_laser){
        offset_x = target.r - LASER_POINT_X;
        offset_y = target.z - LASER_POINT_Y;
    }
    else{
        offset_x = target.r - AIM_POINT_X;
        offset_y = target.z - AIM_POINT_Y;
    }

    // 获取当前关节角
    float theta0 = Arm_GetJointAngle(SERVO_CHASSIS);
    float theta3 = Arm_GetJointAngle(SERVO_JOINT_3);

    // 计算角度增量（像素偏差 × 增益）
    // offset_x > 0 表示目标在右侧，需向右转（theta0减小）
    float delta_theta0 = offset_x * PIXEL_TO_ANGLE_YAW;
    // offset_y > 0 表示目标在下方，需俯仰向下（theta3减小）
    float delta_theta3 = offset_y * PIXEL_TO_ANGLE_PITCH;

    // 更新关节角（增量调整）
    float theta0_new = theta0 - delta_theta0;
    float theta3_new = theta3 - delta_theta3;

    printf("Move to theta0: %.2f -> %.2f, theta3: %.2f -> %.2f\r\n", theta0, theta0_new, theta3, theta3_new);

    // 设置关节角
    bool res1 = Arm_SetJointAngle(SERVO_CHASSIS, theta0_new);
    bool res2 = Arm_SetJointAngle(SERVO_JOINT_3, theta3_new);

    return res1 && res2;
}

/**
 * @brief 相机目标点从相机坐标系到基坐标系的坐标变换
 * @param depth 目标点到相机的深度，单位：米
 * @param target_base 目标点在基坐标系下的坐标
 * @return 转换成功返回true，失败返回false
 */
bool Arm_TF_TargetToBase(float depth, Coord_3D_t* target_base)
{
    /* 机械臂连杆所在平面下的坐标系变换 */
    float theta0 = Arm_GetJointAngle(SERVO_CHASSIS); theta0 = ANGLE_TO_RAD(theta0);
    float theta1 = Arm_GetJointAngle(SERVO_JOINT_1); theta1 = ANGLE_TO_RAD(theta1);
    float theta2 = Arm_GetJointAngle(SERVO_JOINT_2); theta2 = ANGLE_TO_RAD(theta2);
    float theta3 = Arm_GetJointAngle(SERVO_JOINT_3); theta3 = ANGLE_TO_RAD(theta3);
    // 夹爪坐标转换
    float x_eef_2 = LINK3_LENGTH * cosf(theta3);
    float y_eef_2 = LINK3_LENGTH * sinf(theta3);
    float x_eef_1 = (-cosf(theta2) * x_eef_2) + (sinf(theta2) * y_eef_2) + (LINK2_LENGTH * cosf(theta2));
    float y_eef_1 = (-sinf(theta2) * x_eef_2) + (-cosf(theta2) * y_eef_2) + (LINK2_LENGTH * sinf(theta2));
    float x_eef = (-cosf(theta1) * x_eef_1) + (sinf(theta1) * y_eef_1) + (LINK1_LENGTH * cosf(theta1));
    float y_eef = (-sinf(theta1) * x_eef_1) + (-cosf(theta1) * y_eef_1) + (LINK1_LENGTH * sinf(theta1));
    // 目标点坐标转换
    float phi = (theta1 + theta2 + theta3) - 2.0f * PI;
    float x_target = x_eef + depth * cosf(phi);
    float y_target = y_eef + depth * sinf(phi);

    /* 平面坐标系转换为三维坐标系 */
    float r_target = x_target;
    target_base->x = r_target * cosf(theta0);
    target_base->y = r_target * sinf(theta0);
    target_base->z = y_target;
    target_base->angle = RAD_TO_ANGLE(phi);

    return true;
}

/**
 * @brief 机械臂逆运动学解算（基于已瞄准的目标点）
 * @param depth 目标点到相机的深度，单位：米
 * @param angle 目标点的姿态角，单位：度
 * @return 解算成功返回true，失败返回false
 * @note 目标点需要先瞄准，瞄准了之后才能解算，瞄准后在平面内解算
 */
bool Arm_InverseKinematicsWithAim(float depth, float angle)
{
    float theta1, theta2, theta3;
    Coord_3D_t target_base;
    Arm_TF_TargetToBase(depth, &target_base);
    Coord_2D_t target_plane;
    target_plane.r = sqrtf((target_base.x * target_base.x) + (target_base.y * target_base.y));
    target_plane.z = target_base.z;
    // 约束姿态角
    target_plane.angle = angle;

    /* 逆运动学解算：目标位置基础上，关节3位置由姿态角确定，连杆2和连杆1位置与关节3位置构成三角形 */
    float phi = ANGLE_TO_RAD(target_plane.angle);
    float x_joint3 = target_plane.r - LINK3_LENGTH * cosf(phi);
    float y_joint3 = target_plane.z - LINK3_LENGTH * sinf(phi);
    float joint1_3_dist = sqrtf((x_joint3 * x_joint3) + (y_joint3 * y_joint3));
    // 判断是否可达
    float max_reach = LINK1_LENGTH + LINK2_LENGTH;
    float min_reach = fabsf(LINK1_LENGTH - LINK2_LENGTH);
    if(joint1_3_dist > max_reach || joint1_3_dist < min_reach) return false;
    // 余弦定理求解关节2角度（关节2向上解）和关节1角度，关节3角度由姿态角确定
    theta2 = acosf((LINK1_LENGTH * LINK1_LENGTH + LINK2_LENGTH * LINK2_LENGTH - joint1_3_dist * joint1_3_dist) / (2.0f * LINK1_LENGTH * LINK2_LENGTH));
    theta1 = atan2f(y_joint3, x_joint3) + acosf((LINK1_LENGTH * LINK1_LENGTH + joint1_3_dist * joint1_3_dist - LINK2_LENGTH * LINK2_LENGTH) / (2.0f * LINK1_LENGTH * joint1_3_dist));
    theta3 = (2.0f * PI + phi) - (theta1 + theta2);

    // 设置关节角
    Arm_SetJointAngle(SERVO_JOINT_1, RAD_TO_ANGLE(theta1));
    Arm_SetJointAngle(SERVO_JOINT_2, RAD_TO_ANGLE(theta2));
    Arm_SetJointAngle(SERVO_JOINT_3, RAD_TO_ANGLE(theta3));

    return true;
}

/**
 * @brief 机械臂逆运动学解算（基于底座坐标系下的三维目标点）
 * @param x 目标点x坐标，单位：米
 * @param y 目标点y坐标，单位：米
 * @param z 目标点z坐标，单位：米
 * @param angle 目标点姿态角，单位：度
 * @return 解算成功返回true，失败返回false
 */
bool Arm_InverseKinematics(float x, float y, float z, float angle)
{
    float theta0 = atan2f(y, x);
    float r = sqrtf((x * x) + (y * y));
    float theta1, theta2, theta3;
    // 约束姿态角
    float phi = ANGLE_TO_RAD(angle);
    float x_joint3 = r - LINK3_LENGTH * cosf(phi);
    float y_joint3 = z - LINK3_LENGTH * sinf(phi);
    float joint1_3_dist = sqrtf((x_joint3 * x_joint3) + (y_joint3 * y_joint3));
    // 判断是否可达
    float max_reach = LINK1_LENGTH + LINK2_LENGTH;
    float min_reach = fabsf(LINK1_LENGTH - LINK2_LENGTH);
    if(joint1_3_dist > max_reach || joint1_3_dist < min_reach){
        printf("IK Fail: Unreachable Target!\r\n");
        return false;
    }
    // 余弦定理求解关节2角度（关节2向上解）和关节1角度，关节3角度由姿态角确定
    theta2 = acosf((LINK1_LENGTH * LINK1_LENGTH + LINK2_LENGTH * LINK2_LENGTH - joint1_3_dist * joint1_3_dist) / (2.0f * LINK1_LENGTH * LINK2_LENGTH));
    theta1 = atan2f(y_joint3, x_joint3) + acosf((LINK1_LENGTH * LINK1_LENGTH + joint1_3_dist * joint1_3_dist - LINK2_LENGTH * LINK2_LENGTH) / (2.0f * LINK1_LENGTH * joint1_3_dist));
    theta3 = (2.0f * PI + phi) - (theta1 + theta2);

    // 设置关节角
    Arm_SetJointAngle(SERVO_CHASSIS, RAD_TO_ANGLE(theta0));
    Arm_SetJointAngle(SERVO_JOINT_1, RAD_TO_ANGLE(theta1));
    Arm_SetJointAngle(SERVO_JOINT_2, RAD_TO_ANGLE(theta2));
    Arm_SetJointAngle(SERVO_JOINT_3, RAD_TO_ANGLE(theta3));

    return true;
}

/**
 * @brief 设置机械臂夹爪角度
 * @param angle 夹爪角度，单位：度
 * @return 设置成功返回true，失败返回false
 */
bool Arm_SetGripperAngle(float angle)
{
    return Arm_SetJointAngle(SERVO_JOINT_GRIPPER, angle);
}

/* ========================= 私 有 函 数 实 现 ========================= */

/**
 * @brief 角度线性映射为CCR值
 * @param angle 角度值
 * @param angle_of_min_ccr 最小CCR对应的角度值
 * @param angle_of_max_ccr 最大CCR对应的角度值
 * @return 映射后并四舍五入的CCR值
 */
static uint16_t AngleToCCR(float angle, float angle_of_min_ccr, float angle_of_max_ccr)
{
    float angle_range = angle_of_max_ccr - angle_of_min_ccr;
    if(fabsf(angle_range) < 1e-6f) return CCR_MIN;
    float t = (angle - angle_of_min_ccr) / angle_range;
    float ccr = (float)CCR_MIN + t * (float)(CCR_MAX - CCR_MIN);
    ccr = (uint16_t)_constrain(ccr + 0.5f, CCR_MIN, CCR_MAX);

    return ccr;
}

/**
 * @brief CCR值线性映射为角度值
 * @param ccr CCR值
 * @param angle_of_min_ccr 最小CCR对应的角度值
 * @param angle_of_max_ccr 最大CCR对应的角度值
 * @return 映射后的角度值
 */
static float CCRToAngle(uint16_t ccr, float angle_of_min_ccr, float angle_of_max_ccr)
{
    float angle_range = angle_of_max_ccr - angle_of_min_ccr;
    if(fabsf(angle_range) < 1e-6f) return angle_of_min_ccr;
    float t = (float)(ccr - CCR_MIN) / (float)(CCR_MAX - CCR_MIN);
    float angle = angle_of_min_ccr + t * angle_range;

    return angle;
}

/**
 * @brief 设置机械臂关节角度
 * @param servo_id 关节对应的舵机ID
 * @param angle 关节角度，单位：度
 * @return 设置成功返回true，超限返回false
 */
static bool Arm_SetJointAngle(Servo_ID_t servo_id, float angle)
{
    bool res = true;
    const float* range = 0;
    switch(servo_id){
        case SERVO_CHASSIS:
            range = angle_range.chassis;
            break;
        case SERVO_JOINT_1:
            range = angle_range.joint1;
            break;
        case SERVO_JOINT_2:
            range = angle_range.joint2;
            break;
        case SERVO_JOINT_3:
            range = angle_range.joint3;
            break;
        case SERVO_JOINT_GRIPPER:
            range = angle_range.gripper;
            break;
    }

    float a_min = fminf(range[0], range[1]);
    float a_max = fmaxf(range[0], range[1]);
    if(angle < a_min || angle > a_max){
        res = false;
        angle = _constrain(angle, a_min, a_max);
    }
    Servo_SetCCR(servo_id, AngleToCCR(angle, range[0], range[1]));

    return res;
}

/**
 * @brief 获取机械臂关节角度
 * @param servo_id 关节对应的舵机ID
 * @return 关节角度，单位：度
 */
static float Arm_GetJointAngle(Servo_ID_t servo_id)
{
    const float* range = 0;
    switch(servo_id){
        case SERVO_CHASSIS:
            range = angle_range.chassis;
            break;
        case SERVO_JOINT_1:
            range = angle_range.joint1;
            break;
        case SERVO_JOINT_2:
            range = angle_range.joint2;
            break;
        case SERVO_JOINT_3:
            range = angle_range.joint3;
            break;
        case SERVO_JOINT_GRIPPER:
            range = angle_range.gripper;
            break;
    }

    uint16_t ccr = Servo_GetCCR(servo_id);
    return CCRToAngle(ccr, range[0], range[1]);
}
