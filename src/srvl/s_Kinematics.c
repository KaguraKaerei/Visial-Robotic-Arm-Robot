#include "s_Kinematics.h"
#include "Servo_d.h"
#include <math.h>

/* ========================= 接 口 变 量 声 明 ========================= */



/* ========================= 私 有 变 量 声 明 ========================= */

#define PI                  3.14159265f
#define ANGLE_TO_RAD(angle) ((angle) * 3.14159265f / 180.0f)
#define RAD_TO_ANGLE(rad)   ((rad) * 180.0f / 3.14159265f)
#define LINK1_LENGTH        0.120f      // 单位：米
#define LINK2_LENGTH        0.10242f    // 单位：米
#define LINK3_LENGTH        0.02686f    // 单位：米

// TODO: 相机标定得到的目标点在相机坐标系下的位置（单位：像素），需要手眼标定
#define AIM_POINT_X         400
#define AIM_POINT_Y         300

// TODO: 角度范围表，确定上下限关节角和初始角
typedef struct{
    float chassis[3];
    float joint1[3];
    float joint2[3];
    float joint3[3];
    float gripper[3];
} AngleRange_t;

// 最小角、最大角、初始角
static AngleRange_t angle_range = {
    .chassis = {-80.0f, 100.0f, 10.0f},         // 500 ~ -80, 2500 ~ 100
    .joint1 = {-15.0f, 165.0f, 170.0f},         // 600 ~ 165, 2500 ~ -15
    .joint2 = {80.0f, 260.0f, 80.0f},           // 500-80, 2500-260
    .joint3 = {95.0f, 275.0f, 95.0f},           // 500-275, 2500-95
    .gripper = {0.0f, 90.0f, 45.0f},
};

/* ========================= 私 有 函 数 声 明 ========================= */

static bool Arm_SetJointAngle(Servo_ID_t servo_id, float angle);
static float Arm_GetJointAngle(Servo_ID_t servo_id);

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
 * @brief 机械臂瞄准目标点
 * @param target 目标点在相机坐标系下的二维坐标
 * @return 瞄准成功返回true，失败返回false
 */
bool Arm_AimAtTarget(Coord_2D_t target)
{
    // 获取关节角用于计算底座与夹爪位置关系
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
    // x轴偏移则旋转底座
    float offset_x = target.r - AIM_POINT_X;
    float theta0_new = theta0 + atan2f(offset_x, x_eef);
    bool res = Arm_SetJointAngle(SERVO_CHASSIS, RAD_TO_ANGLE(theta0_new));
    if(!res) return false;
    // y轴偏移则调整机械臂俯仰角度
    float phi = 2.0f * PI - (theta1 + theta2 + theta3);
    float offset_y = target.z - AIM_POINT_Y;
    float phi_new = phi + atan2f(offset_y, x_eef);
    float theta3_new = 2.0f * PI - phi_new - theta1 - theta2;
    res = Arm_SetJointAngle(SERVO_JOINT_3, RAD_TO_ANGLE(theta3_new));
    if(!res) return false;

    return true;
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
    float phi = 2.0f * PI - (theta1 + theta2 + theta3);
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
 * @brief 机械臂逆运动学解算
 * @param depth 目标点到相机的深度，单位：米
 * @param angle 目标点的姿态角，单位：度
 * @return 解算成功返回true，失败返回false
 * @note 目标点需要先瞄准，瞄准了之后才能解算，瞄准后在平面内解算
 */
bool Arm_InverseKinematics(float depth, float angle)
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
    // 余弦定理求解关节2角度（关节2向上解）和关节1角度，关节3角度由姿态角确定
    theta2 = acosf((LINK1_LENGTH * LINK1_LENGTH + LINK2_LENGTH * LINK2_LENGTH - joint1_3_dist * joint1_3_dist) / (2.0f * LINK1_LENGTH * LINK2_LENGTH));
    theta1 = atan2f(y_joint3, x_joint3) + acosf((LINK1_LENGTH * LINK1_LENGTH + joint1_3_dist * joint1_3_dist - LINK2_LENGTH * LINK2_LENGTH) / (2.0f * LINK1_LENGTH * joint1_3_dist));
    theta3 = 2.0f * PI - phi - theta1 - theta2;
    
    // 设置关节角
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
 * @brief 设置机械臂关节角度
 */
static bool Arm_SetJointAngle(Servo_ID_t servo_id, float angle)
{
    bool res = true;
    uint16_t ccr = 0;

    switch(servo_id){
        case SERVO_CHASSIS:
            if(angle < angle_range.chassis[0] || angle > angle_range.chassis[1]){
                angle = (angle < angle_range.chassis[0]) ? angle_range.chassis[0] : angle_range.chassis[1];
                res = false;
            }
            ccr = (uint16_t)(1388 + (angle * 2000.0f / 180.0f));
            break;
        case SERVO_JOINT_1:
            if(angle < angle_range.joint1[0] || angle > angle_range.joint1[1]){
                angle = (angle < angle_range.joint1[0]) ? angle_range.joint1[0] : angle_range.joint1[1];
                res = false;
            }
            ccr = (uint16_t)(2342 - (angle * 1900.0f / 180.0f));
            break;
        case SERVO_JOINT_2:
            if(angle < angle_range.joint2[0] || angle > angle_range.joint2[1]){
                angle = (angle < angle_range.joint2[0]) ? angle_range.joint2[0] : angle_range.joint2[1];
                res = false;
            }
            ccr = (uint16_t)(-388 + (angle * 2000.0f / 180.0f));
            break;
        case SERVO_JOINT_3:
            if(angle < angle_range.joint3[0] || angle > angle_range.joint3[1]){
                angle = (angle < angle_range.joint3[0]) ? angle_range.joint3[0] : angle_range.joint3[1];
                res = false;
            }
            ccr = (uint16_t)(-555 + (angle * 2000.0f / 180.0f));
            break;
        case SERVO_JOINT_GRIPPER:
            if(angle < angle_range.gripper[0] || angle > angle_range.gripper[1]){
                angle = (angle < angle_range.gripper[0]) ? angle_range.gripper[0] : angle_range.gripper[1];
                res = false;
            }
            // TODO: 夹爪舵机的CCR计算公式待确认
            ccr = 1500;
            break;
    }

    Servo_SetCCR(servo_id, ccr);
    return res;
}

/**
 * @brief 获取机械臂关节角度
 */
static float Arm_GetJointAngle(Servo_ID_t servo_id)
{
    float angle = 0.0f;
    uint16_t ccr = 0;

    switch(servo_id){
        case SERVO_CHASSIS:
            ccr = Servo_GetCCR(servo_id);
            angle = (float)(ccr - 1388) * 180.0f / 2000.0f;
            break;
        case SERVO_JOINT_1:
            ccr = Servo_GetCCR(servo_id);
            angle = (2342 - ccr) * 180.0f / 1900.0f;
            break;
        case SERVO_JOINT_2:
            ccr = Servo_GetCCR(servo_id);
            angle = (float)(ccr + 388) * 180.0f / 2000.0f;
            break;
        case SERVO_JOINT_3:
            ccr = Servo_GetCCR(servo_id);
            angle = (float)(ccr + 555) * 180.0f / 2000.0f;
            break;
        case SERVO_JOINT_GRIPPER:
            ccr = Servo_GetCCR(servo_id);
            angle = (float)(ccr - 1000) * 90.0f / 1000.0f;
            break;
    }

    return angle;
}
