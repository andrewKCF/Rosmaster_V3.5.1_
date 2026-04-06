#include "app_omni.h"
#include "app_motion.h"
#include "app_pid.h"
#include "app_bat.h"
#include "app.h"
#include "bsp_usart.h"
#include "bsp_motor.h"
#include "bsp_common.h"

#include <math.h>

static float speed_x = 0;
static float speed_y = 0;
static float speed_spin = 0;

static int speed_M1_setup = 0;  // 前左 120°
static int speed_M2_setup = 0;  // 前右 240°
static int speed_M3_setup = 0;  // 後 0°

static int g_offset_yaw = 0;
static uint16_t g_speed_setup = 0;


/**
 * ============ 三輪等邊三角形運動學分析 ============
 * 
 * 三個輪子排成等邊三角形配置：
 *
 *        M1 (前左)         M2 (前右)
 *        120°              240°
 *                \                     /
 *                  \                  /
 *                   \                /
 *                    \              /
 *                      \          /
 *                       \        /
 *                         M3 (後)
 *                         0°
 *
 * 三角形邊長：L
 * 中心到輪子距離：R = L / √3 ≈ 0.577*L
 * 
 * ===== 運動學正解 (Vx, Vy, Vz → 三個電機速度) =====
 * 
 * 假設三個輪子方向為：
 * - M1: 120° 方向 (cos(120°)=-0.5, sin(120°)=√3/2≈0.866)
 * - M2: 240° 方向 (cos(240°)=-0.5, sin(240°)=-√3/2≈-0.866)
 * - M3: 0°   方向 (cos(0°)=1, sin(0°)=0)
 *
 * 基本方程：v_i = Vx*cos(θ_i) + Vy*sin(θ_i) + R*Vz/1000
 *
 * 代入各輪子方向：
 * ┌────────────────────────────────────────┐
 * │ V_M1 = -0.5*Vx + √3/2*Vy + R*Vz/1000   │ (前左 120°)
 * │ V_M2 = -0.5*Vx - √3/2*Vy + R*Vz/1000   │ (前右 240°)
 * │ V_M3 = 1.0*Vx + 0*Vy + R*Vz/1000       │ (後 0°)
 * └────────────────────────────────────────┘
 *
 * 簡化：
 * ┌────────────────────────────────────────┐
 * │ V_M1 = -0.5*Vx + 0.866*Vy + k*Vz       │
 * │ V_M2 = -0.5*Vx - 0.866*Vy + k*Vz       │
 * │ V_M3 = Vx + k*Vz                       │
 * └────────────────────────────────────────┘
 * 其中 k = R/1000 (單位轉換係數)
 *
 * ===== 運動學逆解 (三個電機速度 → Vx, Vy, Vz) =====
 *
 * ┌────────────────────────────────────────┐
 * │ Vx = V_M3 - (V_M1 + V_M2)/2             │
 * │ Vy = (V_M1 - V_M2) / √3                │
 * │ Vz = (V_M1 + V_M2 + V_M3) / (3*k) * 1000│
 * └────────────────────────────────────────┘
 */

// V_x: X軸速度(前正後負：±1000)
// V_y: Y軸速度(左正右負：±1000)
// V_z: 旋轉速度(逆時針正：±5000)
void Omni_Ctrl(int16_t V_x, int16_t V_y, int16_t V_z, uint8_t adjust)
{
    float robot_radius = OMNI_WHEEL_RADIUS;
    float k = robot_radius / 1000.0f;  // 單位轉換係數
    
    // 常數定義 (避免重複計算)
    const float COS_120 = -0.5f;        // cos(120°)
    const float SIN_120 = 0.866025f;    // sin(120°) ≈ √3/2
    const float SIN_240 = -0.866025f;   // sin(240°) ≈ -√3/2
    
    speed_x = V_x;
    speed_y = V_y;
    speed_spin = V_z / 1000.0f * robot_radius;
    
    if (V_x == 0 && V_y == 0 && V_z == 0)
    {
        Motion_Stop(STOP_BRAKE);
        return;
    }

    // 應用三輪等邊三角形運動學方程
    // M1: 前左 120°
    speed_M1_setup = (int16_t)(COS_120 * speed_x + SIN_120 * speed_y + speed_spin);
    
    // M2: 前右 240°
    speed_M2_setup = (int16_t)(COS_120 * speed_x + SIN_240 * speed_y + speed_spin);
    
    // M3: 後 0°
    speed_M3_setup = (int16_t)(1.0f * speed_x + 0.0f * speed_y + speed_spin);

    // 速度限制
    if (speed_M1_setup > CAR_OMNI_MAX_SPEED) speed_M1_setup = CAR_OMNI_MAX_SPEED;
    if (speed_M1_setup < -CAR_OMNI_MAX_SPEED) speed_M1_setup = -CAR_OMNI_MAX_SPEED;
    
    if (speed_M2_setup > CAR_OMNI_MAX_SPEED) speed_M2_setup = CAR_OMNI_MAX_SPEED;
    if (speed_M2_setup < -CAR_OMNI_MAX_SPEED) speed_M2_setup = -CAR_OMNI_MAX_SPEED;
    
    if (speed_M3_setup > CAR_OMNI_MAX_SPEED) speed_M3_setup = CAR_OMNI_MAX_SPEED;
    if (speed_M3_setup < -CAR_OMNI_MAX_SPEED) speed_M3_setup = -CAR_OMNI_MAX_SPEED;

    // 設置速度（M1→M1, M2→M2, M3→M3, M4→0 因為只有三個輪子）
    Motion_Set_Speed(speed_M1_setup, speed_M2_setup, speed_M3_setup, 0);
}


// 通過偏航角計算當前的偏差值，校準小車運動方向
void Omni_Yaw_Calc(float yaw)
{
    float yaw_offset = PID_Yaw_Calc(yaw);
    g_offset_yaw = (int16_t)(yaw_offset * g_speed_setup);

    #if ENABLE_DEBUG_YAW
    static int debug_counter = 0;
    debug_counter++;
    if (debug_counter > 5)
    {
        debug_counter = 0;
        printf("OMNI Yaw Calc:%.5f, %d\n", yaw_offset, g_offset_yaw);
    }
    #endif

    int speed_M1 = speed_M1_setup - g_offset_yaw;
    int speed_M2 = speed_M2_setup - g_offset_yaw;
    int speed_M3 = speed_M3_setup + g_offset_yaw;

    // 速度限制
    if (speed_M1 > CAR_OMNI_MAX_SPEED) speed_M1 = CAR_OMNI_MAX_SPEED;
    if (speed_M1 < -CAR_OMNI_MAX_SPEED) speed_M1 = -CAR_OMNI_MAX_SPEED;
    
    if (speed_M2 > CAR_OMNI_MAX_SPEED) speed_M2 = CAR_OMNI_MAX_SPEED;
    if (speed_M2 < -CAR_OMNI_MAX_SPEED) speed_M2 = -CAR_OMNI_MAX_SPEED;
    
    if (speed_M3 > CAR_OMNI_MAX_SPEED) speed_M3 = CAR_OMNI_MAX_SPEED;
    if (speed_M3 < -CAR_OMNI_MAX_SPEED) speed_M3 = -CAR_OMNI_MAX_SPEED;

    Motion_Set_Speed(speed_M1, speed_M2, speed_M3, 0);
}


// 控制 OMNI 三輪小車運動狀態
// 速度控制：speed=0~1000
// 偏航角調節運動：adjust=1開啟，=0不開啟
void Omni_State(uint8_t state, uint16_t speed, uint8_t adjust)
{
    g_speed_setup = speed;
    switch (state)
    {
    case MOTION_STOP:
        g_speed_setup = 0;
        Motion_Stop(speed == 0 ? STOP_FREE : STOP_BRAKE);
        break;
        
    case MOTION_RUN:  // 前進（沿 X 軸正方向）
        Motion_Set_Yaw_Adjust(adjust);
        Omni_Ctrl(speed, 0, 0, adjust);
        break;
        
    case MOTION_BACK:  // 後退（沿 X 軸負方向）
        Motion_Set_Yaw_Adjust(adjust);
        Omni_Ctrl(-speed, 0, 0, adjust);
        break;
        
    case MOTION_LEFT:  // 左移（沿 Y 軸正方向）
        Motion_Set_Yaw_Adjust(0);
        Omni_Ctrl(0, speed, 0, 0);
        break;
        
    case MOTION_RIGHT:  // 右移（沿 Y 軸負方向）
        Motion_Set_Yaw_Adjust(0);
        Omni_Ctrl(0, -speed, 0, 0);
        break;
        
    case MOTION_SPIN_LEFT:  // 逆時針旋轉
        Motion_Set_Yaw_Adjust(0);
        Omni_Ctrl(0, 0, speed * 5, 0);
        break;
        
    case MOTION_SPIN_RIGHT:  // 順時針旋轉
        Motion_Set_Yaw_Adjust(0);
        Omni_Ctrl(0, 0, -speed * 5, 0);
        break;
        
    case MOTION_BRAKE:  // 刹車
        g_speed_setup = 0;
        Motion_Stop(STOP_BRAKE);
        break;
        
    default:
        g_speed_setup = 0;
        break;
    }
}
