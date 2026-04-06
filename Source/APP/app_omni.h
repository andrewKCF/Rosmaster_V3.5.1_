#ifndef __APP_OMNI_H__
#define __APP_OMNI_H__

#include "stdint.h"

// ============ OMNI 三輪等邊三角形配置 ============
/*
 * 三個輪子排成等邊三角形（俯視圖）：
 *
 *        M1 (前左 120°)      M2 (前右 240°)
 *                \                     /
 *                  \                  /
 *                   \                /
 *                    \              /
 *                      \          /
 *                       \        /
 *                         M3 (後 0°)
 *
 * 三角形邊長：L (毫米)
 * 中心到輪子距離：R = L / √3
 */

// 三輪等邊三角形邊長，單位：毫米
#define OMNI_TRIANGLE_SIDE           (200.0f)    // 根據實際尺寸調整

// 三輪配置下，中心到輪子的距離 R = L / √3
#define OMNI_WHEEL_RADIUS            (OMNI_TRIANGLE_SIDE / 1.732f)  // ≈ 115.47mm

// 三輪電機間距之和的一半（用於角速度計算）
#define OMNI_APB                     (OMNI_WHEEL_RADIUS)

// 三輪轉一整圈的位移，單位為 mm
#define OMNI_CIRCLE_MM               (204.203f)   // 根據實際輪徑調整

// OMNI 三輪最大速度限制
#define CAR_OMNI_MAX_SPEED           (1000)


// ============ 運動學函數 ============

/**
 * @brief OMNI 三輪等邊三角形運動控制
 * @param V_x  X軸速度 (前正後負：±1000 mm/s)
 * @param V_y  Y軸速度 (左正右負：±1000 mm/s)  
 * @param V_z  角速度 (逆時針正：±5000 °/s)
 * @param adjust 偏航角調節開關 (1=開, 0=關)
 * 
 * 三輪等邊三角形運動學方程：
 * ┌─────────────────────────────────────┐
 * │ V_M1 = -0.5*Vx + √3/2*Vy + R*Vz/1000│  (前左)
 * │ V_M2 = -0.5*Vx - √3/2*Vy + R*Vz/1000│  (前右)
 * │ V_M3 = Vx + R*Vz/1000               │  (後)
 * └─────────────────────────────────────┘
 */
void Omni_Ctrl(int16_t V_x, int16_t V_y, int16_t V_z, uint8_t adjust);

/**
 * @brief OMNI 三輪狀態控制
 * @param state 運動狀態 (前進/後退/左右移動/旋轉)
 * @param speed 速度值 (0~1000)
 * @param adjust 偏航角調節開關
 */
void Omni_State(uint8_t state, uint16_t speed, uint8_t adjust);

/**
 * @brief OMNI 三輪偏航角校準
 * @param yaw 當前偏航角 (度)
 */
void Omni_Yaw_Calc(float yaw);

#endif /* __APP_OMNI_H__ */
