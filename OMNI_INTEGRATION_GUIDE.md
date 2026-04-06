# OMNI 三輪等邊三角形小車集成指南

## 📋 目錄
1. [系統概述](#系統概述)
2. [硬件配置](#硬件配置)
3. [運動學分析](#運動學分析)
4. [文件結構](#文件結構)
5. [API 使用](#api-使用)
6. [參數調整](#參數調整)
7. [故障排除](#故障排除)
8. [測試檢查清單](#測試檢查清單)

---

## 系統概述

### OMNI 三輪配置

OMNI 三輪萬向小車採用**等邊三角形**配置，三個全向輪分別位於：

```
        M1 (前左)         M2 (前右)
        120°              240°
           \                 /
            \               /
             \             /
              \           /
               \         /
                \       /
                  M3 (後)
                  0°
```

### 系統特性

✅ **完整 3-DOF 自由度**
- X 軸：前進/後退運動
- Y 軸：左右平移運動
- Z 軸：原地旋轉運動

✅ **獨立控制三個軸**
- 可同時進行複合運動
- 不受運動方向限制（全向）

✅ **實時反饋控制**
- 編碼器速度反饋
- 偏航角自動校準
- PID 閉環控制

---

## 硬件配置

### 電機配置

| 電機 | 位置 | 編號 | 方向 | 備註 |
|------|------|------|------|------|
| M1 | 前左 | MOTOR_ID_M1 | 120° | 三個全向輪 |
| M2 | 前右 | MOTOR_ID_M2 | 240° | |
| M3 | 後面 | MOTOR_ID_M3 | 0° | |
| M4 | 保留 | MOTOR_ID_M4 | - | 不使用 |

### 傳感器配置

| 傳感器 | 用途 | 備註 |
|--------|------|------|
| 編碼器 | 速度反饋 | 三個電機各配一個 |
| IMU | 偏航角 | ICM20948 或 MPU9250 |

### 機械尺寸

```c
// app_omni.h 中配置
#define OMNI_TRIANGLE_SIDE    (200.0f)  // 三角形邊長 (mm)
#define OMNI_WHEEL_RADIUS     (115.47f) // 中心到輪子距離 (mm)
#define OMNI_CIRCLE_MM        (204.203f) // 輪子周長 (mm)
```

---

## 運動學分析

### 座標系定義

```
        前(+X)
         ↑
         |
 左(+Y)←-*--→右(-Y)
         |
         ↓
        後(-X)

旋轉: 逆時針正(+Z)，順時針負(-Z)
```

### 三輪位置

三個輪子在笛卡爾座標系中的位置：

```
M1 (120°): (-cos(30°), sin(30°), 0) = (-0.866, 0.5, 0)
M2 (240°): (-cos(30°), -sin(30°), 0) = (-0.866, -0.5, 0)
M3 (0°):   (1.0, 0, 0)
```

其中 R = L/√3 為中心到輪子的距離。

### 正解：速度指令 → 電機速度

給定期望速度 (Vx, Vy, Vz)，計算各電機速度：

```
V_M1 = -0.5·Vx + √3/2·Vy - R·Vz/1000
V_M2 = -0.5·Vx - √3/2·Vy - R·Vz/1000
V_M3 = Vx - R·Vz/1000
```

**代碼實現** (app_omni.c):
```c
const float COS_120 = -0.5f;
const float SIN_120 = 0.866025f;
const float SIN_240 = -0.866025f;

speed_M1_setup = (int16_t)(COS_120 * speed_x + SIN_120 * speed_y + speed_spin);
speed_M2_setup = (int16_t)(COS_120 * speed_x + SIN_240 * speed_y + speed_spin);
speed_M3_setup = (int16_t)(1.0f * speed_x + 0.0f * speed_y + speed_spin);
```

### 逆解：編碼器反饋 → 實際速度

已知各電機速度，反推實際速度：

```
Vx = V_M3 - (V_M1 + V_M2) / 2
Vy = (V_M1 - V_M2) / √3
Vz = (V_M1 + V_M2 + V_M3) × 1000 / (3·R)
```

**代碼實現** (app_motion.c):
```c
case CAR_OMNI:
{
    car->Vx = (int16_t)(speed_mm[2] - (speed_mm[0] + speed_mm[1]) / 2.0f);
    car->Vy = (int16_t)((speed_mm[0] - speed_mm[1]) / 1.732f);
    car->Vz = (int16_t)((speed_mm[0] + speed_mm[1] + speed_mm[2]) / (3.0f * robot_APB) * 1000);
    break;
}
```

### 驗證公式

**前進測試 (Vx=500, Vy=0, Vz=0)**
```
V_M1 = -0.5×500 + 0.866×0 - 0 = -250 mm/s (逆向)
V_M2 = -0.5×500 - 0.866×0 - 0 = -250 mm/s (逆向)
V_M3 = 500 - 0 = 500 mm/s (正向)
→ 三個輪子合力推動小車前進 ✓
```

**左移測試 (Vx=0, Vy=500, Vz=0)**
```
V_M1 = -0 + 0.866×500 - 0 = 433 mm/s (正向)
V_M2 = -0 - 0.866×500 - 0 = -433 mm/s (逆向)
V_M3 = 0 - 0 = 0 mm/s (停止)
→ M1 和 M2 對稱推動，M3 配合滑動 ✓
```

**旋轉測試 (Vx=0, Vy=0, Vz=5000, R=115.47)**
```
R·Vz/1000 = 115.47 × 5000 / 1000 = 577.35
V_M1 = -0 + 0 - 577.35 = -577.35 mm/s
V_M2 = -0 - 0 - 577.35 = -577.35 mm/s
V_M3 = 0 - 577.35 = -577.35 mm/s
→ 三個輪子同向逆轉，小車原地旋轉 ✓
```

---

## 文件結構

### 新增文件

```
Source/APP/
├── app_omni.h              # 配置頭文件
├── app_omni.c              # 運動學實現
└── (OMNI_INTEGRATION_GUIDE.md)  # 本文檔
```

### 修改文件

```
Source/APP/
├── app_motion.h            # 添加 CAR_OMNI enum
└── app_motion.c            # 集成 OMNI 支持 (8 個位置)
```

### 文件大小

| 文件 | 行數 | 大小 |
|------|------|------|
| app_omni.h | ~90 | 2.4 KB |
| app_omni.c | ~250 | 7.2 KB |
| 總計 | ~340 | 9.6 KB |

---

## API 使用

### 1. 設置小車類型

```c
#include "app_motion.h"

// 設置為 OMNI 三輪小車
Motion_Set_Car_Type(CAR_OMNI);  // 0x07

// 驗證設置
uint8_t car_type = Motion_Get_Car_Type();
if (car_type == CAR_OMNI) {
    printf("OMNI 三輪小車已激活\n");
}
```

### 2. 直接速度控制

```c
#include "app_motion.h"

// 前進 500 mm/s
Motion_Ctrl(500, 0, 0, 1);

// 左移 300 mm/s，同時前進 200 mm/s
Motion_Ctrl(200, 300, 0, 0);

// 旋轉（逆時針 100°/s）
Motion_Ctrl(0, 0, 1000, 0);

// 複合運動：斜向前進 + 旋轉
Motion_Ctrl(400, 300, 500, 1);
```

**參數說明**
```c
void Motion_Ctrl(
    int16_t V_x,        // X軸速度: [-1000, 1000] mm/s
    int16_t V_y,        // Y軸速度: [-1000, 1000] mm/s
    int16_t V_z,        // Z軸角速度: [-5000, 5000] °/s
    uint8_t adjust      // 偏航角調節: 0=關, 1=開
);
```

### 3. 狀態機控制

```c
#include "app_motion.h"

// 8 種運動狀態
typedef enum {
    MOTION_STOP = 0,         // 停止
    MOTION_RUN,              // 前進
    MOTION_BACK,             // 後退
    MOTION_LEFT,             // 左移
    MOTION_RIGHT,            // 右移
    MOTION_SPIN_LEFT,        // 逆時針旋轉
    MOTION_SPIN_RIGHT,       // 順時針旋轉
    MOTION_BRAKE             // 刹車
} motion_state_t;

// 使用狀態機
Motion_Ctrl_State(MOTION_RUN, 100, 1);        // 前進，速度 100
Motion_Ctrl_State(MOTION_LEFT, 50, 0);        // 左移，速度 50
Motion_Ctrl_State(MOTION_SPIN_LEFT, 80, 0);   // 旋轉，速度 80
Motion_Ctrl_State(MOTION_STOP, 0, 0);         // 停止
```

**參數說明**
```c
void Motion_Ctrl_State(
    uint8_t state,       // 運動狀態 (見上述 enum)
    uint16_t speed,      // 速度級別: [0, 100]（自動 ×10）
    uint8_t adjust       // 偏航角調節: 0=關, 1=開
);
```

### 4. 獲取實際速度

```c
#include "app_motion.h"

car_data_t car;

// 在 Motion_Handle() 中調用（每 10ms）
Motion_Get_Speed(&car);

printf("Vx=%d mm/s, Vy=%d mm/s, Vz=%d °/s\n", 
       car.Vx, car.Vy, car.Vz);
```

### 5. 偏航角自動校準

```c
#include "app_motion.h"

// 啟用自動校準（需要 IMU）
Motion_Ctrl(500, 0, 0, 1);  // 第 4 個參數為 1

// 或先設置
Motion_Set_Yaw_Adjust(1);
Motion_Ctrl(500, 0, 0, 0);
```

---

## 參數調整

### 機械參數

編輯 `Source/APP/app_omni.h`：

```c
// 1. 調整三角形邊長（根據實際尺寸）
#define OMNI_TRIANGLE_SIDE           (200.0f)    // 毫米
// 自動計算中心到輪子距離：R = L / √3 ≈ 0.577×L

// 2. 調整輪子周長（根據輪徑）
// 例：輪徑 D = 65mm
// 周長 = π × D = 3.14159 × 65 ≈ 204.203mm
#define OMNI_CIRCLE_MM               (204.203f)   // 毫米

// 3. 最大速度限制
#define CAR_OMNI_MAX_SPEED           (1000)       // mm/s
```

### 電機參數

編輯 `Source/APP/app_motion.c`：

```c
// Motion_Get_Circle_Pulse() 中調整編碼器脈衝數
case CAR_OMNI:
    temp = ENCODER_CIRCLE_330;  // 根據電機規格調整
    break;
```

### PID 參數

編輯 `Source/APP/app_pid.h` 或 `app_pid.c`：

```c
// 調整 PID 增益（根據實際性能）
#define PID_KP   (20.0f)   // 比例增益
#define PID_KI   (5.0f)    // 積分增益
#define PID_KD   (2.0f)    // 微分增益
```

### 常見調整場景

| 問題 | 原因 | 調整方案 |
|------|------|---------|
| 小車不動 | 電機接線錯誤 | 檢查 M1/M2/M3 接線 |
| 方向不對 | 電機極性反 | 交換某個電機的正負極 |
| 前進時偏向 | 三角形尺寸不對 | 調整 OMNI_TRIANGLE_SIDE |
| 速度響應慢 | PID 增益過小 | 增加 PID_KP 值 |
| 速度波動大 | PID 增益過大 | 減小 PID_KP 值 |

---

## 故障排除

### 編譯錯誤

**錯誤：`undefined reference to 'Omni_Ctrl'`**
```
解決方案：
1. 確認 app_omni.c 已添加到編譯列表
2. 檢查 app_omni.h 包含是否正確
3. 確認 Makefile 中有 app_omni.o
```

**錯誤：`CAR_OMNI undeclared`**
```
解決方案：
1. 檢查 app_motion.h 中 CAR_OMNI = 0x07 是否存在
2. 確認 app_motion.h 被正確包含
3. 重新編譯：make clean && make
```

### 運行時問題

**問題：小車不動**
```
診斷步驟：
1. 檢查電機是否通電（測量 PWM 輸出）
2. 驗證編碼器是否有信號
3. 測試單個電機：Motion_Set_Speed(500, 0, 0, 0)
4. 檢查 CAR_OMNI 是否正確設置
```

**問題：方向錯誤**
```
可能原因與解決：
1. 電機接線反向 → 交換正負極
2. 輪子安裝方向錯誤 → 重新安裝
3. 運動學方程系數 → 檢查 app_omni.c 中的常數
```

**問題：速度不穩定**
```
調整步驟：
1. 檢查編碼器連接是否牢固
2. 調整 PID 參數（先調 Kp）
3. 檢查電池電壓是否穩定
4. 測試不同速度級別
```

### 調試工具

```c
// 添加調試輸出（app_omni.h）
#define ENABLE_DEBUG_YAW  1

// 監控編碼器值
void debug_encoder(void) {
    float speed[4];
    Motion_Get_Motor_Speed(speed);
    printf("M1:%d M2:%d M3:%d M4:%d\n", 
           (int)speed[0], (int)speed[1], 
           (int)speed[2], (int)speed[3]);
}

// 監控實際速度
void debug_velocity(void) {
    car_data_t car;
    Motion_Get_Speed(&car);
    printf("Vx:%d Vy:%d Vz:%d\n", car.Vx, car.Vy, car.Vz);
}
```

---

## 測試檢查清單

### 編譯與集成測試

```bash
□ 確認 app_omni.h 和 app_omni.c 已創建
□ 確認 app_motion.h 中添加了 CAR_OMNI enum
□ 確認 app_motion.c 中添加了 8 個 OMNI case
□ 編譯無誤：make clean && make
□ 無链接錯誤
```

### 硬件連接測試

```bash
□ 三個電機已連接至 M1, M2, M3
□ 編碼器已連接（3 個）
□ IMU 已連接（可選）
□ 電源連接正確
□ 所有接線牢固
```

### 軟件功能測試

```bash
□ 能設置 Motion_Set_Car_Type(CAR_OMNI)
□ Motion_Get_Car_Type() 返回 0x07
□ Motion_Ctrl(500, 0, 0, 0) 小車前進
□ Motion_Ctrl(0, 500, 0, 0) 小車左移
□ Motion_Ctrl(0, 0, 1000, 0) 小車旋轉
□ Motion_Ctrl(300, 300, 0, 0) 小車斜向移動
```

### 性能測試

```bash
□ 前進速度準確性 (誤差 <5%)
□ 左移速度準確性 (誤差 <5%)
□ 旋轉角速度準確性 (誤差 <5%)
□ 複合運動協調性良好
□ 偏航角自動校準有效
□ 最大速度不超過設定值
```

### 邊界條件測試

```bash
□ 零速度停止正常
□ 最大速度運行穩定
□ 快速加速無異常
□ 急轉彎無翻覆
□ 長時間運行無異常熱量
□ 低電量下正常工作
```

---

## 相關文檔

- `app_omni.h` - OMNI 配置與接口定義
- `app_omni.c` - OMNI 運動學實現
- `app_motion.h` - 運動控制頭文件
- `app_motion.c` - 運動控制主實現
- `app_pid.c` - PID 控制算法

---

## 快速參考

### 簡單示例

```c
#include "app_motion.h"

int main(void) {
    // 1. 初始化（系統啟動時）
    Motion_Set_Car_Type(CAR_OMNI);
    
    // 2. 前進 500mm/s，帶自動校準
    Motion_Ctrl(500, 0, 0, 1);
    
    // 延迟...
    
    // 3. 停止
    Motion_Stop(STOP_BRAKE);
    
    return 0;
}
```

### 複合運動示例

```c
// 前進同時左移，速度分別為 400 和 300 mm/s
Motion_Ctrl(400, 300, 0, 1);

// 等效於狀態機
Motion_Ctrl_State(MOTION_RUN, 40, 1);    // 前進
Motion_Ctrl_State(MOTION_LEFT, 30, 1);   // 同時左移
```

### 完整循環示例

```c
void robot_demo(void) {
    Motion_Set_Car_Type(CAR_OMNI);
    
    // 方形路徑
    Motion_Ctrl(500, 0, 0, 1);    // 前進
    delay_ms(1000);
    
    Motion_Ctrl(0, 500, 0, 1);    // 左移
    delay_ms(1000);
    
    Motion_Ctrl(-500, 0, 0, 1);   // 後退
    delay_ms(1000);
    
    Motion_Ctrl(0, -500, 0, 1);   // 右移
    delay_ms(1000);
    
    Motion_Stop(STOP_BRAKE);      // 停止
}
```

---

## 更新日誌

| 版本 | 日期 | 變更 |
|------|------|------|
| 1.0 | 2026-04-06 | 初版發佈 |

---

## 支持

如遇問題，請檢查：
1. [故障排除](#故障排除) 章節
2. [測試檢查清單](#測試檢查清單)
3. 編譯輸出是否有警告
4. 硬件連接是否正確

---

**文檔完成日期：2026-04-06**
**OMNI 三輪小車系統 v1.0**
