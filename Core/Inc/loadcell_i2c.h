#ifndef __LOADCELL_I2C_H
#define __LOADCELL_I2C_H

#include "main.h"

#define LOADCELL_I2C_ADDR    (0x08 << 1)
#define LOADCELL_DATA_SIZE   8

typedef struct {
    float displacement;      // 엔코더 변위 (mm)
    float force;             // 로드셀 힘 (g 또는 N)
    uint32_t timestamp;      // 마지막 수신 시각
    uint8_t valid;           // 데이터 유효 여부
    uint8_t error_count;     // 연속 에러 카운트
} LoadCell_Data_TypeDef;

HAL_StatusTypeDef LoadCell_Read(I2C_HandleTypeDef *hi2c, LoadCell_Data_TypeDef *data);
void LoadCell_Init(LoadCell_Data_TypeDef *data);

/* ── Phase 5: Ring buffer 평균 API ──
 * 슬레이브(Arduino+HX711) 80Hz 갱신 (12.5ms). fast_tick(~250Hz)에서
 * LoadCell_Update 호출하면 내부 12ms cap으로 ~83Hz 폴링 → 8샘플 ring
 * buffer 누적. Force PID(slow_tick 10Hz)는 LoadCell_GetAverage() 사용.
 *
 * 동기화: 단일 호출자(main loop fast_tick)만 update, force PID도 같은
 * main 컨텍스트에서 read — lock 불필요.
 */

/** @brief 12ms-cap 폴링 + ring buffer push.
 *  @param hi2c    I2C handle (hi2c2)
 *  @param now_ms  HAL_GetTick() 결과
 *  fast_tick 처리에서 매 호출. 12ms 미만 경과 시 즉시 return (캡).
 */
void LoadCell_Update(I2C_HandleTypeDef *hi2c, uint32_t now_ms);

/** @brief 8샘플 ring buffer의 force 평균값 반환 (g).
 *  buffer가 partial(8개 미만)이면 채워진 분량만 평균. 빈 상태면 0.0.
 */
float LoadCell_GetAverage(void);

/** @brief 최신 displacement 값 반환 (mm). 평균 안 함 (변위는 bias 거의 없음). */
float LoadCell_GetLatestDisp(void);

/** @brief 마지막 valid read 후 timeout_ms 경과했는지 검사.
 *  @return 0 = 신선, 1 = stale (Force PID는 stale 시 PWM=0 안전 차단)
 */
uint8_t LoadCell_IsStale(uint32_t now_ms, uint32_t timeout_ms);

#endif