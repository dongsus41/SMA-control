# ISR 슬림화 + 단방향 파이프라인 리팩토링 설계

작성일: 2026-05-07
대상: STM32G474 펌웨어 (`Core/`) + PyQt5 PC 코드 (`SMA_control/`)

## 배경

현재 펌웨어는 두 개의 TIM ISR (TIM4 ~250 Hz, TIM5 ~10 Hz) 안에서 SPI 스캔, I2C 읽기, PID 계산, UART 송신을 모두 직접 수행하고, 온도/힘 제어 분기가 동일 ISR 내 `if (mode == FORCE)`로 교차한다. `main.c`가 1100여 줄이고 `system / pid / force_ctrl` 세 거대 글로벌이 ISR과 메인 루프 양쪽에서 갱신돼 흐름이 추적하기 어렵다.

리팩토링 목표:

- **B (최우선)**: ISR은 `tick++` 1줄만 남기고, 모든 작업을 메인 루프에서 cooperative 하게 처리.
- **D**: 온도/힘 분기를 채널별 `ctrl_mode`로 통일하여 단방향 파이프라인 구성.
- **C (옵션)**: 거대 전역의 점진적 분리 — D 작업 중 자연스럽게 일부 해소.

비범위 — 현재 코드 규모상 참고 코드(v3.3 softrobot)처럼 모든 주변장치를 별 파일로 쪼개거나(우선순위 A) 레거시 FDCAN 명명을 손대는 작업(E)은 이번 범위 밖.

## 결정 요약

| 결정 | 내용 |
|---|---|
| **B-1** | TIM4/TIM5 두 박자 유지 (하드웨어 시정수상 합리적). ISR은 각자 카운터만 증가, 메인 루프가 atomic-swap으로 소비. |
| **D-1** | 채널별 `ctrl_mode[CTRL_CH]` enum (`OFF / MANUAL / TEMP / FORCE`). 글로벌 모드 + 단일 active_channel 패턴 폐기. |
| **L-1** | 로드셀은 fast_tick(250 Hz) 처리에서 12 ms-cap 폴링 (~83 Hz) → 8샘플 ring buffer 누적 → slow_tick(10 Hz) force PID에서 평균 사용. EXTI 하드웨어 트리거는 차후 과제. |
| **상태 분리** | 단일 `channel_t`로 통합하지 않고, 기존 `Ctrl_Param + PID_Param + Buf_FDCAN_Tx` 3분할 패턴을 유지하면서 mode/force 필드만 자연스럽게 흡수. |
| **프로토콜** | CMD 0x01 페이로드 30 B 크기 동일, 의미 재정의 (호환성 깨짐 — PC 동시 업그레이드 필요). CMD 0x02 24 B 그대로. CMD 0x04 폐기. |

## 1. 상위 제어 흐름

```
   TIM4 ISR (~250 Hz)                    TIM5 ISR (~10 Hz)
        |                                     |
        v                                     v
  g_sys.fast_tick++                     g_sys.slow_tick++
        |                                     |
        └──────────── main loop ──────────────┘
                          │
                          v
   ┌─ if (fast_tick > 0) ──────────────────────────┐
   │   atomic_swap(&g_sys.fast_tick);              │
   │   Sensor_Update();    // 6ch thermo SPI       │
   │                       // 로드셀 12ms-cap 폴링 │
   │                       // → ring buffer 누적   │
   │   Safety_Update();    // 채널별 safety 갱신   │
   │   UartComm_SendState();                       │
   └───────────────────────────────────────────────┘
   ┌─ if (slow_tick > 0) ──────────────────────────┐
   │   atomic_swap(&g_sys.slow_tick);              │
   │   Controller_Update(); // ctrl_mode 디스패치  │
   │                        // → cmd_pwm/cmd_fan   │
   │   Actuator_Apply();    // safety 오버라이드   │
   │                        // + CCR/GPIO 쓰기     │
   └───────────────────────────────────────────────┘
   UartComm_Process();   // RX ring buffer 소비
```

핵심 원칙:

- **ISR은 `tick++` 1줄만**. SPI/I2C/UART 호출 모두 메인 루프로 이동.
- **단방향 파이프라인**: Sensor (입력) → Safety (오버레이) → Controller (계산) → Actuator (출력). 역방향 의존성 없음.
- **UART RX**도 ISR에서는 ring buffer push만, frame parsing은 메인.
- atomic-swap은 참고 코드(v3.3) 패턴 — `__disable_irq()` 짧은 critical section, catch-up 없이 1회만 실행, pending > 1 시 overrun 카운터 누적.

## 2. 모듈 구조

신규 파일은 **`controller.c/h` 하나만**. 나머지는 기존 파일 안에서 책임 분리만.

| 파일 | 책임 | 변경 |
|---|---|---|
| `controller.c/h` | **신규**. `g_cmd / g_ctrl` 보유. `Controller_Update()`가 `g_cmd.mode[i]` switch → temp/force PID 호출 → `g_ctrl.cmd_pwm/cmd_fan` produce. `Actuator_Apply()`도 여기. `Sensor_Update / Safety_Update`도 controller.c가 호출 사이트로 보유. | 신규 |
| `temp_control.c/h` | PID 계산 + safety 상태머신 유지. safety 함수만 외부 호출 가능하게 정리 (`Safety_Update`, `Safety_GetOverride`). PID 함수는 stateless에 가깝게 (`Calculate_Ctrl(pid_state*, temp)`). | 슬림화 (force/manual 분기 제거) |
| `force_control.c/h` | force PID 계산만. `force_ctrl` 글로벌의 `mode` / `active_channel`은 `g_cmd.mode[]`로 흡수되어 사라짐. | 슬림화 |
| `loadcell_i2c.c/h` | 12 ms-cap 폴링 + 8샘플 ring buffer + `LoadCell_GetAverage()` 추가. | 기능 추가 |
| `uart_protocol.c/h` | RX 1바이트 → ISR에서 ring buffer push만. frame parser는 메인의 `UartComm_Process()`. **CMD 0x01 페이로드 새 포맷**, CMD 0x04 폐기. | RX 분리 + 프로토콜 변경 |
| `max31855.c/h` | 변경 없음 | — |
| `main.c` | `MX_*_Init`은 그대로. ISR 본체 1줄, `while(1)` 10줄 이내. `system / pid / force_ctrl` 거대 글로벌은 점진적으로 새 구조체로 이전. | 대폭 슬림 |
| `fdcan.c/h` | 그대로 (legacy, 미사용) | — |

신규 파일을 1개로 한정한 근거:

- `Sensor_Update`는 max31855 + loadcell wrapper만 하면 되는데 별도 파일을 만들기엔 책임이 작음 → controller.c의 정적 함수로 충분.
- `safety.c`를 분리하면 깔끔하지만 temp 의존성이 강해서 temp_control.c 안에 두는 게 자연스러움.
- `actuator.c`도 분리할 수 있지만 controller가 produce → 같은 호출 사이트에서 apply하는 응집도가 더 높음.

## 3. 채널 상태 모델

기존 3분할 패턴 유지. `channel_t` 하나로 합치지 않음.

```c
// 명령 (PC가 set, 채널별 입력) — 기존 Ctrl_Param_typedef 확장
typedef enum {
    CH_OFF    = 0,   // PWM=0, fan=off
    CH_MANUAL = 1,   // user-set PWM 직접 적용
    CH_TEMP   = 2,   // 온도 PID
    CH_FORCE  = 3,   // 힘 PID (현재 단일 로드셀 — 동시 활성 1채널)
} ch_ctrl_e;

typedef struct {
    ch_ctrl_e mode[CTRL_CH];           // 신규
    uint8_t   manual_pwm[CTRL_CH];     // 기존 pwm[]
    uint8_t   manual_fan[CTRL_CH];     // 기존 fan[]
    uint16_t  target[CTRL_CH];         // 기존 target_temp[] 일반화
                                       //  TEMP:  0.25°C/LSB
                                       //  FORCE: 0.1g/LSB (mode로 해석 결정)
} cmd_param_t;

// PID 파라미터 + 상태 (controller 내부) — 기존 PID_Manager_typedef 슬림화
typedef struct {
    PID_Param_TypeDef   temp_params[CTRL_CH];
    Force_PID_TypeDef   force_params[CTRL_CH];
    uint8_t             safety_mode[CTRL_CH];   // 0/1/2/3
    uint8_t             cmd_pwm[CTRL_CH];       // controller → actuator
    uint8_t             cmd_fan[CTRL_CH];
} ctrl_state_t;

// 송신 (MCU → PC) — 기존 Buf_FDCAN_Tx_typedef 그대로
typedef struct {
    uint8_t  pwm[CTRL_CH];
    uint8_t  fan[CTRL_CH];     // safety 코드 매립 (0/1/17/49)
    uint16_t temp[CTRL_CH];
} state_buf_t;

extern cmd_param_t   g_cmd;
extern ctrl_state_t  g_ctrl;
extern state_buf_t   g_state;
```

**Force 동시 1채널 제약**: 현재 하드웨어는 로드셀 1개. `g_cmd.mode[i] == CH_FORCE`인 채널이 2개 이상이면 모두 동일한 `LoadCell_GetAverage()` 값을 보고 각자 PWM을 결정하게 되어 의미 없음. 제약 enforce 위치는 **UART CMD 0x01 수신 핸들러** — `mode[]` 디코딩 시 `CH_FORCE` 카운트가 2 이상이면 프레임 거부 (또는 첫 번째만 수용). 채널별 슬롯 (`force_params[CTRL_CH]`)은 향후 다채널 로드셀 확장 대비.

데이터 흐름:

```
  PC (UART) ─set──► g_cmd.{mode, target, manual_pwm, manual_fan}
  Sensor    ─set──► sensor.temp[6], loadcell.force_avg (모듈 내부)
  Safety    ─set──► g_ctrl.safety_mode[]
  Controller─read── 위 + g_cmd  ─produce──► g_ctrl.{cmd_pwm, cmd_fan}
  Actuator  ─read── g_ctrl + safety_mode ──write──► CCRx, FSWn GPIO
            ─set──► g_state (PC 송신용)
```

기존 거대 글로벌 처리:

- **`system` (System_typedef)** → 부분 해체. UART buffer는 `uart_protocol.c` 내부 static. `ctrl_param_*`는 `g_cmd`로. `pnt_pwm[]` 매핑은 controller.c (또는 actuator 섹션) 내부 static.
- **`pid`** → `params[]` / `enable_pid[]` → `g_ctrl` 또는 `g_cmd.mode[]`로. `shared_data.temp_data[]`는 sensor 모듈 내부.
- **`force_ctrl`** → `loadcell` 데이터는 loadcell_i2c.c 내부, force PID 상태는 `g_ctrl.force_params[]`, `mode` / `active_channel`은 `g_cmd.mode[]`로 자연스럽게 흡수.

## 4. UART 프로토콜 변경

### CMD 0x01 (PC → MCU, Control) — 30 B 동일, 의미 변경

| Offset | Size | Field | Description |
|---|---|---|---|
| 0  |  6 | `mode[6]` | `ch_ctrl_e` (0=OFF, 1=MANUAL, 2=TEMP, 3=FORCE) |
| 6  |  6 | `manual_pwm[6]` | CH_MANUAL일 때만 의미 |
| 12 |  6 | `manual_fan[6]` | CH_MANUAL일 때만 의미 |
| 18 | 12 | `target[6 × u16]` | TEMP: 0.25 °C/LSB / FORCE: 0.1 g/LSB |

**기존**: `pwm[6] + fan[6] + enable_pid[6] + target_temp[6×u16]`
- `enable_pid[i]==1 → TEMP, 0 → MANUAL` 암묵적 관례 → 명시 enum으로 대체.

### CMD 0x02 (MCU → PC, State) — 24 B 그대로

`pwm[6] + fan[6] + temp[6×u16]`. `fan[i]`에 safety 코드(0/1/17/49) 매립 관례 유지. PC 파서 변경 최소.

### CMD 0x03 (PC → MCU, Gain Update) — 13 B 그대로

기존 동일. force PID 게인은 컴파일 타임 상수 또는 향후 별도 CMD로 확장 가능 (이번 범위 밖).

### CMD 0x04 (Force Control) — 폐기

D-1 통합으로 `mode[ch]=CH_FORCE + target[ch]`가 동일 효과. 마이그레이션 단계 4에서 동시 폐기.

### CMD 0x05 (Force State) — 그대로

Force 활성 채널 있을 때 fast_tick에서 송신.

### PC측 변경 (`SMA_control/`)

- `uart_driver.py` frame builder: CMD 0x01 페이로드 레이아웃 변경, CMD 0x04 송신 로직 제거.
- `main.py` GUI: 기존 `MODE_MANUAL/TEMP/FORCE` 시스템 모드 라디오 → 채널별 mode 선택 UI (선택 채널 1개 + mode 콤보박스 1쌍 정도).
- `ctrl.py`, `diag.py`, `Force test.py`도 동일 프로토콜 사용 — 동시 업데이트.

## 5. 안전 / 에러 처리

### Safety overlay 패턴

기존 4단계 히스테리시스 그대로 유지 (0/1/2/3). 핵심 변경은 **safety 적용 위치**:

```c
// safety: temp_control.c 안에서 g_ctrl.safety_mode[ch]만 갱신
void Safety_Update(uint8_t ch, float temp) {
    // 기존 Check_Temperature_Sensor + Check_Safety_Temperature 로직
    // 결과: g_ctrl.safety_mode[ch] = 0/1/2/3
}

// controller는 safety를 모름 — 자기 모드대로만 cmd_pwm/cmd_fan 채움
void Controller_Update(void) {
    for (uint8_t i = 0; i < CTRL_CH; i++) {
        switch (g_cmd.mode[i]) {
            case CH_OFF:
                g_ctrl.cmd_pwm[i] = 0;
                g_ctrl.cmd_fan[i] = 0;
                break;
            case CH_MANUAL:
                g_ctrl.cmd_pwm[i] = g_cmd.manual_pwm[i];
                g_ctrl.cmd_fan[i] = g_cmd.manual_fan[i];
                break;
            case CH_TEMP:
                /* Calculate_Ctrl + Control_Fan_By_Temperature */
                break;
            case CH_FORCE:
                /* mode[]에 CH_FORCE 1채널만 있다는 것이 UART 수신 단계에서 보장됨.
                 * ForceControl_Calculate(LoadCell_GetAverage(), ...) → cmd_pwm[i] */
                break;
        }
    }
}

// safety는 actuator overlay로 일괄 적용 — 한 곳에서만 강제
void Actuator_Apply(void) {
    for (uint8_t i = 0; i < CTRL_CH; i++) {
        uint8_t pwm = g_ctrl.cmd_pwm[i];
        uint8_t fan = g_ctrl.cmd_fan[i];
        switch (g_ctrl.safety_mode[i]) {
            case 1: fan = 1; break;                   // warn: 팬 강제
            case 2:
            case 3: pwm = 0; fan = 1; break;          // critical / sensor_err: PWM 차단
        }
        *PWM_CCR[i] = pwm;
        if (fan) FSW_on(i); else FSW_off(i);
    }
}
```

장점:

- **controller는 safety 무지** — 자기 모드 계산만. 안전 분기가 controller 안에 흩어지지 않음.
- **safety overlay 한 곳** — "왜 PWM이 0됐지?"의 원인이 항상 `Actuator_Apply` 한 함수.
- safety 임계 검사는 fast_tick (~250 Hz)에서 즉시 갱신, controller는 slow_tick (~10 Hz)에서 결과를 *읽기만* → safety 반응이 PID 주기에 갇히지 않음.

사용자 시나리오 검증 — "force 제어가 PWM을 올리라는데 과열 상태"라면: controller는 force PID 결과로 `cmd_pwm[i] = 80` 같은 값을 넣지만, `Actuator_Apply`가 `safety_mode[i] >= 2`를 보고 `pwm = 0`으로 강제 덮어씀.

### 기타 에러 케이스

- **I2C 타임아웃 (loadcell)**: `LoadCell_Read()` 실패 시 ring buffer push 안 함 + stale flag set. Force PID는 stale 검출 시 `cmd_pwm[i] = 0`.
- **SPI fault (max31855)**: 기존처럼 `Safety_Update`에서 sensor_error → `safety_mode = 3` → actuator가 차단.
- **UART CRC fail**: 프레임 폐기, 다음 STX 대기. 기존 동일.
- **TIM tick overrun**: `g_sys.{fast,slow}_overrun_count` 누적. catch-up 없이 1회만 실행 (PID dt 왜곡 / I2C burst 회피).

## 6. 마이그레이션 단계

각 단계 끝나면 동작 검증 가능. 4단계만 펌웨어/PC 동시 업그레이드 필요.

| Phase | 내용 | 의미 변경 | 검증 |
|---|---|---|---|
| **1. ISR 슬림화** | TIM4/TIM5 ISR을 `tick++`만 남기고, 기존 본체를 main.c의 `do_fast_tick()` / `do_slow_tick()`으로 이동. while(1)에서 atomic-swap 후 호출. | ❌ 동작 동일 | 기존 GUI에서 데이터/응답 동일 |
| **2. UART RX 분리** | `HAL_UART_RxCpltCallback` → ring buffer push만. frame parsing은 메인 `UartComm_Process()`. | ❌ | 명령 송수신 동일 |
| **3. 모듈 추출** | `do_fast_tick` / `do_slow_tick` 본체를 `controller.c`의 `Sensor_Update / Safety_Update / Controller_Update / Actuator_Apply`로 이름 분리. 기존 글로벌 그대로. | ❌ | 동일 |
| **4. 채널 모드 일반화 (D-1)** | 새 `g_cmd / g_ctrl / g_state` 도입. `system.ctrl_param_now.*` 사용처 교체. CMD 0x01 새 페이로드, CMD 0x04 폐기. PC측 (`uart_driver.py`, `main.py`, `ctrl.py`, `diag.py`, `Force test.py`) 동시 변경. `force_ctrl.mode` / `active_channel` 제거. | ✅ **호환성 깨짐** | OFF / MANUAL / TEMP / FORCE 4모드 PC에서 모두 set 가능 |
| **5. loadcell ring buffer** | `loadcell_i2c.c`에 12 ms-cap + 8샘플 평균 추가. `Sensor_Update`에서 `LoadCell_Update(HAL_GetTick())` 호출, force 분기에서 `LoadCell_GetAverage()` 사용. | (force 분해능 ↑) | force 응답 노이즈 감소 |
| **6. 거대 글로벌 마무리 (옵션)** | `system_typedef` 잔여 멤버를 모듈로 분산. 사용자 우선순위 C. | ❌ | 동일 |

## 비결정 / 향후 과제

- **EXTI 트리거 기반 로드셀 read** (L-2): 하드웨어상 PC4(`I2C2_INT_Pin`)가 EXTI Falling으로 init 돼 있지만 IRQ handler 미구현. 로드셀 측이 INT 라인을 실제로 토글하는지 검증 필요. 이번 범위 밖.
- **Force PID 게인 원격 업데이트**: CMD 0x03이 temp PID용. 필요 시 CMD 0x07 등 신규 또는 0x03 페이로드에 mode 1바이트 추가.
- **CMD 0x02에 mode/safety 별도 바이트 추가** (24→25 B): 현재는 fan 코드에 매립 유지. 차후 PC 디스플레이 명확성 필요 시 검토.
- **레거시 FDCAN 코드 정리** (사용자 우선순위 E): 사용 안 하지만 컴파일은 됨. 차후.
