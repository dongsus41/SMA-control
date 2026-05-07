# Phase 2: UART RX 분리 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** UART RX 처리를 ISR과 main loop로 분리한다. ISR은 byte 1개를 ring buffer에 push만 하고, frame state machine + handler 디스패치는 main loop의 `UartComm_Process()`가 담당. 의미 변경 없음 — 송수신 동작이 외부에서 동일해야 한다.

**Architecture:** SPSC (single-producer, single-consumer) lock-free ring buffer 256 byte. `head`(ISR producer) / `tail`(main consumer) 각각 `volatile uint8_t` — 자연 wrap-around로 mod 연산 불필요. `HAL_UART_RxCpltCallback`은 push + 다음 IT receive 시작 2줄만 수행, 모든 frame 처리(`Manual_Control`, `LoadCell_Read`, `printf` 등)는 main 컨텍스트로 이동.

**Tech Stack:** STM32G474 HAL UART (`huart3` @ 115200), Cortex-M4 single-cycle byte store. 펌웨어 단위 테스트 인프라 없음 — 변경 검증은 on-target manual verification (PC GUI / `diag.py` 송수신).

**참조 문서:** `docs/superpowers/specs/2026-05-07-isr-pipeline-refactor-design.md` 섹션 6 (Phase 2)

---

## File Structure

| 파일 | 변경 내용 |
|---|---|
| `Core/Inc/uart_protocol.h` | `UartComm_Process()` declaration 추가, `UartComm_ProcessRxByte()` declaration 제거 (내부 함수화) |
| `Core/Src/uart_protocol.c` | Ring buffer 자료구조 + push/pop static helper, `UartComm_Process()` 함수, `UartComm_ProcessRxByte` static 화, `HAL_UART_RxCpltCallback` push만 하도록 수정 |
| `Core/Src/main.c` | `while(1)` 루프 끝에 `UartComm_Process()` 호출 추가 |

세 파일 변경이 모두 한 commit에 묶여야 동작 — Step 2.2(ISR 변경)와 Step 2.4(main loop 호출 추가) 사이에 빌드는 가능해도 RX는 작동 안 함. 따라서 **단일 Task / 단일 commit**으로 진행하고 step만 세분화.

---

## Task 1: UART RX ring buffer 도입 + ISR 분리

ring buffer는 `uart_protocol.c` 내부 static 변수. 외부 노출 불필요. ISR은 push만, main이 pop + state machine.

**Files:**
- Modify: `Core/Inc/uart_protocol.h:56-60` (API 영역)
- Modify: `Core/Src/uart_protocol.c:9-11, 265-342` (private vars / state machine / RX callback)
- Modify: `Core/Src/main.c:383-419` (while(1) 루프)

- [ ] **Step 1.1: 헤더에 `UartComm_Process()` declaration 추가, `UartComm_ProcessRxByte` 제거**

`Core/Inc/uart_protocol.h` 의 API 영역(현재):

```c
void    UartComm_Init(UART_HandleTypeDef *huart);
void    UartComm_ProcessRxByte(uint8_t byte);
void    UartComm_SendState(void);
void    UartComm_SendForceState(void);
uint8_t UartComm_CalcCRC8(const uint8_t *data, uint16_t len);
```

다음으로 변경:

```c
void    UartComm_Init(UART_HandleTypeDef *huart);
void    UartComm_Process(void);          /* main loop에서 주기 호출 — RX byte 소비 + frame 처리 */
void    UartComm_SendState(void);
void    UartComm_SendForceState(void);
uint8_t UartComm_CalcCRC8(const uint8_t *data, uint16_t len);
```

`UartComm_ProcessRxByte`는 이제 `uart_protocol.c` 내부 static 함수 — 외부 노출 불필요.

- [ ] **Step 1.2: `uart_protocol.c`에 ring buffer 자료구조 + helper 추가**

`Core/Src/uart_protocol.c` 의 Private Variables 섹션 (현재 9~11줄):

```c
/*******************************************************************************
 * Private Variables
 ******************************************************************************/
static UART_HandleTypeDef *g_huart = NULL;
static UartRxContext g_rx_ctx;
```

다음으로 교체:

```c
/*******************************************************************************
 * Private Variables
 ******************************************************************************/
static UART_HandleTypeDef *g_huart = NULL;
static UartRxContext g_rx_ctx;

/* ── RX Ring Buffer (SPSC lock-free) ──
 * Producer : HAL_UART_RxCpltCallback (ISR)
 * Consumer : UartComm_Process() (main loop)
 *
 * 동기화 모델:
 *   head : ISR(push)에서만 갱신, main loop(pop)에서 읽음
 *   tail : main loop(pop)에서만 갱신, ISR(push)에서 읽음
 *   둘 다 1바이트 atomic write이므로 lock/critical section 불필요.
 *
 * 크기 256 = uint8_t 자연 wrap-around → mod 연산 불필요.
 * 1 entry는 full/empty 구분 위해 사용 못하므로 효과적 용량 255 byte.
 * UART_MAX_FRAME_SIZE(254 byte)보다 약간 큼 — main loop가 한 frame 분량
 * 시간(115200 baud에서 ~22ms) 안에 한 번 돌면 overflow 안 남.
 *
 * dropped_count : FIFO full 시 폐기된 byte 누적. 디버거로 main loop 지연 진단.
 */
#define UART_RX_FIFO_SIZE   256U

typedef struct {
    uint8_t          buf[UART_RX_FIFO_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
    volatile uint16_t dropped_count;
} UartRxFifo_t;

static UartRxFifo_t g_rx_fifo;

/* ISR-callable: byte 1개 push. FIFO full 시 byte 폐기 + counter 증가. */
static inline void uart_rx_fifo_push(uint8_t b)
{
    uint8_t next = (uint8_t)(g_rx_fifo.head + 1U);
    if (next == g_rx_fifo.tail) {
        g_rx_fifo.dropped_count++;
        return;
    }
    g_rx_fifo.buf[g_rx_fifo.head] = b;
    g_rx_fifo.head = next;
}

/* main-callable: byte 1개 pop. FIFO empty 시 0 반환, 데이터 있으면 1 반환. */
static inline uint8_t uart_rx_fifo_pop(uint8_t *out)
{
    if (g_rx_fifo.head == g_rx_fifo.tail) {
        return 0;
    }
    *out = g_rx_fifo.buf[g_rx_fifo.tail];
    g_rx_fifo.tail = (uint8_t)(g_rx_fifo.tail + 1U);
    return 1;
}
```

- [ ] **Step 1.3: `UartComm_ProcessRxByte`를 static으로 변경**

`Core/Src/uart_protocol.c` 의 `UartComm_ProcessRxByte` 함수 시그니처 (현재 265줄):

```c
void UartComm_ProcessRxByte(uint8_t byte)
```

다음으로 변경:

```c
static void UartComm_ProcessRxByte(uint8_t byte)
```

(함수 본체는 그대로 — state machine 로직 변경 없음.)

- [ ] **Step 1.4: `UartComm_Process()` 함수 추가**

`Core/Src/uart_protocol.c` 의 `UartComm_Init` 함수 **앞**(`UartComm_ProcessRxByte` 함수 바로 다음, 현재 318줄 닫는 중괄호 다음 빈 줄 직후)에 추가:

```c
/*******************************************************************************
 * Process all pending RX bytes (main loop)
 *   FIFO에서 byte를 모두 꺼내 state machine에 공급.
 *   Frame 완성 시 UartComm_HandleFrame이 호출되어 PID/Manual/Force 명령 처리.
 *   Frame 처리에 시간이 걸려도 ISR은 영향 없음 (ring buffer가 흡수).
 ******************************************************************************/
void UartComm_Process(void)
{
    uint8_t byte;
    while (uart_rx_fifo_pop(&byte))
    {
        UartComm_ProcessRxByte(byte);
    }
}
```

- [ ] **Step 1.5: `HAL_UART_RxCpltCallback`을 push만 하도록 변경**

`Core/Src/uart_protocol.c` 의 `HAL_UART_RxCpltCallback` 함수 (현재 335~342줄):

```c
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == g_huart->Instance)
    {
        UartComm_ProcessRxByte(g_rx_ctx.rx_byte);
        HAL_UART_Receive_IT(g_huart, &g_rx_ctx.rx_byte, 1);
    }
}
```

다음으로 변경:

```c
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == g_huart->Instance)
    {
        /* ISR 슬림화 (Phase 2): byte를 ring buffer에 push만,
         * frame state machine은 main loop의 UartComm_Process()가 처리. */
        uart_rx_fifo_push(g_rx_ctx.rx_byte);
        HAL_UART_Receive_IT(g_huart, &g_rx_ctx.rx_byte, 1);
    }
}
```

- [ ] **Step 1.6: `main.c`의 `while(1)`에 `UartComm_Process()` 호출 추가**

`Core/Src/main.c` 의 while(1) 끝(현재 slow_tick 처리 블록 닫는 중괄호 다음, while 닫는 중괄호 바로 앞):

```c
		/* ── slow_tick 처리 (~10Hz) ── */
		if (g_sys.slow_tick > 0U)
		{
			uint32_t pending;

			__disable_irq();
			pending          = g_sys.slow_tick;
			g_sys.slow_tick  = 0U;
			__enable_irq();

			if (pending > 1U)
			{
				g_sys.slow_overrun_count += (pending - 1U);
				if (pending > g_sys.slow_overrun_max)
				{
					g_sys.slow_overrun_max = pending;
				}
			}

			do_slow_tick();
		}
	}
  /* USER CODE END 3 */
```

다음으로 변경 (마지막 `}` 바로 앞에 한 블록 추가):

```c
		/* ── slow_tick 처리 (~10Hz) ── */
		if (g_sys.slow_tick > 0U)
		{
			uint32_t pending;

			__disable_irq();
			pending          = g_sys.slow_tick;
			g_sys.slow_tick  = 0U;
			__enable_irq();

			if (pending > 1U)
			{
				g_sys.slow_overrun_count += (pending - 1U);
				if (pending > g_sys.slow_overrun_max)
				{
					g_sys.slow_overrun_max = pending;
				}
			}

			do_slow_tick();
		}

		/* ── UART RX 처리 ──
		 * ISR이 ring buffer에 누적한 byte를 소비.
		 * Frame 완성 시 PID/Manual/Force 명령 처리 (printf, I2C 포함).
		 */
		UartComm_Process();
	}
  /* USER CODE END 3 */
```

- [ ] **Step 1.7: 빌드 확인 (사용자 수행 — 다른 로컬에서 STM32CubeIDE)**

빌드 시 확인 사항:
- 0 errors
- `UartComm_ProcessRxByte`가 static으로 변경되었지만 같은 파일 안에서만 호출되므로 외부 참조 에러 없어야 함
- `uart_rx_fifo_push` / `uart_rx_fifo_pop` 인라인 함수 — 같은 파일 안에서만 호출

문제 발생 시:
- `UartComm_ProcessRxByte` 외부 참조 에러 → 다른 파일에서 호출하는 코드가 있는지 grep. 현재 코드베이스에서 `UartComm_ProcessRxByte`는 `uart_protocol.c`와 헤더 declaration에서만 사용됨.

- [ ] **Step 1.8: 플래시 + 검증 (사용자 수행)**

보드에 플래시 후 PC GUI(`python main.py`) 또는 `diag.py` 로 다음 시나리오 검증:

1. **수동 PWM 명령 (CMD 0x01)** — 채널 0에 PWM 50% 명령 → GUI에 즉시 반영, SMA 가열. 변경 전 동일.
2. **PID 명령 (CMD 0x01 with enable_pid)** — 채널 0 PID 활성, 목표 50 °C → 제어 응답 동일.
3. **Gain Update (CMD 0x03)** — `diag.py` 또는 GUI에서 PID gain 변경 명령 → MCU `printf` 디버그 출력 "PID Gains updated for CH 0..." 메시지 확인.
4. **Force Control (CMD 0x04)** — `diag.py` 로 force enable → MCU가 force 모드 진입, CMD 0x05 송신 시작.
5. **I2C Test (CMD 0x06)** — `diag.py` 로 I2C 테스트 명령 → MCU `printf` "I2C OK: disp=... force=..." 메시지 확인. (이 명령은 frame 처리에서 I2C blocking read 발생 — 이전엔 ISR에서 발생했지만 이제 main에서 발생, 동작은 동일)
6. **고속 명령 부하 테스트** — PC에서 100ms 주기로 CMD 0x01 송신 (`main.py`의 `tx_timer.setInterval(100)` 그대로) → 모든 명령이 누락 없이 처리되는지 GUI 응답 확인.
7. **`g_rx_fifo.dropped_count` 확인** — 디버거로 0 유지. 누적되면 main loop 한 주기 > frame 도착 주기.

문제 발견 시 대처:
- 명령이 무시됨 → `UartComm_Process()` 호출이 while(1)에 추가됐는지 main.c 확인.
- RX state machine이 frame 못 맞춤 → `g_rx_ctx`는 같은 파일 static 변수라 `UartComm_ProcessRxByte` 호출 컨텍스트 변경에 영향 없어야 함. ring buffer head/tail 동기화 검토.
- 디버그 메시지 출력이 깨짐 → `printf`가 `Manual_Control`/`HandleI2CTest` 안에서 호출되는데 이제 main 컨텍스트라 정상. 변경 전과 동일한 메시지 나와야 함.

- [ ] **Step 1.9: Commit + push**

```bash
cd "C:\Users\Dongsu\Desktop\SMA-control"
git add Core/Inc/uart_protocol.h Core/Src/uart_protocol.c Core/Src/main.c
git commit -m "$(cat <<'EOF'
refactor(phase2): UART RX를 ring buffer 기반으로 분리

HAL_UART_RxCpltCallback (ISR)에서 frame state machine + handler
디스패치(Manual_Control / LoadCell_Read / printf 포함)를 직접 실행
하던 것을 ring buffer 256B push만 하도록 변경. main loop의
UartComm_Process()가 byte를 꺼내 state machine + frame 처리 담당.
의미 변경 없음.

- UartComm_ProcessRxByte는 uart_protocol.c 내부 static으로 변경
- UartComm_Process() public API 신규 (main loop에서 호출)
- ring buffer SPSC lock-free, uint8_t head/tail 자연 wrap-around
- dropped_count로 FIFO overflow 진단 가능

검증:
- CMD 0x01/0x03/0x04/0x06 송수신 동일 동작
- PID/Manual/Force/I2C 디버그 출력 동일
- g_rx_fifo.dropped_count 0 유지
EOF
)"
git push origin main
```

---

## 완료 후 상태

Phase 2 완료 시:
- `HAL_UART_RxCpltCallback` ≤ 5줄 (push + 다음 IT receive)
- 모든 UART frame 처리(Manual_Control, LoadCell_Read I2C blocking read, printf 디버그 메시지)가 main 컨텍스트에서 실행
- UART RX 안정성 ↑ — frame 처리에 시간이 걸려도 ring buffer가 byte 누락 방지

다음 phase: **Phase 3 — 모듈 추출** (controller.c 신규, do_fast_tick / do_slow_tick 본체를 Sensor_Update / Safety_Update / Controller_Update / Actuator_Apply로 이름 분리). 별도 plan 문서로 작성 예정.

---

## 자체 검토 결과

**Spec coverage**: 디자인 문서 섹션 6 Phase 2("`HAL_UART_RxCpltCallback` → ring buffer push만. frame parsing은 메인 `UartComm_Process()`.") — Task 1의 Step 1.5 (ISR 변경) + Step 1.4 (Process 함수 추가) + Step 1.6 (main loop 호출)이 정확히 이 범위를 커버.

**Placeholder scan**: TBD/TODO 없음. 모든 코드 블록이 완전한 상태로 제시됨.

**Type consistency**:
- `UartRxFifo_t`, `g_rx_fifo`, `head`/`tail`/`dropped_count` 필드 일관 사용
- `uart_rx_fifo_push` / `uart_rx_fifo_pop` 함수 시그니처 (Step 1.2 정의 — Step 1.4/1.5에서 사용) 일치
- `UartComm_Process()` (Step 1.1 declaration, Step 1.4 정의, Step 1.6 호출) 모두 동일 시그니처
- `UartComm_ProcessRxByte` static 변경 (Step 1.3) 후 같은 파일 내 호출 (Step 1.4) 일관

**범위/안전성**: 단일 commit이지만 step 단위로 검증. Step 1.5(ISR 변경)와 Step 1.6(main loop Process 호출) 사이는 빌드 가능하지만 RX 동작 안 함 — 두 step을 묶어 commit. 다른 로컬에서 빌드/플래시/검증 후 push된 commit을 revert로 안전하게 롤백 가능.
