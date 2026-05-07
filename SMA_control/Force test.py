"""
force_test.py — CMD 0x04 ~ 0x06 테스트 스크립트
MCU와 UART로 통신하며 힘 제어 관련 새 명령들을 검증합니다.

사용법:
  python force_test.py COM6
"""

import sys
import time
import struct
import serial
import threading

# ─── 프로토콜 상수 ───
STX = 0xAA
CMD_STATE         = 0x02
CMD_FORCE_CONTROL = 0x04
CMD_FORCE_STATE   = 0x05
CMD_I2C_TEST      = 0x06

CTRL_CH = 6


def crc8(data: bytes) -> int:
    crc = 0x00
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def build_frame(cmd: int, payload: bytes) -> bytes:
    length = len(payload)
    crc_data = bytes([cmd, length]) + payload
    return bytes([STX, cmd, length]) + payload + bytes([crc8(crc_data)])


# ─── 수신 스레드 ───
def rx_loop(ser: serial.Serial, running: list):
    """프로토콜 프레임과 printf 디버그 텍스트를 분리하여 표시"""
    state = 'IDLE'
    cmd = 0
    length = 0
    data_buf = bytearray()
    debug_buf = bytearray()

    while running[0]:
        try:
            raw = ser.read(1)
            if not raw:
                if debug_buf:
                    flush_debug(debug_buf)
                    debug_buf = bytearray()
                continue
            byte = raw[0]

            if state == 'IDLE':
                if byte == STX:
                    if debug_buf:
                        flush_debug(debug_buf)
                        debug_buf = bytearray()
                    state = 'WAIT_CMD'
                else:
                    debug_buf.append(byte)
                    if byte == ord('\n'):
                        flush_debug(debug_buf)
                        debug_buf = bytearray()

            elif state == 'WAIT_CMD':
                cmd = byte
                state = 'WAIT_LEN'

            elif state == 'WAIT_LEN':
                length = byte
                data_buf = bytearray()
                if length == 0:
                    state = 'WAIT_CRC'
                elif length > 250:
                    state = 'IDLE'
                else:
                    state = 'WAIT_DATA'

            elif state == 'WAIT_DATA':
                data_buf.append(byte)
                if len(data_buf) >= length:
                    state = 'WAIT_CRC'

            elif state == 'WAIT_CRC':
                crc_input = bytes([cmd, length]) + bytes(data_buf)
                if byte == crc8(crc_input):
                    handle_frame(cmd, bytes(data_buf))
                state = 'IDLE'

        except Exception:
            pass

    if debug_buf:
        flush_debug(debug_buf)


def flush_debug(buf: bytearray):
    try:
        text = buf.decode('ascii', errors='replace').rstrip('\r\n')
        if text:
            print(f"  [MCU] {text}")
    except:
        pass


def handle_frame(cmd: int, data: bytes):
    if cmd == CMD_STATE and len(data) >= 24:
        # 기존 온도 상태 — 간략히 표시
        temps = []
        for i in range(6):
            offset = 12 + i * 2
            raw = data[offset] | (data[offset + 1] << 8)
            temps.append(raw / 4.0)
        pwms = list(data[0:6])
        print(f"  [TEMP] T={temps[0]:.1f}°C  PWM={pwms[0]}%", end='\r')

    elif cmd == CMD_FORCE_STATE and len(data) >= 10:
        mode = data[0]
        channel = data[1]
        force = struct.unpack('<f', data[2:6])[0]
        displacement = struct.unpack('<f', data[6:10])[0]
        mode_str = "FORCE" if mode == 1 else "TEMP"
        print(f"  [FORCE] mode={mode_str}  ch={channel}  "
              f"force={force:.3f}g  disp={displacement:.3f}mm")


# ─── 명령 전송 함수 ───
def send_i2c_test(ser):
    """CMD 0x06: I2C 통신 테스트"""
    frame = build_frame(CMD_I2C_TEST, b'')
    ser.write(frame)
    print(">>> I2C 테스트 명령 전송")


def send_force_enable(ser, channel, target_force):
    """CMD 0x04: 힘 제어 활성화"""
    payload = struct.pack('<BBf', channel, 1, target_force)
    frame = build_frame(CMD_FORCE_CONTROL, bytes(payload))
    ser.write(frame)
    print(f">>> 힘 제어 ON: ch={channel}, target={target_force}g")


def send_force_disable(ser, channel):
    """CMD 0x04: 힘 제어 비활성화"""
    payload = struct.pack('<BBf', channel, 0, 0.0)
    frame = build_frame(CMD_FORCE_CONTROL, bytes(payload))
    ser.write(frame)
    print(f">>> 힘 제어 OFF: ch={channel}")


# ─── 메인 ───
def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM6"
    baud = 115200

    print(f"=== Force Control Test ===")
    print(f"포트: {port} @ {baud}baud")
    print()

    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except Exception as e:
        print(f"시리얼 열기 실패: {e}")
        return

    time.sleep(1)

    # 수신 스레드 시작
    running = [True]
    rx_thread = threading.Thread(target=rx_loop, args=(ser, running), daemon=True)
    rx_thread.start()

    print("명령어:")
    print("  1  →  I2C 통신 테스트 (CMD 0x06)")
    print("  2  →  힘 제어 ON (CMD 0x04)")
    print("  3  →  힘 제어 OFF (CMD 0x04)")
    print("  q  →  종료")
    print()

    try:
        while True:
            cmd = input(">>> ").strip()

            if cmd == '1':
                send_i2c_test(ser)

            elif cmd == '2':
                try:
                    ch = int(input("  채널 (0~5): ").strip())
                    target = float(input("  목표 힘 (g): ").strip())
                    send_force_enable(ser, ch, target)
                except ValueError:
                    print("  잘못된 입력")

            elif cmd == '3':
                try:
                    ch = int(input("  채널 (0~5): ").strip())
                    send_force_disable(ser, ch)
                except ValueError:
                    print("  잘못된 입력")

            elif cmd == 'q':
                break

    except KeyboardInterrupt:
        pass

    print("\n종료 중...")
    running[0] = False
    ser.close()
    print("완료")


if __name__ == "__main__":
    main()