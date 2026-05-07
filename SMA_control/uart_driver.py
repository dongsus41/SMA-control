"""
uart_driver.py
PC <-> MCU UART communication

Frame:  [0xAA] [CMD] [LEN] [DATA...] [CRC8]

  CMD 0x01  Control        PC->MCU  30B  pwm[6]+fan[6]+enable_pid[6]+target_temp[6*uint16]
  CMD 0x02  State          MCU->PC  24B  pwm[6]+fan[6]+temp[6*uint16]
  CMD 0x03  Gain Update    PC->MCU  13B  kp+ki+kd+channel
  CMD 0x04  Force Control  PC->MCU   6B  channel(1)+enable(1)+target_force(float32)
  CMD 0x05  Force State    MCU->PC  10B  mode(1)+channel(1)+force(float32)+displacement(float32)
  CMD 0x06  I2C Test       PC->MCU   0B
"""

import time
import struct
import serial
from PyQt5.QtCore import QThread, pyqtSignal

# ---- Constants ----
STX = 0xAA
CMD_CONTROL       = 0x01
CMD_STATE         = 0x02
CMD_GAIN_UPDATE   = 0x03
CMD_FORCE_CONTROL = 0x04   # PC -> MCU: channel(1) + enable(1) + target_force(float)
CMD_FORCE_STATE   = 0x05   # MCU -> PC: mode(1) + channel(1) + force(float) + disp(float)

CTRL_CH = 6

# Fan status constants (from temp_control.h)
FAN_OFF           = 0
FAN_ON            = 1
FAN_SAFETY_LEVEL1 = 17
FAN_SENSOR_ERROR  = 49

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
    length   = len(payload)
    crc_data = bytes([cmd, length]) + payload
    return bytes([STX, cmd, length]) + payload + bytes([crc8(crc_data)])


class UartWorker(QThread):
    """
    Signals:
        data_received(elapsed, temp_list, fan_list, pwm_list)
        force_received(elapsed, force, displacement)
        debug_message(str)
        error_occurred(str)
    """
    data_received  = pyqtSignal(float, list, list, list)
    force_received = pyqtSignal(float, float, float)   # elapsed, force, displacement
    debug_message  = pyqtSignal(str)
    error_occurred = pyqtSignal(str)

    def __init__(self, port="COM3", baudrate=115200):
        super().__init__()
        self.port       = port
        self.baudrate   = baudrate
        self.running    = False
        self.ser        = None
        self.start_time = 0

    def run(self):
        try:
            self.ser = serial.Serial(
                port=self.port, baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE, timeout=0.1)
        except Exception as e:
            self.error_occurred.emit(f"Serial Init Error: {e}")
            return

        self.running    = True
        self.start_time = time.time()

        state     = 'IDLE'
        cmd       = 0
        length    = 0
        data_buf  = bytearray()
        debug_buf = bytearray()

        while self.running:
            try:
                raw = self.ser.read(1)
                if not raw:
                    if debug_buf:
                        self._flush_debug(debug_buf)
                        debug_buf = bytearray()
                    continue
                byte = raw[0]
 
                if state == 'IDLE':
                    if byte == STX:
                        if debug_buf:
                            self._flush_debug(debug_buf)
                            debug_buf = bytearray()
                        state = 'WAIT_CMD'
                    else:
                        debug_buf.append(byte)
                        if byte == ord('\n'):
                            self._flush_debug(debug_buf)
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
                        debug_buf.extend([STX, cmd, byte])
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
                        self._handle_frame(cmd, bytes(data_buf))
                    else:
                        # CRC mismatch — treat as debug garbage
                        debug_buf.append(STX)
                        debug_buf.append(cmd)
                        debug_buf.append(length)
                        debug_buf.extend(data_buf)
                        debug_buf.append(byte)
                    state = 'IDLE'
 
            except Exception:
                pass

        if debug_buf:
            self._flush_debug(debug_buf)
        if self.ser and self.ser.is_open:
            self.ser.close()

    def _flush_debug(self, buf: bytearray):
        try:
            text = buf.decode('ascii', errors='replace').rstrip('\r\n')
            if text:
                self.debug_message.emit(text)
        except Exception:
            pass

    def _handle_frame(self, cmd: int, data: bytes):
        elapsed = time.time() - self.start_time
        
        # ▼▼▼ 어떤 프레임이 CRC를 통과했는지 확인 ▼▼▼
        # print(f"[RX FRAME OK] CMD: 0x{cmd:02X}, 데이터 길이: {len(data)}")

        if cmd == CMD_STATE:
            if len(data) >= 24:
                # pwm[6] + fan[6] + temp[6*uint16_LE]
                pwm_list  = list(data[0:6])
                fan_list  = list(data[6:12])
                temp_list = [((data[12 + i*2]) | (data[13 + i*2] << 8)) / 4.0
                             for i in range(6)]
                self.data_received.emit(elapsed, temp_list, fan_list, pwm_list)
            else:
                # ▼▼▼ 길이가 모자랄 때 에러 출력 ▼▼▼
                print(f"[RX ERROR] CMD_STATE (0x02) 데이터 길이 부족! "
                      f"수신: {len(data)}바이트, 필요: 최소 24바이트")

        elif cmd == CMD_FORCE_STATE:
            if len(data) >= 10:
                # mode(1) + channel(1) + force(float32_LE) + displacement(float32_LE)
                force, displacement = struct.unpack_from('<ff', data, 2)
                self.force_received.emit(elapsed, force, displacement)
            else:
                # ▼▼▼ 길이가 모자랄 때 에러 출력 ▼▼▼
                print(f"[RX ERROR] CMD_FORCE_STATE (0x05) 데이터 길이 부족! "
                      f"수신: {len(data)}바이트, 필요: 최소 10바이트")
    # ── TX ────────────────────────────────────────────────────

    def send_control_message(self, pwm, fan_on, pid_enable, target_temp):
        """CMD 0x01 — 온도/PWM/FAN 제어"""
        if not self.ser or not self.ser.is_open:
            return

        payload = bytearray()
        pwm_val = max(0, min(100, int(pwm)))
        fan_val = 1 if fan_on    else 0
        pid_val = 1 if pid_enable else 0

        for _ in range(CTRL_CH): payload.append(pwm_val)
        for _ in range(CTRL_CH): payload.append(fan_val)
        for _ in range(CTRL_CH): payload.append(pid_val)

        raw_target = int(target_temp * 4) & 0xFFFF
        for _ in range(CTRL_CH):
            payload.append(raw_target & 0xFF)
            payload.append((raw_target >> 8) & 0xFF)

        try:
            self.ser.write(build_frame(CMD_CONTROL, bytes(payload)))
        except serial.SerialException:
            pass

    def send_force_control_message(self, channel: int, enable: bool,
                                   target_force: float):
        """CMD 0x04 — 힘 제어 모드 설정
        payload: channel(uint8) + enable(uint8) + target_force(float32_LE) = 6 bytes
        """
        if not self.ser or not self.ser.is_open:
            return

        payload = struct.pack('<BBf',
                              int(channel),
                              1 if enable else 0,
                              float(target_force))
        try:
            self.ser.write(build_frame(CMD_FORCE_CONTROL, payload))
        except serial.SerialException:
            pass

    def send_gain_update(self, channel, kp, ki, kd):
        """CMD 0x03 — PID 게인 업데이트"""
        if not self.ser or not self.ser.is_open:
            return
        payload = struct.pack('<fffB', kp, ki, kd, channel)
        try:
            self.ser.write(build_frame(CMD_GAIN_UPDATE, bytes(payload)))
        except serial.SerialException:
            pass

    def send_i2c_test(self):
        """CMD 0x06 — I2C 통신 테스트 (payload 없음)"""
        if not self.ser or not self.ser.is_open:
            return
        try:
            self.ser.write(build_frame(CMD_I2C_TEST, b''))
        except serial.SerialException:
            pass

    def stop(self):
        self.running = False