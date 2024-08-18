from enum import Enum
from time import perf_counter

from .crc import crc16
from .protocol import DataType, DeviceType, Header

# 데이터 수신 상태
class StateLoading(Enum):
    Ready = 0
    """수신 대기"""
    Receiving = 1
    """수신중"""
    Loaded = 2
    """수신 완료 후 명령어 보관소에 대기중"""
    Failure = 3
    """수신 실패"""

# 데이터 섹션 구분
class Section(Enum):
    Start = 0
    """전송 시작 코드"""
    Header = 1
    """헤더"""
    Data = 2
    """데이터"""
    End = 3
    """데이터 확인"""

class Receiver:
    __slots__ = (
        "state",
        "section",
        "section_old",
        "index",
        "header",
        "time_receive_start",
        "time_receive_complete",
        "_buffer",
        "data",
        "crc16received",
        "crc16calculated",
        "message",
    )

    def __init__(self):
        self.state = StateLoading.Ready
        self.section = Section.Start
        self.section_old = Section.End
        self.index: int = 0

        self.header = Header()
        self.time_receive_start: float = 0
        self.time_receive_complete: float = 0

        self._buffer = bytearray()
        self.data = bytearray()

        self.crc16received: int = 0
        self.crc16calculated: int = 0

        self.message: str | None = None

    def _raise_error(self, *args: str):
        self.state = StateLoading.Failure
        self.message = " / ".join(("Error", "Receiver",) + args)
        return self.state

    def call(self, data: int):
        now = perf_counter() * 1000

        self.message = None

        # First Step
        if self.state == StateLoading.Failure:
            self.state = StateLoading.Ready

        # Second Step
        match self.state:
            case StateLoading.Ready:
                self.section = Section.Start
                self.index = 0
            case StateLoading.Receiving:
                # 데이터 수신을 시작한지 600ms 시간이 지난 경우 오류 출력
                if (self.time_receive_start + 600) < now:
                    return self._raise_error(str(StateLoading.Receiving), "Time over.")
            case StateLoading.Loaded:
                return self.state

        # Third Step
        if self.section != self.section_old:
            self.index = 0
            self.section_old = self.section

        # Third Step
        match self.section:
            case Section.Start:
                match self.index:
                    case 0:
                        if data == 0x0A:
                            self.state = StateLoading.Receiving
                        else:
                            self.state = StateLoading.Failure
                            return self.state
                        self.time_receive_start = now
                    case 1:
                        if data != 0x55:
                            self.state = StateLoading.Failure
                            return self.state
                        else:
                            self.section = Section.Header
                    case _:
                        return self._raise_error(str(Section.Start), "Index over.")
            case Section.Header:
                match self.index:
                    case 0:
                        self.header = Header()

                        if data in DataType:
                            self.header.data_type = DataType(data)
                        else:
                            return self._raise_error(str(Section.Header), "DataType Error. 0x{0:02X}".format(data))

                        self.crc16calculated = crc16(data, 0)
                    case 1:
                        self.header.length = data
                        self.crc16calculated = crc16(data, self.crc16calculated)

                        if self.header.length > 128:
                            return self._raise_error(str(Section.Header), "Data length is longer than 128. [{0}]".format(self.header.length))
                    case 2:
                        if data in DeviceType:
                            self.header.from_ = DeviceType(data)
                        else:
                            return self._raise_error(str(Section.Header), "DeviceType Error. 0x{0:02X}".format(data))

                        self.crc16calculated = crc16(data, self.crc16calculated)
                    case 3:
                        if data in DeviceType:
                            self.header.to = DeviceType(data)
                        else:
                            return self._raise_error(str(Section.Header), "DeviceType Error. 0x{0:02X}".format(data))

                        self.crc16calculated = crc16(data, self.crc16calculated)

                        if self.header.length == 0:
                            self.section = Section.End
                        else:
                            self.section = Section.Data
                            self._buffer.clear()
                    case _:
                        return self._raise_error(str(Section.Header), "Index over.")
            case Section.Data:
                self._buffer.append(data)
                self.crc16calculated = crc16(data, self.crc16calculated)

                if self.index == self.header.length - 1:
                    self.section = Section.End
            case Section.End:
                match self.index:
                    case 0:
                        self.crc16received = data
                    case 1:
                        self.crc16received = (data << 8) | self.crc16received

                        if self.crc16received == self.crc16calculated:
                            self.data = self._buffer.copy()
                            self.time_receive_complete = now
                            self.state = StateLoading.Loaded
                            self.message = "Success / Receiver / Section.End / Receive complete / {0} / [receive: 0x{1:04X}]".format(self.header.data_type, self.crc16received)
                            return self.state
                        else:
                            return self._raise_error(str(Section.End), "CRC Error", str(self.header.data_type), f"[receive: 0x{self.crc16received:04X}, calculate: 0x{self.crc16calculated:04X}]")
                    case _:
                        return self._raise_error(str(Section.End), "Index over.")

        # Forth Step
        if self.state == StateLoading.Receiving:
            self.index += 1

        return self.state

    def checked(self):
        self.state = StateLoading.Ready
