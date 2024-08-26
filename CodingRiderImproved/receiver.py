from enum import Enum
from time import perf_counter

from .crc import crc16
from .protocol import DataType, DeviceType, Header

# 데이터 수신 상태
class StateLoading(Enum):
    READY = 0
    """수신 대기"""
    RECEIVING = 1
    """수신중"""
    LOADED = 2
    """수신 완료 후 명령어 보관소에 대기중"""
    FAILURE = 3
    """수신 실패"""

# 데이터 섹션 구분
class Section(Enum):
    START = 0
    """전송 시작 코드"""
    HEADER = 1
    """헤더"""
    DATA = 2
    """데이터"""
    END = 3
    """데이터 확인"""

class Receiver:
    __slots__ = (
        "_buffer",
        "crc16received",
        "crc16calculated",
        "data",
        "header",
        "index",
        "message",
        "section",
        "section_old",
        "state",
        "time_receive_complete",
        "time_receive_start",
    )

    def __init__(self):
        self.state = StateLoading.READY
        self.section = Section.START
        self.section_old = Section.END
        self.index: int = 0

        self.header = Header()
        self.time_receive_start: float = 0
        self.time_receive_complete: float = 0

        self._buffer = bytearray()
        self.data = bytearray()

        self.crc16received: int = 0
        self.crc16calculated: int = 0

        self.message: str | None = None

    def _set_message(self, *args: str):
        self.message = " / ".join(args)

    def _raise_error(self, *args: str):
        self.state = StateLoading.FAILURE
        self._set_message("Error", "Receiver", *args)
        return self.state

    def call(self, data: int):
        now = perf_counter() * 1000

        self.message = None

        # First Step
        if self.state == StateLoading.FAILURE:
            self.state = StateLoading.READY

        # Second Step
        match self.state:
            case StateLoading.READY:
                self.section = Section.START
                self.index = 0
            case StateLoading.RECEIVING:
                # 데이터 수신을 시작한지 600ms 시간이 지난 경우 오류 출력
                if (self.time_receive_start + 600) < now:
                    return self._raise_error(
                        str(StateLoading.RECEIVING), "Time over."
                    )
            case StateLoading.LOADED:
                return self.state

        # Third Step
        if self.section != self.section_old:
            self.index = 0
            self.section_old = self.section

        # Fourth Step
        match self.section:
            case Section.START:
                match self.index:
                    case 0:
                        if data == 0x0A:
                            self.state = StateLoading.RECEIVING
                        else:
                            self.state = StateLoading.FAILURE
                            return self.state
                        self.time_receive_start = now
                    case 1:
                        if data != 0x55:
                            self.state = StateLoading.FAILURE
                            return self.state
                        else:
                            self.section = Section.HEADER
                    case _:
                        return self._raise_error(
                            str(Section.START), "Index over."
                        )
            case Section.HEADER:
                match self.index:
                    case 0:
                        self.header = Header()

                        if data in DataType:
                            self.header.data_type = DataType(data)
                        else:
                            return self._raise_error(
                                str(Section.HEADER),
                                f"DataType Error. 0x{data:02X}"
                            )

                        self.crc16calculated = crc16(data, 0)
                    case 1:
                        self.header.length = data
                        self.crc16calculated = crc16(
                            data, self.crc16calculated
                        )

                        if self.header.length > 128:
                            return self._raise_error(
                                str(Section.HEADER),
                                "Data length is longer than 128. "
                                f"[{self.header.length}]"
                            )
                    case 2:
                        if data in DeviceType:
                            self.header.from_ = DeviceType(data)
                        else:
                            return self._raise_error(
                                str(Section.HEADER),
                                f"DeviceType Error. 0x{data:02X}"
                            )

                        self.crc16calculated = crc16(
                            data, self.crc16calculated
                        )
                    case 3:
                        if data in DeviceType:
                            self.header.to = DeviceType(data)
                        else:
                            return self._raise_error(
                                str(Section.HEADER),
                                f"DeviceType Error. 0x{data:02X}"
                            )

                        self.crc16calculated = crc16(
                            data, self.crc16calculated
                        )

                        if self.header.length == 0:
                            self.section = Section.END
                        else:
                            self.section = Section.DATA
                            self._buffer.clear()
                    case _:
                        return self._raise_error(
                            str(Section.HEADER), "Index over."
                        )
            case Section.DATA:
                self._buffer.append(data)
                self.crc16calculated = crc16(data, self.crc16calculated)

                if self.index == self.header.length - 1:
                    self.section = Section.END
            case Section.END:
                match self.index:
                    case 0:
                        self.crc16received = data
                    case 1:
                        self.crc16received = (data << 8) | self.crc16received

                        if self.crc16received == self.crc16calculated:
                            self.data = self._buffer.copy()
                            self.time_receive_complete = now
                            self.state = StateLoading.LOADED
                            self.message = self._set_message(
                                "Success", "Receiver", str(Section.END),
                                "Receive complete",
                                str(self.header.data_type),
                                f"[receive: 0x{self.crc16received:04X}]"
                            )
                            return self.state
                        else:
                            return self._raise_error(
                                str(Section.END),
                                "CRC Error",
                                str(self.header.data_type),
                                f"[receive: 0x{self.crc16received:04X}"
                                f", calculate: 0x{self.crc16calculated:04X}]"
                            )
                    case _:
                        return self._raise_error(
                            str(Section.END), "Index over."
                        )

        # Fifth Step
        if self.state == StateLoading.RECEIVING:
            self.index += 1

        return self.state

    def checked(self):
        self.state = StateLoading.READY
