from enum import Enum
from struct import calcsize, pack, unpack
from typing import Any, Self

from .system import (DeviceType, Headless, ModeControlFlight, ModeFlight,
                     ModelNumber, ModeMovement, ModeSystem, ModeUpdate,
                     Rotation, SensorOrientation)

class ISerializableBase:
    @staticmethod
    def get_size() -> int: ...

    def to_array(self) -> bytearray | bytes: ...

    @classmethod
    def parse(cls, data_array: bytearray | bytes) -> Self | None: ...

class ISerializable(ISerializableBase):
    @staticmethod
    def unpack(data_array: bytearray | bytes) -> tuple[Any, ...]: ...

def _to_basic_type(v: Any):
    if isinstance(v, Enum):
        return v.value
    elif isinstance(v, ISerializable):
        return v.to_array()
    return v

class SerializableMeta(type):
    def __new__(metacls, name: str, bases, namespace: dict[str, Any], /, fmt: str):
        namespace["get_size"] = lambda: calcsize(fmt)
        namespace["unpack"] = lambda data_array: unpack(fmt, data_array)

        if "to_array" not in namespace:
            namespace["to_array"] \
                = lambda self: pack(
                    fmt,
                    *map(_to_basic_type, self.__dict__.values())
                )
        return type.__new__(metacls, name, bases, namespace)

class DataType(Enum):
    None_ = 0x00
    Ping = 0x01
    """통신 확인"""
    Ack = 0x02
    """데이터 수신에 대한 응답"""
    Error = 0x03
    """오류"""
    Request = 0x04
    """지정한 타입의 데이터 요청"""
    Information = 0x07
    """펌웨어 및 장치 정보"""
    Control = 0x10
    """조종"""

    Command = 0x11
    Pairing = 0x12
    ResponseRate = 0x13

    # Light
    LightManual	 = 0x20
    """LED 수동 제어"""
    LightMode	 = 0x21
    """LED 모드 지정"""
    LightEvent	 = 0x22
    """LED 이벤트"""

    # 센서 RAW 데이터
    RawMotion	 = 0x30
    """Motion 센서 데이터 RAW 값"""

    # 상태,  센서
    State = 0x40
    """드론의 상태(비행 모드, 방위기준, 배터리량)"""
    Altitude = 0x43
    """높이, 고도"""
    Motion = 0x44
    """Motion 센서 데이터 처리한 값(IMU)"""
    VisionSensor = 0x47
    """Vision Sensor X, Y, Z"""

    # 설정
    Count = 0x50
    """카운트"""
    Bias = 0x51
    """엑셀, 자이로 바이어스 값"""
    Trim = 0x52
    LostConnection = 0x54
    """연결이 끊긴 후 반응 시간 설정"""

    # Device
    Motor		 = 0x60
    """모터 제어 및 현재 제어값 확인"""
    Buzzer		 = 0x62
    """버저 제어"""
    Battery		 = 0x64

    # Input
    Button		 = 0x70
    Joystick	 = 0x71

    # Information Assembled
    InformationAssembledForController = 0xA0
    """데이터 모음"""

    EndOfType = 0xDC

class Header(ISerializable, metaclass=SerializableMeta, fmt="<BBBB"):
    def __init__(
            self, data_type: DataType = DataType.None_, length: int = 0,
            from_: DeviceType = DeviceType.None_,
            to: DeviceType = DeviceType.None_):
        self.data_type = data_type
        self.length = length
        self.from_ = from_
        self.to = to

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        data_type, length, from_, to = cls.unpack(data_array)
        return Header(
            DataType(data_type), length, DeviceType(from_), DeviceType(to)
        )

class Ping(ISerializable, metaclass=SerializableMeta, fmt="<Q"):
    def __init__(self, system_time: int = 0):
        self.system_time = system_time

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return
        return Ping(*cls.unpack(data_array))

class Ack(ISerializable, metaclass=SerializableMeta, fmt="<QBH"):
    def __init__(self):
        self.system_time: int = 0
        self.data_type = DataType.None_
        self.crc16: int = 0

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        data = Ack()
        data.system_time, data.data_type, data.crc16 = cls.unpack(data_array)
        data.data_type = DataType(data.data_type)
        return data

class Error(ISerializable, metaclass=SerializableMeta, fmt="<QII"):
    def __init__(self):
        self.system_time: int = 0
        self.error_flags_for_sensor: int = 0
        self.error_flags_for_state: int = 0

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        data = Error()
        data.system_time, data.error_flags_for_sensor, data.error_flags_for_state = cls.unpack(data_array)
        return data

class Request(ISerializable, metaclass=SerializableMeta, fmt="<B"):
    def __init__(self, data_type: DataType = DataType.None_):
        self.data_type = data_type

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        data = Request()
        data.data_type, = cls.unpack(data_array)
        data.data_type = DataType(data.data_type)
        return data

class RequestOption(ISerializable, metaclass=SerializableMeta, fmt="<BI"):
    def __init__(self):
        self.dataType = DataType.None_
        self.option: int = 0

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        data = RequestOption()
        data.dataType, data.option = cls.unpack(data_array)
        data.dataType = DataType(data.dataType)
        return data

class Message(ISerializableBase):
    def __init__(self):
        self.message = ""

    def get_size(self):
        return len(self.message)

    def to_array(self):
        return bytearray(self.message.encode("ascii", "ignore"))

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) == 0:
            return ""

        data = Message()
        data.message = data_array[:len(data_array)].decode()
        return data

class SystemInformation(ISerializable, metaclass=SerializableMeta, fmt="<II"):
    def __init__(self):
        self.crc32bootloader: int = 0
        self.crc32application: int = 0

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        data = SystemInformation()
        data.crc32bootloader, data.crc32application = cls.unpack(data_array)
        return data

class Version(ISerializableBase):
    def __init__(self):
        self.build: int = 0
        self.minor: int = 0
        self.major: int = 0
        self.v: int = 0 # build, minor, major을 하나의 UInt32로 묶은 것(버젼 비교 시 사용)

    @staticmethod
    def get_size():
        return 4

    def to_array(self):
        return pack('<HBB', self.build, self.minor, self.major)

    @classmethod
    def parse(cls, dataArray: bytearray | bytes):
        if len(dataArray) != cls.get_size():
            return None

        data = Version()
        data.v, = unpack('<I', dataArray)
        data.build, data.minor, data.major = unpack('<HBB', dataArray)
        return data

class Information(ISerializableBase):
    def __init__(self):
        self.mode_update = ModeUpdate.None_

        self.model_number = ModelNumber.None_
        self.version = Version()

        self.year: int = 0
        self.month: int = 0
        self.day: int = 0

    @staticmethod
    def get_size():
        return 13

    def to_array(self):
        data_array = bytearray()
        data_array.extend(pack('<B', self.mode_update.value))
        data_array.extend(pack('<I', self.model_number.value))
        data_array.extend(self.version.to_array())
        data_array.extend(pack('<H', self.year))
        data_array.extend(pack('<B', self.month))
        data_array.extend(pack('<B', self.day))
        return data_array

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        data = Information()
        data.mode_update, = unpack("<B", data_array[:1])
        data.model_number, = unpack("<I", data_array[1:5])
        data.version = Version.parse(data_array[5:5+Version.get_size()])
        data.year, = unpack("<H", data_array[5+Version.get_size():7+Version.get_size()])
        data.month, = unpack("<B", data_array[7+Version.get_size():8+Version.get_size()])
        data.day, = unpack("<B", data_array[8+Version.get_size():9+Version.get_size()])

        data.mode_update = ModeUpdate(data.mode_update)
        data.model_number = ModelNumber(data.model_number)

        return data

class UpdateLocation(ISerializable, metaclass=SerializableMeta, fmt="<H"):
    def __init__(self):
        self.index_block_next: int = 0

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        data = UpdateLocation()
        data.index_block_next, = cls.unpack(data_array)
        return data

class Address(ISerializableBase):
    def __init__(self):
        self.address = bytearray()

    @staticmethod
    def get_size():
        return 16

    def to_array(self):
        return self.address

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        data = Address()
        data.address = data_array[:16]
        return data

class Pairing(ISerializable, metaclass=SerializableMeta, fmt="<BBBBBB"):
    def __init__(
            self, address0: int = 0, address1: int = 0, address2: int = 0,
            address3: int = 0, address4: int = 0, channel0: int = 0):
        self.address0 = address0
        self.address1 = address1
        self.address2 = address2
        self.address3 = address3
        self.address4 = address4
        self.channel0 = channel0

    # @staticmethod
    # def get_size():
    #     return 11

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        data = Pairing()
        (
            data.address0, data.address1, data.address2, data.address3,
            data.address4, data.channel0
        ) = cls.unpack(data_array)
        return data

class ResponseRate(ISerializable, metaclass=SerializableMeta, fmt="<B"):
    def __init__(self):
        self.response_rate: int = 0

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        data = ResponseRate()
        data.response_rate, = cls.unpack(data_array)
        return data

class Rssi(ISerializable, metaclass=SerializableMeta, fmt="<b"):
    def __init__(self):
        self.rssi: int = 0

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        data = Rssi()
        data.rssi, = cls.unpack(data_array)
        return data

class State(ISerializable, metaclass=SerializableMeta, fmt="<BBBBBBBB"):
    def __init__(self):
        self.mode_system = ModeSystem.None_
        self.mode_flight = ModeFlight.None_
        self.mode_control_flight = ModeControlFlight.None_
        self.mode_movement = ModeMovement.None_
        self.headless = Headless.None_
        self.control_speed: int = 0
        self.sensor_orientation = SensorOrientation.None_
        self.battery: int = 0

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        data = State()

        (
            data.mode_system, data.mode_flight, data.mode_control_flight,
            data.mode_movement, data.headless, data.control_speed,
            data.sensor_orientation, data.battery
        ) = cls.unpack(data_array)

        data.mode_system = ModeSystem(data.mode_system)
        data.mode_flight = ModeFlight(data.mode_flight)
        data.mode_control_flight = ModeControlFlight(data.mode_control_flight)
        data.mode_movement = ModeMovement(data.mode_movement)
        data.headless = Headless(data.headless)
        data.sensor_orientation = SensorOrientation(data.sensor_orientation)

        return data

class Attitude(ISerializable, metaclass=SerializableMeta, fmt="<hhh"):
    def __init__(self):
        self.roll: int = 0
        self.pitch: int = 0
        self.yaw: int = 0

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        data = Attitude()
        data.roll, data.pitch, data.yaw = cls.unpack(data_array)
        return data

class Position(ISerializable, metaclass=SerializableMeta, fmt="<fff"):
    def __init__(self):
        self.x: float = 0
        self.y: float = 0
        self.z: float = 0

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        data = Position()
        data.x, data.y, data.z = cls.unpack(data_array)
        return data

class Altitude(ISerializable, metaclass=SerializableMeta, fmt="<ffff"):
    def __init__(self):
        self.temperature: float = 0
        self.pressure: float = 0
        self.altitude: float = 0
        self.range_height: float = 0

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        data = Altitude()
        data.temperature, data.pressure, data.altitude, data.range_height = cls.unpack(data_array)
        return data

class Motion(ISerializable, metaclass=SerializableMeta, fmt="<hhhhhhhhh"):
    def __init__(self):
        self.accel_x: int = 0
        self.accel_y: int = 0
        self.accel_z: int = 0
        self.gyro_roll: int = 0
        self.gyro_pitch: int = 0
        self.gyro_yaw: int = 0
        self.angle_roll: int = 0
        self.angle_pitch: int = 0
        self.angle_yaw: int = 0

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        data = Motion()
        (
            data.accel_x, data.accel_y, data.accel_z, data.gyro_roll,
            data.gyro_pitch, data.gyro_yaw, data.angle_roll, data.angle_pitch,
            data.angle_yaw
        ) = cls.unpack(data_array)
        return data

class Range(ISerializable, metaclass=SerializableMeta, fmt="<hhhhhh"):
    def __init__(self):
        self.left: int = 0
        self.front: int = 0
        self.right: int = 0
        self.rear: int = 0
        self.top: int = 0
        self.bottom: int = 0

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        data = Range()
        (
            data.left, data.front, data.right, data.rear, data.top,
            data.bottom
        ) = cls.unpack(data_array)
        return data

class Trim(ISerializable, metaclass=SerializableMeta, fmt="<hhhh"):
    def __init__(self):
        self.roll: int = 0
        self.pitch: int = 0
        self.yaw: int = 0
        self.throttle: int = 0

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        data = Trim()
        data.roll, data.pitch, data.yaw, data.throttle = cls.unpack(data_array)
        return data

class ButtonFlagController(Enum):
    None_ = 0x0000

    # 버튼
    FrontLeft = 0x0001
    FrontRight = 0x0002

    MidUpLeft = 0x0004
    MidUpRight = 0x0008

    MidUp = 0x0010
    MidLeft = 0x0020
    MidRight = 0x0040
    MidDown = 0x0080

    BottomLeft = 0x0100
    BottomRight = 0x0200

class ButtonFlagDrone(Enum):
    None_ = 0x0000

    Reset = 0x0001

class ButtonEvent(Enum):
    None_ = 0x00

    Down = 0x01  # 누르기 시작
    Press = 0x02  # 누르는 중
    Up = 0x03  # 뗌

    EndContinuePress = 0x04  # 연속 입력 종료

class Button(ISerializable, metaclass=SerializableMeta, fmt="<HB"):
    def __init__(self):
        self.button: int = 0
        self.event = ButtonEvent.None_

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        data = Button()
        data.button, data.event = cls.unpack(data_array)
        data.event = ButtonEvent(data.event)
        return data

class BuzzerMode(Enum):
    Stop = 0    # 정지(Mode에서의 Stop은 통신에서 받았을 때 Buzzer를 끄는 용도로 사용, set으로만 호출)

    MuteInstantly = 1    # 묵음 즉시 적용
    MuteContinually = 2    # 묵음 예약

    ScaleInstantly = 3    # 음계 즉시 적용
    ScaleContinually = 4    # 음계 예약

    HzInstantly = 5    # 주파수 즉시 적용
    HzContinually = 6    # 주파수 예약

    EndOfType = 7

class BuzzerScale(Enum):
    C1 = 0x00; CS1 = 0x01; D1 = 0x02; DS1 = 0x03; E1 = 0x04; F1 = 0x05; FS1 = 0x06; G1 = 0x07; GS1 = 0x08; A1 = 0x09; AS1 = 0x0A; B1 = 0x0B
    C2 = 0x0C; CS2 = 0x0D; D2 = 0x0E; DS2 = 0x0F; E2 = 0x10; F2 = 0x11; FS2 = 0x12; G2 = 0x13; GS2 = 0x14; A2 = 0x15; AS2 = 0x16; B2 = 0x17
    C3 = 0x18; CS3 = 0x19; D3 = 0x1A; DS3 = 0x1B; E3 = 0x1C; F3 = 0x1D; FS3 = 0x1E; G3 = 0x1F; GS3 = 0x20; A3 = 0x21; AS3 = 0x22; B3 = 0x23
    C4 = 0x24; CS4 = 0x25; D4 = 0x26; DS4 = 0x27; E4 = 0x28; F4 = 0x29; FS4 = 0x2A; G4 = 0x2B; GS4 = 0x2C; A4 = 0x2D; AS4 = 0x2E; B4 = 0x2F

    C5 = 0x30; CS5 = 0x31; D5 = 0x32; DS5 = 0x33; E5 = 0x34; F5 = 0x35; FS5 = 0x36; G5 = 0x37; GS5 = 0x38; A5 = 0x39; AS5 = 0x3A; B5 = 0x3B
    C6 = 0x3C; CS6 = 0x3D; D6 = 0x3E; DS6 = 0x3F; E6 = 0x40; F6 = 0x41; FS6 = 0x42; G6 = 0x43; GS6 = 0x44; A6 = 0x45; AS6 = 0x46; B6 = 0x47
    C7 = 0x48; CS7 = 0x49; D7 = 0x4A; DS7 = 0x4B; E7 = 0x4C; F7 = 0x4D; FS7 = 0x4E; G7 = 0x4F; GS7 = 0x50; A7 = 0x51; AS7 = 0x52; B7 = 0x53
    C8 = 0x54; CS8 = 0x55; D8 = 0x56; DS8 = 0x57; E8 = 0x58; F8 = 0x59; FS8 = 0x5A; G8 = 0x5B; GS8 = 0x5C; A8 = 0x5D; AS8 = 0x5E; B8 = 0x5F

    EndOfType   = 0x60

    Mute        = 0xEE  # 묵음
    Fin         = 0xFF  # 악보의 끝

class BuzzerMelody(Enum):
    DoMiSol     = 0x00		# 도미솔
    SolMiDo     = 0x01		# 솔미도
    LaLa        = 0x02		# 라라
    SiRaSiRa    = 0x03		# 시라시라

    Warning1    = 0x04		# 경고 1
    Warning2    = 0x05		# 경고 2
    Warning3    = 0x06		# 경고 3
    Warning4    = 0x07		# 경고 4

    Du          = 0x08		# Trim -
    DuDu        = 0x09		# Trim - End
    DiDic       = 0x0A		# Trim Center
    DiDic2      = 0x0B		# Trim Center 2
    Di          = 0x0C		# Trim +
    DiDi        = 0x0D		# Trim + End

    BuzzSound1   = 0x0E
    BuzzSound2   = 0x0F
    BuzzSound3   = 0x10
    BuzzSound4   = 0x11

    Button       = 0x12
    Shot         = 0x13

    EndOfType    = 0x14

class Buzzer(ISerializable, metaclass=SerializableMeta, fmt="<BHH"):
    def __init__(self):
        self.mode = BuzzerMode.Stop
        self.value: int = 0
        self.time: int = 0

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        data = Buzzer()
        data.mode, data.value, data.time = cls.unpack(data_array)
        data.mode = BuzzerMode(data.mode)
        return data

class CommandType(Enum):
    None_ = 0x00

    Stop = 0x01
    """정지"""

    ModeControlFlight = 0x02
    """비행 제어 모드 설정"""
    Headless = 0x03
    """이스케이프 모드 설정"""
    ControlSpeed = 0x04
    """제어 속도 설정"""

    ClearBias = 0x05
    """자이로/엑셀 바이어스 리셋(트림도 같이 초기화 됨)"""
    ClearTrim = 0x06
    """트림 초기화"""

    FlightEvent = 0x07
    """비행 이벤트 실행"""

    SetDefault = 0x08
    """기본 설정으로 초기화"""
    ModeController = 0x0A
    """조종기 동작 모드(0x10:조종, 0x80:링크)"""
    Link = 0x0B
    """링크 제어(0:Client Mode, 1:Server Mode, 2:Pairing Start)"""
    LoadDefaultColor = 0x0C
    """기본 색상으로 변경"""

    Trim = 0x0D
    SetSwarm = 0x0F
    """군집 고도제어 설정"""

    ModeTest = 0xF0
    """테스트 락(테스트를 완료하기 전까진 사용 불가 / 27:활성화, 11:해제)"""

    EndOfType = 0xEC

class Command(ISerializable, metaclass=SerializableMeta, fmt="<BB"):
    def __init__(self, command_type: CommandType = CommandType.None_, option: int = ModeControlFlight.None_.value): # TODO is option intager?
        self.command_type = command_type
        self.option = option

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        data = Command()
        data.command_type, data.option = cls.unpack(data_array)
        data.command_type = CommandType(data.command_type)
        return data

class CommandLightEvent(ISerializableBase):
    def __init__(self):
        self.command = Command()
        self.event = LightEvent()

    @staticmethod
    def get_size():
        return Command.get_size() + LightEvent.get_size()

    def to_array(self):
        data_array = bytearray()
        data_array.extend(self.command.to_array())
        data_array.extend(self.event.to_array())
        return data_array

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        data = CommandLightEvent()
        data.command = Command.parse(data_array[:Command.get_size()])
        data.event = LightEvent.parse(data_array[Command.get_size():Command.get_size()+LightEvent.get_size()])
        return data

class CommandLightEventColor(ISerializableBase):
    def __init__(self):
        self.command = Command()
        self.event = LightEvent()
        self.color = Color()

    @staticmethod
    def get_size():
        return Command.get_size() + LightEvent.get_size() + Color.get_size()

    def to_array(self):
        data_array = bytearray()
        data_array.extend(self.command.to_array())
        data_array.extend(self.event.to_array())
        data_array.extend(self.color.to_array())
        return data_array

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        data = CommandLightEventColor()
        data.command = Command.parse(data_array[:Command.get_size()])
        data.event = LightEvent.parse(data_array[Command.get_size():Command.get_size()+LightEvent.get_size()])
        data.color = Color.parse(data_array[Command.get_size()+LightEvent.get_size():Command.get_size()+LightEvent.get_size()+Color.get_size()])
        return data

class CommandLightEventColors(ISerializableBase):
    def __init__(self):
        self.command = Command()
        self.event = LightEvent()
        self.colors = Colors.Black

    @staticmethod
    def get_size():
        return Command.get_size() + LightEvent.get_size() + 1

    def to_array(self):
        data_array = bytearray()
        data_array.extend(self.command.to_array())
        data_array.extend(self.event.to_array())
        data_array.extend(pack("<B", self.colors.value))
        return data_array

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        data = CommandLightEventColors()
        data.command = Command.parse(data_array[:Command.get_size()])
        data.event = LightEvent.parse(data_array[Command.get_size():Command.get_size()+LightEvent.get_size()])
        data.colors = unpack("<B", data_array[Command.get_size()+LightEvent.get_size():Command.get_size()+LightEvent.get_size()+1])

        data.colors = Colors(data.colors)

        return data

class ControlQuad8(ISerializable, metaclass=SerializableMeta, fmt="<bbbb"):
    def __init__(self, roll: int = 0, pitch: int = 0, yaw: int = 0, throttle: int = 0):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.throttle = throttle

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        data = ControlQuad8()
        data.roll, data.pitch, data.yaw, data.throttle = cls.unpack(data_array)
        return data

class ControlQuad8AndRequestData(
    ISerializable, metaclass=SerializableMeta, fmt="<bbbbb"
):
    def __init__(self):
        self.roll: int = 0
        self.pitch: int = 0
        self.yaw: int = 0
        self.throttle: int = 0
        self.data_type = DataType.None_

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        data = ControlQuad8AndRequestData()
        data.roll, data.pitch, data.yaw, data.throttle, data.data_type = cls.unpack(data_array)
        data.data_type = DataType(data.data_type)
        return data

class ControlPositionShort(ISerializable, metaclass=SerializableMeta, fmt="<hhhhhh"):
    def __init__(self):
        self.x: int = 0
        self.y: int = 0
        self.z: int = 0

        self.velocity: int = 0

        self.heading: int = 0
        self.rotational_velocity: int = 0

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        data = ControlPositionShort()
        data.x, data.y, data.z, data.velocity, data.heading, data.rotational_velocity = cls.unpack(data_array)
        return data

class ControlPosition(ISerializable, metaclass=SerializableMeta, fmt="<ffffhh"):
    def __init__(self):
        self.x: float = 0
        self.y: float = 0
        self.z: float = 0

        self.velocity: float = 0

        self.heading: int = 0
        self.rotational_velocity: int = 0

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        data = ControlPosition()
        data.x, data.y, data.z, data.velocity, data.heading, data.rotational_velocity = cls.unpack(data_array)
        return data

class MotorBlock(ISerializable, metaclass=SerializableMeta, fmt="<Bh"):
    def __init__(self):
        self.rotation: Rotation = Rotation.None_
        self.value: int = 0

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        data = MotorBlock()
        data.rotation, data.value = cls.unpack(data_array)
        data.rotation = Rotation(data.rotation)
        return data

class Motor(ISerializableBase):
    def __init__(
            self, motor0=MotorBlock(), motor1=MotorBlock(),
            motor2=MotorBlock(), motor3=MotorBlock()):
        self.motor = (motor0, motor1, motor2, motor3,)

    @staticmethod
    def get_size():
        return MotorBlock.get_size() * 4

    def to_array(self):
        return bytearray(*(self.motor[i].to_array() for i in range(3)))

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        motor_block_size = MotorBlock.get_size()
        return Motor(*(
            MotorBlock.parse(
                data_array[i * motor_block_size:(i + 1) * motor_block_size]
            ) for i in range(3) 
        )) # type: ignore

class MotorBlockV(ISerializable, metaclass=SerializableMeta, fmt="<h"):
    def __init__(self):
        self.value: int = 0

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        data = MotorBlockV()
        data.value, = cls.unpack(data_array)
        return data

class MotorV(ISerializableBase):
    def __init__(
            self, motor0=MotorBlockV(), motor1=MotorBlockV(),
            motor2=MotorBlockV(), motor3=MotorBlockV()):
        self.motor = (motor0, motor1, motor2, motor3,)

    @staticmethod
    def get_size():
        return MotorBlockV.get_size() * 4

    def to_array(self):
        return bytearray(*(self.motor[i].to_array() for i in range(3)))

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        motor_block_size = MotorBlockV.get_size()
        return Motor(*(
            MotorBlockV.parse(
                data_array[i * motor_block_size:(i + 1) * motor_block_size]
            ) for i in range(3) 
        )) # type: ignore

class MotorSingle(ISerializable, metaclass=SerializableMeta, fmt="<BBh"):
    def __init__(self):
        self.target: int = 0
        self.rotation = Rotation.None_
        self.value: int = 0

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        data = MotorSingle()
        data.target, data.rotation, data.value = cls.unpack(data_array)
        data.rotation = Rotation(data.rotation)
        return data

class MotorSingleV(ISerializable, metaclass=SerializableMeta, fmt="<Bh"):
    def __init__(self):
        self.target: int = 0
        self.value: int = 0

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        data = MotorSingleV()
        data.target, data.value = cls.unpack(data_array)
        return data

class InformationAssembledForController(
    ISerializable, metaclass=SerializableMeta, fmt="<hhhHhhhbbBb"
):
    def __init__(self):
        self.angle_roll: int = 0
        self.angle_pitch: int = 0
        self.angle_yaw: int = 0

        self.rpm: int = 0

        self.pos_x: int = 0
        self.pos_y: int = 0
        self.pos_z: int = 0

        self.speed_x: int = 0
        self.speed_y: int = 0

        self.range_height: int = 0

        self.rssi: int = 0

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        data = InformationAssembledForController()

        (
            data.angle_roll, data.angle_pitch, data.angle_yaw, data.rpm,
            data.pos_x, data.pos_y, data.pos_z,
            data.speed_x, data.speed_y, data.range_height, data.rssi
        ) = cls.unpack(data_array)

        return data

class InformationAssembledForEntry(
    ISerializable, metaclass=SerializableMeta, fmt="<hhhhhhhf"
):
    def __init__(self):
        self.angle_roll: int = 0
        self.angle_pitch: int = 0
        self.angle_yaw: int = 0

        self.pos_x: int = 0
        self.pos_y: int = 0
        self.pos_z: int = 0

        self.range_height: int = 0
        self.altitude: float = 0

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        data = InformationAssembledForEntry()
        (
            data.angle_roll, data.angle_pitch, data.angle_yaw, data.pos_x,
            data.pos_y, data.pos_z, data.range_height, data.altitude
        ) = cls.unpack(data_array)
        return data

class JoystickDirection(Enum):
    None_ = 0x00   # 정의하지 않은 영역(무시함)

    VT = 0x10      #   위(세로)
    VM = 0x20      # 중앙(세로)
    VB = 0x40      # 아래(세로)

    HL = 0x01      #   왼쪽(가로)
    HM = 0x02      #   중앙(가로)
    HR = 0x04      # 오른쪽(가로)

    TL = 0x11
    TM = 0x12
    TR = 0x14
    ML = 0x21
    CN = 0x22
    MR = 0x24
    BL = 0x41
    BM = 0x42
    BR = 0x44

class JoystickEvent(Enum):
    None_ = 0     # 이벤트 없음

    In = 1     # 특정 영역에 진입
    Stay = 2     # 특정 영역에서 상태 유지
    Out = 3     # 특정 영역에서 벗어남

    EndOfType  = 4

class JoystickBlock(ISerializable, metaclass=SerializableMeta, fmt="<bbBB"):
    def __init__(self):
        self.x: int = 0
        self.y: int = 0
        self.direction = JoystickDirection.None_
        self.event = JoystickEvent.None_

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        data = JoystickBlock()
        data.x, data.y, data.direction, data.event = cls.unpack(data_array)
        data.direction = JoystickDirection(data.direction)
        data.event = JoystickEvent(data.event)
        return data

class Joystick(ISerializableBase):
    def __init__(self):
        self.left = JoystickBlock()
        self.right = JoystickBlock()

    @staticmethod
    def get_size():
        return JoystickBlock.get_size() * 2

    def to_array(self):
        data_array = bytearray()
        data_array.extend(self.left.to_array())
        data_array.extend(self.right.to_array())
        return data_array

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        data = Joystick()
        data.left = JoystickBlock.parse(data_array[:JoystickBlock.get_size()])
        data.right = JoystickBlock.parse(data_array[JoystickBlock.get_size():JoystickBlock.get_size()*2])
        return data

class LightModeDrone(Enum):
    None_                   = 0x00

    # Team (Forward, Rear를 동시에 제어)
    TeamRgbNone				= 0x10
    TeamRgbManual			= 0x11		# 수동 제어
    TeamRgbHold				= 0x12		# 지정한 색상을 계속 켬
    TeamRgbFlicker			= 0x13		# 깜빡임			
    TeamRgbFlickerDouble	= 0x14		# 깜빡임(두 번 깜빡이고 깜빡인 시간만큼 꺼짐)			
    TeamRgbDimming			= 0x15		# 밝기 제어하여 천천히 깜빡임
    TeamRgbSunrise			= 0x16		# 꺼진 상태에서 점점 밝아짐
    TeamRgbSunset			= 0x17		# 켜진 상태에서 점점 어두워짐
    TeamRgbRainbow			= 0x18		# 무지개색
    TeamRgbRainbow2			= 0x19		# 무지개색
    TeamRgbRedBlue			= 0x1A		# Red-Blue

    TeamFlowForward			= 0x1E		# 앞으로 흐름 
    TeamWarning				= 0x1F		# 경고

    # Body
    BodyNone				= 0x20		
    BodyManual				= 0x21		# 수동 제어
    BodyHold				= 0x22		# 지정한 색상을 계속 켬
    BodyFlicker				= 0x23		# 깜빡임			
    BodyFlickerDouble		= 0x24		# 깜빡임(두 번 깜빡이고 깜빡인 시간만큼 꺼짐)			
    BodyDimming				= 0x25		# 밝기 제어하여 천천히 깜빡임
    BodySunrise				= 0x26		# 꺼진 상태에서 점점 밝아짐
    BodySunset				= 0x27		# 켜진 상태에서 점점 어두워짐
    BodyRainbow				= 0x28		# 무지개색
    BodyRainbow2			= 0x29		# 무지개색
    BodyRedBlue				= 0x2A		# Red-Blue
    BodyCard				= 0x2B		# 카드 색상 표시(이벤트용)
    BodyWarning				= 0x2F		# 경고

    # Link
    LinkNone				= 0x30		
    LinkManual				= 0x31		# 수동 제어
    LinkHold				= 0x32		# 지정한 색상을 계속 켬
    LinkFlicker				= 0x33		# 깜빡임			
    LinkFlickerDouble		= 0x34		# 깜빡임(두 번 깜빡이고 깜빡인 시간만큼 꺼짐)			
    LinkDimming				= 0x35		# 밝기 제어하여 천천히 깜빡임
    LinkSunrise				= 0x36		# 꺼진 상태에서 점점 밝아짐
    LinkSunset				= 0x37		# 켜진 상태에서 점점 어두워짐

    EndOfType               = 0x60

class LightFlagsDrone(Enum):
    None_ = 0x0000

    BodyRed = 0x0001
    BodyGreen = 0x0002
    BodyBlue = 0x0004

    TeamRed = 0x0008
    TeamBlue = 0x0010

    Link = 0x0080

class LightModeController(Enum):
    # Team
    TeamNone						= 0x10
    TeamManual						= 0x11		# 수동 제어
    TeamHold						= 0x12		# 지정한 색상을 계속 켬
    TeamFlicker						= 0x13		# 깜빡임			
    TeamFlickerDouble				= 0x14		# 깜빡임(두 번 깜빡이고 깜빡인 시간만큼 꺼짐)			
    TeamDimming						= 0x15		# 밝기 제어하여 천천히 깜빡임
    TeamSunrise						= 0x16		# 꺼진 상태에서 점점 밝아짐
    TeamSunset						= 0x17		# 켜진 상태에서 점점 어두워짐
    TeamRedBlue						= 0x1A		# Red-Blue

    # Array 6
    Array6None						= 0x30
    Array6Manual					= 0x31		# 수동 제어
    Array6Hold						= 0x32		# [전체] 지정한 색상을 계속 켬
    Array6Flicker					= 0x33		# [전체] 깜빡임			
    Array6FlickerDouble				= 0x34		# [전체] 깜빡임(두 번 깜빡이고 깜빡인 시간만큼 꺼짐)		
    Array6Dimming					= 0x35		# [전체] 밝기 제어하여 천천히 깜빡임

    # Array 6 Value
    Array6ValueNone					= 0x40		# [개별] 0 ~ 255 사이의 값 표시
    Array6ValueHold					= 0x42		# [개별] 0 ~ 255 사이의 값 표시
    Array6ValueFlicker				= 0x43		# [개별] 0 ~ 255 사이의 값 표시
    Array6ValueFlickerDouble		= 0x44		# [개별] 0 ~ 255 사이의 값 표시
    Array6ValueDimming				= 0x45		# [개별] 0 ~ 255 사이의 값 표시

    # Array 6 Function
    Array6FunctionNone				= 0x50
    Array6Pendulum					= 0x51
    Array6FlowLeft					= 0x52		# [개별] 왼쪽으로 흐름
    Array6FlowRight					= 0x53		# [개별] 오른쪽으로 흐름

    EndOfType                       = 0x60

class LightFlagsController(Enum):
    None_ = 0x0000

    TeamRed	= 0x0001
    TeamBlue = 0x0002

    E0 = 0x0004
    E1 = 0x0008
    E2 = 0x0010
    E3 = 0x0020
    E4 = 0x0040
    E5 = 0x0080

class Color(ISerializable, metaclass=SerializableMeta, fmt="<BBB"):
    def __init__(self):
        self.r: int = 0
        self.g: int = 0
        self.b: int = 0

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        data = Color()
        data.r, data.g, data.b = cls.unpack(data_array)
        return data

class Colors(Enum):
    AliceBlue              = 0
    AntiqueWhite           = 1
    Aqua                   = 2
    Aquamarine             = 3
    Azure                  = 4
    Beige                  = 5
    Bisque                 = 6
    Black                  = 7
    BlanchedAlmond         = 8
    Blue                   = 9
    BlueViolet             = 10
    Brown                  = 11
    BurlyWood              = 12
    CadetBlue              = 13
    Chartreuse             = 14
    Chocolate              = 15
    Coral                  = 16
    CornflowerBlue         = 17
    Cornsilk               = 18
    Crimson                = 19
    Cyan                   = 20
    DarkBlue               = 21
    DarkCyan               = 22
    DarkGoldenRod          = 23
    DarkGray               = 24
    DarkGreen              = 25
    DarkKhaki              = 26
    DarkMagenta            = 27
    DarkOliveGreen         = 28
    DarkOrange             = 29
    DarkOrchid             = 30
    DarkRed                = 31
    DarkSalmon             = 32
    DarkSeaGreen           = 33
    DarkSlateBlue          = 34
    DarkSlateGray          = 35
    DarkTurquoise          = 36
    DarkViolet             = 37
    DeepPink               = 38
    DeepSkyBlue            = 39
    DimGray                = 40
    DodgerBlue             = 41
    FireBrick              = 42
    FloralWhite            = 43
    ForestGreen            = 44
    Fuchsia                = 45
    Gainsboro              = 46
    GhostWhite             = 47
    Gold                   = 48
    GoldenRod              = 49
    Gray                   = 50
    Green                  = 51
    GreenYellow            = 52
    HoneyDew               = 53
    HotPink                = 54
    IndianRed              = 55
    Indigo                 = 56
    Ivory                  = 57
    Khaki                  = 58
    Lavender               = 59
    LavenderBlush          = 60
    LawnGreen              = 61
    LemonChiffon           = 62
    LightBlue              = 63
    LightCoral             = 64
    LightCyan              = 65
    LightGoldenRodYellow   = 66
    LightGray              = 67
    LightGreen             = 68
    LightPink              = 69
    LightSalmon            = 70
    LightSeaGreen          = 71
    LightSkyBlue           = 72
    LightSlateGray         = 73
    LightSteelBlue         = 74
    LightYellow            = 75
    Lime                   = 76
    LimeGreen              = 77
    Linen                  = 78
    Magenta                = 79
    Maroon                 = 80
    MediumAquaMarine       = 81
    MediumBlue             = 82
    MediumOrchid           = 83
    MediumPurple           = 84
    MediumSeaGreen         = 85
    MediumSlateBlue        = 86
    MediumSpringGreen      = 87
    MediumTurquoise        = 88
    MediumVioletRed        = 89
    MidnightBlue           = 90
    MintCream              = 91
    MistyRose              = 92
    Moccasin               = 93
    NavajoWhite            = 94
    Navy                   = 95
    OldLace                = 96
    Olive                  = 97
    OliveDrab              = 98
    Orange                 = 99
    OrangeRed              = 100
    Orchid                 = 101
    PaleGoldenRod          = 102
    PaleGreen              = 103
    PaleTurquoise          = 104
    PaleVioletRed          = 105
    PapayaWhip             = 106
    PeachPuff              = 107
    Peru                   = 108
    Pink                   = 109
    Plum                   = 110
    PowderBlue             = 111
    Purple                 = 112
    RebeccaPurple          = 113
    Red                    = 114
    RosyBrown              = 115
    RoyalBlue              = 116
    SaddleBrown            = 117
    Salmon                 = 118
    SandyBrown             = 119
    SeaGreen               = 120
    SeaShell               = 121
    Sienna                 = 122
    Silver                 = 123
    SkyBlue                = 124
    SlateBlue              = 125
    SlateGray              = 126
    Snow                   = 127
    SpringGreen            = 128
    SteelBlue              = 129
    Tan                    = 130
    Teal                   = 131
    Thistle                = 132
    Tomato                 = 133
    Turquoise              = 134
    Violet                 = 135
    Wheat                  = 136
    White                  = 137
    WhiteSmoke             = 138
    Yellow                 = 139
    YellowGreen            = 140

    EndOfType              = 141

class LightManual(ISerializable, metaclass=SerializableMeta, fmt="<HB"):
    def __init__(self):
        self.flags: int = 0
        self.brightness: int = 0

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        data = LightManual()
        data.flags, data.brightness = cls.unpack(data_array)
        return data

class LightMode(ISerializable, metaclass=SerializableMeta, fmt="<BH"):
    def __init__(self):
        self.mode: int = 0
        self.interval: int = 0

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        data = LightMode()
        data.mode, data.interval = cls.unpack(data_array)
        return data

class LightEvent(ISerializable, metaclass=SerializableMeta, fmt="<BHB"):
    def __init__(self):
        self.event: int = 0
        self.interval: int = 0
        self.repeat: int = 0

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        data = LightEvent()
        data.event, data.interval, data.repeat = cls.unpack(data_array)
        return data

class LightModeColor(ISerializableBase):
    def __init__(self):
        self.mode = LightMode()
        self.color = Color()

    @staticmethod
    def get_size():
        return LightMode.get_size() + Color.get_size()

    def to_array(self):
        data = bytearray()
        data.extend(self.mode.to_array())
        data.extend(self.color.to_array())
        return data

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        data = LightModeColor()
        data.mode = LightMode.parse(data_array[:LightMode.get_size()])
        data.color = Color.parse(data_array[LightMode.get_size():LightMode.get_size()+Color.get_size()])
        return data

class LightModeColors(ISerializableBase):
    def __init__(self):
        self.mode = LightMode()
        self.colors = Colors.Black

    @staticmethod
    def get_size():
        return LightMode.get_size() + 1

    def to_array(self):
        dataArray = bytearray()
        dataArray.extend(self.mode.to_array())
        dataArray.extend(pack("<B", self.colors.value))
        return dataArray

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        data = LightModeColors()
        data.mode = LightMode.parse(data_array[:LightMode.get_size()])
        data.colors, = unpack("<B", data_array[LightMode.get_size():LightMode.get_size()+1])

        data.colors = Colors(data.colors)

        return data

class LightEventColor(ISerializableBase):
    def __init__(self):
        self.event = LightEvent()
        self.color = Color()

    @staticmethod
    def get_size():
        return LightEvent.get_size() + Color.get_size()

    def to_array(self):
        data_array = bytearray()
        data_array.extend(self.event.to_array())
        data_array.extend(self.color.to_array())
        return data_array

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        data = LightEventColor()

        indexStart = 0
        indexEnd = LightEvent.get_size()
        data.event = LightEvent.parse(data_array[indexStart:indexEnd])

        # TODO why??
        # indexStart = indexEnd
        # indexEnd += Color.get_size()
        # data.command = Color.parse(dataArray[indexStart:indexEnd])

        return data

class LightEventColors(ISerializableBase):
    def __init__(self):
        self.event = LightEvent()
        self.colors = Colors.Black

    @staticmethod
    def get_size():
        return LightEvent.get_size() + 1

    def to_array(self):
        data_array = bytearray()
        data_array.extend(self.event.to_array())
        data_array.extend(pack("<B", self.colors.value))
        return data_array

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        data = LightEventColors()
        data.event = LightEvent.parse(data_array[:LightEvent.get_size()])
        data.colors, = unpack("<B", data_array[LightEvent.get_size():LightEvent.get_size()+1])

        data.colors = Colors(data.colors)

        return data

class RawMotion(ISerializable, metaclass=SerializableMeta, fmt="<hhhhhh"):
    def __init__(self):
        self.accel_x: int = 0
        self.accel_y: int = 0
        self.accel_z: int = 0
        self.gyro_roll: int = 0
        self.gyro_pitch: int = 0
        self.gyro_yaw: int = 0

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        data = RawMotion()
        data.accel_x, data.accel_y, data.accel_z, data.gyro_roll, data.gyro_pitch, data.gyro_yaw = cls.unpack(data_array)
        return data

class VisionSensor(ISerializable, metaclass=SerializableMeta, fmt="<fff"):
    def __init__(self):
        self.x: float = 0
        self.y: float = 0
        self.z: float = 0

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        data = VisionSensor()
        data.x, data.y, data.z = cls.unpack(data_array)
        return data

class Count(ISerializable, metaclass=SerializableMeta, fmt="<QHHH"):
    def __init__(self):
        self.time_flight: int = 0

        self.count_takeOff: int = 0
        self.count_landing: int = 0
        self.count_accident: int = 0

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        data = Count()
        (
            data.time_flight, data.count_takeOff, data.count_landing,
            data.count_accident
        )= cls.unpack(data_array)
        return data

class Bias(ISerializable, metaclass=SerializableMeta, fmt="<hhhhhh"):
    def __init__(self):
        self.accel_x: int = 0
        self.accel_y: int = 0
        self.accel_z: int = 0
        self.gyro_roll: int = 0
        self.gyro_pitch: int = 0
        self.gyro_yaw: int = 0

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        data = Bias()
        (
            data.accel_x, data.accel_y, data.accel_z, data.gyro_roll,
            data.gyro_pitch, data.gyro_yaw
        ) = cls.unpack(data_array)
        return data

class Weight(ISerializable, metaclass=SerializableMeta, fmt="<f"):
    def __init__(self):
        self.weight: float = 0

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        data = Weight()
        data.weight, = cls.unpack(data_array)
        return data

class LostConnection(ISerializable, metaclass=SerializableMeta, fmt="<HHI"):
    def __init__(self):
        self.time_neutral: int = 0
        self.time_landing: int = 0
        self.time_stop: int = 0

    @classmethod
    def parse(cls, data_array: bytearray | bytes):
        if len(data_array) != cls.get_size():
            return

        data = LostConnection()
        data.time_neutral, data.time_landing, data.time_stop \
            = cls.unpack(data_array)
        return data