from enum import Enum

class ModelNumber(Enum):
    None_                   = 0x00000000

    #                          AAAABBCC, AAAA(Project Number), BB(Device Type), CC(Revision)
    Drone_11_Drone_P0		= 0x000B1000	# reserve
    Drone_11_Drone_P1		= 0x000B1001	# 65 no coding
    Drone_11_Drone_P2		= 0x000B1002	# fixWing
    Drone_11_Drone_P3		= 0x000B1003	# 65 coding
    Drone_11_Drone_P4		= 0x000B1004	# 95 battle drone
    Drone_11_Drone_P5		= 0x000B1005	# skykick evolution drone
    Drone_11_Drone_P6		= 0x000B1006	# reserve
    Drone_11_Drone_P7		= 0x000B1007	# reserve
    Drone_11_Drone_P8		= 0x000B1008	# reserve
    Drone_11_Drone_P9		= 0x000B1009	# reserve

    Drone_11_Controller_P0	= 0x000B2000	# 65 Drone, No USB, Fixed Wing, Return
    Drone_11_Controller_P1	= 0x000B2001	# 65 drone, USB
    Drone_11_Controller_P2	= 0x000B2002	# 65 drone, No USB
    Drone_11_Controller_P3	= 0x000B2003	# Bird Wing, Fixed Wing, NoReturn
    Drone_11_Controller_P4	= 0x000B2004	#  Battle drone, USB
    Drone_11_Controller_P5	= 0x000B2005	# skykick evolution controller
    Drone_11_Controller_P6	= 0x000B2006	# reserve
    Drone_11_Controller_P7	= 0x000B2007	# reserve
    Drone_11_Controller_P8	= 0x000B2008	# reserve
    Drone_11_Controller_P9	= 0x000B2009	# reserve

    Drone_11_Link_P0		= 0x000B3000	# Drone_11_Link_P0 RoboRobo
    Drone_11_Link_P1		= 0x000B3001	# reserve

    # Drone_12 chip
    Drone_12_Drone_P0		= 0x000C1000	# coding drone stm32f401rc + xn297
    Drone_12_Drone_P1		= 0x000C1001	# coding drone STM32F407VE + nrf24l01
    Drone_12_Drone_P2		= 0x000C1002	# reserve

    Drone_12_Controller_P0	= 0x000C2000	# coding drone stm32f401rc + xn297
    Drone_12_Controller_P1	= 0x000C2001	# coding drone STM32F407VE + nrf24l01
    Drone_12_Controller_P2	= 0x000C2002	# reserve

    Drone_12_Link_P0		= 0x000C3000	# Drone_12_Link_P0
    Drone_12_Link_P1		= 0x000C3001	# 	reserve		

    Drone_12_Tester_P0		= 0x000CA000	# Drone All Tester
    Drone_12_Tester_P1		= 0x000CA001	# reserve

class DeviceType(Enum):
    None_       = 0x00

    Drone       = 0x10
    """드론(Server)"""

    Controller  = 0x20
    """조종기(Client)"""

    LinkClient  = 0x30
    """링크 모듈(Client)"""
    LinkServer  = 0x31
    """링크 모듈(Server)"""
    BleClient   = 0x32
    """BLE 클라이언트"""
    BleServer   = 0x33
    """BLE 서버"""

    Range       = 0x40
    """거리 센서 모듈"""

    Base        = 0x70

    ByScratch   = 0x80
    """바이스크래치"""
    Scratch     = 0x81
    """스크래치"""
    Entry       = 0x82
    """네이버 엔트리"""

    Tester      = 0xA0
    """테스터"""
    Monitor     = 0xA1
    """모니터"""
    Updater     = 0xA2
    """펌웨어 업데이트 도구"""
    Encrypter   = 0xA3
    """암호화 도구"""

    EndOfType   = 0xA4

    Whispering  = 0xFE
    """바로 인접한 장치까지만 전달(받은 장치는 자기 자신에게 보낸 것처럼 처리하고 타 장치에 전달하지 않음)"""
    Broadcasting = 0xFF
    """연결된 모든 장치에 전달"""

class ModeDrone(Enum) :
    None_       = 0x00

    Flight      = 0x10
    """비행"""

    Link        = 0x80
    """중계"""

    Error       = 0xA0
    """오류(문제로 인해 정상적인 동작을 할 수 없는 경우)"""

    EndOfType   = 0xA1

class ModeController(Enum) :
    None_       = 0x00

    Control     = 0x10
    """조종"""
    Setup       = 0x11
    """설정"""

    Link        = 0x80
    """링크"""

    Error       = 0xA0
    """오류"""

    EndOfType   = 0xA1

class ModeConnectionType(Enum):
    None_               = 0x00

    PairingStart		= 0x01
    """페어링 시작(주소 초기화 후 대기 // 한쪽에서는 새로운 주소를 생성하여 전송)"""
    PairingExchange		= 0x02
    """페어링 데이터 교환"""
    PairingComplete		= 0x03
    """드론 페어링 완료"""

    Ready				= 0x04
    """장치와 연결하지 않은 상태(장치와 연결 전 또는 연결 해제 완료 후 이 상태로 전환됨)"""

    ConnectingStart		= 0x05
    """장치 연결 시작"""
    Connecting			= 0x06
    """장치 연결 확인"""
    Connected			= 0x07
    """장치 연결 완료"""

    LostConnection		= 0x08
    """연결을 잃음(Server-Client간 통신이 되지 않는 상태)"""

    Disconnected		= 0x09
    """장치 연결 해제 완료"""

    EndOfPairing        = 0x0A

class ModeSystem(Enum):
    None_        = 0x00

    Boot         = 0x10
    Start        = 0x11
    Running      = 0x12
    ReadyToReset = 0x13

    Error        = 0xA0

    EndOfType    = 0x06

class ModeControlFlight(Enum):
    None_       = 0x00

    Attitude    = 0x10
    """자세 - X, Y는 각도(deg)로 입력받음, Z,Yaw는 속도(m/s)로 입력 받음"""
    Position    = 0x11
    """위치 - X, Y, Z, Yaw는 속도(m/s)로 입력 받음"""

    EndOfType   = 0x14

class ModeFlight(Enum):
    None_       = 0x00

    Ready       = 0x10
    """준비"""

    Start       = 0x11
    """이륙 준비"""
    Takeoff     = 0x12
    """이륙 (Flight로 자동전환)"""
    Flight      = 0x13
    """비행"""
    Landing     = 0x14
    """착륙"""
    Flip        = 0x15
    """회전"""
    Reverse     = 0x16
    """뒤집기"""

    Stop        = 0x20
    """강제 정지"""

    Accident    = 0x30
    """사고 (Ready로 자동전환)"""
    Error       = 0x31
    """오류"""

    Test        = 0x40
    """테스트 모드"""

    EndOfType   = 0x41

class ModeUpdate(Enum):
    None_               = 0x00

    Ready               = 0x01
    """업데이트 가능 상태"""
    Update              = 0x02
    """업데이트 중"""
    Complete            = 0x03
    """업데이트 완료"""

    Failed              = 0x04
    """업데이트 실패(업데이트 완료까지 갔으나 body의 CRC16이 일치하지 않는 경우 등)"""

    NotAvailable        = 0x05
    """업데이트 불가능 상태(Debug 모드 등)"""
    RunApplication      = 0x06
    """어플리케이션 동작 중"""
    NotRegistered       = 0x07
    """등록되지 않음"""

    EndOfType           = 0x08

class ErrorFlagsForSensor(Enum):
    None_                           = 0x00000000

    Motion_NoAnswer                 = 0x00000001
    """Motion 센서 응답 없음"""
    Motion_WrongValue               = 0x00000002
    """Motion 센서 잘못된 값"""
    Motion_NotCalibrated            = 0x00000004
    """Gyro Bias 보정이 완료되지 않음"""
    Motion_Calibrating              = 0x00000008
    """Gyro Bias 보정 중"""

    Pressure_NoAnswer               = 0x00000010
    """압력 센서 응답 없음"""
    Pressure_WrongValue             = 0x00000020
    """압력 센서 잘못된 값"""

    RangeGround_NoAnswer            = 0x00000100
    """바닥 거리 센서 응답 없음"""
    RangeGround_WrongValue          = 0x00000200
    """바닥 거리 센서 잘못된 값"""

    Flow_NoAnswer                   = 0x00001000
    """Flow 센서 응답 없음"""
    Flow_WrongValue                 = 0x00002000
    """Flow 잘못된 값"""
    Flow_CannotRecognizeGroundImage = 0x00004000
    """바닥 이미지를 인식할 수 없음"""

class ErrorFlagsForState(Enum):
    None_                                   = 0x00000000

    NotRegistered                           = 0x00000001
    """장치 등록이 안됨"""
    FlashReadLock_UnLocked                  = 0x00000002
    """플래시 메모리 읽기 Lock이 안 걸림"""
    BootloaderWriteLock_UnLocked            = 0x00000004
    """부트로더 영역 쓰기 Lock이 안 걸림"""
    
    TakeoffFailure_CheckPropellerAndMotor   = 0x00000010
    """이륙 실패"""
    CheckPropellerVibration                 = 0x00000020
    """프로펠러 진동발생"""
    Attitude_NotStable                      = 0x00000040
    """자세가 많이 기울어져 있거나 뒤집어져 있을때"""
    
    CanNotFlip_LowBattery                   = 0x00000100
    """배터리가 30이하"""
    CanNotFlip_TooHeavy                     = 0x00000200
    """기체가 무거움"""

class FlightEvent(Enum):
    None_        = 0x00

    Stop         = 0x10
    """정지"""
    Takeoff      = 0x11
    """이륙"""
    Landing      = 0x12
    """착륙"""

    Reverse      = 0x13
    """뒤집기"""

    FlipFront    = 0x14
    """회전"""
    FlipRear     = 0x15
    """회전"""
    FlipLeft     = 0x16
    """회전"""
    FlipRight    = 0x17
    """회전"""

    Return       = 0x18

    ResetHeading = 0xA0
    """헤딩 리셋(헤드리스 모드 일 때 현재 heading을 0도로 변경)"""

    EndOfType    = 0xA1

class Direction(Enum):
    None_     = 0x00

    Left      = 0x01
    Front     = 0x02
    Right     = 0x03
    Rear      = 0x04

    Top       = 0x05
    Bottom    = 0x06

    Center    = 0x07

    EndOfType = 0x08

class Rotation(Enum):
    None_            = 0x00

    Clockwise        = 0x01
    Counterclockwise = 0x02

    EndOfType        = 0x03

class SensorOrientation(Enum):
    None_        = 0x00

    Normal       = 0x01
    ReverseStart = 0x02
    Reversed     = 0x03

    EndOfType    = 0x04

class Headless(Enum):
    None_     = 0x00

    Headless  = 0x01
    Normal    = 0x02

    EndOfType = 0x03

class TrimDirection(Enum):
    None_               = 0x00

    RollIncrease        = 0x01
    RollDecrease        = 0x02
    PitchIncrease       = 0x03
    PitchDecrease       = 0x04
    YawIncrease         = 0x05
    YawDecrease         = 0x06
    ThrottleIncrease    = 0x07
    ThrottleDecrease    = 0x08

    Reset               = 0x09
    """전체 트림 리셋"""

    EndOfType           = 0x0A

class ModeMovement(Enum):
    None_      = 0x00

    Ready      = 0x01
    Hovering   = 0x02
    Moving     = 0x03
    ReturnHome = 0x04

    EndOfType  = 0x05
