from os import linesep
from queue import Queue
from struct import pack
from sys import stdout
from threading import Thread
from time import perf_counter, sleep, time
from typing import Any, Callable, Literal

from colorama import Back, Fore, Style
from colorama import init as colorama_init
from serial import Serial, SerialException

from .crc import crc16
from .protocol import *
from .receiver import Receiver, StateLoading
from .system import FlightEvent

def bytearray_to_string(data_array: bytes | bytearray) -> str:
    string = ""

    for data in data_array:
        string += f"{data:02X} "

    return string

class Drone:
    __slots__ = (
        "_serialport",
        "_buffer_queue",
        "_buffer_handler",
        "_thread",
        "_flag_thread_run",
        "_receiver",
        "_flag_check_background",
        "_flag_show_error_message",
        "_flag_show_log_message",
        "_flag_show_transfer_data",
        "_flag_show_receive_data",
        "_event_handler",
        "_storage_header",
        "_storage",
        "_storage_count",
        "_parser",
        "initialized_time",
    )

# BaseFunctions Start

    def __init__(
            self, flag_check_background=True, flag_show_error_message=False,
            flag_show_log_message=False, flag_show_transfer_data=False,
            flag_show_receive_data=False):
        self._serialport: Serial | None = None
        self._buffer_queue = Queue(4096)
        self._buffer_handler = bytearray()

        self._thread: Thread | None = None
        self._flag_thread_run = False

        self._receiver = Receiver()

        self._flag_check_background = flag_check_background
        self._flag_show_error_message = flag_show_error_message
        self._flag_show_log_message = flag_show_log_message
        self._flag_show_transfer_data = flag_show_transfer_data
        self._flag_show_receive_data = flag_show_receive_data

        self._event_handler: dict[DataType, Callable[[Any], None] | None] \
            = dict.fromkeys(DataType)

        self._storage_header: dict[DataType, Header | None] \
            = dict.fromkeys(DataType)

        self._storage: dict[DataType, Any] = dict.fromkeys(DataType)
        self._storage_count: dict[DataType, int] = dict.fromkeys(DataType, 0)

        self._parser: dict[DataType, Callable[[Any], Any] | None] \
            = dict.fromkeys(DataType)

        self._parser[DataType.Ping] = Ping.parse
        self._parser[DataType.Ack] = Ack.parse
        self._parser[DataType.Error] = Error.parse
        self._parser[DataType.Information] = Information.parse
        self._parser[DataType.VisionSensor] = VisionSensor.parse
        self._parser[DataType.Pairing] = Pairing.parse

        self._parser[DataType.RawMotion] = RawMotion.parse

        self._parser[DataType.State] = State.parse
    
        self._parser[DataType.Altitude] = Altitude.parse
        self._parser[DataType.Motion] = Motion.parse

        self._parser[DataType.Count] = Count.parse
        self._parser[DataType.Trim] = Trim.parse

        self._parser[DataType.Button] = Button.parse
        self._parser[DataType.Joystick] = Joystick.parse

        self._parser[DataType.InformationAssembledForController] \
            = InformationAssembledForController.parse

        self.initialized_time = time()

        colorama_init()

    def __del__(self):
        self.close()

    def _receive(self):
        self._buffer_queue.put(self._serialport.read())

        # 수신 데이터 백그라운드 확인이 활성화 된 경우 데이터 자동 업데이트
        if self._flag_check_background:
            while self.check() != DataType.None_:
                pass

    def _receiving(self):
        while self._flag_thread_run:
            self._receive()

    def is_open(self) -> bool:
        if self._serialport is not None:
            return self._serialport.is_open
        return False

    def open(self, portname: str) -> bool:
        try:
            self._serialport = Serial(
                port=portname,
                baudrate=57600
            )

            if self.is_open():
                self._flag_thread_run = True
                self._thread = Thread(target=self._receiving, daemon=True)
                self._thread.start()

                # 로그 출력
                self._print_log(f"Connected. ({portname})")

                return True
            else:
                # 오류 메세지 출력
                self._print_error("Could not connect to CodingRiderImproved.")

                return False
        except SerialException:
                # 오류 메세지 출력
                self._print_error("Could not connect to CodingRiderImproved.")

                return False

    def close(self):
        # 로그 출력
        if self.is_open():
            self._print_log("Closing serial port.")

        self._print_log("Thread Flag False.")

        if self._flag_thread_run:
            self._flag_thread_run = False

        self._print_log("Thread Join.")

        if self._thread is not None:
            self._thread.join(timeout=1)
            self._print_receive_data_end()

        self._print_log("Port Close.")

        if self.is_open():
            self._serialport.close()

    def make_transfer_data_array(
            self, header: Header, data: bytes | ISerializable) -> bytearray:
        if isinstance(data, ISerializable):
            data = data.to_array()

        data_array = bytearray()
        data_array.extend((0x0A, 0x55,))
        data_array.extend(header.to_array())
        data_array.extend(data)
        data_array.extend(pack('H', crc16(data, crc16(header.to_array(), 0))))

        return data_array

    def transfer(self, header: Header, data: ISerializable):
        if not self.is_open():
            return

        data_array = self.make_transfer_data_array(header, data)

        self._serialport.write(data_array)

        # 송신 데이터 출력
        self._print_transfer_data(data_array)

        return data_array

    def check(self):
        while not self._buffer_queue.empty():
            data_array = self._buffer_queue.get_nowait()
            self._buffer_queue.task_done()

            if data_array is not None and len(data_array) > 0:
                # 수신 데이터 출력
                self._print_receive_data(data_array)

                self._buffer_handler.extend(data_array)

        while len(self._buffer_handler) > 0:
            state_loading = self._receiver.call(self._buffer_handler.pop(0))

            # 오류 출력
            if state_loading == StateLoading.Failure:
                # 수신 데이터 출력(줄넘김)
                self._print_receive_data_end()

                # 오류 메세지 출력
                self._print_error(self._receiver.message)

            # 로그 출력
            if state_loading == StateLoading.Loaded:
                # 수신 데이터 출력(줄넘김)
                self._print_receive_data_end()

                # 로그 출력
                self._print_log(self._receiver.message)

            if self._receiver.state == StateLoading.Loaded:
                self._handler(self._receiver.header, self._receiver.data)
                return self._receiver.header.data_type

        return DataType.None_

    # def check_detail(self):
    #     while not self._buffer_queue.empty():
    #         data_array = self._buffer_queue.get_nowait()
    #         self._buffer_queue.task_done()

    #         if data_array is not None and len(data_array) > 0:
    #             # 수신 데이터 출력
    #             self._print_receive_data(data_array)

    #             self._buffer_handler.extend(data_array)

    #     while len(self._buffer_handler) > 0:
    #         state_loading = self._receiver.call(self._buffer_handler.pop(0))

    #         # 오류 출력
    #         if state_loading == StateLoading.Failure:
    #             # 수신 데이터 출력(줄넘김)
    #             self._print_receive_data_end()

    #             # 오류 메세지 출력
    #             self._print_error(self._receiver.message)

    #         # 로그 출력
    #         if state_loading == StateLoading.Loaded:
    #             # 수신 데이터 출력(줄넘김)
    #             self._print_receive_data_end()

    #             # 로그 출력
    #             self._print_log(self._receiver.message)

    #         if self._receiver.state == StateLoading.Loaded:
    #             self._handler(self._receiver.header, self._receiver.data)
    #             return self._receiver.header, self._receiver.data

    #     return None, None

    def _handler(self, header: Header, data_array: bytearray):
        # 들어오는 데이터를 저장
        self._run_handler(header, data_array)

        # 콜백 이벤트 실행
        self._run_event_handler(header.data_type)

        # 데이터 처리 완료 확인
        self._receiver.checked()

        return header.data_type

    def _run_handler(self, header: Header, data_array: bytearray):
        # 일반 데이터 처리
        data_type = header.data_type
        if (parser := self._parser[data_type]) is not None:
            self._storage_header[data_type] = header
            self._storage[data_type] = parser(data_array)
            self._storage_count[data_type] += 1

    def _run_event_handler(self, data_type: DataType):
        if ((event_handler := self._event_handler[data_type]) is not None and
                (storage := self._storage[data_type]) is not None):
            return event_handler(storage)
        return None

    def set_event_handler(
            self, data_type: DataType, event_handler: Callable[[Any], None]):
        self._event_handler[data_type] = event_handler

    def _print_log_base(self, prefix: str, message: Any):
        if message is not None:
            stdout.write(
                f"{prefix}[{time() - self.initialized_time:10.03f}] {message}"
                f"{Style.RESET_ALL}{linesep}"
            )

    def _print_log(self, message: Any):
        if self._flag_show_log_message:
            self._print_log_base(Fore.GREEN, message)

    def _print_error(self, message: Any):
        if self._flag_show_error_message:
            self._print_log_base(Fore.RED, message)

    def _print_transfer_data(self, data: bytearray | bytes | None):
        if self._flag_show_transfer_data \
            and data is not None and len(data) > 0:
            stdout.write(
                f"{Back.YELLOW}{Fore.BLACK}{bytearray_to_string(data)}"
                f"{Style.RESET_ALL}{linesep}"
            )

    def _print_receive_data(self, data: bytearray | bytes):
        if self._flag_show_receive_data:
            stdout.write(
                f"{Back.CYAN}{Fore.BLACK}{bytearray_to_string(data)}"
                f"{Style.RESET_ALL}"
            )
            stdout.flush()

    def _print_receive_data_end(self):
        if self._flag_show_receive_data:
            stdout.write(linesep)

# BaseFunctions End

# Common Start

    def send(
            self, data_type: DataType, length: int, from_: DeviceType,
            to: DeviceType, data: ISerializable):
        return self.transfer(Header(data_type, length, from_, to), data)

    def send_command(
            self, device_from: DeviceType, device_to: DeviceType,
            command_type: CommandType, option: int = 0):
        return self.send(
            DataType.Command, Command.get_size(), device_from, device_to,
            Command(command_type, option)
        )

    def send_ping(self, device_type: DeviceType):
        return self.send(
            DataType.Ping, Ping.get_size(), DeviceType.Base, device_type,
            Ping(0)
        )

    def send_request(self, device_type: DeviceType, data_type: DataType):
        return self.send(
            DataType.Request, Request.get_size(), DeviceType.Base, device_type,
            Request(data_type)
        )

    def send_pairing(
            self, device_type: DeviceType, address0: int, address1: int, address2: int,
            address3: int, address4: int, channel0: int):
        return self.send(
            DataType.Pairing, Pairing.get_size(), DeviceType.Base, device_type,
            Pairing(address0, address1, address2, address3, address4, channel0)
        )

# Common Start

# Control Start

    def send_take_off(self):
        return self.send_command(
            DeviceType.Base, DeviceType.Drone,
            CommandType.FlightEvent, FlightEvent.Takeoff.value
        )

    def send_landing(self):
        return self.send_command(
            DeviceType.Base, DeviceType.Drone,
            CommandType.FlightEvent, FlightEvent.Landing.value
        )

    def send_stop(self):
        return self.send_command(
            DeviceType.Base, DeviceType.Drone,
            CommandType.Stop, 0
        )

    def send_control(self, roll: int, pitch: int, yaw: int, throttle: int):
        """
         `roll`, `pitch`, `yaw`, `throttle`: -100 ~ 100
         * `roll` > 0: Right, `roll` < 0: Left
         * `pitch` > 0: Forward, `pitch` < 0: Backward
         * `yaw` > 0: Counter clockwise, `yaw` < 0: Clockwise
         * `throttle` > 0: Up, `throttle` < 0: Down
        """
        return self.send(
            DataType.Control, ControlQuad8.get_size(), DeviceType.Base,
            DeviceType.Drone, ControlQuad8(roll, pitch, yaw, throttle)
        )

    def send_control_while(
            self, roll: int, pitch: int, yaw: int, throttle: int,
            time_ms: int):
        """
         `roll`, `pitch`, `yaw`, `throttle`: -100 ~ 100
         * `roll` > 0: Right, `roll` < 0: Left
         * `pitch` > 0: Forward, `pitch` < 0: Backward
         * `yaw` > 0: Counter clockwise, `yaw` < 0: Clockwise
         * `throttle` > 0: Up, `throttle` < 0: Down
        """
        time_sec = time_ms / 1000
        time_start = perf_counter()

        while (perf_counter() - time_start) < time_sec:
            self.send_control(roll, pitch, yaw, throttle)
            sleep(0.02)

        return self.send_control(roll, pitch, yaw, throttle)

    def send_control_position16(
            self, x: int, y: int, z: int, velocity: int, heading: int,
            rotational_velocity: int):
        data = ControlPositionShort()
        data.x = x
        data.y = y
        data.z = z
        data.velocity = velocity
        data.heading = heading
        data.rotational_velocity = rotational_velocity
        return self.send(
            DataType.Control, ControlPositionShort.get_size(), DeviceType.Base,
            DeviceType.Drone, data
        )

    def send_control_position(
            self, x: float, y: float, z: float, velocity: float, heading: int,
            rotational_velocity: int):
        """
         `x`, `y`, `z`: -10.0 ~ 10.0 meter\\
         `velocity`: 0.1 ~ 2.0 m/s\\
         `heading`: -360 ~ 360 degree\\
         `rotational_velocity`: 10 ~ 360 degree/s
        """
        data = ControlPosition()
        data.x = x
        data.y = y
        data.z = z
        data.velocity = velocity
        data.heading = heading
        data.rotational_velocity = rotational_velocity
        return self.send(
            DataType.Control, ControlPosition.get_size(), DeviceType.Base,
            DeviceType.Drone, data
        )

# Control End

# Setup Start

    def send_command_light_event(
            self, command_type: CommandType, option: int,
            light_event: LightModeDrone | LightModeController, interval: int,
            repeat: int):
        header = Header(
            DataType.Command, CommandLightEvent.get_size(), DeviceType.Base,
            DeviceType.Drone if isinstance(light_event, LightModeDrone)
            else DeviceType.Controller
        )

        data = CommandLightEvent()

        data.command.command_type = command_type
        data.command.option = option

        data.event.event = light_event.value
        data.event.interval = interval
        data.event.repeat = repeat

        return self.transfer(header, data)

    def send_command_light_event_color(
            self, command_type: CommandType, option: int,
            light_event: LightModeDrone | LightModeController, interval: int,
            repeat: int, r: int, g: int, b: int):
        header = Header(
            DataType.Command, CommandLightEventColor.get_size(), DeviceType.Base,
            DeviceType.Drone if isinstance(light_event, LightModeDrone)
            else DeviceType.Controller
        )

        data = CommandLightEventColor()

        data.command.command_type = command_type
        data.command.option = option

        data.event.event = light_event.value
        data.event.interval = interval
        data.event.repeat = repeat

        data.color.r = r
        data.color.g = g
        data.color.b = b

        return self.transfer(header, data)

    def send_command_light_event_colors(
            self, command_type: CommandType, option: int,
            light_event: LightModeDrone | LightModeController, interval: int,
            repeat: int, colors: Colors):
        header = Header(
            DataType.Command, CommandLightEventColors.get_size(),
            DeviceType.Base,
            DeviceType.Drone if isinstance(light_event, LightModeDrone)
            else DeviceType.Controller
        )

        data = CommandLightEventColors()

        data.command.command_type = command_type
        data.command.option = option

        data.event.event = light_event.value
        data.event.interval = interval
        data.event.repeat = repeat

        data.colors = colors

        return self.transfer(header, data)

    def send_mode_control_flight(self, mode_control_flight: ModeControlFlight):
        return self.send_command(DeviceType.Base, DeviceType.Drone, CommandType.ModeControlFlight, mode_control_flight.value)

    def send_headless(self, headless: Headless):
        return self.send_command(DeviceType.Base, DeviceType.Drone, CommandType.Headless, headless.value)

    def send_trim(self, roll: int, pitch: int, yaw: int, throttle: int):
        data = Trim()
        data.roll = roll
        data.pitch = pitch
        data.yaw = yaw
        data.throttle = throttle
        return self.send(
            DataType.Trim, Trim.get_size(), DeviceType.Base, DeviceType.Drone,
            data
        )

    def send_lost_connection(self, time_neutral, time_landing, time_stop):
        data = LostConnection()
        data.time_neutral = time_neutral
        data.time_landing = time_landing
        data.time_stop = time_stop
        return self.send(
            DataType.LostConnection, LostConnection.get_size(), DeviceType.Base,
            DeviceType.Drone, data
        )

    def send_flight_event(self, flight_event: FlightEvent):
        return self.send_command(
            DeviceType.Base, DeviceType.Drone,CommandType.FlightEvent,
            flight_event.value
        )

    def send_clear_bias(self):
        return self.send_command(
            DeviceType.Base, DeviceType.Drone, CommandType.ClearBias, 0
        )

    def send_clear_trim(self): # TODO wtf is DeviceType.Tester
        return self.send_command(
            DeviceType.Tester, DeviceType.Drone, CommandType.ClearTrim, 0
        )

    def send_set_default(self, device_type: DeviceType):
        return self.send_command(
            DeviceType.Base, device_type, CommandType.SetDefault, 0
        )

    def send_set_swarm(self):
        return self.send_command(
            DeviceType.Base, DeviceType.Drone, CommandType.SetSwarm, 0
        )

# Setup End

# Device Start

    def send_motor(self, motor0: int, motor1: int, motor2: int, motor3: int):
        data = Motor()

        data.motor[0].rotation = Rotation.Clockwise
        data.motor[0].value = motor0

        data.motor[1].rotation = Rotation.Counterclockwise
        data.motor[1].value = motor1

        data.motor[2].rotation = Rotation.Clockwise
        data.motor[2].value = motor2

        data.motor[3].rotation = Rotation.Counterclockwise
        data.motor[3].value = motor3

        return self.send(DataType.Motor, Motor.get_size(), DeviceType.Base, DeviceType.Drone, data)

# Device End

# Light Start

    def send_light_manual(
            self,
            device_type: Literal[DeviceType.Drone, DeviceType.Controller],
            flags: int, brightness: int):
        data = LightManual()
        data.flags = flags
        data.brightness = brightness
        return self.send(
            DataType.LightManual, LightManual.get_size(), DeviceType.Base,
            device_type, data
        )

    def send_light_mode_color(self, light_mode: LightModeDrone | LightModeController, interval: int, r: int, g: int, b: int):
        header = Header(
            DataType.LightMode, LightModeColor.get_size(), DeviceType.Base
        )

        if isinstance(light_mode, LightModeDrone):
            header.to = DeviceType.Drone
        elif isinstance(light_mode, LightModeController):
            header.to = DeviceType.Controller

        data = LightModeColor()

        data.mode.mode = light_mode.value
        data.mode.interval = interval

        data.color.r = r
        data.color.g = g
        data.color.b = b

        return self.transfer(header, data)

    def send_light_mode_colors(self, light_mode: LightModeDrone | LightModeController, interval: int, colors: Colors):
        header = Header(
            DataType.LightMode, LightModeColors.get_size(), DeviceType.Base
        )

        if isinstance(light_mode, LightModeDrone):
            header.to = DeviceType.Drone
        elif isinstance(light_mode, LightModeController):
            header.to = DeviceType.Controller

        data = LightModeColors()

        data.mode.mode = light_mode.value
        data.mode.interval = interval

        data.colors = colors

        return self.transfer(header, data)

    def send_light_event_color(self, light_event: LightModeDrone | LightModeController, interval: int, repeat: int, r: int, g: int, b: int):
        header = Header(
            DataType.LightEvent, LightEventColor.get_size(), DeviceType.Base
        )

        if isinstance(light_event, LightModeDrone):
            header.to  = DeviceType.Drone
        elif isinstance(light_event, LightModeController):
            header.to  = DeviceType.Controller

        data = LightEventColor()

        data.event.event = light_event.value
        data.event.interval = interval
        data.event.repeat = repeat

        data.color.r = r
        data.color.g = g
        data.color.b = b

        return self.transfer(header, data)

    def send_light_event_colors(self, light_event: LightModeDrone | LightModeController, interval: int, repeat: int, colors: Colors):
        header = Header(
            DataType.LightEvent, LightEventColors.get_size(), DeviceType.Base
        )

        if isinstance(light_event, LightModeDrone):
            header.to = DeviceType.Drone
        elif isinstance(light_event, LightModeController):
            header.to = DeviceType.Controller

        data = LightEventColors()

        data.event.event = light_event.value
        data.event.interval = interval
        data.event.repeat = repeat

        data.colors = colors

        return self.transfer(header, data)

# Light End

# Buzzer Start

    def send_buzzer(self, mode: BuzzerMode, value: int, time: int):
        data = Buzzer()
        data.mode = mode
        data.value = value
        data.time = time
        return self.send(DataType.Buzzer, Buzzer.get_size(), DeviceType.Base, DeviceType.Controller, data)

    def send_buzzer_mute(self, time: int):
        data = Buzzer()
        data.mode = BuzzerMode.MuteInstantly
        data.value = BuzzerScale.Mute.value
        data.time = time
        return self.send(DataType.Buzzer, Buzzer.get_size(), DeviceType.Base, DeviceType.Controller, data)

    def send_buzzer_mute_reserve(self, time: int):
        data = Buzzer()
        data.mode = BuzzerMode.MuteContinually
        data.value = BuzzerScale.Mute.value
        data.time = time
        return self.send(DataType.Buzzer, Buzzer.get_size(), DeviceType.Base, DeviceType.Controller, data)

    def send_buzzer_scale(self, scale: BuzzerScale, time: int):
        data = Buzzer()
        data.mode = BuzzerMode.ScaleInstantly
        data.value = scale.value
        data.time = time
        return self.send(DataType.Buzzer, Buzzer.get_size(), DeviceType.Base, DeviceType.Controller, data)

    def send_buzzer_scale_reserve(self, scale: BuzzerScale, time: int):
        data = Buzzer()
        data.mode = BuzzerMode.ScaleContinually
        data.value = scale.value
        data.time = time
        return self.send(DataType.Buzzer, Buzzer.get_size(), DeviceType.Base, DeviceType.Controller, data)

    def send_buzzer_hz(self, hz: int, time: int):
        data = Buzzer()
        data.mode = BuzzerMode.HzInstantly
        data.value = hz
        data.time = time
        return self.send(DataType.Buzzer, Buzzer.get_size(), DeviceType.Base, DeviceType.Controller, data)

    def send_buzzer_hz_reserve(self, hz: int, time: int):
        data = Buzzer()
        data.mode = BuzzerMode.HzContinually
        data.value = hz
        data.time = time
        return self.send(DataType.Buzzer, Buzzer.get_size(), DeviceType.Base, DeviceType.Controller, data)

# Buzzer End
