from sys import argv
from sys import exit as sys_exit
from time import sleep

from colorama import Fore, Style
from colorama import init as colorama_init
from serial.tools.list_ports import comports

from ..drone import Drone
from ..protocol import (Buzzer, BuzzerMode, CommandType, DataType, DeviceType,
                        LightModeDrone, State)
from ..system import FlightEvent

def run():
    args = argv[1:]
    count = len(args)

    if count == 0:
        return

    colorama_init()

    match args[0]:
        # python -m CodingRiderImproved upgrade
        # python -m CodingRiderImproved update
        # case "update" | "upgrade":
        #     updater = Updater()
        #     updater.update()

        case "request" if count == 4:
            match args[1]:
                # python -m CodingRiderImproved request state 10 0.2
                # "* State - Ready     Blue    Red     Black   Black   None_             1830   1631   2230      76      "
                case "state":
                    print("         |ModeSystem   |ModeFlight |ModeControlFlight |ModeMovement |Headless |ControlSpeed |SensorOrientation |Battery |")
                    request(DeviceType.Drone, DataType.State, int(args[2]), float(args[3]))

                # python -m CodingRiderImproved request Motion 10 0.2
                # "* Motion      -38      64     -32     457     -40    -400      16       0   20500"
                case "motion":
                    print("         |Accel                  |Gyro                   |Angle                  |")
                    print("         |      X|      Y|      Z|   Roll|  Pitch|    Yaw|   Roll|  Pitch|    Yaw|")
                    request(DeviceType.Drone, DataType.Motion, int(args[2]), float(args[3]))

        # python -m CodingRiderImproved takeoff
        case "takeoff":
            print(Fore.YELLOW + "takeoff" + Style.RESET_ALL)
            command(CommandType.FlightEvent, FlightEvent.Takeoff.value)

        # python -m CodingRiderImproved landing
        case "landing":
            print(Fore.YELLOW + "landing" + Style.RESET_ALL)
            command(CommandType.FlightEvent, FlightEvent.Landing.value)

        # python -m CodingRiderImproved stop
        case "stop":
            print(Fore.YELLOW + "stop" + Style.RESET_ALL)
            command(CommandType.FlightEvent, FlightEvent.Stop.value)

        # 조종
        # 지정한 시간이 종료되면 입력값을 모두 0으로 변경하고 멈춤
        # python -m CodingRiderImproved control <roll> <pitch> <yaw> <throttle> <time(ms)>
        # python -m CodingRiderImproved control 40 0 3
        case "control" if count == 6:
            print(Fore.YELLOW + "control" + Style.RESET_ALL)
            control(*map(int, args[1:6]))

        # 이동
        # python -m CodingRiderImproved position <x> <y> <z> <velocity> [heading] [rotational velocity]
        case "position":
            print(Fore.YELLOW + "position" + Style.RESET_ALL)
            match count:
                case 7:
                    controlPosition(*map(float, args[1:5]), *map(int, args[5:7]))
                case 5:
                    controlPosition(*map(float, args[1:5]), 0, 0)

        # 이동(헤딩)
        # python -m CodingRiderImproved heading [heading] [rotational velocity]
        case "heading" if count == 3:
            print(Fore.YELLOW + "heading" + Style.RESET_ALL)
            controlPosition(0, 0, 0, 0, *map(int, args[1:3]))

        # 버저
        # python -m CodingRiderImproved buzzer [hz] [time(ms)]
        # python -m CodingRiderImproved buzzer 400 2000
        case "buzzer" if count == 3:
            print(Fore.WHITE + "Buzz Sound: " + Fore.YELLOW + args[1] + Fore.WHITE + "Hz, " + Fore.CYAN + args[2] + Fore.WHITE + "ms" + Style.RESET_ALL)
            buzzer(DeviceType.Controller, *map(int, args[1:3]))

        # python -m CodingRiderImproved light body flicker 100 50 50 10
        # python -m CodingRiderImproved light body flickerdouble 100 50 50 10
        # python -m CodingRiderImproved light body dimming 3 50 50 10
        # python -m CodingRiderImproved light body sunrise 5 50 50 10
        # python -m CodingRiderImproved light body sunset 5 50 50 10
        # python -m CodingRiderImproved light body rainbow 8 50 50 10
        # python -m CodingRiderImproved light body rainbow2 8 50 50 10
        case "light" if count == 7:
            print(
                Fore.WHITE
                + "Light: "
                + Fore.YELLOW
                + "{0}, {1}, {2}, ({3}, {4}, {5})".format(
                    args[1], args[2], *map(int, args[3:7])
                )
                + Style.RESET_ALL
            )
            lightModeRgb(args[1], args[2], *map(int, args[3:7]))

        case _:
            help()

def open_drone():
    drone = Drone()
    if not drone.open(
        tuple(filter(
            lambda port_info: port_info.description.find("CH340") != -1,
            comports()
        ))[0].device
    ):
        print(Fore.RED + "* Error : Unable to open serial port." + Style.RESET_ALL)
        sys_exit(1)
    return drone

def request(device_type: DeviceType, data_type: DataType, repeat: int, interval: float):
    drone = open_drone()

    # 이벤트 핸들링 함수 등록
    drone.set_event_handler(DataType.State, eventState)
    drone.set_event_handler(DataType.Motion, eventMotion)

    # 데이터 요청
    for _ in range(repeat):
        drone.send_request(device_type, data_type)
        sleep(interval)

def command(command_type: CommandType, option: int = 0):
    # 데이터 요청
    open_drone().send_command(DeviceType.Base, DeviceType.Drone, command_type, option)

def control(roll: int, pitch: int, yaw: int, throttle: int, time_ms: int):
    drone = open_drone()

    # 데이터 요청
    drone.send_control_while(roll, pitch, yaw, throttle, time_ms)
    drone.send_control_while(0, 0, 0, 0, 200)

def controlPosition(x: float, y: float, z: float, velocity: float, heading: int, rotational_velocity: int):
    # 데이터 요청
    open_drone().send_control_position(x, y, z, velocity, heading, rotational_velocity)
    sleep(0.1)

def lightModeRgb(strLightPart: str, strLightMode: str, interval: int, r: int, g: int, b: int):
    drone = open_drone()

    lightModeHigh = LightModeDrone.None_

    if strLightPart == "body":
        lightModeHigh = LightModeDrone.BodyNone

    lightModeLow = LightModeDrone.None_
    match strLightMode:
        case "hold":
            lightModeLow = LightModeDrone.BodyHold
        case "flicker":
            lightModeLow = LightModeDrone.BodyFlicker
        case "flickerdouble":
            lightModeLow = LightModeDrone.BodyFlickerDouble
        case "dimming":
            lightModeLow = LightModeDrone.BodyDimming
        case "sunrise":
            lightModeLow = LightModeDrone.BodySunrise
        case "sunset":
            lightModeLow = LightModeDrone.BodySunset
        case "rainbow":
            lightModeLow = LightModeDrone.BodyRainbow
        case "rainbow2":
            lightModeLow = LightModeDrone.BodyRainbow2

    lightMode = LightModeDrone(lightModeHigh.value + ((lightModeLow.value) & 0x0F))

    if lightModeHigh != LightModeDrone.None_ and lightModeLow != LightModeDrone.None_:
        drone.send_light_mode_color(lightMode, interval, r, g, b)

def buzzer(target: DeviceType, hz: int, time: int):
    drone = open_drone()

    data = Buzzer()
    data.mode = BuzzerMode.HzInstantly
    data.value = hz
    data.time = time
    drone.send(DataType.Buzzer, Buzzer.get_size(), DeviceType.Base, target, data)

    sleep(time / 1000)

def help():
    print()
    print(Fore.YELLOW + "* Command List " + Style.RESET_ALL)

    # print()
    # print(Fore.CYAN + "  - Firmware Upgrade" + Style.RESET_ALL)
    # print(Fore.GREEN + "   > " + Fore.WHITE + "python -m CodingRiderImproved " + Fore.CYAN + "upgrade" + Style.RESET_ALL)

    print()
    print(Fore.CYAN + "  - Request Data" + Style.RESET_ALL)
    print(Fore.GREEN + "   > " + Fore.WHITE + "python -m CodingRiderImproved " + Fore.CYAN + "request " + Fore.WHITE + "[" + Fore.YELLOW + "data type" + Fore.WHITE + "] [" + Fore.GREEN + "number of times" + Fore.WHITE + "] [" + Fore.YELLOW + "time interval(sec)" + Fore.WHITE + "]" + Style.RESET_ALL)
    print(Fore.GREEN + "   > " + Fore.WHITE + "python -m CodingRiderImproved " + Fore.CYAN + "request " + Fore.YELLOW + "state " + Fore.GREEN + "10 " + Fore.YELLOW + "0.2" + Style.RESET_ALL)
    print(Fore.GREEN + "   > " + Fore.WHITE + "python -m CodingRiderImproved " + Fore.CYAN + "request " + Fore.YELLOW + "motion " + Fore.GREEN + "10 " + Fore.YELLOW + "0.2" + Style.RESET_ALL)
    ## 카드 코딩과 관련된 내용이 최신 버전인지 알 수 없는 관계로 일단 보류(2021.1.4)
    ##print(Fore.GREEN + "   > " + Fore.WHITE + "python -m CodingRiderImproved " + Fore.CYAN + "request " + Fore.YELLOW + "RawCard " + Fore.GREEN + "10 " + Fore.YELLOW + "0.2" + Style.RESET_ALL)
    ##print(Fore.GREEN + "   > " + Fore.WHITE + "python -m CodingRiderImproved " + Fore.CYAN + "request " + Fore.YELLOW + "RawCardRange " + Fore.GREEN + "10 " + Fore.YELLOW + "0.2" + Style.RESET_ALL)

    print()
    print(Fore.CYAN + "  - FlightEvent" + Style.RESET_ALL)
    print(Fore.GREEN + "   > " + Fore.WHITE + "python -m CodingRiderImproved " + Fore.WHITE + "[" + Fore.YELLOW + "FlightEvent" + Fore.WHITE + "]" + Style.RESET_ALL)
    print(Fore.GREEN + "   > " + Fore.WHITE + "python -m CodingRiderImproved " + Fore.YELLOW + "takeoff" + Style.RESET_ALL)
    print(Fore.GREEN + "   > " + Fore.WHITE + "python -m CodingRiderImproved " + Fore.YELLOW + "landing" + Style.RESET_ALL)
    print(Fore.GREEN + "   > " + Fore.WHITE + "python -m CodingRiderImproved " + Fore.YELLOW + "stop" + Style.RESET_ALL)

    print()
    print(Fore.CYAN + "  - Control" + Style.RESET_ALL)
    print(Fore.GREEN + "   > " + Fore.WHITE + "python -m CodingRiderImproved " + Fore.CYAN + "control " + Fore.WHITE + "[" + Fore.RED + "roll" + Fore.WHITE + "] [" + Fore.GREEN + "pitch" + Fore.WHITE + "] [" + Fore.BLUE + "yaw" + Fore.WHITE + "] [" + Fore.MAGENTA + "throttle" + Fore.WHITE + "] [" + Fore.YELLOW + "time(ms)" + Fore.WHITE + "]" + Style.RESET_ALL)
    print(Fore.GREEN + "   > " + Fore.WHITE + "python -m CodingRiderImproved " + Fore.CYAN + "control " + Fore.RED + "0 " + Fore.GREEN + "30 " + Fore.BLUE + "0 " + Fore.MAGENTA + "0 " + Fore.YELLOW + "5000 " + Style.RESET_ALL)

    print()
    print(Fore.CYAN + "  - Control - Position, Heading" + Style.RESET_ALL)
    print(Fore.GREEN + "   > " + Fore.WHITE + "python -m CodingRiderImproved " + Fore.CYAN + "position " + Fore.WHITE + "[" + Fore.RED + "x(meter)" + Fore.WHITE + "] [" + Fore.GREEN + "y(meter)" + Fore.WHITE + "] [" + Fore.BLUE + "z(meter)" + Fore.WHITE + "] [" + Fore.YELLOW + "speed(m/sec)" + Fore.WHITE + "] [" + Fore.MAGENTA + "heading(degree)" + Fore.WHITE + "] [" + Fore.YELLOW + "rotational velocity(deg/sec)" + Fore.WHITE + "]" + Style.RESET_ALL)
    print(Fore.GREEN + "   > " + Fore.WHITE + "python -m CodingRiderImproved " + Fore.CYAN + "position " + Fore.RED + "5 " + Fore.GREEN + "0 " + Fore.BLUE + "0 "  + Fore.YELLOW + "2 " + Fore.MAGENTA + "90 " + Fore.YELLOW + "45 "+ Style.RESET_ALL)

    print()
    print(Fore.CYAN + "  - Control - Position" + Style.RESET_ALL)
    print(Fore.GREEN + "   > " + Fore.WHITE + "python -m CodingRiderImproved " + Fore.CYAN + "position " + Fore.WHITE + "[" + Fore.RED + "x(meter)" + Fore.WHITE + "] [" + Fore.GREEN + "y(meter)" + Fore.WHITE + "] [" + Fore.BLUE + "z(meter)" + Fore.WHITE + "] [" + Fore.YELLOW + "speed(m/sec)" + Fore.WHITE + "]" + Style.RESET_ALL)
    print(Fore.GREEN + "   > " + Fore.WHITE + "python -m CodingRiderImproved " + Fore.CYAN + "position " + Fore.RED + "5 " + Fore.GREEN + "0 " + Fore.BLUE + "0 "  + Fore.YELLOW + "2 " + Style.RESET_ALL)

    print()
    print(Fore.CYAN + "  - Control - Heading" + Style.RESET_ALL)
    print(Fore.GREEN + "   > " + Fore.WHITE + "python -m CodingRiderImproved " + Fore.CYAN + "heading " + Fore.WHITE + "[" + Fore.MAGENTA + "heading(degree)" + Fore.WHITE + "] [" + Fore.YELLOW + "rotational velocity(deg/sec)" + Fore.WHITE + "]" + Style.RESET_ALL)
    print(Fore.GREEN + "   > " + Fore.WHITE + "python -m CodingRiderImproved " + Fore.CYAN + "heading " + Fore.MAGENTA + "90 " + Fore.YELLOW + "45" + Style.RESET_ALL)

    print()
    print(Fore.CYAN + "  - Buzzer" + Style.RESET_ALL)
    print(Fore.GREEN + "   > " + Fore.WHITE + "python -m CodingRiderImproved " + Fore.CYAN + "buzzer " + Fore.WHITE + "[" + Fore.YELLOW + "hz" + Fore.WHITE + "] [" + Fore.GREEN + "time(ms)" + Fore.WHITE + "]" + Style.RESET_ALL)
    print(Fore.GREEN + "   > " + Fore.WHITE + "python -m CodingRiderImproved " + Fore.CYAN + "buzzer " + Fore.YELLOW + "400 " + Fore.GREEN + "2000" + Style.RESET_ALL)

    print()
    print(Fore.CYAN + "  - Vibrator" + Style.RESET_ALL)
    print(Fore.GREEN + "   > " + Fore.WHITE + "python -m CodingRiderImproved " + Fore.CYAN + "vibrator " + Fore.WHITE + "[" + Fore.YELLOW + "on(ms)" + Fore.WHITE + "] [" + Fore.GREEN + "off(ms)" + Fore.WHITE + "] [" + Fore.YELLOW + "total(ms)" + Fore.WHITE + "]" + Style.RESET_ALL)
    print(Fore.GREEN + "   > " + Fore.WHITE + "python -m CodingRiderImproved " + Fore.CYAN + "vibrator " + Fore.YELLOW + "500 " + Fore.GREEN + "500 " + Fore.YELLOW + "2000" + Style.RESET_ALL)

    print()
    print(Fore.CYAN + "  - Light single" + Style.RESET_ALL)
    print(Fore.GREEN + "   > " + Fore.WHITE + "python -m CodingRiderImproved " + Fore.CYAN + "light " + Fore.WHITE + "[" + Fore.MAGENTA + "part" + Fore.WHITE + "] [" + Fore.CYAN + "mode" + Fore.WHITE + "] [" + Fore.YELLOW + "interval" + Fore.WHITE + "]" + Style.RESET_ALL)
    print(Fore.GREEN + "   > " + Fore.WHITE + "python -m CodingRiderImproved " + Fore.CYAN + "light " + Fore.MAGENTA + "rear " + Fore.CYAN + "hold " + Fore.YELLOW + "100" + Style.RESET_ALL)
    print(Fore.GREEN + "   > " + Fore.WHITE + "python -m CodingRiderImproved " + Fore.CYAN + "light " + Fore.MAGENTA + "a " + Fore.CYAN + "hold " + Fore.YELLOW + "100" + Style.RESET_ALL)
    print(Fore.GREEN + "   > " + Fore.WHITE + "python -m CodingRiderImproved " + Fore.CYAN + "light " + Fore.MAGENTA + "b " + Fore.CYAN + "hold " + Fore.YELLOW + "100" + Style.RESET_ALL)

    print()
    print(Fore.CYAN + "  - Light RGB" + Style.RESET_ALL)
    print(Fore.GREEN + "   > " + Fore.WHITE + "python -m CodingRiderImproved " + Fore.CYAN + "light " + Fore.WHITE + "[" + Fore.MAGENTA + "part" + Fore.WHITE + "] [" + Fore.CYAN + "mode" + Fore.WHITE + "] [" + Fore.YELLOW + "interval" + Fore.WHITE + "] [" + Fore.RED + "R" + Fore.WHITE + "] [" + Fore.GREEN + "G" + Fore.WHITE + "] [" + Fore.BLUE + "B" + Fore.WHITE + "] " + Style.RESET_ALL)
    print(Fore.GREEN + "   > " + Fore.WHITE + "python -m CodingRiderImproved " + Fore.CYAN + "light " + Fore.MAGENTA + "body " + Fore.CYAN + "hold " + Fore.YELLOW + "100 " + Fore.RED + "50 " + Fore.GREEN + "50 " + Fore.BLUE + "10" + Style.RESET_ALL)
    print(Fore.GREEN + "   > " + Fore.WHITE + "python -m CodingRiderImproved " + Fore.CYAN + "light " + Fore.MAGENTA + "body " + Fore.CYAN + "flicker " + Fore.YELLOW + "100 " + Fore.RED + "50 " + Fore.GREEN + "50 " + Fore.BLUE + "10" + Style.RESET_ALL)
    print(Fore.GREEN + "   > " + Fore.WHITE + "python -m CodingRiderImproved " + Fore.CYAN + "light " + Fore.MAGENTA + "body " + Fore.CYAN + "flickerdouble " + Fore.YELLOW + "100 " + Fore.RED + "50 " + Fore.GREEN + "50 " + Fore.BLUE + "10" + Style.RESET_ALL)
    print(Fore.GREEN + "   > " + Fore.WHITE + "python -m CodingRiderImproved " + Fore.CYAN + "light " + Fore.MAGENTA + "body " + Fore.CYAN + "dimming " + Fore.YELLOW + "3 " + Fore.RED + "50 " + Fore.GREEN + "50 " + Fore.BLUE + "10" + Style.RESET_ALL)
    print(Fore.GREEN + "   > " + Fore.WHITE + "python -m CodingRiderImproved " + Fore.CYAN + "light " + Fore.MAGENTA + "body " + Fore.CYAN + "sunrise " + Fore.YELLOW + "5 " + Fore.RED + "50 " + Fore.GREEN + "50 " + Fore.BLUE + "10" + Style.RESET_ALL)
    print(Fore.GREEN + "   > " + Fore.WHITE + "python -m CodingRiderImproved " + Fore.CYAN + "light " + Fore.MAGENTA + "body " + Fore.CYAN + "sunset " + Fore.YELLOW + "5 " + Fore.RED + "50 " + Fore.GREEN + "50 " + Fore.BLUE + "10" + Style.RESET_ALL)
    print(Fore.GREEN + "   > " + Fore.WHITE + "python -m CodingRiderImproved " + Fore.CYAN + "light " + Fore.MAGENTA + "body " + Fore.CYAN + "rainbow " + Fore.YELLOW + "8 " + Fore.RED + "50 " + Fore.GREEN + "50 " + Fore.BLUE + "10" + Style.RESET_ALL)
    print(Fore.GREEN + "   > " + Fore.WHITE + "python -m CodingRiderImproved " + Fore.CYAN + "light " + Fore.MAGENTA + "body " + Fore.CYAN + "rainbow2 " + Fore.YELLOW + "8 " + Fore.RED + "50 " + Fore.GREEN + "50 " + Fore.BLUE + "10" + Style.RESET_ALL)
    print()

def eventState(state: State):
    print(  "* State   " +
            Fore.YELLOW + "{0:12}  ".format(state.mode_system.name) +
            Fore.YELLOW + "{0:10}  ".format(state.mode_flight.name) +
            Fore.WHITE + "{0:17}  ".format(state.mode_control_flight    .name) +
            Fore.WHITE + "{0:12}  ".format(state.mode_movement.name) +
            Fore.WHITE + "{0:8}  ".format(state.headless.name) +
            Fore.CYAN + "{0:12}  ".format(state.control_speed) +
            Fore.CYAN + "{0:17}  ".format(state.sensor_orientation.name) +
            Fore.GREEN + "{0:7}".format(state.battery) + Style.RESET_ALL)

def eventMotion(motion):
    print(  "* Motion " +
            Fore.YELLOW + "{0:8}".format(motion.accelX) +
            Fore.YELLOW + "{0:8}".format(motion.accelY) +
            Fore.YELLOW + "{0:8}".format(motion.accelZ) +
            Fore.WHITE + "{0:8}".format(motion.gyroRoll) +
            Fore.WHITE + "{0:8}".format(motion.gyroPitch) +
            Fore.WHITE + "{0:8}".format(motion.gyroYaw) +
            Fore.CYAN + "{0:8}".format(motion.angleRoll) +
            Fore.CYAN + "{0:8}".format(motion.anglePitch) +
            Fore.CYAN + "{0:8}".format(motion.angleYaw) + Style.RESET_ALL)

def eventCardRange(cardRange):
    print(  "* RawCardRange " +
            Fore.RED    + "{0:6}".format(cardRange.range[0][0][0]) +
            Fore.RED    + "{0:6}".format(cardRange.range[0][0][1]) +
            Fore.GREEN  + "{0:6}".format(cardRange.range[0][1][0]) +
            Fore.GREEN  + "{0:6}".format(cardRange.range[0][1][1]) +
            Fore.BLUE   + "{0:6}".format(cardRange.range[0][2][0]) +
            Fore.BLUE   + "{0:6}".format(cardRange.range[0][2][1]) +
            Fore.RED    + "{0:6}".format(cardRange.range[1][0][0]) +
            Fore.RED    + "{0:6}".format(cardRange.range[1][0][1]) +
            Fore.GREEN  + "{0:6}".format(cardRange.range[1][1][0]) +
            Fore.GREEN  + "{0:6}".format(cardRange.range[1][1][1]) +
            Fore.BLUE   + "{0:6}".format(cardRange.range[1][2][0]) +
            Fore.BLUE   + "{0:6}".format(cardRange.range[1][2][1]) + Style.RESET_ALL)

def eventCardRaw(cardRaw):
    print(  "* RawCard " +
            Fore.RED    + "{0:5}".format(cardRaw.rgbRaw[0][0]) +
            Fore.GREEN  + "{0:5}".format(cardRaw.rgbRaw[0][1]) +
            Fore.BLUE   + "{0:5}".format(cardRaw.rgbRaw[0][2]) +
            Fore.RED    + "{0:5}".format(cardRaw.rgbRaw[1][0]) +
            Fore.GREEN  + "{0:5}".format(cardRaw.rgbRaw[1][1]) +
            Fore.BLUE   + "{0:5}".format(cardRaw.rgbRaw[1][2]) +
            Fore.RED    + "{0:4}".format(cardRaw.rgb[0][0]) +
            Fore.GREEN  + "{0:4}".format(cardRaw.rgb[0][1]) +
            Fore.BLUE   + "{0:4}".format(cardRaw.rgb[0][2]) +
            Fore.RED    + "{0:4}".format(cardRaw.rgb[1][0]) +
            Fore.GREEN  + "{0:4}".format(cardRaw.rgb[1][1]) +
            Fore.BLUE   + "{0:4}".format(cardRaw.rgb[1][2]) +
            Fore.RED    + "{0:4}".format(cardRaw.hsvl[0][0]) +
            Fore.GREEN  + "{0:4}".format(cardRaw.hsvl[0][1]) +
            Fore.BLUE   + "{0:4}".format(cardRaw.hsvl[0][2]) +
            Fore.BLUE   + "{0:4}".format(cardRaw.hsvl[0][3]) +
            Fore.RED    + "{0:4}".format(cardRaw.hsvl[1][0]) +
            Fore.GREEN  + "{0:4}".format(cardRaw.hsvl[1][1]) +
            Fore.BLUE   + "{0:4}".format(cardRaw.hsvl[1][2]) +
            Fore.BLUE   + "{0:4}".format(cardRaw.hsvl[1][3]) +
            Fore.CYAN   + "{0:14}".format(cardRaw.color[0].name) +
            Fore.CYAN   + "{0:14}".format(cardRaw.color[1].name) +
            Fore.CYAN   + "{0:14}".format(cardRaw.card.name) + Style.RESET_ALL)
