from os import linesep
from sys import argv
from sys import exit as sys_exit
from time import sleep
from typing import NotRequired, TypedDict

from colorama import Fore, Style
from colorama import init as colorama_init
from serial.tools.list_ports import comports

from ._wrapped import println
from .drone import Drone
from .protocol import (Buzzer, BuzzerMode, CommandType, DataType, DeviceType,
                       LightModeDrone, Motion, State)
from .system import FlightEvent

def _println_style(s: str):
    println(f"{s}{Style.RESET_ALL}")

def run():
    args = argv[1:]
    count = len(args)

    if count == 0:
        return

    colorama_init()

    match args[0]:
        case "request" if count == 2 or count == 4:
            if count == 2:
                repeat = 1
                interval = 1
            else:
                repeat = int(args[2])
                interval = float(args[3])
            match args[1]:
                case "motion":
                    request(
                        f" Accel{"":18}|Gyro{"":19}|Angle{"":18}|{linesep}"
                        f"       X|      Y|      Z|   Roll|  Pitch|    Yaw|   Roll|  Pitch|    Yaw|",
                        DeviceType.Drone, DataType.Motion,
                        repeat, interval
                    )

                case "state":
                    request(
                        f"ModeSystem   |ModeFlight |ModeControlFlight |ModeMovement |Headless |ControlSpeed |SensorOrientation |Battery",
                        DeviceType.Drone, DataType.State,
                        repeat, interval
                    )

        case "takeoff":
            _println_style(f"{Fore.YELLOW}takeoff")
            command(CommandType.FlightEvent, FlightEvent.Takeoff.value)

        case "landing":
            _println_style(f"{Fore.YELLOW}landing")
            command(CommandType.FlightEvent, FlightEvent.Landing.value)

        case "stop":
            _println_style(f"{Fore.YELLOW}stop")
            command(CommandType.FlightEvent, FlightEvent.Stop.value)

        case "control" if count == 6:
            _println_style(f"{Fore.YELLOW}control")
            control(*map(int, args[1:6]))

        # python -m CodingRiderImproved position <x> <y> <z> <velocity> [heading] [rotational velocity]
        case "position":
            _println_style(f"{Fore.YELLOW}position")
            match count:
                case 7:
                    control_position(*map(float, args[1:5]), *map(int, args[5:7]))
                case 5:
                    control_position(*map(float, args[1:5]), 0, 0)

        # python -m CodingRiderImproved heading <heading> <rotational velocity>
        case "heading" if count == 3:
            _println_style(f"{Fore.YELLOW}heading")
            control_position(0, 0, 0, 0, *map(int, args[1:3]))

        case "buzzer" if count == 3:
            print(Fore.WHITE + "Buzz Sound: " + Fore.YELLOW + args[1] + Fore.WHITE + "Hz, " + Fore.CYAN + args[2] + Fore.WHITE + "ms" + Style.RESET_ALL)
            buzzer(DeviceType.Controller, *map(int, args[1:3]))

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
            light_mode_rgb(args[1], args[2], *map(int, args[3:7]))

        case _:
            help()

def open_drone():
    connected_ports = tuple(
        filter(
            lambda port_info: port_info.description.find("CH340") != -1,
            comports()
        )
    )

    if not connected_ports:
        _println_style(f"{Fore.RED}Error: Unable to open serial port.")
        sys_exit(1)

    drone = Drone()
    if not drone.open(connected_ports[0].device):
        _println_style(f"{Fore.RED}Error: Unable to open serial port.")
        sys_exit(1)
    return drone

def request(
        message: str, device_type: DeviceType, data_type: DataType,
        repeat: int, interval: float):
    drone = open_drone()

    println(message)

    # 이벤트 핸들링 함수 등록
    drone.set_event_handler(DataType.State, event_state)
    drone.set_event_handler(DataType.Motion, event_motion)

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

def control_position(x: float, y: float, z: float, velocity: float, heading: int, rotational_velocity: int):
    # 데이터 요청
    open_drone().send_control_position(x, y, z, velocity, heading, rotational_velocity)
    sleep(0.1)

def light_mode_rgb(light_part: str, light_mode: str, interval: int, r: int, g: int, b: int):
    drone = open_drone()

    light_mode_high = LightModeDrone.None_

    if light_part == "body":
        light_mode_high = LightModeDrone.BodyNone

    light_mode_low = LightModeDrone.None_
    match light_mode:
        case "hold":
            light_mode_low = LightModeDrone.BodyHold
        case "flicker":
            light_mode_low = LightModeDrone.BodyFlicker
        case "flickerdouble":
            light_mode_low = LightModeDrone.BodyFlickerDouble
        case "dimming":
            light_mode_low = LightModeDrone.BodyDimming
        case "sunrise":
            light_mode_low = LightModeDrone.BodySunrise
        case "sunset":
            light_mode_low = LightModeDrone.BodySunset
        case "rainbow":
            light_mode_low = LightModeDrone.BodyRainbow
        case "rainbow2":
            light_mode_low = LightModeDrone.BodyRainbow2

    if light_mode_high != LightModeDrone.None_ and light_mode_low != LightModeDrone.None_:
        drone.send_light_mode_color(
            LightModeDrone(
                light_mode_high.value + (light_mode_low.value & 0x0F)
            ), interval, r, g, b
        )

def buzzer(target: DeviceType, hz: int, time: int):
    drone = open_drone()

    data = Buzzer()
    data.mode = BuzzerMode.HzInstantly
    data.value = hz
    data.time = time
    drone.send(DataType.Buzzer, Buzzer.get_size(), DeviceType.Base, target, data)

    sleep(time / 1000)

class Command(TypedDict):
    colors: NotRequired[tuple[str, ...]]
    parameters: tuple[str, ...]
    examples: tuple[tuple[str, ...], ...]

def _get_command_prefix(name: str):
    return (
        f"    {Fore.GREEN}> {Fore.WHITE}python -m CodingRiderImproved "
        f"{Fore.YELLOW}{name} "
    )

def help():
    COLORS = (Fore.CYAN, Fore.GREEN, Fore.MAGENTA, Fore.BLUE, Fore.RED, Fore.YELLOW,)
    COMMAND_GROUPS: dict[str, dict[str, Command]] = {
        "Request data": {
            "request": {
                "parameters": (
                    "motion or state",
                    "number of times",
                    "time interval(sec)",
                ),
                "examples": (
                    ("motion", "10", "0.2",),
                    ("state", "10", "0.2",),
                    ("motion",),
                    ("state",),
                )
            }
        },
        "Flight event": {
            "takeoff": {
                "parameters": (),
                "examples": ()
            },
            "landing": {
                "parameters": (),
                "examples": ()
            },
            "stop": {
                "parameters": (),
                "examples": ()
            }
        },
        "Control": {
            "control": {
                "parameters": (
                    "roll",
                    "pitch",
                    "yaw",
                    "throttle",
                    "time(ms)",
                ),
                "examples": (
                    ("0", "30", "0", "0", "5000",),
                )
            }
        },
        "Control - position, heading": {
            "position": {
                "parameters": (
                    "x(meter)",
                    "y(meter)",
                    "z(meter)",
                    "speed(m/sec)",
                    "heading(degree)",
                    "rotational velocity(deg/sec)",
                ),
                "examples": (
                    ("5", "0", "0", "2", "90", "45",),
                )
            }
        },
        "Control - position": {
            "position": {
                "parameters": (
                    "x(meter)",
                    "y(meter)",
                    "z(meter)",
                    "speed(m/sec)",
                ),
                "examples": (
                    ("5", "0", "0", "2",),
                )
            }
        },
        "Control - heading": {
            "heading": {
                "parameters": (
                    "heading(degree)",
                    "rotational velocity(deg/sec)",
                ),
                "examples": (
                    ("90", "45",),
                )
            }
        },
        "Buzzer": {
            "buzzer": {
                "parameters": (
                    "hz",
                    "time(ms)",
                ),
                "examples": (
                    ("400", "2000",),
                )
            }
        },
        "Vibrator": {
            "vibrator": {
                "parameters": (
                    "on(ms)",
                    "off(ms)",
                    "total(ms)",
                ),
                "examples": (
                    ("500", "500", "2000",),
                )
            }
        },
        "Light": {
            "light": {
                "colors": (
                    *COLORS[:3], Fore.RED, Fore.GREEN, Fore.BLUE
                ),
                "parameters": (
                    "part",
                    "mode",
                    "interval",
                    "R",
                    "G",
                    "B",
                ),
                "examples": (
                    ("body", "hold", "100", "50", "50", "10"),
                    ("body", "flicker", "100", "50", "50", "10"),
                    ("body", "flickerdouble", "100", "50", "50", "10"),
                    ("body", "dimming", "100", "50", "50", "10"),
                    ("body", "sunrise", "100", "50", "50", "10"),
                    ("body", "sunset", "100", "50", "50", "10"),
                    ("body", "rainbow", "100", "50", "50", "10"),
                    ("body", "rainbow2", "100", "50", "50", "10"),
                )
            }
        }
    }

    _println_style(f"{linesep}{Fore.YELLOW}Command List:")

    for group, commands in COMMAND_GROUPS.items():
        _println_style(f"  {Fore.CYAN}{group}:")
        for name, command in commands.items():
            colors = command.get("colors", COLORS)
            parameters = " ".join(map(
                lambda color, param: \
                    f"{Fore.WHITE}[{color}{param}{Fore.WHITE}]",
                colors,
                command["parameters"]
            ))
            _println_style(_get_command_prefix(name) + parameters)

            for example in command["examples"]:
                examples = " ".join(map(
                    lambda color, param: f"{color}{param}",
                    colors,
                    example
                ))
                _println_style(_get_command_prefix(name) + examples)
        println()

def event_state(state: State):
    _println_style(
        f"{Fore.YELLOW}{state.mode_system.name:12}  "
        f"{state.mode_flight.name:10}  "
        f"{Fore.WHITE}{state.mode_control_flight.name:17}  "
        f"{state.mode_movement.name:12}  "
        f"{state.headless.name:8}  "
        f"{Fore.CYAN}{state.control_speed:12}  "
        f"{state.sensor_orientation.name:17}  "
        f"{Fore.GREEN}{state.battery:7}"
    )

def event_motion(motion: Motion):
    _println_style(
        f"{Fore.YELLOW}{motion.accel_x:8}"
        f"{motion.accel_y:8}"
        f"{motion.accel_z:8}"
        f"{Fore.WHITE}{motion.gyro_roll:8}"
        f"{motion.gyro_pitch:8}"
        f"{motion.gyro_yaw:8}"
        f"{Fore.CYAN}{motion.angle_roll:8}"
        f"{motion.angle_pitch:8}"
        f"{motion.angle_yaw:8}"
    )

if __name__ == "__main__":
    run()