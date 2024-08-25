from sys import stdout
from os import linesep

print = stdout.write

def print_flush(s: str):
    print(s)
    stdout.flush()

def println(s: str = ""):
    print(f"{s}{linesep}")
