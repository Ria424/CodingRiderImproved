from os import linesep
from sys import stdout

def print_flush(s: str):
    stdout.write(s)
    stdout.flush()

def println(s: str):
    stdout.write(f"{s}{linesep}")

def print_newline():
    stdout.write(linesep)