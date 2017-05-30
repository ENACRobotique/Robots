'''
EPSON Esc/pos command list
To use the digipos, just create a Serial object, and write these bytes to it.
The serial port will be something like "/dev/ttyUSB0".
'''

mvr = bytearray.fromhex("09")   # Move cursor to right
mvl = bytearray.fromhex("08")   # Move cursor to left
mvu = bytearray.fromhex("1f0a")   # Move cursor up
mvd = bytearray.fromhex("0a")   # Move cursor down
mvrm = bytearray.fromhex("1f0d")   # Move cursor to right-most position
mvlm = bytearray.fromhex("0d")   # Move cursor to left-most position
mvh = bytearray.fromhex("0b")   # Move cursor to home position
mvb = bytearray.fromhex("1f42")   # Move cursor to bottom position
clr = bytearray.fromhex("0c")   # Clear screen
init = bytearray.fromhex("1b40") # initialize display

def mvp(x:int,y:int): # Move cursor to position (x y)
    return = bytearray.fromhex("1f24") + bytes((x,y))

def blink(period:int):
    return bytearray.fromhex("1f45") + bytes((period,))

def text(string):
    return bytes(string,'utf-8')
