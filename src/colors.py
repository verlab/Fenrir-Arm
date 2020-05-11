# Define MACROS
INFO = "\33[41m" + "[INFO]" + "\x1b[0m "
KEY = "\x1b[1;37;42m" + "Press any key to terminate" + "\x1b[0m"
BLINK = "\33[5m"
MX64 = "\33[35m[MX-64]\x1b[0m"
AX12 = "\33[32m[AX-12]\x1b[0m"
WARNING = "\33[33m [WARNING]"
ERROR = "\33[31m [ERROR]"
RANGE = "\33[94m ~ \x1b[0m"

def printC (typeC, text=""):
    if typeC == BLINK or typeC == WARNING or typeC == ERROR:
        print typeC + text + "\x1b[0m"
    elif typeC == MX64 or typeC == AX12:
        print typeC,
    elif typeC == RANGE:
        print text.split(" ")[0] + RANGE + text.split(" ")[1]
    else:
        print typeC + text