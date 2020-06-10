# Define MACROS
TITLE = ""
ENDC = "\x1b[0m"
BLINK = "\33[5m"
BLUE = "\033[44m"
ERROR = "\33[31m[ERROR] "
RANGE = "\33[94m ~ " + ENDC
WARNING = "\33[33m [WARNING]"
MX64 = "\33[35m[MX-64]" + ENDC
AX12 = "\33[32m[AX-12]" + ENDC
INFO = "\33[41m" + "[INFO]" + ENDC + " "
KEY = "\x1b[1;37;42m" + "Press any key to terminate" + ENDC
WARN_SIGN = "\33[5m\x1b[2;30;43m ! " + ENDC + "\33[33m Warning: " + ENDC

def printC (typeC, text=""):
    if typeC == BLINK or typeC == WARNING or typeC == ERROR:
        print typeC + text + "\x1b[0m"
    elif typeC == MX64 or typeC == AX12:
        print typeC,
    elif typeC == RANGE:
        print text.split(" ")[0] + RANGE + text.split(" ")[1]
    elif typeC == TITLE:
        print("\n" + BLUE + "            " + text + "             " + ENDC + "\n")
    else:
        print typeC + text