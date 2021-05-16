"""
Battery Evaluator Client

This code is garbage, do not replicate for your own good.

Yours truly,
Adin Ackerman
"""

import threading
from concurrent.futures import ThreadPoolExecutor
import serial
from serial.tools import list_ports
from dataclasses import dataclass
from tkinter import *
from tkinter.ttk import *
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
from queue import Queue
from time import sleep, time, time_ns
from datetime import timedelta
from typing import *

COMMAND_DICT = {
    'batt-voltage': 0x10,
    'current': 0x11,
    # 'temp1': 0x12,
    'temp2': 0x13,
    'set-current': 0x20,
}

BATT_CHEMS = {
    'Lithium': (3.5, 4.1),
    'Alkaline': (1.2, 1.5),
    'Custom': (0, 0)
}

tasksReady = threading.Condition()
taskQueue = Queue()


@dataclass
class TaskResult:
    value: Any = None


def ioTask(f):
    def inner(*args):
        condition = threading.Condition()
        result = TaskResult()
        with tasksReady:
            tasksReady.notify()

        taskQueue.put((result, condition, lambda: f(*args)))

        with condition:
            condition.wait()

        return result.value

    return inner


def ioLoop():
    while True:
        with tasksReady:
            tasksReady.wait()

        while not taskQueue.empty():
            r, c, task = taskQueue.get()
            r.value = task()
            with c:
                c.notify()


class SerCom:
    def __init__(self):
        self.deviceList = [str(d.name) for d in list_ports.comports()]
        self.lock = threading.Lock()

    def connect(self, comport: str) -> None:
        self.com = serial.Serial(comport, 9600)

    def close(self):
        self.com.close()

    def __readExactly(self, size: int) -> bytearray:
        buf = bytearray()
        with self.lock:
            while len(buf) < size:
                buf += self.com.read(size - len(buf))

        return buf

    @property
    @ioTask
    def battVoltage(self) -> float:
        self.com.write(bytes([COMMAND_DICT['batt-voltage'], 0]))
        msgLen = int.from_bytes(self.com.read(1), 'big')
        battv = self.__readExactly(msgLen).decode()
        battv = round(float(battv), 3)

        return battv

    @property
    @ioTask
    def battCurrent(self) -> float:
        self.com.write(bytes([COMMAND_DICT['current'], 0]))
        msgLen = int.from_bytes(self.com.read(1), 'big')
        current = self.__readExactly(msgLen).decode()
        current = int(round(float(current)*1000, 3))  # mA

        return current

    @battCurrent.setter
    @ioTask
    def battCurrent(self, value: int) -> None:
        msg = int.to_bytes(value, 2, 'big')
        self.com.write(bytes([COMMAND_DICT['set-current'], 2]))
        self.com.write(msg)

    @property
    @ioTask
    def resistorTemp(self) -> float:
        self.com.write(bytes([COMMAND_DICT['temp1'], 0]))
        msgLen = int.from_bytes(self.com.read(1), 'big')
        temp = self.__readExactly(msgLen).decode()
        temp = round(float(temp), 1)  # C

        return temp

    @property
    @ioTask
    def FETTemp(self) -> float:
        self.com.write(bytes([COMMAND_DICT['temp2'], 0]))
        msgLen = int.from_bytes(self.com.read(1), 'big')
        temp = self.__readExactly(msgLen).decode()
        temp = round(float(temp), 1)  # C

        return temp


class BattEval:
    def __init__(self):
        self.battVoltage = 0
        self.battCurrent = 0
        self.FETTemp = 0

        self.battESR = 0

        self.battVoltageHistory = []
        self.adjustedBattVoltageHistory = []
        self.currentHistory = []
        self.temp2History = []
        self.mAhHistory = []

        self.mAh = 0
        self.mWh = 0

        self.ableToTest = False
        self.belowStopPointCount = 0

    def start(self) -> None:
        threading.Thread(target=self.main, daemon=True).start()

    def main(self) -> NoReturn:
        global startTime

        start: Optional[float] = None

        while True:
            self.battVoltage = ser.battVoltage
            self.battCurrent = ser.battCurrent
            self.FETTemp = ser.FETTemp

            FETTempString.set(str(self.FETTemp))

            # battery voltage stuff
            if self.battVoltage > 0.1:
                battVoltageValue['foreground'] = 'green'
                battString.set(str(self.battVoltage)+"v")
                currentString.set(str(self.battCurrent))
                if start is not None:
                    dt = (time_ns() - start)  # ns
                else:
                    dt = 0
                # mA * ns * (s / (10**9) ns) * (h / 3600 s) = mAh
                self.mAh += self.battCurrent*dt / 10**(9) / 3600
                # mA * V * ns * (s / (10**9) ns) * (h / 3600 s) = mWh
                self.mWh += self.battCurrent * \
                    self.battVoltage*dt / 10**(9) / 3600
                start = time_ns()

                mAhString.set(str(int(self.mAh)))
                mWhString.set(str(int(self.mWh)))

                offset = self.battESR*self.battCurrent/1000

                # A constant current test is occuring
                if constCurntStartButton['text'] == "Stop":
                    if int(self.mAh) not in self.mAhHistory:
                        self.battVoltageHistory.append(self.battVoltage)
                        self.adjustedBattVoltageHistory.append(
                            self.battVoltage+offset)
                        self.currentHistory.append(self.battCurrent)
                        self.temp2History.append(self.FETTemp)
                        self.mAhHistory.append(int(self.mAh))

                    updatePlot()

                    timeString.set(
                        str(timedelta(seconds=round(time()-startTime)))+"s")

                    if self.battVoltage+offset < BATT_CHEMS[chemString.get()][0]:
                        self.belowStopPointCount += 1
                    else:
                        self.belowStopPointCount = 0

                    if self.belowStopPointCount > 10:
                        stopConstCurntDraw()

                if chemString.get() != '-':
                    if not self.ableToTest:
                        ready()
                        self.ableToTest = True
                else:
                    if self.ableToTest:
                        notReady()
                        self.ableToTest = False
            else:
                battVoltageValue['foreground'] = 'red'
                battString.set("No Battery Connected")
                if self.ableToTest:
                    notReady()
                    self.ableToTest = False


def updatePlot():
    ax1.cla()
    ax2.cla()
    ax1.grid()
    ax1.set_xlabel("mAh")
    ax1.set_ylabel("volts", color="tab:blue")
    ax2.set_ylabel("milliamps", color="tab:green")
    ax1.plot(board.mAhHistory, board.battVoltageHistory,
             label="Batt Voltage", color="tab:blue")
    ax1.plot(board.mAhHistory, board.adjustedBattVoltageHistory,
             label="Adj Batt Voltage", color="tab:orange")
    ax2.plot(board.mAhHistory, board.currentHistory[-len(board.battVoltageHistory):],
             label="Current Draw", color="tab:green")
    # if chemString.get() != "Custom":
    #     ax1.set_ylim([BATT_CHEMS[chemString.get()][0],
    #                   BATT_CHEMS[chemString.get()][1]])
    # else:
    #     ax1.set_ylim([0, 5])
    ax2.set_ylim([0, 1000])

    canvas.draw()
    canvas.flush_events()


def connect():
    ser.connect(comportString.get())
    connectButton['command'] = disconnect
    connectButton['text'] = "Disconnect"
    battVoltageLabel['state'] = 'normal'
    battVoltageValue['state'] = 'normal'
    temp2Label['state'] = 'normal'
    temp2Value['state'] = 'normal'
    battChemLabel['state'] = 'normal'
    battChemDrop['state'] = 'normal'

    board.start()

    updateStatus("Connection established.")


def disconnect():
    ser.close()
    connectButton['command'] = connect
    connectButton['text'] = "Connect"

    battString.set("")
    currentString.set("")
    FETTempString.set("")
    mAhString.set("")
    mWhString.set("")
    chemString.set("-")

    battVoltageLabel['state'] = 'disabled'
    battVoltageValue['state'] = 'disabled'
    currentLabel['state'] = 'disabled'
    currentValue['state'] = 'disabled'
    # ser.resistorTempString.set("")
    # temp1Label['state'] = 'disabled'
    # temp1Value['state'] = 'disabled'
    temp2Label['state'] = 'disabled'
    temp2Value['state'] = 'disabled'
    mAhLabel['state'] = 'disabled'
    mAhValue['state'] = 'disabled'
    mWhLabel['state'] = 'disabled'
    mWhValue['state'] = 'disabled'
    ESRString.set("")
    ESRValue['state'] = 'disabled'

    battChemLabel['state'] = 'disabled'
    battChemDrop['state'] = 'disabled'

    board.ableToTest = False

    updateStatus("Disconnected.")


def ready():
    constCurntLabel['state'] = 'normal'
    constCurntValue['state'] = 'normal'
    constCurntStartButton['state'] = 'normal'
    constPowerLabel['state'] = 'normal'
    constPowerValue['state'] = 'normal'
    constPowerStartButton['state'] = 'normal'

    currentLabel['state'] = 'normal'
    currentValue['state'] = 'normal'
    mAhLabel['state'] = 'normal'
    mAhValue['state'] = 'normal'
    mWhLabel['state'] = 'normal'
    mWhValue['state'] = 'normal'

    updateStatus("Ready to run tests.")


def notReady():
    constCurntLabel['state'] = 'disabled'
    constCurntValue['state'] = 'disabled'
    constCurntStartButton['state'] = 'disabled'
    constPowerLabel['state'] = 'disabled'
    constPowerValue['state'] = 'disabled'
    constPowerStartButton['state'] = 'disabled'

    currentString.set("")
    mAhString.set("")
    mWhString.set("")

    currentLabel['state'] = 'disabled'
    currentValue['state'] = 'disabled'
    mAhLabel['state'] = 'disabled'
    mAhValue['state'] = 'disabled'
    mWhLabel['state'] = 'disabled'
    mWhValue['state'] = 'disabled'

    updateStatus(
        "Not ready to run tests, ensure battery is connected.")


def getESR(current=100):
    updateStatus("Determining ESR...")
    ser.battCurrent = 0
    startV = 0
    endV = 0

    for _ in range(50):
        startV += ser.battVoltage

    startV /= 50

    ser.battCurrent = current
    for _ in range(50):
        endV += ser.battVoltage

    ser.battCurrent = 0

    endV /= 50

    result = (startV-endV)*(1000/current)

    if result > 0:
        ESRLabel['state'] = 'normal'
        ESRValue['foreground'] = 'black'
        ESRString.set(str(round(result, 3)))
        board.battESR = round(result, 3)
    else:
        ESRLabel['state'] = 'normal'
        ESRValue['foreground'] = 'red'
        ESRString.set("Fault")

    updateStatus("Done.")


def startConstCurntDraw():
    global startTime
    constCurntStartButton['text'] = 'Working'
    constCurntStartButton['state'] = 'disabled'
    constCurntValue['state'] = 'disabled'
    constPowerStartButton['state'] = 'disabled'
    constPowerValue['state'] = 'disabled'
    ESRValue['state'] = 'disabled'
    getESR(int(constCurntString.get()))
    ESRValue['state'] = 'normal'
    timeLabel['state'] = 'normal'
    timeValue['state'] = 'normal'

    startTime = time()

    ser.battCurrent = int(constCurntString.get())
    constCurntStartButton['state'] = 'nornmal'
    constCurntStartButton['text'] = 'Stop'
    constCurntStartButton['command'] = stopConstCurntDraw
    board.battVoltageHistory = []
    board.adjustedBattVoltageHistory = []
    board.currentHistory = []
    # ser.temp1History = []
    board.temp2History = []
    board.mAhHistory = []
    ax1.clear()
    ax2.clear()

    updateStatus("Test started.")

    if ser.battVoltage < BATT_CHEMS[chemString.get()][1]:
        updateStatus("[WARNING] Battery not full, test may be inconclusive.")


def stopConstCurntDraw():
    ser.battCurrent = 0
    constCurntStartButton['text'] = 'Start'
    constCurntStartButton['command'] = startConstCurntDraw
    constCurntValue['state'] = 'normal'
    constPowerStartButton['state'] = 'normal'
    constPowerValue['state'] = 'normal'

    updateStatus("Test ended.")


def updateStatus(msg: str) -> None:
    status.append(msg)
    if len(status) > 5:
        status.pop(0)
    statusString.set("\n".join(status))


status: List[str] = ['']*5

ser = SerCom()

threading.Thread(target=ioLoop, daemon=True).start()


# set up gui
root = Tk()
root.title("Battery Evaluator Client")
comportString = StringVar()
chemString = StringVar()
constCurntString = StringVar()
constPowerString = StringVar()
battString = StringVar()
currentString = StringVar()
resistorTempString = StringVar()
FETTempString = StringVar()
mAhString = StringVar()
mWhString = StringVar()
ESRString = StringVar()
timeString = StringVar()
statusString = StringVar()

board = BattEval()

main = Frame(root)
main.pack(fill=BOTH, expand=True)
main.grid_columnconfigure(0, weight=1)
main.grid_columnconfigure(1, weight=1)
main.grid_columnconfigure(2, weight=1)
main.grid_rowconfigure(0, weight=1)
main.grid_rowconfigure(1, weight=1)

left = Frame(main)
left.grid(row=0, column=0, padx=5, pady=5, sticky="NW")

middle = Frame(main)
middle.grid(row=0, column=1, padx=5, pady=5, sticky="N")

right = Frame(main)
right.grid(row=0, column=2, padx=5, pady=5, sticky="NE")

bottom = Frame(root)
bottom.pack(side=BOTTOM, fill=BOTH, padx=5, pady=5, expand=True)

statusLabel = Label(bottom, textvariable=statusString)
statusLabel.pack(side=BOTTOM, padx=5, pady=5, anchor='w')

# Controls
comDropLabel = Label(left, text="Com Port:")
comDropLabel.grid(row=0, column=0, padx=5, pady=5, sticky="W")

comDrop = OptionMenu(left, comportString, "-", *ser.deviceList)
comDrop.grid(row=0, column=1, padx=5, pady=5, sticky="W")

connectButton = Button(left, text="Connect",
                       command=connect)
connectButton.grid(row=1, column=1, sticky="W")

battChemLabel = Label(left, text="Batt Chemistry:")
battChemLabel.grid(row=2, column=0, padx=5, pady=5, sticky="W")
battChemLabel['state'] = 'disabled'

battChemDrop = OptionMenu(left, chemString, "-", *BATT_CHEMS)
battChemDrop.grid(row=2, column=1, padx=5, pady=5, sticky="W")
battChemDrop['state'] = 'disabled'

constCurntLabel = Label(left, text="Constant Current\nDischarge Test:")
constCurntLabel.grid(row=3, column=0, padx=5, pady=5, sticky="W")
constCurntLabel['state'] = 'disabled'

constCurntValue = Entry(left, textvariable=constCurntString, width=10)
constCurntValue.grid(row=3, column=1, padx=5, pady=5, sticky="SW")
constCurntValue['state'] = 'disabled'

constCurntStartButton = Button(
    left, text="Start", command=startConstCurntDraw)
constCurntStartButton.grid(row=4, column=1, padx=5, pady=5, sticky="W")
constCurntStartButton['state'] = 'disabled'

constPowerLabel = Label(left, text="Constant Power\nDischarge Test:")
constPowerLabel.grid(row=5, column=0, padx=5, pady=5, sticky="W")
constPowerLabel['state'] = 'disabled'

constPowerValue = Entry(left, textvariable=constPowerString, width=10)
constPowerValue.grid(row=5, column=1, padx=5, pady=5, sticky="SW")
constPowerValue['state'] = 'disabled'

constPowerStartButton = Button(
    left, text="Start")
constPowerStartButton.grid(row=6, column=1, padx=5, pady=5, sticky="W")
constPowerStartButton['state'] = 'disabled'

# Measurements
battVoltageLabel = Label(right, text="Batt voltage:")
battVoltageLabel.grid(row=0, column=0, padx=5, pady=5, sticky="W")
battVoltageLabel['state'] = 'disabled'

battVoltageValue = Entry(right, textvariable=battString, width=10)
battVoltageValue.grid(row=0, column=1, padx=5, pady=5, sticky="W")
battVoltageValue['state'] = 'disabled'

currentLabel = Label(right, text="Current:")
currentLabel.grid(row=1, column=0, padx=5, pady=5, sticky="W")
currentLabel['state'] = 'disabled'

currentValue = Entry(right, textvariable=currentString, width=10)
currentValue.grid(row=1, column=1, padx=5, pady=5, sticky="W")
currentValue['state'] = 'disabled'

temp1Label = Label(right, text="Resistor Temp:")
temp1Label.grid(row=2, column=0, padx=5, pady=5, sticky="W")
temp1Label['state'] = 'disabled'

temp1Value = Entry(right, textvariable=resistorTempString, width=10)
temp1Value.grid(row=2, column=1, padx=5, pady=5, sticky="W")
temp1Value['state'] = 'disabled'

temp2Label = Label(right, text="MOS Temp:")
temp2Label.grid(row=3, column=0, padx=5, pady=5, sticky="W")
temp2Label['state'] = 'disabled'

temp2Value = Entry(right, textvariable=FETTempString, width=10)
temp2Value.grid(row=3, column=1, padx=5, pady=5, sticky="W")
temp2Value['state'] = 'disabled'

mAhLabel = Label(right, text="mAh Consumed:")
mAhLabel.grid(row=4, column=0, padx=5, pady=5, sticky="W")
mAhLabel['state'] = 'disabled'

mAhValue = Entry(right, textvariable=mAhString, width=10)
mAhValue.grid(row=4, column=1, padx=5, pady=5, sticky="W")
mAhValue['state'] = 'disabled'

mWhLabel = Label(right, text="mWh Consumed:")
mWhLabel.grid(row=5, column=0, padx=5, pady=5, sticky="W")
mWhLabel['state'] = 'disabled'

mWhValue = Entry(right, textvariable=mWhString, width=10)
mWhValue.grid(row=5, column=1, padx=5, pady=5, sticky="W")
mWhValue['state'] = 'disabled'

timeLabel = Label(right, text="Time Elapsed:")
timeLabel.grid(row=6, column=0, padx=5, pady=5, sticky="W")
timeLabel['state'] = 'disabled'

timeValue = Entry(right, textvariable=timeString, width=10)
timeValue.grid(row=6, column=1, padx=5, pady=5, sticky="W")
timeValue['state'] = 'disabled'

ESRLabel = Label(right, text="ESR:")
ESRLabel.grid(row=7, column=0, padx=5, pady=5, sticky="W")
ESRLabel['state'] = 'disabled'

ESRValue = Entry(right, textvariable=ESRString, width=10)
ESRValue.grid(row=7, column=1, padx=5, pady=5, sticky="W")
ESRValue['state'] = 'disabled'

fig = Figure()

ax1 = fig.add_subplot(111)
ax1.set_ylabel("volts", color="tab:blue")

ax2 = ax1.twinx()
ax2.set_ylabel("milliamps", color="tab:green")

fig.tight_layout()

canvas = FigureCanvasTkAgg(fig, master=middle)
canvas._tkcanvas.pack()

mainloop()
