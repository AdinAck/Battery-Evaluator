from board import *
from busio import UART
from digitalio import DigitalInOut, Direction
from analogio import AnalogIn, AnalogOut
from microcontroller import watchdog

COMMAND_DICT = {
    'batt-voltage': 0x10,
    'current': 0x11,
    # 'temp1': 0x12,
    'temp2': 0x13,
    'set-current': 0x20,
    'set-power': 0x21,
}

MODE = 'idle'

# Pinouts
uart = UART(TX, RX, baudrate=9600)  # UART
faultLed = DigitalInOut(D13)        # Fault LED
faultLed.direction = Direction.OUTPUT
vref = AnalogIn(A5)                 # Voltage reference
mos = AnalogOut(A0)                 # Mos signal (op-amp)
batt = AnalogIn(A2)                 # Battery voltage
current = AnalogIn(A3)              # Current measure
# temp1 = AnalogIn(A1)
temp2 = AnalogIn(A4)                # Temperature measure 2

targetCurrent = 0
targetPower = 0

watchdog = 0
output = 0


def getVoltage(pin):
    return (pin.value/vref.value)*2.5
    # return pin.value / (2**16) * 3.3


def getMeasurement(pin, factor=1, offset=0):
    return getVoltage(pin)*factor+offset


def sendMeasurement(msg):

    uart.write(
        memoryview(
            bytes([
                len(msg)
            ])
        )
    )

    uart.write(
        memoryview(
            bytes(
                [ord(char) for char in msg]
            )
        )
    )


# Main loop
while True:
    battValue = getMeasurement(batt, 1.47)
    currentValue = getMeasurement(current, 1.47)
    temp2Value = getMeasurement(temp2, 100, -50)

    # Receive command
    buf = uart.read(2)

    if buf is not None:
        watchdog = 0
        data = [i for i in buf]

        command = data[0]
        header = data[1]

        if command == COMMAND_DICT['batt-voltage']:
            sendMeasurement(str(battValue))

        elif command == COMMAND_DICT['current']:
            sendMeasurement(str(currentValue))
        # elif command == COMMAND_DICT['temp1']:
        #     sendMeasurement(str(getTemp(temp1)))
        elif command == COMMAND_DICT['temp2']:
            sendMeasurement(str(temp2Value))

        elif command == COMMAND_DICT['set-current']:
            MODE = 'current'
            targetCurrent = int.from_bytes(
                uart.read(header), 'big')/1000

            output = int(targetCurrent*2**16*14.7/5/3.3)
            print("Set current to {}".format(targetCurrent))
        elif command == COMMAND_DICT['set-power']:
            MODE = 'power'
            targetPower = int.from_bytes(
                uart.read(header), 'big')/1000
    else:

        if watchdog > 10:
            print("Watchdog")
            targetCurrent = 0
        else:
            watchdog += 1

    if battValue > 0.1:
        faultLed.value = False
        if MODE == 'power':
            targetCurrent = targetPower/battValue
        error = int((targetCurrent - currentValue) * 2**16 / 3.3)
        output += error//4
        mos.value = min(max(output, 0), 2**16-1)
    else:
        faultLed.value = True
