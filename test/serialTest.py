# Hello World Example
#
# Welcome to the MaixPy IDE!
# 1. Conenct board to computer
# 2. Select board at the top of MaixPy IDE: `tools->Select Board`
# 3. Click the connect buttion below to connect board
# 4. Click on the green run arrow button below to run the script!
STATE_FORWARD = "1 ;"
STATE_OBSTACLE_AVOIDANCE = "2 ;"
STATE_INDICATING_LEFT = "3 ;"
STATE_INDICATING_RIGHT = "4 ;"
STATE_TURNING_RIGHT = "5 ;"
STATE_TURNING_LEFT = "6 ;"
STATE_TURNING_STRAIGHT = "7 ;"
STATE_STOP = "8 ;"
STATE_REVERSE = "9 ;"
#try:
import sensor, image, time, lcd, re
from fpioa_manager import fm
from machine import UART
from board import board_info
from machine import Timer
fm.register (board_info.PIN10, fm.fpioa.UART2_RX)
fm.register (board_info.PIN11, fm.fpioa.UART3_TX)
uartIn = UART(UART.UART2, 115200, bits = 8, parity = None, stop = 1, timeout = 1000, read_buf_len=512)
uartOut = UART(UART.UART3, 115200, bits = 8, parity = None, stop = 1, timeout = 1000, read_buf_len=512)
lcd.init(freq=15000000)
clock = time.clock()                # Create a clock object to track the FPS.

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.run(1)
lcd.rotation(0)
state = STATE_STOP
wall = 1
result = uartOut.write(STATE_INDICATING_LEFT)
time.sleep_ms(750)
result = uartOut.write(STATE_STOP)
time.sleep_ms(750)
result = uartOut.write(STATE_TURNING_LEFT)
time.sleep_ms(750)
result = uartOut.write(STATE_INDICATING_RIGHT)
time.sleep_ms(750)
result = uartOut.write(STATE_STOP)
time.sleep_ms(750)
result = uartOut.write(STATE_TURNING_RIGHT)
time.sleep_ms(750)
result = uartOut.write(STATE_TURNING_STRAIGHT)
time.sleep_ms(750)
result = uartOut.write(STATE_STOP)
time.sleep_ms(750)
while(True):

    result = uartOut.write(state)
    read_data = uartIn.read(512)
    if read_data:
        read_str = read_data.decode('utf-8')
        resInt = [int(i) for i in read_str.split() if i.isdigit()]
        print(read_str)
        if resInt[0] < 300:
             state = STATE_STOP
             if resInt[0] < 200:
                state = STATE_REVERSE
        if resInt[0] > 300:
            state = STATE_FORWARD
        img = sensor.snapshot()

        result = img.draw_string(0,0, str("Distance Front = " + str(resInt[0])), color=(0,128,0), scale=1.5)
        result = img.draw_string(0, 15, "Battery = 80%", color=(0,128,0), scale=1.5)
        if state == STATE_STOP:
            result = img.draw_string(0, 30, "Speed = 0%", color=(0,128,0), scale=1.5)
        else:
            result = img.draw_string(0, 30, "Speed = 20%", color=(0,128,0), scale=1.5)
        lcd.display(img)
#except:
    #import machine
    #machine.reset()
