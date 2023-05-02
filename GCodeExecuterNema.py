import RPi.GPIO as GPIO
from time import sleep
import sys
from RpiMotorLib import RpiMotorLib
from math import pi, sin, cos, sqrt, acos, asin

direction = 21
direction2 =16
step = 20
step2 =12
EN_pin = 26
EN_pin2 = 19
MXposition = 0
MYposition = 0
phase = 0
dir = 0
phase2 = 0
dir2 = 0

phase_seq = [[1, 0, 0, 0], [1, 1, 0, 0], [0, 1, 0, 0], [0, 1, 1, 0], [
    0, 0, 1, 0], [0, 0, 1, 1], [0, 0, 0, 1], [1, 0, 0, 1]]
# half-step sequence. double resolution. But the torque of the stepper motor is not constant
num_phase = len(phase_seq)


def GCD(a, b):  # greatest common diviser
    while b:
        a, b = b, a % b
    return a
def LCM(a, b):  # least common multipler
    return a*b/GCD(a, b)
def sign(a):  # return the sign of number a
    if a > 0:
        return 1
    elif a < 0:
        return -1
    else:
        return 0

# pin number for a1,a2,b1,b2.  a1 and a2 form coil A b1 and b2 form coil B
MX = RpiMotorLib.A4988Nema(direction,step, (-1,-1,-1), "A4988")
#MX = Bipolar_Stepper_Motor(11, 12, 15, 13, 'X')
MY = RpiMotorLib.A4988Nema(direction2,step2, (-1,-1,-1), "A4988")
#MY = Bipolar_Stepper_Motor(16, 18, 37, 36, 'Y')

GPIO.setup(EN_pin, GPIO.OUT)
GPIO.output(EN_pin,GPIO.LOW)
GPIO.setup(EN_pin2, GPIO.OUT)
GPIO.output(EN_pin2,GPIO.LOW)

filename = 'spiral.nc'  # file name of the G code commands

if len(sys.argv) > 1:
    filename = str(sys.argv[1])

dx = 0.075  # resolution in x direction. Unit: mm
dy = 0.075  # resolution in y direction. Unit: mm
#IsPenDown = 2  # 0 - false, 1 - true 2 - unknown

# given a movement command line, return the X Y position
def XYposition(lines):

    xchar_loc = lines.index('X')
    i = xchar_loc+1
    while (47 < ord(lines[i]) < 58) | (lines[i] == '.') | (lines[i] == '-'):
        i += 1
    x_pos = float(lines[xchar_loc+1:i])

    ychar_loc = lines.index('Y')
    i = ychar_loc+1
    while (47 < ord(lines[i]) < 58) | (lines[i] == '.') | (lines[i] == '-'):
        i += 1
    y_pos = float(lines[ychar_loc+1:i])

    return x_pos, y_pos

# given a G02 or G03 movement command line, return the I J position
def IJposition(lines):

    ichar_loc = lines.index('I')
    i = ichar_loc+1
    while (47 < ord(lines[i]) < 58) | (lines[i] == '.') | (lines[i] == '-'):
        i += 1
    i_pos = float(lines[ichar_loc+1:i])

    jchar_loc = lines.index('J')
    i = jchar_loc+1
    while (47 < ord(lines[i]) < 58) | (lines[i] == '.') | (lines[i] == '-'):
        i += 1
    j_pos = float(lines[jchar_loc+1:i])

    return i_pos, j_pos

def Motor_Step(stepper1, step1, stepper2, step2):
    #   control stepper motor 1 and 2 simultaneously
    #   stepper1 and stepper2 are objects of Bipolar_Stepper_Motor class
    #   direction is reflected in the polarity of [step1] or [step2]

    dir1 = sign(step1)  # get dirction from the polarity of argument [step]
    dir2 = sign(step2)

    step1 = abs(step1)
    step2 = abs(step2)

# [total_micro_step] total number of micro steps
# stepper motor 1 will move one step every [micro_step1] steps
# stepper motor 2 will move one step every [micro_step2] steps
# So [total_mirco_step]=[micro_step1]*[step1] if step1<>0  [total_micro_step]=[micro_step2]*[step2] if step2<>0

    if step1 == 0:
        total_micro_step = step2
        micro_step2 = 1
        # set [micro_step1]>[total_micro_step], so stepper motor will not turn
        micro_step1 = step2+100
    elif step2 == 0:
        total_micro_step = step1
        micro_step1 = 1
        micro_step2 = step1+100
    else:
        total_micro_step = LCM(step1, step2)
        micro_step1 = total_micro_step/step1
        micro_step2 = total_micro_step/step2

    # i is the iterator for the micro_step. i cannot start from 0
    for i in range(1, int(total_micro_step)+1):
        if ((i % micro_step1) == 0):  # motor 1 need to turn one step
            move(0,dir1, 1)

        if ((i % micro_step2) == 0):  # motor 2 need to turn one step
            move(1,dir2, 1)

    return 0

def move(motor, dir, steps):
    global phase
    global phase2
    global dir2
    global num_phase
    global MXposition
    global MYposition
    print('Moving one step in direction' + str(dir) + 'with Motor' + str(motor))
    if(dir == -1):
        dir = 0
    multiplier = 1

    # to make bottom left as home position
    if (motor == 0):
        multiplier = -1
        
    if (motor == 0):     
        for _ in range(steps):
            next_phase = (phase + (multiplier * dir)) % num_phase
        GPIO.output(EN_pin,GPIO.LOW)

        MX.motor_go(dir,
                    "Full",
                    steps,
                    .005,
                    False,
                    .05)
        if(dir == 0):
            dir = -1
        phase = next_phase
        MXposition += dir
        GPIO.output(EN_pin,GPIO.HIGH)
    else:
        for _ in range(steps):
            next_phase = (phase2 + (multiplier * dir)) % num_phase
        GPIO.output(EN_pin2,GPIO.LOW)
        if(dir == -1):
            dir = 0
        MY.motor_go(dir,
                    "Full",
                    steps,
                    .005,
                    False,
                    .05)
        phase2 = next_phase
        if(dir == 0):
            dir = -1
        MYposition += dir
        GPIO.output(EN_pin2,GPIO.HIGH)
        

def moveto(MX, x_pos, dx, MY, y_pos, dy):
    # Move to (x_pos,y_pos) (in real unit)
    stepx = int(round(x_pos/dx))-MXposition
    stepy = int(round(y_pos/dy))-MYposition

    print('Movement: Dx=', stepx, '  Dy=', stepy)
    Motor_Step(MX, stepx, MY, stepy)
    
    return 0

try:  # read and execute G code
    #SetPenDown(False)
    for lines in open(filename, 'r'):
        print(lines)
        if lines == []:
            1  # blank lines

        elif lines[0:3] == 'G90':
            print('start')

        elif lines[0:3] == 'G20':  # working in inch
            dx /= 25.4
            dy /= 25.4
            print('Working in inch')

        elif lines[0:3] == 'G21':  # working in mm
            print('Working in mm')

        elif lines[0:3] == 'M05':
            1
            #SetPenDown(False)

        elif lines[0:3] == 'M03':
            1
            #SetPenDown(True)

        elif lines[0:3] == 'M02':
            1
            #SetPenDown(False)
            print('finished. shuting down')
            break

        elif (lines[0:3] == 'G1F') | (lines[0:4] == 'G1 F'):
            1  # do nothing

        elif (lines[0:5] == 'G01 Z'):
            1
            #SetPenDown(True)

        elif (lines[0:5] == 'G00 Z'):
            1
            #SetPenDown(False)

        # |(lines[0:3]=='G02')|(lines[0:3]=='G03'):
        elif (lines[0:3] == 'G0 ') | (lines[0:3] == 'G1 ') | (lines[0:3] == 'G00') | (lines[0:3] == 'G01'):
            # linear engraving movement
            if (lines[0:3] == 'G0 ' or lines[0:3] == 'G00'):
                engraving = False
            else:
                engraving = True

            #SetPenDown(engraving)

            if (lines.find('X') != -1 and lines.find('Y') != -1):
                [x_pos, y_pos] = XYposition(lines)
                moveto(MX, x_pos, dx, MY, y_pos, dy)

        elif (lines[0:3] == 'G02') | (lines[0:3] == 'G03'):  # circular interpolation
            if (lines.find('X') != -1 and lines.find('Y') != -1 and lines.find('I') != -1 and lines.find('J') != -1):
                #SetPenDown(True)

                old_x_pos = x_pos
                old_y_pos = y_pos

                [x_pos, y_pos] = XYposition(lines)
                [i_pos, j_pos] = IJposition(lines)

                xcenter = old_x_pos+i_pos  # center of the circle for interpolation
                ycenter = old_y_pos+j_pos

                Dx = x_pos-xcenter
                # vector [Dx,Dy] points from the circle center to the new position
                Dy = y_pos-ycenter

                r = sqrt(i_pos**2+j_pos**2)   # radius of the circle

                # pointing from center to current position
                e1 = [-i_pos, -j_pos]
                if (lines[0:3] == 'G02'):  # clockwise
                    # perpendicular to e1. e2 and e1 forms x-y system (clockwise)
                    e2 = [e1[1], -e1[0]]
                else:  # counterclockwise
                    # perpendicular to e1. e1 and e2 forms x-y system (counterclockwise)
                    e2 = [-e1[1], e1[0]]

                # [Dx,Dy]=e1*cos(theta)+e2*sin(theta), theta is the open angle

                costheta = (Dx*e1[0]+Dy*e1[1])/r**2
                # theta is the angule spanned by the circular interpolation curve
                sintheta = (Dx*e2[0]+Dy*e2[1])/r**2

                # there will always be some numerical errors! Make sure abs(costheta)<=1
                if costheta > 1:
                    costheta = 1
                elif costheta < -1:
                    costheta = -1

                theta = acos(costheta)
                if sintheta < 0:
                    theta = 2.0*pi-theta

                # number of point for the circular interpolation
                no_step = int(round(r*theta/dx/5.0))

                for i in range(1, no_step+1):
                    tmp_theta = i*theta/no_step
                    tmp_x_pos = xcenter+e1[0] * \
                        cos(tmp_theta)+e2[0]*sin(tmp_theta)
                    tmp_y_pos = ycenter+e1[1] * \
                        cos(tmp_theta)+e2[1]*sin(tmp_theta)
                    moveto(MX, tmp_x_pos, dx, MY, tmp_y_pos, dy)

except KeyboardInterrupt:
    pass

#SetPenDown(False)   # up the pen
moveto(MX, 0, dx, MY, 0, dy)  # move back to Origin

#MZ.stop()
GPIO.output(EN_pin2,GPIO.HIGH)
GPIO.output(EN_pin,GPIO.HIGH)
GPIO.cleanup()