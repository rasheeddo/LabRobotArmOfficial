import time
import pygame
from LabRobotArm import *

pygame.init()
j = pygame.joystick.Joystick(0)
j.init()

def getButton():

    pygame.event.pump()
    button0 = j.get_button(0)
    button1 = j.get_button(1)
    button2 = j.get_button(2)
    button3 = j.get_button(3)
    button4 = j.get_button(4)
    button5 = j.get_button(5)
    button6 = j.get_button(6)
    button7 = j.get_button(7)
    button8 = j.get_button(8)
    button9 = j.get_button(9)
    button10 = j.get_button(10)
    joy_button = [button0, button1, button2, button3, button4, button5, button6, button7,button8, button9, button10]
    
    return joy_button

def getAxis():

    pygame.event.pump()
    axis0 = j.get_axis(0)
    axis1 = j.get_axis(1)
    axis2 = j.get_axis(2)
    axis3 = j.get_axis(4)
    axis4 = j.get_axis(3)
    axis5 = j.get_axis(5)
    joy_axis = [axis0, axis1, axis2, axis3, axis4, axis5]
    return joy_axis

def getHat():
    pygame.event.pump()
    hat0 = j.get_hat(0)
    
    joy_hat = hat0
    return joy_hat


def map(val, in_min, in_max, out_min, out_max):

    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

robotarm = RobotArm()

####################################### Setting Parameters #############################################
# Set a profile for gripper / for the other motor would be set later

# Gripper Profile

robotarm.SetProfile7(120,20)

# Set PID gain 
P_Gain1 = 1500    #800 default
I_Gain1 = 100     #0 default
D_Gain1 = 4000   #4700 default
robotarm.SetPID1(P_Gain1, I_Gain1, D_Gain1)
P_Gain2 = 1500    #800 default
I_Gain2 = 100     #0 default
D_Gain2 = 4000   #4700 default
robotarm.SetPID2(P_Gain2,I_Gain2,D_Gain2)
P_Gain3 = 1500    #800 default
I_Gain3 = 100     #0 default
D_Gain3 = 4000   #4700 default
robotarm.SetPID3(P_Gain3,I_Gain3,D_Gain3)
P_Gain4 = 1500    #800 default
I_Gain4 = 100     #0 default
D_Gain4 = 4000   #4700 default
robotarm.SetPID4(P_Gain4,I_Gain4,D_Gain4)
P_Gain5 = 1500    #800 default
I_Gain5 = 100     #0 default
D_Gain5 = 4000   #4700 default
robotarm.SetPID5(P_Gain5,I_Gain5,D_Gain5)
P_Gain6 = 1500    #800 default
I_Gain6 = 100     #0 default
D_Gain6 = 4000   #4700 default
robotarm.SetPID6(P_Gain6,I_Gain6,D_Gain6)

robotarm.SetPID7(1000,30,2000)

# Set Feedforward gain
FF1_Gain1 = 100
FF2_Gain1 = 50
robotarm.SetFFGain1(FF1_Gain1,FF2_Gain1)
FF1_Gain2 = 100
FF2_Gain2 = 50
robotarm.SetFFGain2(FF1_Gain2,FF2_Gain2)
FF1_Gain3 = 100
FF2_Gain3 = 50
robotarm.SetFFGain3(FF1_Gain3,FF2_Gain3)
FF1_Gain4 = 100
FF2_Gain4 = 50
robotarm.SetFFGain4(FF1_Gain4,FF2_Gain4)
FF1_Gain5 = 100
FF2_Gain5 = 50
robotarm.SetFFGain5(FF1_Gain5,FF2_Gain5)
FF1_Gain6 = 100
FF2_Gain6 = 50
robotarm.SetFFGain6(FF1_Gain6,FF2_Gain6)


# Set goal current of gripper
robotarm.SetGoalCurrentGripper(60)

time.sleep(0.5)

################################################################################################################################
print("---------------------------------------------------")
print("---------Press start to wake the robot up----------")
print("---------------------------------------------------")
waitForAwake = True
while waitForAwake:
    Buttons = getButton()
    Start_Btn = Buttons[7] #Start
    if Start_Btn == 1:
        waitForAwake = False

    time.sleep(0.1)

robotarm.RobotArmAwake()
print("All Torque are ON")
time.sleep(0.5)
robotarm.StandByPos()
print("On the stand by position")
time.sleep(0.5)

## Set Velocity / Acceleration Profile 
# Robot joint's profile
robotarm.SetProfile1(40,15)
robotarm.SetProfile2(40,15)
robotarm.SetProfile3(40,15)
robotarm.SetProfile4(40,15)
robotarm.SetProfile5(40,15)
robotarm.SetProfile6(40,15)
# Gripper Profile
robotarm.SetProfile7(120,70)

## Get Joy Stick 

Buttons = getButton()
J1_Btn = Buttons[0] #A
J2_Btn = Buttons[1] #B
J3_Btn = Buttons[2] #X
J4_Btn = Buttons[3] #Y
J5_Btn = Buttons[4] #LB
J6_Btn = Buttons[5] #RB
Back_Btn = Buttons[6] #Back
Start_Btn = Buttons[7] #Start
StandBy_Btn = Buttons[8] #Logiccool
GripOpen_Btn = Buttons[9] # Analog Left Push
GripClose_Btn = Buttons[10] #Analog Right Push

DegIncrement = 10

################################################## Jog Joint ###################################################
waitForStart = True
print("Press Start Button to JogJoint!")

while waitForStart:

    Buttons = getButton()
    Start_Btn = Buttons[7] #Start

    if Start_Btn == 1:
        waitForStart = False
        startJog = True

    time.sleep(0.1)


print("--------------------------------------------------------------------")
print("--------------------------------------------------------------------")
print("-------------------------Jog Joint started!-------------------------")
print("--------------------------------------------------------------------")
print("--------------------------------------------------------------------")
print("====================================================================")
print("Hold on the axis button, and use analog left stick to jog left-right")
print("====================================================================")
print("Joint1        -->        A")
print("Joint2        -->        B")
print("Joint3        -->        X")
print("Joint4        -->        Y")
print("Joint5        -->        LB")
print("Joint6        -->        RB")
print("============================")
print("One click for gripper button")
print("============================")
print("Gripper open  --> Analog Left Push")
print("Gripper close --> Analog Right Push")

while startJog:
    # Get button signal
    Buttons = getButton()
    J1_Btn = Buttons[0] #A
    J2_Btn = Buttons[1] #B
    J3_Btn = Buttons[2] #X
    J4_Btn = Buttons[3] #Y
    J5_Btn = Buttons[4] #LB
    J6_Btn = Buttons[5] #RB
    Back_Btn = Buttons[6] #Back
    Start_Btn = Buttons[7] #Start
    StandBy_Btn = Buttons[8] #Logiccool
    GripOpen_Btn = Buttons[9] # Analog Left Push
    GripClose_Btn = Buttons[10] #Analog Right Push

    ############ Jog Joint 1 ############
    while J1_Btn == 1:

        Buttons = getButton()
        J1_Btn = Buttons[0] #A
        Axes = getAxis()
        Ax0 = Axes[0] #Analog Left value from -1 to 1
        ReadDeg = robotarm.ReadAngle()
        ReadDeg1 = ReadDeg[0]

        if abs(Ax0) > 0.0001:
            # Joint velocity depends on how much you push on analog stick
            DriveAngle = ReadDeg1 + DegIncrement*Ax0
            robotarm.RunServo1(DriveAngle)


    ############ Jog Joint 2 ############
    while J2_Btn == 1:

        Buttons = getButton()
        J2_Btn = Buttons[1] #B
        Axes = getAxis()
        Ax0 = Axes[0] #Analog Left value from -1 to 1
        ReadDeg = robotarm.ReadAngle()
        ReadDeg2 = ReadDeg[1]

        if abs(Ax0) > 0.0001:
            # Joint velocity depends on how much you push on analog stick
            DriveAngle = ReadDeg2 + DegIncrement*Ax0
            robotarm.RunServo2(DriveAngle)

    ############ Jog Joint 3 ############
    while J3_Btn == 1:

        Buttons = getButton()
        J3_Btn = Buttons[2] #X
        Axes = getAxis()
        Ax0 = Axes[0] #Analog Left value from -1 to 1
        ReadDeg = robotarm.ReadAngle()
        ReadDeg3 = ReadDeg[2]

        if abs(Ax0) > 0.0001:
            # Joint velocity depends on how much you push on analog stick
            DriveAngle = ReadDeg3 + DegIncrement*Ax0
            robotarm.RunServo3(DriveAngle)
            #time.sleep(0.01)

    ############ Jog Joint 4 ############
    while J4_Btn == 1:

        Buttons = getButton()
        J4_Btn = Buttons[3] #Y
        Axes = getAxis()
        Ax0 = Axes[0] #Analog Left value from -1 to 1
        ReadDeg = robotarm.ReadAngle()
        ReadDeg4 = ReadDeg[3]

        if abs(Ax0) > 0.0001:
            # Joint velocity depends on how much you push on analog stick
            DriveAngle = ReadDeg4 + DegIncrement*Ax0
            robotarm.RunServo4(DriveAngle)
            #time.sleep(0.01)

    ############ Jog Joint 5 ############
    while J5_Btn == 1:

        Buttons = getButton()
        J5_Btn = Buttons[4] #LB
        Axes = getAxis()
        Ax0 = Axes[0] #Analog Left value from -1 to 1
        ReadDeg = robotarm.ReadAngle()
        ReadDeg5 = ReadDeg[4]

        if abs(Ax0) > 0.0001:
            # Joint velocity depends on how much you push on analog stick
            DriveAngle = ReadDeg5 + DegIncrement*Ax0
            robotarm.RunServo5(DriveAngle)
            #time.sleep(0.01)


    ############ Jog Joint 6 ############
    while J6_Btn == 1:

        Buttons = getButton()
        J6_Btn = Buttons[5] #RB
        Axes = getAxis()
        Ax0 = Axes[0] #Analog Left value from -1 to 1
        ReadDeg = robotarm.ReadAngle()
        ReadDeg6 = ReadDeg[5]

        if abs(Ax0) > 0.0001:
            # Joint velocity depends on how much you push on analog stick
            DriveAngle = ReadDeg6 + DegIncrement*Ax0
            robotarm.RunServo6(DriveAngle)
            #time.sleep(0.01)

    if GripOpen_Btn == 1:
        robotarm.GripOpen()

    if GripClose_Btn == 1:
        robotarm.GripClose()

    if StandBy_Btn == 1:
        print("Robot is back to Stand by position...")
        time.sleep(1)
        robotarm.StandByPos()
        # Set the velocity again
        robotarm.SetProfile1(40,15)
        robotarm.SetProfile2(40,15)
        robotarm.SetProfile3(40,15)
        robotarm.SetProfile4(40,15)
        robotarm.SetProfile5(40,15)
        robotarm.SetProfile6(40,15)
        print("-----------------------------------------------------------")
        print("-----------------------------------------------------------")
        print("-------------------Jog Joint Resume!-----------------------")
        print("-----------------------------------------------------------")
        print("-----------------------------------------------------------")
        print("Joint1        -->        A")
        print("Joint2        -->        B")
        print("Joint3        -->        X")
        print("Joint4        -->        Y")
        print("Joint5        -->        LB")
        print("Joint6        -->        RB")
        print("Gripper open  --> Analog Left Push")
        print("Gripper close --> Analog Right Push")

    if Back_Btn == 1:
        print("Robot is shutting down...")
        time.sleep(1)
        robotarm.StandByPos()
        startJog = False

    time.sleep(0.1)

robotarm.RobotArmGoHome()
print("--------------------------------------------------")
print("--------------------------------------------------")
print("-------------------Shutdown-----------------------")
print("--------------------------------------------------")
print("--------------------------------------------------")
time.sleep(1)
robotarm.TorqueOff()