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

waitForStart = True
print("--> You can move robot freely by your hand")
print("--> REMEMBER...Before running teach points, the robot will always start from Stand By Poistion")
print("            ...so the first memorized position should be near to Stand By Position")
print("--> Press Start button to record the motion")

while waitForStart:

    Buttons = getButton()
    Start_Btn = Buttons[7] #Start

    if Start_Btn == 1:
        waitForStart = False
        startTeach = True
        runTeach = False

    time.sleep(0.1)


Mem_Ang1 = [None]*10000
Mem_Ang2 = [None]*10000
Mem_Ang3 = [None]*10000
Mem_Ang4 = [None]*10000
Mem_Ang5 = [None]*10000
Mem_Ang6 = [None]*10000
Mem_GripperStatus = [None]*1000
TeachSamplingTime = 0.1
RunSamplingTime = 0.1

GripperStatus = 1 # For open at first
robotarm.TorqueGripperOn()

i = 0
K = 0
############################################### Record Teaching ##################################################
while startTeach:

    Buttons = getButton()
    Back_Btn = Buttons[6] #Back
    Start_Btn = Buttons[7] #Start
    GripOpen_Btn = Buttons[9] # Analog Left Push
    GripClose_Btn = Buttons[10] #Analog Right Push

    if Back_Btn == 1:
        startTeach = False
        waitForStart = True

    if GripOpen_Btn == 1:
        robotarm.GripOpen()
        GripperStatus = 1

    if GripClose_Btn == 1:
        robotarm.GripClose()
        GripperStatus = 0

    
    ReadANG = robotarm.ReadAngle()
    Mem_Ang1[i] = ReadANG[0]
    Mem_Ang2[i] = ReadANG[1]
    Mem_Ang3[i] = ReadANG[2]
    Mem_Ang4[i] = ReadANG[3]
    Mem_Ang5[i] = ReadANG[4]
    Mem_Ang6[i] = ReadANG[5]
    Mem_GripperStatus[i] = GripperStatus

    print("Teach Point: %d" %i)
    if GripperStatus == 1:
        print("Gripper Open")
    elif GripperStatus == 0:
        print("Gripper Close")
    else:
        print("No Gripper status...")
    print("------------------------------")
    print("Press Back to stop teaching...")
    print("------------------------------")

    i = i + 1
    time.sleep(TeachSamplingTime)


print(" ")
print("--> Hold the robot with your hand in front area AWAY FROM THE FRAME")
print(" ")
print("==========================================")
print("Press Start Button to run Recording Motion")
print("==========================================")

############################################### Waiting ##################################################
while waitForStart:

    Buttons = getButton()
    Start_Btn = Buttons[7] #Start

    if Start_Btn == 1:
        waitForStart = False
        runTeach = True

    time.sleep(0.1)

############################################### Run Teaching ##################################################
while runTeach:

    robotarm.TorqueOn()
    time.sleep(0.5)
    robotarm.StandByPos()
    time.sleep(0.5)
    Pause = False

    robotarm.SetProfile1(100,15)
    robotarm.SetProfile2(100,15)
    robotarm.SetProfile3(100,15)
    robotarm.SetProfile4(100,15)
    robotarm.SetProfile5(100,15)
    robotarm.SetProfile6(100,15)

    while K < i:

        Buttons = getButton()
        Start_Btn = Buttons[7] #Start
        Back_Btn = Buttons[6] #Back

        if Back_Btn == 1:
            break

        if Start_Btn == 1:
            Pause = True
            print("===============================")
            print("Press Start Button to run again")
            print("===============================")

        while Pause:
            Buttons = getButton()
            Start_Btn = Buttons[7] #Start
            if Start_Btn == 1:
                Pause = False

        robotarm.RunServo(Mem_Ang1[K], Mem_Ang2[K], Mem_Ang3[K], Mem_Ang4[K], Mem_Ang5[K], Mem_Ang6[K])

        if Mem_GripperStatus[K] == 1:
            robotarm.GripOpen()
            #time.sleep(1)
            print("Open Gripper")
        elif Mem_GripperStatus[K] == 0:
            robotarm.GripClose()
            #time.sleep(1)
            print("Close Gripper")

        print("Total point : %d" %i)
        print("Move to point: %d" %K )
        print("----------------------")

        K = K + 1
        time.sleep(RunSamplingTime)

    waitForDecide = True
    print("==================================================")
    print("        Finished Running all teaching point")
    print(" ")
    print("        -> Running Again?...Press start")
    print("        -> End program......Press back")
    print("==================================================")
    
    while waitForDecide:
        Buttons = getButton()
        Start_Btn = Buttons[7] #Start
        Back_Btn = Buttons[6] #Back
        if Start_Btn == 1:
            K = 0
            print("---------------------------------------")
            print("---------------------------------------")
            print("--------Robot is starting again--------")
            print("---------------------------------------")
            print("---------------------------------------")
            waitForDecide = False
        if Back_Btn == 1:
            runTeach = False
            waitForDecide = False



############################################### End Program ##################################################
time.sleep(0.5)
print("Robot is shuting down...")
robotarm.StandByPos()
time.sleep(0.5)
robotarm.RobotArmGoHome()
print("-------------------------------------------------------------------")
print("-------------------------------------------------------------------")
print("------------------------Back to home position ---------------------")
print("-------------------------------------------------------------------")
print("-------------------------------------------------------------------")
time.sleep(0.5)
robotarm.TorqueOff()





  

