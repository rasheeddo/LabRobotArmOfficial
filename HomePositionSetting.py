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
print("Press Start Button!")

while waitForStart:

    Buttons = getButton()
    Start_Btn = Buttons[7] #Start

    if Start_Btn == 1:
        waitForStart = False
        startTeachHome = True

    time.sleep(0.1)


print("--> Bring the robot to front area by your hand")
print("--> REMEMBER...before going home the robot should stay in Stand By Position")
print("            ...so the first memorized position should be near to that point")
print("--> Press Logicool button to memorize the position")

Home_Ang1 = [None]*100
Home_Ang2 = [None]*100
Home_Ang3 = [None]*100
Home_Ang4 = [None]*100
Home_Ang5 = [None]*100
Home_Ang6 = [None]*100
Mem_GripperStatus = [None]*100

i = 0
J = 0

GripperStatus = 1 # For open at first
robotarm.TorqueGripperOn()

while startTeachHome:

    Buttons = getButton()
    Back_Btn = Buttons[6] #Back
    Start_Btn = Buttons[7] #Start
    Memo_Btn = Buttons[8] #Logiccool
    GripOpen_Btn = Buttons[9] # Analog Left Push
    GripClose_Btn = Buttons[10] #Analog Right Push
    
    ReadANG = robotarm.ReadAngle()
    ANG1 = ReadANG[0]
    ANG2 = ReadANG[1]
    ANG3 = ReadANG[2]
    ANG4 = ReadANG[3]
    ANG5 = ReadANG[4]
    ANG6 = ReadANG[5]
    
    if GripOpen_Btn == 1:
        robotarm.GripOpen()
        GripperStatus = 1

    if GripClose_Btn == 1:
        robotarm.GripClose()
        GripperStatus = 0

    if Memo_Btn == 1:
        ReadANG = robotarm.ReadAngle()
        Home_Ang1[i] = ReadANG[0]
        Home_Ang2[i] = ReadANG[1]
        Home_Ang3[i] = ReadANG[2]
        Home_Ang4[i] = ReadANG[3]
        Home_Ang5[i] = ReadANG[4]
        Home_Ang6[i] = ReadANG[5]
        Mem_GripperStatus[i] = GripperStatus

        print("------------------------------")
        print("------------------------------")
        print("Home_Ang1: %f" %Home_Ang1[i])
        print("Home_Ang2: %f" %Home_Ang2[i])
        print("Home_Ang3: %f" %Home_Ang3[i])
        print("Home_Ang4: %f" %Home_Ang4[i])
        print("Home_Ang5: %f" %Home_Ang5[i])
        print("Home_Ang6: %f" %Home_Ang6[i])
        if GripperStatus == 1:
            print("Gripper Open")
        elif GripperStatus == 0:
            print("Gripper Close")
        else:
            print("No Gripper status...")
        print("------------------------------")
        print("------------------------------")

        i = i + 1
        while Memo_Btn == 1:
            Buttons = getButton()
            Memo_Btn = Buttons[8] #Logiccool

        print("Teach Point: %d" %i)
        print("Press Back button if done")

    if Back_Btn == 1:
        for J in range(0,i):
            print("# Point: %d" %J)
            print "self.Home_Ang1[{:d}] = {:6f}".format(J,Home_Ang1[J])
            print "self.Home_Ang2[{:d}] = {:6f}".format(J,Home_Ang2[J])
            print "self.Home_Ang3[{:d}] = {:6f}".format(J,Home_Ang3[J])
            print "self.Home_Ang4[{:d}] = {:6f}".format(J,Home_Ang4[J])
            print "self.Home_Ang5[{:d}] = {:6f}".format(J,Home_Ang5[J])
            print "self.Home_Ang6[{:d}] = {:6f}".format(J,Home_Ang6[J])
            print(" ")
        startTeachHome = False
        waitForStart = True

print("=====================================================")
print("Please copy all points of the new home position above")
print("=====================================================")
print(" ")
print("--> Hold the robot with your hand in front area AWAY FROM THE FRAME")
print("--> Press Start Button to see the teaching motion")

while waitForStart:

    Buttons = getButton()
    Start_Btn = Buttons[7] #Start

    if Start_Btn == 1:
        waitForStart = False
        runTeach = True

    time.sleep(0.1)

while runTeach:

    robotarm.TorqueOn()
    time.sleep(0.5)
    robotarm.StandByPos()
    time.sleep(0.5)
    #RobotArmGoHome()
    #time.sleep(1)
    PreAng = robotarm.ReadAngle()
    PreAng1 = PreAng[0]
    PreAng2 = PreAng[1]
    PreAng3 = PreAng[2]
    PreAng4 = PreAng[3]
    PreAng5 = PreAng[4]
    PreAng6 = PreAng[5]

    for K in range(0,i):

        Buttons = getButton()
        Back_Btn = Buttons[6] #Back

        if Back_Btn == 1:
            robotarm.StandByPos()
            time.sleep(1)
            robotarm.RobotArmGoHome()
            time.sleep(1)
            break

        GoalPos1 = Home_Ang1[K]
        GoalPos2 = Home_Ang2[K]
        GoalPos3 = Home_Ang3[K]
        GoalPos4 = Home_Ang4[K]
        GoalPos5 = Home_Ang5[K]
        GoalPos6 = Home_Ang6[K]

        DelPos1 = robotarm.DeltaPos(PreAng1,GoalPos1)
        DelPos2 = robotarm.DeltaPos(PreAng2,GoalPos2)
        DelPos3 = robotarm.DeltaPos(PreAng3,GoalPos3)
        DelPos4 = robotarm.DeltaPos(PreAng4,GoalPos4)
        DelPos5 = robotarm.DeltaPos(PreAng5,GoalPos5)
        DelPos6 = robotarm.DeltaPos(PreAng6,GoalPos6)

        VSTD = 40
        ASTD = 10

        TRAJ = robotarm.TrajectoryGeneration3(VSTD,ASTD,DelPos1,DelPos2,DelPos3,DelPos4,DelPos5,DelPos6)

        V1 = TRAJ[0]
        A1 = TRAJ[1]
        V2 = TRAJ[2]
        A2 = TRAJ[3]
        V3 = TRAJ[4]
        A3 = TRAJ[5]
        V4 = TRAJ[6]
        A4 = TRAJ[7]
        V5 = TRAJ[8]
        A5 = TRAJ[9]
        V6 = TRAJ[10]
        A6 = TRAJ[11]

        robotarm.SetProfile1(V1,A1)
        robotarm.SetProfile2(V2,A2)
        robotarm.SetProfile3(V3,A3)
        robotarm.SetProfile4(V4,A4)
        robotarm.SetProfile5(V5,A5)
        robotarm.SetProfile6(V6,A6)

        robotarm.RunServo(Home_Ang1[K], Home_Ang2[K], Home_Ang3[K], Home_Ang4[K], Home_Ang5[K], Home_Ang6[K])
        print("Move to point %d" %K )
        
        MovingFlag = True
        # This delay would make sure that the moving detection loop will not run too fast than actual motion
        time.sleep(0.1)
        while MovingFlag:
            Move1 = robotarm.IsMoving1()
            Move2 = robotarm.IsMoving2()
            Move3 = robotarm.IsMoving3()
            Move4 = robotarm.IsMoving4()
            Move5 = robotarm.IsMoving5()
            Move6 = robotarm.IsMoving6()

            if Move1 == 0 and Move2 == 0 and Move3 == 0 and Move4 == 0 and Move5 == 0 and Move6 == 0:
                MovingFlag = False
                print("Finished point %d" %K)
        

        if Mem_GripperStatus[K] == 1:
            robotarm.GripOpen()
            print("Open Gripper")
        elif Mem_GripperStatus[K] == 0:
            robotarm.GripClose()
            print("Close Gripper")
        
        PreAng1 = GoalPos1
        PreAng2 = GoalPos2
        PreAng3 = GoalPos3
        PreAng4 = GoalPos4
        PreAng5 = GoalPos5
        PreAng6 = GoalPos6


    print("If you are not happy with the motion")
    print("Just turn off and run script again")
    print("=====================================================")
    print("              If you are happy with it...            ")
    print("       replace the Home_Ang on the RobotArm class,    ")
    print("        RobotArmGoHome object to the new one         ")
    print("=====================================================")
    robotarm.TorqueOff()
    runTeach = False




  

