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
        StartJog = True

    time.sleep(0.1)

robotarm.TorqueOn()

Vfix = 40
Afix = 8
incrementX = 30
incrementY = 30
incrementZ = 30
RotIncrement = 5

robotarm.RobotArmAwake()
time.sleep(0.5)
robotarm.StandByPos()
time.sleep(0.5)
print("On the stand by position")

print("--------------------------------------------------------------------")
print("--------------------------------------------------------------------")
print("-------------------------Jog Linear started!------------------------")
print("--------------------------------------------------------------------")
print("--------------------------------------------------------------------")
print("====================================================================")
print("Hold on the axis button, and use analog left stick to jog left-right")
print("====================================================================")
print("Translate X  -->  hold on X  +  push Analog left stick LEFT-RIGHT")
print("Translate Y  -->  hold on Y  +  push Analog left stick UP-DOWN")
print("Translate Z  -->  hold on B  +  push Analog left stick UP-DOWN")
print("  Rotate  X  -->  hold on A  +  push Analog left stick LEFT-RIGHT")
print("  Rotate  Y  -->  hold on LB  + push Analog left stick UP-DOWN")
print("  Rotate  Z  -->  hold on RB  + push Analog left stick UP-DOWN")
print("====================================================================")
print("                  One click for gripper button")
print("====================================================================")
print("               Gripper open  --> Analog Left Push")
print("               Gripper close --> Analog Right Push")

while StartJog:
    Buttons = getButton()
    XRot_Btn = Buttons[0] #A
    Z_Btn = Buttons[1] #B
    X_Btn = Buttons[2] #X
    Y_Btn = Buttons[3] #Y
    YRot_Btn = Buttons[4] #LB
    ZRot_Btn = Buttons[5] #RB
    Back_Btn = Buttons[6] #Back
    Start_Btn = Buttons[7] #Start
    Continue_Btn = Buttons[8] #Logiccool
    GripOpen_Btn = Buttons[9] # Analog Left Push
    GripClose_Btn = Buttons[10] #Analog Right Push


    if Continue_Btn == True:
        # Bring robot back to stand by position
        robotarm.StandByPos()
        Vstd = Vfix
        Astd = Afix
        robotarm.SetProfile1(Vstd,Astd)
        robotarm.SetProfile2(Vstd,Astd)
        robotarm.SetProfile3(Vstd,Astd)
        robotarm.SetProfile4(Vstd,Astd)
        robotarm.SetProfile5(Vstd,Astd)
        robotarm.SetProfile6(Vstd,Astd)
        time.sleep(0.5)
        print("--------------------------------------------------------------------")
        print("--------------------------------------------------------------------")
        print("-------------------------Jog Linear Resume!-------------------------")
        print("--------------------------------------------------------------------")
        print("--------------------------------------------------------------------")
        print("====================================================================")
        print("Hold on the axis button, and use analog left stick to jog left-right")
        print("====================================================================")
        print("Translate X  -->  hold on X  +  push Analog left stick LEFT-RIGHT")
        print("Translate Y  -->  hold on Y  +  push Analog left stick UP-DOWN")
        print("Translate Z  -->  hold on B  +  push Analog left stick UP-DOWN")
        print("  Rotate  X  -->  hold on A  +  push Analog left stick LEFT-RIGHT")
        print("  Rotate  Y  -->  hold on LB  + push Analog left stick UP-DOWN")
        print("  Rotate  Z  -->  hold on RB  + push Analog left stick UP-DOWN")
        print("====================================================================")
        print("                  One click for gripper button")
        print("====================================================================")
        print("               Gripper open  --> Analog Left Push")
        print("               Gripper close --> Analog Right Push")

    if Back_Btn == True:
        robotarm.StandByPos()
        time.sleep(0.5)
        robotarm.RobotArmGoHome()
        time.sleep(0.5)
        robotarm.TorqueOff()
        break

    OnceTrig = True
    ExitLoop = False
    while X_Btn == True:
        Buttons = getButton()
        X_Btn = Buttons[2] #X
        Axes = getAxis()
        Ax0 = Axes[0]       #Analog left push right = +1,  Analog left push left = -1
        #Ax1 = Axes[1]       #Analog left push down = +1,  Analog left push up = -1
        if OnceTrig == True:
            # make a constant value of Y and Z before jogging because these value don't change anyway
            PreReadDeg = robotarm.ReadAngle()
            PreDeg1 = PreReadDeg[0]
            PreDeg2 = PreReadDeg[1]
            PreDeg3 = PreReadDeg[2]
            PreDeg4 = PreReadDeg[3]
            PreDeg5 = PreReadDeg[4]
            PreDeg6 = PreReadDeg[5]
            PreReadXYZ = robotarm.RobotArmFWD(PreDeg1,PreDeg2,PreDeg3,PreDeg4,PreDeg5,PreDeg6)
            Yconst = PreReadXYZ[1]
            Zconst = PreReadXYZ[2]
            Xrot_const = PreReadXYZ[3]
            Yrot_const = PreReadXYZ[4]
            Zrot_const = PreReadXYZ[5]
            OnceTrig = False # it would not come and read this if-loop again until release X_Btn

        if ((abs(Ax0) > 0.0001) and (ExitLoop == False)):

            ReadDeg = robotarm.ReadAngle()
            Deg1 = ReadDeg[0]
            Deg2 = ReadDeg[1]
            Deg3 = ReadDeg[2]
            Deg4 = ReadDeg[3]
            Deg5 = ReadDeg[4]
            Deg6 = ReadDeg[5]
            ReadXYZ = robotarm.RobotArmFWD(Deg1,Deg2,Deg3,Deg4,Deg5,Deg6)
            ReadX = ReadXYZ[0]
            Xcom = ReadX + (Ax0*incrementX) 
            Ycom = Yconst
            Zcom = Zconst
            Xrot_com = Xrot_const
            Yrot_com = Yrot_const
            Zrot_com = Zrot_const
            XYZ = robotarm.WorkspaceHorizontalLimitation(Xcom,Ycom,Zcom)
            #print("Xcom: %f" %XYZ[0])
            #print("Ycom: %f" %XYZ[1])
            #print("Zcom: %f" %XYZ[2])
            #print("------------------------------")
            if ((XYZ[0] is not None) and (XYZ[1] is not None) and (XYZ[2] is not None)):
                DriveAng = robotarm.RobotArmINV(XYZ[0],XYZ[1],XYZ[2],Xrot_com,Yrot_com,Zrot_com)
                robotarm.RunServo(DriveAng[0],DriveAng[1],DriveAng[2],DriveAng[3],DriveAng[4],DriveAng[5])
            else:
                print("Robot is out of workspace in horizontal constraint limit!")
                robotarm.StandByPos()
                # Set velocity back
                Vstd = Vfix
                Astd = Afix
                robotarm.SetProfile1(Vstd,Astd)
                robotarm.SetProfile2(Vstd,Astd)
                robotarm.SetProfile3(Vstd,Astd)
                robotarm.SetProfile4(Vstd,Astd)
                robotarm.SetProfile5(Vstd,Astd)
                robotarm.SetProfile6(Vstd,Astd)
                ExitLoop = True
                time.sleep(0.5)
                
    OnceTrig = True
    ExitLoop = False
    while Y_Btn == True:
        Buttons = getButton()
        Y_Btn = Buttons[3] #Y
        Axes = getAxis()
        #Ax0 = Axes[0]       #Analog left push right = +1,  Analog left push left = -1
        Ax1 = Axes[1]       #Analog left push down = +1,  Analog left push up = -1
        if OnceTrig == True:
            # make a constant value of X and Z before jogging because these value don't change anyway
            PreReadDeg = robotarm.ReadAngle()
            PreDeg1 = PreReadDeg[0]
            PreDeg2 = PreReadDeg[1]
            PreDeg3 = PreReadDeg[2]
            PreDeg4 = PreReadDeg[3]
            PreDeg5 = PreReadDeg[4]
            PreDeg6 = PreReadDeg[5]
            PreReadXYZ = robotarm.RobotArmFWD(PreDeg1,PreDeg2,PreDeg3,PreDeg4,PreDeg5,PreDeg6)
            Xconst = PreReadXYZ[0]
            Zconst = PreReadXYZ[2]
            Xrot_const = PreReadXYZ[3]
            Yrot_const = PreReadXYZ[4]
            Zrot_const = PreReadXYZ[5]
            OnceTrig = False # it would not come and read this if-loop again until release X_Btn

        if ((abs(Ax1) > 0.0001) and (ExitLoop == False)):
            #Axes = getAxis()
            #Ax0 = Axes[0]       #Analog left push right = +1,  Analog left push left = -1
            Ax1 = Axes[1]       #Analog left push down = +1,  Analog left push up = -1
            ReadDeg = robotarm.ReadAngle()
            Deg1 = ReadDeg[0]
            Deg2 = ReadDeg[1]
            Deg3 = ReadDeg[2]
            Deg4 = ReadDeg[3]
            Deg5 = ReadDeg[4]
            Deg6 = ReadDeg[5]
            ReadXYZ = robotarm.RobotArmFWD(Deg1,Deg2,Deg3,Deg4,Deg5,Deg6)
            ReadY = ReadXYZ[1]
            Ycom = ReadY - (Ax1*incrementY) 
            Xcom = Xconst
            Zcom = Zconst
            Xrot_com = Xrot_const
            Yrot_com = Yrot_const
            Zrot_com = Zrot_const
            XYZ = robotarm.WorkspaceHorizontalLimitation(Xcom,Ycom,Zcom)
            #print("Xcom: %f" %Xcom)
            #print("Ycom: %f" %Ycom)
            #print("Zcom: %f" %Zcom)
            #print("------------------------------")
            if ((XYZ[0] is not None) and (XYZ[1] is not None) and (XYZ[2] is not None)):
                DriveAng = robotarm.RobotArmINV(XYZ[0],XYZ[1],XYZ[2],Xrot_com,Yrot_com,Zrot_com)
                robotarm.RunServo(DriveAng[0],DriveAng[1],DriveAng[2],DriveAng[3],DriveAng[4],DriveAng[5])
            else:
                print("Robot is out of workspace in horizontal constraint limit!")
                robotarm.StandByPos()
                Vstd = Vfix
                Astd = Afix
                robotarm.SetProfile1(Vstd,Astd)
                robotarm.SetProfile2(Vstd,Astd)
                robotarm.SetProfile3(Vstd,Astd)
                robotarm.SetProfile4(Vstd,Astd)
                robotarm.SetProfile5(Vstd,Astd)
                robotarm.SetProfile6(Vstd,Astd)
                ExitLoop = True
                time.sleep(0.5)

    OnceTrig = True
    ExitLoop = False
    while Z_Btn == True:
        Buttons = getButton()
        Z_Btn = Buttons[1] #B
        Axes = getAxis()
        #Ax0 = Axes[0]       #Analog left push right = +1,  Analog left push left = -1
        Ax1 = Axes[1]       #Analog left push down = +1,  Analog left push up = -1
        if OnceTrig == True:
            # make a constant value of X and Y before jogging because these value don't change anyway
            PreReadDeg = robotarm.ReadAngle()
            PreDeg1 = PreReadDeg[0]
            PreDeg2 = PreReadDeg[1]
            PreDeg3 = PreReadDeg[2]
            PreDeg4 = PreReadDeg[3]
            PreDeg5 = PreReadDeg[4]
            PreDeg6 = PreReadDeg[5]
            PreReadXYZ = robotarm.RobotArmFWD(PreDeg1,PreDeg2,PreDeg3,PreDeg4,PreDeg5,PreDeg6)
            Xconst = PreReadXYZ[0]
            Yconst = PreReadXYZ[1]
            Xrot_const = PreReadXYZ[3]
            Yrot_const = PreReadXYZ[4]
            Zrot_const = PreReadXYZ[5]
            OnceTrig = False # it would not come and read this if-loop again until release X_Btn

        if ((abs(Ax1) > 0.0001) and (ExitLoop == False)):
            #Axes = getAxis()
            #Ax0 = Axes[0]       #Analog left push right = +1,  Analog left push left = -1
            Ax1 = Axes[1]       #Analog left push down = +1,  Analog left push up = -1
            ReadDeg = robotarm.ReadAngle()
            Deg1 = ReadDeg[0]
            Deg2 = ReadDeg[1]
            Deg3 = ReadDeg[2]
            Deg4 = ReadDeg[3]
            Deg5 = ReadDeg[4]
            Deg6 = ReadDeg[5]
            ReadXYZ = robotarm.RobotArmFWD(Deg1,Deg2,Deg3,Deg4,Deg5,Deg6)
            ReadZ = ReadXYZ[2]
            Zcom = ReadZ - (Ax1*incrementZ) 
            Xcom = Xconst
            Ycom = Yconst
            Xrot_com = Xrot_const
            Yrot_com = Yrot_const
            Zrot_com = Zrot_const
            XYZ = robotarm.WorkspaceHorizontalLimitation(Xcom,Ycom,Zcom)
            #print("Xcom: %f" %Xcom)
            #print("Ycom: %f" %Ycom)
            #print("Zcom: %f" %Zcom)
            #print("------------------------------")
            if ((XYZ[0] is not None) and (XYZ[1] is not None) and (XYZ[2] is not None)):
                DriveAng = robotarm.RobotArmINV(XYZ[0],XYZ[1],XYZ[2],Xrot_com,Yrot_com,Zrot_com)
                robotarm.RunServo(DriveAng[0],DriveAng[1],DriveAng[2],DriveAng[3],DriveAng[4],DriveAng[5])
            else:
                print("Robot is out of workspace in horizontal constraint limit!")
                robotarm.StandByPos()
                Vstd = Vfix
                Astd = Afix
                robotarm.SetProfile1(Vstd,Astd)
                robotarm.SetProfile2(Vstd,Astd)
                robotarm.SetProfile3(Vstd,Astd)
                robotarm.SetProfile4(Vstd,Astd)
                robotarm.SetProfile5(Vstd,Astd)
                robotarm.SetProfile6(Vstd,Astd)
                ExitLoop = True
                time.sleep(0.5)

    OnceTrig = True
    while XRot_Btn == True:
        Buttons = getButton()
        XRot_Btn = Buttons[0] #A
        Axes = getAxis()
        Ax0 = Axes[0]       #Analog left push right = +1,  Analog left push left = -1
        #Ax1 = Axes[1]       #Analog left push down = +1,  Analog left push up = -1
        if OnceTrig == True:

            PreReadDeg = robotarm.ReadAngle()
            PreDeg1 = PreReadDeg[0]
            PreDeg2 = PreReadDeg[1]
            PreDeg3 = PreReadDeg[2]
            PreDeg4 = PreReadDeg[3]
            PreDeg5 = PreReadDeg[4]
            PreDeg6 = PreReadDeg[5]
            PreReadXYZ = robotarm.RobotArmFWD(PreDeg1,PreDeg2,PreDeg3,PreDeg4,PreDeg5,PreDeg6)
            Xconst = PreReadXYZ[0]
            Yconst = PreReadXYZ[1]
            Zconst = PreReadXYZ[2]
            Yrot_const = PreReadXYZ[4]
            Zrot_const = PreReadXYZ[5]
            OnceTrig = False # it would not come and read this if-loop again until release X_Btn

        if ((abs(Ax0) > 0.0001) and (ExitLoop == False)):
            ReadDeg = robotarm.ReadAngle()
            Deg1 = ReadDeg[0]
            Deg2 = ReadDeg[1]
            Deg3 = ReadDeg[2]
            Deg4 = ReadDeg[3]
            Deg5 = ReadDeg[4]
            Deg6 = ReadDeg[5]
            ReadXYZ = robotarm.RobotArmFWD(Deg1,Deg2,Deg3,Deg4,Deg5,Deg6)
            Xcom = Xconst
            Ycom = Yconst
            Zcom = Zconst
            Xrot_com = ReadXYZ[3] + (Ax0*RotIncrement)
            Yrot_com = Yrot_const
            Zrot_com = Zrot_const
            XYZ = robotarm.WorkspaceLimitation(Xcom,Ycom,Zcom)
            #print("Xcom: %f" %Xcom)
            #print("Ycom: %f" %Ycom)
            #print("Zcom: %f" %Zcom)
            #print("Xrot_com: %f" %Xrot_com)
            #print("Yrot_com: %f" %Yrot_com)
            #print("Zrot_com: %f" %Zrot_com)
            #print("------------------------------")
            if ((XYZ[0] is not None) and (XYZ[1] is not None) and (XYZ[2] is not None)):
                DriveAng = robotarm.RobotArmINV(XYZ[0],XYZ[1],XYZ[2],Xrot_com,Yrot_com,Zrot_com)
                robotarm.RunServo(DriveAng[0],DriveAng[1],DriveAng[2],DriveAng[3],DriveAng[4],DriveAng[5])
            else:
                print("Robot is out of workspace in horizontal constraint limit!")
                robotarm.StandByPos()
                Vstd = Vfix
                Astd = Afix
                robotarm.SetProfile1(Vstd,Astd)
                robotarm.SetProfile2(Vstd,Astd)
                robotarm.SetProfile3(Vstd,Astd)
                robotarm.SetProfile4(Vstd,Astd)
                robotarm.SetProfile5(Vstd,Astd)
                robotarm.SetProfile6(Vstd,Astd)
                ExitLoop = True
                time.sleep(0.5)


    OnceTrig = True
    while YRot_Btn == True:
        Buttons = getButton()
        YRot_Btn = Buttons[4] #LB
        Axes = getAxis()
        #Ax0 = Axes[0]       #Analog left push right = +1,  Analog left push left = -1
        Ax1 = Axes[1]       #Analog left push down = +1,  Analog left push up = -1
        if OnceTrig == True:
            PreReadDeg = robotarm.ReadAngle()
            PreDeg1 = PreReadDeg[0]
            PreDeg2 = PreReadDeg[1]
            PreDeg3 = PreReadDeg[2]
            PreDeg4 = PreReadDeg[3]
            PreDeg5 = PreReadDeg[4]
            PreDeg6 = PreReadDeg[5]
            PreReadXYZ = robotarm.RobotArmFWD(PreDeg1,PreDeg2,PreDeg3,PreDeg4,PreDeg5,PreDeg6)
            Xconst = PreReadXYZ[0]
            Yconst = PreReadXYZ[1]
            Zconst = PreReadXYZ[2]
            Xrot_const = PreReadXYZ[3]
            Zrot_const = PreReadXYZ[5]
            OnceTrig = False # it would not come and read this if-loop again until release X_Btn

        if ((abs(Ax1) > 0.0001) and (ExitLoop == False)):
            ReadDeg = robotarm.ReadAngle()
            Deg1 = ReadDeg[0]
            Deg2 = ReadDeg[1]
            Deg3 = ReadDeg[2]
            Deg4 = ReadDeg[3]
            Deg5 = ReadDeg[4]
            Deg6 = ReadDeg[5]
            ReadXYZ = robotarm.RobotArmFWD(Deg1,Deg2,Deg3,Deg4,Deg5,Deg6)
            Xcom = Xconst
            Ycom = Yconst
            Zcom = Zconst
            Yrot_com = ReadXYZ[4] - (Ax1*RotIncrement)
            Xrot_com = Xrot_const
            Zrot_com = Zrot_const
            XYZ = robotarm.WorkspaceLimitation(Xcom,Ycom,Zcom)
            #print("Xcom: %f" %Xcom)
            #print("Ycom: %f" %Ycom)
            #print("Zcom: %f" %Zcom)
            #print("Xrot_com: %f" %Xrot_com)
            #print("Yrot_com: %f" %Yrot_com)
            #print("Zrot_com: %f" %Zrot_com)
            #print("------------------------------")
            if ((XYZ[0] is not None) and (XYZ[1] is not None) and (XYZ[2] is not None)):
                DriveAng = robotarm.RobotArmINV(XYZ[0],XYZ[1],XYZ[2],Xrot_com,Yrot_com,Zrot_com)
                robotarm.RunServo(DriveAng[0],DriveAng[1],DriveAng[2],DriveAng[3],DriveAng[4],DriveAng[5])
            else:
                print("Robot is out of workspace in horizontal constraint limit!")
                robotarm.StandByPos()
                Vstd = Vfix
                Astd = Afix
                robotarm.SetProfile1(Vstd,Astd)
                robotarm.SetProfile2(Vstd,Astd)
                robotarm.SetProfile3(Vstd,Astd)
                robotarm.SetProfile4(Vstd,Astd)
                robotarm.SetProfile5(Vstd,Astd)
                robotarm.SetProfile6(Vstd,Astd)
                ExitLoop = True
                time.sleep(0.5)

    OnceTrig = True
    while ZRot_Btn == True:
        Buttons = getButton()
        ZRot_Btn = Buttons[5] #RB
        Axes = getAxis()
        #Ax0 = Axes[0]       #Analog left push right = +1,  Analog left push left = -1
        Ax1 = Axes[1]       #Analog left push down = +1,  Analog left push up = -1
        if OnceTrig == True:
            PreReadDeg = robotarm.ReadAngle()
            PreDeg1 = PreReadDeg[0]
            PreDeg2 = PreReadDeg[1]
            PreDeg3 = PreReadDeg[2]
            PreDeg4 = PreReadDeg[3]
            PreDeg5 = PreReadDeg[4]
            PreDeg6 = PreReadDeg[5]
            PreReadXYZ = robotarm.RobotArmFWD(PreDeg1,PreDeg2,PreDeg3,PreDeg4,PreDeg5,PreDeg6)
            Xconst = PreReadXYZ[0]
            Yconst = PreReadXYZ[1]
            Zconst = PreReadXYZ[2]
            Xrot_const = PreReadXYZ[3]
            Yrot_const = PreReadXYZ[4]
            OnceTrig = False # it would not come and read this if-loop again until release X_Btn

        if ((abs(Ax1) > 0.0001) and (ExitLoop == False)):
            ReadDeg = robotarm.ReadAngle()
            Deg1 = ReadDeg[0]
            Deg2 = ReadDeg[1]
            Deg3 = ReadDeg[2]
            Deg4 = ReadDeg[3]
            Deg5 = ReadDeg[4]
            Deg6 = ReadDeg[5]
            ReadXYZ = robotarm.RobotArmFWD(Deg1,Deg2,Deg3,Deg4,Deg5,Deg6)
            Xcom = Xconst
            Ycom = Yconst
            Zcom = Zconst
            Zrot_com = ReadXYZ[5] - (Ax1*RotIncrement)
            Xrot_com = Xrot_const
            Yrot_com = Yrot_const
            XYZ = robotarm.WorkspaceLimitation(Xcom,Ycom,Zcom)
            #print("Xcom: %f" %Xcom)
            #print("Ycom: %f" %Ycom)
            #print("Zcom: %f" %Zcom)
            #print("Xrot_com: %f" %Xrot_com)
            #print("Yrot_com: %f" %Yrot_com)
            #print("Zrot_com: %f" %Zrot_com)
            #print("------------------------------")
            if ((XYZ[0] is not None) and (XYZ[1] is not None) and (XYZ[2] is not None)):
                DriveAng = robotarm.RobotArmINV(XYZ[0],XYZ[1],XYZ[2],Xrot_com,Yrot_com,Zrot_com)
                robotarm.RunServo(DriveAng[0],DriveAng[1],DriveAng[2],DriveAng[3],DriveAng[4],DriveAng[5])
            else:
                print("Robot is out of workspace in horizontal constraint limit!")
                robotarm.StandByPos()
                Vstd = Vfix
                Astd = Afix
                robotarm.SetProfile1(Vstd,Astd)
                robotarm.SetProfile2(Vstd,Astd)
                robotarm.SetProfile3(Vstd,Astd)
                robotarm.SetProfile4(Vstd,Astd)
                robotarm.SetProfile5(Vstd,Astd)
                robotarm.SetProfile6(Vstd,Astd)
                ExitLoop = True
                time.sleep(0.5)

    if GripOpen_Btn == True:
        robotarm.GripOpen()
        time.sleep(0.5)
    if GripClose_Btn == True:
        robotarm.GripClose()
        time.sleep(0.5)
