from LabRobotArm import *
import time

robotarm = RobotArm()

robotarm.SetOperatingMode(3)
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

time.sleep(1)

################################################## Kinematics Run ###################################################
robotarm.TorqueOff()

run = True
try:
	while run:

		ReadAng = robotarm.ReadAngle()
		ReadAng1 = ReadAng[0]
		ReadAng2 = ReadAng[1]
		ReadAng3 = ReadAng[2]
		ReadAng4 = ReadAng[3]
		ReadAng5 = ReadAng[4]
		ReadAng6 = ReadAng[5]
		print("*** Present Angle ***")
		print("ReadAng1: %f" %ReadAng1)
		print("ReadAng2: %f" %ReadAng2)
		print("ReadAng3: %f" %ReadAng3)
		print("ReadAng4: %f" %ReadAng4)
		print("ReadAng5: %f" %ReadAng5)
		print("ReadAng6: %f" %ReadAng6)

		FWD = robotarm.RobotArmFWD(ReadAng1,ReadAng2,ReadAng3,ReadAng4,ReadAng5,ReadAng6)
		X = FWD[0]
		Y = FWD[1]
		Z = FWD[2]
		X6_ROT = FWD[3]
		Y6_ROT = FWD[4]
		Z6_ROT = FWD[5]
		print("*** Present Position ***")
		print("X: %f" %X)
		print("Y: %f" %Y)
		print("Z: %f" %Z)

		# Bring X Y Z values to workspace validation function
		# These two workspace function can be modified to expand the working range
		# 
		XYZ = robotarm.WorkspaceHorizontalLimitation(X,Y,Z)
		#XYZ = WorkspaceLimitation(X,Y,Z)
		X = XYZ[0]
		Y = XYZ[1]
		Z = XYZ[2]
		print("*** Position After Validation ***")
		print("X: %s" %X)
		print("Y: %s" %Y)
		print("Z: %s" %Z)
		print("*** Present Hand's Orientation ***")
		print("X6_ROT: %f" %X6_ROT)
		print("Y6_ROT: %f" %Y6_ROT)
		print("Z6_ROT: %f" %Z6_ROT)
		print("-------------------------------------------")
		print("-------------------------------------------")

except(KeyboardInterrupt, SystemExit):

	print("End program...")