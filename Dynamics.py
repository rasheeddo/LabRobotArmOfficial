from LabRobotArm import *
import time

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

####################################### Start the robot #############################################

WakeUpStatus = robotarm.RobotArmAwake()		# If robot is in sleep position, this command would wake the robot up
time.sleep(0.5)

if WakeUpStatus:
	robotarm.StandByPos()						# Move the robot to the stand by position
	Run = True
else:
	Run = False

Mem_X = [None]*20
Mem_Y = [None]*20
Mem_Z = [None]*20

# We are going to input X,Y,Z,X-rot,Y-rot,Z-rot to inverse kinematics
# Keep all variables constant and change just X
# The robot would keep moving in X direction (not perfect linear translation but looks natural move)
Yconst = 250.0
Zconst = 150.0
X6_ROT = 0.0
Y6_ROT = 0.0
Z6_ROT = 0.0

try:

	while Run:

		# Move X in 6 points, the value must not over than Workspace limitation
		Mem_X[0] = 250.0
		Mem_Y[0] = Yconst
		Mem_Z[0] = Zconst

		Mem_X[1] = 0.0
		Mem_Y[1] = Yconst
		Mem_Z[1] = Zconst

		Mem_X[2] = -250.0
		Mem_Y[2] = Yconst
		Mem_Z[2] = Zconst

		Mem_X[3] = 0.0
		Mem_Y[3] = Yconst
		Mem_Z[3] = Zconst

		Mem_X[4] = 250.0
		Mem_Y[4] = Yconst
		Mem_Z[4] = Zconst

		Mem_X[5] = 100.0
		Mem_Y[5] = Yconst
		Mem_Z[5] = Zconst

		# Read starting angle 
		PreAng = robotarm.ReadAngle()
		PreAng1 = PreAng[0]
		PreAng2 = PreAng[1]
		PreAng3 = PreAng[2]
		PreAng4 = PreAng[3]
		PreAng5 = PreAng[4]
		PreAng6 = PreAng[5]

		for K in range(0,6):
			startTime = time.time()
			# Check the input from user whether valid or not by using WorkspaceLimitation function
			# If the input X,Y,Z are valid, then it will return same value
			# If not, it will return None type
			XYZ = robotarm.WorkspaceLimitation(Mem_X[K],Mem_Y[K],Mem_Z[K])
			Com_X = XYZ[0]
			Com_Y = XYZ[1]
			Com_Z = XYZ[2]
			if Com_X == None or Com_Y == None or Com_Z == None:
				break
			else:
				Mem_ANG = robotarm.RobotArmINV(Com_X,Com_Y,Com_Z,X6_ROT,Y6_ROT,Z6_ROT)

				GoalPos1 = Mem_ANG[0]
				GoalPos2 = Mem_ANG[1]
				GoalPos3 = Mem_ANG[2]
				GoalPos4 = Mem_ANG[3]
				GoalPos5 = Mem_ANG[4]
				GoalPos6 = Mem_ANG[5]
				# Calculate difference between goal position and current position
				DelPos1 = robotarm.DeltaPos(PreAng1,GoalPos1)
				DelPos2 = robotarm.DeltaPos(PreAng2,GoalPos2)
				DelPos3 = robotarm.DeltaPos(PreAng3,GoalPos3)
				DelPos4 = robotarm.DeltaPos(PreAng4,GoalPos4)
				DelPos5 = robotarm.DeltaPos(PreAng5,GoalPos5)
				DelPos6 = robotarm.DeltaPos(PreAng6,GoalPos6)
				# specify the velocity and acceralation standard
				VSTD = 100
				ASTD = 12
				# Calculate how much velocity and accerelation in each joint to make it stop at the same time
				# TrajectoryGeneration3 is a function to generate joint velocity profile to make robot looks natural movement
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
				# Set calculated profile to each servo
				robotarm.SetProfile1(V1,A1)
				robotarm.SetProfile2(V2,A2)
				robotarm.SetProfile3(V3,A3)
				robotarm.SetProfile4(V4,A4)
				robotarm.SetProfile5(V5,A5)
				robotarm.SetProfile6(V6,A6)
				# Drive servo to goal position according to that velocity
				robotarm.RunServo(GoalPos1,GoalPos2,GoalPos3,GoalPos4,GoalPos5,GoalPos6)
				print("Move to point %d" %K )
				print("DelPos1: %d" %DelPos1)
				print("DelPos2: %d" %DelPos2)
				print("DelPos3: %d" %DelPos3)
				print("DelPos4: %d" %DelPos4)
				print("DelPos5: %d" %DelPos5)
				print("DelPos6: %d" %DelPos6)
				# Check which kind of motion does the servo moves
				# It can be Step, Rectangle, Trapezoidal, Triangle
				# We expect to let it shows a Trapezoidal profile, but any profile is fine.
				MoveType1 = robotarm.MovingStatus1()
				MoveType2 = robotarm.MovingStatus2()
				MoveType3 = robotarm.MovingStatus3()
				MoveType4 = robotarm.MovingStatus4()
				MoveType5 = robotarm.MovingStatus5()
				MoveType6 = robotarm.MovingStatus6()

				MovingFlag = True
				Block1 = True
				Block2 = True
				Block3 = True
				Block4 = True
				Block5 = True
				Block6 = True
				# This loop will check that all of the servo is in goal position or not
				# If it was, it would print how long does it takes to travel
				# Normally, if all servo move with Trapezoidal profile, it will stop almost at the same time
				# If it uses other profile, you can notice some lead / lag in few seconds.
				while MovingFlag:
					Move1 = robotarm.IsMoving1()
					Move2 = robotarm.IsMoving2()
					Move3 = robotarm.IsMoving3()
					Move4 = robotarm.IsMoving4()
					Move5 = robotarm.IsMoving5()
					Move6 = robotarm.IsMoving6()

					if Move1 == 0 and Block1 == True:
						endTime1 = time.time()
						period1 = endTime1 - startTime
						Block1 = False
					if Move2 == 0 and Block2 == True:
						endTime2 = time.time()
						period2 = endTime2 - startTime 
						Block2 = False 
					if Move3 == 0 and Block3 == True:
						endTime3 = time.time()
						period3 = endTime3 - startTime
						Block3 = False
					if Move4 == 0 and Block4 == True:
						endTime4 = time.time()
						period4 = endTime4 - startTime
					 	Block4 = False
					if Move5 == 0 and Block5 == True:
						endTime5 = time.time()
						period5 = endTime5 - startTime 
						Block5 = False 
					if Move6 == 0 and Block6 == True:
						endTime6 = time.time()
					 	period6 = endTime6 - startTime
						Block6 = False

					if Move1 == 0 and Move2 == 0 and Move3 == 0 and Move4 == 0 and Move5 == 0 and Move6 == 0:
						MovingFlag = False
						print("Period1: %f" %period1)
						print("Period2: %f" %period2) 
						print("Period3: %f" %period3)
						print("Period4: %f" %period4)
						print("Period5: %f" %period5) 
						print("Period6: %f" %period6)
						print("Finished point %d" %K)
						print("--------------------------------------------------")
						print("--------------------------------------------------")

				# Update the current position
				PreAng1 = GoalPos1
				PreAng2 = GoalPos2
				PreAng3 = GoalPos3
				PreAng4 = GoalPos4
				PreAng5 = GoalPos5
				PreAng6 = GoalPos6
				# When it's done with 6 points, it will reset to 0 and keep running from start again
				if K == 5:
					K = 0
					robotarm.StandByPos()

# To stop the robot, you just hit Ctrl+C then the robot will stop there
# Torque is still ON, so you can turn your power supply or switch off to release the torque
# BE CAREFUL, robot will fall down immediately after you turn off
except(KeyboardInterrupt, SystemExit):
	print("End program...")