import time
from dynamixel_sdk import *                    # Uses Dynamixel SDK library
import numpy
import math
import pygame

class RobotArm:

	def __init__(self):
		#### Initialize joy stick ####
		pygame.init()
		self.j = pygame.joystick.Joystick(0)
		self.j.init()
		############################## Math Constant ####################################
		self.rad2deg = 180/math.pi
		self.deg2rad = math.pi/180
		self.pi = math.pi
		########################## Robot's DH parameters ################################
		self.a1 = 0
		self.d1 = 0
		self.alp1 = self.pi/2

		self.a2 = 212
		self.d2 = 0
		self.alp2 = 0

		self.a3 = 74
		self.d3 = 0
		self.alp3 = self.pi/2

		self.a4 = 0
		self.d4 = 139
		self.alp4 = -self.pi/2

		self.a5 = 0
		self.d5 = 0
		self.alp5 = self.pi/2

		self.a6 = 0
		self.d6 = 200
		self.alp6 = 0
		########################## Robot's home position ################################
		# Make an empty buffer for Home_Ang
		self.Home_Ang1 = [None]*10
		self.Home_Ang2 = [None]*10
		self.Home_Ang3 = [None]*10
		self.Home_Ang4 = [None]*10
		self.Home_Ang5 = [None]*10
		self.Home_Ang6 = [None]*10

				## If you set the new home position, these Home_Ang must be changed
		# Point: 0
		self.Home_Ang1[0] = 278.857143
		self.Home_Ang2[0] = 194.637363
		self.Home_Ang3[0] = 155.692308
		self.Home_Ang4[0] = 183.648352
		self.Home_Ang5[0] = 103.032967
		self.Home_Ang6[0] = 181.890110
		 
		# Point: 1
		self.Home_Ang1[1] = 285.450549
		self.Home_Ang2[1] = 175.472527
		self.Home_Ang3[1] = 103.824176
		self.Home_Ang4[1] = 188.483516
		self.Home_Ang5[1] = 174.417582
		self.Home_Ang6[1] = 181.890110


				## Paste the new home position up there

		# ELiminate [None] element in the list
		self.Home_Ang1 = [x for x in self.Home_Ang1 if x is not None]
		self.Home_Ang2 = [x for x in self.Home_Ang2 if x is not None]
		self.Home_Ang3 = [x for x in self.Home_Ang3 if x is not None]
		self.Home_Ang4 = [x for x in self.Home_Ang4 if x is not None]
		self.Home_Ang5 = [x for x in self.Home_Ang5 if x is not None]
		self.Home_Ang6 = [x for x in self.Home_Ang6 if x is not None]

		#################### Set Servo Configuration #############################
		# Control table address

		self.ADDR_PRO_MODEL_NUMBER       		 = 0

		self.ADDR_PRO_OPERATING_MODE     		 = 11

		self.ADDR_PRO_CURRENT_LIMIT      		 = 38
		self.ADDR_PRO_ACCELERATION_LIMIT 		 = 40
		self.ADDR_PRO_VELOCITY_LIMIT     		 = 44

		self.ADDR_PRO_TORQUE_ENABLE      		 = 64               # Control table address is different in Dynamixel model

		self.ADDR_PRO_POSITION_D_GAIN    		 = 80
		self.ADDR_PRO_POSITION_I_GAIN    		 = 82
		self.ADDR_PRO_POSITION_P_GAIN    		 = 84

		self.ADDR_PRO_FEEDFORWARD_2nd_GAIN		 = 88
		self.ADDR_PRO_FEEDFORWARD_1st_GAIN 		 = 90

		self.ADDR_PRO_GOAL_CURRENT       		 = 102
		self.ADDR_PRO_GOAL_VELOCITY      		 = 104

		self.ADDR_PRO_PROFILE_ACCELERATION  	 = 108
		self.ADDR_PRO_PROFILE_VELOCITY   		 = 112

		self.ADDR_PRO_GOAL_POSITION      		 = 116
		self.ADDR_PRO_MOVING             		 = 122
		self.ADDR_PRO_MOVING_STATUS       		 = 123

		self.ADDR_PRO_PRESENT_CURRENT    		 = 126 
		self.ADDR_PRO_PRESENT_POSITION   		 = 132


		# Operating Mode Number
		self.CURRENT_CONTROL                     = 0
		self.POSITION_CONTROL                    = 3 # Default
		self.CURRENT_BASED_POSITION_CONTROL      = 5

		# Protocol version
		self.PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

		# ID
		self.DXL1_ID                      = 1                          
		self.DXL2_ID                      = 2                             
		self.DXL3_ID                      = 3                            
		self.DXL4_ID                      = 4
		self.DXL5_ID                      = 5
		self.DXL6_ID                      = 6
		self.DXL7_ID                      = 7

		self.BAUDRATE                    = 57600             # Dynamixel default self.BAUDRATE : 57600
		self.DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
		                                                	 # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
		self.TORQUE_ENABLE               = 1                 # Value for enabling the torque
		self.TORQUE_DISABLE              = 0                 # Value for disabling the torque

		# Initialize PortHandler instance
		# Set the port path
		# Get methods and members of PortHandlerLinux or PortHandlerWindows
		self.portHandler = PortHandler(self.DEVICENAME)

		# Initialize PacketHandler instance
		# Set the protocol version
		# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
		self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

		# Open port
		if self.portHandler.openPort():
			print("Succeeded to open the port")
		else:
			print("Failed to open the port")
			print("Press any key to terminate...")
			getch()
			quit()

		# Set port BAUDRATE
		if self.portHandler.setBaudRate(self.BAUDRATE):
			print("Succeeded to change the BAUDRATE")
		else:
			print("Failed to change the BAUDRATE")
			print("Press any key to terminate...")
			getch()
			quit()

		###################### Set Operating Mode for the robot #################################
		# It can be POSITION_CONTROL or CURRENT_BASED_POSITION_CONTROL
		self.SetOperatingMode(self.POSITION_CONTROL)
		# For Gripper, it is always CURRENT_BASED_POSITION_CONTROL, and already set, so doesn't need to be set again

		###################### Set Gripper Configuration #################################
		self.SetProfile7(200,60)
		self.SetPID7(2000,30,2000)
		self.SetGoalCurrentGripper(90)

		######################## Set Torque configuration ################################
		self.SetCurrentLimit1(800)
		self.SetCurrentLimit2(1000) #Servo2 requires a lot of current, always set it as nearly maximum
		self.SetCurrentLimit3(800)
		self.SetCurrentLimit4(800)
		self.SetCurrentLimit5(800)
		self.SetCurrentLimit6(800)

		# Goal current shouldn't be too low or too high, the robot must have enough torque to drive itself and take some load, but not too high to damage.
		self.GoalCur1 = 150
		self.GoalCur2 = 800
		self.GoalCur3 = 250
		self.GoalCur4 = 150
		self.GoalCur5 = 250
		self.GoalCur6 = 150

		self.SetGoalCurrent1(self.GoalCur1)
		self.SetGoalCurrent2(self.GoalCur2)
		self.SetGoalCurrent3(self.GoalCur3)
		self.SetGoalCurrent4(self.GoalCur4)
		self.SetGoalCurrent5(self.GoalCur5)
		self.SetGoalCurrent6(self.GoalCur6)

		####################### Set PID & FF Gain configuration #############################
		Present_mode = self.GetOperatingMode()
		if Present_mode == 3:
			P_Gain1 = 1500    #800 default
			I_Gain1 = 100     #0 default
			D_Gain1 = 4000   #4700 default

			P_Gain2 = 1500    #800 default
			I_Gain2 = 100     #0 default
			D_Gain2 = 4000   #4700 default

			P_Gain3 = 1500    #800 default
			I_Gain3 = 100     #0 default
			D_Gain3 = 4000   #4700 default

			P_Gain4 = 1500    #800 default
			I_Gain4 = 100     #0 default
			D_Gain4 = 4000   #4700 default

			P_Gain5 = 1500    #800 default
			I_Gain5 = 100     #0 default
			D_Gain5 = 4000   #4700 default

			P_Gain6 = 1500    #800 default
			I_Gain6 = 100     #0 default
			D_Gain6 = 4000   #4700 default

			# Set Feedforward gain
			FF1_Gain1 = 100
			FF2_Gain1 = 50

			FF1_Gain2 = 100
			FF2_Gain2 = 50

			FF1_Gain3 = 100
			FF2_Gain3 = 50

			FF1_Gain4 = 100
			FF2_Gain4 = 50

			FF1_Gain5 = 100
			FF2_Gain5 = 50

			FF1_Gain6 = 100
			FF2_Gain6 = 50

		elif Present_mode == 5:
			# Set PID gain 
			P_Gain1 = 800    #800 default
			I_Gain1 = 100     #0 default
			D_Gain1 = 9000   #4700 default

			P_Gain2 = 800    #800 default
			I_Gain2 = 100     #0 default
			D_Gain2 = 9000   #4700 default

			P_Gain3 = 800    #800 default
			I_Gain3 = 100     #0 default
			D_Gain3 = 9000   #4700 default

			P_Gain4 = 800    #800 default
			I_Gain4 = 50     #0 default
			D_Gain4 = 7000   #4700 default

			P_Gain5 = 800    #800 default
			I_Gain5 = 50     #0 default
			D_Gain5 = 7000   #4700 default

			P_Gain6 = 800    #800 default
			I_Gain6 = 50     #0 default
			D_Gain6 = 7000   #4700 default

			# Set Feedforward gain
			FF1_Gain1 = 300
			FF2_Gain1 = 50

			FF1_Gain2 = 300
			FF2_Gain2 = 50

			FF1_Gain3 = 300
			FF2_Gain3 = 50

			FF1_Gain4 = 300
			FF2_Gain4 = 50

			FF1_Gain5 = 300
			FF2_Gain5 = 50

			FF1_Gain6 = 300
			FF2_Gain6 = 50

		# Set PID gain 
		self.SetPID1(P_Gain1,I_Gain1,D_Gain1)
		self.SetPID2(P_Gain2,I_Gain2,D_Gain2)
		self.SetPID3(P_Gain3,I_Gain3,D_Gain3)
		self.SetPID4(P_Gain4,I_Gain4,D_Gain4)
		self.SetPID5(P_Gain5,I_Gain5,D_Gain5)
		self.SetPID6(P_Gain6,I_Gain6,D_Gain6)

		# Set Feedforward gain
		self.SetFFGain1(FF1_Gain1,FF2_Gain1)
		self.SetFFGain2(FF1_Gain2,FF2_Gain2)
		self.SetFFGain3(FF1_Gain3,FF2_Gain3)
		self.SetFFGain4(FF1_Gain4,FF2_Gain4)
		self.SetFFGain5(FF1_Gain5,FF2_Gain5)
		self.SetFFGain6(FF1_Gain6,FF2_Gain6)



	def getButton(self):
		#Read input from the two joysticks
		pygame.event.pump()
		button0 = self.j.get_button(0)
		button1 = self.j.get_button(1)
		button2 = self.j.get_button(2)
		button3 = self.j.get_button(3)
		button4 = self.j.get_button(4)
		button5 = self.j.get_button(5)
		button6 = self.j.get_button(6)
		button7 = self.j.get_button(7)
		button8 = self.j.get_button(8)
		button9 = self.j.get_button(9)
		button10 = self.j.get_button(10)
		joy_button = [button0, button1, button2, button3, button4, button5, button6, button7,button8, button9, button10]

		return joy_button

	def getAxis(self):
		#Read input from the two joysticks
		pygame.event.pump()
		axis0 = self.j.get_axis(0)
		axis1 = self.j.get_axis(1)
		axis2 = self.j.get_axis(2)
		axis3 = self.j.get_axis(4)
		axis4 = self.j.get_axis(3)
		axis5 = self.j.get_axis(5)
		joy_axis = [axis0, axis1, axis2, axis3, axis4, axis5]
		return joy_axis

	def getHat(self):
		pygame.event.pump()
		hat0 = self.j.get_hat(0)
		joy_hat = hat0

		return joy_hat

	def map(self, val, in_min, in_max, out_min, out_max):

		return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

	def ReadModelNumber(self,ID):
		# MX106 -> 321
		# XM540 -> 1120
		# XM430 -> 1020
		model_number, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, ID, self.ADDR_PRO_MODEL_NUMBER)

		if model_number == 321:
			print("MX106 is on Servo ID%d" %ID)
		elif model_number == 1020:
			print("XM430 is on Servo ID%d" %ID)
		elif model_number == 1120:
			print("XM540 is on Servo ID%d" %ID)
		else:
			print("Unknown model...")

		return model_number

	def SetOperatingMode(self,MODE):

		self.TorqueOff()

		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_OPERATING_MODE, MODE)
		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_OPERATING_MODE, MODE)
		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_OPERATING_MODE, MODE)
		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_PRO_OPERATING_MODE, MODE)
		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL5_ID, self.ADDR_PRO_OPERATING_MODE, MODE)
		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL6_ID, self.ADDR_PRO_OPERATING_MODE, MODE)


		present_mode, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_OPERATING_MODE)
		if present_mode == 0:
		    # Current (Torque) Control Mode
			print("Now Operating Mode is Torque Control")
		elif present_mode == 3:
		    # Position Control Mode
			print("Now Operating Mode is Position Control")
		elif present_mode == 5:
		    # Current-based Position Control Mode
			print("Now Operating Mode is Current-based Position Control")
		else:
			print("In other Mode that didn't set!")

	def GetOperatingMode(self):
		present_mode, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_OPERATING_MODE)

		return present_mode


	def SetCurrentLimit1(self,ComValue):

		# Each model has difference range and unit
		# MX106 -> [Unit : 3.36mA], [Range(ComValue) : 0 ~ 2047], [Range(ampere) : 0 ~ 6.8779]
		# XM540 -> [Unit : 2.69mA], [Range(ComValue) : 0 ~ 2047], [Range(ampere) : 0 ~ 5.5064]
		# XM430 -> [Unit : 2.69mA], [Range(ComValue) : 0 ~ 1193], [Range(ampere) : 0 ~ 3.2092]

		UnitMX106 = 3.36 
		UnitXM540 = 2.69
		UnitXM430 = 2.69

		# Check what is the model of this servo
		ServoModel = self.ReadModelNumber(1)
		if ServoModel == 321:
			SetUnit = UnitMX106
		elif ServoModel == 1020:
			SetUnit = UnitXM430
		elif ServoModel == 1120:
			SetUnit = UnitXM540

		# user input is in ComValue unit

		# Write ComValue
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_CURRENT_LIMIT,ComValue)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error))

		# Read and confirm input value
		dxl_current_limit, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_CURRENT_LIMIT)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error))

		CurLimit = dxl_current_limit*(SetUnit/1000.0)
		print("Current Limit 1 [ComValue] : %f" %dxl_current_limit)
		print("Current Limit 1 [Ampere] : %f" %CurLimit)

	def SetCurrentLimit2(self,ComValue):

		# Each model has difference range and unit
		# MX106 -> [Unit : 3.36mA], [Range(ComValue) : 0 ~ 2047], [Range(ampere) : 0 ~ 6.8779]
		# XM540 -> [Unit : 2.69mA], [Range(ComValue) : 0 ~ 2047], [Range(ampere) : 0 ~ 5.5064]
		# XM430 -> [Unit : 2.69mA], [Range(ComValue) : 0 ~ 1193], [Range(ampere) : 0 ~ 3.2092]

		UnitMX106 = 3.36 
		UnitXM540 = 2.69
		UnitXM430 = 2.69

		# Check what is the model of this servo
		ServoModel = self.ReadModelNumber(2)
		if ServoModel == 321:
			SetUnit = UnitMX106
		elif ServoModel == 1020:
			SetUnit = UnitXM430
		elif ServoModel == 1120:
			SetUnit = UnitXM540

		# user input is in ComValue unit

		# Write ComValue
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_CURRENT_LIMIT,ComValue)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error))

		# Read and confirm input value
		dxl_current_limit, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_CURRENT_LIMIT)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error))

		CurLimit = dxl_current_limit*(SetUnit/1000.0)
		print("Current Limit 2 [ComValue] : %f" %dxl_current_limit)
		print("Current Limit 2 [Ampere] : %f" %CurLimit)

	def SetCurrentLimit3(self,ComValue):

		# Each model has difference range and unit
		# MX106 -> [Unit : 3.36mA], [Range(ComValue) : 0 ~ 2047], [Range(ampere) : 0 ~ 6.8779]
		# XM540 -> [Unit : 2.69mA], [Range(ComValue) : 0 ~ 2047], [Range(ampere) : 0 ~ 5.5064]
		# XM430 -> [Unit : 2.69mA], [Range(ComValue) : 0 ~ 1193], [Range(ampere) : 0 ~ 3.2092]

		UnitMX106 = 3.36 
		UnitXM540 = 2.69
		UnitXM430 = 2.69

		# Check what is the model of this servo
		ServoModel = self.ReadModelNumber(3)
		if ServoModel == 321:
			SetUnit = UnitMX106
		elif ServoModel == 1020:
			SetUnit = UnitXM430
		elif ServoModel == 1120:
			SetUnit = UnitXM540

		# user input is in ComValue unit

		# Write ComValue
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_CURRENT_LIMIT,ComValue)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error))

		# Read and confirm input value
		dxl_current_limit, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_CURRENT_LIMIT)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error))

		CurLimit = dxl_current_limit*(SetUnit/1000.0)
		print("Current Limit 3 [ComValue] : %f" %dxl_current_limit)
		print("Current Limit 3 [Ampere] : %f" %CurLimit)

	def SetCurrentLimit4(self,ComValue):

		# Each model has difference range and unit
		# MX106 -> [Unit : 3.36mA], [Range(ComValue) : 0 ~ 2047], [Range(ampere) : 0 ~ 6.8779]
		# XM540 -> [Unit : 2.69mA], [Range(ComValue) : 0 ~ 2047], [Range(ampere) : 0 ~ 5.5064]
		# XM430 -> [Unit : 2.69mA], [Range(ComValue) : 0 ~ 1193], [Range(ampere) : 0 ~ 3.2092]

		UnitMX106 = 3.36 
		UnitXM540 = 2.69
		UnitXM430 = 2.69

		# Check what is the model of this servo
		ServoModel = self.ReadModelNumber(4)
		if ServoModel == 321:
			SetUnit = UnitMX106
		elif ServoModel == 1020:
			SetUnit = UnitXM430
		elif ServoModel == 1120:
			SetUnit = UnitXM540

		# user input is in ComValue unit

		# Write ComValue
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_PRO_CURRENT_LIMIT,ComValue)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error))

		# Read and confirm input value
		dxl_current_limit, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_PRO_CURRENT_LIMIT)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error))

		CurLimit = dxl_current_limit*(SetUnit/1000.0)
		print("Current Limit 4 [ComValue] : %f" %dxl_current_limit)
		print("Current Limit 4 [Ampere] : %f" %CurLimit)

	def SetCurrentLimit5(self,ComValue):

		# Each model has difference range and unit
		# MX106 -> [Unit : 3.36mA], [Range(ComValue) : 0 ~ 2047], [Range(ampere) : 0 ~ 6.8779]
		# XM540 -> [Unit : 2.69mA], [Range(ComValue) : 0 ~ 2047], [Range(ampere) : 0 ~ 5.5064]
		# XM430 -> [Unit : 2.69mA], [Range(ComValue) : 0 ~ 1193], [Range(ampere) : 0 ~ 3.2092]

		UnitMX106 = 3.36 
		UnitXM540 = 2.69
		UnitXM430 = 2.69

		# Check what is the model of this servo
		ServoModel = self.ReadModelNumber(5)
		if ServoModel == 321:
			SetUnit = UnitMX106
		elif ServoModel == 1020:
			SetUnit = UnitXM430
		elif ServoModel == 1120:
			SetUnit = UnitXM540

		# user input is in ComValue unit

		# Write ComValue
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL5_ID, self.ADDR_PRO_CURRENT_LIMIT,ComValue)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error))

		# Read and confirm input value
		dxl_current_limit, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL5_ID, self.ADDR_PRO_CURRENT_LIMIT)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error))

		CurLimit = dxl_current_limit*(SetUnit/1000.0)
		print("Current Limit 5 [ComValue] : %f" %dxl_current_limit)
		print("Current Limit 5 [Ampere] : %f" %CurLimit)

	def SetCurrentLimit6(self,ComValue):

		# Each model has difference range and unit
		# MX106 -> [Unit : 3.36mA], [Range(ComValue) : 0 ~ 2047], [Range(ampere) : 0 ~ 6.8779]
		# XM540 -> [Unit : 2.69mA], [Range(ComValue) : 0 ~ 2047], [Range(ampere) : 0 ~ 5.5064]
		# XM430 -> [Unit : 2.69mA], [Range(ComValue) : 0 ~ 1193], [Range(ampere) : 0 ~ 3.2092]

		UnitMX106 = 3.36 
		UnitXM540 = 2.69
		UnitXM430 = 2.69

		# Check what is the model of this servo
		ServoModel = self.ReadModelNumber(6)
		if ServoModel == 321:
			SetUnit = UnitMX106
		elif ServoModel == 1020:
			SetUnit = UnitXM430
		elif ServoModel == 1120:
			SetUnit = UnitXM540

		# user input is in ComValue unit

		# Write ComValue
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL6_ID, self.ADDR_PRO_CURRENT_LIMIT,ComValue)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error))

		# Read and confirm input value
		dxl_current_limit, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL6_ID, self.ADDR_PRO_CURRENT_LIMIT)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error))

		CurLimit = dxl_current_limit*(SetUnit/1000.0)
		print("Current Limit 6 [ComValue] : %f" %dxl_current_limit)
		print("Current Limit 6 [Ampere] : %f" %CurLimit)



	def SetGoalCurrent1(self,ComValue):

		# Each model has difference range and unit
		# MX106 -> [Unit : 3.36mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]
		# XM540 -> [Unit : 2.69mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]
		# XM430 -> [Unit : 2.69mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]

		# user input is in ComValue unit

		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_GOAL_CURRENT, ComValue)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error))

	def SetGoalCurrent2(self,ComValue):

		# Each model has difference range and unit
		# MX106 -> [Unit : 3.36mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]
		# XM540 -> [Unit : 2.69mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]
		# XM430 -> [Unit : 2.69mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]

		# user input is in ComValue unit

		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_GOAL_CURRENT, ComValue)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error))

	def SetGoalCurrent3(self,ComValue):

		# Each model has difference range and unit
		# MX106 -> [Unit : 3.36mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]
		# XM540 -> [Unit : 2.69mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]
		# XM430 -> [Unit : 2.69mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]

		# user input is in ComValue unit

		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_GOAL_CURRENT, ComValue)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error))

	def SetGoalCurrent4(self,ComValue):

		# Each model has difference range and unit
		# MX106 -> [Unit : 3.36mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]
		# XM540 -> [Unit : 2.69mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]
		# XM430 -> [Unit : 2.69mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]

		# user input is in ComValue unit

		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_PRO_GOAL_CURRENT, ComValue)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error))

	def SetGoalCurrent5(self,ComValue):

		# Each model has difference range and unit
		# MX106 -> [Unit : 3.36mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]
		# XM540 -> [Unit : 2.69mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]
		# XM430 -> [Unit : 2.69mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]

		# user input is in ComValue unit

		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL5_ID, self.ADDR_PRO_GOAL_CURRENT, ComValue)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error))

	def SetGoalCurrent6(self,ComValue):

		# Each model has difference range and unit
		# MX106 -> [Unit : 3.36mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]
		# XM540 -> [Unit : 2.69mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]
		# XM430 -> [Unit : 2.69mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]

		# user input is in ComValue unit

		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL6_ID, self.ADDR_PRO_GOAL_CURRENT, ComValue)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error))

	def SetGoalCurrentGripper(self,SetCur):

		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL7_ID, self.ADDR_PRO_GOAL_CURRENT, SetCur)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error))
		else:
			print("Goal Current is set")


	def ReadAllCurrent(self):
		dxl_present_current1, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_PRESENT_CURRENT)
		dxl_present_current2, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_PRESENT_CURRENT)
		dxl_present_current3, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_PRESENT_CURRENT)
		dxl_present_current4, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_PRO_PRESENT_CURRENT)
		dxl_present_current5, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL5_ID, self.ADDR_PRO_PRESENT_CURRENT)
		dxl_present_current6, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL6_ID, self.ADDR_PRO_PRESENT_CURRENT)
		# 1
		if dxl_present_current1 > 32767:
			com_signed_value1 = dxl_present_current1 - 65536
		else:
			com_signed_value1 = dxl_present_current1
		# 2
		if dxl_present_current2 > 32767:
			com_signed_value2 = dxl_present_current2 - 65536
		else:
			com_signed_value2 = dxl_present_current2
		# 3
		if dxl_present_current3 > 32767:
			com_signed_value3 = dxl_present_current3 - 65536
		else:
			com_signed_value3 = dxl_present_current3
		# 4
		if dxl_present_current4 > 32767:
			com_signed_value4 = dxl_present_current4 - 65536
		else:
			com_signed_value4 = dxl_present_current4
		# 5
		if dxl_present_current5 > 32767:
			com_signed_value5 = dxl_present_current5 - 65536
		else:
			com_signed_value5 = dxl_present_current5
		# 6
		if dxl_present_current6 > 32767:
			com_signed_value6 = dxl_present_current6 - 65536
		else:
			com_signed_value6 = dxl_present_current6

		#PresentCur = com_signed_value*(SetUnit/1000.0)
		print("Present current1 [ComValue_signed]: %f" %com_signed_value1)
		print("Present current2 [ComValue_signed]: %f" %com_signed_value2)
		print("Present current3 [ComValue_signed]: %f" %com_signed_value3)
		print("Present current4 [ComValue_signed]: %f" %com_signed_value4)
		print("Present current5 [ComValue_signed]: %f" %com_signed_value5)
		print("Present current6 [ComValue_signed]: %f" %com_signed_value6)

		return com_signed_value1, com_signed_value2, com_signed_value3, com_signed_value4, com_signed_value5, com_signed_value6

	def ReadCurrent1(self):

		# Each model has difference range and unit
		# MX106 -> [Unit : 3.36mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]
		# XM540 -> [Unit : 2.69mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]
		# XM430 -> [Unit : 2.69mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]
		'''
		UnitMX106 = 3.36 
		UnitXM540 = 2.69
		UnitXM430 = 2.69
		
		# Check what is the model of this servo
		ServoModel = ReadModelNumber(1)
		if ServoModel == 321:
			SetUnit = UnitMX106
		elif ServoModel == 1020:
			SetUnit = UnitXM430
		elif ServoModel == 1120:
			SetUnit = UnitXM540
		'''
		dxl_present_current, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_PRESENT_CURRENT)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error))

		if dxl_present_current > 32767:
			com_signed_value = dxl_present_current - 65536
		else:
			com_signed_value = dxl_present_current

		#PresentCur = com_signed_value*(SetUnit/1000.0)
		print("Present current1 [ComValue_signed]: %f" %com_signed_value)
		#print("Present current1 [Ampere]: %f" %PresentCur)

		return com_signed_value

	def ReadCurrent2(self):

		# Each model has difference range and unit
		# MX106 -> [Unit : 3.36mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]
		# XM540 -> [Unit : 2.69mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]
		# XM430 -> [Unit : 2.69mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]
		'''
		UnitMX106 = 3.36 
		UnitXM540 = 2.69
		UnitXM430 = 2.69
		
		# Check what is the model of this servo
		ServoModel = ReadModelNumber(2)
		if ServoModel == 321:
			SetUnit = UnitMX106
		elif ServoModel == 1020:
			SetUnit = UnitXM430
		elif ServoModel == 1120:
			SetUnit = UnitXM540
		'''
		dxl_present_current, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_PRESENT_CURRENT)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error))

		if dxl_present_current > 32767:
			com_signed_value = dxl_present_current - 65536
		else:
			com_signed_value = dxl_present_current

		#PresentCur = com_signed_value*(SetUnit/1000.0)
		print("Present current2 [ComValue_signed]: %f" %com_signed_value)
		#print("Present current [Ampere]: %f" %PresentCur)

		return com_signed_value

	def ReadCurrent3(self):

		# Each model has difference range and unit
		# MX106 -> [Unit : 3.36mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]
		# XM540 -> [Unit : 2.69mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]
		# XM430 -> [Unit : 2.69mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]
		'''
		UnitMX106 = 3.36 
		UnitXM540 = 2.69
		UnitXM430 = 2.69
		
		# Check what is the model of this servo
		ServoModel = ReadModelNumber(1)
		if ServoModel == 321:
			SetUnit = UnitMX106
		elif ServoModel == 1020:
			SetUnit = UnitXM430
		elif ServoModel == 1120:
			SetUnit = UnitXM540
		'''
		dxl_present_current, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_PRESENT_CURRENT)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error))

		if dxl_present_current > 32767:
			com_signed_value = dxl_present_current - 65536
		else:
			com_signed_value = dxl_present_current

		#PresentCur = com_signed_value*(SetUnit/1000.0)
		print("Present current3 [ComValue_signed]: %f" %com_signed_value)
		#print("Present current [Ampere]: %f" %PresentCur)

		return com_signed_value

	def ReadCurrent4(self):

		# Each model has difference range and unit
		# MX106 -> [Unit : 3.36mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]
		# XM540 -> [Unit : 2.69mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]
		# XM430 -> [Unit : 2.69mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]
		'''
		UnitMX106 = 3.36 
		UnitXM540 = 2.69
		UnitXM430 = 2.69
		
		# Check what is the model of this servo
		ServoModel = ReadModelNumber(1)
		if ServoModel == 321:
			SetUnit = UnitMX106
		elif ServoModel == 1020:
			SetUnit = UnitXM430
		elif ServoModel == 1120:
			SetUnit = UnitXM540
		'''
		dxl_present_current, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_PRO_PRESENT_CURRENT)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error))

		if dxl_present_current > 32767:
			com_signed_value = dxl_present_current - 65536
		else:
			com_signed_value = dxl_present_current

		#PresentCur = com_signed_value*(SetUnit/1000.0)
		print("Present current4 [ComValue_signed]: %f" %com_signed_value)
		#print("Present current [Ampere]: %f" %PresentCur)

		return com_signed_value

	def ReadCurrent5(self):

		# Each model has difference range and unit
		# MX106 -> [Unit : 3.36mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]
		# XM540 -> [Unit : 2.69mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]
		# XM430 -> [Unit : 2.69mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]
		'''
		UnitMX106 = 3.36 
		UnitXM540 = 2.69
		UnitXM430 = 2.69
		
		# Check what is the model of this servo
		ServoModel = ReadModelNumber(1)
		if ServoModel == 321:
			SetUnit = UnitMX106
		elif ServoModel == 1020:
			SetUnit = UnitXM430
		elif ServoModel == 1120:
			SetUnit = UnitXM540
		'''
		dxl_present_current, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL5_ID, self.ADDR_PRO_PRESENT_CURRENT)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error))

		if dxl_present_current > 32767:
			com_signed_value = dxl_present_current - 65536
		else:
			com_signed_value = dxl_present_current

		#PresentCur = com_signed_value*(SetUnit/1000.0)
		print("Present current5 [ComValue_signed]: %f" %com_signed_value)
		#print("Present current [Ampere]: %f" %PresentCur)

		return com_signed_value

	def ReadCurrent6(self):

		# Each model has difference range and unit
		# MX106 -> [Unit : 3.36mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]
		# XM540 -> [Unit : 2.69mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]
		# XM430 -> [Unit : 2.69mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]
		'''
		UnitMX106 = 3.36 
		UnitXM540 = 2.69
		UnitXM430 = 2.69
		
		# Check what is the model of this servo
		ServoModel = ReadModelNumber(1)
		if ServoModel == 321:
			SetUnit = UnitMX106
		elif ServoModel == 1020:
			SetUnit = UnitXM430
		elif ServoModel == 1120:
			SetUnit = UnitXM540
		'''
		dxl_present_current, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL6_ID, self.ADDR_PRO_PRESENT_CURRENT)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error))

		if dxl_present_current > 32767:
			com_signed_value = dxl_present_current - 65536
		else:
			com_signed_value = dxl_present_current

		#PresentCur = com_signed_value*(SetUnit/1000.0)
		print("Present current6 [ComValue_signed]: %f" %com_signed_value)
		#print("Present current [Ampere]: %f" %PresentCur)

		return com_signed_value

	def ReadCurrentGripper(self):

		# Each model has difference range and unit
		# MX106 -> [Unit : 3.36mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]
		# XM540 -> [Unit : 2.69mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]
		# XM430 -> [Unit : 2.69mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]
		'''
		UnitMX106 = 3.36 
		UnitXM540 = 2.69
		UnitXM430 = 2.69
		
		# Check what is the model of this servo
		ServoModel = ReadModelNumber(1)
		if ServoModel == 321:
			SetUnit = UnitMX106
		elif ServoModel == 1020:
			SetUnit = UnitXM430
		elif ServoModel == 1120:
			SetUnit = UnitXM540
		'''
		dxl_present_current, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL7_ID, self.ADDR_PRO_PRESENT_CURRENT)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error))

		if dxl_present_current > 32767:
			com_signed_value = dxl_present_current - 65536
		else:
			com_signed_value = dxl_present_current

		#PresentCur = com_signed_value*(SetUnit/1000.0)
		print("Present current7 [ComValue_signed]: %f" %com_signed_value)
		#print("Present current [Ampere]: %f" %PresentCur)

		return com_signed_value

	def RobotArmFWD(self,deg1,deg2,deg3,deg4,deg5,deg6):


		########################## variables ################################
		q1 = (deg1-90)*self.deg2rad
		q2 = (deg2-90)*self.deg2rad
		q3 = (deg3-180)*self.deg2rad
		q4 = (deg4-180)*self.deg2rad
		q5 = (deg5-180)*self.deg2rad
		q6 = (deg6-180)*self.deg2rad

		T01 = numpy.matrix([[math.cos(q1), 0, math.sin(q1), self.a1*math.cos(q1)],[math.sin(q1), 0, -math.cos(q1), self.a1*math.sin(q1)],[0,1,0,0],[0,0,0,1]])
		T12 = numpy.matrix([[math.cos(q2), -math.sin(q2), 0, self.a2*math.cos(q2)],[math.sin(q2), math.cos(q2), 0, self.a2*math.sin(q2)],[0,0,1,0],[0,0,0,1]])
		T23 = numpy.matrix([[math.cos(q3), 0, math.sin(q3), self.a3*math.cos(q3)],[math.sin(q3), 0, -math.cos(q3), self.a3*math.sin(q3)],[0,1,0,0],[0,0,0,1]])
		T34 = numpy.matrix([[math.cos(q4), 0, -math.sin(q4), 0],[math.sin(q4), 0, math.cos(q4), 0],[0,-1,0,self.d4],[0,0,0,1]])
		T45 = numpy.matrix([[math.cos(q5), 0, math.sin(q5), 0],[math.sin(q5), 0, -math.cos(q5), 0],[0,1,0,0],[0,0,0,1]])
		T56 = numpy.matrix([[math.cos(q6), -math.sin(q6), 0, 0],[math.sin(q6), math.cos(q6), 0, 0],[0,0,1,self.d6],[0,0,0,1]])

		T02 = T01*T12
		T03 = T02*T23
		T04 = T03*T34
		T05 = T04*T45
		T06 = T05*T56

		P06 = [T06[0,3],T06[1,3],T06[2,3],T06[3,3]]
		qx = P06[0]
		qy = P06[1]
		qz = P06[2]

		R11 = T06[0,0]
		R12 = T06[0,1]
		R13 = T06[0,2]
		R21 = T06[1,0]
		R22 = T06[1,1]
		R23 = T06[1,2]
		R31 = T06[2,0]
		R32 = T06[2,1]
		R33 = T06[2,2]
		R06 = numpy.matrix([[R11,R12,R13],[R21,R22,R23],[R31,R32,R33]])
		'''
		print("R11: %f" %R11)
		print("R12: %f" %R12)
		print("R13: %f" %R13)
		print("R21: %f" %R21)
		print("R22: %f" %R22)
		print("R23: %f" %R23)
		print("R31: %f" %R31)
		print("R32: %f" %R32)
		print("R33: %f" %R33)
		print(" ")
		'''
		# Inverse RPY constant
		INV_RPY_const = numpy.matrix([[0,0,1],[1,0,0],[0,1,0]])

		TCP = INV_RPY_const*R06
		TCP11 = TCP[0,0]
		TCP12 = TCP[0,1]
		TCP13 = TCP[0,2]
		TCP21 = TCP[1,0]
		TCP22 = TCP[1,1]
		TCP23 = TCP[1,2]
		TCP31 = TCP[2,0]
		TCP32 = TCP[2,1]
		TCP33 = TCP[2,2]
		x6_rot = math.atan2(TCP32,TCP33)
		z6_rot = math.atan2(TCP21,TCP11)
		y6_rot = math.atan2(-TCP31, math.cos(z6_rot)*TCP11 + math.sin(z6_rot)*TCP21)

		x6_rot = x6_rot*self.rad2deg
		y6_rot = y6_rot*self.rad2deg
		z6_rot = z6_rot*self.rad2deg

		return qx, qy, qz, x6_rot, y6_rot, z6_rot

	def RobotArmINV(self,qx,qy,qz,x6_rot,y6_rot,z6_rot):

		rr = x6_rot*self.deg2rad
		pp = y6_rot*self.deg2rad
		yy = z6_rot*self.deg2rad

		E11 = math.cos(yy)*math.cos(pp)
		E12 = math.cos(yy)*math.sin(pp)*math.sin(rr)-math.cos(rr)*math.sin(yy)
		E13 = math.sin(rr)*math.sin(yy)+math.cos(rr)*math.cos(yy)*math.sin(pp)

		E21 = math.cos(pp)*math.sin(yy)
		E22 = math.cos(rr)*math.cos(yy) + math.sin(rr)*math.sin(yy)*math.sin(pp)
		E23 = math.cos(rr)*math.sin(yy)*math.sin(pp)-math.cos(yy)*math.sin(rr)

		E31 = -math.sin(pp)
		E32 = math.cos(pp)*math.sin(rr)
		E33 = math.cos(rr)*math.cos(pp)

		TCP = numpy.matrix([[E11,E12,E13],[E21,E22,E23],[E31,E32,E33]])

		r = 180*self.deg2rad
		p = -90*self.deg2rad
		y = 90*self.deg2rad

		psi = y
		theta = p
		phi = r

		F11 = math.cos(psi)*math.cos(theta)
		F12 = math.cos(psi)*math.sin(phi)*math.sin(theta)-math.cos(phi)*math.sin(psi)
		F13 = math.sin(phi)*math.sin(psi)+math.cos(phi)*math.cos(psi)*math.sin(theta)

		F21 = math.cos(theta)*math.sin(psi)
		F22 = math.cos(phi)*math.cos(psi) + math.sin(phi)*math.sin(psi)*math.sin(theta)
		F23 = math.cos(phi)*math.sin(psi)*math.sin(theta)-math.cos(psi)*math.sin(phi)

		F31 = -math.sin(theta)
		F32 = math.cos(theta)*math.sin(phi)
		F33 = math.cos(phi)*math.cos(theta)

		RPY = numpy.matrix([[F11,F12,F13],[F21,F22,F23],[F31,F32,F33]])

		RPY_TCP = RPY*TCP

		ux = RPY_TCP[0,0]
		uy = RPY_TCP[1,0]
		uz = RPY_TCP[2,0]

		vx = RPY_TCP[0,1]
		vy = RPY_TCP[1,1]
		vz = RPY_TCP[2,1]

		wx = RPY_TCP[0,2]
		wy = RPY_TCP[1,2]
		wz = RPY_TCP[2,2]

		px = qx - self.d6*wx
		py = qy - self.d6*wy
		pz = qz - self.d6*wz


		################################ Find q1 #####################################
		q1 = math.atan(py/px)

		if ((q1 <= self.pi) and (q1 >= 0)):
			q1_1 = q1
			q1_2 = q1 + self.pi
		else:
			q1_1 = q1 + self.pi
			q1_2 = q1
			q1 = q1 + self.pi
		'''
		if q1_1 < 0.000001:
		    q1_2 = 0

		if q1_2 < 0.000001:
		    q1_2 = 0
		'''
		deg1 = [None]*2
		deg1 = numpy.array([q1_1, q1_2])*self.rad2deg



		#print(q1)

		################################ Find q3 #####################################
		k1 = 2*self.a2*self.d4
		k2 = 2*self.a2*self.a3
		k3 = px**2 + py**2 + pz**2 - 2*px*self.a1*math.cos(q1) - 2*py*self.a1*math.sin(q1) + self.a1**2 - self.a2**2 - self.a3**2 - self.d4**2

		q3_1 = 2*math.atan( (k1+ math.sqrt(k1**2 + k2**2 - k3**2)) / (k3+k2) )
		q3_2 = 2*math.atan( (k1- math.sqrt(k1**2 + k2**2 - k3**2)) / (k3+k2) )

		'''
		if q3_1 < 0.000001:
		    q3_1 = 0
		if q3_2 < 0.000001:
		    q3_2 = 0
		'''
		q3 = [None]*2
		q3 = numpy.array([q3_1,q3_2])


		#print(q3)
		deg3 = [None]*2
		deg3 = numpy.array([q3[0],q3[1]])*self.rad2deg

		################################ Find q2 #####################################

		#print("----------------")

		uu1_0 = self.a2 + self.a3*math.cos(q3[0])+self.d4*math.sin(q3[0])
		vv1_0 = -self.a3*math.sin(q3[0]) + self.d4*math.cos(q3[0])
		yy1_0 = px*math.cos(q1) + py*math.sin(q1) - self.a1
		uu2_0 = self.a3*math.sin(q3[0]) - self.d4*math.cos(q3[0])
		vv2_0 = self.a2 + self.a3*math.cos(q3[0]) + self.d4*math.sin(q3[0])
		yy2_0 = pz

		uu1_1 = self.a2 + self.a3*math.cos(q3[1])+self.d4*math.sin(q3[1])
		vv1_1 = -self.a3*math.sin(q3[1]) + self.d4*math.cos(q3[1])
		yy1_1 = px*math.cos(q1) + py*math.sin(q1) - self.a1
		uu2_1 = self.a3*math.sin(q3[1]) - self.d4*math.cos(q3[1])
		vv2_1 = self.a2 + self.a3*math.cos(q3[1]) + self.d4*math.sin(q3[1])
		yy2_1 = pz


		A0 = numpy.array([[uu1_0,vv1_0],[uu2_0,vv2_0]])
		B0 = numpy.array([yy1_0,yy2_0])
		X0 = numpy.linalg.solve(A0,B0)

		A1 = numpy.array([[uu1_1,vv1_1], [uu2_1,vv2_1]])
		B1 = numpy.array([yy1_1,yy2_1])
		X1 = numpy.linalg.solve(A1,B1)

		cq2_1 = X0[0]
		sq2_1 = X0[1]
		cq2_2 = X1[0]
		sq2_2 = X1[1]

		q2 = [None]*2
		q2[0] = math.atan2(sq2_1,cq2_1)
		q2[1] = math.atan2(sq2_2,cq2_2)
		'''
		if q2[0] < 0.000001:
		    q2[0] = 0
		if q2[1] < 0.000001:
		    q2[1] = 0
		'''
		deg2 = [None]*2
		deg2 = numpy.array([q2[0],q2[1]])*self.rad2deg

		#print(q2)
		#print(deg2)

		################################ Find q5 #####################################

		r33 = [None]*2
		r33[0] = wx*math.cos(q1)*math.sin(q2[0]+q3[0]) + wy*math.sin(q1)*math.sin(q2[0]+q3[0]) - wz*math.cos(q2[0]+q3[0])
		r33[1] = wx*math.cos(q1)*math.sin(q2[1]+q3[1]) + wy*math.sin(q1)*math.sin(q2[1]+q3[1]) - wz*math.cos(q2[1]+q3[1])
		#print(r33)
		q5 = [None]*2
		q5[0] = math.acos(r33[0])
		q5[1] = math.acos(r33[1])
		'''
		if q5[0] < 0.000001:
		    q5[0] = 0
		if q5[1] < 0.000001:
		    q5[1] = 0
		'''
		deg5 = [None]*2
		deg5 = numpy.array([q5[0],q5[1]])*self.rad2deg

		#print("----------------")
		#print(q5)
		#print(deg5)

		################################ Find q4 #####################################
		cq4 = [None]*2
		sq4 = [None]*2
		q4 = [None]*2
		deg4 = [None]*2

		if (abs(r33[1]) < 1):
			# Normal Condition
			cq4[0] = (  wx*math.cos(q1)*math.cos(q2[0]+q3[0]) + wy*math.sin(q1)*math.cos(q2[0]+q3[0]) + wz*math.sin(q2[0]+q3[0]) ) / math.sin(q5[0])
			cq4[1] = (  wx*math.cos(q1)*math.cos(q2[1]+q3[1]) + wy*math.sin(q1)*math.cos(q2[1]+q3[1]) + wz*math.sin(q2[1]+q3[1]) ) / math.sin(q5[1])
			sq4[0] = ( wx*math.sin(q1) - wy*math.cos(q1) )/ math.sin(q5[0])
			sq4[1] = ( wx*math.sin(q1) - wy*math.cos(q1) )/ math.sin(q5[1])

			q4[0] = math.atan2(sq4[0],cq4[0])
			q4[1] = math.atan2(sq4[1],cq4[1])
		elif abs(r33[1]) == 1:
			# When q5 = 0
			q4[0] = 0
			q4[1] = 0

		else:
			print("|r33| > 1 : cannotr physically arise")
		'''
		if q4[0] < 0.000001:
		    q4[0] = 0
		if q4[1] < 0.000001:
		    q4[1] = 0
		'''
		deg4 = numpy.array([q4[0],q4[1]])*self.rad2deg
		#print("-----------------")
		#print(q4)
		#print(deg4)

		################################ Find q6 #####################################
		cq6 = [None]*2
		sq6 = [None]*2
		q6 = [None]*2
		deg6 = [None]*2

		if (abs(r33[1]) < 1):
			# Normal Condition
			cq6[0] = -(ux*math.cos(q1)*math.sin(q2[0]+q3[0]) + uy*math.sin(q1)*math.sin(q2[0]+q3[0]) - uz*math.cos(q2[0]+q3[0]) ) / math.sin(q5[0])
			cq6[1] = -(ux*math.cos(q1)*math.sin(q2[1]+q3[1]) + uy*math.sin(q1)*math.sin(q2[1]+q3[1]) - uz*math.cos(q2[1]+q3[1]) ) / math.sin(q5[1])
			sq6[0] = (vx*math.cos(q1)*math.sin(q2[0]+q3[0]) + vy*math.sin(q1)*math.sin(q2[0]+q3[0]) - vz*math.cos(q2[0]+q3[0]) ) / math.sin(q5[0])
			sq6[1] = (vx*math.cos(q1)*math.sin(q2[1]+q3[1]) + vy*math.sin(q1)*math.sin(q2[1]+q3[1]) - vz*math.cos(q2[1]+q3[1]) ) / math.sin(q5[1])

			q6[0] = math.atan2(sq6[0],cq6[0])
			q6[1] = math.atan2(sq6[1],cq6[1])
		elif abs(r33[1]) == 1:
			# When q5 = 0
			q6[0] = 0
			q6[1] = 0
		else:
			print("|r33| > 1 : cannotr physically arise")
		'''
		if q6[0] < 0.000001:
		    q6[0] = 0
		if q6[1] < 0.000001:
		    q6[1] = 0
		'''
		deg6 = numpy.array([q6[0],q6[1]])*self.rad2deg
		#print("-----------------")
		#print(q6)
		#print(deg6)

		DEG_INV = [None]*6
		DEG_INV = numpy.array([deg1[0],deg2[1],deg3[1],deg4[1],deg5[1],deg6[1]])

		##  +90 and +180 for offset the angle of servo
		ServoAng1 = DEG_INV[0]+90
		ServoAng2 = DEG_INV[1]+90
		ServoAng3 = DEG_INV[2]+180
		ServoAng4 = DEG_INV[3]+180
		ServoAng5 = DEG_INV[4]+180
		ServoAng6 = DEG_INV[5]+180
		ServoANG = [None]*6
		ServoANG = numpy.array([ServoAng1,ServoAng2,ServoAng3,ServoAng4,ServoAng5,ServoAng6])

		return ServoANG



	def WorkspaceLimitation(self,x,y,z):

		R = 560.0
		r2 = 75.0
		h = -200.0
		Qy = (R-self.d6)*math.cos(150.0*self.deg2rad)
		Qz = (R-self.d6)*math.sin(150.0*self.deg2rad)
		Sy = (R-self.d6-self.d4)*math.cos(150.0*self.deg2rad)
		Sz = (R-self.d6-self.d4)*math.sin(150.0*self.deg2rad)

		if x**2 + y**2 + z**2 <= R**2 and x**2 + y**2 + z**2 > r2**2:
			if y < 0 and z > 0:
				if x**2 + (y-Qy)**2 + (z-Qz)**2 > self.d6**2 and x**2 + (y-Sy)**2 + (z-Sz)**2 > (self.d6**2 + self.d4**2):
					#print("Work space is valid, in quadrant2")
					return x,y,z
				else:
					print("ERROR: Out of work range in quadrant 2 or 3")
					return [None]*3
			elif y > 0 and z < 0:
				if z > h:
					#print("Work space is valid, in quadrant4")
					return x,y,z
				else:
					print("ERROR: z is lower than lowest range 'h'")
					return [None]*3
			elif y > 0 and z > 0:
				#print("Work space is valid, in quadrant1")
				return x,y,z
			else:
				print("ERROR: y,z point is less than 0")
				return [None]*3

		else:
			print("ERROR: x,y,z point is not on the work envelope")
			return [None]*3

	def WorkspaceHorizontalLimitation(self,x,y,z):

		ymax = 450.0
		ymin = 230.0
		zmin_inner = 74.0
		zmax_inner = 380.0
		zmin_outer = 0.0
		zmax_outer = 250.0

		P1 = [ymax,zmin_outer]
		P2 = [ymax,zmax_outer]
		P3 = [ymin,zmax_inner]
		P4 = [ymin,zmin_inner]

		P1y = P1[0]
		P1z = P1[1]
		P2y = P2[0]
		P2z = P2[1]
		P3y = P3[0]
		P3z = P3[1]
		P4y = P4[0]
		P4z = P4[1]

		m3 = (P4z - P1z)/(P4y - P1y)
		c3 = P1z - m3*P1y
		m4 = (P3z - P2z)/(P3y - P2y)
		c4 = P3z - m4*P3y

		#print("m3: %f" %m3)
		#print("c3: %f" %c3)
		#print("m4: %f" %m4)
		#print("c4: %f" %c4)


		if (y >= 0 and z >= 0):
			if y >= P4y and y <= P1y:
				Zc = m3*y + c3
				Z4 = m4*y + c4
				Rc = Z4 - Zc
				#print("Zc: %f" %Zc)
				#print("Z4: %f" %Z4)
				#print("Rc: %f" %Rc)

				if x**2 + (z-Zc)**2 <= Rc**2:
					#print("Work space is valid")
					return x,y,z
				else:
					print("ERROR: Input is out of XZ plane")
					return [None]*3
			else:
				print("ERROR: Input y is out of range")
				return [None]*3
		else:
			print("ERROR: Input is minus number")
			return [None]*3



	def RunServo(self,inputDeg1,inputDeg2,inputDeg3,inputDeg4,inputDeg5,inputDeg6):

		pos1 = inputDeg1
		pos2 = inputDeg2
		pos3 = inputDeg3
		pos4 = inputDeg4
		pos5 = inputDeg5
		pos6 = inputDeg6

		servo_com1 = self.map(pos1,0.0,360.0,0.0,4095.0)
		servo_com2 = self.map(pos2,0.0,360.0,0.0,4095.0)
		servo_com3 = self.map(pos3,0.0,360.0,0.0,4095.0)
		servo_com4 = self.map(pos4,0.0,360.0,0.0,4095.0)
		servo_com5 = self.map(pos5,0.0,360.0,0.0,4095.0)
		servo_com6 = self.map(pos6,0.0,360.0,0.0,4095.0)

		dxl1_goal_position = int(servo_com1)
		dxl2_goal_position = int(servo_com2)
		dxl3_goal_position = int(servo_com3)
		dxl4_goal_position = int(servo_com4)
		dxl5_goal_position = int(servo_com5)
		dxl6_goal_position = int(servo_com6)

		dxl_comm_result1, dxl_error1 = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_GOAL_POSITION, dxl1_goal_position)
		if dxl_comm_result1 != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result1))
		elif dxl_error1 != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error1))

		dxl_comm_result2, dxl_error2 = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_GOAL_POSITION, dxl2_goal_position)
		if dxl_comm_result2 != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result2))
		elif dxl_error2 != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error2))

		dxl_comm_result3, dxl_error3 = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_GOAL_POSITION, dxl3_goal_position)
		if dxl_comm_result3 != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result3))
		elif dxl_error3 != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error3))

		dxl_comm_result4, dxl_error4 = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_PRO_GOAL_POSITION, dxl4_goal_position)
		if dxl_comm_result4 != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result4))
		elif dxl_error4 != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error4))

		dxl_comm_result5, dxl_error5 = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL5_ID, self.ADDR_PRO_GOAL_POSITION, dxl5_goal_position)
		if dxl_comm_result5 != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result5))
		elif dxl_error5 != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error5))

		dxl_comm_result6, dxl_error6 = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL6_ID, self.ADDR_PRO_GOAL_POSITION, dxl6_goal_position)
		if dxl_comm_result6 != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result6))
		elif dxl_error6 != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error6))		

	def ReadAngle(self):
		dxl_present_position1, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_PRESENT_POSITION)
		dxl_present_position2, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_PRESENT_POSITION)
		dxl_present_position3, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_PRESENT_POSITION)
		dxl_present_position4, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_PRO_PRESENT_POSITION)
		dxl_present_position5, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL5_ID, self.ADDR_PRO_PRESENT_POSITION)
		dxl_present_position6, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL6_ID, self.ADDR_PRO_PRESENT_POSITION)

		pre_pos1 = self.map(dxl_present_position1, 0, 4095, 0.0, 360.0)
		pre_pos2 = self.map(dxl_present_position2, 0, 4095, 0.0, 360.0)
		pre_pos3 = self.map(dxl_present_position3, 0, 4095, 0.0, 360.0)
		pre_pos4 = self.map(dxl_present_position4, 0, 4095, 0.0, 360.0)
		pre_pos5 = self.map(dxl_present_position5, 0, 4095, 0.0, 360.0)
		pre_pos6 = self.map(dxl_present_position6, 0, 4095, 0.0, 360.0)

		return pre_pos1,pre_pos2,pre_pos3,pre_pos4,pre_pos5,pre_pos6

	def ReadAngle7(self):
		dxl_present_position7, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL7_ID, self.ADDR_PRO_PRESENT_POSITION)
		
		pre_pos7 = self.map(dxl_present_position7, 0, 4095, 0.0, 360.0)
		
		return pre_pos7

	def RunServo1(self,inputDeg1):
		pos1 = inputDeg1
		servo_com1 = self.map(pos1,0.0,360.0,0.0,4095.0)
		dxl1_goal_position = int(servo_com1)
		dxl_comm_result1, dxl_error1 = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_GOAL_POSITION, dxl1_goal_position)
		if dxl_comm_result1 != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result1))
		elif dxl_error1 != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error1))

	def RunServo2(self,inputDeg2):
		pos2 = inputDeg2
		servo_com2 = self.map(pos2,0.0,360.0,0.0,4095.0)
		dxl2_goal_position = int(servo_com2)
		dxl_comm_result2, dxl_error2 = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_GOAL_POSITION, dxl2_goal_position)
		if dxl_comm_result2 != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result2))
		elif dxl_error2 != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error2))

	def RunServo3(self,inputDeg3):
		pos3 = inputDeg3
		servo_com3 = self.map(pos3,0.0,360.0,0.0,4095.0)
		dxl3_goal_position = int(servo_com3)
		dxl_comm_result3, dxl_error3 = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_GOAL_POSITION, dxl3_goal_position)
		if dxl_comm_result3 != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result3))
		elif dxl_error3 != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error3))

	def RunServo4(self,inputDeg4):
		pos4 = inputDeg4
		servo_com4 = self.map(pos4,0.0,360.0,0.0,4095.0)
		dxl4_goal_position = int(servo_com4)
		dxl_comm_result4, dxl_error4 = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_PRO_GOAL_POSITION, dxl4_goal_position)
		if dxl_comm_result4 != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result4))
		elif dxl_error4 != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error4))

	def RunServo5(self,inputDeg5):
		pos5 = inputDeg5
		servo_com5 = self.map(pos5,0.0,360.0,0.0,4095.0)
		dxl5_goal_position = int(servo_com5)
		dxl_comm_result5, dxl_error5 = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL5_ID, self.ADDR_PRO_GOAL_POSITION, dxl5_goal_position)
		if dxl_comm_result5 != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result5))
		elif dxl_error5 != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error5))

	def RunServo6(self,inputDeg6):
		pos6 = inputDeg6
		servo_com6 = self.map(pos6,0.0,360.0,0.0,4095.0)
		dxl6_goal_position = int(servo_com6)
		dxl_comm_result6, dxl_error6 = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL6_ID, self.ADDR_PRO_GOAL_POSITION, dxl6_goal_position)
		if dxl_comm_result6 != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result6))
		elif dxl_error6 != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error6))

	def RunServo7(self,inputDeg7):
		pos7 = inputDeg7
		servo_com7 = self.map(pos7,0.0,360.0,0.0,4095.0)
		dxl7_goal_position = int(servo_com7)
		dxl_comm_result7, dxl_error7 = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL7_ID, self.ADDR_PRO_GOAL_POSITION, dxl7_goal_position)
		if dxl_comm_result7 != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result7))
		elif dxl_error7 != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error7))



	def IsMoving1(self):
		Moving1, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_MOVING)
		return Moving1

	def IsMoving2(self):
		Moving2, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_MOVING)
		return Moving2

	def IsMoving3(self):
		Moving3, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_MOVING)
		return Moving3

	def IsMoving4(self):
		Moving4, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_PRO_MOVING)
		return Moving4

	def IsMoving5(self):
		Moving5, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, self.DXL5_ID, self.ADDR_PRO_MOVING)
		return Moving5

	def IsMoving6(self):
		Moving6, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, self.DXL6_ID, self.ADDR_PRO_MOVING)
		return Moving6

	def MovingStatus1(self):
		MovingStat1, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_MOVING_STATUS)

		if MovingStat1 > 48:
			print("Motor1 is in Trapezodal Profile")
		elif MovingStat1 < 35 and MovingStat1 > 20:
			print("Motor1 is in Triangular Profile")
		elif MovingStat1 < 20 and MovingStat1 > 3:
			print("Motor1 is in Rectangular Profile")
		elif MovingStat1 < 3:
			print("Motor1 is in Step Mode (No Profile)")
		else:
			print("UNKNOWN Profile...")

		return MovingStat1

	def MovingStatus2(self):
		MovingStat2, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_MOVING_STATUS)

		if MovingStat2 > 48:
			print("Motor2 is in Trapezodal Profile")
		elif MovingStat2 < 35 and MovingStat2 > 20:
			print("Motor2 is in Triangular Profile")
		elif MovingStat2 < 20 and MovingStat2 > 3:
			print("Motor2 is in Rectangular Profile")
		elif MovingStat2 < 3:
			print("Motor2 is in Step Mode (No Profile)")
		else:
			print("UNKNOWN Profile...")

		return MovingStat2

	def MovingStatus3(self):
		MovingStat3, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_MOVING_STATUS)

		if MovingStat3 > 48:
			print("Motor3 is in Trapezodal Profile")
		elif MovingStat3 < 35 and MovingStat3 > 20:
			print("Motor3 is in Triangular Profile")
		elif MovingStat3 < 20 and MovingStat3 > 3:
			print("Motor3 is in Rectangular Profile")
		elif MovingStat3 < 3:
			print("Motor3 is in Step Mode (No Profile)")
		else:
			print("UNKNOWN Profile...")

		return MovingStat3 

	def MovingStatus4(self):
		MovingStat4, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_PRO_MOVING_STATUS)

		if MovingStat4 > 48:
			print("Motor4 is in Trapezodal Profile")
		elif MovingStat4 < 35 and MovingStat4 > 20:
			print("Motor4 is in Triangular Profile")
		elif MovingStat4 < 20 and MovingStat4 > 3:
			print("Motor4 is in Rectangular Profile")
		elif MovingStat4 < 3:
			print("Motor4 is in Step Mode (No Profile)")
		else:
			print("UNKNOWN Profile...")

		return MovingStat4

	def MovingStatus5(self):
		MovingStat5, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, self.DXL5_ID, self.ADDR_PRO_MOVING_STATUS)

		if MovingStat5 > 48:
			print("Motor5 is in Trapezodal Profile")
		elif MovingStat5 < 35 and MovingStat5 > 20:
			print("Motor5 is in Triangular Profile")
		elif MovingStat5 < 20 and MovingStat5 > 3:
			print("Motor5 is in Rectangular Profile")
		elif MovingStat5 < 3:
			print("Motor5 is in Step Mode (No Profile)")
		else:
			print("UNKNOWN Profile...")

		return MovingStat5

	def MovingStatus6(self):
		MovingStat6, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, self.DXL6_ID, self.ADDR_PRO_MOVING_STATUS)

		if MovingStat6 > 48:
			print("Motor6 is in Trapezodal Profile")
		elif MovingStat6 < 35 and MovingStat6 > 20:
			print("Motor6 is in Triangular Profile")
		elif MovingStat6 < 20 and MovingStat6 > 3:
			print("Motor6 is in Rectangular Profile")
		elif MovingStat6 < 3:
			print("Motor6 is in Step Mode (No Profile)")
		else:
			print("UNKNOWN Profile...")

		return MovingStat6



	def TorqueOn(self):
		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_ENABLE)
		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_ENABLE)
		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_ENABLE)
		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_ENABLE)
		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL5_ID, self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_ENABLE)
		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL6_ID, self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_ENABLE)
		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL7_ID, self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_ENABLE)

	def TorqueOff(self):
		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_DISABLE)
		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_DISABLE)
		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_DISABLE)
		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_DISABLE)
		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL5_ID, self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_DISABLE)
		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL6_ID, self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_DISABLE)
		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL7_ID, self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_DISABLE)

	def TorqueGripperOn(self):
		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL7_ID, self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_ENABLE)

	def TorqueGripperOff(self):
		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL7_ID, self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_DISABLE)

	def GripOpen(self):
		dxl7_goal_position = 3300
		dxl_comm_result7, dxl_error7 = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL7_ID, self.ADDR_PRO_GOAL_POSITION, dxl7_goal_position)

	def GripClose(self):
		dxl7_goal_position = 1600
		dxl_comm_result7, dxl_error7 = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL7_ID, self.ADDR_PRO_GOAL_POSITION, dxl7_goal_position)



	def DeltaPos(self,pre_pos,goal_pos):
		if pre_pos > 360 or goal_pos > 360:
			print("Input position is over than 360!")
			return 0
		else:
			pre_pos_pul = self.map(pre_pos,0.0,360.0,0.0,4095.0)
			pre_pos_pul = int(pre_pos_pul)
			goal_pos_pul = self.map(goal_pos,0.0,360.0,0.0,4095.0)
			goal_pos_pul = int(goal_pos_pul)

			delta_pos = abs(goal_pos_pul - pre_pos_pul)

			return delta_pos

	def SetProfile1(self,set_V_PRFL,set_A_PRFL):
		######################### Set Velocity / Acceleration Profile  ##############################
		set_A_Limit = 32767     # 32767 default                [214.577 rev/min^2]
		set_V_Limit = 500       # 350 Default                  [0.229RPM]

		dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_ACCELERATION_LIMIT, set_A_Limit)
		dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_VELOCITY_LIMIT, set_V_Limit)

		dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_PROFILE_ACCELERATION, int(set_A_PRFL))
		dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_PROFILE_VELOCITY, int(set_V_PRFL))

		acceleration_limit, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_ACCELERATION_LIMIT)
		velocity_limit, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_VELOCITY_LIMIT)

		print("V PRFL 1: %d" %set_V_PRFL)
		print("A PRFL 1: %d" %set_A_PRFL)
		print("--------------------------------")

	def SetProfile2(self,set_V_PRFL,set_A_PRFL):
		######################### Set Velocity / Acceleration Profile  ##############################
		set_A_Limit = 32767     # 32767 default                [214.577 rev/min^2]
		set_V_Limit = 500       # 350 Default                  [0.229RPM]

		#set_A_PRFL = 30      # between 0 ~ set_A_limit      [214.577 rev/min^2]
		#set_V_PRFL = 200      # between 0 ~ set_V_Limit      [0.229RPM]

		dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_ACCELERATION_LIMIT, set_A_Limit)
		dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_VELOCITY_LIMIT, set_V_Limit)

		dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_PROFILE_ACCELERATION, int(set_A_PRFL))
		dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_PROFILE_VELOCITY, int(set_V_PRFL))

		acceleration_limit, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_ACCELERATION_LIMIT)
		velocity_limit, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_VELOCITY_LIMIT)

		print("V PRFL 2: %d" %set_V_PRFL)
		print("A PRFL 2: %d" %set_A_PRFL)
		print("--------------------------------") 

	def SetProfile3(self,set_V_PRFL,set_A_PRFL):
		######################### Set Velocity / Acceleration Profile  ##############################
		set_A_Limit = 32767     # 32767 default                [214.577 rev/min^2]
		set_V_Limit = 500       # 350 Default                  [0.229RPM]

		#set_A_PRFL = 30      # between 0 ~ set_A_limit      [214.577 rev/min^2]
		#set_V_PRFL = 200      # between 0 ~ set_V_Limit      [0.229RPM]

		dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_ACCELERATION_LIMIT, set_A_Limit)
		dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_VELOCITY_LIMIT, set_V_Limit)

		dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_PROFILE_ACCELERATION, int(set_A_PRFL))
		dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_PROFILE_VELOCITY, int(set_V_PRFL))

		acceleration_limit, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_ACCELERATION_LIMIT)
		velocity_limit, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_VELOCITY_LIMIT)

		print("V PRFL 3: %d" %set_V_PRFL)
		print("A PRFL 3: %d" %set_A_PRFL)
		print("--------------------------------")    

	def SetProfile4(self,set_V_PRFL,set_A_PRFL):
		######################### Set Velocity / Acceleration Profile  ##############################
		set_A_Limit = 32767     # 32767 default                [214.577 rev/min^2]
		set_V_Limit = 500       # 350 Default                  [0.229RPM]

		#set_A_PRFL = 30      # between 0 ~ set_A_limit      [214.577 rev/min^2]
		#set_V_PRFL = 200      # between 0 ~ set_V_Limit      [0.229RPM]

		dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_PRO_ACCELERATION_LIMIT, set_A_Limit)
		dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_PRO_VELOCITY_LIMIT, set_V_Limit)

		dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_PRO_PROFILE_ACCELERATION, int(set_A_PRFL))
		dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_PRO_PROFILE_VELOCITY, int(set_V_PRFL))

		acceleration_limit, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_PRO_ACCELERATION_LIMIT)
		velocity_limit, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_PRO_VELOCITY_LIMIT)

		print("V PRFL 4: %d" %set_V_PRFL)
		print("A PRFL 4: %d" %set_A_PRFL)
		print("--------------------------------")

	def SetProfile5(self,set_V_PRFL,set_A_PRFL):
		######################### Set Velocity / Acceleration Profile  ##############################
		set_A_Limit = 32767     # 32767 default                [214.577 rev/min^2]
		set_V_Limit = 500       # 350 Default                  [0.229RPM]

		#set_A_PRFL = 30      # between 0 ~ set_A_limit      [214.577 rev/min^2]
		#set_V_PRFL = 200      # between 0 ~ set_V_Limit      [0.229RPM]

		dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL5_ID, self.ADDR_PRO_ACCELERATION_LIMIT, set_A_Limit)
		dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL5_ID, self.ADDR_PRO_VELOCITY_LIMIT, set_V_Limit)

		dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL5_ID, self.ADDR_PRO_PROFILE_ACCELERATION, int(set_A_PRFL))
		dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL5_ID, self.ADDR_PRO_PROFILE_VELOCITY, int(set_V_PRFL))

		acceleration_limit, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL5_ID, self.ADDR_PRO_ACCELERATION_LIMIT)
		velocity_limit, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL5_ID, self.ADDR_PRO_VELOCITY_LIMIT)

		print("V PRFL 5: %d" %set_V_PRFL)
		print("A PRFL 5: %d" %set_A_PRFL)
		print("--------------------------------")    

	def SetProfile6(self,set_V_PRFL,set_A_PRFL):
		######################### Set Velocity / Acceleration Profile  ##############################
		set_A_Limit = 32767     # 32767 default                [214.577 rev/min^2]
		set_V_Limit = 500       # 350 Default                  [0.229RPM]

		#set_A_PRFL = 30      # between 0 ~ set_A_limit      [214.577 rev/min^2]
		#set_V_PRFL = 200      # between 0 ~ set_V_Limit      [0.229RPM]

		dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL6_ID, self.ADDR_PRO_ACCELERATION_LIMIT, set_A_Limit)
		dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL6_ID, self.ADDR_PRO_VELOCITY_LIMIT, set_V_Limit)

		dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL6_ID, self.ADDR_PRO_PROFILE_ACCELERATION, int(set_A_PRFL))
		dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL6_ID, self.ADDR_PRO_PROFILE_VELOCITY, int(set_V_PRFL))

		acceleration_limit, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL6_ID, self.ADDR_PRO_ACCELERATION_LIMIT)
		velocity_limit, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL6_ID, self.ADDR_PRO_VELOCITY_LIMIT)

		print("V PRFL 6: %d" %set_V_PRFL)
		print("A PRFL 6: %d" %set_A_PRFL)
		print("--------------------------------")

	def SetProfile7(self,set_V_PRFL, set_A_PRFL):
		######################### Set Velocity / Acceleration Profile  ##############################
		set_A_Limit = 32767     # 32767 default                [214.577 rev/min^2]
		set_V_Limit = 500       # 350 Default                  [0.229RPM]

		#set_A_PRFL = 30      # between 0 ~ set_A_limit      [214.577 rev/min^2]
		#set_V_PRFL = 200      # between 0 ~ set_V_Limit      [0.229RPM]

		dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL7_ID, self.ADDR_PRO_ACCELERATION_LIMIT, set_A_Limit)
		dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL7_ID, self.ADDR_PRO_VELOCITY_LIMIT, set_V_Limit)

		dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL7_ID, self.ADDR_PRO_PROFILE_ACCELERATION, int(set_A_PRFL))
		dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL7_ID, self.ADDR_PRO_PROFILE_VELOCITY, int(set_V_PRFL))

		acceleration_limit, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL7_ID, self.ADDR_PRO_ACCELERATION_LIMIT)
		velocity_limit, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL7_ID, self.ADDR_PRO_VELOCITY_LIMIT)

		print("V PRFL 7: %d" %set_V_PRFL)
		print("A PRFL 7: %d" %set_A_PRFL)
		print("--------------------------------")     

	def TrajectoryGeneration3(self,Vstd,Astd,DelPos1,DelPos2,DelPos3,DelPos4,DelPos5,DelPos6):
		DelPos = [DelPos1, DelPos2, DelPos3, DelPos4, DelPos5, DelPos6]
		MIN = min(DelPos)
		MAX = max(DelPos)

		V_Lim = 150
		A_Lim = 80

		## If no travel distance for every servos, return a slow speed profile
		if DelPos1 <= 1 and DelPos2 <= 1 and DelPos3 <= 1 and DelPos4 <= 1 and DelPos5 <= 1 and DelPos6 <= 1:
			V1 = 20
			A1 = 5
			V2 = 20
			A2 = 5
			V3 = 20
			A3 = 5
			V4 = 20
			A4 = 5
			V5 = 20
			A5 = 5
			V6 = 20
			A6 = 5
			print("Low travel distance...")
			return V1, A1, V2, A2, V3, A3, V4, A4, V5, A5, V6, A6
		## If V and A input are over the limit
		if Vstd >= V_Lim or Astd >= A_Lim:

			Vstd = V_Lim
			Astd = A_Lim

			print("Standard Velocity is over the limit!")

		if DelPos1 == MAX:
			V1 = Vstd
			A1 = Astd
			print("Servo1 is the standard")
			t3_1 = 64.0*V1/A1 + 64.0*DelPos1/V1
			t2_1 = 64.0*DelPos1/V1
			t3_std = t3_1
			t2_std = t2_1
			t3_2 = t3_std
			t3_3 = t3_std
			t3_4 = t3_std
			t3_5 = t3_std
			t3_6 = t3_std

			t2_2 = t2_std
			t2_3 = t2_std
			t2_4 = t2_std
			t2_5 = t2_std
			t2_6 = t2_std
			den_std = (t3_std - t2_std)

			V2 = 64.0*DelPos2/t2_2
			V3 = 64.0*DelPos3/t2_3
			V4 = 64.0*DelPos4/t2_4
			V5 = 64.0*DelPos5/t2_5
			V6 = 64.0*DelPos6/t2_6
			A2 = 64*V2 / den_std
			A3 = 64*V3 / den_std
			A4 = 64*V4 / den_std
			A5 = 64*V5 / den_std
			A6 = 64*V6 / den_std

		elif DelPos2 == MAX:
			V2 = Vstd
			A2 = Astd
			print("Servo2 is the standard")
			t3_2 = 64.0*V2/A2 + 64.0*DelPos2/V2
			t2_2 = 64.0*DelPos2/V2
			t3_std = t3_2
			t2_std = t2_2
			t3_1 = t3_std
			t3_3 = t3_std
			t3_4 = t3_std
			t3_5 = t3_std
			t3_6 = t3_std

			t2_1 = t2_std
			t2_3 = t2_std
			t2_4 = t2_std
			t2_5 = t2_std
			t2_6 = t2_std
			den_std = (t3_std - t2_std)
			V1 = 64.0*DelPos1/t2_1
			V3 = 64.0*DelPos3/t2_3
			V4 = 64.0*DelPos4/t2_4
			V5 = 64.0*DelPos5/t2_5
			V6 = 64.0*DelPos6/t2_6
			A1 = 64*V1 / den_std
			A3 = 64*V3 / den_std
			A4 = 64*V4 / den_std
			A5 = 64*V5 / den_std
			A6 = 64*V6 / den_std

		elif DelPos3 == MAX:
			V3 = Vstd
			A3 = Astd
			print("Servo3 is the standard")
			t3_3 = 64.0*V3/A3 + 64.0*DelPos3/V3
			t2_3 = 64.0*DelPos3/V3
			t3_std = t3_3
			t2_std = t2_3
			t3_1 = t3_std
			t3_2 = t3_std
			t3_4 = t3_std
			t3_5 = t3_std
			t3_6 = t3_std

			t2_1 = t2_std
			t2_2 = t2_std
			t2_4 = t2_std
			t2_5 = t2_std
			t2_6 = t2_std
			den_std = (t3_std - t2_std)

			V1 = 64.0*DelPos1/t2_1
			V2 = 64.0*DelPos2/t2_2
			V4 = 64.0*DelPos4/t2_4
			V5 = 64.0*DelPos5/t2_5
			V6 = 64.0*DelPos6/t2_6
			A1 = 64*V1 / den_std
			A2 = 64*V2 / den_std
			A4 = 64*V4 / den_std
			A5 = 64*V5 / den_std
			A6 = 64*V6 / den_std

		elif DelPos4 == MAX:
			V4 = Vstd
			A4 = Astd
			print("Servo4 is the standard")
			t3_4 = 64.0*V4/A4 + 64.0*DelPos4/V4
			t2_4 = 64.0*DelPos4/V4
			t3_std = t3_4
			t2_std = t2_4
			t3_1 = t3_std
			t3_2 = t3_std
			t3_3 = t3_std
			t3_5 = t3_std
			t3_6 = t3_std

			t2_1 = t2_std
			t2_2 = t2_std
			t2_3 = t2_std
			t2_5 = t2_std
			t2_6 = t2_std
			den_std = (t3_std - t2_std)

			V1 = 64.0*DelPos1/t2_1
			V2 = 64.0*DelPos2/t2_2
			V3 = 64.0*DelPos3/t2_3
			V5 = 64.0*DelPos5/t2_5
			V6 = 64.0*DelPos6/t2_6
			A1 = 64*V1 / den_std
			A2 = 64*V2 / den_std
			A3 = 64*V3 / den_std
			A5 = 64*V5 / den_std
			A6 = 64*V6 / den_std

		elif DelPos5 == MAX:
			V5 = Vstd
			A5 = Astd
			print("Servo5 is the standard")
			t3_5 = 64.0*V5/A5 + 64.0*DelPos5/V5
			t2_5 = 64.0*DelPos5/V5
			t3_std = t3_5
			t2_std = t2_5
			t3_1 = t3_std
			t3_2 = t3_std
			t3_3 = t3_std
			t3_4 = t3_std
			t3_6 = t3_std

			t2_1 = t2_std
			t2_2 = t2_std
			t2_3 = t2_std
			t2_4 = t2_std
			t2_6 = t2_std
			den_std = (t3_std - t2_std)

			V1 = 64.0*DelPos1/t2_1
			V2 = 64.0*DelPos2/t2_2
			V3 = 64.0*DelPos3/t2_3
			V4 = 64.0*DelPos4/t2_4
			V6 = 64.0*DelPos6/t2_6
			A1 = 64*V1 / den_std
			A2 = 64*V2 / den_std
			A3 = 64*V3 / den_std
			A4 = 64*V4 / den_std
			A6 = 64*V6 / den_std

		elif DelPos6 == MAX:
			V6 = Vstd
			A6 = Astd
			print("Servo6 is the standard")
			t3_6 = 64.0*V6/A6 + 64.0*DelPos6/V6
			t2_6 = 64.0*DelPos6/V6
			t3_std = t3_6
			t2_std = t2_6
			t3_1 = t3_std
			t3_2 = t3_std
			t3_3 = t3_std
			t3_4 = t3_std
			t3_5 = t3_std

			t2_1 = t2_std
			t2_2 = t2_std
			t2_3 = t2_std
			t2_4 = t2_std
			t2_5 = t2_std
			den_std = (t3_std - t2_std)

			V1 = 64.0*DelPos1/t2_1
			V2 = 64.0*DelPos2/t2_2
			V3 = 64.0*DelPos3/t2_3
			V4 = 64.0*DelPos4/t2_4
			V5 = 64.0*DelPos5/t2_5
			A1 = 64*V1 / den_std
			A2 = 64*V2 / den_std
			A3 = 64*V3 / den_std
			A4 = 64*V4 / den_std
			A5 = 64*V5 / den_std

		return V1, A1, V2, A2, V3, A3, V4, A4, V5, A5, V6, A6



	def SetPID1(self,set_P_Gain,set_I_Gain,set_D_Gain):
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_POSITION_D_GAIN, set_D_Gain)

		position_D_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_POSITION_D_GAIN)
		position_I_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_POSITION_I_GAIN)
		position_P_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_POSITION_P_GAIN)

		print("Position P Gain 1: %d" %position_P_gain)
		print("Position I Gain 1: %d" %position_I_gain)
		print("Position D Gain 1: %d" %position_D_gain)
		print("------------------------------")

	def SetPID2(self,set_P_Gain,set_I_Gain,set_D_Gain):
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_POSITION_D_GAIN, set_D_Gain)

		position_D_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_POSITION_D_GAIN)
		position_I_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_POSITION_I_GAIN)
		position_P_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_POSITION_P_GAIN)

		print("Position P Gain 2: %d" %position_P_gain)
		print("Position I Gain 2: %d" %position_I_gain)
		print("Position D Gain 2: %d" %position_D_gain)
		print("------------------------------")

	def SetPID3(self,set_P_Gain,set_I_Gain,set_D_Gain):
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_POSITION_D_GAIN, set_D_Gain)

		position_D_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_POSITION_D_GAIN)
		position_I_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_POSITION_I_GAIN)
		position_P_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_POSITION_P_GAIN)

		print("Position P Gain 3: %d" %position_P_gain)
		print("Position I Gain 3: %d" %position_I_gain)
		print("Position D Gain 3: %d" %position_D_gain)
		print("------------------------------")

	def SetPID4(self,set_P_Gain,set_I_Gain,set_D_Gain):
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_PRO_POSITION_D_GAIN, set_D_Gain)    

		position_D_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_PRO_POSITION_D_GAIN)
		position_I_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_PRO_POSITION_I_GAIN)
		position_P_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_PRO_POSITION_P_GAIN)

		print("Position P Gain 4: %d" %position_P_gain)
		print("Position I Gain 4: %d" %position_I_gain)
		print("Position D Gain 4: %d" %position_D_gain)
		print("------------------------------")

	def SetPID5(self,set_P_Gain,set_I_Gain,set_D_Gain):
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL5_ID, self.ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL5_ID, self.ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL5_ID, self.ADDR_PRO_POSITION_D_GAIN, set_D_Gain)

		position_D_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL5_ID, self.ADDR_PRO_POSITION_D_GAIN)
		position_I_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL5_ID, self.ADDR_PRO_POSITION_I_GAIN)
		position_P_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL5_ID, self.ADDR_PRO_POSITION_P_GAIN)

		print("Position P Gain 5: %d" %position_P_gain)
		print("Position I Gain 5: %d" %position_I_gain)
		print("Position D Gain 5: %d" %position_D_gain)
		print("------------------------------")

	def SetPID6(self,set_P_Gain,set_I_Gain,set_D_Gain):
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL6_ID, self.ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL6_ID, self.ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL6_ID, self.ADDR_PRO_POSITION_D_GAIN, set_D_Gain)

		position_D_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL6_ID, self.ADDR_PRO_POSITION_D_GAIN)
		position_I_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL6_ID, self.ADDR_PRO_POSITION_I_GAIN)
		position_P_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL6_ID, self.ADDR_PRO_POSITION_P_GAIN)

		print("Position P Gain 6: %d" %position_P_gain)
		print("Position I Gain 6: %d" %position_I_gain)
		print("Position D Gain 6: %d" %position_D_gain)
		print("------------------------------")

	def SetPID7(self,set_P_Gain,set_I_Gain,set_D_Gain):
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL7_ID, self.ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL7_ID, self.ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL7_ID, self.ADDR_PRO_POSITION_D_GAIN, set_D_Gain)

		position_D_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL7_ID, self.ADDR_PRO_POSITION_D_GAIN)
		position_I_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL7_ID, self.ADDR_PRO_POSITION_I_GAIN)
		position_P_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL7_ID, self.ADDR_PRO_POSITION_P_GAIN)

		print("Position P Gain 7: %d" %position_P_gain)
		print("Position I Gain 7: %d" %position_I_gain)
		print("Position D Gain 7: %d" %position_D_gain)
		print("------------------------------")    

	def SetFFGain1(self,set_FF1_Gain,set_FF2_Gain):
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_FEEDFORWARD_1st_GAIN, set_FF1_Gain)
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_FEEDFORWARD_2nd_GAIN, set_FF2_Gain)

		FF1_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_FEEDFORWARD_1st_GAIN)
		FF2_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_FEEDFORWARD_2nd_GAIN)

		print("Feedforward 1st Gain 1: %d" %FF1_gain)
		print("Feedforward 2nd Gain 1: %d" %FF2_gain)
		print("------------------------------") 

	def SetFFGain2(self,set_FF1_Gain,set_FF2_Gain):
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_FEEDFORWARD_1st_GAIN, set_FF1_Gain)
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_FEEDFORWARD_2nd_GAIN, set_FF2_Gain)

		FF1_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_FEEDFORWARD_1st_GAIN)
		FF2_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_FEEDFORWARD_2nd_GAIN)

		print("Feedforward 1st Gain 2: %d" %FF1_gain)
		print("Feedforward 2nd Gain 2: %d" %FF2_gain)
		print("------------------------------")

	def SetFFGain3(self,set_FF1_Gain,set_FF2_Gain):
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_FEEDFORWARD_1st_GAIN, set_FF1_Gain)
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_FEEDFORWARD_2nd_GAIN, set_FF2_Gain)

		FF1_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_FEEDFORWARD_1st_GAIN)
		FF2_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_FEEDFORWARD_2nd_GAIN)

		print("Feedforward 1st Gain 3: %d" %FF1_gain)
		print("Feedforward 2nd Gain 3: %d" %FF2_gain)
		print("------------------------------")

	def SetFFGain4(self,set_FF1_Gain,set_FF2_Gain):
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_PRO_FEEDFORWARD_1st_GAIN, set_FF1_Gain)
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_PRO_FEEDFORWARD_2nd_GAIN, set_FF2_Gain)

		FF1_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_PRO_FEEDFORWARD_1st_GAIN)
		FF2_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_PRO_FEEDFORWARD_2nd_GAIN)

		print("Feedforward 1st Gain 4: %d" %FF1_gain)
		print("Feedforward 2nd Gain 4: %d" %FF2_gain)
		print("------------------------------")   

	def SetFFGain5(self,set_FF1_Gain,set_FF2_Gain):
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL5_ID, self.ADDR_PRO_FEEDFORWARD_1st_GAIN, set_FF1_Gain)
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL5_ID, self.ADDR_PRO_FEEDFORWARD_2nd_GAIN, set_FF2_Gain)

		FF1_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL5_ID, self.ADDR_PRO_FEEDFORWARD_1st_GAIN)
		FF2_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL5_ID, self.ADDR_PRO_FEEDFORWARD_2nd_GAIN)

		print("Feedforward 1st Gain 5: %d" %FF1_gain)
		print("Feedforward 2nd Gain 5: %d" %FF2_gain)
		print("------------------------------")

	def SetFFGain6(self,set_FF1_Gain,set_FF2_Gain):
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL6_ID, self.ADDR_PRO_FEEDFORWARD_1st_GAIN, set_FF1_Gain)
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL6_ID, self.ADDR_PRO_FEEDFORWARD_2nd_GAIN, set_FF2_Gain)

		FF1_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL6_ID, self.ADDR_PRO_FEEDFORWARD_1st_GAIN)
		FF2_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL6_ID, self.ADDR_PRO_FEEDFORWARD_2nd_GAIN)

		print("Feedforward 1st Gain 6: %d" %FF1_gain)
		print("Feedforward 2nd Gain 6: %d" %FF2_gain)
		print("------------------------------")



	def StandByPos(self):

		HomeDataLength = len(self.Home_Ang1)

		# Last Home Position
		Last_Ang1 = self.Home_Ang1[HomeDataLength-1]
		Last_Ang2 = self.Home_Ang2[HomeDataLength-1]
		Last_Ang3 = self.Home_Ang3[HomeDataLength-1]
		Last_Ang4 = self.Home_Ang4[HomeDataLength-1]
		Last_Ang5 = self.Home_Ang5[HomeDataLength-1]
		Last_Ang6 = self.Home_Ang6[HomeDataLength-1]

		InitAng = self.ReadAngle()
		Deg1 = InitAng[0]
		Deg2 = InitAng[1]
		Deg3 = InitAng[2]
		Deg4 = InitAng[3]
		Deg5 = InitAng[4]
		Deg6 = InitAng[5]
		if (abs(Deg1 - Last_Ang1) < 2) and (abs(Deg2 - Last_Ang2) < 2) and (abs(Deg3 - Last_Ang3) < 2) and (abs(Deg4 - Last_Ang4) < 2) and (abs(Deg5 - Last_Ang5) < 2) and (abs(Deg6 - Last_Ang6) < 2):
			print("Robot is too close to sleep position")
			print("Run [RobotArmAwake] first, then [StandByPos]")
			print("DelPos1: %f" %abs(Deg1 - Last_Ang1))
			print("DelPos2: %f" %abs(Deg2 - Last_Ang2))
			print("DelPos3: %f" %abs(Deg3 - Last_Ang3))
			print("DelPos4: %f" %abs(Deg4 - Last_Ang4))
			print("DelPos5: %f" %abs(Deg5 - Last_Ang5))
			print("DelPos6: %f" %abs(Deg6 - Last_Ang6))
			time.sleep(1)
			self.TorqueOff()
		else:
			print("Robot arm is going to standby position")

			Vstd = 20
			Astd = 8

			self.SetProfile1(Vstd,Astd)
			self.SetProfile2(Vstd,Astd)
			self.SetProfile3(Vstd,Astd)
			self.SetProfile4(Vstd,Astd)
			self.SetProfile5(Vstd,Astd)
			self.SetProfile6(Vstd,Astd)

			pos1 = 180
			pos2 = 180
			pos3 = 180
			pos4 = 180
			pos5 = 180
			pos6 = 180

			self.RunServo(pos1,pos2,pos3,pos4,pos5,pos6)

			MovingFlag = True
			# This delay would make sure that the moving detection loop will not run too fast than actual motion
			time.sleep(0.1) 

			while MovingFlag:
				Move1 = self.IsMoving1()
				Move2 = self.IsMoving2()
				Move3 = self.IsMoving3()
				Move4 = self.IsMoving4()
				Move5 = self.IsMoving5()
				Move6 = self.IsMoving6()

				if Move1 == 0 and Move2 == 0 and Move3 == 0 and Move4 == 0 and Move5 == 0 and Move6 == 0:
					MovingFlag = False

	def StandByJogLinear(self):

		print("Robot arm is going to standby position in the Jog Linear Mode")

		Vstd = 20
		Astd = 8

		self.SetProfile1(Vstd,Astd)
		self.SetProfile2(Vstd,Astd)
		self.SetProfile3(Vstd,Astd)
		self.SetProfile4(Vstd,Astd)
		self.SetProfile5(Vstd,Astd)
		self.SetProfile6(Vstd,Astd)

		ServoAng = self.RobotArmINV(0,300,200,0,0,0)

		self.RunServo(ServoAng[0],ServoAng[1],ServoAng[2],ServoAng[3],ServoAng[4],ServoAng[5])

		MovingFlag = True
		# This delay would make sure that the moving detection loop will not run too fast than actual motion
		time.sleep(0.1) 

		while MovingFlag:
			Move1 = self.IsMoving1()
			Move2 = self.IsMoving2()
			Move3 = self.IsMoving3()
			Move4 = self.IsMoving4()
			Move5 = self.IsMoving5()
			Move6 = self.IsMoving6()

			if Move1 == 0 and Move2 == 0 and Move3 == 0 and Move4 == 0 and Move5 == 0 and Move6 == 0:
				MovingFlag = False

	def RobotArmGoHome(self):

		HomeDataLength = len(self.Home_Ang1)

		# Last Home Position #
		Last_Ang1 = self.Home_Ang1[HomeDataLength-1]
		Last_Ang2 = self.Home_Ang2[HomeDataLength-1]
		Last_Ang3 = self.Home_Ang3[HomeDataLength-1]
		Last_Ang4 = self.Home_Ang4[HomeDataLength-1]
		Last_Ang5 = self.Home_Ang5[HomeDataLength-1]
		Last_Ang6 = self.Home_Ang6[HomeDataLength-1]

		InitAng = self.ReadAngle()
		Deg1 = InitAng[0]
		Deg2 = InitAng[1]
		Deg3 = InitAng[2]
		Deg4 = InitAng[3]
		Deg5 = InitAng[4]
		Deg6 = InitAng[5]
		if (abs(Deg1 - Last_Ang1) < 40.0) and (abs(Deg2 - Last_Ang2) < 40) and (abs(Deg3 - Last_Ang3) < 40) and (abs(Deg4 - Last_Ang4) < 40) and (abs(Deg5 - Last_Ang5) < 40) and (abs(Deg6 - Last_Ang6) < 40):
			print("Robot is already in sleep position")
			time.sleep(1)
			self.TorqueOff()
		else:
			print("Robot arm is going to sleep position")
			self.GripOpen()

			K = 0
			PreAng = self.ReadAngle()
			PreAng1 = PreAng[0]
			PreAng2 = PreAng[1]
			PreAng3 = PreAng[2]
			PreAng4 = PreAng[3]
			PreAng5 = PreAng[4]
			PreAng6 = PreAng[5]
			for K in range(0,HomeDataLength):

				GoalPos1 = self.Home_Ang1[K]
				GoalPos2 = self.Home_Ang2[K]
				GoalPos3 = self.Home_Ang3[K]
				GoalPos4 = self.Home_Ang4[K]
				GoalPos5 = self.Home_Ang5[K]
				GoalPos6 = self.Home_Ang6[K]

				DelPos1 = self.DeltaPos(PreAng1,GoalPos1)
				DelPos2 = self.DeltaPos(PreAng2,GoalPos2)
				DelPos3 = self.DeltaPos(PreAng3,GoalPos3)
				DelPos4 = self.DeltaPos(PreAng4,GoalPos4)
				DelPos5 = self.DeltaPos(PreAng5,GoalPos5)
				DelPos6 = self.DeltaPos(PreAng6,GoalPos6)

				VSTD = 40
				ASTD = 15

				TRAJ = self.TrajectoryGeneration3(VSTD,ASTD,DelPos1,DelPos2,DelPos3,DelPos4,DelPos5,DelPos6)

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

				self.SetProfile1(V1,A1)
				self.SetProfile2(V2,A2)
				self.SetProfile3(V3,A3)
				self.SetProfile4(V4,A4)
				self.SetProfile5(V5,A5)
				self.SetProfile6(V6,A6)


				self.RunServo(self.Home_Ang1[K], self.Home_Ang2[K], self.Home_Ang3[K], self.Home_Ang4[K], self.Home_Ang5[K], self.Home_Ang6[K])

				MovingFlag = True
				# This delay would make sure that the moving detection loop will not run too fast than actual motion
				time.sleep(0.1)

				while MovingFlag:
					Move1 = self.IsMoving1()
					Move2 = self.IsMoving2()
					Move3 = self.IsMoving3()
					Move4 = self.IsMoving4()
					Move5 = self.IsMoving5()
					Move6 = self.IsMoving6()
					
					if Move1 == 0 and Move2 == 0 and Move3 == 0 and Move4 == 0 and Move5 == 0 and Move6 == 0:
						MovingFlag = False
					
				PreAng1 = GoalPos1
				PreAng2 = GoalPos2
				PreAng3 = GoalPos3
				PreAng4 = GoalPos4
				PreAng5 = GoalPos5
				PreAng6 = GoalPos6

	def RobotArmAwake(self):

		HomeDataLength = len(self.Home_Ang1)

		# Last Home Position #
		Last_Ang1 = self.Home_Ang1[HomeDataLength-1]
		Last_Ang2 = self.Home_Ang2[HomeDataLength-1]
		Last_Ang3 = self.Home_Ang3[HomeDataLength-1]
		Last_Ang4 = self.Home_Ang4[HomeDataLength-1]
		Last_Ang5 = self.Home_Ang5[HomeDataLength-1]
		Last_Ang6 = self.Home_Ang6[HomeDataLength-1]

		InitAng = self.ReadAngle()
		Deg1 = InitAng[0]
		Deg2 = InitAng[1]
		Deg3 = InitAng[2]
		Deg4 = InitAng[3]
		Deg5 = InitAng[4]
		Deg6 = InitAng[5]
		if (abs(Deg1 - Last_Ang1) < 40) and (abs(Deg2 - Last_Ang2) < 40) and (abs(Deg3 - Last_Ang3) < 40) and (abs(Deg4 - Last_Ang4) < 40) and (abs(Deg5 - Last_Ang5) < 40) and (abs(Deg6 - Last_Ang6) < 40):
			print("Robot is in sleep position")
			AwakenOK = True
			self.TorqueOn()
			self.GripOpen()

			K = HomeDataLength-1
			PreAng = self.ReadAngle()
			PreAng1 = PreAng[0]
			PreAng2 = PreAng[1]
			PreAng3 = PreAng[2]
			PreAng4 = PreAng[3]
			PreAng5 = PreAng[4]
			PreAng6 = PreAng[5]
			while K >= 0:

				print K

				GoalPos1 = self.Home_Ang1[K]
				GoalPos2 = self.Home_Ang2[K]
				GoalPos3 = self.Home_Ang3[K]
				GoalPos4 = self.Home_Ang4[K]
				GoalPos5 = self.Home_Ang5[K]
				GoalPos6 = self.Home_Ang6[K]

				DelPos1 = self.DeltaPos(PreAng1,GoalPos1)
				DelPos2 = self.DeltaPos(PreAng2,GoalPos2)
				DelPos3 = self.DeltaPos(PreAng3,GoalPos3)
				DelPos4 = self.DeltaPos(PreAng4,GoalPos4)
				DelPos5 = self.DeltaPos(PreAng5,GoalPos5)
				DelPos6 = self.DeltaPos(PreAng6,GoalPos6)

				VSTD = 40
				ASTD = 10

				TRAJ = self.TrajectoryGeneration3(VSTD,ASTD,DelPos1,DelPos2,DelPos3,DelPos4,DelPos5,DelPos6)

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

				self.SetProfile1(V1,A1)
				self.SetProfile2(V2,A2)
				self.SetProfile3(V3,A3)
				self.SetProfile4(V4,A4)
				self.SetProfile5(V5,A5)
				self.SetProfile6(V6,A6)

				self.RunServo(self.Home_Ang1[K], self.Home_Ang2[K], self.Home_Ang3[K], self.Home_Ang4[K], self.Home_Ang5[K], self.Home_Ang6[K])

				MovingFlag = True
				# This delay would make sure that the moving detection loop will not run too fast than actual motion
				time.sleep(0.1)
				while MovingFlag:
					Move1 = self.IsMoving1()
					Move2 = self.IsMoving2()
					Move3 = self.IsMoving3()
					Move4 = self.IsMoving4()
					Move5 = self.IsMoving5()
					Move6 = self.IsMoving6()

					if Move1 == 0 and Move2 == 0 and Move3 == 0 and Move4 == 0 and Move5 == 0 and Move6 == 0:
						MovingFlag = False
						
				PreAng1 = GoalPos1
				PreAng2 = GoalPos2
				PreAng3 = GoalPos3
				PreAng4 = GoalPos4
				PreAng5 = GoalPos5
				PreAng6 = GoalPos6

				K = K - 1

		else:
			print("WARNING!: Robot is not in sleep position")
			print("DelPos1: %f" %abs(Deg1 - Last_Ang1))
			print("DelPos2: %f" %abs(Deg2 - Last_Ang2))
			print("DelPos3: %f" %abs(Deg3 - Last_Ang3))
			print("DelPos4: %f" %abs(Deg4 - Last_Ang4))
			print("DelPos5: %f" %abs(Deg5 - Last_Ang5))
			print("DelPos6: %f" %abs(Deg6 - Last_Ang6))

			AwakenOK = False

		return AwakenOK

	def GetXYZ(self):
		ReadAng = self.ReadAngle()
		ReadAng1 = ReadAng[0]
		ReadAng2 = ReadAng[1]
		ReadAng3 = ReadAng[2]
		ReadAng4 = ReadAng[3]
		ReadAng5 = ReadAng[4]
		ReadAng6 = ReadAng[5]

		XYZ = self.RobotArmFWD(ReadAng1,ReadAng2,ReadAng3,ReadAng4,ReadAng5,ReadAng6)
		x = XYZ[0]
		y = XYZ[1]
		z = XYZ[2]
		x_rot = XYZ[3]
		y_rot = XYZ[4]
		z_rot = XYZ[5]

		return x,y,z,x_rot,y_rot,z_rot

	def RobotArmKinematics(self):
		self.TorqueOff()
		run = True
		try:
			while run:
				ReadAng = self.ReadAngle()
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
 
				FWD = self.RobotArmFWD(ReadAng1,ReadAng2,ReadAng3,ReadAng4,ReadAng5,ReadAng6)
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
				XYZ = self.WorkspaceHorizontalLimitation(X,Y,Z)
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

	def RobotArmJogJoint(self):

		print("---------------------------------------------------")
		print("---------Press start to wake the robot up----------")
		print("---------------------------------------------------")
		waitForAwake = True
		while waitForAwake:
			Buttons = self.getButton()
			Start_Btn = Buttons[7] #Start
			if Start_Btn == 1:
				waitForAwake = False

			time.sleep(0.1)

		self.RobotArmAwake()
		print("All Torque are ON")
		time.sleep(0.5)
		self.StandByPos()
		print("On the stand by position")
		time.sleep(0.5)

		DegIncrement = 10

		################################################## Jog Joint ###################################################
		waitForStart = True
		print("Press Start Button to JogJoint!")

		while waitForStart:

			Buttons = self.getButton()
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
			Buttons = self.getButton()
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
				Buttons = self.getButton()
				J1_Btn = Buttons[0] #A
				Axes = self.getAxis()
				Ax0 = Axes[0] #Analog Left value from -1 to 1
				ReadDeg = self.ReadAngle()
				ReadDeg1 = ReadDeg[0]

				if abs(Ax0) > 0.0001:
					# Joint velocity depends on how much you push on analog stick
					DriveAngle = ReadDeg1 + DegIncrement*Ax0
					self.RunServo1(DriveAngle)


			############ Jog Joint 2 ############
			while J2_Btn == 1:

				Buttons = self.getButton()
				J2_Btn = Buttons[1] #B
				Axes = self.getAxis()
				Ax0 = Axes[0] #Analog Left value from -1 to 1
				ReadDeg = self.ReadAngle()
				ReadDeg2 = ReadDeg[1]

				if abs(Ax0) > 0.0001:
					# Joint velocity depends on how much you push on analog stick
					DriveAngle = ReadDeg2 + DegIncrement*Ax0
					self.RunServo2(DriveAngle)

			############ Jog Joint 3 ############
			while J3_Btn == 1:

				Buttons = self.getButton()
				J3_Btn = Buttons[2] #X
				Axes = self.getAxis()
				Ax0 = Axes[0] #Analog Left value from -1 to 1
				ReadDeg = self.ReadAngle()
				ReadDeg3 = ReadDeg[2]

				if abs(Ax0) > 0.0001:
					# Joint velocity depends on how much you push on analog stick
					DriveAngle = ReadDeg3 + DegIncrement*Ax0
					self.RunServo3(DriveAngle)
					#time.sleep(0.01)

			############ Jog Joint 4 ############
			while J4_Btn == 1:

				Buttons = self.getButton()
				J4_Btn = Buttons[3] #Y
				Axes = self.getAxis()
				Ax0 = Axes[0] #Analog Left value from -1 to 1
				ReadDeg = self.ReadAngle()
				ReadDeg4 = ReadDeg[3]

				if abs(Ax0) > 0.0001:
					# Joint velocity depends on how much you push on analog stick
					DriveAngle = ReadDeg4 + DegIncrement*Ax0
					self.RunServo4(DriveAngle)
					#time.sleep(0.01)

			############ Jog Joint 5 ############
			while J5_Btn == 1:

				Buttons = self.getButton()
				J5_Btn = Buttons[4] #LB
				Axes = self.getAxis()
				Ax0 = Axes[0] #Analog Left value from -1 to 1
				ReadDeg = self.ReadAngle()
				ReadDeg5 = ReadDeg[4]

				if abs(Ax0) > 0.0001:
					# Joint velocity depends on how much you push on analog stick
					DriveAngle = ReadDeg5 + DegIncrement*Ax0
					self.RunServo5(DriveAngle)
					#time.sleep(0.01)


			############ Jog Joint 6 ############
			while J6_Btn == 1:

				Buttons = self.getButton()
				J6_Btn = Buttons[5] #RB
				Axes = self.getAxis()
				Ax0 = Axes[0] #Analog Left value from -1 to 1
				ReadDeg = self.ReadAngle()
				ReadDeg6 = ReadDeg[5]

				if abs(Ax0) > 0.0001:
					# Joint velocity depends on how much you push on analog stick
					DriveAngle = ReadDeg6 + DegIncrement*Ax0
					self.RunServo6(DriveAngle)
					#time.sleep(0.01)

			if GripOpen_Btn == 1:
				self.GripOpen()

			if GripClose_Btn == 1:
				self.GripClose()

			if StandBy_Btn == 1:
				print("Robot is back to Stand by position...")
				time.sleep(1)
				self.StandByPos()
				# Set the velocity again
				self.SetProfile1(40,15)
				self.SetProfile2(40,15)
				self.SetProfile3(40,15)
				self.SetProfile4(40,15)
				self.SetProfile5(40,15)
				self.SetProfile6(40,15)
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
				self.StandByPos()
				startJog = False

			time.sleep(0.1)

		self.RobotArmGoHome()
		print("--------------------------------------------------")
		print("--------------------------------------------------")
		print("-------------------Shutdown-----------------------")
		print("--------------------------------------------------")
		print("--------------------------------------------------")
		time.sleep(1)
		self.TorqueOff()

	def RobotArmTeachPoint(self):

		waitForStart = True
		print("Press Start Button!")

		while waitForStart:

			Buttons = self.getButton()
			Start_Btn = Buttons[7] #Start

			if Start_Btn == 1:
				waitForStart = False
				startTeach = True

			time.sleep(0.1)

		print("--> You can move robot freely by your hand")
		print("--> REMEMBER...Before running teach points, the robot will always start from Stand By Poistion")
		print("            ...so the first memorized position should be near to Stand By Position")
		print("--> Press Logicool button to memorize the position")

		Mem_Ang1 = [None]*100
		Mem_Ang2 = [None]*100
		Mem_Ang3 = [None]*100
		Mem_Ang4 = [None]*100
		Mem_Ang5 = [None]*100
		Mem_Ang6 = [None]*100
		Mem_GripperStatus = [None]*100

		i = 0
		J = 0
		runTeach = False
		GripperStatus = 1 # For open at first
		self.TorqueGripperOn()

		while startTeach:

			Buttons = self.getButton()
			Back_Btn = Buttons[6] #Back
			Start_Btn = Buttons[7] #Start
			Memo_Btn = Buttons[8] #Logiccool
			GripOpen_Btn = Buttons[9] # Analog Left Push
			GripClose_Btn = Buttons[10] #Analog Right Push

			ReadANG = self.ReadAngle()
			ANG1 = ReadANG[0]
			ANG2 = ReadANG[1]
			ANG3 = ReadANG[2]
			ANG4 = ReadANG[3]
			ANG5 = ReadANG[4]
			ANG6 = ReadANG[5]

			if GripOpen_Btn == 1:
				self.GripOpen()
				GripperStatus = 1

			if GripClose_Btn == 1:
				self.GripClose()
				GripperStatus = 0

			if Memo_Btn == 1:
				ReadANG = self.ReadAngle()
				Mem_Ang1[i] = ReadANG[0]
				Mem_Ang2[i] = ReadANG[1]
				Mem_Ang3[i] = ReadANG[2]
				Mem_Ang4[i] = ReadANG[3]
				Mem_Ang5[i] = ReadANG[4]
				Mem_Ang6[i] = ReadANG[5]
				Mem_GripperStatus[i] = GripperStatus

				print("------------------------------")
				print("------------------------------")
				print("Mem_Ang1: %f" %Mem_Ang1[i])
				print("Mem_Ang2: %f" %Mem_Ang2[i])
				print("Mem_Ang3: %f" %Mem_Ang3[i])
				print("Mem_Ang4: %f" %Mem_Ang4[i])
				print("Mem_Ang5: %f" %Mem_Ang5[i])
				print("Mem_Ang6: %f" %Mem_Ang6[i])
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
					Buttons = self.getButton()
					Memo_Btn = Buttons[8] #Logiccool

				print("Teach Point: %d" %i)
				print("Press Back button if done")

			if Back_Btn == 1:
				startTeach = False
				waitForStart = True

		print(" ")
		print("--> Hold the robot with your hand in front area AWAY FROM THE FRAME")
		print(" ")
		print("=========================================")
		print("Press Start Button to run Teaching Points")
		print("=========================================")

		while waitForStart:

			Buttons = self.getButton()
			Start_Btn = Buttons[7] #Start

			if Start_Btn == 1:
				waitForStart = False
				runTeach = True

			time.sleep(0.1)

		while runTeach:

			self.TorqueOn()
			time.sleep(0.5)
			self.StandByPos()
			time.sleep(0.5)

			PreAng = self.ReadAngle()
			PreAng1 = PreAng[0]
			PreAng2 = PreAng[1]
			PreAng3 = PreAng[2]
			PreAng4 = PreAng[3]
			PreAng5 = PreAng[4]
			PreAng6 = PreAng[5]

			for K in range(0,i):

				Buttons = self.getButton()
				Back_Btn = Buttons[6] #Back

				if Back_Btn == 1:
					self.StandByPos()
					time.sleep(1)
					self.RobotArmGoHome()
					time.sleep(1)
					break

				GoalPos1 = Mem_Ang1[K]
				GoalPos2 = Mem_Ang2[K]
				GoalPos3 = Mem_Ang3[K]
				GoalPos4 = Mem_Ang4[K]
				GoalPos5 = Mem_Ang5[K]
				GoalPos6 = Mem_Ang6[K]

				DelPos1 = self.DeltaPos(PreAng1,GoalPos1)
				DelPos2 = self.DeltaPos(PreAng2,GoalPos2)
				DelPos3 = self.DeltaPos(PreAng3,GoalPos3)
				DelPos4 = self.DeltaPos(PreAng4,GoalPos4)
				DelPos5 = self.DeltaPos(PreAng5,GoalPos5)
				DelPos6 = self.DeltaPos(PreAng6,GoalPos6)

				VSTD = 100
				ASTD = 10

				TRAJ = self.TrajectoryGeneration3(VSTD,ASTD,DelPos1,DelPos2,DelPos3,DelPos4,DelPos5,DelPos6)

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

				self.SetProfile1(V1,A1)
				self.SetProfile2(V2,A2)
				self.SetProfile3(V3,A3)
				self.SetProfile4(V4,A4)
				self.SetProfile5(V5,A5)
				self.SetProfile6(V6,A6)
				self.RunServo(Mem_Ang1[K], Mem_Ang2[K], Mem_Ang3[K], Mem_Ang4[K], Mem_Ang5[K], Mem_Ang6[K])
				print("Move to point %d" %K )

				MoveType1 = self.MovingStatus1()
				MoveType2 = self.MovingStatus2()
				MoveType3 = self.MovingStatus3()
				MoveType4 = self.MovingStatus4()
				MoveType5 = self.MovingStatus5()
				MoveType6 = self.MovingStatus6()

				MovingFlag = True
				# This delay would make sure that the moving detection loop will not run too fast than actual motion
				time.sleep(0.1)
				while MovingFlag:
					Move1 = self.IsMoving1()
					Move2 = self.IsMoving2()
					Move3 = self.IsMoving3()
					Move4 = self.IsMoving4()
					Move5 = self.IsMoving5()
					Move6 = self.IsMoving6()

					if Move1 == 0 and Move2 == 0 and Move3 == 0 and Move4 == 0 and Move5 == 0 and Move6 == 0:
						MovingFlag = False
						print("Finished point %d" %K)

				if Mem_GripperStatus[K] == 1:
					self.GripOpen()
					time.sleep(0.5)
					print("Open Gripper")
				elif Mem_GripperStatus[K] == 0:
					self.GripClose()
					time.sleep(0.5)
					print("Close Gripper")

				PreAng1 = GoalPos1
				PreAng2 = GoalPos2
				PreAng3 = GoalPos3
				PreAng4 = GoalPos4
				PreAng5 = GoalPos5
				PreAng6 = GoalPos6


			waitForStartAgain = True
			print("Press start to run again")
			print("Press back to exit")

			while waitForStartAgain:
				Buttons = self.getButton()
				Back_Btn = Buttons[6] #Back
				Start_Btn = Buttons[7] #Start

				if Back_Btn == 1:
					self.StandByPos()
					self.RobotArmGoHome()
					waitForStartAgain = False
					runTeach = False
					self.TorqueOff()

				if Start_Btn == 1:
					K = 0
					waitForStartAgain = False

	def RobotArmTeachRecord(self):
		waitForStart = True
		print("--> You can move robot freely by your hand")
		print("--> REMEMBER...Before running teach points, the robot will always start from Stand By Poistion")
		print("            ...so the first memorized position should be near to Stand By Position")
		print("--> Press Start button to record the motion")

		while waitForStart:

			Buttons = self.getButton()
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
		self.TorqueGripperOn()

		i = 0
		K = 0
		############################################### Record Teaching ##################################################
		while startTeach:

			Buttons = self.getButton()
			Back_Btn = Buttons[6] #Back
			Start_Btn = Buttons[7] #Start
			GripOpen_Btn = Buttons[9] # Analog Left Push
			GripClose_Btn = Buttons[10] #Analog Right Push

			if Back_Btn == 1:
				startTeach = False
				waitForStart = True

			if GripOpen_Btn == 1:
				self.GripOpen()
				GripperStatus = 1

			if GripClose_Btn == 1:
				self.GripClose()
				GripperStatus = 0

		    
			ReadANG = self.ReadAngle()
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

			Buttons = self.getButton()
			Start_Btn = Buttons[7] #Start

			if Start_Btn == 1:
				waitForStart = False
				runTeach = True

			time.sleep(0.1)

		############################################### Run Teaching ##################################################
		while runTeach:

			self.TorqueOn()
			time.sleep(0.5)
			self.StandByPos()
			time.sleep(0.5)
			Pause = False

			self.SetProfile1(150,50)
			self.SetProfile2(150,50)
			self.SetProfile3(150,50)
			self.SetProfile4(150,50)
			self.SetProfile5(150,50)
			self.SetProfile6(150,50)

			while K < i:

				Buttons = self.getButton()
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
					Buttons = self.getButton()
					Start_Btn = Buttons[7] #Start
					if Start_Btn == 1:
						Pause = False

				self.RunServo(Mem_Ang1[K], Mem_Ang2[K], Mem_Ang3[K], Mem_Ang4[K], Mem_Ang5[K], Mem_Ang6[K])

				if Mem_GripperStatus[K] == 1:
					self.GripOpen()
					#time.sleep(1)
					print("Open Gripper")
				elif Mem_GripperStatus[K] == 0:
					self.GripClose()
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
				Buttons = self.getButton()
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
		self.StandByPos()
		time.sleep(0.5)
		self.RobotArmGoHome()
		print("-------------------------------------------------------------------")
		print("-------------------------------------------------------------------")
		print("------------------------Back to home position ---------------------")
		print("-------------------------------------------------------------------")
		print("-------------------------------------------------------------------")
		time.sleep(0.5)
		self.TorqueOff()

	def RobotArmJogLinear(self):
		print("---------------------------------------------------")
		print("---------Press start to wake the robot up----------")
		print("---------------------------------------------------")
		waitForAwake = True
		while waitForAwake:
		    Buttons = self.getButton()
		    Start_Btn = Buttons[7] #Start
		    if Start_Btn == 1:
		        waitForAwake = False
		        StartJog = True

		    time.sleep(0.1)

		self.TorqueOn()

		Vfix = 40
		Afix = 8
		incrementX = 30
		incrementY = 30
		incrementZ = 30
		RotIncrement = 5

		self.RobotArmAwake()
		time.sleep(0.5)
		self.StandByPos()
		time.sleep(0.5)
		print("On the stand by position")
		self.StandByJogLinear()

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
			Buttons = self.getButton()
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
				self.StandByJogLinear()
				Vstd = Vfix
				Astd = Afix
				self.SetProfile1(Vstd,Astd)
				self.SetProfile2(Vstd,Astd)
				self.SetProfile3(Vstd,Astd)
				self.SetProfile4(Vstd,Astd)
				self.SetProfile5(Vstd,Astd)
				self.SetProfile6(Vstd,Astd)
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
				self.StandByPos()
				time.sleep(0.5)
				self.RobotArmGoHome()
				time.sleep(0.5)
				self.TorqueOff()
				break

			OnceTrig = True
			ExitLoop = False
			while X_Btn == True:
				Buttons = self.getButton()
				X_Btn = Buttons[2] #X
				Axes = self.getAxis()
				Ax0 = Axes[0]       #Analog left push right = +1,  Analog left push left = -1
				#Ax1 = Axes[1]       #Analog left push down = +1,  Analog left push up = -1
				if OnceTrig == True:
					# make a constant value of Y and Z before jogging because these value don't change anyway
					PreReadDeg = self.ReadAngle()
					PreDeg1 = PreReadDeg[0]
					PreDeg2 = PreReadDeg[1]
					PreDeg3 = PreReadDeg[2]
					PreDeg4 = PreReadDeg[3]
					PreDeg5 = PreReadDeg[4]
					PreDeg6 = PreReadDeg[5]
					PreReadXYZ = self.RobotArmFWD(PreDeg1,PreDeg2,PreDeg3,PreDeg4,PreDeg5,PreDeg6)
					Yconst = PreReadXYZ[1]
					Zconst = PreReadXYZ[2]
					Xrot_const = PreReadXYZ[3]
					Yrot_const = PreReadXYZ[4]
					Zrot_const = PreReadXYZ[5]
					OnceTrig = False # it would not come and read this if-loop again until release X_Btn

				if ((abs(Ax0) > 0.0001) and (ExitLoop == False)):
					ReadDeg = self.ReadAngle()
					Deg1 = ReadDeg[0]
					Deg2 = ReadDeg[1]
					Deg3 = ReadDeg[2]
					Deg4 = ReadDeg[3]
					Deg5 = ReadDeg[4]
					Deg6 = ReadDeg[5]
					ReadXYZ = self.RobotArmFWD(Deg1,Deg2,Deg3,Deg4,Deg5,Deg6)
					ReadX = ReadXYZ[0]
					Xcom = ReadX + (Ax0*incrementX) 
					Ycom = Yconst
					Zcom = Zconst
					Xrot_com = Xrot_const
					Yrot_com = Yrot_const
					Zrot_com = Zrot_const
					XYZ = self.WorkspaceHorizontalLimitation(Xcom,Ycom,Zcom)
					#print("Xcom: %f" %XYZ[0])
					#print("Ycom: %f" %XYZ[1])
					#print("Zcom: %f" %XYZ[2])
					#print("------------------------------")
				if ((XYZ[0] is not None) and (XYZ[1] is not None) and (XYZ[2] is not None)):
					DriveAng = self.RobotArmINV(XYZ[0],XYZ[1],XYZ[2],Xrot_com,Yrot_com,Zrot_com)
					self.RunServo(DriveAng[0],DriveAng[1],DriveAng[2],DriveAng[3],DriveAng[4],DriveAng[5])
				else:
					print("Robot is out of workspace in horizontal constraint limit!")
					self.StandByJogLinear()
					# Set velocity back
					Vstd = Vfix
					Astd = Afix
					self.SetProfile1(Vstd,Astd)
					self.SetProfile2(Vstd,Astd)
					self.SetProfile3(Vstd,Astd)
					self.SetProfile4(Vstd,Astd)
					self.SetProfile5(Vstd,Astd)
					self.SetProfile6(Vstd,Astd)
					ExitLoop = True
					time.sleep(0.5)

			OnceTrig = True
			ExitLoop = False
			while Y_Btn == True:
				Buttons = self.getButton()
				Y_Btn = Buttons[3] #Y
				Axes = self.getAxis()
				#Ax0 = Axes[0]       #Analog left push right = +1,  Analog left push left = -1
				Ax1 = Axes[1]       #Analog left push down = +1,  Analog left push up = -1
				if OnceTrig == True:
					# make a constant value of X and Z before jogging because these value don't change anyway
					PreReadDeg = self.ReadAngle()
					PreDeg1 = PreReadDeg[0]
					PreDeg2 = PreReadDeg[1]
					PreDeg3 = PreReadDeg[2]
					PreDeg4 = PreReadDeg[3]
					PreDeg5 = PreReadDeg[4]
					PreDeg6 = PreReadDeg[5]
					PreReadXYZ = self.RobotArmFWD(PreDeg1,PreDeg2,PreDeg3,PreDeg4,PreDeg5,PreDeg6)
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
					ReadDeg = self.ReadAngle()
					Deg1 = ReadDeg[0]
					Deg2 = ReadDeg[1]
					Deg3 = ReadDeg[2]
					Deg4 = ReadDeg[3]
					Deg5 = ReadDeg[4]
					Deg6 = ReadDeg[5]
					ReadXYZ = self.RobotArmFWD(Deg1,Deg2,Deg3,Deg4,Deg5,Deg6)
					ReadY = ReadXYZ[1]
					Ycom = ReadY - (Ax1*incrementY) 
					Xcom = Xconst
					Zcom = Zconst
					Xrot_com = Xrot_const
					Yrot_com = Yrot_const
					Zrot_com = Zrot_const
					XYZ = self.WorkspaceHorizontalLimitation(Xcom,Ycom,Zcom)
					#print("Xcom: %f" %Xcom)
					#print("Ycom: %f" %Ycom)
					#print("Zcom: %f" %Zcom)
					#print("------------------------------")
					if ((XYZ[0] is not None) and (XYZ[1] is not None) and (XYZ[2] is not None)):
						DriveAng = self.RobotArmINV(XYZ[0],XYZ[1],XYZ[2],Xrot_com,Yrot_com,Zrot_com)
						self.RunServo(DriveAng[0],DriveAng[1],DriveAng[2],DriveAng[3],DriveAng[4],DriveAng[5])
					else:
						print("Robot is out of workspace in horizontal constraint limit!")
						self.StandByJogLinear()
						Vstd = Vfix
						Astd = Afix
						self.SetProfile1(Vstd,Astd)
						self.SetProfile2(Vstd,Astd)
						self.SetProfile3(Vstd,Astd)
						self.SetProfile4(Vstd,Astd)
						self.SetProfile5(Vstd,Astd)
						self.SetProfile6(Vstd,Astd)
						ExitLoop = True
						time.sleep(0.5)

			OnceTrig = True
			ExitLoop = False
			while Z_Btn == True:
				Buttons = self.getButton()
				Z_Btn = Buttons[1] #B
				Axes = self.getAxis()
				#Ax0 = Axes[0]       #Analog left push right = +1,  Analog left push left = -1
				Ax1 = Axes[1]       #Analog left push down = +1,  Analog left push up = -1
				if OnceTrig == True:
					# make a constant value of X and Y before jogging because these value don't change anyway
					PreReadDeg = self.ReadAngle()
					PreDeg1 = PreReadDeg[0]
					PreDeg2 = PreReadDeg[1]
					PreDeg3 = PreReadDeg[2]
					PreDeg4 = PreReadDeg[3]
					PreDeg5 = PreReadDeg[4]
					PreDeg6 = PreReadDeg[5]
					PreReadXYZ = self.RobotArmFWD(PreDeg1,PreDeg2,PreDeg3,PreDeg4,PreDeg5,PreDeg6)
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
					ReadDeg = self.ReadAngle()
					Deg1 = ReadDeg[0]
					Deg2 = ReadDeg[1]
					Deg3 = ReadDeg[2]
					Deg4 = ReadDeg[3]
					Deg5 = ReadDeg[4]
					Deg6 = ReadDeg[5]
					ReadXYZ = self.RobotArmFWD(Deg1,Deg2,Deg3,Deg4,Deg5,Deg6)
					ReadZ = ReadXYZ[2]
					Zcom = ReadZ - (Ax1*incrementZ) 
					Xcom = Xconst
					Ycom = Yconst
					Xrot_com = Xrot_const
					Yrot_com = Yrot_const
					Zrot_com = Zrot_const
					XYZ = self.WorkspaceHorizontalLimitation(Xcom,Ycom,Zcom)
					#print("Xcom: %f" %Xcom)
					#print("Ycom: %f" %Ycom)
					#print("Zcom: %f" %Zcom)
					#print("------------------------------")
					if ((XYZ[0] is not None) and (XYZ[1] is not None) and (XYZ[2] is not None)):
						DriveAng = self.RobotArmINV(XYZ[0],XYZ[1],XYZ[2],Xrot_com,Yrot_com,Zrot_com)
						self.RunServo(DriveAng[0],DriveAng[1],DriveAng[2],DriveAng[3],DriveAng[4],DriveAng[5])
					else:
						print("Robot is out of workspace in horizontal constraint limit!")
						self.StandByJogLinear()
						Vstd = Vfix
						Astd = Afix
						self.SetProfile1(Vstd,Astd)
						self.SetProfile2(Vstd,Astd)
						self.SetProfile3(Vstd,Astd)
						self.SetProfile4(Vstd,Astd)
						self.SetProfile5(Vstd,Astd)
						self.SetProfile6(Vstd,Astd)
						ExitLoop = True
						time.sleep(0.5)

			OnceTrig = True
			while XRot_Btn == True:
				Buttons = self.getButton()
				XRot_Btn = Buttons[0] #A
				Axes = self.getAxis()
				Ax0 = Axes[0]       #Analog left push right = +1,  Analog left push left = -1
				#Ax1 = Axes[1]       #Analog left push down = +1,  Analog left push up = -1
				if OnceTrig == True:
					PreReadDeg = self.ReadAngle()
					PreDeg1 = PreReadDeg[0]
					PreDeg2 = PreReadDeg[1]
					PreDeg3 = PreReadDeg[2]
					PreDeg4 = PreReadDeg[3]
					PreDeg5 = PreReadDeg[4]
					PreDeg6 = PreReadDeg[5]
					PreReadXYZ = self.RobotArmFWD(PreDeg1,PreDeg2,PreDeg3,PreDeg4,PreDeg5,PreDeg6)
					Xconst = PreReadXYZ[0]
					Yconst = PreReadXYZ[1]
					Zconst = PreReadXYZ[2]
					Yrot_const = PreReadXYZ[4]
					Zrot_const = PreReadXYZ[5]
					OnceTrig = False # it would not come and read this if-loop again until release X_Btn

				if ((abs(Ax0) > 0.0001) and (ExitLoop == False)):
					ReadDeg = self.ReadAngle()
					Deg1 = ReadDeg[0]
					Deg2 = ReadDeg[1]
					Deg3 = ReadDeg[2]
					Deg4 = ReadDeg[3]
					Deg5 = ReadDeg[4]
					Deg6 = ReadDeg[5]
					ReadXYZ = self.RobotArmFWD(Deg1,Deg2,Deg3,Deg4,Deg5,Deg6)
					Xcom = Xconst
					Ycom = Yconst
					Zcom = Zconst
					Xrot_com = ReadXYZ[3] + (Ax0*RotIncrement)
					Yrot_com = Yrot_const
					Zrot_com = Zrot_const
					XYZ = self.WorkspaceLimitation(Xcom,Ycom,Zcom)
					#print("Xcom: %f" %Xcom)
					#print("Ycom: %f" %Ycom)
					#print("Zcom: %f" %Zcom)
					#print("Xrot_com: %f" %Xrot_com)
					#print("Yrot_com: %f" %Yrot_com)
					#print("Zrot_com: %f" %Zrot_com)
					#print("------------------------------")
					if ((XYZ[0] is not None) and (XYZ[1] is not None) and (XYZ[2] is not None)):
						DriveAng = self.RobotArmINV(XYZ[0],XYZ[1],XYZ[2],Xrot_com,Yrot_com,Zrot_com)
						self.RunServo(DriveAng[0],DriveAng[1],DriveAng[2],DriveAng[3],DriveAng[4],DriveAng[5])
					else:
						print("Robot is out of workspace in horizontal constraint limit!")
						self.StandByJogLinear()
						Vstd = Vfix
						Astd = Afix
						self.SetProfile1(Vstd,Astd)
						self.SetProfile2(Vstd,Astd)
						self.SetProfile3(Vstd,Astd)
						self.SetProfile4(Vstd,Astd)
						self.SetProfile5(Vstd,Astd)
						self.SetProfile6(Vstd,Astd)
						ExitLoop = True
						time.sleep(0.5)


			OnceTrig = True
			while YRot_Btn == True:
				Buttons = self.getButton()
				YRot_Btn = Buttons[4] #LB
				Axes = self.getAxis()
				#Ax0 = Axes[0]       #Analog left push right = +1,  Analog left push left = -1
				Ax1 = Axes[1]       #Analog left push down = +1,  Analog left push up = -1
				if OnceTrig == True:
					PreReadDeg = self.ReadAngle()
					PreDeg1 = PreReadDeg[0]
					PreDeg2 = PreReadDeg[1]
					PreDeg3 = PreReadDeg[2]
					PreDeg4 = PreReadDeg[3]
					PreDeg5 = PreReadDeg[4]
					PreDeg6 = PreReadDeg[5]
					PreReadXYZ = self.RobotArmFWD(PreDeg1,PreDeg2,PreDeg3,PreDeg4,PreDeg5,PreDeg6)
					Xconst = PreReadXYZ[0]
					Yconst = PreReadXYZ[1]
					Zconst = PreReadXYZ[2]
					Xrot_const = PreReadXYZ[3]
					Zrot_const = PreReadXYZ[5]
					OnceTrig = False # it would not come and read this if-loop again until release X_Btn

				if ((abs(Ax1) > 0.0001) and (ExitLoop == False)):
					ReadDeg = self.ReadAngle()
					Deg1 = ReadDeg[0]
					Deg2 = ReadDeg[1]
					Deg3 = ReadDeg[2]
					Deg4 = ReadDeg[3]
					Deg5 = ReadDeg[4]
					Deg6 = ReadDeg[5]
					ReadXYZ = self.RobotArmFWD(Deg1,Deg2,Deg3,Deg4,Deg5,Deg6)
					Xcom = Xconst
					Ycom = Yconst
					Zcom = Zconst
					Yrot_com = ReadXYZ[4] - (Ax1*RotIncrement)
					Xrot_com = Xrot_const
					Zrot_com = Zrot_const
					XYZ = self.WorkspaceLimitation(Xcom,Ycom,Zcom)
					#print("Xcom: %f" %Xcom)
					#print("Ycom: %f" %Ycom)
					#print("Zcom: %f" %Zcom)
					#print("Xrot_com: %f" %Xrot_com)
					#print("Yrot_com: %f" %Yrot_com)
					#print("Zrot_com: %f" %Zrot_com)
					#print("------------------------------")
					if ((XYZ[0] is not None) and (XYZ[1] is not None) and (XYZ[2] is not None)):
						DriveAng = self.RobotArmINV(XYZ[0],XYZ[1],XYZ[2],Xrot_com,Yrot_com,Zrot_com)
						self.RunServo(DriveAng[0],DriveAng[1],DriveAng[2],DriveAng[3],DriveAng[4],DriveAng[5])
					else:
						print("Robot is out of workspace in horizontal constraint limit!")
						self.StandByJogLinear()
						Vstd = Vfix
						Astd = Afix
						self.SetProfile1(Vstd,Astd)
						self.SetProfile2(Vstd,Astd)
						self.SetProfile3(Vstd,Astd)
						self.SetProfile4(Vstd,Astd)
						self.SetProfile5(Vstd,Astd)
						self.SetProfile6(Vstd,Astd)
						ExitLoop = True
						time.sleep(0.5)

			OnceTrig = True
			while ZRot_Btn == True:
				Buttons = self.getButton()
				ZRot_Btn = Buttons[5] #RB
				Axes = self.getAxis()
				#Ax0 = Axes[0]       #Analog left push right = +1,  Analog left push left = -1
				Ax1 = Axes[1]       #Analog left push down = +1,  Analog left push up = -1
				if OnceTrig == True:
					PreReadDeg = self.ReadAngle()
					PreDeg1 = PreReadDeg[0]
					PreDeg2 = PreReadDeg[1]
					PreDeg3 = PreReadDeg[2]
					PreDeg4 = PreReadDeg[3]
					PreDeg5 = PreReadDeg[4]
					PreDeg6 = PreReadDeg[5]
					PreReadXYZ = self.RobotArmFWD(PreDeg1,PreDeg2,PreDeg3,PreDeg4,PreDeg5,PreDeg6)
					Xconst = PreReadXYZ[0]
					Yconst = PreReadXYZ[1]
					Zconst = PreReadXYZ[2]
					Xrot_const = PreReadXYZ[3]
					Yrot_const = PreReadXYZ[4]
					OnceTrig = False # it would not come and read this if-loop again until release X_Btn

				if ((abs(Ax1) > 0.0001) and (ExitLoop == False)):
					ReadDeg = self.ReadAngle()
					Deg1 = ReadDeg[0]
					Deg2 = ReadDeg[1]
					Deg3 = ReadDeg[2]
					Deg4 = ReadDeg[3]
					Deg5 = ReadDeg[4]
					Deg6 = ReadDeg[5]
					ReadXYZ = self.RobotArmFWD(Deg1,Deg2,Deg3,Deg4,Deg5,Deg6)
					Xcom = Xconst
					Ycom = Yconst
					Zcom = Zconst
					Zrot_com = ReadXYZ[5] - (Ax1*RotIncrement)
					Xrot_com = Xrot_const
					Yrot_com = Yrot_const
					XYZ = self.WorkspaceLimitation(Xcom,Ycom,Zcom)
					#print("Xcom: %f" %Xcom)
					#print("Ycom: %f" %Ycom)
					#print("Zcom: %f" %Zcom)
					#print("Xrot_com: %f" %Xrot_com)
					#print("Yrot_com: %f" %Yrot_com)
					#print("Zrot_com: %f" %Zrot_com)
					#print("------------------------------")
					if ((XYZ[0] is not None) and (XYZ[1] is not None) and (XYZ[2] is not None)):
						DriveAng = self.RobotArmINV(XYZ[0],XYZ[1],XYZ[2],Xrot_com,Yrot_com,Zrot_com)
						self.RunServo(DriveAng[0],DriveAng[1],DriveAng[2],DriveAng[3],DriveAng[4],DriveAng[5])
					else:
						print("Robot is out of workspace in horizontal constraint limit!")
						self.StandByJogLinear()
						Vstd = Vfix
						Astd = Afix
						self.SetProfile1(Vstd,Astd)
						self.SetProfile2(Vstd,Astd)
						self.SetProfile3(Vstd,Astd)
						self.SetProfile4(Vstd,Astd)
						self.SetProfile5(Vstd,Astd)
						self.SetProfile6(Vstd,Astd)
						ExitLoop = True
						time.sleep(0.5)

			if GripOpen_Btn == True:
				self.GripOpen()
				time.sleep(0.5)
			if GripClose_Btn == True:
				self.GripClose()
				time.sleep(0.5)

	def DynamicExample(self):

		WakeUpStatus = self.RobotArmAwake()		# If robot is in sleep position, this command would wake the robot up
		time.sleep(0.5)

		if WakeUpStatus:
			self.StandByPos()						# Move the robot to the stand by position
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
				PreAng = self.ReadAngle()
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
					XYZ = self.WorkspaceLimitation(Mem_X[K],Mem_Y[K],Mem_Z[K])
					Com_X = XYZ[0]
					Com_Y = XYZ[1]
					Com_Z = XYZ[2]
					if Com_X == None or Com_Y == None or Com_Z == None:
						break
					else:
						Mem_ANG = self.RobotArmINV(Com_X,Com_Y,Com_Z,X6_ROT,Y6_ROT,Z6_ROT)

						GoalPos1 = Mem_ANG[0]
						GoalPos2 = Mem_ANG[1]
						GoalPos3 = Mem_ANG[2]
						GoalPos4 = Mem_ANG[3]
						GoalPos5 = Mem_ANG[4]
						GoalPos6 = Mem_ANG[5]
						# Calculate difference between goal position and current position
						DelPos1 = self.DeltaPos(PreAng1,GoalPos1)
						DelPos2 = self.DeltaPos(PreAng2,GoalPos2)
						DelPos3 = self.DeltaPos(PreAng3,GoalPos3)
						DelPos4 = self.DeltaPos(PreAng4,GoalPos4)
						DelPos5 = self.DeltaPos(PreAng5,GoalPos5)
						DelPos6 = self.DeltaPos(PreAng6,GoalPos6)
						# specify the velocity and acceralation standard
						VSTD = 100
						ASTD = 12
						# Calculate how much velocity and accerelation in each joint to make it stop at the same time
						# TrajectoryGeneration3 is a function to generate joint velocity profile to make robot looks natural movement
						TRAJ = self.TrajectoryGeneration3(VSTD,ASTD,DelPos1,DelPos2,DelPos3,DelPos4,DelPos5,DelPos6)

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
						self.SetProfile1(V1,A1)
						self.SetProfile2(V2,A2)
						self.SetProfile3(V3,A3)
						self.SetProfile4(V4,A4)
						self.SetProfile5(V5,A5)
						self.SetProfile6(V6,A6)
						# Drive servo to goal position according to that velocity
						self.RunServo(GoalPos1,GoalPos2,GoalPos3,GoalPos4,GoalPos5,GoalPos6)
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
						MoveType1 = self.MovingStatus1()
						MoveType2 = self.MovingStatus2()
						MoveType3 = self.MovingStatus3()
						MoveType4 = self.MovingStatus4()
						MoveType5 = self.MovingStatus5()
						MoveType6 = self.MovingStatus6()

						MovingFlag = True
						# This delay would make sure that the moving detection loop will not run too fast than actual motion
						time.sleep(0.1)
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
							Move1 = self.IsMoving1()
							Move2 = self.IsMoving2()
							Move3 = self.IsMoving3()
							Move4 = self.IsMoving4()
							Move5 = self.IsMoving5()
							Move6 = self.IsMoving6()

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
							self.StandByPos()

		# To stop the robot, you just hit Ctrl+C then the robot will stop there
		# Torque is still ON, so you can turn your power supply or switch off to release the torque
		# BE CAREFUL, robot will fall down immediately after you turn off
		except(KeyboardInterrupt, SystemExit):
			print("End program...")

	def HarmAvoiding(self):
		PreCur = self.ReadAllCurrent()
		PreCur1 = PreCur[0]
		PreCur2 = PreCur[1]
		PreCur3 = PreCur[2]
		PreCur4 = PreCur[3]
		PreCur5 = PreCur[4]
		PreCur6 = PreCur[5]
		OverLoad1 = abs(PreCur1) >= self.GoalCur1
		OverLoad2 = abs(PreCur2) >= self.GoalCur2
		OverLoad3 = abs(PreCur3) >= self.GoalCur3
		OverLoad4 = abs(PreCur4) >= self.GoalCur4
		OverLoad5 = abs(PreCur5) >= self.GoalCur5
		OverLoad6 = abs(PreCur6) >= self.GoalCur6

		if OverLoad1 or OverLoad2 or OverLoad3 or OverLoad4 or OverLoad5 or OverLoad6:
			
			# For stop the motion
			ReadAng = self.ReadAngle()
			self.RunServo(ReadAng[0],ReadAng[1],ReadAng[2],ReadAng[3],ReadAng[4],ReadAng[5])
			#time.sleep(0.5)

			# For make the overload servo turn itself back a bit, avoid squeezing to object
			ShiftAng = 10
			if OverLoad1:
				self.RunServo1(ReadAng[0]-(ShiftAng*abs(PreCur1)/PreCur1))
			elif OverLoad2:
				self.RunServo2(ReadAng[1]-(ShiftAng*abs(PreCur2)/PreCur2))
			elif OverLoad3:
				self.RunServo3(ReadAng[2]-(ShiftAng*abs(PreCur3)/PreCur3))
			elif OverLoad4:
				self.RunServo4(ReadAng[3]-(ShiftAng*abs(PreCur4)/PreCur4))
			elif OverLoad5:
				self.RunServo5(ReadAng[4]-(ShiftAng*abs(PreCur5)/PreCur5))
			elif OverLoad6:
				self.RunServo6(ReadAng[5]-(ShiftAng*abs(PreCur6)/PreCur6))

			# wait for start button, or can be replaced with delay timer
			print "Press start to continue run again..."
			waitForStart = True
			while waitForStart:
				Buttons = self.getButton()
				Start_Btn = Buttons[7] #Start

				if Start_Btn == 1:
					waitForStart = False

				time.sleep(0.1)					