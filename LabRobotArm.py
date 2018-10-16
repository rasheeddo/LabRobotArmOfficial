import time
from dynamixel_sdk import *                    # Uses Dynamixel SDK library
import numpy
import math

class RobotArm:

	def __init__(self):
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
		####################################################### Set Servo Configuration #############################################################
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

	def SetGoalCurrentGripper(self,SetCur):

		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL7_ID, self.ADDR_PRO_GOAL_CURRENT, SetCur)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error))
		else:
			print("Goal Current is set")

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

		px = qx - d6*wx
		py = qy - d6*wy
		pz = qz - d6*wz


		################################ Find q1 #####################################
		q1 = math.atan(py/px)

		if ((q1 <= pi) and (q1 >= 0)):
			q1_1 = q1
			q1_2 = q1 + pi
		else:
			q1_1 = q1 + pi
			q1_2 = q1
			q1 = q1 + pi
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
					print("Work space is valid, in quadrant2")
					return x,y,z
				else:
					print("ERROR: Out of work range in quadrant 2 or 3")
					return [None]*3
			elif y > 0 and z < 0:
				if z > h:
					print("Work space is valid, in quadrant4")
					return x,y,z
				else:
					print("ERROR: z is lower than lowest range 'h'")
					return [None]*3
			elif y > 0 and z > 0:
				print("Work space is valid, in quadrant1")
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
					print("Work space is valid")
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

	def GripOpen(self):
		dxl7_goal_position = 1700
		dxl_comm_result7, dxl_error7 = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL7_ID, self.ADDR_PRO_GOAL_POSITION, dxl7_goal_position)

	def GripClose(self):
		dxl7_goal_position = 2780
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

		dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(portHandler, self.DXL2_ID, self.ADDR_PRO_PROFILE_ACCELERATION, int(set_A_PRFL))
		dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(portHandler, self.DXL2_ID, self.ADDR_PRO_PROFILE_VELOCITY, int(set_V_PRFL))

		acceleration_limit, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.ortHandler, self.DXL2_ID, self.ADDR_PRO_ACCELERATION_LIMIT)
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

	def StandByPos(self):

		# Last Home Position #
		Last_Ang1 = 282.637363
		Last_Ang2 = 185.846154
		Last_Ang3 = 106.109890
		Last_Ang4 = 197.186813
		Last_Ang5 = 152.879121
		Last_Ang6 = 184.087912
		InitAng = self.ReadAngle()
		Deg1 = InitAng[0]
		Deg2 = InitAng[1]
		Deg3 = InitAng[2]
		Deg4 = InitAng[3]
		Deg5 = InitAng[4]
		Deg6 = InitAng[5]
		if (abs(Deg1 - Last_Ang1) < 40) and (abs(Deg2 - Last_Ang2) < 40) and (abs(Deg3 - Last_Ang3) < 40) and (abs(Deg4 - Last_Ang4) < 40) and (abs(Deg5 - Last_Ang5) < 40) and (abs(Deg6 - Last_Ang6) < 40):
			print("Robot is in sleep position")
			print("Run [RobotArmAwake] first, then [StandByPos]")
			time.sleep(1)
			self.TorqueOff()
		else:
			print("Robot arm is going to standby position")

			Vstd = 20
			Astd = 5

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
			dxl_comm_result2, dxl_error2 = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_GOAL_POSITION, dxl2_goal_position)
			dxl_comm_result3, dxl_error3 = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_GOAL_POSITION, dxl3_goal_position)
			dxl_comm_result4, dxl_error4 = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_PRO_GOAL_POSITION, dxl4_goal_position)
			dxl_comm_result5, dxl_error5 = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL5_ID, self.ADDR_PRO_GOAL_POSITION, dxl5_goal_position)
			dxl_comm_result6, dxl_error6 = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL6_ID, self.ADDR_PRO_GOAL_POSITION, dxl6_goal_position)

			MovingFlag = True

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

		# Last Home Position #
		Last_Ang1 = 282.637363
		Last_Ang2 = 185.846154
		Last_Ang3 = 106.109890
		Last_Ang4 = 197.186813
		Last_Ang5 = 152.879121
		Last_Ang6 = 184.087912
		InitAng = self.ReadAngle()
		Deg1 = InitAng[0]
		Deg2 = InitAng[1]
		Deg3 = InitAng[2]
		Deg4 = InitAng[3]
		Deg5 = InitAng[4]
		Deg6 = InitAng[5]
		if (abs(Deg1 - Last_Ang1) < 40) and (abs(Deg2 - Last_Ang2) < 40) and (abs(Deg3 - Last_Ang3) < 40) and (abs(Deg4 - Last_Ang4) < 40) and (abs(Deg5 - Last_Ang5) < 40) and (abs(Deg6 - Last_Ang6) < 40):
			print("Robot is already in sleep position")
			time.sleep(1)
			self.TorqueOff()
		else:
			print("Robot arm is going to sleep position")

			Vstd = 20
			Astd = 5

			self.SetProfile1(Vstd,Astd)
			self.SetProfile2(Vstd,Astd)
			self.SetProfile3(Vstd,Astd)
			self.SetProfile4(Vstd,Astd)
			self.SetProfile5(Vstd,Astd)
			self.SetProfile6(Vstd,Astd)
			#GripOpen()
			#### Point 0 ####
			Mem_Ang1 = 271.296703
			Mem_Ang2 = 194.021978
			Mem_Ang3 = 157.714286
			Mem_Ang4 = 182.769231
			Mem_Ang5 = 176.000000
			Mem_Ang6 = 181.186813
			self.RunServo(Mem_Ang1,Mem_Ang2,Mem_Ang3,Mem_Ang4,Mem_Ang5,Mem_Ang6)
			time.sleep(1)

			#### Point 1 ####
			Mem_Ang1 = 278.417582
			Mem_Ang2 = 203.956044
			Mem_Ang3 = 157.978022
			Mem_Ang4 = 181.714286
			Mem_Ang5 = 123.164835
			Mem_Ang6 = 184.263736
			self.RunServo(Mem_Ang1,Mem_Ang2,Mem_Ang3,Mem_Ang4,Mem_Ang5,Mem_Ang6)
			time.sleep(1)

			#### Point 2 ####
			Mem_Ang1 = 278.593407
			Mem_Ang2 = 220.131868
			Mem_Ang3 = 149.714286
			Mem_Ang4 = 182.857143
			Mem_Ang5 = 102.769231
			Mem_Ang6 = 183.560440
			self.RunServo(Mem_Ang1,Mem_Ang2,Mem_Ang3,Mem_Ang4,Mem_Ang5,Mem_Ang6)
			time.sleep(1)

			#### Point 3 ####
			Mem_Ang1 = 283.956044
			Mem_Ang2 = 207.472527
			Mem_Ang3 = 116.307692
			Mem_Ang4 = 188.307692
			Mem_Ang5 = 132.747253
			Mem_Ang6 = 187.516484
			self.RunServo(Mem_Ang1,Mem_Ang2,Mem_Ang3,Mem_Ang4,Mem_Ang5,Mem_Ang6)
			time.sleep(1)

			#### Point 4 ####
			Mem_Ang1 = 282.285714
			Mem_Ang2 = 188.483516
			Mem_Ang3 = 107.164835
			Mem_Ang4 = 194.021978
			Mem_Ang5 = 148.043956
			Mem_Ang6 = 184.087912
			self.RunServo(Mem_Ang1,Mem_Ang2,Mem_Ang3,Mem_Ang4,Mem_Ang5,Mem_Ang6)
			time.sleep(1)

			#### Point 5 ####
			Mem_Ang1 = 282.637363
			Mem_Ang2 = 185.846154
			Mem_Ang3 = 106.109890
			Mem_Ang4 = 197.186813
			Mem_Ang5 = 152.879121
			Mem_Ang6 = 184.087912
			self.RunServo(Mem_Ang1,Mem_Ang2,Mem_Ang3,Mem_Ang4,Mem_Ang5,Mem_Ang6)
			time.sleep(1)

	def RobotArmAwake(self):

		Vstd = 20
		Astd = 5

		self.SetProfile1(Vstd,Astd)
		self.SetProfile2(Vstd,Astd)
		self.SetProfile3(Vstd,Astd)
		self.SetProfile4(Vstd,Astd)
		self.SetProfile5(Vstd,Astd)
		self.SetProfile6(Vstd,Astd)

		# Last Home Position #
		Last_Ang1 = 282.637363
		Last_Ang2 = 185.846154
		Last_Ang3 = 106.109890
		Last_Ang4 = 197.186813
		Last_Ang5 = 152.879121
		Last_Ang6 = 184.087912
		InitAng = self.ReadAngle()
		Deg1 = InitAng[0]
		Deg2 = InitAng[1]
		Deg3 = InitAng[2]
		Deg4 = InitAng[3]
		Deg5 = InitAng[4]
		Deg6 = InitAng[5]
		if (abs(Deg1 - Last_Ang1) < 40) and (abs(Deg2 - Last_Ang2) < 40) and (abs(Deg3 - Last_Ang3) < 40) and (abs(Deg4 - Last_Ang4) < 40) and (abs(Deg5 - Last_Ang5) < 40) and (abs(Deg6 - Last_Ang6) < 40):
			print("Robot is in sleep position")
			time.sleep(1)
			AwakenOK = True
			self.TorqueOn()
			self.GripOpen()
			#### Point 0 ####
			Mem_Ang1 = 277.802198
			Mem_Ang2 = 194.021978
			Mem_Ang3 = 110.329670
			Mem_Ang4 = 190.593407
			Mem_Ang5 = 142.593407
			Mem_Ang6 = 177.934066
			self.RunServo(Mem_Ang1,Mem_Ang2,Mem_Ang3,Mem_Ang4,Mem_Ang5,Mem_Ang6)
			time.sleep(0.5)

			#### Point 1 ####
			Mem_Ang1 = 273.670330
			Mem_Ang2 = 199.384615
			Mem_Ang3 = 125.802198
			Mem_Ang4 = 186.901099
			Mem_Ang5 = 119.912088
			Mem_Ang6 = 177.934066
			self.RunServo(Mem_Ang1,Mem_Ang2,Mem_Ang3,Mem_Ang4,Mem_Ang5,Mem_Ang6)
			time.sleep(0.5)

			#### Point 2 ####
			Mem_Ang1 = 271.560440
			Mem_Ang2 = 201.670330
			Mem_Ang3 = 152.351648
			Mem_Ang4 = 184.967033
			Mem_Ang5 = 96.351648
			Mem_Ang6 = 175.296703
			self.RunServo(Mem_Ang1,Mem_Ang2,Mem_Ang3,Mem_Ang4,Mem_Ang5,Mem_Ang6)
			time.sleep(0.5)

			#### Point 3 ####
			Mem_Ang1 = 270.505495
			Mem_Ang2 = 213.890110
			Mem_Ang3 = 169.934066
			Mem_Ang4 = 184.439560
			Mem_Ang5 = 86.681319
			Mem_Ang6 = 174.153846
			self.RunServo(Mem_Ang1,Mem_Ang2,Mem_Ang3,Mem_Ang4,Mem_Ang5,Mem_Ang6)
			time.sleep(0.5)

			#### Point 4 ####
			Mem_Ang1 = 272.175824
			Mem_Ang2 = 219.164835
			Mem_Ang3 = 155.604396
			Mem_Ang4 = 182.593407
			Mem_Ang5 = 180.219780
			Mem_Ang6 = 183.120879
			self.RunServo(Mem_Ang1,Mem_Ang2,Mem_Ang3,Mem_Ang4,Mem_Ang5,Mem_Ang6)
			time.sleep(0.5)

			#### Point 5 ####
			Mem_Ang1 = 224.351648
			Mem_Ang2 = 175.560440
			Mem_Ang3 = 224.967033
			Mem_Ang4 = 173.450549
			Mem_Ang5 = 233.318681
			Mem_Ang6 = 181.098901
			self.RunServo(Mem_Ang1,Mem_Ang2,Mem_Ang3,Mem_Ang4,Mem_Ang5,Mem_Ang6)
			time.sleep(0.5)

			#### Point 6 ####
			Mem_Ang1 = 180.659341
			Mem_Ang2 = 182.153846
			Mem_Ang3 = 186.373626
			Mem_Ang4 = 171.164835
			Mem_Ang5 = 222.857143
			Mem_Ang6 = 177.230769
			self.RunServo(Mem_Ang1,Mem_Ang2,Mem_Ang3,Mem_Ang4,Mem_Ang5,Mem_Ang6)
			time.sleep(0.5)

			#### Point 7 ####
			Mem_Ang1 = 177.494505
			Mem_Ang2 = 207.120879
			Mem_Ang3 = 111.208791
			Mem_Ang4 = 174.065934
			Mem_Ang5 = 222.857143
			Mem_Ang6 = 185.582418
			self.RunServo(Mem_Ang1,Mem_Ang2,Mem_Ang3,Mem_Ang4,Mem_Ang5,Mem_Ang6)
			time.sleep(2)

		else:
			print("WARNING!: Robot is not in sleep position")
			AwakenOK = False
			self.TorqueOff()

		return AwakenOK
