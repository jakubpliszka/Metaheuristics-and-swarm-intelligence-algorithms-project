#!/usr/bin/env python

#--- IMPORT DEPENDENCIES ------------------------------------------------------+

from __future__ import division
import random
import math, copy
import rospy 
import numpy
import time
import csv
import sys
import matplotlib.pyplot as plt

from   nav_msgs.msg            import Odometry
from   ardrone_autonomy.msg    import Navdata
from   geometry_msgs.msg       import Twist
from   Rotation_Transformacion import *
from   std_msgs.msg 	       import Int32

IAE = 0
IAE_plot = []
publishing = 0

# function we are attempting to optimize (minimize)
def func1(D, x):
	global IAE
	IAE = 0
	rospy.init_node('belbic_controller',anonymous=True)
	a = LecturePosicion()
	a.calculos(x)
	rospy.on_shutdown(shutdown_callback)
	#rospy.spin()
	IAE_plot.append(IAE)
	return IAE

#--- MAIN ---------------------------------------------------------------------+
class LecturePosicion:

	def __init__(self):
		
		rospy.Subscriber("ground_truth/state" ,   Odometry,  self.pose_callback)
		rospy.Subscriber("ardrone/navdata"    ,   Navdata ,  self.ang_callback)
		rospy.Subscriber("position_referencia",   Twist   ,  self.droneReferencia)
		rospy.Subscriber("nazwa",   Int32   ,  self.break_point)
		
		self.movePub = rospy.Publisher ("cmd_vel", Twist, queue_size=10)
		global publishing

		#Initial Values

		self.posx_dron = 0.0
		self.posy_dron = 0.0
		self.posz_dron = 0.0
		
		self.roll_dron = 0.0 
		self.pich_dron = 0.0 
		self.yaw_dron  = 0.0

		self.X_d       = 0.0   
		self.Y_d       = 0.0
		self.Z_d       = 2.0
		self.yaw_d     = 0.0

		self.Vth       = 0.0
		self.w         = 0.0
		self.v         = 0.0

		self.alfa      = 0.00002
		self.beta      = 0.00009

		self.f = open("iaeerror.csv", "w")

	
	def break_point(self, data):
		global publishing
		if data.data == 1:
			publishing = 1 

	def pose_callback(self, pose_data):
	
		self.posx_dron = pose_data.pose.pose.position.x
		self.posy_dron = pose_data.pose.pose.position.y
		self.posz_dron = pose_data.pose.pose.position.z

	def ang_callback(self, ang_data):

		self.roll_dron  = ang_data.rotX  # angle of the drone in gradients
		self.pich_dron  = ang_data.rotY  # angle of the drone in gradients
		self.yaw_dron   = ang_data.rotZ  # angle of the drone in gradients

	def droneReferencia(self, data):

		self.X_d   = data.linear.x  # References points
		self.Y_d   = data.linear.y
		self.Z_d   = data.linear.z
		self.yaw_d = data.angular.z


	def calculos(self, PID_PSO):
		
		global IAE
		global publishing
		global IAE_plot
		
		rate = rospy.Rate(10) # 10hz

		Cnt_Z_REW   = pd_controller(5.4  ,  0.5) 
		Cnt_X_REW   = pd_controller(4.0  ,  7.5) 
		Cnt_Y_REW   = pd_controller(4.0  ,  7.5)
		Cnt_Yaw_REW = pd_controller(0.02 , 0.00)

		Cnt_Z_SI   = pid_controller(0.5  ,  0.04 , 0.00) 
		Cnt_X_SI   = pid_controller(PID_PSO[0]  ,  PID_PSO[1] , PID_PSO[2]) 
		Cnt_Y_SI   = pid_controller(PID_PSO[0]  ,  PID_PSO[1] , PID_PSO[2])
		Cnt_Yaw_SI = pid_controller(0.02 , 0.00  , 0.00)


		while publishing == 0:

				teta 	  = [ 0, 0, self.yaw_dron]
				R    	  = eulerAnglesToRotationMatrix(teta)
				T_inversa = Homogenius_Inversa(R, 0,0,0)
				position  = np.array ([[self.posx_dron], [self.posy_dron],[self.posz_dron], [1] ])
				dron      = np.dot   (T_inversa , position) 
				
				x_dron = dron[0]
				y_dron = dron[1]
				z_dron = dron[2]

				position_deseada    = np. array ([[self.X_d],[self.Y_d],[self.Z_d],[1]])
				positionDesada_dron = np.dot(T_inversa , position_deseada) 


				xd_dron = positionDesada_dron[0]
				yd_dron = positionDesada_dron[1]
				zd_dron = positionDesada_dron[2]
				
				error_x   = xd_dron  - x_dron
				error_y   = yd_dron  - y_dron  
				error_z   = self.Z_d    - self.posz_dron
				error_yaw = self.yaw_d  - self.yaw_dron
				
				IAE += float(abs(error_yaw))

				print("IAE: ", IAE)
				print("PID: ", PID_PSO)
				
				#self.f.write(str(error_x[0]) + ";\n")


				#  REW signal for BELBIC

				REW_Z   = Cnt_Z_REW.set_REW(error_z) 
				REW_X   = Cnt_X_REW.set_REW(error_x) 
				REW_Y   = Cnt_Y_REW.set_REW(error_y)
				REW_YAW = Cnt_Yaw_REW.set_REW(error_yaw) 

				#  SI signal for BELBIC
				SI_Z   =  Cnt_Z_SI.set_SI(error_z)
				SI_X   =  Cnt_X_SI.set_SI(error_x) 
				SI_Y   =  Cnt_Y_SI.set_SI(error_y)
				SI_YAW =  Cnt_Yaw_SI.set_SI(error_yaw)


				U1 = self.Belbic(SI_Z,   REW_Z,    0.7)
				U4 = self.Belbic(SI_YAW, REW_YAW,  0.2) 
				U2 = self.Belbic(SI_X,   REW_X,    1.0) 
				U3 = self.Belbic(SI_Y,   REW_Y,    1.0) 


				self.Action(U1,U2,U3,U4)
				rate.sleep()
		self.f.close()
		publishing = 0

	def Belbic (self, SI, REW, limite):

		_limit_out = limite

		A   = (self.v   * SI)
		O   = (self.w   * SI)
		MO  = (A - O) 

		#Ath = (self.Vth * SI)
		#MO  = ((Ath + A) - O) 

		rest = REW - A

		if rest < 0 :
		   rest = 0
		else:
		   pass

		dv   = self.alfa * (rest)     * SI
		dw   = self.beta * (MO - REW) * SI
		#dvth = self.alfa * (rest)     * SI
		#print dv 

		self.v   = self.v + dv  
		self.w   = self.w + dw  
		#self.Vth = self.Vth + dvth 

		U = MO

		if   U  >   _limit_out:
			 U  =   _limit_out
		elif U  <  -_limit_out:
			 U  =  -_limit_out
		
		return U



	def Action (self, U1, U2, U3, U4):

		twist = Twist()
		
		twist.linear.z  = U1
		twist.linear.x  = U2
		twist.linear.y  = U3
		twist.angular.z = U4

		self.movePub.publish(twist)

class pd_controller:

	def __init__(self, p_coef, d_coef):
		
		self.kp 				   = p_coef
		self.kd                    = d_coef
		self._previous_error	   = 0.0
		self._is_error_initialized = False

	def set_REW (self, error):

		output   = error * self.kp

		if self._is_error_initialized:

			error_diff           =  error   - self._previous_error
			output               += self.kd * error_diff
			self._previous_error = error

		else:

			self._previous_error       = error
			self._is_error_initialized = True
		
		return output


class pid_controller:

	def __init__(self, p_coef, d_coef, i_coef):
		
		self.kp = p_coef
		self.kd = d_coef
		self.ki = i_coef

		self._last_time      = 0.0
		self.error_integ     = 0.0
		self._previous_error = 0.0

		self._i_max =  5.0  # The integral upper limit.
		self._i_min = -5.0  # The integral lower limit.
		

		self._is_error_initialized_PID = False


	def set_SI (self, error):

		cur_time = time.time()
		output   = self.kp * error

		if self._is_error_initialized_PID:

			dt                   = cur_time - self._last_time
			self._last_time      = cur_time
			self.error_integ     += error * dt


			#error_diff           = (error - self._previous_error) / dt
			error_diff           = error - self._previous_error 
			self._previous_error = error

			derivativa           = self.kd *  error_diff
			integral             = self.ki *  self.error_integ
			
			if  integral  > self._i_max:
				integral  = self._i_max
			elif integral < self._i_min:
				integral  = self._i_min

			output +=  derivativa + integral 

		else:
			
			self._previous_error           = error
			self._last_time                = cur_time
			self._is_error_initialized_PID = True
		
		return output


def shutdown_callback():
	print "Shutting down position controller."



################# BA ######################
class Bat:
	def __init__(self, D, A, r, F_min, F_max, X_min, X_max, fun, constrains):
		self.D = D	# Dimensions
		self.N_gen = 1 # Generation number
		self.X_min = X_min
		self.X_max = X_max
		self.X = []
		for i in range(self.D):
			rnd = random.random()
			self.X += [self.X_min[i] + (self.X_max[i] - self.X_min[i]) * rnd]
		self.X = np.array(self.X)
		#[X_min + (X_max - X_min)*random.random() for i in range(D) ] #X velocity
		self.A = A
		self.r = r	#something
		self.F_min = F_min
		self.F_max = F_max
		self.F = self.F_min + (self.F_max - self.F_min) * random.uniform(0, 1)		#Frequency
		#self.sol = sys.maxsize	#maximum size of int
		self.V = [0]*D	#Velocity
		self.V = np.array(self.V)
		self.fun = fun
		self.constrains = constrains
		self.sol = self.fun(self.D, self.X)
	def getArray(self):
		li = [a for a in self.X]
		li += [self.sol]
		return li
	
	def __str__(self):
		s = ""
		for k in self.X:
			s += " "+str(k)
		s += "  ans = "+str(self.sol)		
		return s
	
	def updateFrequency(self, rand_num):
		self.F = self.F_min + rand_num * (self.F_max - self.F_min)	# Beta should be random vector from uniform distribution
	
	def adjust_range(self):
		#while self.constrains(self.D , self.X) == False:
		# 	 self.randomize_position()
		while True:
			for i in range(self.D):
				if self.X[i] > self.X_max[i]:
					self.X[i] = self.X_max[i]
				if self.X[i] < self.X_min[i]:
					self.X[i] = self.X_min[i]
			if not self.constrains(self.D, self.X):
				self.randomize_position()
			else:
				break
		self.sol = self.fun(self.D, self.X)
			#self.randomize_position()
	def randomize_position(self):
		for i in range(0, self.D):
			self.X[i] = self.X_min[i] + (self.X_max[i] - self.X_min[i])*random.random()

	def updateVelocity(self, b):
		self.V = np.add(self.V, np.subtract(b.X, self.X) * self.F)
		"""
		for i in range(self.D):
			self.V[i] = self.V[i] + (b.X[i] - self.X[i])*self.F
		"""

	def move(self):
		self.X = np.add(self.X, self.V)
		"""
		for i in range(self.D):
			#self.V[i] = self.V[i] + (b.X[i] - self.X[i])*self.F
			print(self.X[i], " + ", self.V[i], end=" ")
			self.X[i] = self.X[i] + self.V[i]
			print(" = ", self.X[i])
		"""
		self.adjust_range()
		
	def jump(self, b):	# I guess lavy flight
		for i in range(self.D):
			self.X[i] = b.X[i] + 0.3 * random.gauss(0, 1)
		self.adjust_range()
		
	def next_generation(self):
		self.N_gen += 1
	
	def changeA(self, a):
		self.A = self.A * a
	
	def changeR(self, g):
		self.r = self.r * (1 - math.exp(-g))
	
	def getCopy(self):
		return copy.deepcopy(self)

def constraints(D, sol):
	return True

class BatAlgorithm:
	def __init__(self, D, NP, N_gen, A_min, A_max, F_min, F_max, X_min, X_max, fitness_fun, constrains):
		self.D = D	#Dimention
		self.NP = NP	#Population
		self.N_gen = N_gen	# Number of generations
		self.A_min = A_min	#min loudness
		self.A_max = A_max	#max Loudness
		
		self.F_min = F_min	#min frequency
		self.F_max = F_max	#max frequency
		self.fitness_fun = fitness_fun	#fitness function

		self.X_min = X_min	#min X
		self.X_max = X_max	#max X
		
		self.alpha = 0.95
		self.gamma = 0.05
		
		self.constrains = constrains

		#self.Fitness = [0.0]*NP
		self.bats = []
		
		for i in range(NP):
			b = Bat(D, A_min + (A_max - A_min)*random.random(), random.random(), F_min, F_max, X_min, X_max, fitness_fun, constrains)
			b.sol = self.fitness_fun(self.D, b.X)
			self.bats += [b]

		self.best_bat = self.get_best_bat()

	def get_best_bat(self):
		i = 0
		for j in range(0, self.NP):
			if(self.bats[i].sol > self.bats[j].sol):
				i = j
		return self.bats[i]

	def move_bats(self):
		const_sol = self.best_bat.sol
		counter = 0
		with open("batalgo.csv","w") as batfile:
			for b in  self.bats:
				b.sol = b.fun(b.D, b.X)

			"""
			plt.axis([self.X_min , self.X_max, 0, 26])
			x = np.arange(self.X_min, self.X_max, 0.1)
			y = x**2
			"""
			writer=csv.writer(batfile)
			#for t in range(1, self.N_gen + 1):
			t = 0
			while True:
				t += 1
				cnt = 1
				x_graph, y_graph = [], []
				
				for index, bat in enumerate(self.bats):
						rnd_num = random.uniform(0, 1)	# Returns number in [0.0, 1.0)
						self.bats[index].updateFrequency(rnd_num)
						self.bats[index].updateVelocity(self.best_bat)
						tmp_bat = self.bats[index].getCopy()

						tmp_bat.move()

						#tmp_bat.sol = self.fitness_fun(tmp_bat.D, tmp_bat.X)

						rnd_num = random.random()	# Returns number in [0.0, 1.0]

						if rnd_num > self.bats[index].r:
							tmp_best_bat = self.get_best_bat()
							tmp_bat.jump(tmp_best_bat)
							#break
						
						tmp_bat.sol = self.fitness_fun(self.D, tmp_bat.X)
						
						rnd_num = random.random()
						print(self.bats[index].X, self.bats[index].sol)
						if rnd_num < tmp_bat.A and tmp_bat.sol < self.bats[index].sol:	#my change
							#print("prev = ", bat.X, end = " ")
							self.bats[index] = tmp_bat.getCopy()	#Accept bat
							self.bats[index].changeA(self.alpha)
							self.bats[index].changeR(self.gamma*t)
							#flag = 1
						if self.bats[index].sol < self.best_bat.sol:
							#print("Bat sol = ", bat.sol)
							self.best_bat = bat.getCopy()
							#flag = 1
						# if flag:
						# 	break
						
						cnt += 1
					
						y_graph += [self.bats[index].sol]
						x_graph += [self.bats[index].X[0]]
				if const_sol == self.best_bat.sol:
					counter += 1
				else:
					const_sol= self.best_bat.sol
					counter = 0
				if counter > 1	:
					print("Breaked")
					break
				"""
				plt.clf()
				plt.scatter(np.array(x_graph), np.array(y_graph))
				plt.plot(x, y, 'C1')
				
				plt.pause(0.5)
				"""
				#plt.show()
				writer.writerow(self.best_bat.getArray())
				#print("Yoo ", self.best_bat)
		print("\n\nGenerations: ", self.N_gen)
		print("Position: ", self.best_bat.X)
		print("Minimized value = ", self.best_bat.sol)
		return self.best_bat.X

#--- RUN ----------------------------------------------------------------------+

xmin = [0.0, 0.0, 0.0]
xmax = [10.0, 10.0, 1.0]
Algorithm = BatAlgorithm(3, 2, 1, 0.0, 0.5, 0.0, 2.0, xmin, xmax, func1, constraints)
Algorithm.move_bats()
plt.plot(IAE_plot)
plt.xlabel('Lap number')
plt.ylabel('IAE')
plt.grid()
plt.show()

#--- END ----------------------------------------------------------------------+