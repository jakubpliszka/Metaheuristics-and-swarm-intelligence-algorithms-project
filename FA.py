#!/usr/bin/env python
#------------------------------------------------------------------------------+
#
#   Nathan A. Rooy
#   Simple Particle Swarm Optimization (PSO) with Python
#   July, 2016
#
#------------------------------------------------------------------------------+

#--- IMPORT DEPENDENCIES ------------------------------------------------------+

from __future__ import division
import random
import math
import rospy 
import numpy
import time
import matplotlib.pyplot as plt

from FireflyAlgorithm import *

from   nav_msgs.msg            import Odometry
from   ardrone_autonomy.msg    import Navdata
from   geometry_msgs.msg       import Twist
from   Rotation_Transformacion import *
from   std_msgs.msg 	       import Int32

#--- COST FUNCTION ------------------------------------------------------------+
IAE = 0
IAE_plot = []
publishing = 0

# function we are attempting to optimize (minimize)
def Fun(D, x):
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

		Cnt_Z_SI   = pid_controller(9.474734238856335  ,  3.229160332828819 , 5.639394865567629) 
		Cnt_X_SI   = pid_controller(8.642134298152383  ,  3.108677603244394 ,  1.1313240972424565) 
		Cnt_Y_SI   = pid_controller(8.642134298152383  ,  3.108677603244394 ,  1.1313240972424565)
		Cnt_Yaw_SI = pid_controller(PID_PSO[0]  ,  PID_PSO[1] , PID_PSO[2])


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






Algorithm = FireflyAlgorithm(3, 3, 9, 0.5, 0.2, 1.0, 0.0, 10.0, Fun)
Best = Algorithm.Run()

print (Best)
plt.plot(IAE_plot)
plt.show()

#--- END ----------------------------------------------------------------------+