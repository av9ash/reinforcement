#! /usr/bin/python

import rospy
import math
import numpy as np
import time
import tf

from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry

class PID(object):

	def __init__(self, target_pose, mode, direction):
		self.target_pose = target_pose
		self.mode = mode
		self.direction = direction
		self.current_pose = Pose()
		self.init_pose = Pose()
		self.init_val_flag = 0
		self.vel = Twist()
		
		self.KP = 1.0
		self.KD = 0.5
		self.KP_rot = 5.0
		self.KD_rot = 100.0
		self.KP_rot_angular = 5.0
		self.KD_rot_angular = 100.0

		self.max_vel = 0.45
		self.max_rot = 3.0
		self.max_rot_angular = 0.3
		
		self.p_error_x = 0.0
		self.p_error_last_x = 0.0
		self.d_error_x = 0.0

		self.p_error_angular_z_linear = 0.0
		self.p_error_angular_z_linear_last = 0.0
		self.d_error_angular_z_linear  = 0.0
		
		self.p_error_angular_z_rot = 0.0
		self.p_error_angular_z_rot_last = 0.0
		self.d_error_angular_z_rot  = 0.0

		self.last_time = None

		rospy.Subscriber('/odom', Odometry, self.pose_callback, queue_size=1)
		self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
		self.controller_status_publisher = rospy.Publisher('/Controller_Status', String, queue_size=1)

	def set_current_pose(self, current_pose):
		self.current_pose = current_pose
		if self.init_val_flag == 0:
			self.init_pose = self.current_pose
			self.init_val_flag = 1

	def euler_from_pose(self, pose):
		quat = (pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quat)
		return euler

	def get_error(self, target_pose):
		error_position_x = self.target_pose.position.x - self.current_pose.position.x
		error_position_y = self.target_pose.position.y - self.current_pose.position.y

		euler_target = self.euler_from_pose(target_pose)
		euler_current = self.euler_from_pose(self.current_pose)
		
		error_orientation_z = euler_target[2] - euler_current[2]

		return error_position_x, error_position_y, error_orientation_z

	def linear_error_conditions(self, goal_pose):
		error_dist_x,error_dist_y,_ = self.get_error(goal_pose)
		#print error_dist_x
		if error_dist_x<0 and (self.target_pose.orientation.z==1.0 or self.target_pose.orientation.z<-0.8):
			#print "Cond 1"
			error = -(error_dist_x)
		elif error_dist_x>0 and (self.target_pose.orientation.z==1.0 or self.target_pose.orientation.z<-0.8):
			#print "Cond 2"
			error = error_dist_x
		elif error_dist_y<0 and self.target_pose.orientation.z<-0.5:
			#print "Cond 3"
			error = -(error_dist_y)
		elif error_dist_y>0 and self.target_pose.orientation.z<-0.5:
			#print "Cond 4"
			error = -error_dist_y
		elif error_dist_y>0 and self.target_pose.orientation.z>0.1 and self.target_pose.orientation.z<0.8:
			#print "Cond 5"
			error = error_dist_y
		elif error_dist_y<0 and self.target_pose.orientation.z>0.1 and self.target_pose.orientation.z<0.8:
			#print "Cond 6"
			error = error_dist_y
		elif error_dist_x<0 and self.target_pose.orientation.z==0.0:
			#print "Cond 7"
			error = error_dist_x
		elif error_dist_x>0 and self.target_pose.orientation.z==0.0:
			#print "Cond 8"
			error = error_dist_x
		else:
			#print "Cond 9"
			error = error_dist_x 
		
		return error


	def linear_vel(self, goal_pose, dt=None):
		if dt == None:
			cur_time = time.time()
			if self.last_time is None:
				self.last_time = cur_time
			dt = cur_time - self.last_time
			self.last_time = cur_time
		
		
		self.p_error_x = self.linear_error_conditions(goal_pose)
		
		if dt == 0.0:
			return self.KP * self.max_vel * self.p_error_x

		self.d_error_x = (self.p_error_last_x - self.p_error_x) / dt
		
		self.p_error_last_x = self.p_error_x
		
		return (self.KP * self.max_vel * self.p_error_x) + (self.KD * self.max_vel * self.d_error_x)

	def get_steering_angle(self, goal_pose):
		target_euler = self.euler_from_pose(goal_pose)
		current_euler = self.euler_from_pose(self.current_pose)
		
		if target_euler[2]>3 and current_euler[2]<0:
			steer_orientation = -(target_euler[2] - abs(current_euler[2]))
		else:
			steer_orientation = target_euler[2] - current_euler[2]
		
		if self.direction == 'y':
			steer_positional_diff = math.atan2(goal_pose.position.x - self.current_pose.position.x, goal_pose.position.y - self.current_pose.position.y)
		elif self.direction == 'x':
			steer_positional_diff = math.atan2(goal_pose.position.y - self.current_pose.position.y, goal_pose.position.x - self.current_pose.position.x)
		elif self.direction == '-x':
			steer_positional_diff = math.atan2(abs(goal_pose.position.y) - abs(self.current_pose.position.y), abs(goal_pose.position.x) - abs(self.current_pose.position.x))
		elif self.direction == '-y':
			steer_positional_diff = math.atan2(abs(goal_pose.position.x) - abs(self.current_pose.position.x), abs(goal_pose.position.y) - abs(self.current_pose.position.y))
		else:
			return steer_orientation, 0.0, current_euler


		if (self.direction=='y' or self.direction=='-y') and ((abs(self.current_pose.position.y) - abs(self.init_pose.position.y))<0.001 or (abs(self.target_pose.position.y) - abs(self.current_pose.position.y))<0.0001):
			return steer_orientation, 0.0, current_euler
		elif (self.direction=='x' or self.direction=='-x') and ((abs(self.current_pose.position.x) - abs(self.init_pose.position.x))<0.001 or (abs(self.target_pose.position.x) - abs(self.current_pose.position.x))<0.0001):
			# print "This Condition"
			return steer_orientation, 0.0, current_euler
		elif target_euler[2]<-0.5 and self.mode=='linear':
			return steer_orientation, steer_positional_diff, current_euler
		elif target_euler[2]>3 and self.mode=='linear':
			return steer_orientation, steer_positional_diff, current_euler
		elif self.direction=='x':
			return steer_orientation, steer_positional_diff, current_euler
		else:
			return steer_orientation, -steer_positional_diff, current_euler

		

	def angular_vel(self, goal_pose, dt=None):
		if dt == None:
			cur_time = time.time()
			if self.last_time is None:
				self.last_time = cur_time
			dt = cur_time - self.last_time
			self.last_time = cur_time

		error_angular_z, differntial_error, current_euler = self.get_steering_angle(goal_pose)
		if dt == 0.0:
			if self.mode=='linear':
				self.p_error_angular_z_linear = error_angular_z + differntial_error
				return (self.max_rot * self.KP_rot * self.p_error_angular_z_linear)
			elif self.mode=='rotational':
				self.p_error_angular_z_rot = error_angular_z
				return (self.max_rot_angular * self.KP_rot_angular * self.p_error_angular_z_rot)
			
		
		
		if self.mode=='linear':
			self.p_error_angular_z_linear = error_angular_z + differntial_error
			self.d_error_angular_z_linear = (self.p_error_angular_z_linear - self.p_error_angular_z_linear_last) / dt
			self.p_error_angular_z_linear_last = self.p_error_angular_z_linear
			return ((self.max_rot *self.KP_rot * self.p_error_angular_z_linear) + (self.max_rot*self.KD_rot*self.d_error_angular_z_linear))
		
		elif self.mode=='rotational':
			self.p_error_angular_z_rot = error_angular_z
			self.d_error_angular_z_rot = (self.p_error_angular_z_rot - self.p_error_angular_z_rot_last) / dt
			self.p_error_angular_z_rot_last = self.p_error_angular_z_rot

			return (self.max_rot_angular * self.KP_rot_angular * self.p_error_angular_z_rot) + (self.KD_rot_angular * self.max_rot_angular * self.d_error_angular_z_rot)

		
			

		

	def publish_velocity(self):

		rospy.sleep(1)
		start_time = time.time()
		step_time = time.time()
		
		error_x = abs(abs(self.target_pose.position.x) - abs(self.current_pose.position.x))
		error_y = abs(abs(self.target_pose.position.y) - abs(self.current_pose.position.y))
		if self.target_pose.orientation.z==1.0:
			current_euler = self.euler_from_pose(self.current_pose)
			target_euler = self.euler_from_pose(self.target_pose)
			angular_error = abs(target_euler[2] - abs(current_euler[2]))
		else:
			current_euler = self.euler_from_pose(self.current_pose)
			target_euler = self.euler_from_pose(self.target_pose)
			angular_error = abs(target_euler[2] - current_euler[2])

		if len(self.direction)==1:
			direction = self.direction
		else:
			direction = list(self.direction)[1]
		
		if self.mode == 'linear':
			error = 'error_'+direction
			while eval(error)>0.001:
				linear_velocity = self.linear_vel(self.target_pose)
				self.vel.linear.x = linear_velocity
				self.vel.linear.y = 0.0
				self.vel.linear.z = 0.0
				self.vel.angular.x = 0.0
				self.vel.angular.y = 0.0
				self.vel.angular.z = self.angular_vel(self.target_pose)

				self.velocity_publisher.publish(self.vel)
				error_x = abs(abs(self.target_pose.position.x) - abs(self.current_pose.position.x))
				error_y = abs(abs(self.target_pose.position.y) - abs(self.current_pose.position.y))
			
			self.vel.linear.x = 0.0
			self.vel.linear.y = 0.0
			self.vel.linear.z = 0.0
			self.vel.angular.x = 0.0
			self.vel.angular.y = 0.0
			self.vel.angular.z = 0.0

			self.velocity_publisher.publish(self.vel)
			
			self.mode = 'rotational'
			
			if self.target_pose.orientation.z==1.0:
				current_euler = self.euler_from_pose(self.current_pose)
				target_euler = self.euler_from_pose(self.target_pose)
				angular_error = abs(target_euler[2] - abs(current_euler[2]))
			else:
				current_euler = self.euler_from_pose(self.current_pose)
				target_euler = self.euler_from_pose(self.target_pose)
				angular_error = abs(target_euler[2] - current_euler[2])
			while angular_error>0.001:
				angular_velocity = self.angular_vel(self.target_pose)
				self.vel.linear.x = 0.0
				self.vel.linear.y = 0.0
				self.vel.linear.z = 0.0
				self.vel.angular.x = 0.0
				self.vel.angular.y = 0.0
				self.vel.angular.z = angular_velocity

				self.velocity_publisher.publish(self.vel)
				if self.target_pose.orientation.z==1.0:
					current_euler = self.euler_from_pose(self.current_pose)
					target_euler = self.euler_from_pose(self.target_pose)
					angular_error = abs(target_euler[2] - abs(current_euler[2]))
				else:
					current_euler = self.euler_from_pose(self.current_pose)
					target_euler = self.euler_from_pose(self.target_pose)
					angular_error = abs(target_euler[2] - current_euler[2])

			self.mode= 'linear'

		

		elif self.mode == 'rotational':
			
			while angular_error>0.001:
				#print angular_error
				angular_velocity = self.angular_vel(self.target_pose)
				self.vel.linear.x = 0.0
				self.vel.linear.y = 0.0
				self.vel.linear.z = 0.0
				self.vel.angular.x = 0.0
				self.vel.angular.y = 0.0
				self.vel.angular.z = angular_velocity

				self.velocity_publisher.publish(self.vel)
				if self.target_pose.orientation.z==1.0:
					current_euler = self.euler_from_pose(self.current_pose)
					target_euler = self.euler_from_pose(self.target_pose)
					angular_error = abs(target_euler[2] - abs(current_euler[2]))
				else:
					current_euler = self.euler_from_pose(self.current_pose)
					target_euler = self.euler_from_pose(self.target_pose)
					angular_error = abs(target_euler[2] - current_euler[2])
		
		self.vel.linear.x = 0.0
		self.vel.linear.y = 0.0
		self.vel.linear.z = 0.0
		self.vel.angular.x = 0.0
		self.vel.angular.y = 0.0
		self.vel.angular.z = 0.0

		self.velocity_publisher.publish(self.vel)

		self.controller_status_publisher.publish('Done')
		

	def pose_callback(self, msg):
		self.set_current_pose(msg.pose.pose)


if __name__ == "__main__":
	rospy.init_node('PID_Node')
	