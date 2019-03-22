#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelState,ModelStates
from reinforcment.srv import PlaceActionMsg
from reinforcment.srv import PickActionMsg
from reinforcment.srv import CheckEdge
from reinforcment.srv import IsTerminalState
from std_msgs.msg import String
from reinforcment.srv import RemoveBlockedEdgeMsg
from reinforcment.srv import MoveActionMsg
import numpy as np
import tf
import math

class RobotActionsServer:
	def __init__(self,object_dict):
		# rospy.init_node("action_server")
		self.object_dict = object_dict
		self.failure = -1
		self.success = 1
		self.empty = True
		self.status = String(data='Idle')
		self.world_state = None
		self.carring = None 
		self.placed_books = []
		self.model_state_publisher = rospy.Publisher("/gazebo/set_model_state",ModelState,queue_size = 10)
		self.action_publisher = rospy.Publisher("/actions",String,queue_size= 10)
		self.status_publisher = rospy.Publisher("/status",String,queue_size=10)
		self.state_subscriber = rospy.Subscriber("/gazebo/model_states",ModelStates,self.state_callback)
		rospy.Service("execute_careful_place",PlaceActionMsg,self.execute_careful_place)
		rospy.Service("execute_careful_pick",PickActionMsg,self.execute_careful_pick)
		rospy.Service("execute_careful_moveF",MoveActionMsg,self.execute_careful_moveF)
		rospy.Service("execute_careful_TurnCW",MoveActionMsg,self.execute_careful_TurnCW)
		rospy.Service("execute_careful_TurnCCW",MoveActionMsg,self.execute_normal_TurnCCW)
		rospy.Service("execute_normal_place",PlaceActionMsg,self.execute_normal_place)
		rospy.Service("execute_normal_pick",PickActionMsg,self.execute_normal_pick)
		rospy.Service("execute_normal_moveF",MoveActionMsg,self.execute_normal_moveF)
		rospy.Service("execute_normal_TurnCW",MoveActionMsg,self.execute_normal_TurnCW)
		rospy.Service("execute_normal_TurnCCW",MoveActionMsg,self.execute_normal_TurnCCW)
		print "Action Server Initiated"

	def state_callback(self,data):
		self.world_state = data

	def get_turtlebot_location(self,state):
		str_part1 = state.split("turtlebot3_waffle,")[1]
		str_part2 = str_part1.split(")")[0]
		x,y,orientation = str_part2.split(",")
		return abs(float(x)),abs(float(y)),orientation

	def generate_current_state(self):
		state = ""
		for i in range(len(self.world_state.name)):
			obj =self.world_state.name[i]
			if obj in self.object_dict["books"] or obj in self.object_dict["bins"] or obj == "turtlebot3_waffle":
				if obj == "turtlebot3_waffle":
					x = self.world_state.pose[i].position.x
					y = self.world_state.pose[i].position.y
					orientation = self.world_state.pose[i].orientation
					quat = (orientation.x,orientation.y,orientation.z,orientation.w)
					yaw = tf.transformations.euler_from_quaternion(quat)[2]
					if yaw > (-math.pi /4.0) and yaw < (math.pi / 4.0):
						direction = "EAST"
					elif yaw > (math.pi /4.0) and yaw < (3*math.pi / 4.0):
						direction = "NORTH"
					elif yaw > (-3.0*math.pi /4.0) and yaw < (-math.pi /4.0):
						direction = "SOUTH"
					else:
						direction = "WEST"
					state+="(turtlebot3_waffle, {},{},{});".format(round(x*2)/2.0,round(2*y)/2.0,direction)
				else:
					x = self.world_state.pose[i].position.x
					y = self.world_state.pose[i].position.y
					state+="({},{},{});".format(obj,round(x,1),round(y,1))
		for book in self.placed_books:
			state+="(placed {});".format(book)
		if self.carring is not None:
			state+="(in_basket {});".format(self.carring)
		return state


	def change_state(self,book_name,target_transform):
		model_state_msg = ModelState()
		model_state_msg.model_name = book_name
		model_state_msg.pose.position.x = target_transform[0]
		model_state_msg.pose.position.y = target_transform[1]
		model_state_msg.pose.position.z = target_transform[2]
		self.model_state_publisher.publish(model_state_msg)

	def remove_edge(self,book_name):
		rospy.wait_for_service('remove_blocked_edge')
		try:
			remove_edge = rospy.ServiceProxy('remove_blocked_edge',RemoveBlockedEdgeMsg)
			_ = remove_edge(book_name)
		except rospy.ServiceException,e:
			print "Sevice call failed: %s"%e

	def check_edge(self,x1,y1,x2,y2):
		print "Checking edge {},{},{},{}".format(x1,y1,x2,y2)
		rospy.wait_for_service('check_is_edge')
		try:
			check_is_edge = rospy.ServiceProxy('check_is_edge',CheckEdge)
			if x1 <= x2 and y1 <= y2:
				result = check_is_edge(x1,y1,x2,y2)
			else:
				result = check_is_edge(x2,y2,x1,y1)
				print "Edge: ",result.value
			return result.value == 1
		except rospy.ServiceException,e:
			print "Sevice call failed: %s"%e

	def is_terminal_state(self,state):
		rospy.wait_for_service('is_terminal_state')
		try:
			is_term = rospy.ServiceProxy('is_terminal_state',IsTerminalState)
			response = is_term(state)
			return response.value == 1
		except rospy.ServiceException,e:
			print "Sevice call failed: %s"%e

	def execute_careful_place(self,req):
		if not self.is_terminal_state(self.generate_current_state()):
			if np.random.random() < 0.85:
				book_name = req.book_name
				bin_name = req.bin_name
				robot_state = self.get_turtlebot_location(self.generate_current_state())
				if book_name in self.object_dict["books"] and bin_name in self.object_dict["bins"]:
					if (robot_state[0],robot_state[1]) in self.object_dict["bins"][bin_name]["load_loc"]:
						goal_loc = list(self.object_dict["bins"][bin_name]["loc"])
						goal_loc[0] = goal_loc[0] + 0.5
						goal_loc[1] = goal_loc[1] + 0.5
						self.change_state(book_name, goal_loc + [3])
						rospy.Rate(1).sleep()
						self.empty = True
						self.carring = None
						self.placed_books.append(book_name)
						self.status_publisher.publish(self.status)
						if self.object_dict["books"][book_name]["size"] == self.object_dict["bins"][bin_name]["size"] and self.object_dict["books"][book_name]["subject"] == self.object_dict["bins"][bin_name]["subject"]:
							return self.success,self.generate_current_state(),500
						else:
							return self.success,self.generate_current_state(),-25
				self.status_publisher.publish(self.status)
				return self.failure,self.generate_current_state(),-25
			else:
				return self.failure,self.generate_current_state(),-25
		else:
			return self.failure,self.generate_current_state(),0

	def execute_careful_pick(self,req):
		if not self.is_terminal_state(self.generate_current_state()):
			if np.random.random() <  0.85:
				book_name = req.book_name
				robot_state = self.get_turtlebot_location(self.generate_current_state())
				if book_name in self.object_dict["books"]:
					if (robot_state[0],robot_state[1]) in self.object_dict["books"][book_name]["load_loc"]:
						if self.empty:
							self.change_state(book_name,list(robot_state[:2])+[2])
							self.empty = False
							self.carring = book_name
							rospy.Rate(1).sleep()
							_ = self.remove_edge(book_name)
							self.status_publisher.publish(self.status)
							return self.success,self.generate_current_state(),-25
				self.status_publisher.publish(self.status)
				return self.failure,self.generate_current_state(),-25
			else:
				return self.failure,self.generate_current_state(),-25
		else:
			return self.failure,self.generate_current_state(),0


	def execute_normal_place(self,req):
		if not self.is_terminal_state(self.generate_current_state()):
			if np.random.random() < 0.60:
				book_name = req.book_name
				bin_name = req.bin_name
				robot_state = self.get_turtlebot_location(self.generate_current_state())
				if book_name in self.object_dict["books"] and bin_name in self.object_dict["bins"]:
					if (robot_state[0],robot_state[1]) in self.object_dict["bins"][bin_name]["load_loc"]:
						goal_loc = list(self.object_dict["bins"][bin_name]["loc"])
						goal_loc[0] = goal_loc[0] + 0.5
						goal_loc[1] = goal_loc[1] + 0.5
						self.change_state(book_name, goal_loc + [3])
						rospy.Rate(1).sleep()
						self.empty = True
						self.carring = None
						self.placed_books.append(book_name)
						self.status_publisher.publish(self.status)
						if self.object_dict["books"][book_name]["size"] == self.object_dict["bins"][bin_name]["size"] and self.object_dict["books"][book_name]["subject"] == self.object_dict["bins"][bin_name]["subject"]:
							return self.success,self.generate_current_state(),500
						else:
							return self.success,self.generate_current_state(),-10
				self.status_publisher.publish(self.status)
				return self.failure,self.generate_current_state(),-10
			else:
				return self.failure,self.generate_current_state(),-10
		else:
			return self.failure,self.generate_current_state(),0

	def execute_normal_pick(self,req):
		if not self.is_terminal_state(self.generate_current_state()):
			if np.random.random() <  0.60:
				book_name = req.book_name
				robot_state = self.get_turtlebot_location(self.generate_current_state())
				if book_name in self.object_dict["books"]:
					if (robot_state[0],robot_state[1]) in self.object_dict["books"][book_name]["load_loc"]:
						if self.empty:
							self.change_state(book_name,list(robot_state[:2])+[2])
							self.empty = False
							self.carring = book_name
							rospy.Rate(1).sleep()
							_ = self.remove_edge(book_name)
							self.status_publisher.publish(self.status)
							return self.success,self.generate_current_state(),-10
				self.status_publisher.publish(self.status)
				return self.failure,self.generate_current_state(),-10
			else:
				return self.failure,self.generate_current_state(),-10
		else:
			return self.failure,self.generate_current_state(),0

	def execute_careful_moveF(self,req):
		if not self.is_terminal_state(self.generate_current_state()): 
			if np.random.random() < 0.85:
				robot_state = self.get_turtlebot_location(self.generate_current_state())
				x1 = robot_state[0]
				y1 = robot_state[1]
				if "EAST" in robot_state[2]:
					x2 = x1 + 0.5
					y2 = y1
				elif "WEST" in robot_state[2]:
					x2 = x1 - 0.5
					y2 = y1
				elif "NORTH" in robot_state[2]:
					x2 = x1
					y2 = y1 + 0.5
				else:
					x2 = x1
					y2 = y1 - 0.5
				if self.check_edge(x1,x2,y1,y2):
					action_str = "MoveF"
					self.action_publisher.publish(String(data=action_str))
					rospy.wait_for_message("/status",String)
					return self.success,self.generate_current_state(),-10
				else:
					return self.failure,self.generate_current_state(),-10
			else:
				return self.failure,self.generate_current_state(),-10
		else:
			return self.failure,self.generate_current_state(),0

	def execute_careful_TurnCW(self,req):
		if not self.is_terminal_state(self.generate_current_state()):
			if np.random.random() < 0.85:
				action_str = "TurnCW"
				self.action_publisher.publish(String(data=action_str))
				rospy.wait_for_message("/status",String)
				return self.success,self.generate_current_state(),-10
			else:
				action_str = "TurnCCW"
				self.action_publisher.publish(String(data=action_str))
				rospy.wait_for_message("/status",String)
				return self.failure,self.generate_current_state(),-10
		else:
			return self.failure,self.generate_current_state(),0

	def execute_careful_TurnCCW(self,req):
		if not self.is_terminal_state(self.generate_current_state()):
			if np.random.random() < 0.85:
				action_str = "TurnCCW"
				self.action_publisher.publish(String(data=action_str))
				rospy.wait_for_message("/status",String)
				return self.success,self.generate_current_state(),-10
			else:
				action_str = "TurnCW"
				self.action_publisher.publish(String(data=action_str))
				rospy.wait_for_message("/status",String)
				return self.failure,self.generate_current_state(),-10
		else:
			return self.failure,self.generate_current_state(),0


	def execute_normal_moveF(self,req):
		if not self.is_terminal_state(self.generate_current_state()):
			if np.random.random() < 0.60:
				robot_state = self.get_turtlebot_location(self.generate_current_state())
				x1 = robot_state[0]
				y1 = robot_state[1]
				print robot_state
				if "EAST" in robot_state[2]:
					x2 = x1 + 0.5
					y2 = y1
				elif "WEST" in robot_state[2]:
					x2 = x1 - 0.5
					y2 = y1
				elif "NORTH" in robot_state[2]:
					x2 = x1
					y2 = y1 + 0.5
				else:
					x2 = x1
					y2 = y1 - 0.5
				if self.check_edge(x1,y1,x2,y2):
					action_str = "MoveF"
					self.action_publisher.publish(String(data=action_str))
					rospy.wait_for_message("/status",String)
					return self.success,self.generate_current_state(),-2
				else:
					return self.failure,self.generate_current_state(),-2
			else:
				return self.failure,self.generate_current_state(),-2
		else:
			return self.failure,self.generate_current_state(),0

	def execute_normal_TurnCW(self,req):
		if not self.is_terminal_state(self.generate_current_state()):
			if np.random.random() < 0.60:
				action_str = "TurnCW"
				self.action_publisher.publish(String(data=action_str))
				rospy.wait_for_message("/status",String)
				return self.success,self.generate_current_state(),-2
			else:
				action_str = "TurnCCW"
				self.action_publisher.publish(String(data=action_str))
				rospy.wait_for_message("/status",String)
				return self.failure,self.generate_current_state(),-2
		else:
			return self.failure,self.generate_current_state(),0

	def execute_normal_TurnCCW(self,req):
		if not self.is_terminal_state(self.generate_current_state()):
			if np.random.random() < 0.60:
				action_str = "TurnCCW"
				self.action_publisher.publish(String(data=action_str))
				rospy.wait_for_message("/status",String)
				return self.success,self.generate_current_state(),-2
			else:
				action_str = "TurnCW"
				self.action_publisher.publish(String(data=action_str))
				rospy.wait_for_message("/status",String)
				return self.failure,self.generate_current_state(),-2
		else:
			return self.failure,self.generate_current_state(),0


if __name__ == "__main__":
	object_dict = None
	RobotActionsServer(object_dict)