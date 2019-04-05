#!/usr/bin/env python
import sys
import rospy
from reinforcement.srv import *
from std_msgs.msg import String


def get_current_state():
    """
    This function calls get_initial_state service to recive the initial state of the turtlebot.

    return:  x_cord - initial x-cordinate of turtlebot           
             y_cord - initial y-cordinate of turtlebot
             direction - initial orientation
    """
    rospy.wait_for_service('get_current_state')
    try:
        get_initial_state = rospy.ServiceProxy('get_current_state', GetInitialState)
        response = get_initial_state()
        return response.state
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def is_terminal_state():
    rospy.wait_for_service('is_terminal_state')
    try:
        is_term_state = rospy.ServiceProxy('is_terminal_state', IsTerminalState)
        response = is_term_state()
        return response.value
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def reset_world():
    rospy.wait_for_service('reset_world')
    try:
        handle = rospy.ServiceProxy('reset_world',ResetWorldMsg)
        response = handle()
    except rospy.ServiceException, e:
        print "Service call failedL %s"%e

def get_all_actions():
    """
    This function calls is_goal_state service to check if the current state is the goal state or not.

    parameters:  x_cord - current x-cordinate of turtlebot           return:   1 : if current state is the goal state
                 y_cord - current y-cordinate of turtlebot                     0 : if current state is not the goal state
    """
    rospy.wait_for_service('get_all_actions')
    try:
        all_actions = rospy.ServiceProxy('get_all_actions', GetActions)
        response = all_actions()
        return response.actions.split(";")
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def execute_careful_pick(book_name):
    rospy.wait_for_service('execute_careful_pick')
    try:
        pick_action = rospy.ServiceProxy('execute_careful_pick',PickActionMsg)
        response = pick_action(book_name)
        return response.result,response.next_state,response.reward
    except rospy.ServiceException,e:
        print "Sevice call failed: %s"%e

def execute_careful_place(book_name,bin_name):
    rospy.wait_for_service('execute_careful_place')
    try:
        place_action = rospy.ServiceProxy('execute_careful_place',PlaceActionMsg)
        response = place_action(book_name,bin_name)
        return response.result,response.next_state,response.reward
    except rospy.ServiceException,e:
        print "Sevice call failed: %s"%e

def execute_careful_moveF():
    rospy.wait_for_service('execute_careful_moveF')
    try:
        careful_moveF = rospy.ServiceProxy("execute_careful_moveF",MoveActionMsg)
        response = careful_moveF()
        return response.success,response.next_state,response.reward
    except rospy.ServiceException,e:
        print "Sevice call failed: %s"%e

def execute_careful_TurnCW():
    rospy.wait_for_service('execute_careful_TurnCW')
    try:
        careful_TurnCW = rospy.ServiceProxy("execute_careful_TurnCW",MoveActionMsg)
        response = careful_TurnCW()
        return response.success,response.next_state,response.reward
    except rospy.ServiceException,e:
        print "Sevice call failed: %s"%e

def execute_careful_TurnCCW():
    rospy.wait_for_service('execute_careful_TurnCCW')
    try:
        careful_TurnCCW = rospy.ServiceProxy("execute_careful_TurnCCW",MoveActionMsg)
        response = careful_TurnCCW()
        return response.success,response.next_state,response.reward
    except rospy.ServiceException,e:
        print "Sevice call failed: %s"%e



def execute_normal_pick(book_name):
    rospy.wait_for_service('execute_normal_pick')
    try:
        pick_action = rospy.ServiceProxy('execute_normal_pick',PickActionMsg)
        response = pick_action(book_name)
        return response.result,response.next_state,response.reward
    except rospy.ServiceException,e:
        print "Sevice call failed: %s"%e

def execute_normal_place(book_name,bin_name):
    rospy.wait_for_service('execute_normal_place')
    try:
        place_action = rospy.ServiceProxy('execute_normal_place',PlaceActionMsg)
        response = place_action(book_name,bin_name)
        return response.result,response.next_state,response.reward
    except rospy.ServiceException,e:
        print "Sevice call failed: %s"%e

def execute_normal_moveF():
    rospy.wait_for_service('execute_normal_moveF')
    try:
        normal_moveF = rospy.ServiceProxy("execute_normal_moveF",MoveActionMsg)
        response = normal_moveF()
        return response.success,response.next_state,response.reward
    except rospy.ServiceException,e:
        print "Sevice call failed: %s"%e

def execute_normal_TurnCW():
    rospy.wait_for_service('execute_normal_TurnCW')
    try:
        normal_TurnCW = rospy.ServiceProxy("execute_normal_TurnCW",MoveActionMsg)
        response = normal_TurnCW()
        return response.success,response.next_state,response.reward
    except rospy.ServiceException,e:
        print "Sevice call failed: %s"%e

def execute_normal_TurnCCW():
    rospy.wait_for_service('execute_normal_TurnCCW')
    try:
        normal_TurnCCW = rospy.ServiceProxy("execute_normal_TurnCCW",MoveActionMsg)
        response = normal_TurnCCW()
        return response.success,response.next_state,response.reward
    except rospy.ServiceException,e:
        print "Sevice call failed: %s"%e


def usage():
    return "%s [x y]"%sys.argv[0]
