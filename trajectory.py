#!/usr/bin/env python

from problem import *
from numpy.random import choice

if __name__ == '__main__':
	s = ""
	rospy.init_node('tester')
	cur_state, book_name, bin_name, action_sequence, success, reward = get_current_state(), '', '', [], 1, 0
	while True:
		s += cur_state + ","
		ch = raw_input("Enter Action: ")
		#print(cur_state, success, reward)
		if ch == 'careful_moveF':
			s += "(careful_moveF),"
			success, cur_state, reward = execute_careful_moveF()
			action_sequence.append('Careful MoveF '+str(reward))
		elif ch == 'normal_moveF':
			s += "(normal_moveF),"
			success, cur_state, reward = execute_normal_moveF()
			action_sequence.append('Normal MoveF '+str(reward))
		elif ch == 'careful_TurnCW':
			s += "(careful_TurnCCW),"
			success, cur_state, reward = execute_careful_TurnCW()
			action_sequence.append('Careful TurnCW '+str(reward))
		elif ch == 'normal_TurnCW':
			s += "(normal_TurnCW),"
			success, cur_state, reward = execute_normal_TurnCW()
			action_sequence.append('Normal TurnCW '+str(reward))
		elif ch == 'careful_TurnCCW':
			s += "(careful_TurnCCW),,"
			success, cur_state, reward = execute_careful_TurnCCW()
			action_sequence.append('Careful TurnCCW '+str(reward))
		elif ch == 'normal_TurnCCW':
			s += ",(normal_TurnCCW),"
			success, cur_state, reward = execute_normal_TurnCCW()
			action_sequence.append('Normal TurnCCW '+str(reward))
		elif ch == 'careful_pick':
			book_name = raw_input("Enter book name: ")
			s += ",(careful_pick, {}),".format(book_name)
			success, cur_state, reward = execute_careful_pick(book_name, cur_state)
			action_sequence.append('Careful Pick '+book_name+' '+str(reward))
		elif ch == 'normal_pick':
			book_name = raw_input("Enter book name: ")
			s += ",(normal_pick, {}),".format(book_name)
			success, cur_state, reward = execute_normal_pick(book_name, cur_state)
			action_sequence.append('Normal Pick '+book_name+' '+str(reward))
		elif ch == 'careful_place':
			book_name = raw_input("Enter book name: ")
			bin_name = raw_input("Enter bin name: ")
			s += ",(careful_place, {}, {}),".format(book_name,bin_name)
			success, cur_state, reward = execute_careful_place(book_name, bin_name, cur_state)
			action_sequence.append('Careful Place '+book_name+' '+bin_name+' '+str(reward))
		elif ch == 'normal_place':
			book_name = raw_input("Enter book name: ")
			bin_name = raw_input("Enter bin name: ")
			s += ",(normal_place, {}, {}),".format(book_name,bin_name)
			success, cur_state, reward = execute_normal_place(book_name, bin_name, cur_state)
			action_sequence.append('Normal Place '+book_name+' '+bin_name+' '+str(reward))
		print cur_state
		s+=str(reward)+","
		if is_terminal_state():
			break

	with open("../trajectories.txt","a") as f:
		f.write(s[:-1]+"\n")