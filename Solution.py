#! /usr/bin/python
import os
import heapq
import problem
import rospy
from std_msgs.msg import String
import numpy as np
import ast
from enum import Enum
import subprocess
import random
from decimal import Decimal
import glob
import csv
import time
from collections import OrderedDict
import sys


l_bins = {}
l_books = {}

pruned_actions = ['normal_moveF', 'normal_TurnCW', 'normal_TurnCCW',
                  'careful_moveF', 'careful_TurnCW', 'careful_TurnCCW']


class QTable:

    def __init__(self):
        self.table = {}
        self.alpha = 0.3
        self.gamma = 0.9
        self.epsilon = 0.1
        self.reward = 0
        self.old_value = 0
        self.next_max = 0
        self.r_gamma = 1
        self.count_actions = 0


class QLearning:
    def __init__(self, track = None):
        self.init_state = problem.get_current_state()
        self.track = track
        self.actions = [action.replace('(','').replace(')', '') for action in problem.get_all_actions()]
        self.actions.sort()
        self.index = {x: i for i, x in enumerate(self.actions)}
        self.total_actions = len(self.actions)
        self.publisher = rospy.Publisher("/results", String, queue_size=10)
        rospy.init_node("solution")

    def execute_track_t1(self):
        q = QTable()
        q.table = OrderedDict()
        print("Executing trajectory.")
        try:
            for curr_state, action_reward in self.track.items():
                action, reward = action_reward
                action = action.replace('(', '').replace(')', '').replace(',', '')
                reward = float(reward)

                if curr_state not in q.table:
                    q.table[curr_state] = [0] * self.total_actions

                # Q value for this state and action Q(s,a)
                q.old_value = float(q.table[curr_state][self.index[action]])

                # Execute selected action
                status, new_state, _ = take_step(action)

                # print status, new_state, reward
                q.count_actions += 1

                if reward > 0:
                    print(action, reward)

                # Get max Q from next state
                if new_state in q.table:
                    q.next_max = float(max(q.table[new_state]))

                new_value = (1 - q.alpha) * q.old_value + q.alpha * (reward + q.gamma * q.next_max)
                q.table[curr_state][self.index[action]] = new_value

                if q.count_actions == 1:
                    q.reward += reward
                else:
                    q.r_gamma *= 0.9
                    q.reward += (q.r_gamma * reward)

            print("Reached terminal state")
        finally:
            message = ''
            for k,v in q.table.items():
                action_value = {}
                for i,item in enumerate(v):
                    action_value[self.actions[i]] = item

                q.table[k] = action_value

            for k,v in q.table.items():
                message+=k+' : '
                message+=str(v)
                message+='-'

            message = message[:-1]
            print 'publish message'
            self.publisher.publish(message)
            problem.reset_world()
            print('World Reset')
            q.reward = 0
            q.r_gamma = 1

    def train_t2(self, max_episodes):
        q = QTable()
        curr_state = self.init_state
        curr_state = sort_state(curr_state)

        print("Searching the purpose of life.")
        for epoch in range(0, max_episodes*2):
            try:
                if epoch % 2 == 0:
                    print ("Executing actions along with Exlporation")
                else:
                    print ("Executing actions with Learned Policy only")

                while not problem.is_terminal_state():
                    if curr_state not in q.table:
                        q.table[curr_state] = [0]*self.total_actions

                    # evaluate policy one time and other time try to execute epsilon greedy
                    if random.uniform(0, 1) < q.epsilon and (epoch % 2) == 0:
                        # print("Exploring..")
                        action = self.actions[random.randint(0, 9)]
                    else:
                        m = max(q.table[curr_state])
                        indices = [j for j, k in enumerate(q.table[curr_state]) if k == m]
                        # This keeps bot from always picking the 1st best
                        # action and randomly chooses from all best choices
                        idx = random.choice(indices)
                        action = self.actions[idx]

                    # Q value for this state and action Q(s,a)
                    q.old_value = float(q.table[curr_state][self.index[action]])

                    # Execute selected action
                    status, new_state, reward = take_step(action)
                    sort_state(new_state)
                    q.count_actions += 1

                    if reward > 0:
                        print(action, reward)

                    # Get max Q from next state
                    if new_state in q.table:
                        q.next_max = float(max(q.table[new_state]))

                    new_value = (1 - q.alpha) * q.old_value + q.alpha * (reward + q.gamma * q.next_max)
                    q.table[curr_state][self.index[action]] = new_value

                    if q.count_actions == 1:
                        q.reward += reward
                    else:
                        q.r_gamma *= 0.9
                        q.reward += (q.r_gamma*reward)

                    message = "("+curr_state+","+action+","+new_state+","+str(reward)+")"+" : "+ str(new_value)
                    self.publisher.publish(message)

                    # print message
                    curr_state = new_state
                print ("Total Reward: "+ str(q.reward))
                print("Reached terminal state")
            finally:
                problem.reset_world()
                print('World Reset')
                q.reward = 0
                q.r_gamma = 1

    def train_t3(self, max_episodes):
        q = QTable()
        get_load_locations(self.init_state)
        curr_state = self.init_state
        curr_state = sort_state(curr_state)
        print("Searching the purpose of life.")
        for epoch in range(0, 2*max_episodes):
            try:
                if epoch % 2 == 0:
                    print("Executing actions along with Exlporation")
                else:
                    print("Executing actions with Learned Policy only")

                while not problem.is_terminal_state():
                    if curr_state not in q.table:
                        q.table[curr_state] = [0]*self.total_actions

                    if random.uniform(0, 1) < q.epsilon and epoch%2==0:
                        # print("Exploring..")
                        action = self.actions[random.randint(0, 9)]
                    else:
                        m = max(q.table[curr_state])
                        indices = [j for j, k in enumerate(q.table[curr_state]) if k == m]
                        # This keeps bot from always picking the 1st best
                        # action and randomly chooses from all best choices
                        idx = random.choice(indices)
                        action = self.actions[idx]

                    if ('place' in action and not is_bin_location(curr_state)) \
                            or ('pick' in action and not is_book_location(curr_state)):
                        # Choose best action from pruned actions
                        # a_rewards = q.table[curr_state][0:3]+q.table[curr_state][43:46]
                        # idx = a_rewards.index(max(a_rewards))
                        # action = pruned_actions[idx]
                        action = random.choice(pruned_actions)

                    # Q value for this state and action Q(s,a)
                    q.old_value = float(q.table[curr_state][self.index[action]])

                    # Execute selected action
                    status, new_state, reward = take_step(action)

                    if reward > 0:
                        print(action, reward, status)

                    sort_state(new_state)
                    # Get max Q from next state
                    if new_state in q.table:
                        q.next_max = float(max(q.table[new_state]))

                    new_value = (1 - q.alpha) * q.old_value + q.alpha * (reward + q.gamma * q.next_max)
                    q.table[curr_state][self.index[action]] = new_value

                    if q.count_actions == 1:
                        q.reward += reward
                    else:
                        q.r_gamma *= 0.9
                        q.reward += (q.r_gamma*reward)

                    message = "("+curr_state+","+action+","+new_state+","+str(reward)+")"+" : "+ str(new_value)
                    self.publisher.publish(message)
                    # print message

                    curr_state = new_state
                print ("Total Reward: " + str(q.reward))
                print("Reached terminal state")
            finally:
                problem.reset_world()
                print('World Reset')
                q.reward = 0
                q.r_gamma = 1


def sort_state(curr_state):
    curr_state = curr_state.split(';')
    curr_state.sort()
    curr_state = ';'.join(curr_state)
    return curr_state


def take_step(action):
    if 'pick' in action:
        # is_book = is_book_location(curr_state)
        action, book = action.split(' ')
        status, curr_state, reward = exec_action(action=action, book=book)
    elif 'place' in action:
        # is_bin = is_bin_location(curr_state)
        action, book, bin = action.split(' ')
        status, curr_state, reward = exec_action(action, book, bin)
    else:
        status, curr_state, reward = exec_action(action=action)

    return status, curr_state, reward


def exec_action(action, book='', bin=''):

        methods = {'normal_moveF': problem.execute_normal_moveF, 'normal_TurnCW': problem.execute_normal_TurnCW,
                   'normal_TurnCCW': problem.execute_normal_TurnCCW, 'normal_place': problem.execute_normal_place,
                   'normal_pick': problem.execute_normal_pick, 'careful_moveF': problem.execute_careful_moveF,
                   'careful_TurnCW': problem.execute_careful_TurnCW, 'careful_TurnCCW': problem.execute_careful_TurnCCW,
                   'careful_place': problem.execute_careful_place, 'careful_pick': problem.execute_careful_pick}

        f = methods[action]

        if book and bin:
            return f(book, bin)
        elif book:
            return f(book)
        else:
            return f()


def get_load_locations(i_state):
    items = i_state.split(';')

    for item in items:
        item = item.replace('(','').replace(')', '')
        if 'trolly' in item:
            itm = item.split(',')

            tx1 = str(Decimal(itm[1])-Decimal('0.5'))
            ty1 = str(Decimal(itm[2]))

            tx2 = str(Decimal(itm[1]))
            ty2 = str(Decimal(itm[2]) - Decimal('0.5'))

            tx3 = str(Decimal(itm[1]) + Decimal('0.5'))
            ty3 = str(Decimal(itm[2]) - Decimal('0.5'))

            l_bins[(itm[1], itm[2])] = itm[0]
            l_bins[(tx1, ty1)] = itm[0]
            l_bins[(tx2, ty2)] = itm[0]
            l_bins[(tx3, ty3)] = itm[0]

            print(itm[0], itm[1], itm[2])
            print(itm[0], tx1, ty1)
            print(itm[0], tx2, ty2)
            print(itm[0], tx3, ty3)

        elif 'book' in item:
            itm = item.split(',')
            t1 = Decimal(itm[1])
            t2 = Decimal(itm[2])

            tmpx = itm[1]
            tmpy = itm[2]

            m = Decimal('0.5')
            r1 = t1 % m
            if r1 > Decimal('0'):
                itm[1] = float(t1 - r1)
                tmpx = float(t1 + r1)
                itm[1] = str(itm[1])
                tmpx = str(tmpx)

            r2 = t2 % m
            if r2 > Decimal('0'):
                itm[2] = float(t2 - r2)
                tmpy = float(t2 + r2)
                itm[2] = str(itm[2])
                tmpy = str(tmpy)

            print(itm[0], itm[1], itm[2])
            print(itm[0], tmpx, tmpy)

            l_books[(itm[1], itm[2])] = itm[0]
            l_books[(tmpx, tmpy)] = itm[0]


def is_bin_location(curr_state):
    res = get_bot_location(curr_state) in l_bins
    return res


def is_book_location(curr_state):
    res = get_bot_location(curr_state) in l_books
    return res


def get_bot_location(curr_state):
    items = curr_state.split(';')
    for item in items:
        item = item.replace('(', '').replace(')', '')
        if 'turtlebot3' in item:
            itm = item.split(',')
            return (itm[1].replace(' ', ''), itm[2].replace(' ', ''))


def get_tracks():
    cwd = os.getcwd()
    path = os.getcwd()+'/src/reinforcement-master'
    os.chdir(path)
    with open('trajectories.txt') as file:
        content = file.readlines()

    content = [x.strip() for x in content]

    commands = []
    str_tracks = []
    tracks = []

    for line in content:
        if "-sub" in line:
            commands.append(line)
        else:
            str_tracks.append(line)

    str_tracks = list(filter(None, str_tracks))

    sar = [OrderedDict()]*3

    for j,track in enumerate(str_tracks):
        s_a_r = track.split(",,")
        i = 0
        while i< len(s_a_r):
            sar[j][s_a_r[i+0]] = (s_a_r[i+1],s_a_r[i+2])
            i += 3

    os.chdir(cwd)
    return sar,commands


def open_new_server(params):
    # Make sure you are in catkin_ws
    # os.chdir(path)
    print("cwd: ", os.getcwd())
    p1 = subprocess.Popen(["catkin_make"], stdout=subprocess.PIPE)
    # (output, err) = p1.communicate()
    # print output, err
    # Make sure setup.bash has executable permission on system
    p2 = subprocess.Popen(["./devel/setup.bash"], stdout=subprocess.PIPE)
    # (output, err) = p2.communicate()
    # print output, err
    params = (params.split(' '))
    p3 = subprocess.Popen(["rosrun", "reinforcement", "server.py"]+params)
    # (output, err) = p3.communicate()
    # print output, err

    print("Environement setup done!")
    return p1, p2, p3


if __name__ == '__main__':

    argv = list(sys.argv)
    if argv[1] == 't1':
        sar, params = get_tracks()
        for i,item in enumerate(params):
            a, b, c = open_new_server(item)
            et = QLearning(sar[i])
            et.execute_track_t1()
            a.kill()
            b.kill()
            c.kill()
            time.sleep(5)

    elif argv[1] == 't2':
        # auto environment setup is not done for t2 and t3
        qlearn = QLearning()
        qlearn.train_t2(int(argv[2]))

    elif argv[1] == 't3':
        # auto environment setup is not done for t2 and t3
        qlearn = QLearning()
        qlearn.train_t2(int(argv[2]))

