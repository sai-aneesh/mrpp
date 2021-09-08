#!/usr/bin/env python3


'''
1st variant

simple              +       simple
(expected idleness)  (value function)
AN_MRPP
epsilon-greedy


ROS Params
'''
import rospy
import rospkg
import numpy as np
import networkx as nx
from mrpp_sumo.srv import NextTaskBot, NextTaskBotResponse, AlgoReady, AlgoReadyResponse
from mrpp_sumo.msg import AtNode
import random as rn
import csv
import math

class AN_MRPP:

    def __init__(self, graph, num_bots):
        self.ready = False
        self.graph = graph
        self.stamp = 0.
        self.num_bots = num_bots
        self.vel = 10.                                #MAX VELOCITY SET FOR SIMULATIONS
        self.nodes = list(self.graph.nodes())

        self.current_node = {}
        self.old_node = {}

        self.idle_expect = {}                         #expected idleness: depends on bot, current node
        self.idle_true = {}                           #true idleness :    depends only on current node
        self.value_func = {}                          #value function :  a reward of (expect-true)/expect

        self.value_exp = {}                           #fraction of (exponential of the value_func [selecting this current node])
                                                      #            (sum of exponential of all value func [other nodes it couldve went to from the old node])

        for i in range(self.num_bots):
            self.idle_expect['bot_{}'.format(i)] = {}
            self.value_func['bot_{}'.format(i)] = {}
            self.value_exp['bot_{}'.format(i)] = {}
            self.current_node['bot_{}'.format(i)] = None
            self.old_node['bot_{}'.format(i)] = None

            for n in self.nodes:
                self.idle_expect['bot_{}'.format(i)][n] = 0
                self.value_func['bot_{}'.format(i)][n] = {}
                self.value_exp['bot_{}'.format(i)][n] = {}

                for m in self.graph.successors(n):
                    self.value_func['bot_{}'.format(i)][n][m] = 0
                    self.value_exp['bot_{}'.format(i)][n][m] = 1./(len(list(self.graph.successors(n))))

        for n in self.nodes:
            self.idle_true[n] = 0.

        self.ready = True

    def callback_idle(self, data):
        if self.stamp < data.stamp:
            dev = data.stamp - self.stamp
            self.stamp = data.stamp

            #update true idleness
            for n in self.nodes:
                self.idle_true[n] += dev

            #update expected idleness
            for i in range(self.num_bots):
                for n in self.nodes:
                    self.idle_expect['bot_{}'.format(i)][n] += dev

            for i, n in enumerate(data.robot_id):
                self.current_node[n] = data.node_id[i]
                with open('{}.csv'.format(n), 'a+', newline='') as file:
                    writer = csv.writer(file)

                    if self.old_node[n] is not None :
                        expect = self.idle_expect[n][self.current_node[n]]
                        true = self.idle_true[self.current_node[n]]
                        t = self.stamp

                        #simple value function
                        self.value_func[n][self.old_node[n]][self.current_node[n]] -= (expect - true)/(expect)

                        #calculating the exponential of value function and summing the all possibilties to 1
                        summ = 0
                        for m in self.graph.successors(self.old_node[n]):
                            summ += math.exp(self.value_func[n][self.old_node[n]][m])

                        for m in self.graph.successors(self.old_node[n]):
                            self.value_exp[n][self.old_node[n]][m] = (math.exp(self.value_func[n][self.old_node[n]][m]))/summ

                        #wrting the data into a csv file
                        writer.writerow([t,self.graph[self.old_node[n]][self.current_node[n]]['name'],self.old_node[n],self.current_node[n],self.value_exp[n][self.old_node[n]][self.current_node[n]],self.value_func[n][self.old_node[n]][self.current_node[n]],expect,true,expect-true,len(list(self.graph.successors(self.old_node[n])))])

                self.old_node[n] = data.node_id[i]

            for i, n in enumerate(data.robot_id):
                self.idle_expect[n][data.node_id[i]] = 0.

            for i in enumerate(data.node_id):
                self.idle_true[i] = 0.

    def callback_next_task(self, req):
        node = req.node_done
        t = req.stamp
        bot = req.name

        self.idle_expect[bot][node] = 0.
        self.idle_true[node] = 0.

        neigh = list(self.graph.successors(node))

        rand = rn.random()
        epsilon = 0.1

        #epsilon greedy
        if rand > epsilon:
            idles = []
            for n in neigh:

                #Decision making using product of expected idlness and value_exp
                idles.append(self.idle_expect[bot][n]*self.value_exp[bot][node][n])

            max_id = 0
            if len(neigh) > 1:
                max_ids = list(np.where(idles == np.amax(idles))[0])
                max_id = rn.sample(max_ids, 1)[0]
            next_walk = [node, neigh[max_id]]
            next_departs = [t]
            return NextTaskBotResponse(next_departs, next_walk)

        else :
            next_node = rn.sample(neigh, 1)[0]
            next_walk = [node, next_node]
            next_departs = [t]
            return NextTaskBotResponse(next_departs, next_walk)

    def callback_ready(self, req):
        algo_name = req.algo
        if algo_name == '1_sim_sim' and self.ready:
            return AlgoReadyResponse(True)
        else:
            return AlgoReadyResponse(False)

if __name__ == '__main__':
    rospy.init_node('AN_MRPP', anonymous = True)
    dirname = rospkg.RosPack().get_path('mrpp_sumo')
    done = False
    graph_name = rospy.get_param('/graph')
    num_bots = rospy.get_param ('/init_bots')
    g = nx.read_graphml(dirname + '/graph_ml/' + graph_name + '.graphml')

    s = AN_MRPP(g, num_bots)

    rospy.Subscriber('at_node', AtNode, s.callback_idle)
    rospy.Service('bot_next_task', NextTaskBot, s.callback_next_task)
    rospy.Service('algo_ready', AlgoReady, s.callback_ready)
    while not done:
        done = rospy.get_param('/done')
