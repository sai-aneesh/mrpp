#!/usr/bin/env python3


'''
CR_MRPP

ROS Params
'''
import rospy
import rospkg
import numpy as np
import networkx as nx
from mrpp_sumo.srv import NextTaskBot, NextTaskBotResponse
from mrpp_sumo.srv import AlgoReady, AlgoReadyResponse
from mrpp_sumo.msg import AtNode
import random as rn

class CR_MRPP:

    def __init__(self, graph, num_bots):
        self.ready = False
        self.graph = graph
        self.stamp = 0.
        self.num_bots = num_bots
        self.nodes = list(self.graph.nodes())

        self.idle_expect = {}
        self.idle_true = {}
        self.reward_coeff = {}
        self.current_node = {}
        self.old_node = {}

        for i in range(self.num_bots):
            self.idle_expect['bot_{}'.format(i)] = {}
            self.reward_coeff['bot_{}'.format(i)] = {}
            self.current_node['bot_{}'.format(i)] = 0.
            self.old_node['bot_{}'.format(i)] = 0.

            for n in self.nodes:
                self.idle_expect['bot_{}'.format(i)][n] = 0.
                self.reward_coeff['bot_{}'.format(i)][n] = {}

                for m in self.graph.successors(n):
                    self.reward_coeff['bot_{}'.format(i)][n][m] = 1./(len(list(self.graph.successors(n))))

        for n in self.nodes:
            self.idle_true[n] = 0.

        self.ready = True

    def callback_idle(self, data):
        if self.stamp < data.stamp:
            dev = data.stamp - self.stamp
            self.stamp = data.stamp

            for n in range(self.num_bots):
                for i in self.nodes:
                    self.idle_expect['bot_{}'.format(n)][i] += dev

            for i in self.nodes:
                self.idle_true[i] += dev

            for i, n in enumerate(data.robot_id):

                self.current_node[n] = data.node_id[i]
                expect = self.idle_expect[n][data.node_id[i]]
                true = self.idle_true[data.node_id[i]]
                t = self.stamp
                if self.old_node[n] !=0. :

                    if expect > true:
                        self.reward_coeff[n][self.old_node[n]][self.current_node[n]] -= ((0.9)**t)*((expect - true)/expect)
                        for m in self.graph.successors(self.old_node[n]):
                            if m !=self.current_node[n]:
                                self.reward_coeff[n][self.old_node[n]][m] += ((0.9)**t)*((expect - true)/expect)/(len(list(self.graph.successors(self.old_node[n])))-1)

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
        idles = []
        for n in neigh:
            idles.append((self.idle_expect[bot][n])*(self.reward_coeff[bot][node][n]))

        max_id = 0
        if len(neigh) > 1:
            max_ids = list(np.where(idles == np.amax(idles))[0])
            max_id = rn.sample(max_ids, 1)[0]

        next_walk = [node, neigh[max_id]]
        next_departs = [t]
        return NextTaskBotResponse(next_departs, next_walk)

    def callback_ready(self, req):
        algo_name = req.algo
        if algo_name == 'aneesh_mrpp3' and self.ready:
            return AlgoReadyResponse(True)
        else:
            return AlgoReadyResponse(False)

if __name__ == '__main__':
    rospy.init_node('CR_MRPP', anonymous = True)
    dirname = rospkg.RosPack().get_path('mrpp_sumo')
    done = False
    graph_name = rospy.get_param('/graph')
    num_bots = rospy.get_param ('/init_bots')
    g = nx.read_graphml(dirname + '/graph_ml/' + graph_name + '.graphml')

    s = CR_MRPP(g, num_bots)

    rospy.Subscriber('at_node', AtNode, s.callback_idle)
    rospy.Service('bot_next_task', NextTaskBot, s.callback_next_task)
    rospy.Service('algo_ready', AlgoReady, s.callback_ready)
    while not done:
        done = rospy.get_param('/done')
