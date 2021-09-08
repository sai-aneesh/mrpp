#!/usr/bin/env python3

'''
Conscientious Reactive
'''

import rospkg
import numpy as np
import rospy
import networkx as nx
from mrpp_sumo.srv import NextTaskBot, NextTaskBotResponse
from mrpp_sumo.msg import AtNode
import random as rn

class CR:

    def __init__(self, g, num_bots):
        self.graph = g
        self.stamp = 0.
        self.num_bots = num_bots

        self.nodes = list(self.graph.nodes())
        self.idleness = {}
        for i in range(num_bots):
            self.idleness['bot_{}'.format(i)] = {}
            for n in self.nodes:
                self.idleness['bot_{}'.format(i)][n] = 0.


    def callback_idle(self, data):
        if self.stamp < data.stamp:
            dev = data.stamp - self.stamp
            self.stamp = data.stamp
            for n in range(num_bots):
                for i in self.nodes:
                    self.idleness['bot_{}'.format(n)][i] += dev

            for i, n in enumerate(data.robot_id):
                self.idleness[n][data.node_id[i]] = 0.


    def callback_next_task(self, req):
        node = req.node_done
        t = req.stamp
        bot = req.name

        self.idleness[bot][node] = 0.

        neigh = list(self.graph.successors(node))
        idles = []
        for n in neigh:
            idles.append(self.idleness[bot][n])

        max_id = 0
        if len(neigh) > 1:
            max_ids = list(np.where(idles == np.amax(idles))[0])
            max_id = rn.sample(max_ids, 1)[0]
        next_walk = [node, neigh[max_id]]
        next_departs = [t]
        return NextTaskBotResponse(next_departs, next_walk)


if __name__ == '__main__':
    rospy.init_node('cr', anonymous= True)
    dirname = rospkg.RosPack().get_path('mrpp_sumo')
    done = False
    graph_name = rospy.get_param('/graph')
    num_bots = int(rospy.get_param('/init_bots'))
    g = nx.read_graphml(dirname + '/graph_ml/' + graph_name + '.graphml')

    s = CR(g, num_bots)

    rospy.Subscriber('at_node', AtNode, s.callback_idle)
    rospy.Service('bot_next_task', NextTaskBot, s.callback_next_task)

    done = False
    while not done:
        done = rospy.get_param('/done')
