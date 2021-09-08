#!/usr/bin/env python3


'''
Observation model based approach
AN_MRPP

ROS Params
'''
import rospy
import rospkg
import numpy as np
import networkx as nx
from mrpp_sumo.srv import NextTaskBot, NextTaskBotResponse
from mrpp_sumo.srv import AlgoReady, AlgoReadyResponse
from mrpp_sumo.msg import AtNode
import random
import csv
import math

class AN_MRPP:

    def __init__(self, graph, num_bots):
        self.ready = False
        self.graph = graph
        self.stamp = 0.
        self.num_bots = num_bots
        self.vel = 10. #MAX VELOCITY SET FOR SIMULATIONS
        self.nodes = list(self.graph.nodes())

        self.idle_expect = {}
        self.idle_true = {}
        self.value_coeff = {}
        self.value_exp = {}
        self.current_node = {}
        self.old_node = {}
        self.array = {}

        for i in range(self.num_bots):
            self.idle_expect['bot_{}'.format(i)] = {}
            self.value_coeff['bot_{}'.format(i)] = {}
            self.value_exp['bot_{}'.format(i)] = {}
            self.array['bot_{}'.format(i)] = {}
            self.current_node['bot_{}'.format(i)] = None
            self.old_node['bot_{}'.format(i)] = None

            for n in self.nodes:
                self.idle_expect['bot_{}'.format(i)][n] = {}
                self.value_coeff['bot_{}'.format(i)][n] = {}
                self.value_exp['bot_{}'.format(i)][n] = {}
                self.array['bot_{}'.format(i)][n] = {}

                for m in self.graph.successors(n):
                    self.idle_expect['bot_{}'.format(i)][n][m] = self.graph[n][m]['length']/self.vel
                    self.value_coeff['bot_{}'.format(i)][n][m] = 1./(len(list(self.graph.successors(n))))
                    self.value_exp['bot_{}'.format(i)][n][m] = 1./(len(list(self.graph.successors(n))))
                    self.array['bot_{}'.format(i)][n][m] = {}

        for n in self.nodes:
            self.idle_true[n] = 0.

        self.ready = True

    def callback_idle(self, data):
        if self.stamp < data.stamp:
            dev = data.stamp - self.stamp
            self.stamp = data.stamp

            for n in self.nodes:
                self.idle_true[n] += dev

            # for i in range(self.num_bots):
            #     for n in self.nodes:
            #         for m in self.graph.successors(n):
            #             self.idle_expect['bot_{}'.format(i)][n][m] += dev
                # with open('bot_{}.csv'.format(i), 'a+', newline='') as file:
                #     writer = csv.writer(file)
                #     writer.writerow(['','','','','','','','',self.stamp,dev,'expect += dev',self.idle_expect])


            for i,n in enumerate(data.robot_id):
                self.current_node[n] = data.node_id[i]
                if self.old_node[n] is not None :
                    self.array[n][self.old_node[n]][self.current_node[n]][self.stamp] = self.idle_true[data.node_id[i]]
                # with open('{}.csv'.format(n), 'a+', newline='') as file:
                #     writer = csv.writer(file)
                #     writer.writerow(['','','','','','','','','','',self.stamp,self.old_node[n],self.current_node[n],'array = true idleness',self.array])

            for i in self.array.keys():
                for j in self.array[i].keys():
                    for m in self.array[i][j].keys():
                        sum = 0
                        leng = 0
                        for k in self.array[i][j][m].keys():
                            # writer.writerow([i,j,m,k,self.array[i][j][m][k]])
                            sum += self.array[i][j][m][k]
                            leng += 1
                        if leng != 0:
                            self.idle_expect[i][j][m] = sum/leng

            for i, n in enumerate(data.robot_id):
                with open('{}.csv'.format(n), 'a+', newline='') as file:
                    writer = csv.writer(file)
                    # writer.writerow(['','','','','','','','','','',self.stamp,self.old_node[n],self.current_node[n],'EXPECTED IDLENESS',self.idle_expect])
                    # writer.writerow(["time","edge","old node","current node","probability (value_exp)","value_coeff","expect","true","no.of neighbours"])

                    if self.old_node[n] is not None :
                        expect = self.idle_expect[n][self.old_node[n]][self.current_node[n]]
                        true = self.idle_true[data.node_id[i]]
                        t = self.stamp
                        lr = 0.6
                        gamma = 0.95
                        if expect > true:
                            reward = -(math.log(expect -true))
                            self.value_coeff[n][self.old_node[n]][self.current_node[n]] += lr*(reward + gamma*max(self.value_coeff[n][self.current_node[n]].values())- self.value_coeff[n][self.old_node[n]][self.current_node[n]])
                        if true > expect :
                            reward = (math.log(true - expect))
                            self.value_coeff[n][self.old_node[n]][self.current_node[n]] += lr*(reward + gamma*max(self.value_coeff[n][self.current_node[n]].values())- self.value_coeff[n][self.old_node[n]][self.current_node[n]])
                        else:
                            reward = 0
                            self.value_coeff[n][self.old_node[n]][self.current_node[n]] += lr*(reward + gamma*max(self.value_coeff[n][self.current_node[n]].values())- self.value_coeff[n][self.old_node[n]][self.current_node[n]])
                        summ = 0
                        for m in self.graph.successors(self.old_node[n]):
                            #self.value_coeff[n][self.old_node[n]][m] += ((expect - true)/expect)/(len(list(self.graph.successors(self.old_node[n]))))
                            summ += math.exp(self.value_coeff[n][self.old_node[n]][m])
                            #writer.writerow([n,t,self.graph[self.old_node[n]][m]['name'],self.value_coeff[n][self.old_node[n]][m],summ,expect,true,len(list(self.graph.successors(self.old_node[n])))])

                        for m in self.graph.successors(self.old_node[n]):
                            self.value_exp[n][self.old_node[n]][m] = (math.exp(self.value_coeff[n][self.old_node[n]][m]))/summ


                        writer.writerow([t,self.graph[self.old_node[n]][self.current_node[n]]['name'],self.old_node[n],self.current_node[n],self.value_exp[n][self.old_node[n]][self.current_node[n]],self.value_coeff[n][self.old_node[n]][self.current_node[n]],self.idle_expect[n][self.old_node[n]][self.current_node[n]],self.idle_true[self.current_node[n]],self.idle_expect[n][self.old_node[n]][self.current_node[n]]-self.idle_true[self.current_node[n]],len(list(self.graph.successors(self.old_node[n])))])

                        # self.idle_expect[n][self.old_node[n]][self.current_node[n]] = 0.

                self.old_node[n] = data.node_id[i]

            for i in enumerate(data.node_id):
                self.idle_true[i] = 0.

    def callback_next_task(self, req):
        node = req.node_done
        t = req.stamp
        bot = req.name
        self.idle_true[node] = 0.

        neigh = list(self.graph.successors(node))
        prob = {}
        prob_sum = {}

        for n in neigh:
            prob[n] = 0
            prob_sum[n] = 0

        for n in neigh:
            prob[n] = self.value_exp[bot][node][n]

        summ = 0
        for n in neigh:
            summ += prob[n]
            prob_sum[n] = summ

        rand = random.random()
        max_id = 0
        for n in neigh:
            if rand > prob_sum[n]:
                max_id += 1
        # with open('{}.csv'.format(bot), 'a+', newline='') as file:
        #     writer = csv.writer(file)
        #     writer.writerow(['','','','','','','','','',node,prob,prob_sum,rand,max_id, neigh[max_id]])

        next_walk = [node, neigh[max_id]]
        next_departs = [t]
        return NextTaskBotResponse(next_departs, next_walk)

    def callback_ready(self, req):
        algo_name = req.algo
        if algo_name == 'observe_mrpp2' and self.ready:
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
