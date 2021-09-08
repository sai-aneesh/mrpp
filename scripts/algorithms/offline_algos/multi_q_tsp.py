#!/usr/bin/env python3


'''
Parameters:
1. Graph
2. Number of episodes
3. Number of threads
4. Max number of bots
5. algo_name == 'ant_q_tsp'
6. random_string

Outputs:
1. 'random_string'_vis.html - Visualization of optimal cycle across episodes
2. 'random_string'_seq.in - sequence of node visits
'''

import networkx as nx
import rospkg
import sys
import random as rn
import numpy as np
import plotly.graph_objects as go
import plotly.io as pio
pio.kaleido.scope.mathjax = None
import rosparam
import matplotlib.animation as anplot
import matplotlib.pyplot as plt

class Multi_Q:

    def __init__(self, graph, time_period, num_threads, q_0 = 0.8, alpha = 0.1, gamma = 0.3, delta = 3, beta = 1, W = 10000):
        self.graph = graph.copy()
        self.edges_og = list(graph.edges())
        self.nodes = list(graph.nodes())
        self.paths = {}
        self.num_threads = min(num_threads, len(self.nodes))
        for i in self.nodes:
            for j in self.nodes:
                if i != j and not (i, j) in self.edges_og:
                    self.graph.add_edge(i, j)
                    self.graph[i][j]['name'] = '{}to{}'.format(i, j)
                    self.graph[i][j]['length'] = nx.dijkstra_path_length(graph, i, j, 'length')
                    self.paths[(i, j)] = nx.dijkstra_path(graph, i, j, 'length')
                elif i != j:
                    self.paths[(i, j)] = [i, j]

        self.edges = list(self.graph.edges())
        self.lengths = [float(self.graph[e[0]][e[1]]['length']) for e in self.edges]
        avg_len = sum(self.lengths)/len(self.edges)
        max_len = max(self.lengths)
        for e in self.edges:
            self.graph[e[0]][e[1]]['h_value'] = max_len/float(self.graph[e[0]][e[1]]['length'])
            self.graph[e[0]][e[1]]['q_value'] = max_len/(avg_len)
        self.q = q_0
        self.q_rem = 1 - q_0
        self.alpha = alpha
        self.beta = beta
        self.gamma = gamma
        self.delta = delta
        self.W = W
        self.best_walk = []
        self.best_len = np.inf
        self.best_walk_directions = []
        self.best_walk_edges = []
        self.time_period = time_period
        self.init_plot()    
        # self.num_threads = self.nodes
        # self.fig.show()

    def init_plot(self):

        d = 20 #radius of nodes

        # edge_x = []
        # edge_y = []
        self.edge_traces = [] 
        self.edges_plot = []
        for edge in self.edges_og:
            if not (edge[1], edge[0]) in self.edges_plot:
                self.edges_plot.append(edge)
                x0, y0 = self.graph.nodes[edge[0]]['x'], self.graph.nodes[edge[0]]['y']
                x1, y1 = self.graph.nodes[edge[1]]['x'], self.graph.nodes[edge[1]]['y']
                self.edge_traces.append(go.Scatter(x=[x0, x1, None], y=[y0, y1, None], line=dict(width=0.5, color = 'blue'), opacity = 1, hoverinfo='none', mode='lines'))
        
        node_x = []
        node_y = []

        for node in list(self.nodes):
            x, y = self.graph.nodes[node]['x'], self.graph.nodes[node]['y']
            node_x.append(x)
            node_y.append(y)

        self.node_trace = go.Scatter(
            x=node_x, y=node_y,
            mode='markers',
            # hoverinfo='text',
            marker=dict(
                showscale=False,
                reversescale=False,
                color='white',
                size=d,
                line_width=0))

        self.frames = [go.Frame(data=[self.node_trace] + self.edge_traces.copy())]

        # return fig
    
    def run(self):
        actions = self.edges.copy()
        in_degree = [False for n in self.nodes]
        out_degree = [False for n in self.nodes]
        edges_in_run = {}

        while not (any(in_degree) and any(out_degree)):
            values = [(self.graph[act[0]][act[1]]['q_value'] ** self.delta) * (self.graph[act[0]][act[1]]['h_value'] ** self.beta) for act in actions]
            # values = [self.graph[cur_node][node]['q_value'] for node in actions]
            tot_val = sum(values)
            values = [v/tot_val for v in values]
            if rn.random() <= self.q:
                act = actions[np.argmax(values)]

            else:
                #random-proportional
                r_val = rn.random()
                cum_sum = 0.
                i = -1
                while r_val < cum_sum:
                    i += 1
                    cum_sum += values[i]
                act = actions[i]
                
            edges_in_run[act[0]] = act
            actions.remove(act)

            out_degree[self.nodes.index(act[0])] = True
            in_degree[self.nodes.index(act[1])] = True

            for n in self.graph.successors(act[0]):
                if (act[0], n) in actions:
                    actions.remove((act[0], n))

            for n in self.graph.predecessors(act[1]):
                if (n, act[1]) in actions:
                    actions.remove((n, act[1]))

            if actions:
                next_val = max([self.graph[nex[0]][nex[1]]['q_value'] for nex in actions])
                self.graph[act[0]][act[1]]['q_value'] *= (1 - self.alpha)
                self.graph[act[0]][act[1]]['q_value'] += self.alpha * self.gamma * next_val 


        # le = 0
        # for i in range(len(self.nodes)):
        #     # print(self.graph[walk_so_far[i]][walk_so_far[i + 1]]['length'])
        #     le += self.graph[walk_so_far[i]][walk_so_far[i + 1]]['length']

        return (edges_in_run)
    

    def cost_of_walks(self, edg):
        remain_edges = edg.copy()
        edge_sets = []
        edge_lens = []
        cur_edges = []
        cur_len = 0
        nodes_rem = self.nodes.copy()
        while nodes_rem:
            if len(cur_edges) == 0:
                start_node = nodes_rem.pop(0)
                cur_edges.append(remain_edges[start_node])
                cur_node = cur_edges[-1][1]
                cur_len += self.graph[cur_edges[-1][0]][cur_edges[-1][1]]['length'] 
            else:
                nodes_rem.remove(cur_node)
                cur_edges.append(remain_edges[cur_node])
                cur_node = cur_edges[-1][-1]
                cur_len += self.graph[cur_edges[-1][0]][cur_edges[-1][1]]['length']
                if cur_node == start_node:
                    edge_sets.append(cur_edges)
                    edge_lens.append(cur_len)
                    cur_edges = []
                    cur_len = 0
        return edge_sets, edge_lens

    def episode(self, ep_count):
        walks = []
        lens = []
        costs = []
        if ep_count > 100 and self.q < 0.99:
            self.q += (self.q_rem/2) ** 2
            self.q_rem = 1 - self.q 

        for i in range(self.num_threads):
            # to be run as separate threads
            edges_run = self.run()
            # print(i, w, l)
            es, ls = self.cost_of_walks(edges_run)
            cost = 0
            for c in ls:
                cost += (c//self.time_period + 1)
            walks.append(es)
            lens.append(ls)
            costs.append(cost)



        iter_best_walk = walks[np.argmin(costs)]
        iter_best = min(costs)
        iter_best_edges = []
        for w in range(len(iter_best_walk) - 1):
            for a in w:
                path = self.paths[(a[0], a[1])]
                for j in range(len(path) - 1):
                    iter_best_edges.append((path[j], path[j + 1]))


        if iter_best < self.best_len:
            self.best_len = iter_best
            self.best_walk = iter_best_walk
            self.best_walk_directions = iter_best_edges
            self.best_walk_edges = []
            for i in range(len(self.best_walk) - 1):
                path = self.paths[(self.best_walk[i], self.best_walk[i + 1])]
                for j in range(len(path) - 1):
                    if (path[j], path[j + 1]) in self.edges_plot:
                        self.best_walk_edges.append((path[j], path[j + 1]))
                    else:
                        self.best_walk_edges.append((path[j + 1], path[j]))
                    if (path[j], path[j + 1]) not in self.best_walk_directions:
                        self.best_walk_directions.append((path[j], path[j + 1]))

        del_aq = self.W/iter_best
        edge_counts = {tuple(e): 0 for e in self.edges_plot}
        for w in walks:
            for i in range(len(w) - 1):
                self.graph[w[i]][w[i + 1]]['q_value'] =  self.graph[w[i]][w[i + 1]]['q_value'] + self.alpha * del_aq
                path = self.paths[(w[i], w[i + 1])]
                for j in range(len(path) - 1):
                    if (path[j], path[j + 1]) in self.edges_plot:
                        edge_counts[(path[j], path[j + 1])] += 1
                    else:
                        edge_counts[(path[j + 1], path[j])] += 1



        edge_traces = self.edge_traces.copy()

        # #test code
        # for i in range(len(self.edges_plot)):
        #     edge_traces[i].line.color = 'blue'
        #     edge_traces[i].opacity = 1
        #     if (ep_count) % 2 == 0:
        #         edge_traces[i].line.color = 'orange'

        for i in range(len(self.edges_plot)):
            edge_traces[i].line.width = 1
            edge_traces[i].line.color = 'white'
            edge_traces[i].opacity = min(edge_counts[tuple(self.edges_plot[i])]/len(self.nodes), 1)
            if tuple(self.edges_plot[i]) in self.best_walk_edges:
                edge_traces[i].line.color = 'orange'
                edge_traces[i].opacity = 1
                edge_traces[i].line.width = 4
        # if ep_count % 10 == 1:
        frame = go.Frame(data=[self.node_trace] + edge_traces, 
                        layout= go.Layout(title = 'Episode {}, Best Length = {}m'.format(ep_count, self.best_len), title_x = 0.4))
        
        self.frames.append(frame)   
        return (iter_best_walk, iter_best)


if __name__ == '__main__':
    dirname = rospkg.RosPack().get_path('mrpp_sumo')
    sim_file = sys.argv[1]

    params = rosparam.load_file(sim_file)[0][0]
    graph_name = params['graph']
    num_episodes = params['num_episodes']
    num_threads = params['num_threads']
    algo = params['algo_name']
    sim_name = params['random_string']
    time_period = params['time_period']

    if algo != 'multi_q_tsp':
        print('Not this algorithm')
        exit

    g = nx.read_graphml(dirname + '/graph_ml/' + graph_name + '.graphml')
    
    
    # outputs = ant_q_tsp(g)

    test = Multi_Q(g, time_period, num_threads)
    # print(test.run('0'))
    for i in range(num_episodes):
        w, l = test.episode(i + 1)
        # print(i, l)

    fig = go.Figure(data=[test.node_trace] + test.edge_traces,
            layout=go.Layout(
            title='Graph \'{}\''.format(graph_name),
            # titlefont_size=16,
            showlegend=False,
            hovermode='closest',
            margin=dict(b=20,l=5,r=5,t=40),
            xaxis=dict(showgrid=False, zeroline=False, showticklabels=False),
            yaxis=dict(showgrid=False, zeroline=False, showticklabels=False),
            plot_bgcolor='black',
            updatemenus=[dict(type="buttons", buttons=[dict(label="Play", method="animate", args=[None, {"frame": {"duration": 5, "redraw": True}}])])]),
            frames = test.frames
            )
    fig.update_yaxes(scaleanchor = 'x', scaleratio = 1)
    fig.write_html('{}/{}_tsp.html'.format(dirname + '/outputs', sim_name))

    # Writer = anplot.writers['ffmpeg']
    # writer = Writer(fps=15, metadata=dict(artist='S Deepak Mallya'), bitrate=1800)

    # ani = anplot.ArtistAnimation(plt.figure(), test.frames, interval=5, blit=True, repeat_delay=1000)
    # ani.save(dirname + '/outputs/{}_video.mp4'.format(sim_name), writer = writer)

    

    best_walk = test.best_walk_directions
    with open(dirname + '/outputs/{}_visit_seq.in'.format(sim_name), 'w') as f:
        f.write(str(test.best_len) + '\n')
        f.write('\n')
        for n in best_walk:
            f.write(str(n[0])+ '\n')
        