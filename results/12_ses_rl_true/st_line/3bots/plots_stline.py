import os
import rospkg
import pandas as pd
import matplotlib.pyplot as plt
import networkx as nx

g = nx.read_graphml(rospkg.RosPack().get_path('mrpp_sumo') + '/graph_ml/st_line.graphml')

# PATH = rospkg.RosPack().get_path('mrpp_sumo') + '/results/12_ses_rl_true/grid_5_5/3bots'
# Get the current working directory
PATH = os.getcwd()
fileNames = os.listdir(PATH)

fileNames = [file for file in fileNames if '.csv' in file]

plt.figure(figsize=(22, 12))

for i,file in enumerate(sorted(fileNames)):
    df = pd.read_csv(PATH + '/'+ file)

    for elem in sorted(df['current node'].unique()):
        new_df = df[df['current node'] == elem]
        ax = plt.subplot(1, len(fileNames) , i + 1)
        ax.scatter(new_df["time"], new_df["expect-true"])

        plt.xlabel('Time (s)')
        plt.ylabel('Error'+ ' ' + '('+file.replace('.csv', '')+')')
    plt.legend(sorted(df['current node'].unique()),loc = 'upper right')
    plt.savefig('Error', bbox_inches= "tight")


for n,file in enumerate(sorted(fileNames)):                                          #for all nodes and 3 bots
    ax = plt.subplot(1, len(fileNames) , n + 1)
    df = pd.read_csv(PATH + '/'+ file)
    plt.xlabel('Time (s)')
    plt.ylabel('Expected Idleness'+ ' ' + '('+file.replace('.csv', '')+')')

    new_df = df[(df['old node'] == 'gneJ4') | (df['old node'] == 'gneJ6') | (df['old node'] == 'gneJ10')]

    for elem in sorted(new_df['edge'].unique()):
        new_df2 = new_df[new_df['edge'] == elem]

        ax.scatter(new_df2["time"], new_df2["expect"])

        ax.legend(sorted(new_df['edge'].unique()),loc = 'upper right')
ax.figure.savefig('Expect_Idle_all',bbox_inches= "tight")


for i,file in enumerate(sorted(fileNames)):
    df = pd.read_csv(PATH + '/'+ file)
    new_df = df[(df['old node'] == 'gneJ4') | (df['old node'] == 'gneJ6') | (df['old node'] == 'gneJ10')]

    for j,elem in enumerate(sorted(new_df['old node'].unique())):
        new_df2 = new_df[new_df['old node'] == elem]


        for elem in sorted(new_df2['edge'].unique()):
            new_df3 = new_df[new_df['edge'] == elem]
            ax3 = plt.subplot(len(new_df['old node'].unique()), len(fileNames) , 3*i+j+1)
            ax3.scatter(new_df3["time"], new_df3["probability (value_exp)"])
            ax3.legend(sorted(new_df2['edge'].unique()),loc = 'upper right')


        plt.xlabel('Time (s)')
        plt.ylabel('value_exp'+ ' ' + '('+file.replace('.csv', '')+')')
ax3.figure.savefig('value_exp',bbox_inches= "tight")

for i,file in enumerate(sorted(fileNames)):
    df = pd.read_csv(PATH + '/'+ file)

    nodes = sorted(list((g.nodes())))
    d= df['old node'].value_counts()
    d1 = d.to_dict()
    freq = []
    for n in nodes:
        if n in d1.keys():
            freq.append(d1[n])
        else:
            freq.append(0)
    ax2 = plt.subplot(1, len(fileNames) , i+1)
    ax2.bar(nodes,freq)
    plt.xlabel('Node')
    plt.ylabel('No. of visits'+ ' ' + '('+file.replace('.csv', '')+')')

ax2.figure.savefig('Visits',bbox_inches= "tight")



for i,file in enumerate(sorted(fileNames)):                                                 #for few nodes and 3 bots
    df = pd.read_csv(PATH + '/'+ file)
    new_df = df[(df['old node'] == 'gneJ4') | (df['old node'] == 'gneJ6') | (df['old node'] == 'gneJ10')]

    for j,elem in enumerate(sorted(new_df['old node'].unique())):
        new_df2 = new_df[new_df['old node'] == elem]


        for elem in sorted(new_df2['edge'].unique()):
            new_df3 = new_df[new_df['edge'] == elem]
            ax = plt.subplot(len(new_df['old node'].unique()), len(fileNames) , 3*i+j+1)
            ax.scatter(new_df3["time"], new_df3["expect"])
            ax.legend(sorted(new_df2['edge'].unique()),loc = 'upper right')


        plt.xlabel('Time (s)')
        plt.ylabel('Expected Idleness'+ ' ' + '('+file.replace('.csv', '')+')')
ax.figure.savefig('Expect_Idle',bbox_inches= "tight")
