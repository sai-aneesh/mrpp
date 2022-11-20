import os
import rospkg
import pandas as pd
import matplotlib.pyplot as plt

PATH = rospkg.RosPack().get_path('mrpp_sumo') + '/results/2_sim_rl/grid_5_5/1bot'
fileNames = os.listdir(PATH)

fileNames = [file for file in fileNames if '.csv' in file]

for file in fileNames:
    df = pd.read_csv(PATH + '/'+ file)
    f = plt.figure()
    f.set_figwidth(20)
    f.set_figheight(20)
    plt.xlabel('Time (s)')
    plt.ylabel('Error'+ ' ' + '('+file.replace('.csv', '')+')')
    for elem in df['edge'].unique():
        new_df = df[df['edge'] == elem]
        plt.plot(new_df["time"], new_df["expect-true"])
    plt.legend(df['edge'].unique(),loc = 'center left')
    plt.savefig('Error'+ ' ' + '_'+file.replace('.csv', ''), bbox_inches= "tight")

for file in fileNames:
    df = pd.read_csv(PATH + '/'+ file)
    f = plt.figure()
    f.set_figwidth(20)
    f.set_figheight(20)
    plt.xlabel('Time (s)')
    plt.ylabel('Expected Idleness'+ ' ' + '('+file.replace('.csv', '')+')')
    for elem in df['edge'].unique():
        new_df = df[df['edge'] == elem]
        plt.plot(new_df["time"], new_df["expect"])
    plt.legend(df['edge'].unique(),loc = 'center left')
    plt.savefig('Expect_Idle'+ ' ' + '_'+file.replace('.csv', ''),bbox_inches= "tight")


for file in fileNames:
    df = pd.read_csv(PATH + '/'+ file)
    f = plt.figure()
    f.set_figwidth(20)
    f.set_figheight(20)
    df['old node'].value_counts().plot(kind='bar')
    plt.xlabel('Node')
    plt.ylabel('No. of visits'+ ' ' + '('+file.replace('.csv', '')+')')
    plt.savefig('Visits'+ ' ' + '_'+file.replace('.csv', ''),bbox_inches= "tight")

for file in fileNames:
    df = pd.read_csv(PATH + '/'+ file)
    f = plt.figure()
    f.set_figwidth(20)
    f.set_figheight(20)
    plt.xlabel('Time (s)')
    plt.ylabel('True Idleness'+ ' ' + '('+file.replace('.csv', '')+')')
    for elem in df['current node'].unique():
        new_df = df[df['current node'] == elem]
        plt.plot(new_df["time"], new_df["true"])
    plt.legend(df['current node'].unique(),loc = 'center left')
    plt.savefig('True_Idle'+ ' ' + '_'+file.replace('.csv', ''),bbox_inches= "tight")


for file in fileNames:
    df = pd.read_csv(PATH + '/'+ file)
    f = plt.figure()
    f.set_figwidth(20)
    f.set_figheight(20)
    plt.xlabel('Time (s)')
    plt.ylabel('value_exp'+ ' ' + '('+file.replace('.csv', '')+')')
    for elem in df['edge'].unique():
        new_df = df[df['edge'] == elem]
        plt.plot(new_df["time"], new_df["probability (value_exp)"])
    plt.legend(df['edge'].unique(),loc = 'center left')
    plt.savefig('value_exp'+ ' ' + '_'+file.replace('.csv', ''),bbox_inches= "tight")
