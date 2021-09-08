import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('bot_1.csv')
df = df.drop(['probability (value_exp)','no.of neighbours','expect'],axis = 1)
df = df.dropna()
df = df.reset_index(drop = True)


f = plt.figure()
f.set_figwidth(4)
f.set_figheight(1)
plt.xlabel('Time (s)')
plt.ylabel('True Idleness')
for elem in df['current node'].unique():
    new_df = df[df['current node'] == elem]
    plt.plot(new_df["time"], new_df["true"])
plt.legend(df['current node'].unique(),loc = 'center left')
plt.show()
