import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('bot_1.csv')
df = df.drop(['true','no.of neighbours','expect'],axis = 1)
df = df.dropna()
df = df.reset_index(drop = True)


f = plt.figure()
f.set_figwidth(4)
f.set_figheight(1)
plt.xlabel('Time (s)')
plt.ylabel('value_exp')
for elem in df['edge'].unique():
    new_df = df[df['edge'] == elem]
    plt.scatter(new_df["time"], new_df["probability (value_exp)"], marker='o')
plt.legend(df['edge'].unique(),loc = 'center left')
# plt.savefig('value_bot1.png', dpi=3000, bbox_inches='tight')
plt.show()
