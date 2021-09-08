import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('bot_1.csv')
df = df.drop(['true','no.of neighbours','expect'],axis = 1)
df = df.dropna()
df = df.reset_index(drop = True)


f = plt.figure()
f.set_figwidth(4)
f.set_figheight(1)
for elem in df['edge'].unique():
    new_df = df[df['edge'] == elem]
    plt.plot(new_df["time"], new_df["probability (value_coeff)"])
plt.legend(df['edge'].unique(),loc = 'center left')
plt.show()
