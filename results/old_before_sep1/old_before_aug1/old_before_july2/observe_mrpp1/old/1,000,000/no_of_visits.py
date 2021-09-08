import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('bot_0.csv')
#df = df.drop(['time','edge','true','no.of neighbours','expect'],axis = 1)
df = df.dropna()
df = df.reset_index(drop = True)


f = plt.figure()
f.set_figwidth(4)
f.set_figheight(1)

df['edge'].value_counts().plot(kind='bar')
plt.xlabel('Edge')
plt.ylabel('No. of visits')

plt.show()
