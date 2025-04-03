import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd

# 读取数据
data = pd.read_csv('jam_wait/random-32-32-20.csv', header=None).values

# 方法 1: 使用 Matplotlib 的 imshow
plt.imshow(data, cmap='hot', interpolation='nearest')
plt.colorbar()
plt.title("Heatmap (Matplotlib)")
plt.show()

# 方法 2: 使用 Seaborn 的 heatmap
sns.heatmap(data, annot=True, fmt="d", cmap="YlGnBu")  # `annot=True` 显示数值
plt.title("Heatmap (Seaborn)")
plt.show()
