import numpy as np
import os
import matplotlib.pyplot as plt

arr = np.load(os.getcwd()+'/results/evaluations.npy')
rn = np.linspace(0, 10 * len(arr), len(arr))
plt.plot(rn, arr)
plt.title("Average Reward")
plt.xlabel("Episodes")
plt.ylabel("Evaluation Reward")
plt.show()
