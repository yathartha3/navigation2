import matplotlib.pyplot as plt
import pickle

with open('tb3_rewards.p', 'rb') as f:
    x = pickle.load(f)

print("File loaded.")
plt.plot(x)
plt.title("Average validation reward for 3 runs")
plt.show()