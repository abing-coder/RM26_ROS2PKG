import numpy as np

txts = np.loadtxt("time.log")

total = 0.0
for t in txts:
    total += t

print("Average time:", total / len(txts))
