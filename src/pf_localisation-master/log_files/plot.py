import numpy as np
import matplotlib.pyplot as plt

weighted = np.load("weighted.npy")
kmeans = np.load("kmeans.npy")

shape = min(weighted.shape[0], kmeans.shape[0])
max_val_pos = max(np.max(weighted[:shape, 0]), np.max(kmeans[:shape, 0]))
max_val_or = max(np.max(weighted[:shape, 1]), np.max(kmeans[:shape, 1]))

fig, axs = plt.subplots(2)
axs[0].plot(weighted[:shape, 0]/max_val_pos, 'r', label='weighted position')
axs[0].plot(kmeans[:shape, 0]/max_val_pos, 'b', label='kmeans  position')
axs[1].plot(weighted[:shape, 1]/max_val_or, 'r', label='weighted orientation')
axs[1].plot(kmeans[:shape, 1]/max_val_or, 'b', label='kmeans orientation')
axs[0].legend()
axs[1].legend()
axs[0].set_title('Iterations vs Normalized L2 position error')
axs[1].set_title('Iterations vs Normalized L1 orientation error')
plt.show()
