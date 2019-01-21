import matplotlib.pyplot as plt
import numpy as np
import time

# create the figure
fig = plt.figure()
ax = fig.add_subplot(111)
im = ax.imshow(np.random.random((3,3)))
plt.show(block=False)

# draw some data in loop
for i in range(10):
    # wait for a second
    time.sleep(1)
    # replace the image contents
    im.set_array(np.random.random((3,3)))
    # redraw the figure
    fig.canvas.draw()