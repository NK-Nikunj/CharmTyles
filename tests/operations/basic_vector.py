import numpy as np
import time

size = 10000000

start = time.time()
A = np.full((size), 1.)
B = np.full((size), 2.)
C = np.full((size), -4.)
D = np.full((size), 3.)

E = A + B + C + D
end = time.time()

print(f"Execution Time: {end - start}s")
