import numpy as np
import struct
a = np.array([61, 105, 25, 143, 69, 69, 69], dtype=np.uint8)


by = bytearray(a[0:4])
print(by)
print(struct.unpack('>f', a[0:4]))
