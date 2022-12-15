#!/usr/bin/env python3

from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import FormatStrFormatter

dt = []

with Reader('rosbag2_2022_12_14-21_15_41') as reader:
    prev_t = None
    for connection, timestamp, rawdata in reader.messages():
        if connection.topic == '/diagnostics':
            msg = deserialize_cdr(rawdata, connection.msgtype)
            t = msg.header.stamp.sec + 1.0e-9 * msg.header.stamp.nanosec
            if prev_t is not None:
                dt.append(t - prev_t)
            prev_t = t

print(min(dt))
print(max(dt))
d = np.array(dt[10:])
fig, axs = plt.subplots(1, 1, tight_layout=True)
axs.xaxis.set_major_formatter(FormatStrFormatter('%0.6f'))
N, bins, patches = axs.hist(d, bins=50)
plt.show()
