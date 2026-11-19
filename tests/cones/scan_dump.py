# read /scan from the bag once -> npz (stamp, bag time, ranges, angle_min, inc, range_min/max)
import sys, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/scan']))
st, bt, R = [], [], []
meta = None
while r.has_next():
    _, data, t = r.read_next()
    m = deserialize_message(data, LaserScan)
    st.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9); bt.append(t * 1e-9)
    R.append(np.asarray(m.ranges, np.float32))
    if meta is None:
        meta = (m.angle_min, m.angle_increment, m.range_min, m.range_max, m.header.frame_id)
np.savez(sys.argv[2], stamp=np.array(st), bagt=np.array(bt), R=np.array(R), amin=meta[0], inc=meta[1],
         rmin=meta[2], rmax=meta[3])
print(len(st), 'scans, frame', meta[4], 'stamp-bag dt median %.3f s' % np.median(np.array(st) - np.array(bt)))
