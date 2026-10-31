"""Is a `sb-new <you> --from ifs09` copy the same detector as perception:=ifs09?

Feeds the same recorded scans to the vendored detector twice (it uses random
RANSAC, so it doesn't match itself bit for bit) and to the student copy, and
compares the cones (matched within 10 cm). Run inside the sandbox shell after
`sb-new zztest --from ifs09 && sb-build`:

    python3 compare_copy.py <bag dir or .mcap> <number of scans>
"""
import sys, logging, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from scipy.spatial import cKDTree
from cone_detection.config import ConeDetectionConfig as VCfg
from cone_detection.strategies import BaseConeDetection
from sandbox_tools.ifs09 import Ifs09Perception
from sandbox_tools.pointcloud import read_fields
from zztest_pipeline import perception as student
bag, maxn = sys.argv[1], int(sys.argv[2])
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=bag, storage_id=""), rosbag2_py.ConverterOptions("cdr","cdr"))
r.set_filter(rosbag2_py.StorageFilter(topics=["/lidar_points"])); msgs=[]
while r.has_next(): msgs.append(r.read_next()[1])
msgs = msgs[::max(1,len(msgs)//maxn)][:maxn]; log=logging.getLogger("x")
A = Ifs09Perception(BaseConeDetection, VCfg(), log); A2 = Ifs09Perception(BaseConeDetection, VCfg(), log)
B = Ifs09Perception(student.STRATEGY, student.ConeDetectionConfig(), log)
def cmp(a, b):
    if len(a)==0 or len(b)==0: return len(a)==len(b), 0.0, abs(len(a)-len(b))
    d,_ = cKDTree(b).query(a); m = d < 0.1
    return False, float(np.max(d[m])) if m.any() else 0.0, (len(a)-m.sum()) + (len(b)-m.sum())
for name, X, Y in (("vendored vs vendored", A, A2), ("vendored vs student", A, B)):
    exact=0; maxd=0; unmatched=0; total=0
    for data in msgs:
        p = read_fields(deserialize_message(data, PointCloud2), ("x","y","z","intensity"))
        a = X.find_cones(p)[0]; b = Y.find_cones(p)[0]
        exact += a.shape==b.shape and np.array_equal(a,b)
        _, md, um = cmp(a,b); maxd=max(maxd,md); unmatched+=um; total+=len(a)
    print(f"{bag.split('/')[-1]} {name}: exact {exact}/{len(msgs)}, cones {total}, unmatched {unmatched}, max pos diff {maxd*100:.1f} cm")
