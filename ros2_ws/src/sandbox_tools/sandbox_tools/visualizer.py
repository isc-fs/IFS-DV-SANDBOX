"""Turns sandbox topics into markers for Lichtblick / Foxglove.

  /sandbox/cones        -> /sandbox/viz/cones   (what perception sees, tall cones)
  /sandbox/map          -> /sandbox/viz/map     (the map, short cylinders)
  /testing_only/track   -> /sandbox/viz/track   (ground truth, small grey dots)

You never need to write marker code: publish the contract topics and
they show up here.
"""
from __future__ import annotations

import rclpy
from fs_msgs.msg import Track
from rclpy.node import Node
from sandbox_msgs.msg import Cone, ConeArray
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

from sandbox_tools import contract
from sandbox_tools.node_utils import run

COLORS = {
    Cone.YELLOW: (1.0, 0.85, 0.0),
    Cone.BLUE: (0.1, 0.35, 1.0),
    Cone.ORANGE_BIG: (1.0, 0.45, 0.0),
    Cone.ORANGE_SMALL: (1.0, 0.6, 0.2),
    Cone.UNKNOWN: (0.8, 0.8, 0.8),
}


def make_markers(header: Header, cones, ns: str, height: float, radius: float,
                 alpha: float = 1.0, gray: bool = False) -> MarkerArray:
    out = MarkerArray()
    clear = Marker(header=header, ns=ns, action=Marker.DELETEALL)
    out.markers.append(clear)
    for i, (x, y, color) in enumerate(cones):
        m = Marker(header=header, ns=ns, id=i, type=Marker.CYLINDER, action=Marker.ADD)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), height / 2
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = 2 * radius
        m.scale.z = height
        r, g, b = (0.5, 0.5, 0.5) if gray else COLORS.get(color, COLORS[Cone.UNKNOWN])
        m.color = ColorRGBA(r=r, g=g, b=b, a=alpha)
        out.markers.append(m)
    return out


class Visualizer(Node):
    def __init__(self):
        super().__init__("visualizer")
        self.pub_cones = self.create_publisher(MarkerArray, "/sandbox/viz/cones", 10)
        self.pub_map = self.create_publisher(MarkerArray, "/sandbox/viz/map", 10)
        self.pub_track = self.create_publisher(MarkerArray, "/sandbox/viz/track", contract.LATCHED_QOS)
        self.create_subscription(ConeArray, contract.CONES, self.on_cones, 10)
        self.create_subscription(ConeArray, contract.MAP, self.on_map, 10)
        self.create_subscription(Track, contract.GT_TRACK, self.on_track, contract.LATCHED_QOS)

    def on_cones(self, msg: ConeArray):
        cones = [(c.x, c.y, c.color) for c in msg.cones]
        self.pub_cones.publish(make_markers(msg.header, cones, "detections", 0.5, 0.12))

    def on_map(self, msg: ConeArray):
        cones = [(c.x, c.y, c.color) for c in msg.cones]
        self.pub_map.publish(make_markers(msg.header, cones, "map", 0.2, 0.2, alpha=0.8))

    def on_track(self, msg: Track):
        header = Header(frame_id=contract.MAP_FRAME, stamp=self.get_clock().now().to_msg())
        cones = [(c.location.x, c.location.y, c.color) for c in msg.track]
        self.pub_track.publish(make_markers(header, cones, "gt", 0.05, 0.08, alpha=0.6, gray=True))


def main():
    run(Visualizer)


if __name__ == "__main__":
    main()
