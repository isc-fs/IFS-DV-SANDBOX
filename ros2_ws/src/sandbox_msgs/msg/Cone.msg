# One cone. Coordinates are in the frame of the ConeArray header
# (base_link for detections, map for a map).

uint8 YELLOW=0        # right side of the track
uint8 BLUE=1          # left side of the track
uint8 ORANGE_BIG=2    # start/finish gate
uint8 ORANGE_SMALL=3
uint8 UNKNOWN=4       # "I saw a cone but don't know its colour"

float64 x
float64 y
uint8 color
float32 confidence    # 0..1. Use 1.0 if you don't care.
