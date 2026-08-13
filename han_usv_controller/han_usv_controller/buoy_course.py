"""Extra buoy layout used to exercise lidar avoidance in Wayfinding."""


# (entity name, model color, x, y). Coordinates are in the Sydney VRX ENU frame.
BUOY_SPECS = (
    ('han_gate_1_red', 'red', -542.32, 172.72),
    ('han_gate_1_green', 'green', -514.30, 183.44),
    ('han_gate_2_red', 'red', -547.36, 185.88),
    ('han_gate_2_green', 'green', -519.34, 196.61),
    ('han_gate_3_red', 'red', -552.40, 199.05),
    ('han_gate_3_green', 'green', -524.38, 209.77),
    ('han_gate_4_red', 'red', -509.45, 233.53),
    ('han_gate_4_green', 'green', -522.74, 206.63),
    ('han_gate_5_red', 'red', -480.37, 219.17),
    ('han_gate_5_green', 'green', -493.66, 192.27),
    ('han_gate_6_red', 'red', -451.30, 204.80),
    ('han_gate_6_green', 'green', -464.58, 177.90),
    ('han_obstacle_1', 'orange', -542.04, 180.38),
    ('han_obstacle_2', 'orange', -524.66, 202.11),
    ('han_obstacle_3', 'orange', -496.82, 223.94),
    ('han_obstacle_4', 'orange', -477.22, 187.49),
)


# Six red/green gates add visible radar structure to the COLREGs lesson while
# keeping the four dedicated orange obstacles exclusive to the buoy course.
COLREGS_LEARNING_BUOYS = BUOY_SPECS[:12]


# A short barrier centered on the first long Wayfinding leg. The 4 m spacing
# overlaps after the configured 3 m occupancy inflation, forcing a curved plan
# around one end while leaving ample open water on both sides.
LATTICE_STRESS_SPECS = (
    ('han_lattice_barrier_south', 'orange', -534.65, 205.84),
    ('han_lattice_barrier_center', 'orange', -538.39, 204.41),
    ('han_lattice_barrier_north', 'orange', -542.13, 202.98),
)
