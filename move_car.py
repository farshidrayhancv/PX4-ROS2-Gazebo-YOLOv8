#!/usr/bin/env python3
"""Drive a car around the Sonoma Raceway track using Ackermann steering.

The car follows the track centerline using a pure pursuit controller.
Speed varies with curvature: faster on straights, slower on bends.

The car model must have the gz-sim-ackermann-steering-system plugin
configured (see models/hatchback_blue/model.sdf).
"""

import math
import subprocess
import time

# ---------------------------------------------------------------------------
# Track centerline waypoints [x, y, z] extracted from Sonoma Raceway mesh.
# 368 waypoints at ~8m spacing forming a closed 3.3km loop.
# fmt: off
WAYPOINTS = [
    [283.5,-154.5,2.4],[291.5,-157.2,2.3],[298.5,-162.1,2.0],[305.5,-167.7,1.7],
    [312.5,-173.4,1.7],[319.5,-179.0,1.7],[326.5,-184.7,1.5],[333.5,-190.4,1.1],
    [340.5,-196.1,1.0],[347.5,-201.6,1.0],[354.5,-206.9,0.8],[361.5,-212.1,0.5],
    [368.5,-217.6,0.4],[375.5,-223.6,0.3],[381.5,-229.0,0.2],[387.4,-234.6,0.1],
    [393.0,-240.5,0.0],[399.1,-247.5,-0.0],[404.8,-254.5,-0.1],[409.8,-261.5,-0.1],
    [414.4,-268.5,-0.1],[417.4,-276.5,-0.1],[416.4,-284.5,-0.1],[416.2,-292.5,-0.1],
    [418.5,-300.5,-0.1],[421.0,-308.5,-0.1],[423.5,-316.5,-0.1],[426.0,-324.5,-0.1],
    [428.5,-332.5,-0.1],[431.1,-340.5,-0.1],[433.8,-348.5,-0.1],[436.5,-356.5,-0.1],
    [439.0,-364.5,-0.1],[441.3,-372.5,-0.1],[443.8,-380.5,-0.1],[446.3,-388.5,-0.1],
    [448.8,-396.5,-0.1],[451.4,-404.5,-0.1],[453.8,-412.5,-0.1],[456.2,-420.5,-0.1],
    [458.9,-428.5,-0.1],[462.4,-436.5,-0.1],[466.7,-444.5,-0.1],[471.5,-451.5,-0.1],
    [476.5,-458.5,-0.1],[483.5,-462.4,-0.1],[491.5,-465.4,-0.1],[499.5,-466.5,-0.1],
    [507.5,-466.2,-0.1],[515.5,-463.5,-0.1],[522.0,-458.2,-0.1],[527.3,-451.5,-0.1],
    [531.4,-443.5,-0.1],[533.2,-435.5,-0.1],[532.4,-427.5,-0.1],[530.1,-419.5,-0.1],
    [527.5,-411.5,-0.1],[524.6,-403.5,-0.1],[521.8,-395.5,-0.1],[519.2,-387.5,-0.1],
    [516.6,-379.5,-0.1],[513.8,-371.5,-0.1],[511.2,-363.5,-0.1],[508.5,-355.5,-0.1],
    [505.8,-347.5,-0.1],[503.2,-339.5,-0.1],[500.9,-331.5,-0.1],[498.8,-323.5,-0.1],
    [496.4,-315.5,-0.1],[492.4,-307.5,-0.1],[489.3,-299.5,-0.1],[488.5,-291.5,-0.1],
    [487.2,-283.5,-0.1],[484.0,-275.5,-0.1],[479.9,-268.5,-0.1],[477.2,-260.5,-0.2],
    [475.6,-252.5,-0.2],[473.2,-244.5,-0.2],[470.8,-236.5,-0.2],[468.4,-228.5,-0.2],
    [465.8,-220.5,-0.2],[463.2,-212.5,-0.3],[460.6,-204.5,-0.3],[458.3,-196.5,-0.3],
    [455.8,-188.5,-0.3],[453.4,-180.5,-0.3],[451.0,-172.5,-0.4],[448.5,-164.5,-0.4],
    [445.8,-156.5,-0.4],[443.4,-148.5,-0.4],[440.8,-140.5,-0.4],[437.9,-132.5,-0.5],
    [435.0,-124.5,-0.5],[431.3,-116.5,-0.5],[427.0,-108.5,-0.5],[423.1,-101.5,-0.5],
    [418.4,-94.5,-0.5],[412.8,-87.5,-0.5],[407.3,-81.5,-0.5],[401.5,-75.9,-0.5],
    [395.5,-70.4,-0.5],[388.5,-64.8,-0.4],[381.5,-60.6,-0.3],[373.5,-56.6,-0.0],
    [365.5,-52.8,0.3],[357.5,-49.6,0.8],[349.5,-47.0,1.2],[341.5,-44.4,1.5],
    [333.5,-41.6,1.7],[326.1,-37.9,1.8],[319.2,-32.0,2.1],[312.5,-26.4,2.4],
    [305.5,-21.4,2.6],[298.5,-16.4,2.7],[291.5,-10.8,2.8],[285.5,-5.3,2.8],
    [279.5,0.5,2.8],[273.5,6.5,2.8],[267.5,12.4,2.8],[260.8,18.6,2.8],
    [254.6,24.6,2.9],[248.4,31.2,3.1],[241.5,37.3,3.4],[234.5,43.1,3.5],
    [227.5,49.0,3.6],[220.5,55.1,3.7],[213.5,61.0,3.7],[206.5,66.8,3.8],
    [199.5,72.7,3.9],[192.5,78.6,4.0],[185.5,84.1,4.3],[178.5,89.9,4.7],
    [171.5,95.9,5.1],[164.5,101.6,5.6],[157.5,107.2,6.1],[150.5,112.9,6.7],
    [143.5,118.8,7.1],[136.5,124.6,7.6],[129.5,130.2,8.0],[122.5,136.0,8.2],
    [115.5,142.1,8.5],[108.5,148.0,8.7],[101.5,153.6,9.0],[94.5,159.7,9.3],
    [87.5,164.6,9.7],[79.5,167.2,10.4],[71.8,171.5,10.9],[65.8,177.8,10.9],
    [59.4,184.5,10.9],[53.7,190.5,10.9],[47.2,197.0,10.9],[40.5,203.4,11.2],
    [34.5,208.8,11.5],[27.5,214.6,11.8],[20.5,219.6,11.8],[13.5,223.6,11.9],
    [5.5,227.7,11.9],[-2.5,230.8,12.0],[-10.5,232.8,12.2],[-18.5,233.5,12.4],
    [-26.5,233.5,12.8],[-34.5,233.5,13.3],[-42.5,233.5,13.8],[-50.5,233.5,14.2],
    [-58.5,233.9,14.7],[-66.5,234.4,15.0],[-74.5,234.8,15.2],[-82.5,236.1,15.3],
    [-90.5,238.1,15.4],[-98.5,241.2,15.5],[-106.5,245.6,15.5],[-113.5,250.2,15.6],
    [-120.5,255.9,15.7],[-126.5,261.5,15.7],[-132.2,267.5,15.8],[-137.8,273.5,16.0],
    [-143.5,279.5,16.1],[-149.5,285.5,16.3],[-155.5,291.4,16.5],[-161.5,297.0,16.8],
    [-168.5,303.1,17.1],[-175.5,308.9,17.5],[-182.5,313.8,17.9],[-189.5,317.8,18.4],
    [-197.5,321.5,19.0],[-205.5,324.1,19.5],[-213.5,325.9,19.9],[-221.5,328.0,20.4],
    [-229.5,330.8,20.7],[-237.5,334.8,21.1],[-244.5,338.8,21.5],[-251.5,342.9,21.8],
    [-258.5,347.5,22.2],[-265.5,352.5,22.5],[-272.5,357.8,22.9],[-279.5,363.5,23.2],
    [-286.5,369.4,23.6],[-293.5,375.5,23.9],[-299.5,380.8,24.2],[-305.5,386.4,24.5],
    [-311.5,391.7,24.8],[-318.5,397.6,25.1],[-325.5,403.7,25.5],[-332.5,409.7,25.8],
    [-339.5,415.3,26.1],[-346.5,419.5,26.4],[-354.5,422.7,26.7],[-362.5,424.3,27.0],
    [-370.5,423.8,27.3],[-378.5,420.7,27.5],[-385.4,415.3,27.6],[-391.0,409.5,27.7],
    [-396.8,402.5,27.8],[-402.2,395.5,28.1],[-407.1,388.5,28.3],[-412.2,381.5,28.6],
    [-417.6,374.5,29.0],[-423.0,367.5,29.4],[-428.4,360.5,29.9],[-433.6,353.5,30.2],
    [-438.8,346.5,30.5],[-444.1,339.5,31.0],[-449.4,332.5,31.6],[-454.6,325.5,31.9],
    [-460.0,318.5,32.1],[-465.4,311.5,32.6],[-470.9,304.5,33.1],[-476.2,297.5,33.6],
    [-481.4,290.5,34.1],[-486.8,283.5,34.6],[-492.0,276.5,35.1],[-497.1,269.5,35.4],
    [-502.2,262.5,35.8],[-507.7,255.5,36.3],[-513.2,248.5,36.8],[-518.7,241.5,37.2],
    [-523.8,234.5,37.7],[-528.0,227.5,38.1],[-530.0,219.5,38.3],[-528.2,211.5,38.1],
    [-524.2,203.5,38.0],[-519.9,196.5,38.0],[-515.2,189.5,37.9],[-510.5,182.5,37.9],
    [-505.8,175.5,37.9],[-501.4,168.5,38.0],[-497.0,161.5,38.0],[-492.5,154.5,38.1],
    [-487.9,147.5,38.2],[-483.4,140.5,38.3],[-478.8,133.5,38.5],[-474.1,126.5,38.7],
    [-469.3,119.5,38.9],[-464.7,112.5,39.1],[-460.2,105.5,39.4],[-455.6,98.5,39.7],
    [-450.8,91.5,39.9],[-446.2,84.5,40.3],[-441.5,77.5,40.6],[-436.8,70.5,40.9],
    [-432.2,63.5,41.0],[-427.5,56.5,41.3],[-422.9,49.5,41.5],[-418.4,42.5,41.6],
    [-413.9,35.5,41.7],[-409.4,28.5,41.7],[-404.8,21.5,41.8],[-399.8,14.5,41.8],
    [-394.5,7.5,41.8],[-388.3,0.5,41.7],[-381.5,-5.6,41.6],[-374.5,-10.1,41.4],
    [-366.5,-13.7,41.0],[-358.5,-15.8,40.6],[-350.5,-17.0,40.1],[-342.5,-18.0,39.6],
    [-334.5,-18.0,39.0],[-326.5,-17.5,38.5],[-318.5,-17.5,37.9],[-310.5,-17.5,37.4],
    [-302.5,-17.5,37.0],[-294.5,-17.8,36.6],[-286.5,-18.7,36.4],[-278.5,-20.9,36.4],
    [-270.5,-24.8,36.5],[-263.5,-29.5,36.7],[-256.5,-35.6,37.0],[-250.9,-41.5,37.3],
    [-245.0,-48.5,37.6],[-239.6,-55.5,37.9],[-234.7,-62.5,38.0],[-229.8,-69.5,38.1],
    [-224.6,-76.5,38.1],[-219.2,-83.5,38.1],[-213.6,-90.5,38.0],[-207.6,-97.5,37.8],
    [-201.7,-104.5,37.7],[-195.6,-111.5,37.5],[-190.2,-117.5,37.3],[-184.5,-123.5,37.1],
    [-178.8,-129.5,36.9],[-172.2,-136.0,36.6],[-166.5,-141.6,36.4],[-160.5,-147.5,36.1],
    [-154.5,-153.2,35.8],[-148.5,-158.8,35.5],[-142.5,-164.2,35.2],[-135.5,-170.1,34.9],
    [-128.5,-175.4,34.6],[-121.5,-180.3,34.2],[-114.5,-185.1,33.9],[-107.5,-189.2,33.6],
    [-99.5,-192.8,33.2],[-91.5,-194.9,32.8],[-83.5,-195.5,32.3],[-75.5,-195.2,31.6],
    [-67.5,-193.7,30.7],[-59.5,-190.5,29.5],[-52.5,-185.8,28.0],[-46.1,-179.4,26.3],
    [-41.0,-172.5,24.7],[-37.0,-165.5,23.1],[-33.1,-157.5,21.5],[-29.6,-149.5,20.1],
    [-26.1,-141.5,18.7],[-21.8,-133.5,17.3],[-16.8,-126.5,16.1],[-10.9,-119.5,14.9],
    [-5.4,-113.5,14.0],[0.5,-107.5,13.2],[6.5,-102.0,12.4],[13.5,-96.2,11.7],
    [20.5,-91.5,11.1],[28.5,-87.2,10.5],[36.5,-83.6,9.8],[44.5,-81.2,9.3],
    [52.5,-80.0,8.9],[60.5,-79.2,8.4],[68.5,-78.7,8.0],[76.5,-78.2,7.7],
    [84.5,-77.6,7.4],[92.5,-77.5,7.1],[100.5,-77.5,6.9],[108.5,-77.5,6.8],
    [116.5,-78.0,6.5],[124.5,-78.5,6.3],[132.5,-78.6,6.1],[140.5,-79.1,5.9],
    [148.5,-79.5,5.8],[156.5,-79.9,5.6],[164.5,-80.4,5.4],[172.5,-80.7,5.3],
    [180.5,-81.8,5.1],[188.5,-83.9,4.9],[196.5,-86.6,4.7],[204.5,-90.4,4.4],
    [211.5,-94.4,4.3],[218.5,-99.1,4.1],[225.5,-104.4,3.9],[232.5,-109.9,3.8],
    [239.5,-115.3,3.5],[246.5,-120.9,3.3],[253.5,-126.6,3.2],[259.5,-132.3,3.2],
    [263.8,-139.5,3.1],[268.6,-146.1,2.8],[275.5,-151.6,2.6],[283.5,-154.5,2.4],
]

# Curvature-based target speeds (m/s) for each waypoint.
SPEEDS = [
    15.0,10.7,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,
    15.0,15.0,15.0,15.0,11.6,7.7,13.3,10.3,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,
    15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,13.5,15.0,8.5,12.6,11.8,12.2,
    10.8,9.7,11.6,12.3,11.2,10.2,12.3,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,
    15.0,15.0,15.0,15.0,12.6,13.6,11.0,15.0,11.8,12.8,11.9,13.1,13.5,15.0,15.0,15.0,
    15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,13.7,
    15.0,15.0,15.0,15.0,15.0,13.1,13.7,15.0,15.0,15.0,15.0,15.0,12.9,11.6,15.0,15.0,
    15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,
    15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,13.5,
    10.7,12.1,10.9,15.0,15.0,15.0,15.0,15.0,15.0,15.0,13.5,15.0,13.6,13.1,12.5,13.7,
    15.0,15.0,15.0,15.0,15.0,15.0,13.1,15.0,13.2,13.1,15.0,13.7,15.0,15.0,15.0,15.0,
    15.0,15.0,15.0,15.0,15.0,15.0,13.7,15.0,13.2,13.6,15.0,13.6,13.4,15.0,15.0,15.0,
    15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,13.0,12.6,12.3,11.1,
    10.3,10.8,13.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,
    15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,10.3,8.1,11.5,
    15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,
    15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,13.4,12.8,12.8,
    12.6,13.3,15.0,13.1,15.0,15.0,15.0,15.0,15.0,13.7,12.7,12.4,12.9,13.2,13.7,15.0,
    15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,
    15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,13.3,12.7,12.2,13.3,12.6,12.2,11.9,12.3,
    12.9,13.4,15.0,15.0,15.0,15.0,13.2,15.0,15.0,15.0,15.0,15.0,13.5,13.6,15.0,12.9,
    13.1,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,
    13.3,13.3,15.0,13.1,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,11.0,15.0,11.0,10.3,
]
# fmt: on

# ---------------------------------------------------------------------------
# Configuration
MODEL_NAME = "hatchback_blue_1"
WORLD_NAME = "default"
WHEELBASE = 2.6          # metres (must match model.sdf)
LOOKAHEAD = 10.0         # pure pursuit lookahead distance (m)
UPDATE_HZ = 10           # control loop rate
MAX_STEER = 0.6          # max steering angle (rad, must match model.sdf)

_cmd_proc = None


def send_twist(linear_x, angular_z):
    """Publish a Twist message to the Ackermann steering plugin."""
    global _cmd_proc
    if _cmd_proc is not None and _cmd_proc.poll() is None:
        return  # previous command still running

    msg = f'"linear: {{x: {linear_x:.2f}}}, angular: {{z: {angular_z:.4f}}}"'
    _cmd_proc = subprocess.Popen(
        ['gz', 'topic', '-t',
         f'/model/{MODEL_NAME}/cmd_vel',
         '-m', 'gz.msgs.Twist',
         '-p', f'linear: {{x: {linear_x:.2f}}}, angular: {{z: {angular_z:.4f}}}'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )


def get_pose():
    """Query the car's current pose from Gazebo (returns x, y, yaw or None).

    Reads one message from the dynamic_pose/info topic and extracts the
    top-level model pose matching MODEL_NAME.
    """
    try:
        result = subprocess.run(
            ['gz', 'topic', '-t',
             f'/world/{WORLD_NAME}/dynamic_pose/info',
             '-e', '-n', '1'],
            capture_output=True, text=True, timeout=3
        )
        output = result.stdout
        lines = output.split('\n')
        in_model = False
        depth = 0
        x = y = qz = qw = 0.0
        got_position = False
        got_orientation = False
        in_position = False
        in_orientation = False
        for line in lines:
            s = line.strip()
            if not in_model:
                if f'name: "{MODEL_NAME}"' in s:
                    in_model = True
                    depth = 0
                continue
            # Track brace depth so we only read top-level position/orientation
            if '{' in s:
                depth += 1
            if '}' in s:
                depth -= 1
                if depth < 0:
                    break  # exited this pose block
                if in_position:
                    in_position = False
                if in_orientation:
                    in_orientation = False
                continue
            if depth == 1 and 'position {' in s:
                in_position = True
                continue
            if depth == 1 and 'orientation {' in s:
                in_orientation = True
                continue
            if in_position:
                if s.startswith('x:'):
                    x = float(s.split(':')[1])
                elif s.startswith('y:'):
                    y = float(s.split(':')[1])
                    got_position = True
            if in_orientation:
                if s.startswith('z:'):
                    qz = float(s.split(':')[1])
                elif s.startswith('w:'):
                    qw = float(s.split(':')[1])
                    got_orientation = True

        if in_model and got_position and got_orientation:
            yaw = math.atan2(2 * qz * qw, 1 - 2 * qz * qz)
            return x, y, yaw
    except Exception:
        pass
    return None


def normalize_angle(a):
    """Wrap angle to [-pi, pi]."""
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def find_nearest_waypoint(x, y, start_idx=0):
    """Find the index of the waypoint nearest to (x, y), searching forward."""
    n = len(WAYPOINTS)
    best_dist = float('inf')
    best_idx = start_idx
    # Search in a window around start_idx
    for offset in range(-20, 40):
        idx = (start_idx + offset) % n
        wp = WAYPOINTS[idx]
        d = (wp[0] - x) ** 2 + (wp[1] - y) ** 2
        if d < best_dist:
            best_dist = d
            best_idx = idx
    return best_idx


def find_lookahead_point(x, y, nearest_idx, lookahead):
    """Find the point on the path that is `lookahead` metres ahead."""
    n = len(WAYPOINTS)
    accum = 0.0
    idx = nearest_idx
    px, py = x, y

    while accum < lookahead:
        next_idx = (idx + 1) % n
        wp = WAYPOINTS[next_idx]
        dx = wp[0] - px
        dy = wp[1] - py
        seg_len = math.sqrt(dx * dx + dy * dy)
        if accum + seg_len >= lookahead:
            # Interpolate
            remaining = lookahead - accum
            ratio = remaining / seg_len if seg_len > 0 else 0
            lx = px + dx * ratio
            ly = py + dy * ratio
            return lx, ly, next_idx
        accum += seg_len
        px, py = wp[0], wp[1]
        idx = next_idx

    wp = WAYPOINTS[(nearest_idx + 1) % n]
    return wp[0], wp[1], (nearest_idx + 1) % n


def cross_track_error(x, y, nearest_idx):
    """Compute signed cross-track error (positive = car is to the RIGHT of path).

    Projects the car position onto the line segment from the nearest waypoint
    to the next, and returns the signed perpendicular distance.
    """
    n = len(WAYPOINTS)
    wp_a = WAYPOINTS[nearest_idx]
    wp_b = WAYPOINTS[(nearest_idx + 1) % n]

    # Path segment vector
    seg_x = wp_b[0] - wp_a[0]
    seg_y = wp_b[1] - wp_a[1]
    seg_len = math.sqrt(seg_x ** 2 + seg_y ** 2)
    if seg_len < 0.01:
        return 0.0

    # Vector from waypoint A to car
    dx = x - wp_a[0]
    dy = y - wp_a[1]

    # Cross product (2D): positive if car is to the right of the path
    return (seg_x * dy - seg_y * dx) / seg_len


# Cross-track error correction gain
CTE_GAIN = 0.03


def pure_pursuit_steering(x, y, yaw, lookahead_x, lookahead_y, cte=0.0):
    """Compute steering angle using pure pursuit geometry + cross-track correction.

    Returns (steering_angle, is_forward) where is_forward indicates whether
    the lookahead point is ahead of the car (True) or behind (False).
    """
    dx = lookahead_x - x
    dy = lookahead_y - y
    # Transform to vehicle frame
    local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
    local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy

    ld = math.sqrt(local_x ** 2 + local_y ** 2)
    if ld < 0.1:
        return 0.0, True

    # Pure pursuit: steering = atan(2 * L * sin(alpha) / ld)
    curvature = 2.0 * local_y / (ld * ld)
    steering = math.atan(WHEELBASE * curvature)

    # Add cross-track error correction (clamped to ±0.1 rad max)
    cte_correction = max(-0.1, min(0.1, CTE_GAIN * cte))
    steering -= cte_correction

    return max(-MAX_STEER, min(MAX_STEER, steering)), local_x > 0


def main():
    n = len(WAYPOINTS)
    print(f"Driving '{MODEL_NAME}' around Sonoma Raceway")
    print(f"  Waypoints : {n}")
    print(f"  Track     : ~3.3 km closed loop")
    print(f"  Speed     : {min(SPEEDS):.0f}-{max(SPEEDS):.0f} m/s")
    print(f"  Lookahead : {LOOKAHEAD} m")
    print("Press Ctrl+C to stop.\n")

    dt = 1.0 / UPDATE_HZ
    current_idx = 0
    laps = 0
    prev_idx = 0

    # Dead-reckoned pose (used if gz service fails)
    dr_x, dr_y, dr_yaw = WAYPOINTS[0][0], WAYPOINTS[0][1], 0.0
    use_dead_reckoning = False

    while True:
        # Get current pose
        pose = get_pose()
        if pose is not None:
            x, y, yaw = pose
            use_dead_reckoning = False
        elif use_dead_reckoning:
            x, y, yaw = dr_x, dr_y, dr_yaw
        else:
            # First call failed, use waypoint 0
            x, y, yaw = WAYPOINTS[0][0], WAYPOINTS[0][1], 0.0
            use_dead_reckoning = True

        # Find nearest waypoint
        current_idx = find_nearest_waypoint(x, y, current_idx)

        # Detect lap completion
        if current_idx < 10 and prev_idx > n - 20:
            laps += 1
            print(f"  Lap {laps} complete!")
        prev_idx = current_idx

        # Target speed from curvature profile, capped at 8 m/s.
        # Look 20 waypoints ahead and take the minimum to brake early.
        target_speed = min(8.0,
                           min(SPEEDS[(current_idx + i) % n] for i in range(20)))

        # Find lookahead point and compute steering with CTE correction
        lx, ly, _ = find_lookahead_point(x, y, current_idx, LOOKAHEAD)
        cte = cross_track_error(x, y, current_idx)
        steering, is_forward = pure_pursuit_steering(x, y, yaw, lx, ly, cte)

        # If the car is facing completely wrong (>90°), slow to recovery speed
        heading_to_la = math.atan2(ly - y, lx - x)
        heading_err = abs(normalize_angle(heading_to_la - yaw))
        if heading_err > math.radians(90) or not is_forward:
            target_speed = 3.0

        # Send command
        send_twist(target_speed, steering)

        # Dead-reckon update (fallback)
        dr_x = x + target_speed * math.cos(yaw) * dt
        dr_y = y + target_speed * math.sin(yaw) * dt
        dr_yaw = yaw + (target_speed / WHEELBASE) * math.tan(steering) * dt

        time.sleep(dt)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        # Stop the car
        send_twist(0.0, 0.0)
        print("\nStopped.")
