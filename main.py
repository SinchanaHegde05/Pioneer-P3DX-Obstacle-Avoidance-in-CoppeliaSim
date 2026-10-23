import math
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# -------- SETTINGS --------
HOST = "localhost"
PORT = 23000

FORWARD_SPEED = 2.0        # wheel rad/s
TURN_SPEED = 1.0           # slower = less overshoot
REVERSE_SPEED = -1.5

TURN_ANGLE = 10            # degrees: every avoidance turn rotates exactly this much

STOP_DISTANCE = 0.15       # start avoiding when something is this close ahead
SLOW_DISTANCE = 0.20       # begin slowing down inside this range
CRITICAL_DISTANCE = 0.9   # only back up first if this close (must be < STOP_DISTANCE)
SIDE_DISTANCE = 0.12       # steer away from walls close on the sides
REAR_CLEARANCE = 0.10      # don't reverse if something is this close behind

KEEP_DIRECTION_TIME = 3.0  # seconds: if blocked again this soon after a turn, keep turning the same way
REVERSE_TIME = 0.8         # seconds
MAX_TURN_TIME = 8.0        # seconds (safety limit for one turn)
MAX_SIM_TIME = 120         # seconds
DEBUG_PERIOD = 1.0         # seconds between status prints

# Floor edge protection (the floor has no walls, so the robot would fall off)
EDGE_OFFSET = 0.30         # metres from robot centre to the edge when it should react
# If automatic floor detection fails, set (xmin, xmax, ymin, ymax) in world metres:
MANUAL_ARENA = None        # e.g. (-2.5, 2.5, -2.5, 2.5)

INF = float("inf")


def main():
    client = RemoteAPIClient(host=HOST, port=PORT)
    sim = client.require("sim")

    # ---------- Find motors ----------
    objects = sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0)

    aliases = {}
    for obj in objects:
        try:
            aliases[obj] = sim.getObjectAlias(obj, 1)
        except Exception:
            pass

    print("\n--- Searching for Pioneer robot components ---")

    left_motor = None
    right_motor = None

    for obj, name in aliases.items():
        n = name.lower()
        if "leftmotor" in n and left_motor is None:
            left_motor = obj
            print("Left motor:", name)
        elif "rightmotor" in n and right_motor is None:
            right_motor = obj
            print("Right motor:", name)

    if left_motor is None or right_motor is None:
        print("ERROR: Wheel motors not found.")
        print("Check the motor aliases in the scene hierarchy.")
        return

    robot = sim.getObjectParent(left_motor)
    if robot == -1:
        print("ERROR: Could not identify the Pioneer base.")
        return
    print("Robot base:", sim.getObjectAlias(robot, 1))

    # ---------- Make obstacles detectable by the sensors ----------
    robot_tree = set(sim.getObjectsInTree(robot, sim.handle_all, 0))
    robot_tree.add(robot)

    def set_detectable(handle):
        try:
            sim.setObjectSpecialProperty(
                handle, sim.objectspecialproperty_detectable_all
            )
            return True
        except Exception:
            pass
        try:
            prop = sim.getObjectProperty(handle)
            sim.setObjectProperty(handle, prop | sim.objectproperty_detectable)
            return True
        except Exception:
            return False

    made_detectable = 0
    failed = 0
    for obj, name in aliases.items():
        if obj in robot_tree or "floor" in name.lower():
            continue
        try:
            if sim.getObjectType(obj) != sim.sceneobject_shape:
                continue
        except Exception:
            continue
        if set_detectable(obj):
            made_detectable += 1
        else:
            failed += 1

    print(f"Made {made_detectable} obstacle shapes detectable"
          + (f" ({failed} failed - set Detectable manually)" if failed else ""))

    # ---------- Floor bounds (virtual wall at the edge) ----------
    def get_floor_bounds():
        best = None
        best_area = 0.0
        for obj, name in aliases.items():
            if obj in robot_tree or "floor" not in name.lower():
                continue
            try:
                if sim.getObjectType(obj) != sim.sceneobject_shape:
                    continue
                pos = sim.getObjectPosition(obj, sim.handle_world)
                try:
                    mnx = sim.getObjectFloatParam(obj, sim.objfloatparam_objbbox_min_x)
                    mxx = sim.getObjectFloatParam(obj, sim.objfloatparam_objbbox_max_x)
                    mny = sim.getObjectFloatParam(obj, sim.objfloatparam_objbbox_min_y)
                    mxy = sim.getObjectFloatParam(obj, sim.objfloatparam_objbbox_max_y)
                except Exception:
                    mnx = sim.getObjectFloatParameter(obj, sim.objfloatparam_objbbox_min_x)
                    mxx = sim.getObjectFloatParameter(obj, sim.objfloatparam_objbbox_max_x)
                    mny = sim.getObjectFloatParameter(obj, sim.objfloatparam_objbbox_min_y)
                    mxy = sim.getObjectFloatParameter(obj, sim.objfloatparam_objbbox_max_y)
                b = (pos[0] + mnx, pos[0] + mxx, pos[1] + mny, pos[1] + mxy)
                area = (b[1] - b[0]) * (b[3] - b[2])
                if area > best_area:
                    best_area, best = area, (b, name)
            except Exception:
                continue
        return best

    arena = None
    if MANUAL_ARENA is not None:
        arena = MANUAL_ARENA
        print("Using manual arena bounds:", arena)
    else:
        found = get_floor_bounds()
        if found:
            arena = found[0]
            print(f"Floor '{found[1]}' bounds: x[{arena[0]:.2f}, {arena[1]:.2f}] "
                  f"y[{arena[2]:.2f}, {arena[3]:.2f}]")
        else:
            print("WARNING: floor not found. The robot can fall off the edge. "
                  "Set MANUAL_ARENA = (xmin, xmax, ymin, ymax).")

    def ray_to_edge(px, py, dx, dy):
        xmin, xmax, ymin, ymax = arena
        t = INF
        if dx > 1e-6:
            t = min(t, (xmax - px) / dx)
        elif dx < -1e-6:
            t = min(t, (xmin - px) / dx)
        if dy > 1e-6:
            t = min(t, (ymax - py) / dy)
        elif dy < -1e-6:
            t = min(t, (ymin - py) / dy)
        return max(t, 0.0)

    # ---------- Find sensors belonging to this robot ----------
    try:
        prox_type = sim.sceneobject_proximitysensor
    except Exception:
        prox_type = sim.object_proximitysensor_type

    sensor_handles = sim.getObjectsInTree(robot, prox_type, 0)

    if not sensor_handles:
        sensor_handles = [o for o, n in aliases.items()
                          if "ultrasonicsensor" in n.lower()]

    if not sensor_handles:
        print("ERROR: No ultrasonic sensors found.")
        return

    print(f"Found {len(sensor_handles)} ultrasonic sensors.")

    # ---------- Helpers ----------
    def drive(left, right):
        sim.setJointTargetVelocity(left_motor, left)
        sim.setJointTargetVelocity(right_motor, right)

    def run_for(seconds, left, right):
        t0 = sim.getSimulationTime()
        while sim.getSimulationTime() - t0 < seconds:
            drive(left, right)
            sim.step()
        drive(0, 0)
        for _ in range(3):
            sim.step()

    fwd = [1.0, 0.0]       # forward direction in the robot frame (calibrated below)
    spin_sign = 1          # +1 if drive(-v, +v) turns the robot left (calibrated below)

    def heading():
        """World yaw (radians) of the robot's calibrated forward direction."""
        m = sim.getObjectMatrix(robot, sim.handle_world)
        wx = m[0] * fwd[0] + m[1] * fwd[1]
        wy = m[4] * fwd[0] + m[5] * fwd[1]
        return math.atan2(wy, wx)

    def wrap(a):
        return math.atan2(math.sin(a), math.cos(a))

    def turn(direction):
        """direction: +1 = turn left, -1 = turn right."""
        v = direction * spin_sign * TURN_SPEED
        drive(-v, v)

    # ---------- Start simulation ----------
    sim.setStepping(True)
    sim.startSimulation()

    try:
        for _ in range(5):
            sim.step()

        z_start = sim.getObjectPosition(robot, sim.handle_world)[2]

        # ---------- Calibrate: which way is forward? ----------
        print("\n--- Calibrating direction ---")
        p0 = sim.getObjectPosition(robot, sim.handle_world)
        M = sim.getObjectMatrix(robot, sim.handle_world)
        run_for(0.8, FORWARD_SPEED, FORWARD_SPEED)
        p1 = sim.getObjectPosition(robot, sim.handle_world)

        dx, dy, dz = p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]
        lx = M[0] * dx + M[4] * dy + M[8] * dz
        ly = M[1] * dx + M[5] * dy + M[9] * dz
        moved = math.hypot(lx, ly)

        if moved < 0.02:
            print("WARNING: robot barely moved during calibration "
                  f"({moved:.3f} m). Is another script controlling the wheels?")
        else:
            fwd[0], fwd[1] = lx / moved, ly / moved
        print(f"Forward axis in robot frame: ({fwd[0]:+.2f}, {fwd[1]:+.2f}), "
              f"moved {moved:.2f} m")

        h0 = heading()
        run_for(0.6, -TURN_SPEED, TURN_SPEED)
        h1 = heading()
        delta = wrap(h1 - h0)
        spin_sign = 1 if delta > 0 else -1
        print(f"Spin test: turned {math.degrees(delta):+.1f} deg "
              f"-> left-turn sign = {spin_sign:+d}")

        left_axis = (-fwd[1], fwd[0])

        # ---------- Bearing of each sensor from the direction it POINTS ----------
        sensors = []
        for h in sensor_handles:
            m = sim.getObjectMatrix(h, robot)
            zx, zy = m[2], m[6]
            f = zx * fwd[0] + zy * fwd[1]
            l = zx * left_axis[0] + zy * left_axis[1]
            sensors.append((h, math.degrees(math.atan2(l, f))))

        print("Sensor bearings (deg):",
              ", ".join(f"{b:.0f}" for _, b in sorted(sensors, key=lambda s: s[1])))

        def merge(s, b, dist):
            """Add a reading at bearing b (0 ahead, + left, - right) to the summary."""
            if abs(b) > 110:
                s["rear"] = min(s["rear"], dist)
                return
            if abs(b) <= 60:
                s["front"] = min(s["front"], dist)
            if b > 0:
                s["left"] = min(s["left"], dist)
                if b > 60:
                    s["side_left"] = min(s["side_left"], dist)
            else:
                s["right"] = min(s["right"], dist)
                if b < -60:
                    s["side_right"] = min(s["side_right"], dist)

        def scan():
            s = {"front": INF, "left": INF, "right": INF,
                 "side_left": INF, "side_right": INF, "rear": INF, "hits": 0}

            for handle, b in sensors:
                detected, dist, _pt, _obj, _n = sim.readProximitySensor(handle)
                if not detected:
                    continue
                s["hits"] += 1
                merge(s, b, dist)

            # Floor edge acts like a wall
            if arena is not None:
                pos = sim.getObjectPosition(robot, sim.handle_world)
                psi = heading()
                for b in (0, 45, -45, 90, -90, 180):
                    ang = psi + math.radians(b)
                    t = ray_to_edge(pos[0], pos[1], math.cos(ang), math.sin(ang))
                    merge(s, b, max(t - EDGE_OFFSET, 0.0))

            return s

        # ---------- Main loop ----------
        start = sim.getSimulationTime()
        state = "FORWARD"
        state_start = start
        turn_dir = 1              # +1 = left, -1 = right
        turn_h0 = 0.0
        last_turn_end = -INF      # simulation time when the last turn finished
        last_debug = start

        def set_state(new_state, now):
            nonlocal state, state_start, turn_h0
            if new_state != state:
                print(f"[{now - start:6.2f}s] {state} -> {new_state}")
            if new_state == "TURN":
                turn_h0 = heading()
            state = new_state
            state_start = now

        print("\n--- Navigating ---")
        while sim.getSimulationTime() - start < MAX_SIM_TIME:
            now = sim.getSimulationTime()

            # Safety: stop if the robot has dropped off the floor
            if sim.getObjectPosition(robot, sim.handle_world)[2] < z_start - 0.15:
                print("Robot fell off the floor - stopping.")
                break

            s = scan()
            elapsed = now - state_start

            if now - last_debug >= DEBUG_PERIOD:
                last_debug = now
                print(f"[{now - start:6.2f}s] {state:8s} front={s['front']:.2f} "
                      f"left={s['left']:.2f} right={s['right']:.2f} "
                      f"rear={s['rear']:.2f} hits={s['hits']}")

            if state == "FORWARD":
                if s["front"] < STOP_DISTANCE:
                    print(f"Obstacle at {s['front']:.2f} m | "
                          f"left={s['left']:.2f}, right={s['right']:.2f}")

                    # Blocked again right after a turn: keep the same direction
                    # so the robot doesn't flip back and forth.
                    if now - last_turn_end > KEEP_DIRECTION_TIME:
                        turn_dir = 1 if s["left"] >= s["right"] else -1
                    drive(0, 0)

                    if s["front"] < CRITICAL_DISTANCE and s["rear"] > REAR_CLEARANCE:
                        set_state("BACKUP", now)
                    else:
                        set_state("TURN", now)
                else:
                    scale = (s["front"] - STOP_DISTANCE) / (SLOW_DISTANCE - STOP_DISTANCE)
                    speed = FORWARD_SPEED * max(0.3, min(1.0, scale))

                    steer = 0.0
                    if s["side_left"] < SIDE_DISTANCE:
                        steer = -0.4        # steer right
                    elif s["side_right"] < SIDE_DISTANCE:
                        steer = 0.4         # steer left

                    a = speed * (1 - steer * spin_sign)
                    b = speed * (1 + steer * spin_sign)
                    drive(a, b)

            elif state == "BACKUP":
                if s["rear"] < REAR_CLEARANCE or elapsed >= REVERSE_TIME:
                    drive(0, 0)
                    set_state("TURN", now)
                else:
                    drive(REVERSE_SPEED, REVERSE_SPEED)

            elif state == "TURN":
                turned = abs(math.degrees(wrap(heading() - turn_h0)))

                if turned >= TURN_ANGLE:
                    # Turn complete: exactly TURN_ANGLE degrees
                    drive(0, 0)
                    last_turn_end = now
                    set_state("FORWARD", now)
                elif elapsed >= MAX_TURN_TIME:
                    # Safety: turn never completed, try the other way after backing up
                    turn_dir = -turn_dir
                    drive(0, 0)
                    set_state("BACKUP", now)
                else:
                    turn(turn_dir)

            sim.step()

    except KeyboardInterrupt:
        print("Navigation stopped by user.")

    finally:
        try:
            drive(0, 0)
            if sim.getSimulationState() != sim.simulation_stopped:
                sim.stopSimulation()
        except Exception:
            pass

        print("Robot stopped.")
        print("Program finished.")


if __name__ == "__main__":
    main()