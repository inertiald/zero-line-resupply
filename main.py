import rtde_receive
import rtde_control
import robotiq_gripper
import time
import math

# --- CONFIGURATION ---
ROBOT_IP = "192.168.1.100"
IS_FRIENDLY = False  # Default to locked
CAMERA_IN_BASE_POSE = [0.35, -0.10, 0.55, 0.0, 3.14, 0.0]
PRE_GRASP_OFFSET = [0.0, 0.0, -0.10, 0.0, 0.0, 0.0]
SERVO_LOOKAHEAD_TIME = 0.1
SERVO_GAIN = 300
SERVO_FREQUENCY_HZ = 500.0
GRASP_FORCE = 50
GRIPPER_TIMEOUT_S = 3.0
GRIPPER_POLL_INTERVAL_S = 0.02
CONTACT_APPROACH_VECTOR = [0.0, 0.0, -0.02, 0.0, 0.0, 0.0]
CONTACT_DIRECTION = [0.0, 0.0, -1.0, 0.0, 0.0, 0.0]
CONTACT_ACCELERATION = 0.2


def main():
    global IS_FRIENDLY

    print(f"--- Attempting to connect to {ROBOT_IP} ---")
    try:
        # rtde_receive: For reading data (sensors, position)
        r_inter = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
        # rtde_control: For sending moves (this uploads a script to the robot)
        c_inter = rtde_control.RTDEControlInterface(ROBOT_IP)
        print("✅ CONNECTED to UR3e")

        # Connect to Robotiq Gripper
        print("Attempting to connect to Robotiq Gripper...")
        gripper = robotiq_gripper.RobotiqGripper()
        gripper.connect(ROBOT_IP, 63352)
        print("✅ CONNECTED to Robotiq Gripper")

        # Activate gripper if it isn't already
        if not gripper.is_active():
            print("Activating gripper...")
            gripper.activate()

    except Exception as e:
        print(f"❌ CONNECTION FAILED: {e}")
        print("Check Ethernet IP settings on both Mac and Robot.")
        return

    while True:
        print("\n--- MACBOOK LAB CONTROLLER ---")
        print(f"STATUS: {'🔓 FRIENDLY (Unlocked)' if IS_FRIENDLY else '🔒 UNFRIENDLY (Locked)'}")
        print("1. Toggle Authentication (Simulate BLE)")
        print("2. Measure Workspace (Record TCP Coordinates)")
        print("3. Execute DUMMY MOVEMENT (Safe Z-hop)")
        print("4. Execute Gripper Control")
        print("5. Execute vision-guided grasp")
        print("6. Exit")

        choice = input("Select option: ")

        if choice == '1':
            IS_FRIENDLY = not IS_FRIENDLY
            print(f"Auth state switched to: {IS_FRIENDLY}")

        elif choice == '2':
            measure_workspace(r_inter)

        elif choice == '3':
            if IS_FRIENDLY:
                execute_dummy_move(c_inter, r_inter)
            else:
                print("⛔️ ACCESS DENIED. Toggle Auth first.")

        elif choice == '4':
            print("Closing gripper...")
            gripper.move_and_wait_for_pos(255, 255, 255)  # Full close, max speed, max force
            print("Gripper closed.")

            time.sleep(1.0)

            print("Opening gripper...")
            gripper.move_and_wait_for_pos(0, 255, 255)  # Full open, max speed, max force
            print("Gripper opened.")

        elif choice == '5':
            if IS_FRIENDLY:
                execute_vision_guided_grasp(c_inter, r_inter, gripper)
            else:
                print("⛔️ ACCESS DENIED. Toggle Auth first.")

        elif choice == '6':
            c_inter.stopScript()
            gripper.disconnect()
            break


def measure_workspace(r_inter):
    """
    Helps you map the physical world for Isaac Sim.
    Manually move the robot to corners of your table/drone prop
    and hit Enter to log the coordinates.
    """
    print("\n--- WORKSPACE MAPPING MODE ---")
    print("1. Press 'Free Drive' button on back of Teach Pendant.")
    print("2. Move robot to a point of interest (e.g., table corner, drone drum center).")
    print("3. Press Enter here to log coordinates.")
    print("4. Type 'q' to return to menu.")

    while True:
        user_in = input("Press Enter to log point (or 'q' to quit): ")
        if user_in.lower() == 'q':
            break

        # Get actual TCP pose [x, y, z, rx, ry, rz]
        pose = r_inter.getActualTCPPose()
        # Get Joint angles (good for setting 'Home' in Isaac)
        joints = r_inter.getActualQ()

        print(f"📍 POSE (meters/rads): {pose}")
        print(f"🤖 JOINTS (rads):      {joints}")
        print("-" * 30)

def check_robot_safety(r_inter):
    """Checks if the robot is in a state capable of movement."""
    # 1 = Normal, 2 = Reduced, 3 = Protective Stop, 4 = Recovery, etc.
    safety_mode = r_inter.getSafetyMode()
    # 0 = Robot State Power Off, 3 = Idle, 5 = Running, 7 = Updating Firmware
    robot_mode = r_inter.getRobotMode()

    print(f"Safety Mode: {safety_mode} | Robot Mode: {robot_mode}")

    if safety_mode != 1:
        print("⚠️ Robot is NOT in Normal mode. Check the Teach Pendant for popups.")
        return False
    if robot_mode != 7: # 7 is usually 'Power On' for e-Series
        print("⚠️ Robot motors may not be enabled.")
        return False
    return True

def execute_dummy_move(c_inter, r_inter):

    if not check_robot_safety(r_inter):
        print("Aborting move due to safety state.")
        return

    start_pose = r_inter.getActualTCPPose()
    target_pose = start_pose[:]
    target_pose[2] += 0.05  # CORRECTED INDEX TO 2 (Z-axis)

    # Use slightly higher acceleration to break static friction,
    # but keep speed low for safety.
    speed = 0.1
    accel = 1.2  # Standard UR default is often 1.2

    print(f"Moving from {start_pose[2]} to {target_pose[2]}")
    c_inter.moveL(target_pose, speed, accel)


def parse_pose(user_input):
    values = [float(v.strip()) for v in user_input.split(",")]
    if len(values) != 6:
        raise ValueError("Pose must include 6 comma-separated values: x,y,z,rx,ry,rz")
    return values


def camera_pose_to_base(c_inter, object_pose_camera):
    return c_inter.poseTrans(CAMERA_IN_BASE_POSE, object_pose_camera)


def compute_pre_grasp(c_inter, object_pose_base):
    return c_inter.poseTrans(object_pose_base, PRE_GRASP_OFFSET)


def run_servo_tracking(c_inter, target_pose_fn, duration_s=0.5):
    dt = 1.0 / SERVO_FREQUENCY_HZ
    cycles = max(1, int(duration_s / dt))

    for _ in range(cycles):
        t_start = c_inter.initPeriod()
        target_pose = target_pose_fn()
        c_inter.servoL(target_pose, 0.0, 0.0, dt, SERVO_LOOKAHEAD_TIME, SERVO_GAIN)
        c_inter.waitPeriod(t_start)

    c_inter.servoStop()


def move_until_contact(c_inter):
    try:
        c_inter.moveUntilContact(CONTACT_APPROACH_VECTOR, CONTACT_DIRECTION, CONTACT_ACCELERATION)
    except TypeError:
        c_inter.moveUntilContact(CONTACT_APPROACH_VECTOR)


def close_gripper_with_feedback(gripper):
    ok, _ = gripper.move(255, speed=255, force=GRASP_FORCE)
    if not ok:
        return False

    t0 = time.time()
    while time.time() - t0 < GRIPPER_TIMEOUT_S:
        status = gripper.objectDetectionStatus()
        if status != robotiq_gripper.RobotiqGripper.ObjectStatus.MOVING:
            return status == robotiq_gripper.RobotiqGripper.STOPPED_INNER_OBJECT
        time.sleep(GRIPPER_POLL_INTERVAL_S)
    return False


def execute_vision_guided_grasp(c_inter, r_inter, gripper):
    if not check_robot_safety(r_inter):
        print("Aborting move due to safety state.")
        return

    raw_pose = input("Enter object pose in camera frame (x,y,z,rx,ry,rz), or press Enter for demo pose: ").strip()
    object_pose_camera = parse_pose(raw_pose) if raw_pose else [0.0, 0.0, 0.25, 0.0, 0.0, 0.0]

    object_pose_base = camera_pose_to_base(c_inter, object_pose_camera)
    pre_grasp_pose = compute_pre_grasp(c_inter, object_pose_base)
    print(f"Object pose in base frame: {object_pose_base}")
    print(f"Pre-grasp pose: {pre_grasp_pose}")

    run_servo_tracking(c_inter, lambda: pre_grasp_pose, duration_s=0.5)
    move_until_contact(c_inter)

    if close_gripper_with_feedback(gripper):
        print("Object successfully grasped!")
    else:
        print("Grasp failed or timed out.")

# def execute_dummy_move(c_inter, r_inter):
#     """
#     Performs a relative movement.
#     It reads where the robot is NOW, and moves 5cm UP, then back DOWN.
#     This prevents the robot from flying into a wall using absolute coordinates.
#     """
#     print("\n⚠️ WARNING: Robot is about to move.")
#     print("Keep hand near E-STOP.")
#     confirm = input("Type 'y' to confirm execution: ")
#
#     if confirm.lower() != 'y':
#         print("Aborted.")
#         return
#
#     # 1. Get current position
#     start_pose = r_inter.getActualTCPPose()
#     # PRINT FOR DEBUGGING (WORLD SPACE POSITIONS, ROTIONXYZ)
#
#     # 2. Create a waypoint 5cm (0.05m) HIGHER in Z
#     # Pose format: [x, y, z, rx, ry, rz]
#     target_pose = start_pose[:]  # Copy list
#     target_pose[2] += 0.05  # Add 5cm to Z axis
#
#     speed = 0.5  # Low speed for safety (m/s)
#     accel = 0.5  # Reduced acceleration for smoother short moves
#
#     print("🚀 Moving UP 5cm...")
#     # moveL expects arguments in the order: pose, speed, acceleration
#     c_inter.moveJ_IK(target_pose, speed, accel)
#
#     time.sleep(1)
#
#     print("⬇️ Moving DOWN to start...")
#     c_inter.moveL(start_pose, speed, accel)
#
#     print("✅ Move complete.")




if __name__ == "__main__":
    main()

# rqsafe
