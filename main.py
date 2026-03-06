import rtde_receive
import rtde_control
import robotiq_gripper
import time
import math

# --- CONFIGURATION ---
ROBOT_IP = "192.168.1.100"
IS_FRIENDLY = False  # Default to locked
SERVO_FREQUENCY = 500  # Hz for visual servoing loop
SERVO_DT = 1.0 / SERVO_FREQUENCY
SERVO_LOOKAHEAD = 0.1
SERVO_GAIN = 300


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
        print("5. Compute Approach Pose (Camera to Robot Transform)")
        print("6. Visual Servoing (Smooth Dynamic Tracking)")
        print("7. Safe Grasp (moveUntilContact + Gripper)")
        print("8. Exit")

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
                compute_approach_pose(c_inter, r_inter)
            else:
                print("⛔️ ACCESS DENIED. Toggle Auth first.")

        elif choice == '6':
            if IS_FRIENDLY:
                visual_servo_tracking(c_inter, r_inter)
            else:
                print("⛔️ ACCESS DENIED. Toggle Auth first.")

        elif choice == '7':
            if IS_FRIENDLY:
                safe_grasp(c_inter, r_inter, gripper)
            else:
                print("⛔️ ACCESS DENIED. Toggle Auth first.")

        elif choice == '8':
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



def compute_approach_pose(c_inter, r_inter):
    """
    Demonstrates coordinate transformation from camera frame to robot base frame
    using poseTrans, and computes a pre-grasp (approach) pose offset from the object.

    In production, the object_pose_camera would come from Nvidia Foundation Pose.
    Here we simulate it using the current TCP pose for demonstration.
    """
    if not check_robot_safety(r_inter):
        print("Aborting due to safety state.")
        return

    print("\n--- COORDINATE TRANSFORMATION (Camera → Robot) ---")

    # In production: camera_to_robot is your extrinsic calibration matrix as a pose.
    # Here we use the current TCP pose as a placeholder for demonstration.
    # Replace with your actual camera-to-robot calibration pose.
    camera_to_robot = r_inter.getActualTCPPose()
    print(f"Camera-to-Robot base pose: {camera_to_robot}")

    # Simulated object pose from Nvidia Foundation Pose (relative to camera)
    # In production, replace this with the real 6D pose from the vision system
    object_pose_camera = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    print(f"Object pose (camera frame): {object_pose_camera}")

    # Transform object pose from camera frame to robot base frame
    object_pose_robot = c_inter.poseTrans(camera_to_robot, object_pose_camera)
    print(f"Object pose (robot frame):  {object_pose_robot}")

    # Compute a pre-grasp "approach" pose: offset -10cm along the object's local Z-axis
    # This positions the TCP before the object for a safe approach
    approach_offset = [0.0, 0.0, -0.10, 0.0, 0.0, 0.0]
    approach_pose = c_inter.poseTrans(object_pose_robot, approach_offset)
    print(f"Approach pose (10cm above): {approach_pose}")

    confirm = input("Move to approach pose? (y/n): ")
    if confirm.lower() == 'y':
        speed = 0.1
        accel = 1.2
        print("🚀 Moving to approach pose...")
        c_inter.moveL(approach_pose, speed, accel)
        print("✅ At approach pose.")


def visual_servo_tracking(c_inter, r_inter):
    """
    Smooth dynamic tracking using servoL in a 500Hz control loop.

    Uses initPeriod/waitPeriod for precise timing to minimize network jitter.
    In production, the target_pose would be continuously updated from the
    Nvidia Foundation Pose vision pipeline via the edge controller.
    """
    if not check_robot_safety(r_inter):
        print("Aborting due to safety state.")
        return

    print("\n--- VISUAL SERVOING (500Hz Tracking Loop) ---")
    print("The robot will hold its current position using servoL.")
    print("In production, replace the target with live vision data.")
    print("Press Ctrl+C to stop tracking.\n")

    target_pose = r_inter.getActualTCPPose()

    try:
        while True:
            t_start = c_inter.initPeriod()

            # In production: get latest 6D pose from Nvidia Foundation Pose
            # and convert to robot frame using poseTrans here.
            # target_pose = get_latest_vision_target(c_inter, r_inter)

            # Send updated target to the robot
            c_inter.servoL(target_pose, 0.0, 0.0, SERVO_DT, SERVO_LOOKAHEAD, SERVO_GAIN)

            # Sleep exactly enough to maintain 500Hz
            c_inter.waitPeriod(t_start)
    except KeyboardInterrupt:
        print("\n⏹ Servo tracking stopped.")
    finally:
        c_inter.servoStop()
        print("✅ ServoL stopped cleanly.")


def safe_grasp(c_inter, r_inter, gripper):
    """
    Performs a safe grasp sequence:
    1. Hovers above the current position (approach pose via poseTrans)
    2. Uses moveUntilContact to slowly descend until contact is detected
    3. Closes the gripper and checks if the object was successfully grasped
    """
    if not check_robot_safety(r_inter):
        print("Aborting due to safety state.")
        return

    print("\n--- SAFE GRASP SEQUENCE ---")
    print("The robot will descend until contact, then close the gripper.")

    # Step 1: Record current position as the approach pose
    approach_pose = r_inter.getActualTCPPose()
    print(f"Approach pose: {approach_pose}")

    # Step 2: Use moveUntilContact to slowly descend toward the object
    # xd = tool speed vector [vx, vy, vz, wx, wy, wz] in base frame
    # Move downward (negative Z) at 0.02 m/s
    speed_vector = [0.0, 0.0, -0.02, 0.0, 0.0, 0.0]
    # direction = wrench to watch for contact (force in Z)
    contact_direction = [0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
    accel = 0.5

    print("⬇️ Descending until contact...")
    c_inter.moveUntilContact(speed_vector, contact_direction, accel)
    print("✅ Contact detected!")

    contact_pose = r_inter.getActualTCPPose()
    print(f"Contact pose: {contact_pose}")

    # Step 3: Close gripper and check grasp status
    print("🤏 Closing gripper...")
    gripper.move_and_wait_for_pos(255, 255, 50)  # Full close, max speed, moderate force
    status = gripper.object_detection_status()
    if status == robotiq_gripper.RobotiqGripper.ObjectStatus.STOPPED_INNER_OBJECT:
        print("✅ Object successfully grasped!")
    elif status == robotiq_gripper.RobotiqGripper.ObjectStatus.STOPPED_OUTER_OBJECT:
        print("⚠️ Object detected (outer grip). Check grasp quality.")
    elif status == robotiq_gripper.RobotiqGripper.ObjectStatus.AT_DEST:
        print("❌ Gripper closed fully without detecting an object.")
    else:
        print("❌ Grasp status unknown. Gripper may still be moving.")

    # Step 4: Retract to approach pose
    print("⬆️ Retracting to approach pose...")
    c_inter.moveL(approach_pose, 0.1, 1.2)
    print("✅ Safe grasp sequence complete.")


if __name__ == "__main__":
    main()

# rqsafe