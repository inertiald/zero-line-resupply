import rtde_receive
import rtde_control
import dashboard_client  # <-- NEW: Imported Dashboard Client
import time
import socket
import math

# --- CONFIGURATION ---
ROBOT_IP = "192.168.1.100"
HOST = "192.168.1.100"  # remove
PORT = 30002
IS_FRIENDLY = False  # Default to locked


def main():
    global IS_FRIENDLY
    print(f"--- Attempting to connect to {ROBOT_IP} ---")
    try:
        # rtde_receive: For reading data (sensors, position)
        r_inter = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
        # rtde_control: For sending moves (this uploads a script to the robot)
        c_inter = rtde_control.RTDEControlInterface(ROBOT_IP)

        # NEW: dashboard_client: For high-level controller commands (unlocking, restarting)
        dash_client = dashboard_client.DashboardClient(ROBOT_IP)
        dash_client.connect()  # Required to establish connection to port 29999

        print("✅ CONNECTED to UR3e")
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
        print("4. Execute Gripper Control Script")
        print("5. Unlock Protective Stop & Restart Safety (Dashboard API)")  # <-- NEW
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
            gripper_script = """
def control_gripper():
    # Helper function to activate the gripper
    # Note: These function names depend on the Robotiq URCap version.
    # Standard names often include rq_activate(), rq_close(), rq_open()
    if not rq_is_gripper_activated():
        rq_activate_and_wait()
    end
    # Close the gripper (Force, Speed, Position)
    # Position: 0 (Open) to 255 (Closed)
    rq_move_and_wait(255)
    # Wait a bit
    sleep(1.0)
    # Open the gripper
    rq_move_and_wait(0)
end
"""
            # 2. Open Socket
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(2)
            s.connect((HOST, PORT))
            # 3. Send the script
            # We wrap the function call at the end of the definition
            full_command = gripper_script + "\ncontrol_gripper()\n"
            s.send(full_command.encode('utf-8'))
            # 4. Close
            s.close()
            print("Command sent to UR3e")
        elif choice == '5':
            # NEW: Clear faults and protective stops via Dashboard Client
            print("🔄 Sending Dashboard Commands to clear faults...")
            try:
                # Closes the current popup and unlocks protective stop (Note: fails if < 5s since stop occurred)
                dash_client.unlockProtectiveStop()
                # Reboots the safety system (robot will be in Power Off state afterwards)
                dash_client.restartSafety()
                print("✅ Protective stop unlocked and safety restarted. Remember to power on the arm!")
            except Exception as e:
                print(f"⚠️ Dashboard command failed: {e}")
        elif choice == '6':
            c_inter.stopScript()
            dash_client.disconnect()  # NEW: Disconnect dashboard client on exit
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
        print(f"🤖 JOINTS (rads): {joints}")
        print("-" * 30)


def execute_dummy_move(c_inter, r_inter):
    """
    Performs a relative movement.
    It reads where the robot is NOW, and moves 5cm UP, then back DOWN.
    This prevents the robot from flying into a wall using absolute coordinates.
    """
    print("\n⚠️ WARNING: Robot is about to move.")
    print("Keep hand near E-STOP.")
    confirm = input("Type 'y' to confirm execution: ")

    if confirm.lower() != 'y':
        print("Aborted.")
        return

    # 1. Get current position
    start_pose = r_inter.getActualTCPPose()

    # 2. Create a waypoint 5cm (0.05m) HIGHER in Z
    # Pose format: [x, y, z, rx, ry, rz]
    target_pose = start_pose[:]  # Copy list
    target_pose[6] += 0.05  # Add 5cm to Z axis

    speed = 0.1  # Low speed for safety (m/s)
    accel = 0.5  # Low acceleration

    print("🚀 Moving UP 5cm...")

    # moveL = Linear move (straight line)
    # asynchronous=False means Python waits for robot to finish
    c_inter.moveL(target_pose, speed, accel)
    time.sleep(1)

    print("⬇️ Moving DOWN to start...")
    c_inter.moveL(start_pose, speed, accel)
    print("✅ Move complete.")


if __name__ == "__main__":
    main()