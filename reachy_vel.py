import reachy2_sdk
import time

"""
NAVIGATION VELOCITY LIMITS:
- Linear: min 0.2 [m/s], max 0.5 [m/s]
- Angular: min 0.7 [rad/s], max 1.1 [rad/s]

Based on the maximum linear velocity (case in which I have more acceleration 
and consequently more stopping delay), I set:
- safety slowdown distance to 0.65 [m]
- safety critical distance to 0.5 [m]
"""

def main():
    reachy = reachy2_sdk.ReachySDK(host="192.168.0.121")
    reachy.turn_on()
    reachy.mobile_base.reset_odometry()
    #reachy.goto_posture("default", wait = True)

    reachy.mobile_base.lidar.safety_enabled = True
    reachy.mobile_base.lidar.safety_slowdown_distance = 0.65
    reachy.mobile_base.lidar.safety_critical_distance = 0.5
    reachy.mobile_base.set_goal_speed(vx=0.0, vy=0.0, vtheta=(1.1 * 180 / 3.14))
    start_time = time.time()
    i = 0
    j = 0

    while time.time() - start_time < 2:
        reachy.mobile_base.send_speed_command()
        i += 1
        if i == 20:
            j += 1
            i = 0
            print(f"Odometry update - iteration {j}:")
            print(reachy.mobile_base.get_current_odometry())
            print("\n")

    reachy.mobile_base.lidar.get_map()

if __name__ == "__main__":
    main()