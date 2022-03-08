import subprocess
import time

def robot_is_moving() -> bool:
    try:
        vels = subprocess.check_output(
            ["rostopic", "echo", "-n", "1", "-p", "/cmd_vel"],
            timeout=1
        )
    except subprocess.TimeoutExpired:
        return False
    vels = vels.decode()
    vels = dict(zip(*(row.split(",") for row in vels.strip().splitlines())))
    return not (
        vels["field.linear.x"] == "0.0"
        and vels["field.linear.y"] == "0.0"
        and vels["field.linear.z"] == "0.0"
        and vels["field.angular.x"] == "0.0"
        and vels["field.angular.y"] == "0.0"
        and vels["field.angular.z"] == "0.0"
    )

while not robot_is_moving():
    print("waiting for initial movement...")

WAIT_FOR_S = 10.0

t0 = time.perf_counter()
while (time.perf_counter() - t0) < WAIT_FOR_S:
    if robot_is_moving():
        t0 = time.perf_counter()
    print(f"stopped for {round(time.perf_counter() - t0, 2)} s out of {WAIT_FOR_S} s")
