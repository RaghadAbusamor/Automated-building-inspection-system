# launch_parallel.py
import subprocess
import time

if __name__ == "__main__":
    script1_process = subprocess.Popen(["python", "/home/raghad/turtlebot2_ws/src/test1.py"])

    # Wait for 30 seconds
    time.sleep(10)

    script2_process = subprocess.Popen(["python", "/home/raghad/turtlebot2_ws/src/edgesGoal.py"])

    # Wait for both processes to finish
    script1_process.wait()
    script2_process.wait()

