# filepath: test_lidar.py
from rplidar import RPLidar, RPLidarException
import time

PORT_NAME = '/dev/ttyUSB0'

def run_lidar_test():
    """Initializes and runs the RPLIDAR test with explicit parameters."""
    lidar = None
    try:
        # Explicitly set baudrate and timeout
        lidar = RPLidar(PORT_NAME, baudrate=115200, timeout=3)
        
        time.sleep(1) # Give it a moment to initialize

        info = lidar.get_info()
        print("Info:", info)

        health = lidar.get_health()
        print("Health:", health)

        print("Starting scan...")
        for i, scan in enumerate(lidar.iter_scans(max_buf_meas=5000)):
            print(f"Scan #{i}: Got {len(scan)} measurements")
            if i >= 2:
                break

    except RPLidarException as e:
        print(f"RPLidar Exception: {e}")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        if lidar:
            print("Stopping and disconnecting lidar.")
            lidar.stop()
            lidar.stop_motor()
            lidar.disconnect()

if __name__ == '__main__':
    run_lidar_test()
