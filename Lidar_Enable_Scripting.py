import math as m
import pygame as pg
import time
from pyrplidar import PyRPlidar
import numpy as np
import subprocess
import atexit

# Screen setup
WIDTH, HEIGHT = 800, 800
CENTER = (WIDTH // 2, HEIGHT // 2)
MIN_DISTANCE = 50     # mm (5 cm)
MAX_DISTANCE = 3000   # mm (300 cm)
SCALE = (WIDTH // 2) / MAX_DISTANCE

# Colors
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)
YELLOW = (255, 255, 0)
RED = (255, 0, 0)
DARK_GREEN = (0, 100, 0)
WHITE = (255, 255, 255)

#Global array to store multiple angles scanned from Lidar
points = []

# def turn_on_screen():
	# subprocess.run(['xrandr','--output','DSI-1','--auto','--right-of','HDMI-1'])
	
	# subprocess.run(['gpio','-g','mode','18','pwm'])
	# subprocess.run(['gpio','-g','pwm','18','1023'])
	
# def turn_off_screen():
	# subprocess.run(['xrandr','--output','DSI-1','--off'])
	
	# subprocess.run(['gpio','-g','pwm','18','0'])
#"""" LOW PRIORITY STUFFS """""#



def smoothing_lidar_data(ranges, window_size = 3):
	""" smooth Lidar data reading using average filer
	
	Return np.ndarray: 2D array with same structure as input but smooth distances."""
	
	data = np.array(ranges, dtype=float)
		
	angles = data[:, 0]
	distances = data[:,1]
	pad = window_size//2
	distances = np.where(np.isfinite(distances) & (distances > 0), distances, np.nan)
	padded = np.pad(distances, (pad,pad), mode = 'edge')
	
	smooth_distances = np.array([np.nanmean(padded[i:i+window_size])for i in range(len(distances))])
	
	smoothed = list(zip(angles, smooth_distances))
	return smoothed


def polar_to_cartesian(angle_deg, distance_mm):
    angle_rad = m.radians(angle_deg)
    r = distance_mm * SCALE
    x = CENTER[0] + int(r * m.cos(angle_rad))
    y = CENTER[1] - int(r * m.sin(angle_rad))
    return x, y



def radar_with_circles_and_colors():
    pg.init()
    screen = pg.display.set_mode((WIDTH, HEIGHT))
    pg.display.set_caption("Lidar Radar System")
    clock = pg.time.Clock()

    font_small = pg.font.SysFont(None, 20)
    font_title = pg.font.SysFont(None, 28, bold=True)

    pookie = PyRPlidar()
    pookie.connect(port="/dev/ttyUSB0", baudrate=460800, timeout=3)
    pookie.set_motor_pwm(500)
    time.sleep(2)
    scan_generator = pookie.force_scan()

    running = True
    try:
        prev_angle = None

        for scan in scan_generator():
            for event in pg.event.get():
                if event.type == pg.QUIT:
                    running = False

            if MIN_DISTANCE <= scan.distance <= MAX_DISTANCE:
                points.append((scan.angle, scan.distance))

            # Detect sweep completion
            if prev_angle is not None and scan.angle < prev_angle:
                # Clear screen
                screen.fill(BLACK)

                # Draw reference circles + labels
                for r in range(1000, MAX_DISTANCE + 1, 1000):  # every 50 cm
                    pg.draw.circle(screen, DARK_GREEN, CENTER, int(r * SCALE), 1)
                    label = font_small.render(f"{r//10} cm", True, WHITE)  # mm → cm
                    screen.blit(label, (CENTER[0] + int(r * SCALE) - 25, CENTER[1]))
                    
				#Draw all points with distance-based colors
				# smoothed_data = smoothing_lidar_data(points, window_size=3)
                for ang, dist in smoothing_lidar_data(points):					
                    px, py = polar_to_cartesian(ang, dist)
                    if dist <= 1000:       # 0.05–1.0 m
                        color = RED
                    elif dist <= 2000:     # 1.0–2.0 m
                        color = YELLOW
                    else:                  # 2.0–3.0 m
                        color = GREEN
                    pg.draw.circle(screen, color, (px, py), 2)

                # Draw title text at bottom
                title_surface = font_title.render("Lidar Radar System", True, WHITE)
                screen.blit(title_surface, (WIDTH // 2 - title_surface.get_width() // 2, HEIGHT - 40))

                pg.display.flip()
                clock.tick(60)
                points.clear()

            prev_angle = scan.angle

            if not running:
                break

    except KeyboardInterrupt:
        print("\nStopped")
    finally:
        pookie.stop()
        pookie.set_motor_pwm(0)
        pookie.disconnect()
        pg.quit()






if __name__ == "__main__":
	radar_with_circles_and_colors()
