"""
Live 3D LIDAR Scanner
Real-time 3D point cloud visualization as you scan

Hardware: 2D LIDAR + Manual/Motor rotation
Display: Live updating 3D point cloud
"""

import numpy as np
import time
import csv
import os
from datetime import datetime
from pyrplidar import PyRPlidar
import threading
import queue

# ============================================================================
# Config
# ============================================================================

# LIDAR settings
MIN_DISTANCE = 50      # mm
MAX_DISTANCE = 3000    # mm

# Scanning parameters
SCAN_DURATION = 90.0              # Total scan time in seconds
ROTATION_UPDATE_INTERVAL = 5.0    # Update rotation angle every X seconds
DISPLAY_UPDATE_RATE = 2.0         # Update 3D display every X seconds

# Data directories
CSV_DIR = './CSV'
PLY_DIR = './PLY'

# Visualization settings
VOXEL_SIZE = 10.0  # mm - downsampling for display performance
POINT_SIZE = 2.0   # Display point size

# ============================================================================
# Mathematical functions
# ============================================================================

def convert_to_3d_cartesian(lidar_angle, distance, rotation_angle):
    """Convert 2D LIDAR measurement to 3D coordinates"""
    lidar_rad = np.radians(lidar_angle)
    rotation_rad = np.radians(rotation_angle)
    
    y = distance * np.sin(lidar_rad)
    z_temp = distance * np.cos(lidar_rad)
    
    x = -z_temp * np.sin(rotation_rad)
    z = z_temp * np.cos(rotation_rad)
        
    return x, y, z

def distance_to_color(distance, min_dist=MIN_DISTANCE, max_dist=MAX_DISTANCE):
    """Generate color based on distance (rainbow gradient)"""
    norm = (distance - min_dist) / (max_dist - min_dist)
    norm = max(0.0, min(1.0, norm))
    
    # Rainbow: Blue -> Cyan -> Green -> Yellow -> Red
    if norm < 0.25:
        # Blue to Cyan
        t = norm / 0.25
        r, g, b = 0, t, 1
    elif norm < 0.5:
        # Cyan to Green
        t = (norm - 0.25) / 0.25
        r, g, b = 0, 1, 1 - t
    elif norm < 0.75:
        # Green to Yellow
        t = (norm - 0.5) / 0.25
        r, g, b = t, 1, 0
    else:
        # Yellow to Red
        t = (norm - 0.75) / 0.25
        r, g, b = 1, 1 - t, 0
    
    return [r, g, b]

# ============================================================================
# 3D Scanner
# ============================================================================

class Live3DScanner:
    """
    Real-time 3D LIDAR scanner with live 3D visualization
    """
    
    def __init__(self, port='/dev/ttyUSB0', baudrate=460800):
        """Initialize LIDAR"""
        
        print("\n" + "="*60)
        print("LIVE 3D LIDAR SCANNER")
        print("Real-time 3D Point Cloud Visualization")
        print("="*60)
        
        # Initialize LIDAR
        print(f"\nConnecting to LIDAR on {port}...")
        try:
            self.lidar = PyRPlidar()
            self.lidar.connect(port=port, baudrate=baudrate, timeout=3)
            self.lidar.set_motor_pwm(500)
            time.sleep(2)
            print(f"✓ LIDAR connected at {baudrate} baud")
        except Exception as e:
            print(f"✗ Failed to connect: {e}")
            print("\nTroubleshooting:")
            print(f"1. Check device: ls -l {port}")
            print(f"2. Fix permissions: sudo chmod 666 {port}")
            print("3. Try baudrate: 460800")
            raise
        
        # Check Open3D
        try:
            import open3d as o3d
            self.o3d = o3d
            print("✓ Open3D available for 3D visualization")
        except ImportError:
            print("✗ Open3D not installed!")
            print("\nInstall with: pip3 install open3d --break-system-packages")
            raise
        
        # Scanning state
        self.is_scanning = False
        self.current_rotation_angle = 0.0
        self.start_time = None
        self.last_rotation_update = None
        self.last_display_update = None
        
        # Data storage
        self.points_3d = []       # List of (x, y, z) coordinates
        self.colors_3d = []       # List of [r, g, b] colors
        self.distances = []       # List of distances for coloring
        self.all_scan_data = []   # For CSV export
        
        # Statistics
        self.total_points = 0
        self.rotations_completed = 0
        
        # Thread-safe queue for visualization updates
        self.update_queue = queue.Queue()
        
        print("✓ Scanner initialized\n")
    
    def scan_with_live_3d(self, duration=SCAN_DURATION, rotation_interval=ROTATION_UPDATE_INTERVAL):
        """
        Scan with live 3D visualization
        
        Args:
            duration: Total scan duration in seconds
            rotation_interval: Seconds between rotation angle updates
        """
        
        print("="*60)
        print("STARTING LIVE 3D SCAN")
        print("="*60)
        print(f"\nDuration: {duration} seconds")
        print(f"Rotation update interval: {rotation_interval} seconds")
        print("\nInstructions:")
        print("  - LIDAR will scan continuously")
        print("  - 3D point cloud builds up in real-time")
        print("  - Rotate LIDAR steadily (manual or motor)")
        print("  - Mouse controls in 3D window:")
        print("    * Left drag: Rotate view")
        print("    * Scroll: Zoom")
        print("    * Right drag: Pan")
        print("  - Close 3D window or press Ctrl+C to stop")
        print("="*60 + "\n")
        
        input("Press ENTER to start scanning...")
        
        # Setup CSV file
        os.makedirs(CSV_DIR, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_filename = f"scan3D_live_{timestamp}.csv"
        csv_path = os.path.join(CSV_DIR, csv_filename)
        
        # Initialize timing
        self.is_scanning = True
        self.start_time = time.time()
        self.last_rotation_update = time.time()
        self.last_display_update = time.time()
        
        print(f"🔴 SCANNING IN PROGRESS...")
        print(f"Saving to: {csv_path}\n")
        
        # Start visualization thread
        vis_thread = threading.Thread(
            target=self._visualization_thread,
            args=(duration,),
            daemon=True
        )
        vis_thread.start()
        
        # Start scanning thread
        scan_generator = self.lidar.force_scan()
        prev_angle = None
        
        try:
            with open(csv_path, mode='w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(['Quality', 'Angle', 'Distance', 'Rotation'])
                
                for scan in scan_generator():
                    current_time = time.time()
                    elapsed = current_time - self.start_time
                    
                    # Check if duration exceeded
                    if elapsed > duration:
                        print("\n✓ Scan duration complete!")
                        break
                    
                    # Auto-update rotation angle
                    if current_time - self.last_rotation_update > rotation_interval:
                        progress = elapsed / duration
                        self.current_rotation_angle = progress * 180.0  # 0° to 180°
                        self.last_rotation_update = current_time
                        print(f"Rotation angle: {self.current_rotation_angle:.1f}° ({progress*100:.1f}%)")
                    
                    # Filter and store data
                    if MIN_DISTANCE <= scan.distance <= MAX_DISTANCE:
                        # Convert to 3D
                        x, y, z = convert_to_3d_cartesian(
                            scan.angle,
                            scan.distance,
                            self.current_rotation_angle
                        )
                        
                        # Store point
                        self.points_3d.append([x, y, z])
                        self.distances.append(scan.distance)
                        
                        # Generate color
                        color = distance_to_color(scan.distance)
                        self.colors_3d.append(color)
                        
                        # Write to CSV
                        writer.writerow([
                            15,
                            scan.angle,
                            scan.distance,
                            self.current_rotation_angle
                        ])
                        
                        self.total_points += 1
                    
                    # Detect full LIDAR rotation
                    if prev_angle is not None and scan.angle < prev_angle:
                        self.rotations_completed += 1
                        
                        # Print progress
                        if self.rotations_completed % 5 == 0:
                            print(f"  Points: {self.total_points:,}, Rotations: {self.rotations_completed}")
                        
                        # Flush CSV periodically
                        csvfile.flush()
                    
                    prev_angle = scan.angle
        
        except KeyboardInterrupt:
            print("\n⚠ Scan interrupted by user")
        
        self.is_scanning = False
        
        # Wait for visualization thread to finish
        print("\nWaiting for visualization to close...")
        vis_thread.join(timeout=5)
        
        # Final summary
        elapsed = time.time() - self.start_time
        print("\n" + "="*60)
        print("SCAN COMPLETE")
        print("="*60)
        print(f"Duration: {elapsed:.1f} seconds")
        print(f"Total points: {self.total_points:,}")
        print(f"Rotations: {self.rotations_completed}")
        print(f"CSV saved: {csv_path}")
        print("="*60 + "\n")
        
        # Save final PLY
        print("Saving final high-resolution point cloud...")
        ply_path = self.save_ply(csv_path)
        
        if ply_path:
            print(f"✓ PLY saved: {ply_path}")
            
            # Offer to re-visualize
            visualize = input("\nRe-open final point cloud? (y/n): ").strip().lower()
            if visualize == 'y':
                self.visualize_static_ply(ply_path)
    
    def _visualization_thread(self, duration):
        """
        Thread for real-time 3D visualization
        Runs in parallel with scanning
        """
        
        # Create visualizer
        vis = self.o3d.visualization.VisualizerWithKeyCallback()
        vis.create_window(window_name="Live 3D LIDAR Scan", width=1200, height=800)
        
        # Create point cloud object
        pcd = self.o3d.geometry.PointCloud()
        
        # Add to visualizer
        vis.add_geometry(pcd)
        
        # Configure rendering
        render_option = vis.get_render_option()
        render_option.background_color = np.array([0.1, 0.1, 0.1])  # Dark gray
        render_option.point_size = POINT_SIZE
        render_option.show_coordinate_frame = True
        
        # Set initial camera view
        ctr = vis.get_view_control()
        ctr.set_zoom(0.5)
        
        print("✓ 3D visualization window opened\n")
        
        last_update = time.time()
        
        # Visualization loop
        while self.is_scanning:
            current_time = time.time()
            
            # Update display at regular intervals
            if current_time - last_update > DISPLAY_UPDATE_RATE:
                
                if len(self.points_3d) > 0:
                    # Create point cloud from collected data
                    points_array = np.array(self.points_3d, dtype=np.float64)
                    colors_array = np.array(self.colors_3d, dtype=np.float64)
                    
                    # Downsample for performance
                    if len(points_array) > 50000:
                        # Voxel downsampling
                        pcd_temp = self.o3d.geometry.PointCloud()
                        pcd_temp.points = self.o3d.utility.Vector3dVector(points_array)
                        pcd_temp.colors = self.o3d.utility.Vector3dVector(colors_array)
                        pcd_temp = pcd_temp.voxel_down_sample(voxel_size=VOXEL_SIZE)
                        
                        points_array = np.asarray(pcd_temp.points)
                        colors_array = np.asarray(pcd_temp.colors)
                    
                    # Update point cloud
                    pcd.points = self.o3d.utility.Vector3dVector(points_array)
                    pcd.colors = self.o3d.utility.Vector3dVector(colors_array)
                    
                    # Update visualization
                    vis.update_geometry(pcd)
                
                last_update = current_time
            
            # Update visualizer
            vis.poll_events()
            vis.update_renderer()
            
            # Small sleep to prevent CPU overuse
            time.sleep(0.05)
        
        # Keep window open briefly after scan
        print("\nScan complete. Showing final point cloud...")
        for _ in range(20):  # Show for ~2 seconds
            vis.poll_events()
            vis.update_renderer()
            time.sleep(0.1)
        
        vis.destroy_window()
        print("✓ 3D window closed")
    
    def save_ply(self, csv_path):
        """Save collected data to PLY file"""
        
        os.makedirs(PLY_DIR, exist_ok=True)
        csv_filename = os.path.splitext(os.path.basename(csv_path))[0]
        ply_path = os.path.join(PLY_DIR, f"{csv_filename}.ply")
        
        if not self.points_3d:
            print("Error: No points to save!")
            return None
        
        # Create point cloud
        pcd = self.o3d.geometry.PointCloud()
        pcd.points = self.o3d.utility.Vector3dVector(np.array(self.points_3d))
        pcd.colors = self.o3d.utility.Vector3dVector(np.array(self.colors_3d))
        
        # Save
        self.o3d.io.write_point_cloud(ply_path, pcd)
        
        return ply_path
    
    def visualize_static_ply(self, ply_path):
        """Visualize saved PLY file"""
        
        print(f"\nLoading point cloud...")
        pcd = self.o3d.io.read_point_cloud(ply_path)
        
        print(f"  {len(pcd.points):,} points loaded")
        
        # Optional: downsample for smoother viewing
        if len(pcd.points) > 100000:
            print("  Applying voxel downsampling...")
            pcd = pcd.voxel_down_sample(voxel_size=VOXEL_SIZE)
            print(f"  {len(pcd.points):,} points after downsampling")
        
        print("\nOpening 3D viewer...")
        print("Controls:")
        print("  - Left drag: Rotate")
        print("  - Scroll: Zoom")
        print("  - Right drag: Pan")
        print("  - Press 'Q' to close\n")
        
        self.o3d.visualization.draw_geometries(
            [pcd],
            window_name="3D LIDAR Scan - Final",
            width=1200,
            height=800,
            point_show_normal=False
        )
    
    def cleanup(self):
        """Cleanup resources"""
        print("\nCleaning up...")
        try:
            self.lidar.stop()
            self.lidar.set_motor_pwm(0)
            self.lidar.disconnect()
        except:
            pass
        print("✓ Cleanup complete")

# ============================================================================
# MAIN
# ============================================================================

def main():
    import argparse
    
    parser = argparse.ArgumentParser(
        description='Live 3D LIDAR Scanner with Real-time Visualization',
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    
    parser.add_argument('--duration', type=float, default=90.0,
                       help='Scan duration in seconds (default: 90)')
    parser.add_argument('--interval', type=float, default=5.0,
                       help='Rotation update interval in seconds (default: 5)')
    parser.add_argument('--port', type=str, default='/dev/ttyUSB0',
                       help='LIDAR serial port (default: /dev/ttyUSB0)')
    parser.add_argument('--baudrate', type=int, default=460800,
                       help='LIDAR baudrate (default: 460800)')
    
    args = parser.parse_args()
    
    try:
        # Create scanner
        scanner = Live3DScanner(port=args.port, baudrate=args.baudrate)
        
        try:
            # Start scanning with live 3D display
            scanner.scan_with_live_3d(
                duration=args.duration,
                rotation_interval=args.interval
            )
        finally:
            scanner.cleanup()
    
    except KeyboardInterrupt:
        print("\n\nProgram interrupted")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()