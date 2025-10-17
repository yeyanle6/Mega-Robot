#!/usr/bin/env python3
"""
Export RTAB-Map database to various formats
Supports: 2D occupancy grid (PGM), 3D point cloud (PCD/PLY), and pose graph
"""

import os
import sys
import argparse
import subprocess
from pathlib import Path
from datetime import datetime


class RTABMapExporter:
    """Export RTAB-Map database to different formats"""

    def __init__(self, database_path):
        self.database_path = os.path.abspath(database_path)
        if not os.path.exists(self.database_path):
            raise FileNotFoundError(f"Database not found: {self.database_path}")

        self.db_size_mb = os.path.getsize(self.database_path) / (1024 * 1024)
        print(f"Database: {self.database_path}")
        print(f"Size: {self.db_size_mb:.2f} MB")

    def export_2d_map(self, output_dir, grid_resolution=0.05):
        """
        Export 2D occupancy grid map (PGM format for Nav2)

        Note: rtabmap-export doesn't directly support 2D grid export.
        Use ROS 2 map_saver_cli to save the /map topic while RTAB-Map is running:
            ros2 run nav2_map_server map_saver_cli -f map_name

        Args:
            output_dir: Output directory for map files
            grid_resolution: Grid cell size in meters (default: 0.05 = 5cm)
        """
        output_dir = os.path.abspath(output_dir)
        os.makedirs(output_dir, exist_ok=True)

        print(f"\n[Export 2D Map]")
        print(f"  Output directory: {output_dir}")
        print(f"  ⚠️  2D map export from database is not supported by rtabmap-export")
        print(f"")
        print(f"  Alternative method:")
        print(f"  1. Make sure RTAB-Map is running and publishing /map topic")
        print(f"  2. Install nav2_map_server: sudo apt install ros-humble-nav2-map-server")
        print(f"  3. Save map with: ros2 run nav2_map_server map_saver_cli -f {output_dir}/map")
        print(f"")
        print(f"  Or subscribe to /map topic and convert to PGM manually")

        return False

    def export_3d_cloud(self, output_file, format='pcd', assemble=True,
                       voxel_size=0.01, max_range=0.0):
        """
        Export 3D point cloud

        Args:
            output_file: Output file path (.pcd or .ply)
            format: Output format ('pcd' or 'ply')
            assemble: Assemble all clouds into one (default: True)
            voxel_size: Voxel filter size in meters (0 = no filter)
            max_range: Maximum point range in meters (0 = unlimited)
        """
        output_file = os.path.abspath(output_file)
        os.makedirs(os.path.dirname(output_file), exist_ok=True)

        print(f"\n[Export 3D Point Cloud]")
        print(f"  Output file: {output_file}")
        print(f"  Format: {format.upper()}")
        print(f"  Voxel size: {voxel_size}m")
        if max_range > 0:
            print(f"  Max range: {max_range}m")

        # Build rtabmap-export command (new version requires explicit --cloud)
        # Use --scan for LiDAR data instead of depth/stereo
        cmd = ['rtabmap-export', '--cloud', '--scan']

        # Add point cloud export options
        if voxel_size > 0:
            cmd.append(f'--voxel={voxel_size}')

        if max_range > 0:
            cmd.append(f'--max_range={max_range}')

        # Output format
        if format.lower() == 'ply':
            cmd.append('--ply')

        # Output file (remove extension as rtabmap-export adds it)
        output_base = os.path.splitext(output_file)[0]
        output_dir = os.path.dirname(output_file)
        output_name = os.path.basename(output_base)

        cmd.extend(['--output_dir', output_dir, '--output', output_name, self.database_path])

        try:
            result = subprocess.run(cmd, check=True, capture_output=True, text=True)

            # rtabmap-export adds extension and _cloud suffix automatically
            possible_files = [
                output_file,
                f"{output_base}.pcd",
                f"{output_base}.ply",
                f"{output_base}_cloud.pcd",
                f"{output_base}_cloud.ply",
                os.path.join(output_dir, f"{output_name}_cloud.pcd"),
                os.path.join(output_dir, f"{output_name}_cloud.ply")
            ]

            exported_file = None
            for f in possible_files:
                if os.path.exists(f):
                    exported_file = f
                    break

            if exported_file:
                file_size_mb = os.path.getsize(exported_file) / (1024 * 1024)
                print(f"  ✓ Point cloud exported successfully")
                print(f"    - {exported_file}")
                print(f"    - Size: {file_size_mb:.2f} MB")
                return True
            else:
                print(f"  ✗ Export failed: Output file not found")
                if result.stdout:
                    print(f"  stdout: {result.stdout}")
                if result.stderr:
                    print(f"  stderr: {result.stderr}")
                return False

        except subprocess.CalledProcessError as e:
            print(f"  ✗ Export failed: {e}")
            if e.stdout:
                print(f"  stdout: {e.stdout}")
            if e.stderr:
                print(f"  stderr: {e.stderr}")
            return False

    def export_poses(self, output_file, format='tum'):
        """
        Export robot trajectory/poses

        Args:
            output_file: Output file path
            format: Output format ('tum', 'kitti', 'g2o')
        """
        output_file = os.path.abspath(output_file)
        os.makedirs(os.path.dirname(output_file), exist_ok=True)

        print(f"\n[Export Trajectory]")
        print(f"  Output file: {output_file}")
        print(f"  Format: {format.upper()}")

        # Build command based on format
        # New version uses --poses with format specified separately
        cmd = ['rtabmap-export', '--poses']

        # Output file (remove extension as rtabmap-export may add it)
        output_base = os.path.splitext(output_file)[0]
        output_dir = os.path.dirname(output_file)
        output_name = os.path.basename(output_base)

        # Add format-specific options if needed
        if format.lower() == 'g2o':
            cmd.append('--g2o')
        # tum and kitti are handled by --poses automatically

        cmd.extend(['--output_dir', output_dir, '--output', output_name, self.database_path])

        try:
            result = subprocess.run(cmd, check=True, capture_output=True, text=True)

            # Check for various possible output file names
            possible_files = [
                output_file,
                f"{output_base}.txt",
                f"{output_base}_poses.txt",
                f"{output_base}.g2o",
                os.path.join(output_dir, f"{output_name}_poses.txt"),
                os.path.join(output_dir, f"{output_name}.g2o"),
                os.path.join(output_dir, f"{output_name}.txt")
            ]

            exported_file = None
            for f in possible_files:
                if os.path.exists(f):
                    exported_file = f
                    break

            if exported_file:
                print(f"  ✓ Trajectory exported successfully")
                print(f"    - {exported_file}")
                return True
            else:
                print(f"  ✗ Export failed: Output file not found")
                print(f"  Checked: {', '.join(possible_files)}")
                if result.stdout:
                    print(f"  stdout: {result.stdout}")
                if result.stderr:
                    print(f"  stderr: {result.stderr}")
                return False

        except subprocess.CalledProcessError as e:
            print(f"  ✗ Export failed: {e}")
            if e.stdout:
                print(f"  stdout: {e.stdout}")
            if e.stderr:
                print(f"  stderr: {e.stderr}")
            return False

    def export_all(self, output_dir, map_name=None):
        """
        Export all available formats

        Args:
            output_dir: Base output directory
            map_name: Optional map name (uses timestamp if None)
        """
        if map_name is None:
            map_name = datetime.now().strftime("%Y%m%d_%H%M%S")

        output_dir = os.path.abspath(output_dir)
        map_dir = os.path.join(output_dir, map_name)
        os.makedirs(map_dir, exist_ok=True)

        print(f"\n{'='*60}")
        print(f"Exporting map: {map_name}")
        print(f"Output directory: {map_dir}")
        print(f"{'='*60}")

        results = {}

        # Export 2D occupancy grid
        results['2d_map'] = self.export_2d_map(
            os.path.join(map_dir, '2d_map'),
            grid_resolution=0.05
        )

        # Export 3D point cloud (PCD)
        results['3d_pcd'] = self.export_3d_cloud(
            os.path.join(map_dir, '3d_cloud', 'map.pcd'),
            format='pcd',
            voxel_size=0.01
        )

        # Export trajectory (TUM format)
        results['trajectory'] = self.export_poses(
            os.path.join(map_dir, 'trajectory.txt'),
            format='tum'
        )

        # Print summary
        print(f"\n{'='*60}")
        print("Export Summary:")
        for key, success in results.items():
            status = "✓" if success else "✗"
            print(f"  {status} {key}")
        print(f"{'='*60}")

        return all(results.values())


def main():
    parser = argparse.ArgumentParser(
        description='Export RTAB-Map database to various formats',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Export all formats to a directory
  %(prog)s ~/.ros/rtabmap.db -o ./maps/my_map --all

  # Export only 2D map for Nav2
  %(prog)s ~/.ros/rtabmap.db -o ./maps/nav2_map --2d

  # Export 3D point cloud with custom voxel size
  %(prog)s ~/.ros/rtabmap.db -o ./maps/cloud.pcd --3d --voxel 0.02

  # Export trajectory in TUM format
  %(prog)s ~/.ros/rtabmap.db -o ./trajectory.txt --poses
        """
    )

    parser.add_argument('database',
                       help='Path to RTAB-Map database (.db file)')
    parser.add_argument('-o', '--output', required=True,
                       help='Output path (directory or file depending on mode)')

    # Export mode selection
    mode_group = parser.add_mutually_exclusive_group(required=True)
    mode_group.add_argument('--all', action='store_true',
                           help='Export all formats (2D map, 3D cloud, poses)')
    mode_group.add_argument('--2d', action='store_true', dest='map_2d',
                           help='Export 2D occupancy grid (PGM + YAML)')
    mode_group.add_argument('--3d', action='store_true', dest='cloud_3d',
                           help='Export 3D point cloud')
    mode_group.add_argument('--poses', action='store_true',
                           help='Export trajectory/poses')

    # Optional parameters
    parser.add_argument('--name',
                       help='Map name (for --all mode, default: timestamp)')
    parser.add_argument('--resolution', type=float, default=0.05,
                       help='Grid resolution in meters (for 2D map, default: 0.05)')
    parser.add_argument('--voxel', type=float, default=0.01,
                       help='Voxel filter size in meters (for 3D cloud, default: 0.01)')
    parser.add_argument('--max-range', type=float, default=0.0,
                       help='Max point range in meters (for 3D cloud, 0=unlimited)')
    parser.add_argument('--format', choices=['pcd', 'ply', 'tum', 'kitti', 'g2o'],
                       help='Output format (pcd/ply for 3D, tum/kitti/g2o for poses)')

    args = parser.parse_args()

    # Check if rtabmap-export is available
    if subprocess.run(['which', 'rtabmap-export'],
                     capture_output=True).returncode != 0:
        print("Error: rtabmap-export command not found")
        print("Please install RTAB-Map tools:")
        print("  sudo apt install ros-humble-rtabmap-ros")
        return 1

    try:
        exporter = RTABMapExporter(args.database)

        success = False

        if args.all:
            # Export all formats
            success = exporter.export_all(args.output, args.name)

        elif args.map_2d:
            # Export 2D occupancy grid
            success = exporter.export_2d_map(args.output, args.resolution)

        elif args.cloud_3d:
            # Export 3D point cloud
            fmt = args.format if args.format in ['pcd', 'ply'] else 'pcd'
            if not args.output.endswith(f'.{fmt}'):
                args.output = f"{args.output}.{fmt}"
            success = exporter.export_3d_cloud(
                args.output,
                format=fmt,
                voxel_size=args.voxel,
                max_range=args.max_range
            )

        elif args.poses:
            # Export trajectory
            fmt = args.format if args.format in ['tum', 'kitti', 'g2o'] else 'tum'
            success = exporter.export_poses(args.output, format=fmt)

        return 0 if success else 1

    except FileNotFoundError as e:
        print(f"Error: {e}")
        return 1
    except Exception as e:
        print(f"Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == '__main__':
    sys.exit(main())
