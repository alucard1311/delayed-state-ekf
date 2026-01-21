#!/usr/bin/env python3
# Copyright 2026 Vinay
# SPDX-License-Identifier: MIT

"""
Plot navigation metrics from CSV data.

Generates three publication-quality plots:
1. Position error sawtooth (error_sawtooth.png)
2. Trajectory comparison (trajectory.png)
3. Covariance consistency (covariance_bounds.png)

Usage:
    python3 plot_results.py /path/to/navigation_metrics.csv
    python3 plot_results.py --help
"""

import argparse
import os
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt


def parse_args():
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description='Plot navigation metrics from CSV data',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python3 plot_results.py /tmp/navigation_metrics.csv
    python3 plot_results.py data.csv --output-dir ./plots
        """
    )
    parser.add_argument(
        'csv_file',
        nargs='?',
        default='/tmp/navigation_metrics.csv',
        help='Path to navigation metrics CSV file (default: /tmp/navigation_metrics.csv)'
    )
    parser.add_argument(
        '--output-dir',
        type=str,
        default=None,
        help='Directory for output plots (default: same as CSV file)'
    )
    parser.add_argument(
        '--dpi',
        type=int,
        default=300,
        help='Plot resolution in DPI (default: 300)'
    )
    return parser.parse_args()


def load_data(csv_path: str) -> dict:
    """Load navigation metrics from CSV file using numpy."""
    if not os.path.exists(csv_path):
        print(f"Error: File not found: {csv_path}", file=sys.stderr)
        sys.exit(1)

    # Load CSV with header
    try:
        data = np.genfromtxt(csv_path, delimiter=',', names=True, dtype=float)
    except Exception as e:
        print(f"Error loading CSV: {e}", file=sys.stderr)
        sys.exit(1)

    required_cols = [
        'timestamp', 'truth_x', 'truth_y', 'truth_z',
        'est_x', 'est_y', 'est_z', 'error_3d',
        'sigma_x', 'sigma_y', 'sigma_z'
    ]

    if data.dtype.names is None:
        print("Error: CSV has no header row", file=sys.stderr)
        sys.exit(1)

    missing_cols = [col for col in required_cols if col not in data.dtype.names]
    if missing_cols:
        print(f"Error: Missing columns in CSV: {missing_cols}", file=sys.stderr)
        sys.exit(1)

    return data


def plot_error_sawtooth(data: np.ndarray, output_path: str, dpi: int):
    """
    Plot 1: Position Error Sawtooth

    Shows 3D position error over time. Expected pattern:
    - Error grows during dead reckoning (IMU + DVL only)
    - Error drops sharply when USBL fix arrives
    """
    fig, ax = plt.subplots(figsize=(10, 6))

    ax.plot(data['timestamp'], data['error_3d'], 'b-', linewidth=1.5, label='3D Position Error')

    ax.set_xlabel('Time (seconds)', fontsize=12)
    ax.set_ylabel('Position Error (meters)', fontsize=12)
    ax.set_title('Position Error Over Time', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.set_xlim(left=0)
    ax.set_ylim(bottom=0)
    ax.legend(loc='upper right')

    plt.tight_layout()
    plt.savefig(output_path, dpi=dpi, bbox_inches='tight')
    plt.close()
    print(f"Saved: {output_path}")


def plot_trajectory(data: np.ndarray, output_path: str, dpi: int):
    """
    Plot 2: Trajectory Comparison

    Shows truth and estimated trajectories in 2D (East-North plane).
    Uses equal aspect ratio to prevent distortion.
    """
    fig, ax = plt.subplots(figsize=(10, 10))

    # Plot truth trajectory (blue solid)
    ax.plot(data['truth_y'], data['truth_x'], 'b-', linewidth=2, label='Truth')

    # Plot estimated trajectory (orange dashed)
    ax.plot(data['est_y'], data['est_x'], 'orange', linestyle='--', linewidth=2, label='Estimate')

    # Mark start and end points
    ax.plot(data['truth_y'][0], data['truth_x'][0], 'go', markersize=10, label='Start')
    ax.plot(data['truth_y'][-1], data['truth_x'][-1], 'r^', markersize=10, label='End')

    ax.set_xlabel('East (meters)', fontsize=12)
    ax.set_ylabel('North (meters)', fontsize=12)
    ax.set_title('Trajectory: Truth vs Estimate', fontsize=14, fontweight='bold')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best')

    plt.tight_layout()
    plt.savefig(output_path, dpi=dpi, bbox_inches='tight')
    plt.close()
    print(f"Saved: {output_path}")


def plot_covariance_bounds(data: np.ndarray, output_path: str, dpi: int):
    """
    Plot 3: Covariance Consistency

    Shows actual error with 3-sigma uncertainty bounds.
    If EKF is consistent, error should stay within bounds 99% of the time.
    """
    fig, ax = plt.subplots(figsize=(10, 6))

    # Calculate 3-sigma bound: 3 * sqrt(sigma_x^2 + sigma_y^2 + sigma_z^2)
    sigma_3d = np.sqrt(data['sigma_x']**2 + data['sigma_y']**2 + data['sigma_z']**2)
    bound_3sigma = 3 * sigma_3d

    time = data['timestamp']
    error = data['error_3d']

    # Plot 3-sigma bounds as shaded region
    ax.fill_between(time, 0, bound_3sigma, alpha=0.3, color='blue', label='3-sigma bound')

    # Plot actual error
    ax.plot(time, error, 'r-', linewidth=1.5, label='Actual Error')

    ax.set_xlabel('Time (seconds)', fontsize=12)
    ax.set_ylabel('Position Error (meters)', fontsize=12)
    ax.set_title('Error with 3-sigma Bounds', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.set_xlim(left=0)
    ax.set_ylim(bottom=0)
    ax.legend(loc='upper right')

    plt.tight_layout()
    plt.savefig(output_path, dpi=dpi, bbox_inches='tight')
    plt.close()
    print(f"Saved: {output_path}")


def print_statistics(data: np.ndarray):
    """Print summary statistics from the data."""
    error = data['error_3d']

    # RMSE (Root Mean Square Error)
    rmse = np.sqrt(np.mean(error**2))

    # Max and final error
    max_error = np.max(error)
    final_error = error[-1]

    # Mean error
    mean_error = np.mean(error)

    # Duration
    duration = data['timestamp'][-1] - data['timestamp'][0]

    print("\n" + "="*50)
    print("Navigation Performance Summary")
    print("="*50)
    print(f"  Duration:      {duration:.1f} seconds")
    print(f"  Samples:       {len(data)}")
    print(f"  RMSE:          {rmse:.4f} m")
    print(f"  Mean Error:    {mean_error:.4f} m")
    print(f"  Max Error:     {max_error:.4f} m")
    print(f"  Final Error:   {final_error:.4f} m")
    print("="*50 + "\n")


def main():
    """Main entry point."""
    args = parse_args()

    # Load data
    print(f"Loading: {args.csv_file}")
    data = load_data(args.csv_file)
    print(f"Loaded {len(data)} samples")

    # Determine output directory
    if args.output_dir:
        output_dir = Path(args.output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
    else:
        output_dir = Path(args.csv_file).parent

    # Generate plots
    print("\nGenerating plots...")
    plot_error_sawtooth(
        data,
        str(output_dir / 'error_sawtooth.png'),
        args.dpi
    )
    plot_trajectory(
        data,
        str(output_dir / 'trajectory.png'),
        args.dpi
    )
    plot_covariance_bounds(
        data,
        str(output_dir / 'covariance_bounds.png'),
        args.dpi
    )

    # Print statistics
    print_statistics(data)


if __name__ == '__main__':
    main()
