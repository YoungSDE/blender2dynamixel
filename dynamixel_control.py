import os
import json
import time
import numpy as np
import matplotlib.pyplot as plt
from dynamixel_sdk import *
from enum import Enum

class MultiJointDynamixelController:
    def __init__(self, port="COM3", baudrate=1000000):
        """Initialize Dynamixel controller (Windows environment)"""
        self.port = port
        self.baudrate = baudrate
        
        # Initialize Dynamixel SDK handlers
        self.portHandler = PortHandler(self.port)
        self.packetHandler = PacketHandler(2.0)  # Use protocol 2.0
        
        # Group sync read setup
        self.groupSyncRead = GroupSyncRead(
            self.portHandler,
            self.packetHandler,
            132,  # ADDR_PRESENT_POSITION
            4     # LEN_PRESENT_POSITION
        )
        
        # Add group sync write setup
        self.groupSyncWrite = GroupSyncWrite(
            self.portHandler,
            self.packetHandler,
            116,  # ADDR_GOAL_POSITION
            4     # LEN_GOAL_POSITION
        )
        
        # Address constants
        self.ADDR_OPERATING_MODE = 11
        self.ADDR_TORQUE_ENABLE = 64
        self.ADDR_PROFILE_VELOCITY = 112
        self.ADDR_GOAL_POSITION = 116
        self.ADDR_PRESENT_POSITION = 132
        
        # Connection setup
        try:
            if not self.portHandler.openPort():
                raise Exception(f"Failed to open port {self.port}")
            
            if not self.portHandler.setBaudRate(self.baudrate):
                raise Exception(f"Failed to set baudrate to {self.baudrate}")
                
            print(f"Dynamixel controller initialized on {self.port} at {self.baudrate} baud")
            
        except Exception as e:
            print(f"Error initializing controller: {e}")
            raise
    
    def setup_motors(self, motor_ids, velocity=1023):
        """Setup multiple motors"""
        for motor_id in motor_ids:
            try:
                # Disable torque
                dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
                    self.portHandler, motor_id, self.ADDR_TORQUE_ENABLE, 0
                )
                if dxl_comm_result != COMM_SUCCESS:
                    print(f"Failed to disable torque on motor {motor_id}: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                    continue
                
                # Set extended position control mode
                dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
                    self.portHandler, motor_id, self.ADDR_OPERATING_MODE, 4
                )
                if dxl_comm_result != COMM_SUCCESS:
                    print(f"Failed to set operating mode on motor {motor_id}: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                    continue
                
                # Set profile velocity
                dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
                    self.portHandler, motor_id, self.ADDR_PROFILE_VELOCITY, velocity
                )
                if dxl_comm_result != COMM_SUCCESS:
                    print(f"Failed to set velocity on motor {motor_id}: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                    continue
                
                # Enable torque
                dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
                    self.portHandler, motor_id, self.ADDR_TORQUE_ENABLE, 1
                )
                if dxl_comm_result != COMM_SUCCESS:
                    print(f"Failed to enable torque on motor {motor_id}: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                    continue
                
                # Add sync read parameter
                self.groupSyncRead.addParam(motor_id)
                
                print(f"Motor ID {motor_id} setup complete (Extended Position Mode, Velocity: {velocity})")
                
            except Exception as e:
                print(f"Error setting up motor {motor_id}: {e}")
    
    def set_multiple_positions_simultaneously(self, motor_positions):
        """Set motor positions (using GroupSyncWrite)"""
        # Clear existing data
        self.groupSyncWrite.clearParam()
        
        # Add target position for each motor
        for motor_id, position in motor_positions.items():
            position = max(-256000, min(256000, int(position)))
            
            # Generate 4-byte position data
            position_bytes = [
                position & 0xFF,
                (position >> 8) & 0xFF,
                (position >> 16) & 0xFF,
                (position >> 24) & 0xFF
            ]
            
            # Add to group
            dxl_addparam_result = self.groupSyncWrite.addParam(motor_id, position_bytes)
            if not dxl_addparam_result:
                print(f"Failed to add param for motor {motor_id}")
        
        dxl_comm_result = self.groupSyncWrite.txPacket()
        
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Failed to send group sync write: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            return False
        
        return True
    
    def set_single_position(self, motor_id, position):
        """Set single motor position"""
        position = max(-256000, min(256000, int(position)))
        
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
            self.portHandler, motor_id, self.ADDR_GOAL_POSITION, position
        )
        
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Failed to set position for motor {motor_id}: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        
        return dxl_comm_result == COMM_SUCCESS
    
    def read_positions(self, motor_ids):
        """Read motor positions"""

        dxl_comm_result = self.groupSyncRead.txRxPacket()
        
        # Dictionary to store results
        positions = {}
        
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Failed to read positions: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        else:
            # Read data from each motor
            for motor_id in motor_ids:
                # Check data 
                if self.groupSyncRead.isAvailable(motor_id, self.ADDR_PRESENT_POSITION, 4):
                    # Get current position
                    position = self.groupSyncRead.getData(motor_id, self.ADDR_PRESENT_POSITION, 4)
                    
                    # convert to 32-bit signed integer
                    if position > 2147483647:  # 2^31 - 1
                        position = position - 4294967296  # 2^32
                    
                    positions[motor_id] = position
                else:
                    print(f"Failed to get position data from motor ID {motor_id}")
                    positions[motor_id] = None
        
        return positions
    
    def close(self):
        """Close port"""
        self.portHandler.closePort()
        print("Port closed")

class SpeedOverflowStrategy(Enum):
    #This part hasn't been debugged yet
    REJECT = "reject"                   
    AUTO_SLOW = "auto_slow"             
    FRAME_SKIP = "frame_skip"           
    INTERPOLATE = "interpolate"         


class TimeBasedSimultaneousPlayer:
    def __init__(self, controller, animation_file):
        """Initialize"""
        self.controller = controller
        
        # Load animation
        try:
            with open(animation_file, 'r') as f:
                self.animation_data = json.load(f)
            
            # Extract metadata
            self.metadata = self.animation_data["metadata"]
            self.motors = self.animation_data["motors"]
            self.fps = self.metadata["fps"]
            
            # Generate motor ID list
            self.motor_ids = [self.motors[joint]["motor_id"] for joint in self.motors]
            
            # Motor direction settings (True = forward, False = reverse)
            self.motor_directions = {}
            for motor_id in self.motor_ids:
                self.motor_directions[motor_id] = True  # Default: forward
            
            # Variables for speed analysis
            self.max_position_change_per_frame = {}
            self.calculate_max_position_changes()
            
            # Variables for relative mode
            self.base_positions = {}  # Base position for each motor
            self.animation_offsets = {}  # Animation offsets
            
            print(f"Loaded animation with {len(self.motor_ids)} motor joints")
            print(f"Motor IDs: {self.motor_ids}")
            print(f"Duration: {self.metadata['duration_seconds']} seconds")
            
        except Exception as e:
            print(f"Error loading animation file: {e}")
            raise
    
    def calculate_max_position_changes(self):
        """Calculate maximum position change per frame for each motor"""
        frames = self.animation_data["frames"]
        
        for motor_id in self.motor_ids:
            max_change = 0
            prev_position = None
            
            for frame in frames:
                for joint_name, joint_data in frame["joints"].items():
                    if joint_data["motor_id"] == motor_id:
                        current_position = joint_data["dynamixel_position"]
                        
                        if prev_position is not None:
                            change = abs(current_position - prev_position)
                            max_change = max(max_change, change)
                        
                        prev_position = current_position
                        break
            
            self.max_position_change_per_frame[motor_id] = max_change
            print(f"Motor ID {motor_id}: Max position change per frame = {max_change} units ({max_change*360/4096:.1f}°)")
    
    def calculate_animation_offsets(self):
        """Calculate relative changes in animation"""
        frames = self.animation_data["frames"]
        
        # Set first frame position as reference point
        first_frame = frames[0]
        motor_base_values = {}
        
        for joint_name, joint_data in first_frame["joints"].items():
            motor_id = joint_data["motor_id"]
            base_value = joint_data["dynamixel_position"]
            motor_base_values[motor_id] = base_value
        
        # Calculate offset
        self.animation_offsets = {}
        
        for motor_id in self.motor_ids:
            self.animation_offsets[motor_id] = []
            base_value = motor_base_values[motor_id]
            
            for frame in frames:
                for joint_name, joint_data in frame["joints"].items():
                    if joint_data["motor_id"] == motor_id:
                        current_value = joint_data["dynamixel_position"]
                        offset = current_value - base_value
                        self.animation_offsets[motor_id].append(offset)
                        break
        
        print("\n=== Animation Analysis ===")
        
        # Offset range analysis
        for motor_id in self.motor_ids:
            offsets = self.animation_offsets[motor_id]
            min_offset = min(offsets)
            max_offset = max(offsets)
            range_offset = max_offset - min_offset
            
            print(f"Motor {motor_id}: Offset range {min_offset} ~ {max_offset} "
                  f"(Total change: {range_offset} units, {range_offset*360/4096:.1f}°)")
    
    def set_base_positions(self):
        """Set current motor positions as reference points"""
        
        current_positions = self.controller.read_positions(self.motor_ids)
        
        for motor_id in self.motor_ids:
            current_pos = current_positions.get(motor_id)
            if current_pos is not None:
                self.base_positions[motor_id] = current_pos
                current_angle = current_pos * 360 / 4096
                print(f"  Motor {motor_id}: Base position {current_pos} units ({current_angle:.1f}°)")
            else:
                print(f"  Motor {motor_id}: Position read failed, using default value 0")
                self.base_positions[motor_id] = 0 #This part hasn't been debugged yet
        
        print("Position setup complete")
    
    def get_relative_position(self, motor_id, frame_index):
        """Calculate relative position for frame index"""
        if motor_id not in self.base_positions or motor_id not in self.animation_offsets:
            return 0
        
        base_pos = self.base_positions[motor_id]
        offset = self.animation_offsets[motor_id][frame_index]
        
        # Apply direction setting
        if not self.motor_directions.get(motor_id, True):
            offset = -offset
        
        absolute_pos = base_pos + offset
        
        # Extended position mode range limit
        absolute_pos = max(-256000, min(256000, int(absolute_pos)))
        
        return absolute_pos
    
    def set_motor_direction(self, motor_id, reverse=False):
        """Set rotation direction"""
        if motor_id in self.motor_ids:
            self.motor_directions[motor_id] = not reverse
            direction_str = "Reverse" if reverse else "Forward"
            print(f"Motor ID {motor_id}: {direction_str} direction set")
        else:
            print(f"Motor ID {motor_id} not found")
    
    def configure_motor_directions(self):
        print(f"\n=== Motor Rotation Direction Configuration ===")
        print(f"Total of {len(self.motor_ids)} motors available")
        
        for motor_id in self.motor_ids:
            while True:
                response = input(f"Reverse direction for Motor ID {motor_id}? (y/n, default: n): ").lower()
                if response in ['y', 'yes']:
                    self.set_motor_direction(motor_id, reverse=True)
                    break
                elif response in ['n', 'no', '']:
                    self.set_motor_direction(motor_id, reverse=False)
                    break
                else:
                    print("Please enter y or n")
        
        print("\n=== Motor Direction Settings ===")
        for motor_id in self.motor_ids:
            direction_str = "Forward" if self.motor_directions[motor_id] else "Reverse"
            print(f"Motor ID {motor_id}: {direction_str}")
    
    def check_speed_feasibility(self, speed_factor=1.0):
       # This part hasn't been debugged yet
        frame_time = 1.0 / (self.fps * speed_factor)
        feasible = True
        
        print(f"\n=== Speed Feasibility Check ===")
        print(f"Speed factor: {speed_factor}x (frame time: {frame_time:.3f}s)")
        
        max_feasible_speed = float('inf')
        
        for motor_id in self.motor_ids:
            max_change = self.max_position_change_per_frame[motor_id]
            
            # Calculate required angular velocity
            angle_change = max_change * 360 / 4096
            required_angular_velocity = angle_change / frame_time
            required_velocity_units = required_angular_velocity * 60 / (360 * 0.229)
            
            if required_velocity_units > 1023:  # Exceeds Dynamixel maximum speed
                print(f"Motor {motor_id}: Requires {required_velocity_units:.0f} units (MAX: 1023)")
                feasible = False
            else:
                print(f"Motor {motor_id}: Requires {required_velocity_units:.0f} units")
            
            # Calculate maximum possible speed for this motor
            if max_change > 0:
                motor_max_speed = (1023 * 0.229 * 360) / (60 * max_change * 360 / 4096 * self.fps)
                max_feasible_speed = min(max_feasible_speed, motor_max_speed)
        
        if not feasible:
            if max_feasible_speed != float('inf'):
                print(f"Recommended maximum speed factor: {max_feasible_speed:.2f}x")
        
        return feasible, max_feasible_speed if max_feasible_speed != float('inf') else speed_factor
    
    def handle_speed_overflow(self, speed_factor, strategy):
        """Handle speed overflow"""
        # This part hasn't been debugged yet
        feasible, max_feasible_speed = self.check_speed_feasibility(speed_factor)
        
        if feasible:
            print(f"Speed {speed_factor}x is feasible for all motors")
            return speed_factor, "original"
        
        print(f"Speed {speed_factor}x exceeds motor capabilities")
        
        if strategy == SpeedOverflowStrategy.REJECT:
            print(f"Rejecting playback at {speed_factor}x speed")
            print(f"Maximum feasible speed: {max_feasible_speed:.2f}x")
            return None, "rejected"
            
        elif strategy == SpeedOverflowStrategy.AUTO_SLOW:
            safe_speed = max_feasible_speed * 0.9
            print(f"Auto-adjusting speed to {safe_speed:.2f}x (90% of maximum)")
            return safe_speed, "auto_slowed"
            
        elif strategy == SpeedOverflowStrategy.FRAME_SKIP:
            skip_ratio = max(2, int(speed_factor / max_feasible_speed) + 1)
            effective_speed = max_feasible_speed * 0.9
            print(f"⏭Frame skip strategy: Playing every {skip_ratio} frames")
            print(f"Effective speed: {effective_speed:.2f}x with {skip_ratio}x frame skip")
            return effective_speed, f"frame_skip_{skip_ratio}"
            
        elif strategy == SpeedOverflowStrategy.INTERPOLATE:
            interpolation_factor = max(2, int(speed_factor / max_feasible_speed) + 1)
            new_speed = max_feasible_speed * 0.9
            print(f"Interpolation strategy: {interpolation_factor}x frame interpolation")
            print(f"Playing at {new_speed:.2f}x with smoother motion")
            return new_speed, f"interpolated_{interpolation_factor}"
        
        else:
            # Default: automatic speed reduction
            # This part hasn't been debugged yet
            safe_speed = max_feasible_speed * 0.9
            print(f"🔄 Auto-adjusting speed to {safe_speed:.2f}x (90% of maximum)")
            return safe_speed, "auto_slowed"
    
    def setup(self):
        """Motor initial setup (1023 maximum speed)"""
        print("Setting up motors with maximum velocity (1023)...")
        self.controller.setup_motors(self.motor_ids, velocity=1023)
        
        # Initialize relative mode
        self.calculate_animation_offsets()
        self.set_base_positions()
    
    def play_simultaneous_relative(self, speed_factor=1.0, strategy=SpeedOverflowStrategy.AUTO_SLOW):
        """Play animation"""
        
        # Handle speed overflow
        adjusted_speed, strategy_used = self.handle_speed_overflow(speed_factor, strategy)
        
        if adjusted_speed is None:
            print("Playback cancelled due to speed constraints.")
            return None, None, None, None
        
        frames = self.animation_data["frames"]
        
        # Handle frame skip
        if strategy_used.startswith("frame_skip"):
            skip_ratio = int(strategy_used.split("_")[-1])
            frames = frames[::skip_ratio]
            print(f"Using frame skip: {len(frames)} frames")
        
        # Data for recording
        times = []
        target_positions = {motor_id: [] for motor_id in self.motor_ids}
        actual_positions = {motor_id: [] for motor_id in self.motor_ids}
        position_errors = {motor_id: [] for motor_id in self.motor_ids}
        
        print(f"\n=== Animation Playback ===")
        print(f"Final speed: {adjusted_speed}x, Total frames: {len(frames)}")
        print(f"Motor speed: 1023 (Maximum speed)")
        
        start_time = time.time()
        
        try:
            for i, frame in enumerate(frames):
                current_time = (time.time() - start_time) * adjusted_speed
                target_time = frame["time"] / adjusted_speed
                
                # Time synchronization
                if current_time < target_time:
                    time.sleep(target_time - current_time)
                
                # Calculate positions for all motors
                motor_positions = {}
                for motor_id in self.motor_ids:
                    final_position = self.get_relative_position(motor_id, i)
                    motor_positions[motor_id] = final_position
                
                # Send commands to all motors
                success = self.controller.set_multiple_positions_simultaneously(motor_positions)
                
                # Debug output (every 20 frames)
                if i % 20 == 0:
                    motor_info = []
                    for motor_id in self.motor_ids:
                        direction_str = "Forward" if self.motor_directions[motor_id] else "Reverse"
                        base_pos = self.base_positions[motor_id]
                        offset = self.animation_offsets[motor_id][i]
                        final_pos = motor_positions[motor_id]
                        motor_info.append(f"M{motor_id}({direction_str}): {base_pos}+{offset}={final_pos}")
                    
                    print(f"Frame {i+1}/{len(frames)} | Time: {target_time:.2f}s | {' | '.join(motor_info)}")
                
                time.sleep(0.0005)
                
                # Read all motor positions
                actual_position_dict = self.controller.read_positions(self.motor_ids)
                
                # Record data
                times.append(target_time)
                
                for motor_id in self.motor_ids:
                    target_pos = motor_positions[motor_id]
                    actual_pos = actual_position_dict.get(motor_id, 0)
                    
                    target_positions[motor_id].append(target_pos)
                    actual_positions[motor_id].append(actual_pos)
                    
                    if actual_pos is not None:
                        error = abs(target_pos - actual_pos)
                        position_errors[motor_id].append(error)
                    else:
                        position_errors[motor_id].append(None)
                
                # Show progress
                if i % 30 == 0:
                    progress = (i + 1) / len(frames) * 100
                    print(f"Progress: {progress:.1f}%", end="\r")
            
            print(f"\n\n=== Animation Complete ===")
            
            # Final position summary
            print("\n=== Final Position Summary ===")
            final_positions = self.controller.read_positions(self.motor_ids)
            
            for motor_id in self.motor_ids:
                base_pos = self.base_positions[motor_id]
                final_pos = final_positions.get(motor_id, 0)
                total_movement = final_pos - base_pos if final_pos is not None else 0
                
                print(f"Motor {motor_id}: {base_pos} → {final_pos} "
                      f"(Movement: {total_movement} units, {total_movement*360/4096:.1f}°)")
            
            # Final error statistics
            print("\n=== Final Error Statistics ===")
            for motor_id in self.motor_ids:
                valid_errors = [e for e in position_errors[motor_id] if e is not None]
                if valid_errors:
                    avg_error = sum(valid_errors) / len(valid_errors)
                    max_error = max(valid_errors)
                    print(f"Motor {motor_id}: Avg {avg_error:.1f} units ({avg_error*360/4096:.1f}°), Max {max_error:.0f} units ({max_error*360/4096:.1f}°)")
            
        except KeyboardInterrupt:
            print("\n\nAnimation was interrupted by user")
        except Exception as e:
            print(f"\nError during animation playback: {e}")
        
        return times, target_positions, actual_positions, position_errors
    
    def plot_results(self, times, target_positions, actual_positions, position_errors=None):
        """Display playback results as graphs"""
        try:
            fig, axes = plt.subplots(len(self.motor_ids), 2, figsize=(15, 4*len(self.motor_ids)))
            if len(self.motor_ids) == 1:
                axes = axes.reshape(1, -1)
            
            for i, motor_id in enumerate(self.motor_ids):
                # Position graph
                ax1 = axes[i, 0]
                ax1.plot(times, target_positions[motor_id], 'b-', label=f'Target', linewidth=2)
                
                # Plot actual position
                actual_times = []
                actual_pos = []
                for t, pos in zip(times, actual_positions[motor_id]):
                    if pos is not None:
                        actual_times.append(t)
                        actual_pos.append(pos)
                
                if actual_times:
                    ax1.plot(actual_times, actual_pos, 'r-', label=f'Actual', linewidth=1)
                
                ax1.set_xlabel('Time (seconds)')
                ax1.set_ylabel('Position (units)')
                ax1.set_title(f'Motor ID {motor_id}')
                ax1.legend()
                ax1.grid(True)
                
                # Second y-axis (angle)
                ax1_deg = ax1.twinx()
                min_pos = min(target_positions[motor_id]) if target_positions[motor_id] else 0
                max_pos = max(target_positions[motor_id]) if target_positions[motor_id] else 4096
                ax1_deg.set_ylim(min_pos * 360 / 4096, max_pos * 360 / 4096)
                ax1_deg.set_ylabel('Angle (degrees)')
                
                # Error graph
                ax2 = axes[i, 1]
                if position_errors and motor_id in position_errors:
                    error_times = []
                    errors = []
                    for t, err in zip(times, position_errors[motor_id]):
                        if err is not None:
                            error_times.append(t)
                            errors.append(err)
                    
                    if error_times:
                        ax2.plot(error_times, errors, 'g-', linewidth=1)
                        ax2.fill_between(error_times, errors, alpha=0.3, color='green')
                        
                        # Average error line
                        avg_error = sum(errors) / len(errors)
                        ax2.axhline(y=avg_error, color='orange', linestyle='--', 
                                   label=f'Avg: {avg_error:.1f} units')
                
                ax2.set_xlabel('Time (seconds)')
                ax2.set_ylabel('Position Error (units)')
                ax2.set_title(f'Motor ID {motor_id} - Position Error')
                ax2.legend()
                ax2.grid(True)
                
                # Second y-axis (angle error)
                ax2_deg = ax2.twinx()
                ax2_deg.set_ylabel('Error (degrees)')
                if position_errors and errors:
                    ax2_deg.set_ylim(0, max(errors) * 360 / 4096)
            
            plt.tight_layout()
            plt.show()
            
        except Exception as e:
            print(f"Error plotting results: {e}")


def get_overflow_strategy():
    """Select speed overflow handling strategy"""
    # This part hasn't been debugged yet
    strategies = [
        (SpeedOverflowStrategy.REJECT, "Reject playback - Try again with reduced speed"),
        (SpeedOverflowStrategy.AUTO_SLOW, "Auto speed reduction - Automatically adjust to safe speed"), 
        (SpeedOverflowStrategy.FRAME_SKIP, "Frame skip - Play by skipping some frames"),
        (SpeedOverflowStrategy.INTERPOLATE, "Frame interpolation - Correct with smoother motion")
    ]
    
    print("\n=== Select Speed Overflow Handling Method ===")
    for i, (strategy, description) in enumerate(strategies, 1):
        print(f"{i}. {description}")
    
    while True:
        try:
            choice = int(input(f"\nSelect handling method (1-{len(strategies)}, default: 2): ") or 2)
            if 1 <= choice <= len(strategies):
                return strategies[choice - 1][0]
            else:
                print(f"Please enter a number between 1 and {len(strategies)}")
        except ValueError:
            print("Please enter a valid number")


# Main execution section
if __name__ == "__main__":
    try:
        print("=== Blender2Dynamixel Animation Player ===")
        
        # Get user input
        port = input("Enter COM port (default: COM3): ") or "COM3"
        
        # Initialize Dynamixel controller
        controller = MultiJointDynamixelController(port=port)
        
        # Set default animation folder path
        animation_folder = "your_animations_folder_location"
        
        # Animation file input
        file_input = input(f"Animation file name (default: example.json): ") or "example.json"
        
        if not file_input.endswith('.json'):
            file_input += '.json'
        
        animation_file = os.path.join(animation_folder, file_input)
        
        if not os.path.exists(animation_file):
            print(f"Error: File not found at {animation_file}")
            exit(1)
        
        #  Initialize
        player = TimeBasedSimultaneousPlayer(controller, animation_file)
        
        # Motor direction configuration
        configure_directions = input("Configure motor rotation directions? (y/n, default: n): ").lower()
        if configure_directions in ['y', 'yes']:
            player.configure_motor_directions()
        
        # Motor setup
        player.setup()
        
        # Playback speed setting
        speed = float(input("Playback speed multiplier (default: 1.0): ") or 1.0)
        
        # Select speed overflow handling strategy
        strategy = get_overflow_strategy()
        
        # Safety confirmation
        input("Press Enter to start animation (starting from current position)...")
        
        # Play animation
        times, target, actual, errors = player.play_simultaneous_relative(
            speed_factor=speed, 
            strategy=strategy
        )
        
        # Show graph if results exist
        if times is not None:
            if input("Show result graphs? (y/n, default: y): ").lower() != 'n':
                player.plot_results(times, target, actual, errors)
        
    except Exception as e:
        print(f"Error: {e}")
    
    finally:
        # Close connection
        if 'controller' in locals():
            controller.close()