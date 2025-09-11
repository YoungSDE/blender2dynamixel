import bpy
import json
import math
import os

# No actual printfs if you run this in your Blender script

def export_all_armatures_animation(output_path, continuous_rotation=True):
    
    scene = bpy.context.scene
    frame_start = scene.frame_start
    frame_end = scene.frame_end
    fps = scene.render.fps / scene.render.fps_base
    
    # Find all Armature
    armature_objects = [obj for obj in bpy.data.objects if obj.type == 'ARMATURE']
    
    if not armature_objects:
        print("No Armatures found in scene")
        return False
    
    print(f"=== Discovered Armature List ===")
    for i, armature in enumerate(armature_objects, 1):
        motor_bones = [bone for bone in armature.pose.bones if "motor_id" in bone]
        print(f"{i}. {armature.name}: {len(motor_bones)} motor bones")
    
    # Separate into motor bones and dummy bones
    all_motor_bones = []
    all_dummy_bones = []
    
    for armature in armature_objects:
        for bone in armature.pose.bones:
            bone_info = {
                'armature_name': armature.name,
                'bone_name': bone.name,
                'bone_object': bone,
                'full_name': f"{armature.name}.{bone.name}"
            }
            
            if "motor_id" in bone:
                all_motor_bones.append(bone_info)
            else:
                all_dummy_bones.append(bone_info)
    
    print(f"\n=== 🔍 Overall Bone Analysis ===")
    print(f"Total motor bones: {len(all_motor_bones)}")
    print(f"Total dummy bones: {len(all_dummy_bones)}")
    print(f"Rotation mode: {'Continuous rotation (extended position)' if continuous_rotation else 'Limited rotation (-180°~180°)'}")
    
    # Result data structure
    animation_data = {
        "metadata": {
            "fps": fps,
            "duration_seconds": (frame_end - frame_start) / fps,
            "armature_count": len(armature_objects),
            "motor_joint_count": len(all_motor_bones),
            "dummy_joint_count": len(all_dummy_bones),
            "continuous_rotation": continuous_rotation,
            "position_mode": "extended" if continuous_rotation else "limited",
            "armature_list": [arm.name for arm in armature_objects]
        },
        "motors": {},
        "dummy_joints": {},
        "frames": []
    }
    
    # Check for duplicate motor IDs
    motor_ids = []
    for bone_info in all_motor_bones:
        motor_id = int(bone_info['bone_object'].get("motor_id"))
        if motor_id in motor_ids:
            print(f" Warning: Motor ID {motor_id} duplicate. ({bone_info['full_name']})")
        motor_ids.append(motor_id)
    
    # Collect metadata for each motor bone
    for bone_info in all_motor_bones:
        pose_bone = bone_info['bone_object']
        full_name = bone_info['full_name']
        
        # Basic properties
        motor_id = int(pose_bone.get("motor_id"))
        motor_model = pose_bone.get("motor_model", "X Series")
        motor_rpm = pose_bone.get("motor_rpm", 60)
        gear_ratio = pose_bone.get("gear_ratio", 1.0)
        
        print(f"\n Motor ID {motor_id} ({full_name}):")
        
        # Check rotation axis
        rotation_axis_candidates = [
            "rotation_axis", "axis", "primary_axis", "motor_axis", 
            "rotate_axis", "rot_axis", "main_axis"
        ]
        
        primary_axis = None
        found_key = None
        
        for key in rotation_axis_candidates:
            if key in pose_bone:
                value = pose_bone[key]
                print(f"  Found: '{key}' = {repr(value)}")
                if primary_axis is None:
                    primary_axis = str(value).upper()
                    found_key = key
        
        # Use default value
        if primary_axis is None:
            primary_axis = "Z"
            print(f"  Rotation axis property not found, using default: {primary_axis}")
        else:
            print(f" Axis: {primary_axis} (key: '{found_key}')")
        
        # Save metadata
        animation_data["motors"][full_name] = {
            "armature_name": bone_info['armature_name'],
            "bone_name": bone_info['bone_name'],
            "motor_id": motor_id,
            "motor_model": motor_model,
            "motor_rpm": motor_rpm,
            "gear_ratio": gear_ratio,
            "primary_rotation_axis": primary_axis,
            "found_property_key": found_key,
            "continuous_rotation": continuous_rotation,
            "debug_all_properties": dict(pose_bone.items())
        }
    
    # Dummy joint
    for bone_info in all_dummy_bones:
        pose_bone = bone_info['bone_object']
        full_name = bone_info['full_name']
        parent_name = pose_bone.parent.name if pose_bone.parent else None
        
        animation_data["dummy_joints"][full_name] = {
            "armature_name": bone_info['armature_name'],
            "bone_name": bone_info['bone_name'],
            "parent": parent_name
        }

    
    # Process each frames
    for frame in range(frame_start, frame_end + 1):
        scene.frame_set(frame)
        time = (frame - frame_start) / fps
        
        frame_data = {
            "frame": frame,
            "time": time,
            "joints": {},
            "dummy_joints": {}
        }
        
        # Collect rotation values for each motor bone
        for bone_info in all_motor_bones:
            pose_bone = bone_info['bone_object']
            full_name = bone_info['full_name']
            motor_data = animation_data["motors"][full_name]
            gear_ratio = motor_data["gear_ratio"]
            primary_axis = motor_data["primary_rotation_axis"].lower()
            
            # Get current rotation values
            if pose_bone.rotation_mode == 'QUATERNION':
                rotation = pose_bone.rotation_quaternion.to_euler()
            else:
                rotation = pose_bone.rotation_euler
            
            # Convert rotation values to degrees
            degrees = {
                "x": math.degrees(rotation.x),
                "y": math.degrees(rotation.y),
                "z": math.degrees(rotation.z)
            }
            
            # Calculate motor rotation angle considering gear ratio
            motor_degrees = {
                "x": degrees["x"] * gear_ratio,
                "y": degrees["y"] * gear_ratio,
                "z": degrees["z"] * gear_ratio
            }
            
            # Use primary rotation axis value
            primary_motor_rotation = motor_degrees[primary_axis]
            total_rotation = primary_motor_rotation
            
            # Calculate Dynamixel position according to rotation mode
            if continuous_rotation:
                # Calculate extended position value
                dynamixel_position = int(total_rotation * 4096 / 360)
                dynamixel_position = max(-256000, min(256000, dynamixel_position))
                position_mode = "extended"
                position_range = "±256000 (±22.5 rotations)"
            else:
                # not used
                normalized_rotation = ((total_rotation + 180) % 360) - 180
                dynamixel_position = int(normalized_rotation * 4096 / 360)
                dynamixel_position = max(-2048, min(2047, dynamixel_position))
                position_mode = "limited"
                position_range = "±2048 (±180°)"
            
            # Output debugging information
            if frame == frame_start:
                print(f"   {full_name}: axis={primary_axis}, rotation={primary_motor_rotation:.1f}°, "
                      f"position={dynamixel_position} ({position_mode})")
            
            # Save joint data
            frame_data["joints"][full_name] = {
                "armature_name": bone_info['armature_name'],
                "bone_name": bone_info['bone_name'],
                "rotation_degrees": degrees,
                "motor_rotation_degrees": motor_degrees,
                "primary_axis": primary_axis,
                "primary_motor_rotation": primary_motor_rotation,
                "total_rotation": total_rotation,
                "dynamixel_position": dynamixel_position,
                "position_mode": position_mode,
                "continuous_rotation_enabled": continuous_rotation,
                "motor_id": motor_data["motor_id"]
            }
        
        # Process dummy joints
        for bone_info in all_dummy_bones:
            pose_bone = bone_info['bone_object']
            full_name = bone_info['full_name']
            
            if pose_bone.rotation_mode == 'QUATERNION':
                rotation = pose_bone.rotation_quaternion.to_euler()
            else:
                rotation = pose_bone.rotation_euler
            
            degrees = {
                "x": math.degrees(rotation.x),
                "y": math.degrees(rotation.y),
                "z": math.degrees(rotation.z)
            }
            
            position = pose_bone.head.copy()
            
            frame_data["dummy_joints"][full_name] = {
                "armature_name": bone_info['armature_name'],
                "bone_name": bone_info['bone_name'],
                "rotation_degrees": degrees,
                "position": {"x": position.x, "y": position.y, "z": position.z}
            }
        
        animation_data["frames"].append(frame_data)
        
        # Show progress
        if frame % 20 == 0 or frame == frame_end:
            progress = (frame - frame_start + 1) / (frame_end - frame_start + 1) * 100
            print(f"   Progress: {progress:.1f}%")
    
    # Save as JSON file
    with open(output_path, 'w') as f:
        json.dump(animation_data, f, indent=4)


def export_all_armatures_continuous(output_path):
    """Export Armatures """
    return export_all_armatures_animation(output_path, continuous_rotation=True)


def export_all_armatures_limited(output_path):
    # not used
    return export_all_armatures_animation(output_path, continuous_rotation=False)


# Usage example
if __name__ == "__main__":

    export_all_armatures_continuous(
        "your_animations_folder_location/example.json"
    )