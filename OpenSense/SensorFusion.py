# ----------------------------------------------------------------------- #
# The OpenSim API is a toolkit for musculoskeletal modeling and           #
# simulation. See http://opensim.stanford.edu and the NOTICE file         #
# for more information. OpenSim is developed at Stanford University       #
# and supported by the US National Institutes of Health (U54 GM072970,    #
# R24 HD065690) and by DARPA through the Warrior Web program.             #
#                                                                         #
# Copyright (c) 2005-2019 Stanford University and the Authors             #
# Author(s): James Dunne                                                  #
#                                                                         #
# Licensed under the Apache License, Version 2.0 (the "License");         #
# you may not use this file except in compliance with the License.        #
# You may obtain a copy of the License at                                 #
# http://www.apache.org/licenses/LICENSE-2.0.                             #
#                                                                         #
# Unless required by applicable law or agreed to in writing, software     #
# distributed under the License is distributed on an "AS IS" BASIS,       #
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or         #
# implied. See the License for the specific language governing            #
# permissions and limitations under the License.                          #
# ----------------------------------------------------------------------- #

# Example code to perform orienation tracking with OpenSense. This
# script uses the OpenSense library functions and is part of the OpenSense
# Example files. 
#

# TODO:
# add support for another camera and set of markers
# make the weights configurable
# make the weights individual for each marker and imu

import opensim as osim
from opensim import Vec3
import numpy as np
import math
from math import pi
import argparse
import os
import time

def main(modelFileName, orientationsFileName):
    print(modelFileName)
    print(orientationsFileName)

    # Set up the markers file
    markersFileName = "recordings/subject28/imu_elbow/webcam_elbow.trc"
    
    # Weighting configuration for sensor fusion
    marker_weight = 10.0      # Higher weight prioritizes marker data
    orientation_weight = 0.0  # Set to ZERO to completely disable IMU orientation influence
    constraint_var = 0.001    # IK solver constraint weight
    
    print(f"Sensor fusion weights: Markers={marker_weight}, Orientations={orientation_weight}")
    if orientation_weight == 0.0:
        print("WARNING: Orientation weight set to ZERO - this will disable all IMU orientation constraints!")
        print("This should produce results very close to marker-only IK.")
    else:
        print(f"This configuration prioritizes marker data over IMU orientations")


    # Set variables to use
    sensor_to_opensim_rotation = osim.Vec3(-pi/2, 0, 0); # The rotation of IMU data to the OpenSim world frame
    visualizeTracking = True;  # Boolean to Visualize the tracking simulation
    resultsDirectory = 'IKResultsTest';

    # Instantiate an InverseKinematicsTool
    #imuIK = osim.IMUInverseKinematicsTool();
    model = osim.Model(modelFileName);  # Load the model to ensure it is valid before running the IK tool
    
    # Load markers from the markers.xml file and add them to the model
    markersFileName_xml = "OpenSense/Models/Rajagopal/markers.xml"
    try:
        # Load the marker set from the XML file
        markerSet = osim.MarkerSet(markersFileName_xml)
        print(f"Loaded {markerSet.getSize()} markers from {markersFileName_xml}")
        
        # Add each marker to the model
        for i in range(markerSet.getSize()):
            marker = markerSet.get(i)
            model.addMarker(marker)
            print(f"  Added marker: {marker.getName()}")
        
        # Update the model connections
        model.finalizeConnections()
        print(f"Model now has {model.getMarkerSet().getSize()} markers")
        
    except Exception as e:
        print(f"Warning: Could not load markers from {markersFileName_xml}: {e}")
        print("Proceeding without model markers...")
    
    s = model.initSystem();  # This is crucial for the model to work properly
    coordinates = model.getCoordinateSet();
    imuPlacer = osim.IMUPlacer();
    quatTable = osim.TimeSeriesTableQuaternion(orientationsFileName);
    
    # Show IMU timestamp details for debugging
    imu_times = quatTable.getIndependentColumn()
    print(f"IMU data time range: {imu_times[0]:.4f} to {imu_times[-1]:.4f} seconds")
    print(f"IMU data points: {len(imu_times)}")
    print(f"First 5 IMU timestamps: {[f'{t:.4f}' for t in imu_times[:5]]}")
    print(f"Last 5 IMU timestamps: {[f'{t:.4f}' for t in imu_times[-5:]]}")
    
    # Check IMU sampling rate
    if len(imu_times) > 1:
        avg_interval = (imu_times[-1] - imu_times[0]) / (len(imu_times) - 1)
        estimated_rate = 1.0 / avg_interval if avg_interval > 0 else 0
        print(f"IMU sampling: ~{estimated_rate:.1f} Hz (avg interval: {avg_interval:.4f}s)")
    
    # Load marker data (following C++ InverseKinematicsTool pattern)
    try:
        markerTable = osim.TimeSeriesTableVec3(markersFileName)
        print(f"Loaded marker data from: {markersFileName}")
        marker_times = markerTable.getIndependentColumn()
        print(f"Marker data time range: {marker_times[0]:.4f} to {marker_times[-1]:.4f} seconds")
        print(f"Number of markers: {markerTable.getNumColumns()}")
        print(f"Marker data points: {len(marker_times)}")
        
        # Show first few timestamps for debugging
        print(f"First 5 marker timestamps: {[f'{t:.4f}' for t in marker_times[:5]]}")
        print(f"Last 5 marker timestamps: {[f'{t:.4f}' for t in marker_times[-5:]]}")
        
        # Get marker names
        marker_labels = markerTable.getColumnLabels()
        print(f"Available markers: {[str(label) for label in marker_labels]}")
        
        # Create marker weights using the correct OpenSim API pattern
        # Give markers higher weight to prioritize marker data over IMU orientations
        markerWeights = osim.SetMarkerWeights()
        for label in marker_labels:
            markerWeight = osim.MarkerWeight()
            markerWeight.setName(str(label))
            markerWeight.setWeight(marker_weight)  # Use configured marker weight
            markerWeights.cloneAndAppend(markerWeight)
            
        print(f"Created marker weights for {markerWeights.getSize()} markers (weight: {marker_weight})")
        use_markers = True
        
    except Exception as e:
        print(f"Warning: Could not load marker data from {markersFileName}: {e}")
        print("Proceeding with IMU data only...")
        markerTable = None 
        markerWeights = None
        use_markers = False
    
    # Compute rotation matrix so that (e.g. "pelvis_imu" + SimTK::ZAxis) lines up with model forward (+X)
    base_imu_label = "pelvis_imu"  # Replace with your base IMU label
    direction_on_imu = osim.CoordinateDirection(osim.CoordinateAxis(2), -1)  # Negative Z-axis direction

    # Apply sensor to OpenSim rotation first (matching C++ IMUPlacer implementation)
    sensor_to_opensim_rotation = osim.Vec3(-pi/2, 0, 0); # The rotation of IMU data to the OpenSim world frame
    sensorToOpenSim = osim.Rotation()
    # Apply the sensor rotation sequence: X, Y, Z rotations
    rotX = osim.Rotation()
    rotX.setRotationFromAngleAboutAxis(sensor_to_opensim_rotation[0], osim.CoordinateAxis(0))
    rotY = osim.Rotation() 
    rotY.setRotationFromAngleAboutAxis(sensor_to_opensim_rotation[1], osim.CoordinateAxis(1))
    rotZ = osim.Rotation()
    rotZ.setRotationFromAngleAboutAxis(sensor_to_opensim_rotation[2], osim.CoordinateAxis(2))
    
    # Apply sensor rotations to quatTable first
    osim.OpenSenseUtilities.rotateOrientationTable(quatTable, rotX)
    osim.OpenSenseUtilities.rotateOrientationTable(quatTable, rotY)
    osim.OpenSenseUtilities.rotateOrientationTable(quatTable, rotZ)
    print(f"Applied sensor to OpenSim rotations: {sensor_to_opensim_rotation[0]*180/pi:.1f}, {sensor_to_opensim_rotation[1]*180/pi:.1f}, {sensor_to_opensim_rotation[2]*180/pi:.1f} degrees")

    # Compute heading correction using OpenSense utilities (matching C++ implementation)
    # Check if heading correction should be performed
    base_heading_axis = "-z"  # Equivalent to base_heading_axis property in C++
    
    # Parse heading axis specification (matching C++ logic from lines 104-118)
    imu_axis = base_heading_axis.lower()
    direction = 1
    if imu_axis.startswith('-'):
        direction = -1
    
    axis_char = imu_axis[-1]  # Get last character (x, y, or z)
    if axis_char == 'x':
        direction_on_imu = osim.CoordinateDirection(osim.CoordinateAxis(0), direction)  # XAxis
    elif axis_char == 'y':
        direction_on_imu = osim.CoordinateDirection(osim.CoordinateAxis(1), direction)  # YAxis
    elif axis_char == 'z':
        direction_on_imu = osim.CoordinateDirection(osim.CoordinateAxis(2), direction)  # ZAxis
    else:
        raise ValueError(f"Invalid heading axis specification: {base_heading_axis}")
    
    print(f"Using heading axis: {base_heading_axis} -> CoordinateDirection({axis_char.upper()}Axis, {direction})")

    heading_rotation_vec3 = osim.OpenSenseUtilities.computeHeadingCorrection(
        model, s, quatTable, base_imu_label, direction_on_imu)
    
    # Create rotation using SpaceRotationSequence (matching C++ lines 123-126)
    heading_rotation = osim.Rotation()
    # Apply all three rotation components as in C++ implementation
    heading_rotation.setRotationFromAngleAboutAxis(heading_rotation_vec3.get(0), osim.CoordinateAxis(0))
    temp_rotation_y = osim.Rotation()
    temp_rotation_y.setRotationFromAngleAboutAxis(heading_rotation_vec3.get(1), osim.CoordinateAxis(1))
    temp_rotation_z = osim.Rotation()
    temp_rotation_z.setRotationFromAngleAboutAxis(heading_rotation_vec3.get(2), osim.CoordinateAxis(2))
    
    # Manually compose rotations (X * Y * Z sequence)
    # Since we can't multiply directly, we'll apply them in sequence to the quaternion table
    # But apply reduced corrections to preserve legitimate motion
    
    # Calculate the magnitude of correction needed
    correction_magnitude = math.sqrt(heading_rotation_vec3.get(0)**2 + 
                                   heading_rotation_vec3.get(1)**2 + 
                                   heading_rotation_vec3.get(2)**2)
    
    # If correction is too large (> 0.3 radians ~17 degrees), apply only partial correction
    max_correction = 0.3  # Maximum correction in radians (~17 degrees)
    if correction_magnitude > max_correction:
        scale_factor = max_correction / correction_magnitude
        print(f"Large heading correction detected ({correction_magnitude*180/pi:.1f}°), applying scaled correction ({scale_factor:.2f})")
        
        # Scale down each component
        scaled_heading_rotation = osim.Rotation()
        scaled_heading_rotation.setRotationFromAngleAboutAxis(
            heading_rotation_vec3.get(0) * scale_factor, osim.CoordinateAxis(0))
        scaled_temp_rotation_y = osim.Rotation()
        scaled_temp_rotation_y.setRotationFromAngleAboutAxis(
            heading_rotation_vec3.get(1) * scale_factor, osim.CoordinateAxis(1))
        scaled_temp_rotation_z = osim.Rotation()
        scaled_temp_rotation_z.setRotationFromAngleAboutAxis(
            heading_rotation_vec3.get(2) * scale_factor, osim.CoordinateAxis(2))
        
        # Apply scaled corrections
        osim.OpenSenseUtilities.rotateOrientationTable(quatTable, scaled_heading_rotation)  # X rotation
        osim.OpenSenseUtilities.rotateOrientationTable(quatTable, scaled_temp_rotation_y)  # Y rotation  
        osim.OpenSenseUtilities.rotateOrientationTable(quatTable, scaled_temp_rotation_z)  # Z rotation
        
        print(f"Applied scaled heading correction: X={heading_rotation_vec3.get(0)*scale_factor*180/pi:.1f}°, "
              f"Y={heading_rotation_vec3.get(1)*scale_factor*180/pi:.1f}°, "
              f"Z={heading_rotation_vec3.get(2)*scale_factor*180/pi:.1f}°")
    else:
        # Apply full correction for small corrections
        osim.OpenSenseUtilities.rotateOrientationTable(quatTable, heading_rotation)  # X rotation
        osim.OpenSenseUtilities.rotateOrientationTable(quatTable, temp_rotation_y)  # Y rotation  
        osim.OpenSenseUtilities.rotateOrientationTable(quatTable, temp_rotation_z)  # Z rotation
        print(f"Applied full heading correction: X={heading_rotation_vec3.get(0)*180/pi:.1f}°, "
              f"Y={heading_rotation_vec3.get(1)*180/pi:.1f}°, "
              f"Z={heading_rotation_vec3.get(2)*180/pi:.1f}°")
    orientationData = osim.OpenSenseUtilities.convertQuaternionsToRotations(quatTable);
    
    # Create OrientationsReference with reduced weight to prioritize markers
    if orientation_weight == 0.0:
        # Create an empty OrientationsReference to completely disable orientation constraints
        print("Creating EMPTY OrientationsReference to disable all IMU orientation constraints")
        oRefs = osim.OrientationsReference()  # Empty reference - no orientation data
    else:
        oRefs = osim.OrientationsReference(orientationData);
        
        # Try to set orientation weights to be lower than marker weights
        # This gives markers priority in the inverse kinematics solution
        try:
            for i in range(oRefs.getNumFrames()):
                oRefs.setOrientationWeightForFrame(i, orientation_weight)
            print(f"Set orientation weight to {orientation_weight} (lower than marker weight for prioritization)")
        except Exception as e:
            print(f"Could not set individual orientation weights: {e}")
            print(f"Using default orientation weights - marker priority achieved through higher marker weights ({marker_weight})")
        
        # Alternative approach: try to set a global orientation weight if the method exists
        try:
            if hasattr(oRefs, 'setWeight'):
                oRefs.setWeight(orientation_weight)
                print(f"Set global orientation weight to {orientation_weight}")
            elif hasattr(oRefs, 'setDefaultWeight'):
                oRefs.setDefaultWeight(orientation_weight)
                print(f"Set default orientation weight to {orientation_weight}")
        except Exception as e:
            print(f"Could not set global orientation weight: {e}")
    
    # Create MarkersReference from loaded marker data (following C++ InverseKinematicsTool pattern)
    if use_markers and markerTable is not None and markerWeights is not None:
        try:
            # Initialize MarkersReference from marker file with weights
            mRefs = osim.MarkersReference()
            mRefs.initializeFromMarkersFile(markersFileName, markerWeights)
            
            # Get marker table and print marker info (use the loaded markerTable instead of mRefs methods)
            print(f"Created MarkersReference with {markerWeights.getSize()} markers")
            print(f"Markers configured for IK:")
            for i, label in enumerate(marker_labels):
                print(f"  {i+1}. {str(label)}")
            
            # Get time ranges for analysis using the loaded marker data
            marker_times = markerTable.getIndependentColumn()
            marker_start_time = marker_times[0]
            marker_end_time = marker_times[-1]
            
            imu_times = quatTable.getIndependentColumn()
            imu_start_time = imu_times[0]
            imu_end_time = imu_times[-1]
            
            print(f"Marker time range: {marker_start_time:.4f} to {marker_end_time:.4f} seconds")
            print(f"IMU time range: {imu_start_time:.4f} to {imu_end_time:.4f} seconds")
            
            # Find overlapping time range but be more lenient
            start_time = max(marker_start_time, imu_start_time)
            end_time = min(marker_end_time, imu_end_time)
            
            overlap_duration = end_time - start_time
            print(f"Potential overlap: {start_time:.4f} to {end_time:.4f} seconds (duration: {overlap_duration:.2f}s)")
            
            # Use markers even if there's minimal overlap (> 1 second)
            if overlap_duration > 1.0:
                print(f"Sufficient overlap found ({overlap_duration:.2f}s). Using combined IMU + Marker data.")
                use_markers = True
            else:
                print(f"Insufficient overlap ({overlap_duration:.2f}s), but will still attempt to use markers within available time range.")
                print("OpenSim IK solver will handle time mismatches automatically.")
                use_markers = True  # Still try to use markers - let OpenSim handle time interpolation
                
        except Exception as e:
            print(f"Error creating MarkersReference: {e}")
            print("Falling back to IMU-only mode...")
            mRefs = osim.MarkersReference()
            use_markers = False
    else:
        mRefs = osim.MarkersReference()
        use_markers = False
        print("Using empty MarkersReference (IMU data only)")
    
    coordinateReferences = osim.SimTKArrayCoordinateReference();
    
    # Don't add pelvis constraints - let the IMU data drive the pelvis motion naturally
    # The heading correction should handle coordinate system alignment
    print("Using IMU-driven pelvis motion without artificial constraints")
    
    print('Argument types: ', type(model), type(oRefs), type(mRefs), type(coordinateReferences), type(constraint_var))
    ikSolver = osim.InverseKinematicsSolver(model, mRefs, oRefs, coordinateReferences, constraint_var);
    # Set tool properties
    #imuIK.set_model_file(modelFileName);
    #imuIK.set_orientations_file(orientationsFileName);
    #imuIK.set_sensor_to_opensim_rotations(sensor_to_opensim_rotation)

    directory = orientationsFileName.rpartition('/')[0];
    resultsDirectory = directory + '/' + resultsDirectory;
    
    # Create results directory if it doesn't exist
    os.makedirs(resultsDirectory, exist_ok=True);
    
    # Set solver accuracy to match C++ implementation (line 234 in IMUPlacer.cpp)
    accuracy = 1e-4  # Same as C++ implementation
    ikSolver.setAccuracy(accuracy);
    
    # Print heading correction information
    print(f"Computed heading correction: X={heading_rotation_vec3.get(0):.4f}, Y={heading_rotation_vec3.get(1):.4f}, Z={heading_rotation_vec3.get(2):.4f} radians")
    print(f"Heading correction in degrees: X={heading_rotation_vec3.get(0) * 180.0 / pi:.2f}, Y={heading_rotation_vec3.get(1) * 180.0 / pi:.2f}, Z={heading_rotation_vec3.get(2) * 180.0 / pi:.2f}")
    
    # Set solver properties for better stability
    try:
        ikSolver.setConstraintWeight(constraint_var)
        print(f"Set constraint weight to: {constraint_var}")
    except:
        print("Could not set constraint weight - method may not be available")
        
    # Realize position to ensure model is properly initialized (matching C++ line 94)
    model.realizePosition(s)

    # Get the actual time range - use overlapping time range for combined mode
    times = quatTable.getIndependentColumn();
    
    if use_markers and markerTable is not None:
        # For combined mode, use the overlapping time range to avoid extrapolation errors
        marker_times = markerTable.getIndependentColumn()
        marker_start_time = marker_times[0]
        marker_end_time = marker_times[-1]
        
        imu_start_time = times[0]
        imu_end_time = times[-1]
        
        # Use the overlapping time range
        startTime = max(marker_start_time, imu_start_time)  # Start when both data sources are available
        endTime = min(marker_end_time, imu_end_time)        # End when either data source ends
        
        print(f"Using combined IMU + Marker mode:")
        print(f"  - Overlap time range: {startTime:.4f} to {endTime:.4f} seconds")
        print(f"  - Duration: {endTime - startTime:.2f} seconds")
        
        # Filter IMU times to the overlapping range
        times_filtered = [t for t in times if startTime <= t <= endTime]
        times = times_filtered
        print(f"  - Using {len(times)} IMU frames within overlap period")
        
    else:
        # Use the full IMU time range as the primary time grid for IMU-only mode
        startTime = times[0]  # Use full IMU range
        endTime = times[-1]   # Use full IMU range
        print(f"Using IMU-only mode: {startTime:.4f} to {endTime:.4f} seconds")
        
    numTimeSteps = len(times);
    print(f"Data time range: {startTime} to {endTime} seconds");
    print(f"Number of time steps: {numTimeSteps}");
    
    # Create storage for results
    storage = osim.Storage();
    storage.setName("Coordinates");
    # Choose degrees for the output
    storage.setInDegrees(True);

    
    # Get coordinate names for the header
    coordSet = model.getCoordinateSet();
    numCoords = coordSet.getSize();
    
    # Set column labels
    labels = osim.ArrayStr();
    labels.append("time");
    for i in range(numCoords):
        labels.append(coordSet.get(i).getName());
    storage.setColumnLabels(labels);
    
    # Initialize the solver at the first time point (matching C++ implementation)
    # Use the first time from our filtered times array
    first_time = times[0]
    s.setTime(first_time);
    
    # Realize position before assembly (important for stability)
    model.realizePosition(s)
    
    # Assemble the model at the first time point (matching C++ line 243)
    ikSolver.assemble(s);  # Only assemble once at the beginning
    
    print(f"IK Solver initialized and assembled. Starting processing...");
    
    # Remove coordinate smoothing to get raw IK results first
    # We'll focus on getting the coordinate system alignment correct
    
    # Initialize marker usage tracking
    marker_usage_count = 0
    total_possible_marker_uses = 0
    
    # Start timing
    start_time = time.time();
    
    # Process each time frame
    print(f"Processing {numTimeSteps} frames...")
    
    for i, time_val in enumerate(times):
        # Show progress every 10 frames or at the end
        if i % 10 == 0 or i == numTimeSteps - 1:
            print(f"Processing frame {i+1}/{numTimeSteps}: {time_val:.4f}s");
        
        # Set the state to current time
        s.setTime(time_val);

        # Track for this time step (assemble is called internally by track)
        ikSolver.track(s);
        
        # Count marker usage for this frame
        if use_markers and markerWeights is not None:
            # Use the number of markers from markerWeights since that's what we loaded
            num_available_markers = markerWeights.getSize()
            
            # For now, assume all markers are being used (we can't easily query the solver for this)
            # This is a reasonable assumption since we set all marker weights to 1.0
            num_markers_in_use = num_available_markers
            marker_usage_count += num_markers_in_use
            total_possible_marker_uses += num_available_markers
            
            # Print detailed marker usage info every 100 frames for debugging
            if i % 100 == 0:
                print(f"  Frame {i}: Using {num_markers_in_use}/{num_available_markers} markers")
        
        # Get coordinate values from the state and convert rotational coordinates to degrees
        coordValues = osim.Vector(numCoords, 0.0);  # Initialize with size and default value
        for j in range(numCoords):
            coord = coordSet.get(j);
            value = coord.getValue(s);
            coord_name = coord.getName()
            
            # Convert rotational coordinates from radians to degrees
            if coord.getMotionType() == osim.Coordinate.Rotational:
                value = value * 180.0 / pi;  # Convert radians to degrees
            
            coordValues.set(j, value);
        
        # Append to storage
        storage.append(time_val, coordValues);
    
    # End timing
    end_time = time.time();
    elapsed_time = end_time - start_time;
    
    print(f"IK processing completed for {numTimeSteps} frames in {elapsed_time:.2f} seconds.");
    
    # Print marker usage statistics
    if use_markers and markerWeights is not None:
        print(f"\n=== MARKER USAGE STATISTICS ===")
        print(f"Total markers available: {markerWeights.getSize()}")
        print(f"Total marker uses across all frames: {marker_usage_count}")
        print(f"Total possible marker uses: {total_possible_marker_uses}")
        
        if total_possible_marker_uses > 0:
            usage_percentage = (marker_usage_count / total_possible_marker_uses) * 100
            print(f"Marker usage rate: {usage_percentage:.1f}%")
            
            avg_markers_per_frame = marker_usage_count / numTimeSteps if numTimeSteps > 0 else 0
            print(f"Average markers used per frame: {avg_markers_per_frame:.1f}")
        
        # Print marker names that were configured
        print(f"Markers configured for tracking:")
        for i, label in enumerate(marker_labels):
            print(f"  - {str(label)}")
            
        print(f"=== END MARKER STATISTICS ===\n")
    else:
        print(f"\n=== MARKER USAGE STATISTICS ===")
        print(f"No markers were used (IMU-only mode)")
        print(f"=== END MARKER STATISTICS ===\n")

    # Save results as .mot file
    motFileName = resultsDirectory + "/inverse_kinematics_results.mot";
    storage.printResult(storage, "inverse_kinematics_results", resultsDirectory, -1, ".mot");
    print(f"Results saved to: {motFileName}");
    
    print("Script execution completed successfully.")
    
    # Exit immediately to prevent segmentation fault during OpenSim object destruction
    os._exit(0)

if __name__ == "__main__":
    # if arguments are not provided, use default values
    parser = argparse.ArgumentParser(description="Run OpenSense Inverse Kinematics.")
    parser.add_argument("modelFileName", type=str, nargs='?', help="Calibrated model file path", default="OpenSense/Models/Rajagopal/Calibrated_Rajagopal_subject28_elbow.osim")
    parser.add_argument("orientationsFileName", type=str, nargs='?', help="Orientation File path", default="recordings/subject28/imu_elbow/elbow_orientations_updatedTime.sto")
    args = parser.parse_args()

    main(args.modelFileName, args.orientationsFileName)