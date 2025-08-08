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
# clean the code
# make the code more modular
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
    marker_weight = 15     # Higher weight prioritizes marker data
    orientation_weight = 100  # Set to ZERO to completely disable IMU orientation influence
    constraint_var = 100   # IK solver constraint weight: Set the relative weighting for constraints. Use Infinity to identify the strict enforcement of constraints, otherwise any positive weighting will append the constraint errors to the assembly cost which the solver will minimize.
    
    print(f"Sensor fusion weights: Markers={marker_weight}, Orientations={orientation_weight}")


    # Set variables to use
    sensor_to_opensim_rotation = osim.Vec3(-pi/2, 52*pi/180, 0); # The rotation of IMU data to the OpenSim world frame
    resultsDirectory = 'IKResultsTest';

    # model = osim.Model(modelFileName);  # Load the model to ensure it is valid before running the IK tool
    
    # # Load markers from the markers.xml file and add them to the model
    # markersFileName_xml = "OpenSense/Models/Rajagopal/markers.xml"
    # try:
    #     # Load the marker set from the XML file
    #     markerSet = osim.MarkerSet(markersFileName_xml)
    #     print(f"Loaded {markerSet.getSize()} markers from {markersFileName_xml}")
        
    #     # Add each marker to the model
    #     for i in range(markerSet.getSize()):
    #         marker = markerSet.get(i)
    #         model.addMarker(marker)
    #         print(f"  Added marker: {marker.getName()}")
        
    #     # Update the model connections
    #     model.finalizeConnections()
    #     print(f"Model now has {model.getMarkerSet().getSize()} markers")
        
    # except Exception as e:
    #     print(f"Warning: Could not load markers from {markersFileName_xml}: {e}")
    #     print("Proceeding without model markers...")

    # Scale the model to match with the marker data too
    scale_tool = osim.ScaleTool("OpenSense/scaling_setup.xml")
    generic_model_maker = scale_tool.getGenericModelMaker()
    print("Marker Set File Name:", generic_model_maker.getMarkerSetFileName())

    static_trial = scale_tool.getModelScaler()
    print("Model Scaler File Name:", static_trial.getMarkerFileName())

    model_path = scale_tool.getGenericModelMaker().getModelFileName()
    print("Model File Name:", model_path)
    scale_tool.run()

    model = osim.Model("OpenSense/Models/Rajagopal/Calibrated_and_scaled_Rajagopal_subject28_elbow.osim")

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

    # Downsampling the orientation data to match marker timestamps test
    downsample = True
    if downsample and use_markers and markerTable is not None:
        print("\n=== DOWNSAMPLING ORIENTATION DATA TO MATCH MARKER TIMESTAMPS ===")
        
        imu_times = quatTable.getIndependentColumn()
        marker_times = markerTable.getIndependentColumn()
        
        print(f"IMU times: {len(imu_times)} samples")
        print(f"Marker times: {len(marker_times)} samples")
        
        # Create a new table for downsampled orientations
        downsampled_orientations = osim.TimeSeriesTableQuaternion()
        downsampled_orientations.setColumnLabels(quatTable.getColumnLabels())
        
        # Iterate through marker timestamps and find closest IMU orientation
        for marker_time in marker_times:
            closest_time = min(imu_times, key=lambda t: abs(t - marker_time))
            row = quatTable.getRowAtIndex(quatTable.getNearestRowIndexForTime(closest_time))
            downsampled_orientations.appendRow(marker_time, row)
        
        # Replace quatTable with the downsampled orientations
        quatTable = downsampled_orientations
        
        print(f"Downsampling complete: {quatTable.getNumRows()} rows")
    
    # Compute rotation matrix so that (e.g. "pelvis_imu" + - Z Axis) lines up with model forward (+X)
    base_imu_label = "pelvis_imu"  # Replace with your base IMU label
    direction_on_imu = osim.CoordinateDirection(osim.CoordinateAxis(2), -1)  # Negative Z-axis direction

    # Apply sensor to OpenSim rotation first (matching C++ IMUPlacer implementation)
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
    max_correction = 0.1  # Maximum correction in radians (~17 degrees)
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

    # OrientationWeights
    orientationWeights = osim.OrientationWeightSet()
    for label in orientationData.getColumnLabels():
        orientationWeight = osim.OrientationWeight()
        orientationWeight.setName(str(label))
        orientationWeight.setWeight(orientation_weight)  # Use configured orientation weight
        orientationWeights.cloneAndAppend(orientationWeight)
    
    # Create OrientationsReference with reduced weight to prioritize markers
    if orientation_weight == 0.0:
        # Create an empty OrientationsReference to completely disable orientation constraints
        print("Creating EMPTY OrientationsReference to disable all IMU orientation constraints")
        oRefs = osim.OrientationsReference()  # Empty reference - no orientation data
    else:
        oRefs = osim.OrientationsReference(orientationData, orientationWeights);
     

    # Interpolation test
    interpolation = False
    if interpolation == True and use_markers and markerTable is not None:
        print("\n=== MARKER DATA INTERPOLATION (OpenSim method) ===")
        
        imu_times = quatTable.getIndependentColumn()
        marker_times = markerTable.getIndependentColumn()
        
        print(f"IMU times: {len(imu_times)} samples")
        print(f"Marker times: {len(marker_times)} samples")
        
        # Check if we need interpolation
        times_match = (len(imu_times) == len(marker_times) and 
                    all(abs(imu_times[i] - marker_times[i]) < 1e-6 for i in range(len(imu_times))))
        
        if not times_match:
            print("Interpolating marker data to match IMU timestamps...")
            
            # Create a new marker table with IMU timestamps cut to the overlapping range
            marker_start_time = marker_times[0]
            marker_end_time = marker_times[-1]
            imu_start_time = imu_times[0]
            imu_end_time = imu_times[-1]
            start_time = max(marker_start_time, imu_start_time)
            end_time = min(marker_end_time, imu_end_time)
            print(f"Using overlapping time range: {start_time:.4f} to {end_time:.4f} seconds")
            # only keep rows within the overlapping time range
            overlapping_times = [t for t in imu_times if start_time <= t <= end_time]
            print(f"Found {len(overlapping_times)} overlapping IMU timestamps in the range")

            # Create new marker table with IMU timestamps
            interpolated_table = osim.TimeSeriesTableVec3()
            interpolated_table.setColumnLabels(markerTable.getColumnLabels())
        
    
            # Use OpenSim's interpolation on the overlapping time range
            for imu_time in overlapping_times:
                #try:
                    # Get interpolated row from original table
                    #interpolated_row = markerTable.getNearestRow(imu_time)
                    #interpolated_table.appendRow(imu_time, interpolated_row)
                #except:
                    if imu_time <= marker_times[0]:
                        # Use first row
                        first_row = markerTable.getRowAtIndex(0)
                        interpolated_table.appendRow(imu_time, first_row)
                    elif imu_time >= marker_times[-1]:
                        # Use last row  
                        last_row = markerTable.getRowAtIndex(len(marker_times) - 1)
                        interpolated_table.appendRow(imu_time, last_row)
                    else:
                                                # Find surrounding time points and interpolate
                        for i in range(len(marker_times) - 1):
                            if marker_times[i] <= imu_time <= marker_times[i + 1]:
                                # Linear interpolation factor
                                t = (imu_time - marker_times[i]) / (marker_times[i + 1] - marker_times[i])
                                
                                # Get surrounding rows - these are RowVectorViewVec3 objects
                                row1 = markerTable.getRowAtIndex(i)
                                row2 = markerTable.getRowAtIndex(i + 1)
                                
                                # Instead of manually creating RowVectorVec3, let's create a list of Vec3 objects
                                interpolated_positions = []
                                
                                # Interpolate each marker position
                                for j in range(row1.size()):
                                    pos1 = row1.getElt(0, j)
                                    pos2 = row2.getElt(0, j)
                                    
                                    interp_x = pos1.get(0) * (1 - t) + pos2.get(0) * t
                                    interp_y = pos1.get(1) * (1 - t) + pos2.get(1) * t
                                    interp_z = pos1.get(2) * (1 - t) + pos2.get(2) * t
                                    
                                    interp_pos = osim.Vec3(interp_x, interp_y, interp_z)
                                    interpolated_positions.append(interp_pos)
                                
                                # Create RowVectorVec3 from the list of positions
                                try:
                                    # Method 1: Try creating from a list/array
                                    interp_row = osim.RowVectorVec3()
                                    for pos in interpolated_positions:
                                        interp_row.append(pos)
                                except:
                                    try:
                                        # Method 2: Try creating with size and setting elements differently
                                        interp_row = osim.RowVectorVec3(len(interpolated_positions))
                                        for k, pos in enumerate(interpolated_positions):
                                            # Try different ways to set the element
                                            try:
                                                interp_row.set(k, pos)  # Try set method
                                            except:
                                                try:
                                                    interp_row[k] = pos  # Try bracket notation
                                                except:
                                                    print(f"Cannot set element {k} in RowVectorVec3")
                                                    break
                                    except Exception as e:
                                        print(f"Failed to create interpolated row: {e}")
                                        # Skip this interpolation
                                        continue
                                
                                # Add the interpolated row to the table
                                interpolated_table.appendRow(imu_time, interp_row)
                                break
                    
            # Replace original table with interpolated one, but cut off using the max start and min end times from both tables
            markerTable = interpolated_table
            print(f"Interpolation complete: {markerTable.getNumRows()} rows")

        else:
            print("Timestamps already match - no interpolation needed")
        
        print("=== INTERPOLATION COMPLETE ===\n")

        
    # Create MarkersReference from loaded marker data (following C++ InverseKinematicsTool pattern)
    if use_markers and markerTable is not None and markerWeights is not None and marker_weight > 0.0:
        try:
            # Initialize MarkersReference from markerTable with weights
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
    
    ikSolver = osim.InverseKinematicsSolver(model, mRefs, oRefs, coordinateReferences, constraint_var);
    
    
    # Add this right after creating ikSolver:
    print(f"\n=== SOLVER CONFIGURATION ANALYSIS ===")

    # Check what the solver actually sees
    print(f"Solver configuration:")
    print(f"  - Markers reference: {mRefs.getNumFrames() if hasattr(mRefs, 'getNumFrames') else 'unknown'} frames")
    print(f"  - Orientations reference: {len(oRefs.getTimes()) if hasattr(oRefs, 'getTimes') else 'unknown'} frames")

    print("="*60)


    directory = orientationsFileName.rpartition('/')[0];
    resultsDirectory = directory + '/' + resultsDirectory;
    
    # Create results directory if it doesn't exist
    os.makedirs(resultsDirectory, exist_ok=True);
    
    # Set solver accuracy to match C++ implementation (line 234 in IMUPlacer.cpp)
    accuracy = 1e-9  # Same as C++ implementation
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

    # Add weight verification
    print(f"\n=== WEIGHT VERIFICATION ===")
    print(f"Configured weights: Marker={marker_weight}, Orientation={orientation_weight}")

    # Check marker weights
    if use_markers and mRefs.getNames().size() > 0:
        try:
            marker_weights_check = osim.SimTKArrayDouble()
            mRefs.getWeights(s, marker_weights_check)
            print(f"Actual marker weights: {[marker_weights_check.getElt(i) for i in range(min(3, marker_weights_check.size()))]}")
        except Exception as e:
            print(f"Cannot read marker weights: {e}")

    # Check orientation weights  
    if orientation_weight > 0 and oRefs.getNames().size() > 0:
        try:
            orient_weights_check = osim.SimTKArrayDouble()
            oRefs.getWeights(s, orient_weights_check)
            print(f"Actual orientation weights: {[orient_weights_check.getElt(i) for i in range(min(3, orient_weights_check.size()))]}")
        except Exception as e:
            print(f"Cannot read orientation weights: {e}")

    print("="*60)

    
    
    # Realize position before assembly (important for stability)
    model.realizePosition(s)
    
    # Assemble the model at the first time point (matching C++ line 243)
    ikSolver.assemble(s);  # Only assemble once at the beginning
    
    
    print(f"IK Solver initialized and assembled. Starting processing...");

    # Add detailed data quality analysis
    print(f"\n=== DATA QUALITY ANALYSIS ===")

    # Test solver responsiveness at first frame
    test_time = times[0]
    s.setTime(test_time)
    ikSolver.track(s)

    # Check errors at this time point
    marker_perfect_fit = False
    orientation_perfect_fit = False

    if use_markers and mRefs.getNames().size() > 0:
        try:
            marker_errors = osim.SimTKArrayDouble()
            ikSolver.computeCurrentMarkerErrors(marker_errors)
            if marker_errors.size() > 0:
                marker_error_values = [marker_errors.getElt(i) for i in range(marker_errors.size())]
                avg_marker_error = sum(marker_error_values) / len(marker_error_values)
                max_marker_error = max(marker_error_values)
                min_marker_error = min(marker_error_values)
                
                print(f"Marker errors (meters):")
                print(f"  - Average: {avg_marker_error:.8f}m")
                print(f"  - Range: {min_marker_error:.8f}m to {max_marker_error:.8f}m")
                print(f"  - All values: {[f'{e:.6f}' for e in marker_error_values[:5]]}...")  # Show first 5
                
                # Check if markers have near-perfect fit
                if avg_marker_error < 1e-6:  # Less than 1 micrometer
                    marker_perfect_fit = True
                    print("🚨 MARKERS HAVE NEAR-PERFECT FIT - This will dominate the solution!")
                elif avg_marker_error < 1e-4:  # Less than 0.1mm
                    print("⚠️  Markers have very good fit - may dominate orientation data")
                else:
                    print("✓ Markers have reasonable errors")
            else:
                print("❌ No marker errors available")
        except Exception as e:
            print(f"❌ Cannot compute marker errors: {e}")

    if orientation_weight > 0 and oRefs.getNames().size() > 0:
        try:
            orientation_errors = osim.SimTKArrayDouble()
            ikSolver.computeCurrentOrientationErrors(orientation_errors)
            if orientation_errors.size() > 0:
                orient_error_values = [orientation_errors.getElt(i) for i in range(orientation_errors.size())]
                avg_orient_error = sum(orient_error_values) / len(orient_error_values)
                max_orient_error = max(orient_error_values)
                min_orient_error = min(orient_error_values)
                
                print(f"Orientation errors (radians):")
                print(f"  - Average: {avg_orient_error:.8f}rad ({avg_orient_error*180/3.14159:.6f}°)")
                print(f"  - Range: {min_orient_error:.8f}rad to {max_orient_error:.8f}rad")
                print(f"  - All values: {[f'{e:.6f}' for e in orient_error_values[:5]]}...")  # Show first 5
                
                # Check if orientations have near-perfect fit
                if avg_orient_error < 1e-6:  # Less than 0.00006 degrees
                    orientation_perfect_fit = True
                    print("🚨 ORIENTATIONS HAVE NEAR-PERFECT FIT - This will dominate the solution!")
                elif avg_orient_error < 1e-4:  # Less than 0.006 degrees
                    print("⚠️  Orientations have very good fit - may dominate marker data")
                else:
                    print("✓ Orientations have reasonable errors")
            else:
                print("❌ No orientation errors available")
        except Exception as e:
            print(f"❌ Cannot compute orientation errors: {e}")

    # Analyze cost function contributions
    print(f"\nCost function analysis:")
    if use_markers and mRefs.getNames().size() > 0:
        try:
            marker_errors = osim.SimTKArrayDouble()
            ikSolver.computeCurrentMarkerErrors(marker_errors)
            if marker_errors.size() > 0:
                # Calculate weighted marker cost (sum of weight * error^2)
                marker_cost = 0
                for i in range(marker_errors.size()):
                    error = marker_errors.getElt(i)
                    marker_cost += marker_weight * (error ** 2)
                print(f"  - Weighted marker cost: {marker_cost:.10f}")
        except:
            marker_cost = 0
            print(f"  - Weighted marker cost: Cannot compute")
    else:
        marker_cost = 0
        print(f"  - Weighted marker cost: 0 (no markers)")

    if orientation_weight > 0 and oRefs.getNames().size() > 0:
        try:
            orientation_errors = osim.SimTKArrayDouble()
            ikSolver.computeCurrentOrientationErrors(orientation_errors)
            if orientation_errors.size() > 0:
                # Calculate weighted orientation cost
                orientation_cost = 0
                for i in range(orientation_errors.size()):
                    error = orientation_errors.getElt(i)
                    orientation_cost += orientation_weight * (error ** 2)
                print(f"  - Weighted orientation cost: {orientation_cost:.10f}")
        except:
            orientation_cost = 0
            print(f"  - Weighted orientation cost: Cannot compute")
    else:
        orientation_cost = 0
        print(f"  - Weighted orientation cost: 0 (disabled)")

    # Determine dominance
    total_data_cost = marker_cost + orientation_cost
    if total_data_cost > 0:
        if marker_cost > 0 and orientation_cost > 0:
            ratio = marker_cost / orientation_cost
            if ratio > 1000:
                print(f"🚨 MARKERS DOMINATE: {ratio:.1f}:1 ratio - orientation weights are ineffective!")
            elif ratio < 0.001:
                print(f"🚨 ORIENTATIONS DOMINATE: {1/ratio:.1f}:1 ratio - marker weights are ineffective!")
            else:
                print(f"✓ Balanced cost ratio: {ratio:.2f}:1 (markers:orientations)")
        elif marker_cost > 0:
            print(f"⚠️  Only markers contributing to cost function")
        elif orientation_cost > 0:
            print(f"⚠️  Only orientations contributing to cost function")
    else:
        print(f"❌ No cost function data available")

    # Final diagnosis
    if marker_perfect_fit or orientation_perfect_fit:
        print(f"\n🚨 DIAGNOSIS: Near-perfect fit detected!")
        print(f"   This explains why changing weights has no effect.")
        if marker_perfect_fit:
            print(f"   → Markers fit so perfectly that orientation weights are irrelevant")
            print(f"   → Try: marker_weight=1, orientation_weight=1000 to test")
        if orientation_perfect_fit:
            print(f"   → Orientations fit so perfectly that marker weights are irrelevant")
            print(f"   → Try: marker_weight=1000, orientation_weight=1 to test")
    else:
        print(f"\n✓ No near-perfect fits detected - weights should be effective")

    print("="*60)

    
    # print(f"\n=== UPDATING SOLVER WEIGHTS SAFELY ===")

    # # Update orientation weights if orientations are being used
    # if orientation_weight > 0 and hasattr(oRefs, 'getNames') and oRefs.getNames().size() > 0:
    #     try:
    #         # Get the actual number of orientations the solver expects
    #         solver_num_orientations = oRefs.getNames().size()
            
    #         print(f"Updating orientation weights: {solver_num_orientations} orientations")
    #         orientationWeightsA = osim.SimTKArrayDouble(solver_num_orientations, orientation_weight)
    #         ikSolver.updateOrientationWeights(orientationWeightsA)
    #         print(f"✓ Orientation weights updated to {orientation_weight}")
            
    #     except Exception as e:
    #         print(f"Failed to update orientation weights: {e}")
    #         print("Continuing without weight update...")

    # # Update marker weights if markers are being used  
    # if use_markers and marker_weight > 0 and markerWeights is not None and mRefs.getNames().size() > 0:
    #     try:
    #         # Get the actual number of markers the solver expects
    #         solver_num_markers = mRefs.getNames().size()

    #         print(f"Updating marker weights: {solver_num_markers} markers")
    #         markerWeightsA = osim.SimTKArrayDouble(solver_num_markers, marker_weight)
    #         ikSolver.updateMarkerWeights(markerWeightsA)
    #         print(f"✓ Marker weights updated to {marker_weight}")
            
    #     except Exception as e:
    #         print(f"Failed to update marker weights: {e}")
    #         print("Continuing without weight update...")

    # print("="*60)

    # ikSolver.assemble(s);  # Re-assemble after setting weights
    
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
            #Try to get marker errors to verify markers are being used
            try:
                if use_markers and marker_weight > 0:
                    marker_errors = osim.SimTKArrayDouble()
                    ikSolver.computeCurrentMarkerErrors(marker_errors)
                    if marker_errors.size() > 0:
                        total_marker_error = sum(marker_errors.getElt(j) for j in range(marker_errors.size()))
                        avg_marker_error = total_marker_error / marker_errors.size()
                        print(f"  Marker error: {avg_marker_error:.6f}m ({marker_errors.size()} markers)")
                    else:
                        print(f"  Marker error: No marker data available")
                else:
                    print(f"  Marker error: Markers disabled (weight={marker_weight})")
            except Exception as e:
                print(f"  Marker error: Cannot compute ({e})")
            
            
            # Same for orientation errors
            try:
                if orientation_weight > 0:
                    orientation_errors = osim.SimTKArrayDouble()
                    ikSolver.computeCurrentOrientationErrors(orientation_errors)
                    if orientation_errors.size() > 0:
                        total_orientation_error = sum(orientation_errors.getElt(j) for j in range(orientation_errors.size()))
                        avg_orientation_error = total_orientation_error / orientation_errors.size()
                        print(f"  Orientation error: {avg_orientation_error:.6f}rad ({orientation_errors.size()} orientations)")
                    else:
                        print(f"  Orientation error: No orientation data available")
                else:
                    print(f"  Orientation error: Orientations disabled (weight={orientation_weight})")
            except Exception as e:
                print(f"  Orientation error: Cannot compute ({e})")



            # Show weights - but only if the references have data
            if orientation_weight > 0.0:
                try:
                    read_o_weights = osim.SimTKArrayDouble()
                    oRefs.getWeights(s, read_o_weights)
                    first_o_weight = float(read_o_weights.getElt(0))
                    print(f"  Orientation weights: {first_o_weight}")  # Show first weight
                except Exception as e:
                    print(f"  Orientation weights: Cannot query (error: {e})")
            else:
                print(f"  Orientation weights: 0.0 (disabled)")
            
            if marker_weight > 0.0:
                try:
                    read_m_weights = osim.SimTKArrayDouble()
                    mRefs.getWeights(s, read_m_weights)
                    first_weight = float(read_m_weights.getElt(0))
                    print(f"  Marker weights: {first_weight}")  # Show first weight
                except Exception as e:
                    print(f"  Marker weights: Cannot query (error: {e})")
            else:
                print(f"  Marker weights: Not using markers")
    
        # Check if the solver has both references properly registered
        #try:
        #    # Try to get solver info
        #    marker_tasks = ikSolver.getNumMarkersInUse() if hasattr(ikSolver, 'getNumMarkersInUse') else 'unknown'
        #    orient_tasks = ikSolver.getNumOrientationSensorsInUse() if hasattr(ikSolver, 'getNumOrientationSensorsInUse') else 'unknown'
        #    print(f"  - Solver sees {marker_tasks} marker tasks")
        #    print(f"  - Solver sees {orient_tasks} orientation tasks")
        #except Exception as e:
        #    print(f"  - Cannot query solver tasks: {e}")

        # Set the state to current time
        s.setTime(time_val);

        # Track for this time step (assemble is called internally by track)
        ikSolver.track(s);
        
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