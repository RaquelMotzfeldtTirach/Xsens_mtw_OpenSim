
#  Copyright (c) 2003-2024 Movella Technologies B.V. or subsidiaries worldwide.
#  All rights reserved.
#  
#  Redistribution and use in source and binary forms, with or without modification,
#  are permitted provided that the following conditions are met:
#  
#  1.	Redistributions of source code must retain the above copyright notice,
#  	this list of conditions, and the following disclaimer.
#  
#  2.	Redistributions in binary form must reproduce the above copyright notice,
#  	this list of conditions, and the following disclaimer in the documentation
#  	and/or other materials provided with the distribution.
#  
#  3.	Neither the names of the copyright holders nor the names of their contributors
#  	may be used to endorse or promote products derived from this software without
#  	specific prior written permission.
#  
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
#  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
#  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
#  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
#  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
#  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
#  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
#  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
#  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
#  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
#  
import sys
import os 
import time

module_path = "/home/raquel/Documents/Xsens/xda_python/my_python3_9_venv/lib/python3.9/site-packages"
sys.path.insert(0, module_path)
import xsensdeviceapi.xsensdeviceapi_py39_64 as xda

import time
from threading import Lock
import argparse

def parsing_device(control, mtDevice, startTime, logfileName, dirName):
        # Get the device object
        device = control.device(mtDevice)
        assert(device != 0)

        print("Device: %s, with ID: %s found in file" % (device.productCode(), device.deviceId().toXsString()))
        #print("filter enabled? ", device.isSoftwareFilteringEnabled())
        print("Filter type: %s" % device.xdaFilterProfile().toXsString())

        # By default XDA does not retain data for reading it back.
        # By enabling this option XDA keeps the buffered data in a cache so it can be accessed 
        # through XsDevice::getDataPacketByIndex or XsDevice::takeFirstDataPacketInQueue
        device.setOptions(xda.XSO_RetainBufferedData, xda.XSO_None)
    
        # Load the log file and wait until it is loaded
        # Wait for logfile to be fully loaded, there are three ways to do this:
        # - callback: Demonstrated here, which has loading progress information
        # - waitForLoadLogFileDone: Blocking function, returning when file is loaded
        # - isLoadLogFileInProgress: Query function, used to query the device if the loading is done

        print("Loading the file...")
        device.loadLogFile()
        while device.isLoadLogFileInProgress():
            time.sleep(0)
        print("File is fully loaded")


        # Get total number of samples
        packetCount = device.getDataPacketCount()
        print("Total number of samples: %d" % packetCount)
        

        # Export the data
        print("Exporting the data...")
        
        # Header
        s = "// Start Time: " + str(startTime) + "\n"
        s += "// Update Rate: " + str(device.updateRate()) + "Hz \n"
        s += "// Filter Profile: " + device.xdaFilterProfile().toXsString() +"\n"
        s += "// Option Flags: AHS Disabled ICC Disabled \n"
        s += "// Firmware Version: 4.0.2 \n"
        s += "PacketCounter\tSampleTimeFine\tYear\tMonth\tDay\tSecond\tUTC_Nano\tUTC_Year\tUTC_Month\tUTC_Day\tUTC_Hour\tUTC_Minute\tUTC_Second\tUTC_Valid\tAcc_X\tAcc_Y\tAcc_Z	Mat[1][1]\tMat[2][1]\tMat[3][1]\tMat[1][2]\tMat[2][2]\tMat[3][2]\tMat[1][3]\tMat[2][3]\tMat[3][3]"
        s += "\n"

        index = 0
        while index < packetCount:
            # Retrieve a packet
            packet = device.getDataPacketByIndex(index)

            s += str(index) + "\t\t\t\t\t\t\t\t\t\t\t\t\t\t"

            if packet.containsCalibratedData():
                acc = packet.calibratedAcceleration()
                s += str(acc[0]) + "\t" +  str(acc[1]) + "\t"+ str(acc[2])


            if packet.containsOrientation():
                matrix = packet.orientationMatrix()
                s += "\t" + str(matrix[0][0]) + "\t" + str(matrix[1][0]) + "\t" + str(matrix[2][0]) + "\t" + str(matrix[0][1]) + "\t" + str(matrix[1][1]) + "\t" + str(matrix[2][1]) + "\t" + str(matrix[0][2]) + "\t" + str(matrix[1][2]) + "\t" + str(matrix[2][2])

            s += "\n"

            index += 1

        exportFileName = logfileName.removesuffix('.mtb').removeprefix('recordings/') + "-000_" + device.deviceId().toXsString() + ".txt"
        exportFileName = os.path.join(dirName, exportFileName)

        with open(exportFileName, "w") as outfile:
            outfile.write(s)
        print("File is exported to: %s" % exportFileName)
        
        print("Closing log file...")
        reset = device.resetLogFileReadPosition()
        if not reset:
            print("Failed to reset log file read position.")
        print("Log file closed.")
        device = 0
        mtDevice = 0



def mtw_parsing(fileName, startTime):

    print("Creating XsControl object...")
    control = xda.XsControl_construct()
    if control is None:
        print("Failed to construct XsControl instance.")
        sys.exit(1)
    assert(control != 0)

    xdaVersion = xda.XsVersion()
    xda.xdaVersion(xdaVersion)
    print("Using XDA version %s" % xdaVersion.toXsString())

    # Making a folder for the files
    dirName = fileName.removesuffix('.mtb')
    os.makedirs(dirName, exist_ok=True)

    try:
        print("Opening log file...")
        logfileName = fileName
        if not control.openLogFile(logfileName):
            raise RuntimeError("Failed to open log file. Aborting.")
        print("Opened log file: %s" % logfileName)

        deviceIdArray = control.deviceIds()
        for i in range(deviceIdArray.size()):
            if deviceIdArray[i].isMtw():
                mtDevice = deviceIdArray[i]
                parsing_device(control, mtDevice, startTime, logfileName, dirName)

                # I don't know why, but the XsControl object needs to be closed and re-opened
                # to be able to open the next log file. Otherwise it will fail.
                print("Closing XsControl object...")
                control.close()

                print("Creating new XsControl object...")
                control = xda.XsControl_construct()
                if control is None:
                    print("Failed to construct XsControl instance.")
                    sys.exit(1)
                assert(control != 0)

                xdaVersion = xda.XsVersion()
                xda.xdaVersion(xdaVersion)
                print("Opening log file...")
                logfileName = fileName
                if not control.openLogFile(logfileName):
                    raise RuntimeError("Failed to open log file. Aborting.")
                print("Opened log file: %s" % logfileName)


        if not mtDevice:
            raise RuntimeError("No MTw device found. Aborting.")
        
    
    except RuntimeError as error:
        print(error)
    except:
        print("An unknown fatal error has occured. Aborting.")
    else:
        print("Successful exit.")



if __name__ == '__main__':
    # Create the argument parser
    parser = argparse.ArgumentParser(description="Process Xsens mtb files.")
    parser.add_argument('mtbFile', type=str, help='The mtb file to process (e.g., MT_01200627-000.mtb).')
    parser.add_argument('startTime', type=float, help='The start time of the recording.')

    # Parse the arguments
    args = parser.parse_args()

    # Pass the logfile name to the main function
    mtw_parsing(args.mtbFile, args.startTime)
