#Modify the following line with your own serial port details
#   Currently set COM6 as serial port at 115.2kbps 8N1
#   Refer to PySerial API for other options.  One option to consider is
#   the "timeout" - allowing the program to proceed if after a defined
#   timeout period.  The default = 0, which means wait forever.


import numpy as np
import open3d as o3d

import serial
import math

import time
s = serial.Serial('COM6 ', 115200, timeout=10)

print("Opening: " + s.name)

# reset the buffers of the UART port to delete the remaining data in the buffers

s.reset_output_buffer()
s.reset_input_buffer()

rotations = int(input("Enter number of rotations: "))
print("\n be ready to quickly press onboard button PJ1 after each rotation\n")

# wait for user's signal to start the program
input("Press 's' to start communication...")
# send the character 's' to MCU via UART
# This will signal MCU to start the transmission
s.write('s'.encode())
# recieve 10 measurements from UART of MCU

array = []

text_file = open("measurements.txt", "w")


for j in range(rotations):
    print("\n press PJ1\n")
    for i in range(32):
        x = s.readline()
        print(x.decode())


        string = str(x)
        string = string.strip("b'")
        string = string[0:-4]
            

        array.append(string)
            
        text_file.write(string+"\n")

    time.sleep(9)
     
    
    

       
text_file.close()


# the encode() and decode() function are needed to convert string to bytes
# because pyserial library functions work with type "bytes"


#close the port
print("Closing: " + s.name)
s.close()


if __name__ == "__main__":
    #Remember the goals of modularization
    #   -- smaller problems, reuse, validation, debugging
    #To simulate the data from the sensor lets create a new file with test data 
    f = open("coords.xyz", "w")    #create a new file for writing 
    text_file = open("measurements.txt", "r")
    #Test data: Lets make a rectangular prism as a point cloud in XYZ format
    #   A simple prism would only require 8 vertices, however we
    #   will sample the prism along its x-axis a total of 10x
    #   4 vertices repeated 10x = 40 vertices
    #   This for-loop generates our test data in xyz format
    for i in range(rotations):
        for x in range(32):

            string = text_file.readline()
            string = string.rstrip()
            x_coor = int(string) * math.cos(x*(math.pi/16))
            y_coor = int(string) * math.sin(x*(math.pi/16))
            z_coor = i*100
            

            f.write('{0:f} {1:f} {2:f}\n'.format(x_coor,y_coor,z_coor))  #write x,1,0 (xyz) to file as p4

    f.close()   #there should now be a file containing 40 vertex coordinates                               
    
    #Read the test data in from the file we created        
    print("Read in the prism point cloud data (pcd)")
    pcd = o3d.io.read_point_cloud("coords.xyz", format="xyz")

    #Lets see what our point cloud data looks like numerically       
    print("The PCD array:")
    print(np.asarray(pcd.points))

    #Lets see what our point cloud data looks like graphically       
    print("Lets visualize the PCD: (spawns seperate interactive window)")
    o3d.visualization.draw_geometries([pcd])

    yz_slice_vertex = []
    for x in range(0,rotations*32):
        yz_slice_vertex.append([x])

    #Define coordinates to connect lines in each yz slice        
    lines = []  
    for x in range(rotations):
        lines.append([yz_slice_vertex[32*x], yz_slice_vertex[32*x+1]])
        lines.append([yz_slice_vertex[32*x+1], yz_slice_vertex[32*x+2]])
        lines.append([yz_slice_vertex[32*x+2], yz_slice_vertex[32*x+3]])
        lines.append([yz_slice_vertex[32*x+3], yz_slice_vertex[32*x+4]])
        lines.append([yz_slice_vertex[32*x+4], yz_slice_vertex[32*x+5]])
        lines.append([yz_slice_vertex[32*x+5], yz_slice_vertex[32*x+6]])
        lines.append([yz_slice_vertex[32*x+6], yz_slice_vertex[32*x+7]])
        lines.append([yz_slice_vertex[32*x+7], yz_slice_vertex[32*x+8]])
        lines.append([yz_slice_vertex[32*x+8], yz_slice_vertex[32*x+9]])
        lines.append([yz_slice_vertex[32*x+9], yz_slice_vertex[32*x+10]])
        lines.append([yz_slice_vertex[32*x+10], yz_slice_vertex[32*x+11]])
        lines.append([yz_slice_vertex[32*x+11], yz_slice_vertex[32*x+12]])
        lines.append([yz_slice_vertex[32*x+12], yz_slice_vertex[32*x+13]])
        lines.append([yz_slice_vertex[32*x+13], yz_slice_vertex[32*x+14]])
        lines.append([yz_slice_vertex[32*x+14], yz_slice_vertex[32*x+15]])
        lines.append([yz_slice_vertex[32*x+15], yz_slice_vertex[32*x+16]])
        lines.append([yz_slice_vertex[32*x+16], yz_slice_vertex[32*x+17]])
        lines.append([yz_slice_vertex[32*x+17], yz_slice_vertex[32*x+18]])
        lines.append([yz_slice_vertex[32*x+18], yz_slice_vertex[32*x+19]])
        lines.append([yz_slice_vertex[32*x+19], yz_slice_vertex[32*x+20]])
        lines.append([yz_slice_vertex[32*x+20], yz_slice_vertex[32*x+21]])
        lines.append([yz_slice_vertex[32*x+21], yz_slice_vertex[32*x+22]])
        lines.append([yz_slice_vertex[32*x+22], yz_slice_vertex[32*x+23]])
        lines.append([yz_slice_vertex[32*x+23], yz_slice_vertex[32*x+24]])
        lines.append([yz_slice_vertex[32*x+24], yz_slice_vertex[32*x+25]])
        lines.append([yz_slice_vertex[32*x+25], yz_slice_vertex[32*x+26]])
        lines.append([yz_slice_vertex[32*x+26], yz_slice_vertex[32*x+27]])
        lines.append([yz_slice_vertex[32*x+27], yz_slice_vertex[32*x+28]])
        lines.append([yz_slice_vertex[32*x+28], yz_slice_vertex[32*x+29]])
        lines.append([yz_slice_vertex[32*x+29], yz_slice_vertex[32*x+30]])
        lines.append([yz_slice_vertex[32*x+30], yz_slice_vertex[32*x+31]])
        lines.append([yz_slice_vertex[32*x+31], yz_slice_vertex[32*x]])

        

    #Define coordinates to connect lines between current and next yz slice        
    
    for x in range(rotations - 1):
        lines.append([yz_slice_vertex[32*x], yz_slice_vertex[32*x+32]])
        lines.append([yz_slice_vertex[32*x+1], yz_slice_vertex[32*x+33]])
        lines.append([yz_slice_vertex[32*x+2], yz_slice_vertex[32*x+34]])
        lines.append([yz_slice_vertex[32*x+3], yz_slice_vertex[32*x+35]])
        lines.append([yz_slice_vertex[32*x+4], yz_slice_vertex[32*x+36]])
        lines.append([yz_slice_vertex[32*x+5], yz_slice_vertex[32*x+37]])
        lines.append([yz_slice_vertex[32*x+6], yz_slice_vertex[32*x+38]])
        lines.append([yz_slice_vertex[32*x+7], yz_slice_vertex[32*x+39]])
        lines.append([yz_slice_vertex[32*x+8], yz_slice_vertex[32*x+40]])
        lines.append([yz_slice_vertex[32*x+9], yz_slice_vertex[32*x+41]])
        lines.append([yz_slice_vertex[32*x+10], yz_slice_vertex[32*x+42]])
        lines.append([yz_slice_vertex[32*x+11], yz_slice_vertex[32*x+43]])
        lines.append([yz_slice_vertex[32*x+12], yz_slice_vertex[32*x+44]])
        lines.append([yz_slice_vertex[32*x+13], yz_slice_vertex[32*x+45]])
        lines.append([yz_slice_vertex[32*x+14], yz_slice_vertex[32*x+46]])
        lines.append([yz_slice_vertex[32*x+15], yz_slice_vertex[32*x+47]])
        lines.append([yz_slice_vertex[32*x+16], yz_slice_vertex[32*x+48]])
        lines.append([yz_slice_vertex[32*x+17], yz_slice_vertex[32*x+49]])
        lines.append([yz_slice_vertex[32*x+18], yz_slice_vertex[32*x+50]])
        lines.append([yz_slice_vertex[32*x+19], yz_slice_vertex[32*x+51]])
        lines.append([yz_slice_vertex[32*x+20], yz_slice_vertex[32*x+52]])
        lines.append([yz_slice_vertex[32*x+21], yz_slice_vertex[32*x+53]])
        lines.append([yz_slice_vertex[32*x+22], yz_slice_vertex[32*x+54]])
        lines.append([yz_slice_vertex[32*x+23], yz_slice_vertex[32*x+55]])
        lines.append([yz_slice_vertex[32*x+24], yz_slice_vertex[32*x+56]])
        lines.append([yz_slice_vertex[32*x+25], yz_slice_vertex[32*x+57]])
        lines.append([yz_slice_vertex[32*x+26], yz_slice_vertex[32*x+58]])
        lines.append([yz_slice_vertex[32*x+27], yz_slice_vertex[32*x+59]])
        lines.append([yz_slice_vertex[32*x+28], yz_slice_vertex[32*x+60]])
        lines.append([yz_slice_vertex[32*x+29], yz_slice_vertex[32*x+61]])
        lines.append([yz_slice_vertex[32*x+30], yz_slice_vertex[32*x+62]])
        lines.append([yz_slice_vertex[32*x+31], yz_slice_vertex[32*x+63]])
    
    #This line maps the lines to the 3d coordinate vertices
    line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)),lines=o3d.utility.Vector2iVector(lines))

    #Lets see what our point cloud data with lines looks like graphically       
    o3d.visualization.draw_geometries([line_set])
