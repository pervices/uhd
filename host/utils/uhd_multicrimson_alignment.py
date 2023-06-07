 
import numpy as np  
from matplotlib import pyplot as plt
import sys
import string
from scipy import signal
import os
import shutil

def lag(wave1, wave2):

    forewards = np.argmax(signal.correlate(wave1, wave2)) - len(wave1) + 1
    backwards = np.argmax(signal.correlate(wave2, wave1)) - len(wave2) + 1

    # This is the lag in number of samples
    sign = 1
    if forewards != 0:
        sign = forewards/abs(forewards)
    lag = (abs(forewards) + abs(backwards)) / 2.0 * sign
    return int(lag)

if (len(sys.argv) != 3) & (len(sys.argv) != 4) :
    print("Incorrect number of arguments, expected:")
    print("    MGMT IP address of first crimson unit")
    print("    MGMT IP address of second crimson unit")
    print("    (optional) verify (do not write delays to crimson)")
    print("Example: ", str(sys.argv[0]), "192.168.10.101 192.168.10.102")
    exit()

if sys.argv

# copy the txrx_multi_device_loopback to the current directory where it will 
# have permission to write an output file
directory = os.getcwd()
shutil.copy('/lib/uhd/examples/txrx_multi_device_loopback', directory)

# TODO: set all of the iq_delay properties to 0

# instruct the user to connect crimson for tx alignemnt
print()
print("Prepare for TX alignment")
print("Connect TXA of Crimson",sys.argv[1], "to RXA of crimson", sys.argv[1])
print("Connect TXA of Crimson",sys.argv[2], "to RXB of crimson", sys.argv[1])
input("Press Enter to continue...")
print()

# Get the data from the crimson units
print("Starting TX alignment of crimson units...")
os.system("./txrx_multi_device_loopback --args \"addr="+sys.argv[1]+" addr="+sys.argv[2]+"\" --rate 162500000 --tx_channels \"0 0\" --rx_channels \"0,1 0\" --tx_gain \"0 0\" --rx_gain \"0,0 0\" --tx_freq \"0 0\" --rx_freq \"0,0 0\" --time_ref \"external external\" --clock_ref \"external external\" --ampl \"0.24 0.24\" --wave_freq \"3000000 3000000\" --nsamps 900 --start_time 10")

# this is where the data is saved
filename1 = os.getcwd()+"/output/rx_0_ch_0.dat"
filename2 = os.getcwd()+"/output/rx_0_ch_1.dat"

# read the data from the file for channel 0
data=np.fromfile(filename1, dtype = 'short')
i = data[0::2]
q = data[1::2]
# convert from numpy ndarray to list so that the correlate function will work
i_data1 = i.tolist()
# drop first samples because wave hasn't started yet
i_data1d = i_data1[386:786]
# interpolation x2 so that we are at 325MSPS for the most fine possible delay control
i_data1 = []
for i in range(len(i_data1d)-1):
    i_data1.append(i_data1d[i])
    i_data1.append((i_data1d[i] + i_data1d[i+1])/2)

# read the data from the file for channel 0
data=np.fromfile(filename2, dtype = 'short')
i = data[0::2]
# convert from numpy ndarray to list so that the correlate function will work
i_data2 = i.tolist()
# drop first samples because wave hasn't started yet
i_data2d = i_data2[386:786]
# interpolation x2 so that we are at 325MSPS for the most fine possible delay control
i_data2 = []
for i in range(len(i_data2d)-1):
    i_data2.append(i_data2d[i])
    i_data2.append((i_data2d[i] + i_data2d[i+1])/2)

# calculate the offset
offset12 = lag(i_data1,i_data2)

# print the tx offset for the user
# TODO: write the delay to crimson
if (offset12 >= 0) :
    print("Delay crimson", sys.argv[1], "TX by:", 0)
    print("Delay crimson", sys.argv[2], "TX by:", offset12)
elif (offset12 < 0) :
    print("Delay crimson", sys.argv[1], "TX by:", offset12*-1)
    print("Delay crimson", sys.argv[2], "TX by:", 0)


# instruct the user to connect crimson for rx alignemnt
print()
print("Prepare for RX alignment")
print("Connect TXA of Crimson",sys.argv[1], "to RXA of crimson", sys.argv[1])
print("Connect TXA of Crimson",sys.argv[2], "to RXA of crimson", sys.argv[2])
input("Press Enter to continue...")
print()

# Get the data from the crimson units
print("Starting RX alignment of crimson units...")
os.system("./txrx_multi_device_loopback --args \"addr="+sys.argv[1]+" addr="+sys.argv[2]+"\" --rate 162500000 --tx_channels \"0 0\" --rx_channels \"0 0\" --tx_gain \"0 0\" --rx_gain \"0 0\" --tx_freq \"0 0\" --rx_freq \"0 0\" --time_ref \"external external\" --clock_ref \"external external\" --ampl \"0.24 0.24\" --wave_freq \"3000000 3000000\" --nsamps 900 --start_time 10")

# this is where the data is saved
filename1 = os.getcwd()+"/output/rx_0_ch_0.dat"
filename2 = os.getcwd()+"/output/rx_1_ch_0.dat"

# read the data from the file for channel 0
data=np.fromfile(filename1, dtype = 'short')
i = data[0::2]
q = data[1::2]
# convert from numpy ndarray to list so that the correlate function will work
i_data1 = i.tolist()
# drop first samples because wave hasn't started yet
i_data1d = i_data1[386:786]
# interpolation x2 so that we are at 325MSPS for the most fine possible delay control
i_data1 = []
for i in range(len(i_data1d)-1):
    i_data1.append(i_data1d[i])
    i_data1.append((i_data1d[i] + i_data1d[i+1])/2)

# read the data from the file for channel 0
data=np.fromfile(filename2, dtype = 'short')
i = data[0::2]
# convert from numpy ndarray to list so that the correlate function will work
i_data2 = i.tolist()
# drop first samples because wave hasn't started yet
i_data2d = i_data2[386:786]
# interpolation x2 so that we are at 325MSPS for the most fine possible delay control
i_data2 = []
for i in range(len(i_data2d)-1):
    i_data2.append(i_data2d[i])
    i_data2.append((i_data2d[i] + i_data2d[i+1])/2)

# calculate the offset
offset12 = lag(i_data1,i_data2)

# print the tx offset for the user
# TODO: write the delay to crimson
if (offset12 >= 0) :
    print("Delay crimson", sys.argv[1], "RX by:", 0)
    print("Delay crimson", sys.argv[2], "RX by:", offset12)
elif (offset12 < 0) :
    print("Delay crimson", sys.argv[1], "RX by:", offset12*-1)
    print("Delay crimson", sys.argv[2], "RX by:", 0)


