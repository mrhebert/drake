import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import csv
import numpy as np

file_loc = '/home/mhebert/TorqueReplayData/'


with open( file_loc + 'timing_list_.csv', 'rb') as csvfile:
    reader = csv.reader(csvfile, delimiter=',')
    timing = list(reader)
    timing = [[float(j)/1e6 for j in i] for i in timing]
    timing = np.array(timing)
    # np.divide(timing , 1e6)
# print timing
# ----------------------------------------------------------
with open( file_loc + 'command_list_.csv', 'rb') as csvfile:
    reader = csv.reader(csvfile, delimiter=',')
    command_list = list(reader)
    command_list = np.array(command_list)
# print  np.array(command_list)[:,0]

# ax = subplot(1,1,1)
# p1, = ax.plot(command_list[:,0], label="J0", color='red')
plt.plot(timing[:,3] ,command_list[:,0], color='blue' , label="J0")
plt.plot(timing[:,3] ,command_list[:,1], color='red' , label="J1")
plt.plot(timing[:,3] ,command_list[:,2], color='green' , label="J2")
plt.plot(timing[:,3] ,command_list[:,3], color='black' , label="J3")
plt.plot(timing[:,3] ,command_list[:,4], color='yellow' , label="J4")
plt.plot(timing[:,3] ,command_list[:,5], color='purple' , label="J5")
plt.plot(timing[:,3] ,command_list[:,6], color='orange' , label="J6")
plt.ylabel('Commanded Torque')
plt.xlabel('Time (s)')
plt.title('Plan Runner Trajectory (0.0 to 0.5 all joints)')

plt.legend(loc='upper right')
# red_patch = mpatches.Patch(color='red', label='The red data')
# plt.legend(handles=[red_patch])
plt.savefig(file_loc + 'command_list_.png')

plt.clf()

# ----------------------------------------------------------

with open( file_loc + 'spline_status_list_.csv', 'rb') as csvfile:
    reader = csv.reader(csvfile, delimiter=',')
    command_list = list(reader)
    command_list = np.array(command_list)
# print  np.array(command_list)[:,0]

# ax = subplot(1,1,1)
# p1, = ax.plot(command_list[:,0], label="J0", color='red')
plt.plot(timing[:,3] , command_list[:,0], color='blue' , label="J0")
plt.plot(timing[:,3] ,command_list[:,1], color='red' , label="J1")
plt.plot(timing[:,3] ,command_list[:,2], color='green' , label="J2")
plt.plot(timing[:,3] ,command_list[:,3], color='black' , label="J3")
plt.plot(timing[:,3] ,command_list[:,4], color='yellow' , label="J4")
plt.plot(timing[:,3] ,command_list[:,5], color='purple' , label="J5")
plt.plot(timing[:,3] ,command_list[:,6], color='orange' , label="J6")

plt.ylabel('Joint Position (rad)')
plt.xlabel('Time (s)')
plt.title('Plan Runner Trajectory (0.0 to 0.5 all joints)')

plt.legend(loc='upper right')
# red_patch = mpatches.Patch(color='red', label='The red data')
# plt.legend(handles=[red_patch])
plt.savefig(file_loc + 'spline_status_list_.png')

plt.clf()

# ----------------------------------------------------------

with open( file_loc + 'status_list_.csv', 'rb') as csvfile:
    reader = csv.reader(csvfile, delimiter=',')
    command_list = list(reader)
    command_list = np.array(command_list)
# print  np.array(command_list)[:,0]

# ax = subplot(1,1,1)
# p1, = ax.plot(command_list[:,0], label="J0", color='red')
plt.plot(timing[:,2] ,command_list[:,0], color='blue' , label="J0")
plt.plot(timing[:,2] ,command_list[:,1], color='red' , label="J1")
plt.plot(timing[:,2] ,command_list[:,2], color='green' , label="J2")
plt.plot(timing[:,2] ,command_list[:,3], color='black' , label="J3")
plt.plot(timing[:,2] ,command_list[:,4], color='yellow' , label="J4")
plt.plot(timing[:,2] ,command_list[:,5], color='purple' , label="J5")
plt.plot(timing[:,2] ,command_list[:,6], color='orange' , label="J6")

plt.ylabel('Joint Position (rad)')
plt.xlabel('Time (s)')
plt.title('C++ Replay - Plan Runner Trajectory (0.0 to 0.5 all joints)')

plt.legend(loc='upper right')
# red_patch = mpatches.Patch(color='red', label='The red data')
# plt.legend(handles=[red_patch])
plt.savefig(file_loc + 'status_list_.png')

plt.clf()
