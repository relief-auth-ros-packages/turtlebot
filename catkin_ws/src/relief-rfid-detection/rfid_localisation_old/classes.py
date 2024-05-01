
import numpy as np





class Reader:
    def __init__(self,IP):
        self.IP=IP
        self.antennas=[]
        self.readings=[]
        self.epcs=[]

#    def print(self):
        #print(self.IP)
        #print(len(self.readings))
        #for antenna in self.antennas:
           #print(antenna.index)
           #print(len(antenna.measurements))

    def len_readings(self):
        return len(self.readings)

    def get_readings(self):
        return self.readings

    def get_epcs(self):
        return self.epcs

    def find_antenna_with_index(self,index):
        for a in self.antennas:
            if a.index==index:
                return a
        return []


class Antenna:
    def __init__(self,index):
        self.index=index
        self.poses=[]
        self.epcs=[]
        self.measurements=[]
        self.unwrapped_measurements=[]

    def len_poses(self):
        return len(self.poses)

    def len_measurements(self):
        return len(self.measurements)

    def get_poses(self):
        return self.poses

    def get_measurements(self):
        return self.measurements




class Tag:
    def __init__(self,EPC):
        self.EPC=EPC
        self.x=[]
        self.y=[]
        self.z=[]
        self.CI=[]
        self.to_locate=0
        self.readers=[]


    def get_all_unwrapped_measurements(self):

        x_antenna=[]
        y_antenna=[]
        z_antenna=[]
        th_antenna=[]
        phase=[]
        rssi=[]
        lamda=[]

        for r in self.readers:
            for a in r.antennas:
                if len(a.unwrapped_measurements)>5:
                    x_antenna.append(a.unwrapped_measurements[:,1])
                    y_antenna.append(a.unwrapped_measurements[:,2])
                    z_antenna.append(a.unwrapped_measurements[:,3])
                    th_antenna.append(a.unwrapped_measurements[:,4])
                    phase.append(a.unwrapped_measurements[:,5])
                    rssi.append(a.unwrapped_measurements[:,6])
                    lamda.append(a.unwrapped_measurements[:,7])

        antenna_indices=list(range(0,len(x_antenna)))

        return x_antenna, y_antenna, z_antenna, th_antenna, phase, rssi, lamda, antenna_indices



    def get_all_wrapped_measurements(self):

        x_antenna=[]
        y_antenna=[]
        z_antenna=[]
        th_antenna=[]
        phase=[]
        rssi=[]
        lamda=[]

        for r in self.readers:
            for a in r.antennas:
                if len(a.measurements)>0:
                    x_antenna.append(a.measurements[:,5])
                    y_antenna.append(a.measurements[:,6])
                    z_antenna.append(a.measurements[:,7])
                    th_antenna.append(a.measurements[:,8])
                    phase.append(a.measurements[:,2])
                    rssi.append(a.measurements[:,3])
                    lamda.append(a.measurements[:,4])

        antenna_indices=list(range(0,len(x_antenna)))

        return x_antenna, y_antenna, z_antenna, th_antenna, phase, rssi, lamda, antenna_indices



    def return_antenna_indices(self):
        antenna_indices=[]
        for r in self.readers:
            for a in r.antennas:
                antenna_indices.append([r.IP,a.index])
        return antenna_indices


    def find_antenna_with_IP_and_index(self,IP,index):
        for r in self.readers:
            if r.IP==IP:
                for a in r.antennas:
                    if a.index==index:
                        return a
        return []

    def find_reader_with_IP(self,IP):
        for r in self.readers:
            if r.IP==IP:
                return r
        return []

    def print_location(self):
        print(self.EPC)
        print(self.x)
        print(self.y)
        print(self.z)
        print(self.CI)

#    def print(self):
        #print(self.EPC)
        #print(self.to_locate)
        #for r in self.readers:
            #print(r.IP)
            #for a in r.antennas:
                #print(a.index)
                #print(len(a.measurements))
                #print(len(a.unwrapped_measurements))












class Window:
    def __init__(self,wrapped_measurements):
        self.wrapped_measurements=wrapped_measurements
        self.sequences=[]
        self.unwrapped_measurements=np.empty((0,wrapped_measurements.shape[1]))
        self.k=[]

#    def print(self):
        #print('measurements')
        #print(len(self.wrapped_measurements))
        #print('unwrapped measurements')
        #print(len(self.unwrapped_measurements))
        #print('sequences')
        #print(len(self.sequences))
        #for s in self.sequences:
            #print(' measurements')
            #print(len(s.wrapped_measurements))
            #print('unwrapped measurements')
            #print(len(s.unwrapped_measurements))

class Sequence:
    def __init__(self,wrapped_measurements):
        self.wrapped_measurements=wrapped_measurements
        self.unwrapped_measurements=np.empty((0,wrapped_measurements.shape[1]))
