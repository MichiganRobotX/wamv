""" cRIO server requests
    'acquire_raw_data'

"""
import socket
import scipy
from scipy.fftpack import fft
import math as m
import numpy as np

def get_channels_from_bytes(bytes_data):
    str_data = bytes_data.decode('utf-8')
    str_list = str_data.split(',') # Or whatever separator
    float_list = [float(s) for s in str_list[:-1]]

    num_samples = int(len(float_list)/3)
    CH0 = [float_list[i*3] for i in range(num_samples)]
    CH1 = [float_list[i*3+1] for i in range(num_samples)]
    CH2 = [float_list[i*3+2] for i in range(num_samples)]

    return (CH0, CH1, CH2)

##Fuction to calculate heading based on 3 hydrophone's data
def get_relative_heading(phase_1, phase_2, phase_ref):

    num = phase_ref - phase_2   #phase difference in X
    denom = phase_ref - phase_1  #phase difference in Y

    heading = m.atan2(num, denom) #take inverse tangent and return radians
    heading = m.degrees(heading) #convert to degrees

    return(heading)


class CRIOManager:
    def __init__(self):
        self.target_freq = 5000
        self.margin = 1000

        self.host = ''  # Standard loopback interface address (localhost)
        self.port = 50000
        self.bin_data = None

        self.Freq_Upper_Bound = self.target_freq + self.margin
        self.Freq_Lower_Bound = self.target_freq - self.margin

    def receive_data(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((self.host, self.port))
            s.listen()
            conn, addr = s.accept()
            with conn:
                print('Connected by', addr)
                if addr[0] == '192.168.1.106':
                    print('cRIO')
                    while True:
                        bin_data = conn.recv(2048)
                        if not bin_data:
                            break
                        if self.bin_data is None:
                            self.bin_data = bin_data
                        else:
                            self.bin_data += bin_data
                else:
                    print('not cRIO')

    def get_phase_from_channels(self, CH0, CH1, CH2):
        ##Calculate FFTs
        #hydrophone 1
        y1 = fft(CH1)   #take fft
        m1 = abs(y1)   #get magnitude
        p1 = np.angle(y1) #get phase in radians

        #hydrophone 2
        y2 = fft(CH2)
        m2 = abs(y2)
        p2 = np.angle(y2)

        #hydrophone reference
        yref = np.fft.fft(CH0)
        mref = abs(yref)
        pref = np.angle(yref)

        ##Determine Maximum Frequency Peak and Phase at Peak
        #hydrophone1
        max_mag1 = 0
        Freq_Max_Idx_1 = 0

        for i in range(self.Freq_Lower_Bound, self.Freq_Upper_Bound+1): #inclusive lower bound, exclusive upper bound
            if m1[i]>max_mag1:
                max_mag1 = m1[i]
                Freq_Max_Idx_1=i

        phase_1 = p1[Freq_Max_Idx_1] #find phase related to freq peak, hydrophone on y-axis, 1

        #hydrophone2
        phase_2 = p2[Freq_Max_Idx_1]; #find phase related to freq peak, hydrophone on x-axis, 2

        #hydrophoneref
        phase_ref = pref[Freq_Max_Idx_1]; #find phase related to freq peak, hydrophone ref

        return phase_1, phase_2, phase_ref


    def process_data(self):

        CH0, CH1, CH2 = get_channels_from_bytes(self.bin_data)
        print('Channel lengths (0, 1, 2): {}, {}, {}'.format(len(CH0), len(CH1), len(CH2)))

        phase_1, phase_2, phase_ref = self.get_phase_from_channels(CH0, CH1, CH2)
        print('Phases (1, 2, ref): {}, {}, {}'.format(phase_1, phase_2, phase_ref))

        h_rel = get_relative_heading(phase_1, phase_2, phase_ref)    #relative heading of beacon
        print('Relative heading: {}'.format(h_rel))

    def callback(self):
        self.receive_data()
        self.process_data()



man = CRIOManager()
man.callback()
