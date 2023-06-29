import time
from scipy.interpolate import pchip_interpolate
import numpy as np
from flexsea import fxUtils as fxu # pylint: disable=no-name-in-module
from flexsea import fxEnums as fxe # pylint: disable=no-name-in-module
from flexsea import flexsea as flex
import csv
from dataclasses import dataclass
import threading

#initiate flexSEA object
fxs = flex.FlexSEA()
dev_id=0
motor_current_to_torque = 0.000120
transmission_ratio = 54
transmission_efficiency = 0.67
mode = 1
start_time = 0
start_angle = 0


@dataclass
class DataContainer:
    ''' Class to store instataneous data'''
    state_time: float = 0
    loop_time: float = 0
    # accel_x: float = 0
    # accel_y: float = 0
    # accel_z: float = 0
    # gyro_x: float = 0
    # gyro_y: float = 0
    # gyro_z: float = 0
    motor_current: int = 0
    motor_volt: int = 0
    batt_current: int = 0
    batt_volt: int = 0
    motor_angle: int = 0
    motor_velocity: float = 0
    motor_torque: float = 0
    ankle_angle: float = 0
    ankle_velocity: float = 0
    ankle_torque: float = 0
    temperature: int = 0

def connect_actpack():
    global dev_id
    port='/dev/ttyACM0'
    baud_rate = 230400
    dev_id = fxs.open(port, baud_rate, log_level = 3)
    if dev_id:
        print('Connected')
    else:
        print('Not Connected')
    fxs.start_streaming(dev_id, 200,True)
    
def get_user_input():
    global peak_torque, mode 
    while True:
        user_input = input()
        if user_input[0]=='0':
            mode = 0
            print("Pausing actuation")
        
        elif user_input[0]=='1':
            mode = 1
            print("Restarting actuation")

        elif user_input[0]=='p':
            user_input = user_input[1:]
            try:
                peak_torque = int(user_input)
                print("Updated peak torque: ", user_input)
            except:
                print("Enter int value for torque")

        else:
            continue

def reel_in(cut_off_torque):
    Kp_val = 30
    Ki_val = 300
    Kd_val = 0
    ff_val = 0
    reel_in_mV = 1200
    update_gains(Kp=Kp_val,Kd=Kd_val,Ki=Ki_val,ff=ff_val)
    print('Updated_Gains for reel-in')
    reel_in_time = time.perf_counter()
    command_voltage(dev_id,reel_in_mV)
    while(True):
        now = time.perf_counter()
        read_data(dev_id, data_packet)
        write_data(data_packet)
        print("Motor_current: ",data_packet.motor_current,'Torque_now: ', data_packet.ankle_torque)
        
        if data_packet.motor_current>3000:
            print('Current Overshoot')
            break

        if data_packet.ankle_torque > cut_off_torque:
            command_voltage(dev_id, 0)
            print('Torque_now: ', data_packet.ankle_torque)
            break

        if now-reel_in_time > 10:
            command_voltage(dev_id, 0)
            print('Exiting due to exceeding time')
            print('Torque_now: ', data_packet.ankle_torque)
            break
        
        time.sleep(0.005)

    update_gains()
    print("Reeled In")

def reel_out(reel_out_angle):
    Kp_val = 30
    Ki_val = 300
    Kd_val = 0
    ff_val = 0
    reel_out_mV = -1*1200
    update_gains(Kp=Kp_val,Kd=Kd_val,Ki=Ki_val,ff=ff_val)
    command_voltage(dev_id,reel_out_mV)
    reel_out_time = time.perf_counter()
    initiated_angle = data_packet.ankle_angle
    while(True):
        now = time.perf_counter()
        read_data(dev_id, data_packet)
        write_data(data_packet)
        if now-reel_out_time > 2:
            print('Reeled out: ', data_packet.ankle_angle-initiated_angle)
            break
        time.sleep(0.005)

    update_gains()
    print("Reeled Out")

def four_pt_spline(rise_fraction = 0.2, peak_fraction = 0.55, peak_hold_time = 0.02, fall_fraction = 0.65, bias_torque = 5, peak_torque=10, spline_time = 1.2):
    Kp_val = 40
    Ki_val = 400
    Kd_val = 0
    ff_val = 120
    update_freq = 200
    update_gains(Kp=Kp_val,Ki=Ki_val,Kd=Kd_val,ff=ff_val)
    spline_x = [0,rise_fraction,peak_fraction,peak_fraction+peak_hold_time,fall_fraction,1]
    spline_y = [bias_torque,bias_torque,peak_torque,peak_torque,bias_torque,bias_torque]
    phase = np.linspace(0,1,update_freq*spline_time)
    delay = 1/update_freq
    torque_spline = pchip_interpolate(spline_x,spline_y,phase)
    
    for desired_torque in torque_spline:
        read_data(dev_id,data_packet)
        write_data(data_packet)
        command_torque(dev_id,desired_torque=desired_torque)
        time.sleep(delay)

    update_gains()
    print("Stance phase complete")

def read_data(dev_id, data_packet):
    data = fxs.read_device(dev_id)
    data_packet.state_time = data.state_time
    data_packet.loop_time = (data.state_time - start_time)/1000
    data_packet.motor_current = data.mot_cur
    data_packet.motor_volt =  data.mot_volt
    data_packet.batt_volt = data.batt_volt
    data_packet.batt_current = data.batt_curr
    data_packet.motor_angle = data.mot_ang-start_angle
    data_packet.motor_velocity = data.mot_vel
    data_packet.motor_torque = data.mot_cur * motor_current_to_torque
    data_packet.ankle_angle = data_packet.motor_angle / transmission_ratio
    data_packet.ankle_velocity = 0
    data_packet.ankle_torque =  data_packet.motor_torque*transmission_ratio*transmission_efficiency

def write_data(data_packet):
    subfolder_name = 'exo_datalog/'
    filename = subfolder_name + \
        time.strftime("%Y%m%d_%H%M_") + '.csv'
    my_file = open(filename, 'w', newline='')
    writer = csv.DictWriter(my_file, fieldnames=data_packet.__dict__.keys())
    writer.writeheader()
    writer.writerow(data_packet.__dict__)
    my_file.close()

def update_gains(Kp=0,Kd=0,Ki=0,ff=0):
    global dev_id
    K_val = 0
    B_val = 0
    fxs.set_gains(dev_id = dev_id, kp=Kp, ki=Ki, kd=Kd, k_val = K_val, b_val= B_val, ff=ff)

def command_torque(dev_id, desired_torque):
    motor_torque = desired_torque/(transmission_ratio*transmission_efficiency)
    desired_mA = motor_torque/motor_current_to_torque

    if desired_mA<3000:
        fxs.send_motor_command(dev_id, fxe.FX_CURRENT, value=desired_mA)
    else:
        print('Desired torque is more than permitted value')

def command_angle(dev_id,desired_angle):
    print('Sent desired angle to motor')
    fxs.send_motor_command(dev_id,fxe.FX_POSITION, value=desired_angle)

def command_voltage(dev_id, desired_mV):
    fxs.send_motor_command(dev_id, fxe.FX_VOLTAGE, desired_mV)

def disconnect_actpack(dev_id):
    fxs.stop_streaming(dev_id)
    fxs.close(dev_id)
    print("disconnected")


if __name__=='__main__':
    connect_actpack()
    peak_torque = input("Peak Torque: ")
    actpack_data = fxs.read_device(dev_id)
    input_thread = threading.Thread(target = get_user_input)
    input_thread.daemon = True
    input_thread.start()
    torque_now = actpack_data.mot_cur*motor_current_to_torque*transmission_ratio
    print('Starting Motor_angle: ', actpack_data.mot_ang)
    print('Ankle Torque: ', torque_now)
    start_angle = actpack_data.mot_ang
    start_time = actpack_data.state_time
    data_packet = DataContainer()
    while(True):
        if mode==0:
            print("Paused")
            command_voltage(dev_id,0)
            update_gains()
            if KeyboardInterrupt:
                print('Ctr-C detected, Exiting Gracefully')
                break
            continue

        try:
            # command_voltage(dev_id,1200)
            time.sleep(3)
            reel_in(cut_off_torque=10)
            four_pt_spline(peak_torque=int(peak_torque))
            reel_out(reel_out_angle=60)
        
        except KeyboardInterrupt:
            print('Ctr-C detected, Exiting Gracefully')
            break

    update_gains()
    disconnect_actpack(dev_id)
    print('Done')