from actuator.MoteusVersion.ACTUATOR_CODE_MOTEUS import SpringActuator_moteus
from dataclasses import dataclass, field, InitVar
import constants
import time
import csv
from enum import Enum
from scipy import interpolate

class ExoSide(Enum):
    LEFT = 0
    RIGHT = 1

class Exo(SpringActuator_moteus):
    def __init__(self, moteus_motor_ctrl, data_file_name: str, exo_side: ExoSide, sync_detector=None):
        '''
        Exo class referencing SpringActuator superclass
        NOTE: Uses Spring Actuator in Binary Mode: High-Force Mode (applying open loop cable force > 8N) or Low-Force Mode (Closed-loop tracking of low-force = 0.3N)
        '''
        motor_sign = -1 if exo_side==ExoSide.LEFT else 1
        super().__init__(moteus_motor_ctrl, dataFile_name = None, motor_sign = motor_sign)

        self.file_ID = data_file_name
        self.side = exo_side
        self.do_include_sync = True if sync_detector else False
        self.data = self.extendedDataContainer(do_include_sync=self.do_include_sync)
        self.fn_torque_multiplier_wrt_ankle_angle = interpolate.PchipInterpolator(constants.ANKLE_PTS, constants.TORQUE_MULTIPLIER_PTS)
        self.fn_change_in_cable_length_to_ankle_angle = interpolate.PchipInterpolator(constants.ANKLE_PTS, constants.FREE_CABLE_LENGTH)
        self.has_calibrated = False
        self.last_ankle_angle = None
        self.setup_data_writer(file_ID=self.file_ID)

    @dataclass
    class extendedDataContainer(SpringActuator_moteus.DataContainer):
        do_include_sync: InitVar[bool] = False
        ankle_angle: float = None
        ankle_velocity: float = None
        exo_torque: float = None
        commanded_exo_torque: float = None
        gait_phase: float = None
        did_heel_strike: bool = False
        did_toe_off: bool = False
        sync: bool = field(init=False)

        def __post_init__(self, do_include_sync):
            if do_include_sync:
                self.sync = True
            
    async def read_data(self, loop_time=None):
        await super().read_data(loop_time)
        self._apply_IMU_transformation()
        self.data.ankle_angle = self._estimate_ankle_angle()
        self.data.ankle_velocity = self._estimate_ankle_velocity()
        self.data.exo_torque = self._estimate_exo_torque()
        self.data.commanded_exo_torque = None
    
    async def command_exo_torque(self, desired_torque:float, do_ease_torque_off = True):
        self.data.commanded_exo_torque =  desired_torque
        saturated_des_torque = self._saturate_torque(desired_torque)
        des_cable_force = (saturated_des_torque / self.fn_torque_multiplier_wrt_ankle_angle(self.data.ankle_angle)) / constants.EXO_LEVER_ARM
        await self.command_cable_force(des_cable_force) 

    async def command_near_zero_impedance(self):
        await self.command_cam_angle(constants.SPRING_DEFLECTION_FOR_ZERO_IMPEDANCE)

    async def command_exo_impedance(self, desired_K: float, desired_B: float):
        '''TODO: Define an impedance controller for Exo, achieve desired impedance via software'''
        pass

    def setup_data_writer(self, file_ID: str):
        '''file_ID is used as a custom file identifier after date.'''
        if file_ID is not None:
            subfolder_name = 'exo_data/'
            self.filename = subfolder_name + \
                time.strftime("%Y%m%d_%H%M_") + file_ID + \
                '_' + self.side.name + '.csv'
            self.my_file = open(self.filename, 'w', newline='')
            self.writer = csv.DictWriter(
                self.my_file, fieldnames=self.data.__dict__.keys())
            self.writer.writeheader()
    
    def write_data(self):
        if self.file_ID is not None:
            self.writer.writerow(self.data.__dict__)

    def close_file(self):
        if self.file_ID is not None:
            self.my_file.close()

    ## Helper Functions:
    def _apply_IMU_transformation(self):
        '''
        In base class of actuator IMU is oriented with,
        Z along motor Shaft, Y along power connectors, X follows the right hand rule
        For individual use apply transformation to IMU,
        customize this function to re-orient IMU XYZ axis as per application
        '''
        pass

    def _estimate_ankle_angle(self):
        '''
        Estimate the net added or removed cable length by spring actuator wrt initial value from calibration, and use it estimate ankle angle
        TODO: Add Kalman Filter for better estimation, thus preventing random jumps
        '''
        free_cable_length_from_actuator = (self.data.actuator_angle - self.actuator_offset)
        free_cable_length_from_spring = self.func_camAng_to_cableLen(self.data.cam_angle)
        changed_cable_length_at_exo = free_cable_length_from_actuator - free_cable_length_from_spring
        return self.fn_change_in_cable_length_to_ankle_angle(changed_cable_length_at_exo)

    def _estimate_ankle_velocity(self):
        '''
        Differential of ankle angle wrt to time. Can be filtered to 6Hz or around 10Hz as limited by human body dynamics
        TODO: Apply filter to velocity
        '''
        ankle_velocity = (self.data.ankle_angle - self.last_ankle_angle)*self.config.control_loop_freq
        return ankle_velocity

    def _estimate_exo_torque(self):
        return (self.data.cable_force * constants.EXO_LEVER_ARM) * self.fn_torque_multiplier_wrt_ankle_angle(self.data.ankle_angle)

    def _saturate_torque(self, torque: float)->float:
        sat_torque = min(torque, -1*constants.MIN_EXO_TORQUE) if torque<0 else max(torque, constants.MIN_EXO_TORQUE)
        sat_torque = min(sat_torque, constants.MAX_EXO_TORQUE) if sat_torque>0 else max(sat_torque, -1*constants.MAX_EXO_TORQUE)
        return sat_torque

    async def standing_calibration(self):
        await super().initial_calibration()
        self.has_calibrated = True