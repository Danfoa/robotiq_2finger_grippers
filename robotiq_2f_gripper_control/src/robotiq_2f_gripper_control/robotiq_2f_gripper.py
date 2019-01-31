import serial
from serial.serialutil import SerialException
from modbus_crc import verify_modbus_rtu_crc
from modbus_crc import compute_modbus_rtu_crc,verify_modbus_rtu_crc
import numpy as np
import array

ACTION_REQ_IDX = 7
POS_INDEX      = 10 
SPEED_INDEX    = 11
FORCE_INDEX    = 12

class Robotiq2FingerGripper:
    def __init__(self, device_id=0, stroke=0.085, comport='/dev/ttyUSB0',baud=115200):
        try:
            self.serial_com = serial.Serial(comport, baud, timeout = 0.2)
        except IOError as error:
            self.init_success = False
            raise IOError("Communication error with gripper %d: %s " % (device_id, error.message))
        except:
            self.init_success = False
            raise Exception("Communication with gripper %d on serial port: %s and baud rate: %d not achieved" % (device_id, comport, baud))
        
        self.init_success = True
        self._shutdown_driver = False
        self.device_id = device_id+9
        self.stroke = stroke
        self.initialize_communication_variables()
    
    def initialize_communication_variables(self):
        # Comunication registers
        self.rPR = 0
        self.rSP = 255
        self.rFR = 150
        self.rARD = 1
        self.rATR = 0
        self.rGTO = 0
        self.rACT = 0
        self.gSTA = 0
        self.gACT = 0
        self.gGTO = 0
        self.gOBJ = 0
        self.gFLT = 0
        self.gPO = 0
        self.gPR = 0
        self.gCU = 0
        self.act_cmd = [0] * 0x19
        self.act_cmd[:7] = [self.device_id, 0x10, 0x03, 0xE8, 0x00,0x08, 0x10]
        self.act_cmd_bytes = ""
        self._update_cmd()
        self.stat_cmd = [self.device_id, 0x03, 0x07, 0xD0, 0x00, 0x08]
        compute_modbus_rtu_crc(self.stat_cmd)
        self.stat_cmd_bytes = array.array('B',self.stat_cmd).tostring()
        self._max_force = 100.0 # [%]
        
    def shutdown(self):
        self._shutdown_driver = True
        self.serial_com.close()
    
    def process_action_cmd(self):
        if (self._shutdown_driver):
            return False
        try:    
            self.serial_com.write(self.act_cmd_bytes)
            rsp = self.serial_com.read(8)
            rsp = [ord(x) for x in rsp]
            if (len(rsp) != 8):
                return False
            return verify_modbus_rtu_crc(rsp)
        except:
            return False
        
    def process_status_cmd(self):
        try:
            self.serial_com.write(self.stat_cmd_bytes)
            rsp = self.serial_com.read(21)
            rsp = [ord(x) for x in rsp]
            if (len(rsp) != 21):
                return False
            return self.parse_rsp(rsp)
        except:
            return False

    def activate_gripper(self):
        self.rACT = 1
        self.rPR = 0
        self.rSP = 255
        self.rFR = 150
        self._update_cmd()
    
    def deactivate_gripper(self):
        self.rACT = 0
        self._update_cmd()
        
    def activate_emergency_release(self,open_gripper=True):
        self.rATR = 1
        self.rARD = 1

        if (open_gripper):
            self.rARD=0
        self._update_cmd()
                
    def deactivate_emergency_release(self):
        self.rATR = 0
        self._update_cmd()

    def goto(self, pos, vel, force):
        self.rACT = 1
        self.rGTO = 1
        self.rPR = int(np.clip((3. - 230.)/self.stroke * pos + 230., 0, 255))
        self.rSP = int(np.clip(255./(0.1 - 0.013) * vel-0.013, 0, 255))
        self.rFR = int(np.clip(255./(self._max_force) * force, 0, 255))
        self._update_cmd()
        

    def stop(self):
        self.rACT = 1
        self.rGTO = 0
        self._update_cmd()
        
    def parse_rsp(self,rsp):
        if (verify_modbus_rtu_crc(rsp)):
            self.gACT = rsp[3] & 0x1
            self.gGTO = (rsp[3] & 0x8) >> 3
            self.gSTA = (rsp[3] & 0x30) >> 4
            self.gOBJ = (rsp[3] & 0xC0) >> 6
            self.gFLT = rsp[5] & 0x0F
            self.gPR  = rsp[6] & 0xFF
            self.gPO  = rsp[7] & 0xFF
            self.gCU  = (rsp[8] & 0xFF)
            return True
        return False
                
    def is_ready(self):
        return self.gSTA == 3 and self.gACT == 1

    def is_reset(self):
        return self.gSTA == 0 or self.gACT == 0

    def is_moving(self):
        return self.gGTO == 1 and self.gOBJ == 0

    def is_stopped(self):
        return self.gOBJ != 0

    def object_detected(self):
        return self.gOBJ == 1 or self.gOBJ == 2

    def get_fault_status(self):
        return self.gFLT

    def get_pos(self):
        po = float(self.gPO)
        return np.clip(self.stroke/(3.-230.)*(po-230.), 0, self.stroke)

    def get_req_pos(self):
        pr = float(self.gPR)
        return np.clip(self.stroke/(3.-230.)*(pr-230.), 0, self.stroke)

    def get_current(self):
        return self.gCU * 0.1

    def _update_action_req(self):
        self._act_req = self.rACT | (self.rGTO << 3) | (self.rATR << 4) | (self.rARD << 5)

    def _update_cmd(self):
        self._update_action_req()
        self.act_cmd = self.act_cmd[:len(self.act_cmd)-2]
        self.act_cmd[ACTION_REQ_IDX] = self._act_req & 0x39
        self.act_cmd[POS_INDEX] = self.rPR & 0xFF
        self.act_cmd[SPEED_INDEX] = self.rSP & 0xFF
        self.act_cmd[FORCE_INDEX] = self.rFR & 0xFF
        compute_modbus_rtu_crc(self.act_cmd)
        self.act_cmd_bytes = array.array('B',self.act_cmd).tostring()
