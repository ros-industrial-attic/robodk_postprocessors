# Copyright 2017 - RoboDK Software S.L. - http://www.robodk.com/
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# ----------------------------------------------------
# This file is a POST PROCESSOR for Robot Offline Programming to generate programs 
# for a Kuka robot with RoboDK
#
# To edit/test this POST PROCESSOR script file:
# Select "Program"->"Add/Edit Post Processor", then select your post or create a new one.
# You can edit this file using any text editor or Python editor. Using a Python editor allows to quickly evaluate a sample program at the end of this file.
# Python should be automatically installed with RoboDK
#
# You can also edit the POST PROCESSOR manually:
#    1- Open the *.py file with Python IDLE (right click -> Edit with IDLE)
#    2- Make the necessary changes
#    3- Run the file to open Python Shell: Run -> Run module (F5 by default)
#    4- The "test_post()" function is called automatically
# Alternatively, you can edit this file using a text editor and run it with Python
#
# To use a POST PROCESSOR file you must place the *.py file in "C:/RoboDK/Posts/"
# To select one POST PROCESSOR for your robot in RoboDK you must follow these steps:
#    1- Open the robot panel (double click a robot)
#    2- Select "Parameters"
#    3- Select "Unlock advanced options"
#    4- Select your post as the file name in the "Robot brand" box
#
# To delete an existing POST PROCESSOR script, simply delete this file (.py file)
#
# ----------------------------------------------------
# More information about RoboDK Post Processors and Offline Programming here:
#     http://www.robodk.com/help#PostProcessor
#     http://www.robodk.com/doc/en/PythonAPI/postprocessor.html
# ----------------------------------------------------


# ----------------------------------------------------
# Import RoboDK tools
from robodk import *


HEADER = ''';FOLD INI
BAS (#INITMOV,0 )
;ENDFOLD (INI)

;FOLD STARTPOS
$BWDSTART = FALSE
PDAT_ACT = {VEL 50,ACC 100,APO_DIST 10}
BAS(#PTP_DAT)
FDAT_ACT = {TOOL_NO 0,BASE_NO 0,IPO_FRAME #BASE}
BAS (#FRAMES)
BAS (#VEL_PTP,50)
;ENDFOLD


;FOLD SET DEFAULT SPEED
$VEL.CP=0.2
BAS(#VEL_PTP,50)
BAS(#TOOL,0)
BAS(#BASE,0)
;ENDFOLD

;FOLD PTP FIRST POSITION
$BWDSTART = FALSE
PDAT_ACT = {VEL 50,ACC 100,APO_DIST 10}
FDAT_ACT = {TOOL_NO 0,BASE_NO 0,IPO_FRAME #BASE}
BAS(#FRAMES)
$ADVANCE = 5
;ENDFOLD

PTP $AXIS_ACT ; skip BCO quickly
'''

# ----------------------------------------------------
def pose_2_str(pose):
    """Converts a pose target to a string"""
    #[x,y,z,w,p,r] = Pose_2_KUKA(pose)
    #return ('X %.3f, Y %.3f, Z %.3f, A %.3f, B %.3f, C %.3f' % (x,y,z,r,p,w)) # old version had to be switched
    [x,y,z,r,p,w] = pose_2_xyzrpw(pose)
    return ('X %.3f,Y %.3f,Z %.3f,A %.3f,B %.3f,C %.3f' % (x,y,z,w,p,r))

def pose_2_str_ext(pose,joints):
    njoints = len(joints)
    if njoints <= 6:
        return pose_2_str(pose)
    else:     
        extaxes = ''
        for i in range(njoints-6):
            extaxes = extaxes + (',E%i %.5f' % (i+1, joints[i+6]))
        return pose_2_str(pose) + extaxes
    
def angles_2_str(angles):
    """Prints a joint target"""
    str = 'AXIS: '
    data = ['A1','A2','A3','A4','A5','A6','E1','E2','E3','E4','E5','E6']
    for i in range(len(angles)):
        str = str + ('%s %.5f,' % (data[i], angles[i]))
    str = str[:-1]
    return str

# ----------------------------------------------------    
# Object class that handles the robot instructions/syntax
class RobotPost(object):
    """Robot post object"""
    PROG_EXT = 'src'         # set the program extension
    MAX_LINES_X_PROG = 1400  # maximum number of lines per program. It will then generate multiple "pages (files)"
    INCLUDE_SUB_PROGRAMS = True
    
    # other variables
    ROBOT_POST = ''
    ROBOT_NAME = ''
    
    # Multiple program files variables
    PROG_NAME = 'unknown' # single program name
    PROG_NAMES = []
    PROG_FILES = []    
    PROG_LIST = []
    nLines = 0
    nProgs = 0
    
    PROG = ''
    LOG = ''
    nAxes = 6
    APO_VALUE = 1
    C_DIS = ''#' C_DIS'
    C_PTP = ''#' C_PTP'
    
    
    def __init__(self, robotpost=None, robotname=None, robot_axes = 6, **kwargs):
        self.ROBOT_POST = robotpost
        self.ROBOT_NAME = robotname
        self.PROG = ''
        self.LOG = ''
        self.nAxes = robot_axes
        for k,v in kwargs.items():
            if k == 'lines_x_prog':
                self.MAX_LINES_X_PROG = v       
        
    def ProgStart(self, progname, new_page = False):
        progname_i = progname
        if new_page:
            nPages = len(self.PROG_LIST)
            if nPages == 0:
                progname_i = progname
            else:
                progname_i = "%s%i" % (self.PROG_NAME, nPages)
            
        else:
            self.PROG_NAME = progname
            self.nProgs = self.nProgs + 1
            self.PROG_NAMES = []
            if self.nProgs > 1 and not self.INCLUDE_SUB_PROGRAMS:
                return

        self.PROG_NAMES.append(progname_i)    
    
        self.addline('DEF %s ( )' % progname_i)
        if not new_page:
            self.PROG = self.PROG + HEADER
            if self.nAxes > 6:
                self.addline('$ACT_EX_AX = %i' % (self.nAxes-6))    
        
    def ProgFinish(self, progname, new_page = False):        
        if new_page:
            self.PROG = self.PROG + "END\n"
            self.PROG_LIST.append(self.PROG)
            self.PROG = ''
            self.nLines = 0
        elif self.nProgs <= 1 or self.INCLUDE_SUB_PROGRAMS:
            self.PROG = self.PROG + "END\n"
        
    def progsave(self, folder, progname, ask_user = False, show_result = False):
        progname = progname + '.' + self.PROG_EXT
        if ask_user or not DirExists(folder):
            filesave = getSaveFile(folder, progname, 'Save program as...')
            if filesave is not None:
                filesave = filesave.name
            else:
                return
        else:
            filesave = folder + '/' + progname
        fid = open(filesave, "w")
        fid.write("&ACCESS RVP\n")
        fid.write("&REL 1\n")
        fid.write("&COMMENT Generated by RoboDK\n")
        fid.write("&PARAM TEMPLATE = C:\\KRC\\Roboter\\Template\\vorgabe\n")
        fid.write("&PARAM EDITMASK = *\n")
        fid.write(self.PROG)
        fid.close()
        print('SAVED: %s\n' % filesave) # tell RoboDK the path of the saved file
        self.PROG_FILES.append(filesave)
        
        # open file with default application
        if show_result:
            if type(show_result) is str:
                # Open file with provided application
                import subprocess
                p = subprocess.Popen([show_result, filesave])
            elif type(show_result) is list:
                import subprocess
                p = subprocess.Popen(show_result + [filesave])   
            else:
                # open file with default application
                import os
                

                os.startfile(filesave)
            if len(self.LOG) > 0:
                mbox('Program generation LOG:\n\n' + self.LOG)

    def ProgSave(self, folder, progname, ask_user = False, show_result = False):
        if len(self.PROG_LIST) >= 1:
            if self.nLines > 0:
                self.PROG_LIST.append(self.PROG)
                self.PROG = ''
                self.nLines = 0
                
            npages = len(self.PROG_LIST)
            progname_main = progname + "Main"
            mainprog = "DEF %s ( )\n" % progname_main
            for i in range(npages):
                self.PROG = self.PROG_LIST[i]
                mainprog += "%s()\n" % self.PROG_NAMES[i]
                
            mainprog += "END\n"
            self.PROG = mainprog
            self.progsave(folder, progname_main, ask_user, show_result)
            self.LOG = ''
            if len(self.PROG_FILES) == 0:
                # cancelled by user
                return
                
            first_file = self.PROG_FILES[0]
            folder_user = getFileDir(first_file)
            # progname_user = getFileName(self.FILE_SAVED)
            
            for i in range(npages):
                self.PROG = self.PROG_LIST[i]
                self.progsave(folder_user, self.PROG_NAMES[i], False, show_result)
                
        else:
            self.progsave(folder, progname, ask_user, show_result)
            
    def ProgSendRobot(self, robot_ip, remote_path, ftp_user, ftp_pass):
        """Send a program to the robot using the provided parameters. This method is executed right after ProgSave if we selected the option "Send Program to Robot".
        The connection parameters must be provided in the robot connection menu of RoboDK"""
        UploadFTP(self.PROG_FILES, robot_ip, remote_path, ftp_user, ftp_pass)
        
    def MoveJ(self, pose, joints, conf_RLF=None):
        """Add a joint movement"""
        self.addline('PTP {' + angles_2_str(joints) + '}' + self.C_PTP)
        
    def MoveL(self, pose, joints, conf_RLF=None):
        """Add a linear movement"""
        self.addline('LIN {' + pose_2_str_ext(pose,joints) + '}' + self.C_DIS)
        
    def MoveC(self, pose1, joints1, pose2, joints2, conf_RLF_1=None, conf_RLF_2=None):
        """Add a circular movement"""
        self.addline('CIRC {' + pose_2_str_ext(pose1,joints1) + '},{' + pose_2_str_ext(pose2,joints2) + '}' + self.C_DIS)
        
    def setFrame(self, pose, frame_id=None, frame_name=None):
        """Change the robot reference frame"""
        if self.nAxes <= 6:
            self.addline('$BASE = {FRAME: ' + pose_2_str(pose) + '}')
        else:
            self.addline('$BASE = EK (MACHINE_DEF[2].ROOT, MACHINE_DEF[2].MECH_TYPE, { %s })' % pose_2_str(pose))
            #self.addline('$BASE=EK(EX_AX_DATA[1].ROOT,EX_AX_DATA[1].EX_KIN, { %s })' % pose_2_str(pose))
            #self.addline('$BASE=EK(EX_AX_DATA[1].ROOT,EX_AX_DATA[1].EX_KIN,EX_AX_DATA[1].OFFSET)')
        
    def setTool(self, pose, tool_id=None, tool_name=None):
        """Change the robot TCP"""
        self.addline('$TOOL = {FRAME: ' + pose_2_str(pose) + '}')
        
    def Pause(self, time_ms):
        """Pause the robot program"""
        if time_ms <= 0:
            self.addline('HALT')
        else:
            self.addline('WAIT SEC %.3f' % (time_ms*0.001))
        
    def setSpeed(self, speed_mms):
        """Changes the robot speed (in mm/s)"""
        self.addline('$VEL.CP = %.5f' % (speed_mms/1000.0))
    
    def setAcceleration(self, accel_mmss):
        """Changes the current robot acceleration"""
        self.addline('$ACC.CP = %.5f' % (accel_mmss/1000.0))
                
    def setSpeedJoints(self, speed_degs):
        """Changes the robot joint speed (in deg/s)"""
        self.addline('$VEL.ORI1 = %.5f' % speed_degs)
        self.addline('$VEL.ORI2 = %.5f' % speed_degs)
    
    def setAccelerationJoints(self, accel_degss):
        """Changes the robot joint acceleration (in deg/s2)"""
        self.addline('$ACC.ORI1 = %.5f' % accel_degss)
        self.addline('$ACC.ORI2 = %.5f' % accel_degss)
        
    def setZoneData(self, zone_mm):
        """Changes the zone data approach (makes the movement more smooth)"""
        self.APO_VALUE = zone_mm
        if zone_mm >= 0:
            self.addline('$APO.CPTP = %.3f' % zone_mm)
            self.addline('$APO.CDIS = %.3f' % zone_mm)
            self.C_DIS = ' C_DIS'
            self.C_PTP = ' C_PTP'
        else:
            self.C_DIS = ''
            self.C_PTP = ''
        
    def setDO(self, io_var, io_value):
        """Sets a variable (output) to a given value"""
        if type(io_var) != str:  # set default variable name if io_var is a number
            io_var = '$OUT[%s]' % str(io_var)        
        if type(io_value) != str: # set default variable value if io_value is a number            
            if io_value > 0:
                io_value = 'TRUE'
            else:
                io_value = 'FALSE'
        
        # at this point, io_var and io_value must be string values
        self.addline('%s=%s' % (io_var, io_value))
        
    def waitDI(self, io_var, io_value, timeout_ms=-1):
        """Waits for an input io_var to attain a given value io_value. Optionally, a timeout can be provided."""
        if type(io_var) != str:  # set default variable name if io_var is a number
            io_var = '$IN[%s]' % str(io_var)        
        if type(io_value) != str: # set default variable value if io_value is a number            
            if io_value > 0:
                io_value = 'TRUE'
            else:
                io_value = 'FALSE'
        
        # at this point, io_var and io_value must be string values
        if timeout_ms < 0:
            self.addline('WAIT FOR (%s==%s)' % (io_var, io_value))
        else:
            self.addline('START_TIMER:')
            self.addline('$TIMER_STOP[1]=TRUE')
            self.addline('$TIMER_FLAG[1]=FALSE')
            self.addline('$TIMER[1]=%.3f' % (float(timeout_ms)*0.001))
            self.addline('$TIMER_STOP[1]=FALSE')
            self.addline('WAIT FOR (%s==%s OR $TIMER_FLAG[1])' % (io_var, io_value))
            self.addline('$TIMER_STOP[1]=TRUE')
            self.addline('IF $TIMER_FLAG[1]== TRUE THEN')
            self.addline('    HALT ; Timed out!')
            self.addline('    GOTO START_TIMER')
            self.addline('ENDIF')      
        
    def RunCode(self, code, is_function_call = False):
        """Adds code or a function call"""
        if is_function_call:
            code.replace(' ','_')
            if not code.endswith(')'):
                code = code + '()'
            self.addline(code)
        else:
            self.addline(code)
        
    def RunMessage(self, message, iscomment = False):
        """Add a joint movement"""
        if iscomment:
            self.addline('; ' + message)
        else:
            self.addline('Wait for StrClear($LOOP_MSG[])')
            self.addline('$LOOP_CONT = TRUE')
            self.addline('$LOOP_MSG[] = "%s"' % message)
            
# ------------------ private ----------------------                       
    def addline(self, newline):
        """Add a program line"""
        if self.nProgs > 1 and not self.INCLUDE_SUB_PROGRAMS:
            return
        
        if self.nLines > self.MAX_LINES_X_PROG:
            self.nLines = 0
            self.ProgFinish(self.PROG_NAME, True)
            self.ProgStart(self.PROG_NAME, True)

        self.PROG = self.PROG + newline + '\n'
        self.nLines = self.nLines + 1
        
    def addlog(self, newline):
        """Add a log message"""
        if self.nProgs > 1 and not self.INCLUDE_SUB_PROGRAMS:
            return
            
        self.LOG = self.LOG + newline + '\n'

# -------------------------------------------------
# ------------ For testing purposes ---------------   
def Pose(xyzrpw):
    [x,y,z,r,p,w] = xyzrpw
    a = r*math.pi/180
    b = p*math.pi/180
    c = w*math.pi/180
    ca = math.cos(a)
    sa = math.sin(a)
    cb = math.cos(b)
    sb = math.sin(b)
    cc = math.cos(c)
    sc = math.sin(c)
    return Mat([[cb*ca, ca*sc*sb - cc*sa, sc*sa + cc*ca*sb, x],[cb*sa, cc*ca + sc*sb*sa, cc*sb*sa - ca*sc, y],[-sb, cb*sc, cc*cb, z],[0,0,0,1]])

def test_post():
    """Test the post with a basic program"""

    robot = RobotPost('Kuka_custom', 'Generic Kuka')

    robot.ProgStart("Program")
    robot.RunMessage("Program generated by RoboDK", True)
    robot.setFrame(Pose([807.766544, -963.699898, 41.478944, 0, 0, 0]))
    robot.setTool(Pose([62.5, -108.253175, 100, -60, 90, 0]))
    robot.MoveJ(Pose([200, 200, 500, 180, 0, 180]), [-46.18419, -6.77518, -20.54925, 71.38674, 49.58727, -302.54752] )
    robot.MoveL(Pose([200, 250, 348.734575, 180, 0, -150]), [-41.62707, -8.89064, -30.01809, 60.62329, 49.66749, -258.98418] )
    robot.MoveL(Pose([200, 200, 262.132034, 180, 0, -150]), [-43.73892, -3.91728, -35.77935, 58.57566, 54.11615, -253.81122] )
    robot.RunMessage("Setting air valve 1 on")
    robot.RunCode("TCP_On", True)
    robot.Pause(1000)
    robot.MoveL(Pose([200, 250, 348.734575, 180, 0, -150]), [-41.62707, -8.89064, -30.01809, 60.62329, 49.66749, -258.98418] )
    robot.MoveL(Pose([250, 300, 278.023897, 180, 0, -150]), [-37.52588, -6.32628, -34.59693, 53.52525, 49.24426, -251.44677] )
    robot.MoveL(Pose([250, 250, 191.421356, 180, 0, -150]), [-39.75778, -1.04537, -40.37883, 52.09118, 54.15317, -246.94403] )
    robot.RunMessage("Setting air valve off")
    robot.RunCode("TCP_Off", True)
    robot.Pause(1000)
    robot.MoveL(Pose([250, 300, 278.023897, 180, 0, -150]), [-37.52588, -6.32628, -34.59693, 53.52525, 49.24426, -251.44677] )
    robot.MoveL(Pose([250, 200, 278.023897, 180, 0, -150]), [-41.85389, -1.95619, -34.89154, 57.43912, 52.34162, -253.73403] )
    robot.MoveL(Pose([250, 150, 191.421356, 180, 0, -150]), [-43.82111, 3.29703, -40.29493, 56.02402, 56.61169, -249.23532] )
    robot.ProgFinish("Program")
    # robot.ProgSave(".","Program",True)
    print(robot.PROG)
    if len(robot.LOG) > 0:
        mbox('Program generation LOG:\n\n' + robot.LOG)

    input("Press Enter to close...")

if __name__ == "__main__":
    """Function to call when the module is executed by itself: test"""
    test_post()
