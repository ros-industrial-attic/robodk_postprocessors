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
# This file is a sample POST PROCESSOR script to generate robot programs for
# KUKA CNC
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

JOINT_DATA = ['X','Y','Z','A','B','C','X1=','Y1=','Z1=']

# ----------------------------------------------------
def pose_2_str(pose, joints = None):
    """Prints a pose target"""
    [x,y,z,r,p,w] = Pose_2_KUKA(pose)        
    str_xyzwpr = 'X%.4f Y%.4f Z%.4f A%.4f B%.4f C%.4f' % (x,y,z,r,p,w)
    if joints is not None:        
        if len(joints) > 6:
            for j in range(6,len(joints)):
                str_xyzwpr += (' %s%.6f ' % (JOINT_DATA[j], joints[j]))
    
    return str_xyzwpr
    
def joints_2_str(joints):
    """Prints a joint target"""
    str = ''
    for i in range(len(joints)):
        str = str + ('%s%.6f ' % (JOINT_DATA[i], joints[i]))
    str = str[:-1]
    return str

# ----------------------------------------------------    
# Object class that handles the robot instructions/syntax
class RobotPost(object):
    """Robot post object"""
    PROG_EXT = 'nc'        # set the program extension
    
    # other variables
    ROBOT_POST = ''
    ROBOT_NAME = ''
    PROG_FILES = []
    
    PROG = []
    LOG = ''
    nAxes = 6
    nId = 0
    SPEED_F = ' F400' # Set default speed
    MoveType = -1
    
    MOVE_TYPE_NONE = -1
    MOVE_NAMES = []
    MOVE_TYPE_MCS = 0
    MOVE_NAMES.append("MCS")
    MOVE_TYPE_HSC = 1
    MOVE_NAMES.append("HSC")
    MOVE_TYPE_PTP = 2
    MOVE_NAMES.append("PTP")
    
    
    REF_FRAME = eye(4)

    
    def __init__(self, robotpost=None, robotname=None, robot_axes = 6, **kwargs):
        self.ROBOT_POST = robotpost
        self.ROBOT_NAME = robotname
        self.PROG = []
        self.LOG = ''
        self.nAxes = robot_axes
        
    def ProgStart(self, progname):
        self.addline('%% program: %s()' % progname, False)
        self.addline('', False)
        self.addline('G90')
        self.addline('#HSC [BSPLINE PATH_DEV 0.0000 TRACK_DEV 0.0000]', False)
        self.addline('#CS OFF ALL')
        self.addline('#CS DEF[1][0.0000,0.0000,0.0000,0.0000,0.0000,0.0000]')
        self.addline('#CS ON[1]')
        self.addline('G54')
        self.addline('', False)
        
    def ProgFinish(self, progname):
        self.set_move_type(self.MOVE_TYPE_NONE)
        self.addline('%% End of program %s' % progname, False)
        
    def ProgSave(self, folder, progname, ask_user=False, show_result=False):
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
        for line in self.PROG:
            fid.write(line + '\n')
        fid.close()
        print('SAVED: %s\n' % filesave)
        self.PROG_FILES = filesave
        #---------------------- show result
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
    
    def ProgSendRobot(self, robot_ip, remote_path, ftp_user, ftp_pass):
        """Send a program to the robot using the provided parameters. This method is executed right after ProgSave if we selected the option "Send Program to Robot".
        The connection parameters must be provided in the robot connection menu of RoboDK"""
        UploadFTP(self.PROG_FILES, robot_ip, remote_path, ftp_user, ftp_pass)
    
    def set_move_type(self, move_type):
        if self.MoveType == move_type:
            return
            
        if self.MoveType >= 0:
            self.addline("#%s OFF" % self.MOVE_NAMES[self.MoveType])
            self.MoveType = -1
        
        if move_type >= 0:
            self.addline('',False) # add empty line
            self.addline("#%s ON" % self.MOVE_NAMES[move_type])
            self.MoveType = move_type
            
    def MoveJ(self, pose, joints, conf_RLF=None):
        """Add a joint movement"""
        self.set_move_type(self.MOVE_TYPE_MCS)
        self.addline('G0 ' + joints_2_str(joints))
        
        if pose is not None:
            self.set_move_type(self.MOVE_TYPE_PTP)
            self.addline('G0 ' + pose_2_str(pose, joints))
        
    def MoveL(self, pose, joints, conf_RLF=None):
        """Add a linear movement"""
        self.set_move_type(self.MOVE_TYPE_HSC)
        self.addline('G1 ' + pose_2_str(self.REF_FRAME*pose, joints) + self.SPEED_F)
        self.SPEED_F = ''
        
    def MoveC(self, pose1, joints1, pose2, joints2, conf_RLF_1=None, conf_RLF_2=None):
        """Add a circular movement"""
        xyz1 = (self.REF_FRAME*pose1).Pos()
        xyz2 = (self.REF_FRAME*pose2).Pos()        
        self.addline('G2 X%.3f Y%.3f Z%.3f I1=%.3f J1=%.3f K1=%.3f' % (xyz2[0], xyz2[1], xyz2[2], xyz1[0], xyz1[1], xyz1[2]))
        #self.addline('N%02i G102 X%.3f Y%.3f Z%.3f I1=%.3f J1=%.3f K1=%.3f' % (self.nId, xyz1[0], xyz1[1], xyz1[2], xyz2[0]-xyz1[0], xyz2[1]-xyz1[1], xyz2[2]-xyz1[2]))		
        
    def setFrame(self, pose, frame_id=None, frame_name=None):
        """Change the robot reference frame"""
        self.REF_FRAME = pose
        self.addline('%% Using Reference %s: %s' % (frame_name, pose_2_str(pose)), False)
        self.addline('%% (Using absolute coordinates)', False)
        
    def setTool(self, pose, tool_id=None, tool_name=None):
        """Change the robot TCP"""
        self.addline('', False)
        self.addline('%% Using Tool %s: %s' % (tool_name, pose_2_str(pose)), False)
        if tool_id < 1:
            tool_id = 99

        self.addline('M5')
        self.addline('#TRAFO OFF')
        self.addline('M6 T%i' % tool_id)
        self.addline('#TRAFO ON')
        self.addline('M3 S3501')
        self.addline('G131 100')
        self.addline('G133 150')
        self.addline('G134 150')
        self.addline('', False)
        
        
    def Pause(self, time_ms):
        """Pause the robot program"""
        if time_ms < 0:
            self.addline('%% PAUSE', False)
        else:
            self.addline('%% WAIT %.3f' % (time_ms*0.001), False)
    
    def setSpeed(self, speed_mms):
        """Changes the robot speed (in mm/s)"""
        self.SPEED_F = ' F%.3f' % (speed_mms*60)
    
    def setAcceleration(self, accel_mmss):
        """Changes the robot acceleration (in mm/s2)"""
        self.addlog('setAcceleration not defined')
    
    def setSpeedJoints(self, speed_degs):
        """Changes the robot joint speed (in deg/s)"""
        self.addlog('setSpeedJoints not defined')
    
    def setAccelerationJoints(self, accel_degss):
        """Changes the robot joint acceleration (in deg/s2)"""
        self.addlog('setAccelerationJoints not defined')
        
    def setZoneData(self, zone_mm):
        """Changes the rounding radius (aka CNT, APO or zone data) to make the movement smoother"""
        self.addlog('setZoneData not defined (%.1f mm)' % zone_mm)

    def setDO(self, io_var, io_value):
        """Sets a variable (digital output) to a given value"""
        if type(io_var) != str:  # set default variable name if io_var is a number
            io_var = 'OUT[%s]' % str(io_var)
        if type(io_value) != str: # set default variable value if io_value is a number
            if io_value > 0:
                io_value = 'TRUE'
            else:
                io_value = 'FALSE'

        # at this point, io_var and io_value must be string values
        self.addline('%s=%s' % (io_var, io_value))

    def waitDI(self, io_var, io_value, timeout_ms=-1):
        """Waits for a variable (digital input) io_var to attain a given value io_value. Optionally, a timeout can be provided."""
        if type(io_var) != str:  # set default variable name if io_var is a number
            io_var = 'IN[%s]' % str(io_var)
        if type(io_value) != str: # set default variable value if io_value is a number
            if io_value > 0:
                io_value = 'TRUE'
            else:
                io_value = 'FALSE'

        # at this point, io_var and io_value must be string values
        if timeout_ms < 0:
            self.addline('WAIT FOR %s==%s' % (io_var, io_value))
        else:
            self.addline('WAIT FOR %s==%s TIMEOUT=%.1f' % (io_var, io_value, timeout_ms))
        
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
        """Display a message in the robot controller screen (teach pendant)"""
        if iscomment:
            self.addline('%% ' + message, False)
        else:
            self.addline('%% Show message: %s' % message, False)
        
# ------------------ private ----------------------                
    def addline(self, newline, add_N = True):
        """Add a program line"""
        if add_N:
            self.nId += 1
            newline = 'N%i ' % self.nId + newline
        
        self.PROG.append(newline)
        
    def addlog(self, newline):
        """Add a log message"""
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

    robot = RobotPost()

    robot.ProgStart("Program")
    robot.RunMessage("Program generated by RoboDK using a custom post processor", True)
    robot.setFrame(Pose([807.766544, -963.699898, 41.478944, 0, 0, 0]), 3, 'Reference 2')
    robot.setTool(Pose([62.5, -108.253175, 100, -60, 90, 0]), 5, 'Tool 5')
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
    robot.MoveJ(None, [-46.18419, -6.77518, -20.54925, 71.38674, 49.58727, -302.54752] )
    robot.ProgFinish("Program")
    # robot.ProgSave(".","Program",True)
    for line in robot.PROG:
        print(line)
        
    if len(robot.LOG) > 0:
        mbox('Program generation LOG:\n\n' + robot.LOG)

    input("Press Enter to close...")

if __name__ == "__main__":
    """Function to call when the module is executed by itself: test"""
    test_post()
