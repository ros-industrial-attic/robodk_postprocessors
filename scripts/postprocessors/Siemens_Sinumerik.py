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
# This file is a sample POST PROCESSOR script to generate robot programs for a
# Siemens controller (Siemens Sinumerik programming language)
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
#     http://www.robodk.com/doc/PythonAPI/postprocessor.html
# ----------------------------------------------------


# ----------------------------------------------------
# Import RoboDK tools
from robodk import *

# ----------------------------------------------------    
# Object class that handles the robot instructions/syntax
class RobotPost(object):
    """Robot post object"""
    PROG_EXT = 'mpf'        # set the program extension
    
    # other variables
    ROBOT_POST = ''
    ROBOT_NAME = ''
    PROG_FILES = []
    
    PROG = ''
    LOG = ''
    nAxes = 6
    nId = 0
    REF_FRAME = eye(4)
    
    SPEED_MM_MIN = 100
    Nline = 0
    
    LAST_X = None
    LAST_Y = None
    LAST_Z = None
    LAST_A = None   
    LAST_B = None   
    LAST_C = None   
    
    
    # ----------------------------------------------------
    def pose_2_str(self, pose, remember_last=False):
        """Prints a pose target"""
        [x,y,z,a,b,c] = Pose_2_KUKA(pose)
        if remember_last:
            G_LINE = ''
            if self.LAST_X != x:
                G_LINE += 'X%.3f ' % x
            if self.LAST_Y != y:
                G_LINE += 'Y%.3f ' % y
            if self.LAST_Z != z or len(G_LINE) == 0:
                G_LINE += 'Z%.3f ' % z
            if self.LAST_A != a:
                G_LINE += 'A=%.3f ' % a
            if self.LAST_B != b:
                G_LINE += 'B=%.3f ' % b
            if self.LAST_C != c:
                G_LINE += 'C=%.3f ' % c
            self.LAST_X = x
            self.LAST_Y = y
            self.LAST_Z = z
            self.LAST_A = a   
            self.LAST_B = b   
            self.LAST_C = c               
            G_LINE = G_LINE[:-1]        
            return G_LINE
        else:
            return ('X%.3f Y%.3f Z%.3f A%.3f B%.3f C%.3f' % (x,y,z,a,b,c))
        
    def joints_2_str(self, joints):
        """Prints a joint target"""
        str = ''
        data = ['JT1','JT2','JT3','A','B','C','G','H','I','J','K','L']
        for i in range(len(joints)):
            str = str + ('%s %.6f ' % (data[i], joints[i]))
        str = str[:-1]
        return str
    
    
    def __init__(self, robotpost=None, robotname=None, robot_axes = 6, **kwargs):
        self.ROBOT_POST = robotpost
        self.ROBOT_NAME = robotname
        self.PROG = ''
        self.LOG = ''
        self.nAxes = robot_axes
        
    def ProgStart(self, progname):
        import datetime
        self.addcomment('FILE: %s' % progname)
        self.addcomment('CREATED: %s' % datetime.date.today())        
        self.addline('G17 G70 G90 G40 G64 ; XY plane selection, absolute, no tool radius compensation, look ahead')
        self.addline('M32')
        self.addline('STOPRE ; stop preprocessing')
        self.addline('R6=1.181')
        self.addcomment('R19 = Part Area in SqFt')
        self.addline('R19=0.00')
        self.addcomment('R20 = Linear Feet Cut')
        self.addline('R20=83.00')
        self.addline('STOPRE')
        self.addcomment('T1 BLADE1')
        self.addcomment('T1 D1 ; should be called automatically')
        self.addline('S1700.0 M80 ; set speed and power on')
        
        
    def ProgFinish(self, progname):
        self.addcomment('End of program ' + progname)
        
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
        fid.write(self.PROG)
        fid.close()
        print('SAVED: %s\n' % filesave)
        #---------------------- show result
        if show_result:
            if type(show_result) is str:
                # Open file with provided application
                import subprocess
                p = subprocess.Popen([show_result, filesave])   
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
        
    def MoveJ(self, pose, joints, conf_RLF=None):
        """Add a joint movement"""
        self.addline('TRAFOOF')
        self.addline('SUPA G0 G90 ' + self.joints_2_str(joints) + ' F%.1f' % self.SPEED_MM_MIN)
        self.addline('TRAORI')
        
    def MoveL(self, pose, joints, conf_RLF=None):
        """Add a linear movement"""
        if pose is None:
            self.addline('G1 ' + self.joints_2_str(joints) + ' F%.1f' % self.SPEED_MM_MIN)
        else:
            #self.addline('G1 ' + self.pose_2_str(self.REF_FRAME*pose, True) + ' F%.1f' % self.SPEED_MM_MIN)
            self.addline('G1 ' + self.pose_2_str(pose, True) + ' F%.1f' % self.SPEED_MM_MIN)
        
    def MoveC(self, pose1, joints1, pose2, joints2, conf_RLF_1=None, conf_RLF_2=None):
        """Add a circular movement"""
        self.nId = self.nId + 1
        #xyz1 = (self.REF_FRAME*pose1).Pos()
        #xyz2 = (self.REF_FRAME*pose2).Pos()  
        xyz1 = (pose1).Pos()
        xyz2 = (pose2).Pos()          
        self.addline('G2 X%.3f Y%.3f Z%.3f I1=%.3f J1=%.3f K1=%.3f' % (self.nId, xyz2[0], xyz2[1], xyz2[2], xyz1[0], xyz1[1], xyz1[2]) + ' F%.1f' % self.SPEED_MM_MIN)
        
    def setFrame(self, pose, frame_id=None, frame_name=None):
        """Change the robot reference frame"""
        self.REF_FRAME = pose
        self.addcomment('---------------------------- ')
        self.addcomment('Using local part reference ')
        self.addline('G500')
        [x,y,z,a,b,c] = Pose_2_KUKA(pose)
        self.addline('$P_uifr[10,x,tr]=%.5f' % x)
        self.addline('$P_uifr[10,y,tr]=%.5f' % y)
        self.addline('$P_uifr[10,z,tr]=%.5f' % z)
        self.addline('$P_uifr[10,z,rt]=%.5f' % c)
        self.addline('$P_uifr[10,y,rt]=%.5f' % b)
        self.addline('$P_uifr[10,x,rt]=%.5f' % a)
                
    def setTool(self, pose, tool_id=None, tool_name=None):
        """Change the robot TCP"""
        self.nId = self.nId + 1
        if tool_id is not None:
            self.addcomment('T%i' % tool_id)
        self.addcomment('---------------------------- ')
        self.addcomment('Using TCP')
        self.addline('TRAORI')
        [x,y,z,a,b,c] = Pose_2_KUKA(pose)
        self.addline('$TC_DP5[1,1]=%.5f' % x)
        self.addline('$TC_DP4[1,1]=%.5f' % y)
        self.addline('$TC_DP3[1,1]=%.5f' % z)
        self.addline('$TC_DPC3[1,1]=%.5f' % c)
        self.addline('$TC_DPC2[1,1]=%.5f' % b)
        self.addline('$TC_DPC1[1,1]=%.5f' % a)       
        
    def Pause(self, time_ms):
        """Pause the robot program"""
        if time_ms < 0:
            self.addcomment('PAUSE')
        else:
            self.addcomment('WAIT %.3f seconds' % (time_ms*0.001))
    
    def setSpeed(self, speed_mms):
        """Changes the robot speed (in mm/s)"""
        self.SPEED_MM_MIN = speed_mms/60.0
    
    def setAcceleration(self, accel_mmss):
        """Changes the robot acceleration (in mm/s2)"""
        self.addcomment('Acceleration set to %.3f mm/s2' % accel_mmss)
    
    def setSpeedJoints(self, speed_degs):
        """Changes the robot joint speed (in deg/s)"""
        self.addcomment('Joint speed set to %.3f deg/s' % speed_degs)
    
    def setAccelerationJoints(self, accel_degss):
        """Changes the robot joint acceleration (in deg/s2)"""
        self.addcomment('Joint acceleration set to %.3f deg/s2' % accel_degss)
        
    def setZoneData(self, zone_mm):
        """Changes the rounding radius (aka CNT, APO or zone data) to make the movement smoother"""
        self.addcomment('Look ahead desired tolerance: %.1f mm' % zone_mm)

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
        self.addcomment('%s=%s' % (io_var, io_value))

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
            self.addcomment('WAIT FOR %s==%s' % (io_var, io_value))
        else:
            self.addcomment('WAIT FOR %s==%s TIMEOUT=%.1f' % (io_var, io_value, timeout_ms))
        
    def RunCode(self, code, is_function_call = False):
        """Adds code or a function call"""
        if is_function_call:
            code.replace(' ','_')
            self.addcomment(code)
        else:
            self.addcomment(code)
        
    def RunMessage(self, message, iscomment = False):
        """Display a message in the robot controller screen (teach pendant)"""
        if iscomment:
            self.addcomment(message)
        else:
            self.addcomment('Display message: %s' % message)
        
# ------------------ private ----------------------                
    def addline(self, newline):
        """Add a program line"""
        self.Nline = self.Nline + 1
        self.PROG = self.PROG + ('N%02i ' % self.Nline) + newline + '\n'        
        
    def addcomment(self, newline):
        """Add a comment line"""
        self.PROG = self.PROG + '; ' + newline + '\n'
        
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
    robot.MoveJ(None, [-46.18419, -6.77518, -20.54925, 71.38674, 49.58727, -302.54752] )
    robot.ProgFinish("Program")
    # robot.ProgSave(".","Program",True)
    print(robot.PROG)
    if len(robot.LOG) > 0:
        mbox('Program generation LOG:\n\n' + robot.LOG)

    input("Press Enter to close...")

if __name__ == "__main__":
    """Function to call when the module is executed by itself: test"""
    test_post()
