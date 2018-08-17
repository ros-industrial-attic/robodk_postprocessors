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
# This file is POST PROCESSOR for Robot Offline Programming to generate programs
# for Denso robots using RC8 controller (PACScript programming)
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

# ----------------------------------------------------
def pose_2_str(pose):
    """Converts a pose target to a string"""
    [x,y,z,w,p,r] = Pose_2_Adept(pose)
    return ('P(%.3f,%.3f,%.3f,%.3f,%.3f,%.3f)' % (x,y,z,w,p,r))
    
def joints_2_str(angles):
    """Contverts a joint target to a string"""
    return 'J(%s)' % (','.join(format(ji, ".5f") for ji in angles))

# ----------------------------------------------------    
# Object class that handles the robot instructions/syntax
class RobotPost(object):
    """Robot post object"""
    PROG_EXT = 'pcs'        # set the program extension
    
    # other variables
    ROBOT_POST = ''
    ROBOT_NAME = ''
    PROG_FILES = []
    
    PROG = ''
    LOG = ''
    nAxes = 6
    TAB = ''
    SPEED_MPS = 10
    PASS = '@0' # Nothing = default = '@0'
    ACTIVE_TOOL = 2
    ACTIVE_WOBJ = 3
    nProgs = 0
    
    def __init__(self, robotpost=None, robotname=None, robot_axes = 6, **kwargs):
        self.ROBOT_POST = robotpost
        self.ROBOT_NAME = robotname
        self.PROG = ''
        self.LOG = ''
        self.nAxes = robot_axes
        
    def ProgStart(self, progname):
        #self.addline('Sub Main%s()' % progname)
        self.nProgs = self.nProgs + 1
        if self.nProgs <= 1:
            self.addline('Sub Main')
        else:
            self.addline('Sub %s' % progname)
        self.TAB = '  '
        
    def ProgFinish(self, progname):
        self.TAB = ''
        self.addline('End Sub')
        
    def ProgSave(self, folder, progname, ask_user = False, show_result = False):
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
        print('SAVED: %s\n' % filesave) # tell RoboDK the path of the saved file
        self.PROG_FILES = filesave
        
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
        
    def ProgSendRobot(self, robot_ip, remote_path, ftp_user, ftp_pass):
        """Send a program to the robot using the provided parameters. This method is executed right after ProgSave if we selected the option "Send Program to Robot".
        The connection parameters must be provided in the robot connection menu of RoboDK"""
        UploadFTP(self.PROG_FILES, robot_ip, remote_path, ftp_user, ftp_pass)
        
    def MoveJ(self, pose, joints, conf_RLF=None):
        """Add a joint movement"""
        #MOVE P, @P J(20.007, 70.475, 85.691, -137.266, 18.729, 64.496), Speed=20
        self.addline('MOVE P, %s %s, Speed=20' % (self.PASS, joints_2_str(joints)))
        
    def MoveL(self, pose, joints, conf_RLF=None):
        """Add a linear movement"""
        #MOVE L,@P P( -28.99, 20, 67.243, -90, -50.442, -180)
        if pose is None:
            self.addline('MOVE L, %s %s, Speed=20' % (self.PASS, joints_2_str(joints)))
        else:
            self.addline('MOVE L, %s %s, Speed=Mps(%.1f)' % (self.PASS, pose_2_str(pose), self.SPEED_MPS))
        
    def MoveC(self, pose1, joints1, pose2, joints2, conf_RLF_1=None, conf_RLF_2=None):
        """Add a circular movement"""
        #self.addlog('Circular move is not supported!')
        #MOVE C, P( -25.306, 20, 68.895, -90, -49.948, -180),@[5] P( -19.256, 20, 62, -90, -50.06, -180), Speed=Mps(3)
        self.addline('MOVE C, %s,%s %s, Speed=Mps(%.1f)' % (pose_2_str(pose1), self.PASS, pose_2_str(pose1), self.SPEED_MPS))
        
    def setFrame(self, pose, frame_id=None, frame_name=None):
        """Change the robot reference frame"""
        if frame_id is not None and frame_id > 0:
            self.ACTIVE_TOOL = frame_id
        self.addline('Work %i, %s' % (self.ACTIVE_TOOL, pose_2_str(pose)))
        self.addline('ChangeWork %i' % self.ACTIVE_TOOL)
           
    def setTool(self, pose, tool_id=None, tool_name=None):
        """Change the robot TCP"""
        if tool_id is not None and tool_id > 0:
            self.ACTIVE_WOBJ = frame_id
        self.addline('Tool %i, %s' % (self.ACTIVE_WOBJ, pose_2_str(pose)))
        self.addline('ChangeTool %i' % self.ACTIVE_WOBJ)     
        
    def Pause(self, time_ms):
        """Pause the robot program"""
        if time_ms <= 0:
            self.addline('PAUSE') # Important for calibration
        else:
            self.addline('DELAY %i' % time_ms)
    
    def setSpeed(self, speed_mms):
        """Changes the robot speed (in mm/s)"""
        self.SPEED_MPS = speed_mms/1000.0
    
    def setAcceleration(self, accel_mmss):
        """Changes the robot acceleration (in mm/s2)"""
        # Warning! Here, acceleration in mmss is considered as a percentage!
        #accel_percentage = min(100,max(0,accel_mmss))
        #self.addline('ACCEL %.1f, %.1f' % (accel_percentage, accel_percentage))
        pass
    
    def setSpeedJoints(self, speed_degs):
        """Changes the robot joint speed (in deg/s)"""
        #self.addlog('setSpeedJoints not defined')
        pass
    
    def setAccelerationJoints(self, accel_degss):
        """Changes the robot joint acceleration (in deg/s2)"""
        #self.addlog('setAccelerationJoints not defined')
        pass
        
    def setZoneData(self, zone_mm):
        """Changes the zone data approach (makes the movement more smooth)"""
        #self.addlog('setZoneData not defined (%.1f mm)' % zone_mm)
        # Encoder value check / pass @0 (fine), @P (approximated), @value (pulse value approximation) or @E
        if zone_mm <=0:
            self.PASS = '@0'
        else:
            self.PASS = '@P'
            #self.PASS = '@[%i]' % zone_mm # Define zone data in pulses
        
    def setDO(self, io_var, io_value):
        """Sets a variable (output) to a given value"""
        if type(io_var) != str:  # set default variable name if io_var is a number
            io_var = 'IO[%s]' % str(io_var)        
        if type(io_value) != str: # set default variable value if io_value is a number            
            if io_value > 0:
                io_value = 'ON'
            else:
                io_value = 'OFF'
        
        # at this point, io_var and io_value must be string values
        self.addline('OUT %s = %s' % (io_var, io_value))
        
    def waitDI(self, io_var, io_value, timeout_ms=-1):
        """Waits for an input io_var to attain a given value io_value. Optionally, a timeout can be provided."""
        if type(io_var) != str:  # set default variable name if io_var is a number
            io_var = 'IO[%s]' % str(io_var)        
        if type(io_value) != str: # set default variable value if io_value is a number            
            if io_value > 0:
                io_value = 'ON'
            else:
                io_value = 'OFF'
        
        # at this point, io_var and io_value must be string values
        if timeout_ms < 0:
            self.addline('WAIT %s = %s' % (io_var, io_value))
        else:
            self.addline('WAIT %s = %s, %.0f' % (io_var, io_value, timeout_ms))   
        
    def RunCode(self, code, is_function_call = False):
        """Adds code or a function call"""
        if is_function_call:
            code.replace(' ','_')
            if code.find('(') < 0:
                code = code + '()'
            self.addline(code)
        else:
            self.addline(code)
        
    def RunMessage(self, message, iscomment = False):
        """Add a joint movement"""
        if iscomment:
            self.addline('\' ' + message)
        else:
            self.addline('\'Output "' + message + '"')
        
# ------------------ private ----------------------                
    def addline(self, newline):
        """Add a program line"""
        self.PROG = self.PROG + self.TAB + newline + '\n'
        
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

    robot = RobotPost('Vplus_custom', 'Generic Adept Robot')

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
    robot.RunCode("TCP_Off(55)", True)
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
