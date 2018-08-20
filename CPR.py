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
# for a CPR robot (Common place robotics: http://www.cpr-robots.com/)
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
from robolink import *


# ----------------------------------------------------
def pose_2_str(pose):
    """Prints a pose target"""
    [x,y,z,r,p,w] = Pose_2_KUKA(pose)
    return ('x="%.3f" y="%.3f" z="%.3f" a="%.3f" b"%.3f" c="%.3f"' % (x,y,z,r,p,w))
    
def joints_2_str(joints):
    """Prints a joint target"""
    str = ''
    jnt_tags = ['a1', 'a2', 'a3', 'a4', 'a5', 'a6', 'a7', 'a8']
    for i in range(len(joints)):
        str = str + ('%s="%.5f" ' % (jnt_tags[i], joints[i]))
    str = str[:-1]
    return str

# ----------------------------------------------------    
# Object class that handles the robot instructions/syntax
class RobotPost(object):
    """Robot post object"""
    PROG_EXT = 'xml'        # set the program extension    
    
    # other variables
    ROBOT_POST = ''
    ROBOT_NAME = ''
    PROG_FILES = []    
    
    PROG = ''
    LOG = ''
    TAB = ''
    MAIN_PROG_NAME = None
    nAxes = 6
    Prog_count = 0
    SMOOTH = 'True'
    SPEED = 70
    SPEED_PERCENT = 35
    NR_ID = 0
    FRAME = eye(4)
    TOOL = eye(4)

    
    def __init__(self, robotpost=None, robotname=None, robot_axes = 6, **kwargs):
        self.ROBOT_POST = robotpost
        self.ROBOT_NAME = robotname
        self.PROG = ''
        self.LOG = ''
        self.nAxes = robot_axes
        
    def ProgStart(self, progname):
        self.Prog_count = self.Prog_count + 1
        self.addline('<?xml version="1.0" encoding="utf-8"?>')
        self.addline('<!-- values in mm and degree -->')
        self.addline('<Program>')
        self.addline('<Header ProgramName ="CPRog recording" Author="nn" SetUpDate=""  LastChangeDate="" Kinematic="CPRFour"/>')
        self.TAB = '    '        
        if self.MAIN_PROG_NAME is None:        
            self.MAIN_PROG_NAME = progname            
        else:
            pass
        
    def ProgFinish(self, progname):
        self.TAB = ''
        self.addline('</Program>')
        self.addline('<!-- End of program: %s -->' % progname)
        self.addline('')
        if self.Prog_count == 1:
            pass      
        
    def ProgSave(self, folder, progname, ask_user=False, show_result=False):
        import subprocess
        progname = progname + '.' + self.PROG_EXT
        if ask_user or not DirExists(folder):
            filesave = getSaveFile(folder, progname, 'Save program as...')
            if filesave is not None:
                filesave = filesave.name
            else:
                return
        else:
            filesave = folder + '/' + progname
        
        # retrieve robot IP
        RDK = Robolink()
        robot = RDK.Item(self.ROBOT_NAME, ITEM_TYPE_ROBOT)
        [server_ip, port, remote_path, username, password] = robot.ConnectionParams()            
            
        fid = open(filesave, "w")
        fid.write(self.PROG)        
        print('SAVED: %s\n' % filesave)
        self.PROG_FILES = filesave
        #---------------------- show result
        
        #selection_view = mbox('Do you want to view/edit the program or run it on the robot?', 'View', 'Run')            
        selection_view = True
        
        if selection_view and show_result:
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
        
        if not selection_view:
            RDK.ShowMessage('Running program...', False)
            proc = subprocess.Popen(['python', filesave])
            
            # Using the same pipe
            #import io
            #proc = subprocess.Popen(['python', filesave], stdout=subprocess.PIPE)
            #for line in io.TextIOWrapper(proc.stdout, encoding="utf-8"):  # or another encoding
            #    RDK.ShowMessage(line, False)
        
        
    def ProgSendRobot(self, robot_ip, remote_path, ftp_user, ftp_pass):
        """Send a program to the robot using the provided parameters. This method is executed right after ProgSave if we selected the option "Send Program to Robot".
        The connection parameters must be provided in the robot connection menu of RoboDK"""
        import subprocess
        import os
        import sys
        import time
        
        print("POPUP: Starting process...")     
        print("Python path " + PATH_PYTHON)
        print("Program file: " + self.PROG_FILES)
        sys.stdout.flush()
        
        # Start process
        cmd_run = self.PROG_FILES # run py file itself
        if PATH_PYTHON != '':
            # if a python path is specified, use it to run the Py file
            cmd_run = [PATH_PYTHON, self.PROG_FILES]
            
        process = subprocess.Popen(cmd_run, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)
        
        # Poll process for new output until finished
        for stdout_line in iter(process.stdout.readline, ""):
            print("POPUP: " + stdout_line.strip())
            sys.stdout.flush()
            
        process.stdout.close()
        return_code = process.wait()
        if return_code:
            raise subprocess.CalledProcessError(return_code, cmd_run)

        if (return_code == 0):
            return
        else:
            raise ProcessException(command, return_code, output)
        
    def MoveJ(self, pose, joints, conf_RLF=None):
        """Add a joint movement"""
        self.NR_ID = self.NR_ID + 1
        self.addline('<Joint Nr="%i" %s velPercent="%.0f" acc="40" smooth="%s" Descr="" />' % (self.NR_ID, joints_2_str(joints), self.SPEED_PERCENT, self.SMOOTH))

    def MoveL(self, pose, joints, conf_RLF=None):
        """Add a linear movement"""
        self.NR_ID = self.NR_ID + 1
        if pose is None:
            # no linear movement allowed by providing joints
            self.addline('<Joint Nr="%i" %s velPercent="%.0f" acc="40" smooth="%s" Descr="" />' % (self.NR_ID, joints_2_str(joints), self.SPEED_PERCENT, self.SMOOTH))
        else:
            self.addline('<Joint Nr="%i" %s vel="%.0f" acc="40" smooth="%s" Descr="" />' % (self.NR_ID, pose_2_str(self.FRAME*pose*invH(self.TOOL)), self.SPEED, self.SMOOTH))
        
    def MoveC(self, pose1, joints1, pose2, joints2, conf_RLF_1=None, conf_RLF_2=None):
        """Add a circular movement"""
        self.addlog("Cicular moves not supported")
        
    def setFrame(self, pose, frame_id=None, frame_name=None):
        """Change the robot reference frame"""
        self.FRAME = pose
        self.addline("<!-- Using Reference: %s -->" % pose_2_str(pose))
        
    def setTool(self, pose, tool_id=None, tool_name=None):
        """Change the robot TCP"""
        self.TOOL = pose
        self.addline("<!-- Using Tool: %s -->" % pose_2_str(pose))
        
    def Pause(self, time_ms):
        """Pause the robot program"""
        self.NR_ID = self.NR_ID + 1
        if time_ms < 0:
            self.addline('<Wait Nr="%i" Seconds="%.3f" Descr="" />' % (self.NR_ID, float(9999)))
        else:
            #self.addline('time.sleep(%.3f)' % (float(time_ms)*0.001))
            self.addline('<Wait Nr="%i" Seconds="%.3f" Descr="" />' % (self.NR_ID, float(time_ms)*0.001))
    
    def setSpeed(self, speed_mms):
        """Changes the robot speed (in mm/s)"""
        self.SPEED_PERCENT = speed_mms
        
    def setAcceleration(self, accel_mmss):
        """Changes the robot acceleration (in mm/s2)"""
        self.addlog('Linear acceleration not supported')
    
    def setSpeedJoints(self, speed_degs):
        """Changes the robot joint speed (in deg/s)"""
        self.SPEED_PERCENT = min(speed_degs, 100)
    
    def setAccelerationJoints(self, accel_degss):
        """Changes the robot joint acceleration (in deg/s2)"""
        self.addlog('Joint acceleration not supported')
        
    def setZoneData(self, zone_mm):
        """Changes the smoothing radius (aka CNT, APO or zone data) to make the movement smoother"""
        if zone_mm > 0:
            self.SMOOTH = 'True'
        else:
            self.SMOOTH = 'False'

    def setDO(self, io_var, io_value):
        """Sets a variable (digital output) to a given value"""
        if type(io_var) != str:  # set default variable name if io_var is a number
            io_var = '%s' % str(io_var)
        if type(io_value) != str: # set default variable value if io_value is a number
            if io_value > 0:
                io_value = 'True'
            else:
                io_value = 'False'

        self.NR_ID = self.NR_ID + 1
        # at this point, io_var and io_value must be string values
        # self.addline('%s=%s' % (io_var, io_value))
        self.addline('<Output Nr="%i" Local="True" DIO="%s" State="%s" Descr="" />' % (self.NR_ID, io_var, io_value))
        
    def waitDI(self, io_var, io_value, timeout_ms=-1):
        """Waits for a variable (digital input) io_var to attain a given value io_value. Optionally, a timeout can be provided."""
        if type(io_var) != str:  # set default variable name if io_var is a number
            io_var = '%s' % str(io_var)
        if type(io_value) != str: # set default variable value if io_value is a number
            if io_value > 0:
                io_value = 'True'
            else:
                io_value = 'False'

        self.NR_ID = self.NR_ID + 1
        # at this point, io_var and io_value must be string values
        self.addline('<WaitInput Nr="%i" Local="True" DIO="%s" State="%s" Descr="" />' % (self.NR_ID, io_var, io_value))
        
    
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
            self.addline('<!-- %s -->' % message)
        else:
            self.addline('<!-- %s -->' % message)
        
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
