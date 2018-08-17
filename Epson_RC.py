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
# for an Epson robot with RC controller and RoboDK
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
    """Prints a pose target"""
    [x,y,z,a,b,c] = Pose_2_Staubli(pose)
    return ('XY(%.3f, %.3f, %.3f, %.3f, %.3f, %.3f)' % (x,y,z,c,b,a))
    
def angles_2_str(angles):
    """Prints a joint target"""
    str = ''
    for i in range(len(angles)):
        #data[3] = data[3] + -0.00
        str = str + ('%.3f,' %  (angles[i]))
    str = str[:-1]
    return "AglToPls(" + str + ")"

# ----------------------------------------------------    
# Object class that handles the robot instructions/syntax
class RobotPost(object):
    """Robot post object"""
    PROG_EXT = 'gpl'        # set the program extension
    
    # other variables
    # ROBOT_POST = 'unset'
    ROBOT_NAME = ''
    PROG_FILES = []
    
    nPROGS = 0
    PROG = ''
    TAB = ''
    LOG = ''
    nAxes = 6
    FRAME = eye(4)
    TOOL = eye(4)

    
    def __init__(self, robotpost=None, robotname=None, robot_axes = 6, **kwargs):
        #self.ROBOT_POST = robotpost
        self.ROBOT_NAME = robotname
        self.PROG = ''
        self.LOG = ''
        self.nAxes = robot_axes
        
    def ProgStart(self, progname):
        self.nPROGS = self.nPROGS + 1
        if self.nPROGS == 1:
            self.addline('Function %s' % progname)
            self.TAB = '    '
            self.addline('Motor On')
            self.addline('Power High')            
        
    def ProgFinish(self, progname):
        self.TAB = ''
        self.addline('Fend')
        
    def ProgSave(self, folder, progname, ask_user=True, show_result=True):
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
        
    def MoveJ(self, pose, joints, conf_RLF=None):
        """Add a joint movement"""
        #poseabs = self.FRAME * pose * invH(self.TOOL)
        # Use of Move AglToPls(,,,) is also possible
        self.addline('Go %s' % angles_2_str(joints))
        
    def MoveL(self, pose, joints, conf_RLF=None):
        """Add a linear movement"""
        poseabs = self.FRAME * pose
        self.addline('TGo %s' % pose_2_str(poseabs))
        
    def MoveC(self, pose1, joints1, pose2, joints2, conf_RLF_1=None, conf_RLF_2=None):
        """Add a circular movement"""
        poseabs1 = self.FRAME * pose1
        poseabs2 = self.FRAME * pose2
        self.addline('Arc %s, %s ' % (pose_2_str(poseabs1), pose_2_str(poseabs2)))        
        
    def setFrame(self, pose, frame_id = None, frame_name = None):
        """Change the robot reference frame"""
        self.FRAME = pose
        self.addline("' Using reference frame %s: %s" % (frame_name, pose_2_str(pose)))
        
    def setTool(self, pose, tool_id = None, tool_name = None):
        """Change the robot TCP"""
        self.TOOL = pose
        #self.addline('Tool ' + pose_2_str(pose))
        pose_str = pose_2_str(pose)
        self.addline("' Using Tool frame %s: %s" % (tool_name, pose_str))
        if tool_id <= 0:
            tool_id = 1
        
        self.addline('TLSet ' + str(tool_id) + ", " + pose_str)
        self.addline('Tool ' + str(tool_id))
        
    def Pause(self, time_ms):
        """Pause the robot program"""
        self.addline('Wait %.3f' % (time_ms*0.001))
    
    def setSpeed(self, speed_mms):
        """Changes the robot speed (in mm/s)"""
        self.addline('Speed %.2f' % speed_mms)
    
    def Acceleration(self, accel_ptg):
        """Changes the robot acceleration (in %)"""
        accel_ptg = min(accel_ptg, 100)
        self.addline('Accel %.2f, %.2f' % (accel_ptg, accel_ptg))
    
    def setSpeedJoints(self, speed_degs):
        """Changes the robot joint speed (in deg/s)"""
        pass
        #self.addlog('setSpeedJoints not defined')
    
    def setAccelerationJoints(self, accel_degss):
        """Changes the robot joint acceleration (in deg/s2)"""
        pass
        # self.addlog('setAccelerationJoints not defined')
        
    def setZoneData(self, zone_mm):
        """Changes the zone data approach (makes the movement more smooth)"""
        pass
        #self.addlog('setZoneData not defined (%.1f mm)' % zone_mm)
        
    def setDO(self, io_var, io_value):
        """Sets a variable (output) to a given value"""
        if type(io_var) != str:  # set default variable name if io_var is a number
            io_var = '%s' % str(io_var)        
        if type(io_value) != str: # set default variable value if io_value is a number            
            if io_value > 0:
                io_value = 'On'
            else:
                io_value = 'Off'
        
        # at this point, io_var and io_value must be string values
        self.addline('%s %s' % (io_value, io_var))
        
    def waitDI(self, io_var, io_value, timeout_ms=-1):
        """Waits for an input io_var to attain a given value io_value. Optionally, a timeout can be provided."""
        if type(io_var) != str:  # set default variable name if io_var is a number
            io_var = 'In(%s)' % str(io_var)        
        if type(io_value) != str: # set default variable value if io_value is a number            
            if io_value > 0:
                io_value = 'On'
            else:
                io_value = 'Off'
        
        # at this point, io_var and io_value must be string values
        if timeout_ms < 0:
            self.addline('Wait %s = %s' % (io_var, io_value))
        else:
            self.addline('Wait %s = %s Timeout = %.1f' % (io_var, io_value, timeout_ms))   
        
    def RunCode(self, code, is_function_call = False):
        """Adds code or a function call"""
        if is_function_call:
            code.replace(' ','_')
            self.addline("Call " + code)
        else:
            self.addline("Xqt 2, " + code)
        
    def RunMessage(self, message, iscomment = False):
        """Display a message in the robot controller screen (teach pendant)"""
        if iscomment:
            self.addline("' " + message)
        else:
            self.addline('Print "%s"' % message)
        
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
   
    robot.MoveJ(Pose([200, 200, 500, 180, 0, 180]), [-46.18419, -6.77518, -20.54925, 71.38674, 49.58727, -302.54752] )
    robot.MoveL(Pose([200, 250, 15, 180, 0, -150]), [-41.62707, -8.89064, -30.01809, 60.62329, 49.66749, -258.98418] )
    robot.MoveC(Pose([200, 200, 34, 180, 0, -150]), [-43.73892, -3.91728, -35.77935, 58.57566, 54.11615, -253.81122],Pose([200, 200, 15, 180, 0, -150]), [-43.73892, -3.91728, -35.77935, 58.57566, 54.11615, -253.81122])
    #robot.setFrame(Pose([192.305408, 222.255946, 0.000000, 0.000000, 0.000000, 0.000000]),5,'Frame 5')
    robot.setSpeed(30)
    #robot.Delay(1000)
    
    #robot.ProgFinish("Program")
    #robot.ProgSave(".","Program",True)
    print(robot.PROG)
    if len(robot.LOG) > 0:
        mbox('Program generation LOG:\n\n' + robot.LOG)

    input("Press Enter to close...")

if __name__ == "__main__":
    """Function to call when the module is executed by itself: test"""
    test_post()
