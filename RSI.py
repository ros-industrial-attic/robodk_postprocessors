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
# for Robotic Systems Integration controllers, targeted to Comau and other robot brands
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
    [x,y,z,w,p,r] = Pose_2_TxyzRxyz(pose)
    return ('pose([%.6f, %.6f, %.6f, %.6f, %.6f, %.6f])' % (x,y,z,r,p,w))
    
def angles_2_str(angles):
    """Prints a joint target"""
    njoints = len(angles)
    if njoints == 6:
        return ('[%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]' % (angles[0], angles[1], angles[2], angles[3], angles[4], angles[5]))
    else:
        return 'this post only supports 6-axis robots'

# ----------------------------------------------------    
# Object class that handles the robot instructions/syntax
class RobotPost(object):
    """Robot post object"""
    PROG_EXT = 'txt'        # set the program extension
    SPEED_MMS       = 0.3    # default speed for linear moves in m/s
    SPEED_DEGS      = 0.75   # default speed for joint moves in rad/s
    ACCEL_MMSS      = 3      # default acceleration for lineaer moves in m/ss
    ACCEL_DEGSS     = 1.2    # default acceleration for joint moves in rad/ss
    BLEND_RADIUS_MM = 0.001  # default smooth radius in mm (corners smoothing)
    REF_FRAME       = eye(4) # default reference frame (the robot reference frame)
    MOVE_COUNT = 0
    
    # other variables
    ROBOT_POST = 'unset'
    ROBOT_NAME = 'generic'
    PROG_FILES = []
    MAIN_PROGNAME = 'unknown'
    PROG = ''
    TAB = ''
    LOG = ''    
    
    def __init__(self, robotpost=None, robotname=None, robot_axes = 6, **kwargs):
        self.ROBOT_POST = robotpost
        self.ROBOT_NAME = robotname
        self.nPROGS = 0
        self.PROG = ''
        self.LOG = ''
        
    def ProgStart(self, progname):
        self.nPROGS = self.nPROGS + 1
        self.addline('def %s():' % progname)        
        self.TAB = '    '
        if self.nPROGS == 1:
            self.MAIN_PROGNAME = progname
            self.addline('# default parameters:')
            self.addline('global speed_mms    = %.3f' % self.SPEED_MMS)
            self.addline('global speed_degs   = %.3f' % self.SPEED_DEGS)    
            self.addline('global accel_mmss   = %.3f' % self.ACCEL_MMSS)
            self.addline('global accel_degss  = %.3f' % self.ACCEL_DEGSS)
            self.addline('global smooth_radius_mm = %.3f' % self.BLEND_RADIUS_MM)        
        
    def ProgFinish(self, progname):
        self.TAB = ''
        self.addline('')
                
    def ProgSave(self, folder, progname, ask_user = False, show_result = False):
        self.addline('%s()' % self.MAIN_PROGNAME)
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
        
        # write the joint list of points as a separate file
        RDK = Robolink()
        progitem = RDK.Item(self.MAIN_PROGNAME, ITEM_TYPE_PROGRAM)
        filesave_control = filesave + '.control'
        if not progitem.Valid():
            error_msg = 'Problems getting the program'
            error_code = -1
        else:
            error_msg, joint_list, error_code = progitem.InstructionListJoints(0.5, 0.5, filesave + '.control')
            
        if error_code != 0:
            mbox('Program issues:\n\n' + error_msg)
            
        # open file with default application
        if show_result:
            if type(show_result) is str:
                # Open file with provided application
                import subprocess
                p = subprocess.Popen([show_result, filesave])
                p = subprocess.Popen([show_result, filesave_control])               
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
        self.MOVE_COUNT = self.MOVE_COUNT + 1
        self.addline('movej(%s,accel_degss,speed_degs,0,smooth_radius_mm) # move_id = %.0f' % (angles_2_str(joints), self.MOVE_COUNT))
        
    def MoveL(self, pose, joints, conf_RLF=None):
        """Add a linear movement"""
        self.MOVE_COUNT = self.MOVE_COUNT + 1
        # Movement in joint space or Cartesian space should give the same result:
        self.addline('movel(%s,accel_mss,speed_mms,0,smooth_radius_mm) # move_id = %.0f' % (angles_2_str(joints), self.MOVE_COUNT))
        
    def MoveC(self, pose1, joints1, pose2, joints2, conf_RLF_1=None, conf_RLF_2=None):
        """Add a circular movement"""
        self.MOVE_COUNT = self.MOVE_COUNT + 1
        self.addline('movec(%s,%s,accel_mmss,speed_mms,0,smooth_radius_mm) # move_id = %.0f' % (angles_2_str(joints1),angles_2_str(joints2), self.MOVE_COUNT))
        
    def setFrame(self, pose, frame_id=None, frame_name=None):
        """Change the robot reference frame"""
        # the reference frame is not needed if we use joint space for joint and linear movements
        # the reference frame is also not needed if we use cartesian moves with respect to the robot base frame
        # the cartesian targets must be pre-multiplied by the active reference frame
        self.REF_FRAME = pose    
        self.addline('set_reference(%s)' % pose_2_str(pose))
        
    def setTool(self, pose, tool_id=None, tool_name=None):
        """Change the robot TCP"""
        self.addline('set_tcp(%s)' % pose_2_str(pose))
        
    def Pause(self, time_ms):
        """Pause the robot program"""
        if time_ms <= 0:
            self.addline('halt()')
        else:
            self.addline('sleep(%.3f)' % (time_ms*0.001))
        
    def setSpeed(self, speed_mms):
        """Changes the robot speed (in mm/s)"""
        self.addline('speed_mms    = %.3f' % speed_mms)
        
    def setAcceleration(self, accel_mmss):
        """Changes the robot acceleration (in mm/s2)"""    
        self.addline('accel_mmss   = %.3f' % accel_mmss)
        
    def setSpeedJoints(self, speed_degs):
        """Changes the robot joint speed (in deg/s)"""
        self.addline('speed_degs  = %.3f' % speed_degs)
    
    def setAccelerationJoints(self, accel_degss):
        """Changes the robot joint acceleration (in deg/s2)"""
        self.addline('accel_degss = %.3f' % accel_degss)
        
    def setZoneData(self, zone_mm):
        """Changes the zone data approach (makes the movement more smooth)"""
        if zone_mm < 0:
            zone_mm = 0            
        self.BLEND_RADIUS_M = zone_mm / 1000.0
        self.addline('smooth_radius_mm = %.3f' % self.BLEND_RADIUS_M)
        
    def setDO(self, io_var, io_value):
        """Sets a variable (output) to a given value"""
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
        """Waits for an input io_var to attain a given value io_value. Optionally, a timeout can be provided."""
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
            self.addline(code + '()')
        else:
            #self.addline(code)
            self.addline('# ' + code) #generate custom code as a comment
        
    def RunMessage(self, message, iscomment = False):
        """Show a message on the controller screen"""
        if iscomment:
            self.addline('# ' + message)
        else:
            self.addline('popup("%s")' % message)
        
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

    robot = RobotPost('Universal Robotics', 'Generic UR robot')

    robot.ProgStart("Program")
    robot.RunMessage("Program generated by RoboDK", True)
    robot.setFrame(Pose([807.766544, -963.699898, 41.478944, 0, 0, 0]))
    robot.setTool(Pose([62.5, -108.253175, 100, -60, 90, 0]))
    robot.setSpeed(100) # set speed to 100 mm/s
    robot.setAcceleration(3000) # set speed to 3000 mm/ss    
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
