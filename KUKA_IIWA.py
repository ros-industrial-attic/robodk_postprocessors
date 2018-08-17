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
# for a Kuka IIWA robot (7 axis) with RoboDK
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


HEADER = '''package application;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.deviceModel.LBRE1Redundancy;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;

import com.kuka.roboticsAPI.motionModel.LIN;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.motionModel.Spline;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;

public class %s extends RoboticsAPIApplication {
\tprivate LBR robot;
\tprivate Tool TOOL;
\t// private Frame FRAME;
\t
\tpublic void initialize() {		
\t\trobot = getContext().getDeviceFromType(LBR.class);
\t}
\tpublic void run() {
\t\tSpline move_curve;
'''

# ----------------------------------------------------
def pose_2_str(pose, rot_in_deg = False):
    """Converts a pose target to a string"""
    torad = pi/180.0
    if rot_in_deg:
        torad = 1.0
    #[x,y,z,w,p,r] = Pose_2_KUKA(pose)
    [x,y,z,r,p,w] = pose_2_xyzrpw(pose)
    return ('%.3f,%.3f,%.3f,%.5f,%.5f,%.5f' % (x,y,z,w*torad,p*torad,r*torad))

def pose_2_str_ext(pose,joints):
    njoints = len(joints)
    if njoints <= 7:
        return pose_2_str(pose)
    else:     
        extaxes = ''
        for i in range(njoints-7):
            extaxes = extaxes + (',%.5f' % (i+1, joints[i+7]))
        return pose_2_str(pose) + extaxes
    
def angles_2_str(angles):
    """Prints a joint target"""
    str = ''
    for i in range(len(angles)):
        str = str + ('%.5f,' % (angles[i]*pi/180.0))
    str = str[:-1]
    return str

# ----------------------------------------------------    
# Object class that handles the robot instructions/syntax
class RobotPost(object):
    """Robot post object"""
    PROG_EXT = 'java'        # set the program extension
    
    # other variables
    ROBOT_POST = ''
    ROBOT_NAME = ''
    PROG_FILES = []
    
    PROG = ''
    LOG = ''
    nAxes = 7
    MOVE_OBJECT = 'robot'
    REF_FRAME = eye(4)
    TARGET_PTP_id = 0
    TARGET_LIN_id = 0 
    BLENDING_MM = 0    
    SPEED_MMS = 300
    SPEED_RADS = 0.250
    ACCEL_MMSS = 500
    TAB = '\t\t'
    MAIN_DONE = False
    SPLINE = ''
    SPLINE_COUNT = 0
    SPLINE_LAST_FRAME = ''
    
    
    
    def __init__(self, robotpost=None, robotname=None, robot_axes = 6, **kwargs):
        self.ROBOT_POST = robotpost
        self.ROBOT_NAME = robotname
        #self.PROG = HEADER
        self.LOG = ''
        self.nAxes = robot_axes
        
    def ProgStart(self, progname):
        if self.MAIN_DONE:
            self.addline('public void %s() {' % progname.replace(' ','_'))
            self.TAB = '\t\t'
        else:
            self.MAIN_DONE = True
        
    def ProgFinish(self, progname):
        self.spline_flush()
        self.TAB = '\t'
        self.addline('}')
        
    def ProgSave(self, folder, progname, ask_user = False, show_result = False):
        self.TAB = ''
        self.addline('}')
        progname_base = getFileName(progname)
        progname = progname + '.' + self.PROG_EXT
        if ask_user or not DirExists(folder):
            filesave = getSaveFile(folder, progname, 'Save program as...')
            if filesave is not None:
                filesave = filesave.name
                progname_base = getFileName(filesave)
            else:
                return
        else:
            filesave = folder + '/' + progname
        fid = open(filesave, "w")
        fid.write(HEADER % progname_base)
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
        self.spline_flush()
        #poseabs = self.REF_FRAME*pose
        self.TARGET_PTP_id = self.TARGET_PTP_id + 1
        targetname = ('MovePTP_%i' % self.TARGET_PTP_id)        
        self.addline('')
        #self.addline('getLogger().info("Move ptp to %s");' % targetname)        
        #self.addline('robot.move(ptp(%s).setCartVelocity(%.2f).setBlendingCart(%.2f));' % (angles_2_str(joints), self.SPEED_MMS, self.BLENDING_MM))
        self.addline('PTP %s = ptp(%s);' % (targetname, angles_2_str(joints)))
        self.addline('%s.move(%s);' % (self.MOVE_OBJECT, targetname))
        
    def MoveL(self, pose, joints, conf_RLF=None):
        """Add a linear movement"""   
        self.TARGET_LIN_id = self.TARGET_LIN_id + 1
        targetname = ('TargetLIN_%i' % self.TARGET_LIN_id)        
        framename = ('FrameLIN_%i' % self.TARGET_LIN_id)
        #self.addline('')
        #self.addline('getLogger().info("Move linear to %s");' % targetname)
        #self.addline('robot.move(lin(new Frame(%s)).setCartVelocity(%.2f).setBlendingCart(%.2f));' % (pose_2_str(poseabs), self.SPEED_MMS, self.BLENDING_MM))
        if pose is not None:
            poseabs = self.REF_FRAME*pose  
            self.addline('Frame %s = new Frame(%s);' % (framename, pose_2_str(poseabs)))
            #self.addline('// LIN %s = lin(%s);' % (targetname, framename))    
            #self.addline('robot.move(%s);' % targetname)
            self.spline_addbuffer(framename)
        else:
            # move linear by joints
            self.spline_flush()
            self.addline('LIN %s = lin(%s);' % (targetname, angles_2_str(joints)))
            self.addline('%s.move(%s);' % (self.MOVE_OBJECT, targetname))
            
            
        
    def MoveC(self, pose1, joints1, pose2, joints2, conf_RLF_1=None, conf_RLF_2=None):
        """Add a circular movement"""
        self.spline_flush()
        pose1abs = self.REF_FRAME*pose1
        pose2abs = self.REF_FRAME*pose2        
        self.TARGET_CIRC_id = self.TARGET_CIRC_id + 1
        framename1 = ('FrameCIRC_%i' % self.TARGET_CIRC_id)  
        self.TARGET_CIRC_id = self.TARGET_CIRC_id + 1
        framename2 = ('FrameCIRC_%i' % self.TARGET_CIRC_id)
        self.addline('')
        self.addline('getLogger().info("Move circular to %s and %s");' % (framename1, framename2))
        self.addline('Frame %s = new Frame(%s);' % (framename1, pose_2_str(pose1abs)))
        self.addline('Frame %s = new Frame(%s);' % (framename2, pose_2_str(pose2abs)))       
        self.addline('%s.move(circ(%s, %s).setCartVelocity(%.2f).setBlendingCart(%.2f));' % (self.MOVE_OBJECT, framename1, framename2, self.SPEED_MMS, self.BLENDING_MM))
        
    def setFrame(self, pose, frame_id=None, frame_name=None):
        """Change the robot reference frame"""
        if frame_name is None:
            frame_name = "Reference"
        self.REF_FRAME = pose
        self.spline_flush()
        # self.REF_FRAME = pose
        self.addline('// Using Reference Frame: %s' % frame_name)
        self.addline('// robot.addDefaultMotionFrame("%s", Transformation.ofDeg(%s);' % (frame_name,pose_2_str(pose, True)))
        
    def setTool(self, pose, tool_id=None, tool_name=None):
        """Change the robot TCP"""
        if tool_name is None:
            tool_name = "Tool"
        self.spline_flush()
        self.addline('// Using TCP: %s' % tool_name)
        self.addline('TOOL = new Tool("%s");' % tool_name)
        self.addline('TOOL.attachTo(robot.getFlange(), Transformation.ofDeg(%s));' % pose_2_str(pose, True))
        self.MOVE_OBJECT = 'TOOL'
        
        
    def Pause(self, time_ms):
        """Pause the robot program"""
        self.spline_flush()
        if time_ms < 0:
            self.addline('if (getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "Program paused. Select Cancel to stop the program.", "OK", "Cancel") == 1)')
            self.addline('{')
            self.addline('\treturn;')
            self.addline('}')
        elif time_ms > 0:
            self.addline('Thread.sleep(%.3f);' % (time_ms))
        
    def setSpeed(self, speed_mms):
        """Changes the robot speed (in mm/s)"""
        self.spline_flush()
        self.SPEED_MMS = speed_mms
        self.addline('// Linear speed set to %.3f mm/s' % self.SPEED_MMS)
    
    def setAcceleration(self, accel_mmss):
        """Changes the current robot acceleration"""
        self.addline('// Warning: set linear acceleration to %.3f mm/ss has no effect' % accel_mmss)
        self.ACCEL_MMSS = accel_mmss
        
    def setSpeedJoints(self, speed_degs):
        """Changes the robot joint speed (in deg/s)"""        
        self.SPEED_RADS = speed_degs*pi()/180.0
        self.addline('// Joint speed set to %.3f rad/s' % self.SPEED_RADS)
    
    def setAccelerationJoints(self, accel_degss):
        """Changes the robot joint acceleration (in deg/s2)"""
        self.addline('// Warning: set angular acceleration to %.3f deg/s has no effect' % accel_degss)
        
    def setZoneData(self, zone_mm):
        """Changes the zone data approach (makes the movement more smooth)"""        
        if zone_mm < 0:
            zone_mm = 0            
        self.BLENDING_MM = zone_mm
        self.addline('// smoothing/blending set to %.1f mm' % zone_mm)
        #self.addline('IMotion.setBlendingCart(%.1f); ' % zone_mm)
        
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
        self.spline_flush()
        if is_function_call:
            code.replace(' ','_')
            #self.addline(code + '();')
            self.addline(code + ';')
        else:
            self.addline(code)
        
    def RunMessage(self, message, iscomment = False):
        """Add a joint movement"""
        self.spline_flush()
        self.addline('')
        if iscomment:
            self.addline('// ' + message)
        else:
            self.addline('getLogger().info("%s");' % message)      
            
# ------------------ private ----------------------                
    def addline(self, newline):
        """Add a program line"""
        self.PROG = self.PROG + self.TAB + newline + '\n'
        
    def spline_addbuffer(self, frame):
        self.SPLINE = self.SPLINE + ('spl(%s),' % frame)
        self.SPLINE_COUNT = self.SPLINE_COUNT + 1
        self.SPLINE_LAST_FRAME = frame
        
    def spline_flush(self):
        if self.SPLINE_COUNT == 1:
            self.addline('%s.move(lin(%s));' % (self.MOVE_OBJECT, self.SPLINE_LAST_FRAME))
        elif self.SPLINE_COUNT > 1:
            self.PROG = self.PROG + self.TAB + 'move_curve = new Spline(' + self.SPLINE[:-1] + ');\n'
            self.PROG = self.PROG + self.TAB + ('%s.move(move_curve);\n' % self.MOVE_OBJECT)
        
        self.SPLINE = ''
        self.SPLINE_COUNT = 0
        self.SPLINE_LAST_FRAME = ''
        
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

    robot = RobotPost('Kuka_custom', 'Generic Kuka')

    robot.ProgStart("Program")
    robot.RunMessage("Program generated by RoboDK", True)
    robot.setFrame(Pose([807.766544, -963.699898, 41.478944, 0, 0, 0]))
    robot.setDO(123,1)
    robot.waitDI(125,1,1000)
    robot.waitDI(126,1)
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
