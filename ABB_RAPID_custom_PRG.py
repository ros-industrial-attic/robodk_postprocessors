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
# for an ABB robot with RoboDK
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

ONETAB = '    ' # one tab equals 4 spaces

# Define a custom header (variable declaration)
CUSTOM_HEADER = '''
    ! -------------------------------
    ! Define your variables here
    ! ...
'''

# Define your custom programs (predefined functions, not altered by RoboDK):
CUSTOM_FUNCTIONS = '''
    ! -------------------------------
    ! Define your functions here
    ! ...
'''

# ----------------------------------------------------
def pose_2_str(pose):
    """Prints a pose target"""
    [x,y,z,q1,q2,q3,q4] = Pose_2_ABB(pose)
    return ('[%.3f, %.3f, %.3f],[%.8f, %.8f, %.8f, %.8f]' % (x,y,z,q1,q2,q3,q4))
    
def angles_2_str(angles):
    """Prints a joint target"""
    njoints = len(angles)
    # extend the joint target if the robot has less than 6 degrees of freedom
    if njoints < 6:
        angles.extend([0]*(6-njoints))
    # Generate a string like:
    # [10,20,30,40,50,60]
    # with up to 6 decimals
    return '[%s]' % (','.join(format(ji, ".6f") for ji in angles[0:6]))

def extaxes_2_str(angles):
    """Prints the external axes, if any"""
    # extend the joint target if the robot has less than 6 degrees of freedom
    njoints = len(angles)
    if njoints <= 6:
        # should print 9E9 for unset external axes
        # [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]
        return '[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]'
    extaxes_str = (','.join(format(ji, ".6f") for ji in angles[6:njoints]))
    if njoints < 12:
        extaxes_str = extaxes_str + ',' + ','.join(['9E9']*(12-njoints))
    # If angles is [j1,j2,j3,j4,j5,j6,10,20], it will generate a string like:
    # [10,20,9E9,9E9,9E9,9E9]
    # with up to 6 decimals
    return '[%s]' % extaxes_str

# ----------------------------------------------------    
# Object class that handles the robot instructions/syntax
class RobotPost(object):
    """Robot post object"""
    PROG_EXT = 'prg'        # set the program extension
    
    # other variables
    ROBOT_POST = 'unset'
    ROBOT_NAME = 'generic'
    PROG_FILES = []
    
    nPROGS = 0
    PROG = ''
    TAB = ''
    LOG = ''
    ZONEDATA = 'fine'
    SPEEDDATA = '[500,500,5000,1000]'
    FRAME_NAME = 'rdkWObj'
    TOOL_NAME = 'rdkTool'
    
    def __init__(self, robotpost=None, robotname=None, robot_axes = 6, **kwargs):
        self.ROBOT_POST = robotpost
        self.ROBOT_NAME = robotname
        self.PROG = ''
        self.LOG = ''
        
    def ProgStart(self, progname):
        self.nPROGS = self.nPROGS + 1
        if self.nPROGS == 1:
            self.addline('%%%')
            self.addline('  VERSION:1')
            self.addline('  LANGUAGE:ENGLISH')
            self.addline('%%%')
            self.addline('')
            self.addline('MODULE MOD_%s' % progname)
            self.TAB = ONETAB
            self.addline('')
            self.addline('PERS tooldata rdkTool := [TRUE,[[0,0,0],[1,0,0,0]],[2,[0,0,15],[1,0,0,0],0,0,0.005]];')
            self.addline('PERS wobjdata rdkWObj := [FALSE, TRUE, "", [[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];')
            self.addcode(CUSTOM_HEADER)
            self.addcode(CUSTOM_FUNCTIONS)
        self.addline('')
        self.TAB = ONETAB
        self.addline('PROC %s()' % progname)
        self.TAB = ONETAB + ONETAB # instructions need two tabs
        
    def ProgFinish(self, progname):
        self.TAB = ONETAB
        self.addline('ENDPROC')
        
    def ProgSave(self, folder, progname, ask_user = False, show_result = False):
        self.addline('')
        self.TAB = ''
        self.addline('ENDMODULE')
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
        self.addline('MoveAbsJ [%s,%s],%s,%s,%s,\WObj:=%s;' % (angles_2_str(joints), extaxes_2_str(joints), self.SPEEDDATA, self.ZONEDATA, self.TOOL_NAME, self.FRAME_NAME))
        
    def MoveL(self, pose, joints, conf_RLF=None):
        """Add a linear movement"""
        #self.addline('MoveL [%s,[0,0,0,0],%s],%s,%s,%s,\WObj:=%s;' % (pose_2_str(pose), extaxes_2_str(joints), self.SPEEDDATA, self.ZONEDATA, self.TOOL_NAME, self.FRAME_NAME))
        if conf_RLF is None:
            conf_RLF = [0,0,0]
        #self.addline('MoveL [%s,[0,0,0,0],%s],%s,%s,rdkTool,\WObj:=rdkWObj;' % (pose_2_str(pose), extaxes_2_str(joints), self.SPEEDDATA, self.ZONEDATA))
        [REAR, LOWERARM, FLIP] = conf_RLF
        cf1 = math.floor(joints[0]/90.0)
        cf4 = math.floor(joints[3]/90.0)
        cf6 = math.floor(joints[5]/90.0)
        cfx = 4*REAR + 2*LOWERARM + FLIP
        #self.addline('MoveL [%s,[%i,%i,%i,%i],%s],%s,%s,%s,\WObj:=%s;' % (pose_2_str(pose), extaxes_2_str(joints), self.SPEEDDATA, self.ZONEDATA, self.TOOL_NAME, self.FRAME_NAME))
        self.addline('MoveL [%s,[%i,%i,%i,%i],%s],%s,%s,%s,\WObj:=%s;' % (pose_2_str(pose), cf1, cf4, cf6,cfx, extaxes_2_str(joints), self.SPEEDDATA, self.ZONEDATA, self.TOOL_NAME, self.FRAME_NAME))
       
        
    def MoveC(self, pose1, joints1, pose2, joints2, conf_RLF_1=None, conf_RLF_2=None):
        """Add a circular movement"""
        target1 = ''
        target2 = ''
        if conf_RLF_1 is None:
            conf_RLF_1 = [0,0,0]                
        cf1_1 = math.floor(joints1[0]/90.0)
        cf4_1 = math.floor(joints1[3]/90.0)
        cf6_1 = math.floor(joints1[5]/90.0)
        [REAR, LOWERARM, FLIP] = conf_RLF_1
        cfx_1 = 4*REAR + 2*LOWERARM + FLIP
        target1 = '[%s,[%i,%i,%i,%i],%s]' % (pose_2_str(pose1), cf1_1, cf4_1, cf6_1,cfx_1, extaxes_2_str(joints1))
            
        if conf_RLF_2 is None:
            conf_RLF_2 = [0,0,0]
        cf1_2 = math.floor(joints2[0]/90.0)
        cf4_2 = math.floor(joints2[3]/90.0)
        cf6_2 = math.floor(joints2[5]/90.0)
        [REAR, LOWERARM, FLIP] = conf_RLF_2
        cfx_2 = 4*REAR + 2*LOWERARM + FLIP
        target2 = '[%s,[%i,%i,%i,%i],%s]' % (pose_2_str(pose2), cf1_2, cf4_2, cf6_2,cfx_2, extaxes_2_str(joints2))

        self.addline('MoveC %s,%s,%s,%s,%s,\WObj:=%s;' % (target1, target2, self.SPEEDDATA, self.ZONEDATA, self.TOOL_NAME, self.FRAME_NAME))
        
    def setFrame(self, pose, frame_id=None, frame_name=None):
        """Change the robot reference frame"""
        #self.addline('rdkWObj := [FALSE, TRUE, "", [%s],[[0,0,0],[1,0,0,0]]];' % pose_2_str(pose))
        #if frame_name == None:
        #    frame_name = self.FRAME_NAME
        #frame_name = frame_name.replace(' ','_')
        #self.FRAME_NAME = frame_name
        self.addline('%s := [FALSE, TRUE, "", [%s],[[0,0,0],[1,0,0,0]]];' % (self.FRAME_NAME, pose_2_str(pose)))
        
    def setTool(self, pose, tool_id=None, tool_name=None):
        """Change the robot TCP"""
        #if tool_name == None:
        #    tool_name = self.TOOL_NAME
        #tool_name = tool_name.replace(' ','_')
        #self.TOOL_NAME = tool_name
        self.addline('%s := [TRUE,[%s],[1,[0,0,50],[1,0,0,0],0,0,0.005]];' % (self.TOOL_NAME, pose_2_str(pose)))
        
    def Pause(self, time_ms):
        """Pause the robot program"""
        if time_ms <= 0:
            self.addline('STOP;')
        else:
            self.addline('WaitTime %.3f' % (time_ms*0.001))
        
    def setSpeed(self, speed_mms):
        """Changes the robot speed (in mm/s)"""
        #self.SPEEDDATA = 'v%i' % speed_mms
        self.SPEEDDATA = '[%.2f,500,5000,1000]' % speed_mms
    
    def setAcceleration(self, accel_mmss):
        """Changes the robot acceleration (in mm/s2)"""
        self.addlog('setAcceleration is not defined')
        
    def setSpeedJoints(self, speed_degs):
        """Changes the robot joint speed (in deg/s)"""
        self.addlog('setSpeedJoints not defined')
    
    def setAccelerationJoints(self, accel_degss):
        """Changes the robot joint acceleration (in deg/s2)"""
        self.addlog('setAccelerationJoints not defined')
        
    def setZoneData(self, zone_mm):
        """Changes the zone data approach (makes the movement more smooth)"""
        if zone_mm < 0:
            self.ZONEDATA = 'fine'
        else:
            self.ZONEDATA = 'z%i' % zone_mm
        
    def setDO(self, io_var, io_value):
        """Sets a variable (output) to a given value"""
        if type(io_var) != str:  # set default variable name if io_var is a number
            io_var = 'D_OUT_%s' % str(io_var)        
        if type(io_value) != str: # set default variable value if io_value is a number            
            if io_value > 0:
                io_value = '1'
            else:
                io_value = '0'
        
        # at this point, io_var and io_value must be string values
        self.addline('SetDO %s, %s;' % (io_var, io_value))
        
    def waitDI(self, io_var, io_value, timeout_ms=-1):
        """Waits for an input io_var to attain a given value io_value. Optionally, a timeout can be provided."""
        if type(io_var) != str:  # set default variable name if io_var is a number
            io_var = 'D_IN_%s' % str(io_var)        
        if type(io_value) != str: # set default variable value if io_value is a number            
            if io_value > 0:
                io_value = '1'
            else:
                io_value = '0'
        
        # at this point, io_var and io_value must be string values
        if timeout_ms < 0:
            self.addline('WaitDI %s, %s;' % (io_var, io_value))
        else:
            self.addline('WaitDI %s, %s, \MaxTime:=%.1f;' % (io_var, io_value, timeout_ms*0.001))   
        
    def RunCode(self, code, is_function_call = False):
        """Adds code or a function call"""
        if is_function_call:
            code = code.replace(' ','_')
            self.addline(code + ';')
        else:
            if code.startswith('END') or code.startswith('ELSEIF'):
                # remove tab after ENDWHILE or ENDIF
                self.TAB = self.TAB[:-len(ONETAB)]
                
            self.addline(code.replace('\t','  '))# replace each tab by 2 spaces

            if code.startswith('IF ') or code.startswith('ELSEIF ') or code.startswith('WHILE '):
                # add tab (one tab = two spaces)
                self.TAB = self.TAB + ONETAB
            
        
    def RunMessage(self, message, iscomment = False):
        """Add a joint movement"""
        if iscomment:
            self.addline('! ' + message)
        else:
            self.addline('TPWrite "%s";' % message)
        
# ------------------ private ----------------------                
    def addline(self, newline):
        """Add a program line"""
        self.PROG = self.PROG + self.TAB + newline + '\n'
        
    def addlog(self, newline):
        """Add a log message"""
        self.LOG = self.LOG + newline + '\n'

    def addcode(self, code):
        """Adds custom code, such as a custom header"""
        self.PROG = self.PROG + code

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

    robot = RobotPost('ABB RAPID custom', 'Generic ABB robot')

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
