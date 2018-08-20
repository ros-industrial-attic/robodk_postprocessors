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
# for a GSK robot with RoboDK
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
    [x,y,z,r,p,w] = Pose_2_Motoman(pose)
    return ('%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,0.0,0.0' % (x,y,z,r,p,w))
    
def angles_2_str(angles, pulses_x_deg):
    """Prints a joint target"""
    str = ''
    for i in range(len(angles)):
        str = str + ('%.3f,' % (angles[i]*pulses_x_deg[i]))
    str = str[:-1]
    return str

# ----------------------------------------------------    
# Object class that handles the robot instructions/syntax
class RobotPost(object):
    """Robot post object"""
    PROG_EXT = 'PRL'         # set the program extension
    MAX_LINES_X_PROG = 999  # maximum number of lines per program. It will then generate multiple "pages (files)"
    PULSES_X_DEG = [1,1,1,1,1,1]
    INCLUDE_SUB_PROGRAMS = False
    
    # other variables
    ROBOT_POST = ''
    ROBOT_NAME = ''
    
    # Multiple program files variables
    PROG_NAME = 'unknown' # single program name
    PROG_NAMES = []
    PROG_FILES = []    
    PROG_LIST = []
    DATA_LIST = []
    
    nLines = 0
    nProgs = 0
    
    PROG = ''
    DATA = ''
    LOG = ''
    nAxes = 6
    P_COUNT = 0
    TOOL = eye(4)
    FRAME = eye(4)
    LAST_POSE = None
    Z_VALUE = 0
    V_VALUE_J = 100 # default joint speed in percentage
    V_VALUE_L = 500 # default linear speed in mm/s
    
    
    
    def __init__(self, robotpost=None, robotname=None, robot_axes = 6, **kwargs):
        self.ROBOT_POST = robotpost
        self.ROBOT_NAME = robotname
        self.PROG = ''
        self.DATA = ''        
        self.LOG = ''
        self.nAxes = robot_axes
        for k,v in kwargs.items():
            if k == 'lines_x_prog':
                self.MAX_LINES_X_PROG = v    
            if k == 'pulses_x_deg':
                self.PULSES_X_DEG = v                    
        
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
        self.addline("MAIN;")
        self.addline("# Program: %s" % progname_i)
        if not new_page:
            pass
        
    def ProgFinish(self, progname, new_page = False):
        if self.nProgs > 1 and not self.INCLUDE_SUB_PROGRAMS:
            return
            
        self.PROG = self.PROG + "END;\n"
        if new_page:
            self.PROG_LIST.append(self.PROG)
            self.DATA_LIST.append(self.DATA)
            self.DATA = ''
            self.PROG = ''
            self.P_COUNT = 0
            self.nLines = 0
        
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
        fid.write(self.DATA)
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
            mainprog = "MAIN;\n"
            mainprog += "# %s \n" % progname_main
            for i in range(npages):
                self.PROG = self.PROG_LIST[i]
                mainprog += "CALL %s;\n" % self.PROG_NAMES[i]
                
            mainprog += "END;\n"
            self.DATA = ''
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
                self.DATA = self.DATA_LIST[i]
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
        if self.nLines > self.MAX_LINES_X_PROG:
            self.P_COUNT = 0
                
        if pose is None:   
            pname = 'P%03i' % self.P_COUNT     
            self.adddata('%s=%s;' % (pname, angles_2_str(joints, self.PULSES_X_DEG)))
            self.addline('MOVJ P%i, V%i, Z%i;' % (self.P_COUNT, self.V_VALUE_J, self.Z_VALUE))
            self.P_COUNT = self.P_COUNT + 1
            self.LAST_POSE = None
        else:
            #pose_abs = self.FRAME*pose*invH(self.TOOL)
            pname = 'P%03i' % self.P_COUNT        
            self.adddata('%s=%s;' % (pname, pose_2_str(pose)))
            self.addline('MOVJ P%i, V%i, Z%i;' % (self.P_COUNT, self.V_VALUE_L, self.Z_VALUE))            
            self.P_COUNT = self.P_COUNT + 1
        
    def MoveL(self, pose, joints, conf_RLF=None):
        """Add a linear movement"""
        if self.nLines > self.MAX_LINES_X_PROG:
            self.P_COUNT = 0
        if pose is None:
            pname = 'P%03i' % self.P_COUNT        
            self.adddata('%s=%s;' % (pname, angles_2_str(joints, self.PULSES_X_DEG)))
            self.addline('MOVL P%i, V%i, Z%i;' % (self.P_COUNT, self.V_VALUE_L, self.Z_VALUE))
            self.P_COUNT = self.P_COUNT + 1
        else:
            pose_abs = self.FRAME*pose*invH(self.TOOL)
            pname = 'P%03i' % self.P_COUNT        
            self.adddata('%s=%s;' % (pname, pose_2_str(pose)))
            self.addline('MOVL P%i, V%i, Z%i;' % (self.P_COUNT, self.V_VALUE_L, self.Z_VALUE))            
            self.P_COUNT = self.P_COUNT + 1
        self.LAST_POSE = pose
        
    def MoveC(self, pose1, joints1, pose2, joints2, conf_RLF_1=None, conf_RLF_2=None):
        """Add a circular movement"""
        if (self.nLines + 2) > self.MAX_LINES_X_PROG:
            self.P_COUNT = 0
        if self.LAST_POSE is not None:
            pose_abs = self.FRAME*self.LAST_POSE*invH(self.TOOL)
            pname = 'P%03i' % self.P_COUNT        
            self.addline('MOVC P%i, V%i, Z%i;' % (self.P_COUNT, self.V_VALUE_L, self.Z_VALUE))
            self.adddata('%s=%s;' % (pname, pose_2_str(pose_abs)))
            self.P_COUNT = self.P_COUNT + 1
        
        # mid point
        pose_abs1 = self.FRAME*pose1*invH(self.TOOL)
        pname = 'P%03i' % self.P_COUNT        
        self.addline('MOVC P%i, V%i, Z%i;' % (self.P_COUNT, self.V_VALUE_L, self.Z_VALUE))
        self.adddata('%s=%s;' % (pname, pose_2_str(pose_abs1)))
        self.P_COUNT = self.P_COUNT + 1
        
        # end point
        pose_abs2 = self.FRAME*pose2*invH(self.TOOL)
        pname = 'P%03i' % self.P_COUNT        
        self.addline('MOVC P%i, V%i, Z%i;' % (self.P_COUNT, self.V_VALUE_L, self.Z_VALUE))
        self.adddata('%s=%s;' % (pname, pose_2_str(pose_abs2)))
        self.P_COUNT = self.P_COUNT + 1        
        self.LAST_POSE = None
        
    def setFrame(self, pose, frame_id=None, frame_name=None):
        """Change the robot reference frame"""
        self.FRAME = pose
        self.addline('# using reference frame: %s' % frame_name)
        self.addline('# ' + pose_2_str(pose))        
        
    def setTool(self, pose, tool_id=None, tool_name=None):
        """Change the robot TCP"""
        self.TOOL = pose
        self.addline('# using tool frame: %s' % tool_name)
        self.addline('# ' + pose_2_str(pose))
        
    def Pause(self, time_ms):
        """Pause the robot program"""
        if time_ms <= 0:
            self.addline('STOP')
        else:
            self.addline('DELAY T%.3f;' % (time_ms*0.001))
        
    def setSpeed(self, speed_mms):
        """Changes the robot speed (in mm/s)"""
        self.V_VALUE_L = speed_mms
    
    def setAcceleration(self, accel_mmss):
        """Changes the current robot acceleration"""
        pass
                
    def setSpeedJoints(self, speed_degs):
        """Changes the robot joint speed (in deg/s)"""
        self.V_VALUE_J = min(speed_degs, 100)
    
    def setAccelerationJoints(self, accel_degss):
        """Changes the robot joint acceleration (in deg/s2)"""
        pass
        
    def setZoneData(self, zone_mm):
        """Changes the zone data approach (makes the movement more smooth)"""
        self.Z_VALUE = max(min(zone_mm,4),0)
        
    def setDO(self, io_var, io_value):
        """Sets a variable (output) to a given value"""
        if type(io_var) != str:  # set default variable name if io_var is a number
            io_var = 'OT%s' % str(io_var)        
        if type(io_value) != str: # set default variable value if io_value is a number            
            if io_value > 0:
                io_value = 'ON'
            else:
                io_value = 'OFF'
        
        # At this point, io_var and io_value are string values
        self.addline('DOUT %s, %s;' % (io_var, io_value))
        
    def waitDI(self, io_var, io_value, timeout_ms=-1):
        """Waits for an input io_var to attain a given value io_value. Optionally, a timeout can be provided."""
        if type(io_var) != str:  # set default variable name if io_var is a number
            io_var = 'IN%s' % str(io_var)        
        if type(io_value) != str: # set default variable value if io_value is a number            
            if io_value > 0:
                io_value = 'ON'
            else:
                io_value = 'OFF'        
        # At this point, io_var and io_value are string values
        self.addline('WAIT %s, %s, T%.1f;' % (io_var, io_value, max(timeout_ms,0)))
        
    def RunCode(self, code, is_function_call = False):
        """Adds code or a function call"""
        if is_function_call:
            code.replace(' ','_')
            if not code.endswith(')'):
                code = 'CALL ' + code + ';'
            self.addline(code)
        else:
            if not code.endswith(';'):
                code = code + ';'
            self.addline(code)
        
    def RunMessage(self, message, iscomment = False):
        """Add a joint movement"""
        if iscomment:
            self.addline('# ' + message)
        else:
            self.addline('# Show message: ' + message)
            
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
        
    def adddata(self, newline):
        """Add a program line"""
        if self.nProgs > 1 and not self.INCLUDE_SUB_PROGRAMS:
            return
            
        self.DATA = self.DATA + newline + '\n'
        
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
    robot.MoveJ(None, [-46.18419, -6.77518, -20.54925, 71.38674, 49.58727, -302.54752] )
    robot.MoveJ(Pose([200, 250, 348.734575, 180, 0, -150]), [-41.62707, -8.89064, -30.01809, 60.62329, 49.66749, -258.98418] )
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
    print(robot.DATA)
    print(robot.PROG)
    if len(robot.LOG) > 0:
        mbox('Program generation LOG:\n\n' + robot.LOG)

    input("Press Enter to close...")

if __name__ == "__main__":
    """Function to call when the module is executed by itself: test"""
    test_post()
