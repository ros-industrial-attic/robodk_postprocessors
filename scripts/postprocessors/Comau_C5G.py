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
# for Comau C5G robots
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
def pose_2_str(pose,joints=None):
    """Converts a pose target to a string"""
    [x,y,z,w,p,r] = Pose_2_Comau(pose)
    return ('POS(%.4f,%.4f,%.4f,%.4f,%.4f,%.4f)' % (x,y,z,w,p,r))
    
def joints_2_str(angles):
    """Contverts a joint target to a string"""
    return '{%s}' % (','.join(format(ji, ".5f") for ji in angles))

# ----------------------------------------------------    
# Object class that handles the robot instructions/syntax
class RobotPost(object):
    """Robot post processor class"""
    PROG_EXT = 'txt'        # set the program extension
    MAX_LINES_X_PROG = 2000  # maximum number of lines per program. It will then generate multiple "pages (files)"
    INCLUDE_SUB_PROGRAMS = False
    
    # other variables
    ROBOT_POST = ''
    ROBOT_NAME = ''
    
    # Multiple program files variables
    PROG_NAME = 'unknown' # single program name
    PROG_NAMES = []
    PROG_FILES = []    
    PROG_LIST = []    
    
    PROG = ''
    nLines = 0
    nProgs = 0
    LOG = ''
    nAxes = 6
    TAB = ''
    LAST_POSE = None
    LAST_E_LENGTH = 0
    NEW_E_LENGTH = 0
    
    def __init__(self, robotpost=None, robotname=None, robot_axes = 6, **kwargs):
        self.ROBOT_POST = robotpost
        self.ROBOT_NAME = robotname
        self.PROG = ''
        self.LOG = ''
        self.nAxes = robot_axes
        
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
        self.addline('PROGRAM %s' % progname_i)
        self.addline('CONST')
        self.addline('VAR')
        self.addline('BEGIN')
        self.TAB = '  '
        self.addline('$ORNT_TYPE := RS_WORLD')
        self.addline('$MOVE_TYPE := JOINT')
        self.addline('$CNFG_CARE := FALSE')
        self.addline('$TURN_CARE := FALSE')
        self.addline('$SING_CARE := FALSE')
        self.addline('$TERM_TYPE := NOSETTLE')
        
    def ProgFinish(self, progname, new_page = False):
        self.TAB = ''
        self.PROG = self.PROG + "END\n"
        if new_page:
            self.PROG_LIST.append(self.PROG)
            self.PROG = ''
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
        self.FILE_SAVED = filesave
        fid = open(filesave, "w")
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
            mainprog = "PROGRAM %s\n" % progname_main
            mainprog += "CONST\nVAR\nBEGIN\n"
            for i in range(npages):
                self.PROG = self.PROG_LIST[i]
                mainprog += "  PROGRAM %s()\n" % self.PROG_NAMES[i]
                
            mainprog += "END\n"
            self.PROG = mainprog
            self.progsave(folder, progname_main, ask_user, show_result)
            self.LOG = ''
            folder_user = getFileDir(self.FILE_SAVED)
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
        self.addline('MOVE JOINT TO ' + joints_2_str(joints))
        
    def new_move(self, pose1, pose2):
        if pose1 is None:
            return
            
        def Calculate_Time(Dist, Vmax, Amax):
            '''Calculate the time it takes to move a distance Dist at Amax acceleration and Vmax speed'''
            tacc = Vmax/Amax;
            Xacc = 0.5*Amax*tacc*tacc;
            if Dist <= 2*Xacc:
                # Vmax is not reached
                tacc = sqrt(Dist/Amax)
                Ttot = tacc*2
            else:
                # Vmax is reached
                Xvmax = Dist - 2*Xacc
                Tvmax = Xvmax/Vmax
                Ttot = 2*tacc + Tvmax
            return Ttot
            
        add_material = self.NEW_E_LENGTH - self.LAST_E_LENGTH
        self.LAST_E_LENGTH = self.NEW_E_LENGTH
        
        if add_material > 0:
            distance_mm = norm(subs(pose1.Pos(), pose2.Pos()))
            # calculate movement time in seconds
            time_s = Calculate_time(distance_mm, self.SPEED_MMS, self.ACCEL_MMSS)
            # add material
            self.addline("$AOUT[5] := %.3f" % add_material/time_s)
        else:
            # DO not add material
            self.addline("$AOUT[5] := 0")
            
    def new_movec(self, pose1, pose2, pose3):
        return
        
    def MoveL(self, pose, joints, conf_RLF=None):
        """Add a linear movement"""
        self.new_move(self.LAST_POSE, pose)
        self.addline('MOVE LINEAR TO ' + pose_2_str(pose,joints))
        self.LAST_POSE = pose
        
    def MoveC(self, pose1, joints1, pose2, joints2, conf_RLF_1=None, conf_RLF_2=None):
        """Add a circular movement"""
        self.new_movec(self.LAST_POSE, pose1, pose2)
        self.addline('MOVE CIRCULAR TO ' + pose_2_str(pose2,joints2) + ' VIA ' + pose_2_str(pose1,joints1))
        self.LAST_POSE = pose2
        
    def setFrame(self, pose, frame_id=None, frame_name=None):
        """Change the robot reference frame"""
        self.addline('$UFRAME := ' + pose_2_str(pose))
        
    def setTool(self, pose, tool_id=None, tool_name=None):
        """Change the robot TCP"""
        self.addline('$TOOL := ' + pose_2_str(pose))
        
    def Pause(self, time_ms):
        """Pause the robot program"""
        if time_ms <= 0:
            self.addline('PAUSE')
        else:
            self.addline('DELAY %.3f' % (time_ms))
    
    def setSpeed(self, speed_mms):
        """Changes the robot speed (in mm/s)"""        
        self.addline('$SPD_OPT := SPD_LIN')
        self.addline('$LIN_SPD := %.3f' % (speed_mms*0.001))
    
    def setAcceleration(self, accel_mmss):
        """Changes the robot acceleration (in mm/s2)"""
        self.addlog('setAcceleration not defined')
    
    def setSpeedJoints(self, speed_degs):
        """Changes the robot joint speed (in deg/s)"""
        self.addline('$ROT_SPD := %.3f' % (speed_mms*pi/180.0))
    
    def setAccelerationJoints(self, accel_degss):
        """Changes the robot joint acceleration (in deg/s2)"""
        self.addlog('setAccelerationJoints not defined')
        
    def setZoneData(self, zone_mm):
        """Changes the zone data approach (makes the movement more smooth)"""
        self.addlog('setZoneData not defined (%.1f mm)' % zone_mm)
        
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
            if code.startswith("Extruder("):
                # if the program call is Extruder(123.56), we extract the number as a string and convert it to a number
                self.NEW_E_LENGTH = float(code[9:-1]) # it needs to retrieve the extruder length from the program call
                # Do not generate program call
                return
                
            code.replace(' ','_')
            if code.find('(') < 0:
                code = code + '()'
            self.addline('PROGRAM ' + code)
        else:
            self.addline(code)
        
    def RunMessage(self, message, iscomment = False):
        """Add a joint movement"""
        if iscomment:
            self.addline('; ' + message)
        else:
            self.addline('TYPE "' + message + '"')
        
# ------------------ private ----------------------                
    def addline(self, newline):
        """Add a program line"""
        if self.nProgs > 1 and not self.INCLUDE_SUB_PROGRAMS:
            return
        
        if self.nLines > self.MAX_LINES_X_PROG:
            self.nLines = 0
            self.ProgFinish(self.PROG_NAME, True)
            self.ProgStart(self.PROG_NAME, True)

        self.PROG = self.PROG + self.TAB + newline + '\n'
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

    robot = RobotPost('Comau_custom', 'Generic Comau robot')

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
