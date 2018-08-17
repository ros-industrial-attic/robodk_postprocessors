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
# for Nachi robots (FD controllers and AX) as well as OTC robots with FD controllers
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
def pose_2_str(pose,joints=None,reference=None):
    """Converts a pose target to a string"""
    if reference is not None:
        pose = reference*pose
    [x,y,z,r,p,w] = Pose_2_Nachi(pose)
    return ('(%.3f,%.3f,%.3f,%.3f,%.3f,%.3f)' % (x,y,z,r,p,w))
    
def angles_2_str(angles):
    """Converts a joint target to a string"""
    return '(%s)' % (','.join(format(ji, ".5f") for ji in angles))

# ----------------------------------------------------    
# Object class that handles the robot instructions/syntax
class RobotPost(object):
    """Robot post object"""
    BASE_PROGNAME = None #'MZ07L-01-A'
    MAX_LINES_X_PROG = 95000 # maximum number of lines per program
    PROG_ID = 5          # Program ID to store the program
    nPROGS = 0
    SKIP_OUTPUT = False
    
    # other variables
    ROBOT_POST = ''
    ROBOT_NAME = ''
    PROG_FILES = []
    PROGRAM_NAME = 'unknown'
    PROG = ''
    PROGS = []
    LOG = ''
    nAxes = 6
    TAB = ''
    REF_FRAME = None
    TOOL_ID = 1 # default id for the tool (H=TOOL_ID)
    SPEED_MMS = 500 # default 500mm/s speed    
    CURRENT_LINES = 0
    VALUE_A = '' #'A=1P,' 1 to 8 or 1P to 8P
    VALUE_SM = '' # 'SM=0,' 0 to 3
    
    def __init__(self, robotpost=None, robotname=None, robot_axes = 6, **kwargs):
        self.ROBOT_POST = robotpost
        self.ROBOT_NAME = robotname
        self.PROG = ''
        self.LOG = ''
        self.nAxes = robot_axes
        if self.BASE_PROGNAME is None and type(robotname) == str:
            robotnamelist = robotname.split(' ')
            if len(robotnamelist) > 1:
                self.BASE_PROGNAME = robotnamelist[1] + '-01-A'
            else:
                self.BASE_PROGNAME = 'MZ07L-01-A'
                
        for k,v in kwargs.items():
            if k == 'lines_x_prog':
                self.MAX_LINES_X_PROG = v    
        
    def ProgStart(self, progname, generate_sub_program=False):
        if self.nPROGS > 0 and not generate_sub_program:
            self.SKIP_OUTPUT = True
            return
        
        import re
        m = re.search(r'\d+$', progname)
        # if the string ends in digits m will be a Match object, or None otherwise.
        if m is not None:
            self.PROG_ID = int(m.group())
        
        self.nPROGS = self.nPROGS + 1
        self.CURRENT_LINES = 0
        self.PROGRAM_NAME = progname
        self.RunMessage('Program %s' % progname, True) # comment
        self.TAB = ''
        self.PROG = ''
        
    def ProgFinish(self, progname):
        self.TAB = ''
        self.addline('END')
        
    def ProgSave(self, folder, progname, ask_user = False, show_result = False):
        # File extensions are the program number and the file name is the robot type and –A.  So the file name for the SRA120EL program 200 would be SRA120EL-01-A.200.
        progname = self.BASE_PROGNAME
        progname_base = progname
        if ask_user or not DirExists(folder):
            filesave = getSaveFile(folder, progname, 'Save program as...')
            if filesave is not None:
                filesave = filesave.name
                folder = getFileDir(filesave)                
            else:
                return
        else:
            filesave = '%s/%s.%03i' % (folder, progname, self.PROG_ID)
        #----------------------------
        # Save file(s)
        self.PROG_FILES = []
        if self.nPROGS > 1: # save multiple programs
            self.PROGS.append(self.PROG)
            #self.nPROGS = self.nPROGS + 1 # Not required: We already added the counter
            self.nPROGS = len(self.PROGS)
            mainprog = '\' Main program %s calls %i subprograms\n' % (self.PROGRAM_NAME, self.nPROGS)
            for i in range(self.nPROGS):
                fsavei = ('%s/%s.%03i' % (folder, progname_base, self.PROG_ID+i+1))
                #mainprog = mainprog + ('%s.%03i\n' % (progname_base, self.PROG_ID+i+1))
                mainprog += 'CALLP [%03i]\n' % (self.PROG_ID+i+1)
                fid = open(fsavei, "w")
                fid.write(self.PROGS[i])
                fid.close()
                self.PROG_FILES.append(fsavei)
            mainprog = mainprog + 'END\n'
            fid = open(filesave, "w")
            fid.write(mainprog)
            fid.close()
            self.PROG_FILES.append(filesave)
            print('SAVED: %s\n' % filesave) # tell RoboDK the path of the saved file
                           
        else: # save one single program
            #filesave = '%s.%03i' % (filesave, self.PROG_ID)
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
                for file in self.PROG_FILES:
                    p = subprocess.Popen([show_result, file])
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
        #MOVEX A=1,M1X,P,(1960,0,1725,0,0,-180),R=5.0,H=1,MS,CONF=0000
        self.addline('MOVEX M1J,P,%s,S=%.2f,H=%i,MS' % (angles_2_str(joints) , self.SPEED_MMS , self.TOOL_ID))
        
    def MoveL(self, pose, joints, conf_RLF=None):
        """Add a linear movement"""
        #MOVEX A=1,M1J,L,(0,90,-90,0),R= 5.0,H=1,MS
        # MOVEX A=1P,AC=1,SM=1,F,M1X,P,(1200,0,1800,0,0,-180),R=10.0,H=1,MS,CONF=0000   
        if pose is None:
            self.addline('MOVEX %s%sM1J,L,%s,S=%.2f,H=%i,MS' % (self.VALUE_A, self.VALUE_SM, angles_2_str(joints) , self.SPEED_MMS , self.TOOL_ID))
        else:
            self.addline('MOVEX %s%sM1X,L,%s,S=%.2f,H=%i,MS' % (self.VALUE_A, self.VALUE_SM, pose_2_str(pose,joints,self.REF_FRAME) , self.SPEED_MMS , self.TOOL_ID))
        
    def MoveC(self, pose1, joints1, pose2, joints2, conf_RLF_1=None, conf_RLF_2=None):
        """Add a circular movement"""
        self.addline('MOVEX %s%sM1X,C1,%s,S=%.2f,H=%i,MS' % (self.VALUE_A, self.VALUE_SM, pose_2_str(pose1,joints1,self.REF_FRAME) , self.SPEED_MMS , self.TOOL_ID))
        self.addline('MOVEX %s%sM1X,C2,%s,S=%.2f,H=%i,MS' % (self.VALUE_A, self.VALUE_SM, pose_2_str(pose2,joints2,self.REF_FRAME) , self.SPEED_MMS , self.TOOL_ID))
        
    def setFrame(self, pose, frame_id=None, frame_name=None):
        """Change the robot reference frame"""
        self.RunMessage('Using the reference frame:', True)
        self.RunMessage('USERFRAME=%s' % (pose_2_str(pose)), True)
        self.RunMessage('(using all targets with respect to the robot reference)', True)
        self.REF_FRAME = pose
        # all targets are given with respect to the robot base
        # alternatively, we can use:
        #P1 = (500,0,0,0,0,0)
        #P2 = (500,500,0,0,0,0)
        #P3 = (1000,0,0,0,0,0)
        #MODUSRCOORD 1,1,2,3
        #CHGCOORD 1
        
    def setTool(self, pose, tool_id=None, tool_name=None):
        """Change the robot TCP"""
        #self.addline('TOOL[%i]=%s' % (self.TOOL_ID , pose_2_str(pose)))
        if tool_id is None or tool_id < 0:
            tool_id = 1
        self.TOOL_ID = tool_id
        self.RunMessage('Using the tool:', True)
        self.RunMessage('TOOL%i=%s' % (self.TOOL_ID , pose_2_str(pose)), True)
        [x,y,z,w,p,r] = Pose_2_Nachi(pose)
        #LETVF V11!,62.5
        #LETVF V12!,-108.253
        #LETVF V13!,100
        #LETVF V14!,0
        #LETVF V15!,90
        #LETVF V16!,-60
        #SETTOOL 1,1,V11!
        self.addline('MOVEX M1J,P,P*,S=%.1f,H=%i,MS' % (self.SPEED_MMS, self.TOOL_ID))
        #self.addline('LETVF V11!,%.3f' % x)
        #self.addline('LETVF V12!,%.3f' % y)
        #self.addline('LETVF V13!,%.3f' % z)
        #self.addline('LETVF V14!,%.3f' % w)
        #self.addline('LETVF V15!,%.3f' % p)
        #self.addline('LETVF V16!,%.3f' % r)
        #self.addline('SETTOOL %i,1,V11!' % self.TOOL_ID)
        
    def Pause(self, time_ms):
        """Pause the robot program"""
        if time_ms <= 0:
            self.addline('PAUSE 99999')
        else:
            self.addline('PAUSE %.3f' % (time_ms*0.001))
    
    def setSpeed(self, speed_mms):
        """Changes the robot speed (in mm/s)"""
        self.SPEED_MMS = speed_mms
    
    def setAcceleration(self, accel_mmss):
        """Changes the robot speed (in mm/s)"""
        self.addlog('setAcceleration not defined')
    
    def setSpeedJoints(self, speed_degs):
        """Changes the robot joint speed (in deg/s)"""
        self.addlog('setSpeedJoints not defined')
    
    def setAccelerationJoints(self, accel_degss):
        """Changes the robot joint acceleration (in deg/s2)"""
        self.addlog('setAccelerationJoints not defined')
        
    def setZoneData(self, zone_mm):
        """Changes the zone data approach (makes the movement more smooth)"""
        # This value is changed by using a rounding instruction: set Program-Set Rounding Instruction.         
        # MOVEX A=1P,AC=1,SM=1,F,M1X,P,(1200,0,1800,0,0,-180),R=10.0,H=1,MS,CONF=0000    
        self.RADIUS = zone_mm
        sm = zone_mm // 25
        sm = min(max(sm, 0), 3)
        self.VALUE_SM = 'SM=%i,' % sm
        
        # If we want movements to be accurate:
        if zone_mm < 0:
            self.VALUE_A = 'A=1P,'
        else:
            self.VALUE_A = ''          
        
    def setDO(self, io_var, io_value):
        """Sets a variable (output) to a given value"""
        setreset = "SET"
        if type(io_var) != str:  # set default variable name if io_var is a number
            io_var = '[%02i]' % int(io_var)
        if type(io_value) != str: # set default variable value if io_value is a number            
            if io_value > 0:
                setreset = "SET"
            else:
                setreset = "RESET"
        
        # at this point, io_var and io_value must be string values
        #self.addline('SETM %s,%s' % (io_var, io_value))
        self.addline('%s %s' % (setreset, io_var))
        
    def waitDI(self, io_var, io_value, timeout_ms=-1):
        """Waits for an input io_var to attain a given value io_value. Optionally, a timeout can be provided."""
        waitij = ""
        if type(io_var) != str:  # set default variable name if io_var is a number
            io_var = '[%02i]' % (int(io_var))
        if type(io_value) != str: # set default variable value if io_value is a number            
            if io_value > 0:
                waitij = 'WAITI'
            else:
                waitij = 'WAITJ'
        
        # at this point, io_var and io_value must be string values
        self.addline('%s %s' % (waitij, io_var))
        #if timeout_ms < 0:
        #    self.addline('WAITI %s' % (io_var))
        #else:
        #    self.addline('WAITI %s' % (io_var)) 
        
    def RunCode(self, code, is_function_call = False):
        """Adds code or a function call"""
        if is_function_call:
            self.RunMessage('Call program %s:' % code, True) # comment
            #self.addline('%s.%i' % (self.BASE_PROGNAME , name_2_id(code)))
            self.addline('CALLP %03i' % (name_2_id(code)))
            #code.replace(' ','_')            
            #if code.find('(') < 0:
            #    code = code + '()'            
        else:
            self.addline(code)
        
    def RunMessage(self, message, iscomment = False):
        """Add a joint movement"""
        if iscomment:
            self.addline('\' ' + message)
        else:
            self.addline('REM "' + message + '"')
        
# ------------------ private ----------------------                
    def addline(self, newline):
        """Add a program line"""
        if self.SKIP_OUTPUT:
            return
            
        if self.CURRENT_LINES > self.MAX_LINES_X_PROG:
            self.CURRENT_LINES = 0
            self.ProgFinish(self.PROGRAM_NAME)
            self.PROGS.append(self.PROG)
            self.ProgStart(self.PROGRAM_NAME, True)
        #-----------------------
        self.PROG += self.TAB + newline + '\n'
        self.CURRENT_LINES = self.CURRENT_LINES + 1            
        
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
    global robot
    robot = RobotPost('Nachi program', 'Generic Nachi Robot')

    robot.ProgStart("Program")
    robot.RunMessage("Program generated by RoboDK", True)
    robot.setFrame(Pose([807.766544, -963.699898, 41.478944, 0, 0, 0]))
    robot.setTool(Pose([62.5, -108.253175, 100, -60, 90, 0]))
    robot.setZoneData(100)
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
    #if robot.nPROGS > 0:
    #    for i in range(len(robot.PROGS)):
    #        print(robot.PROGS[i])
    #else:
    print(robot.PROG)
    if len(robot.LOG) > 0:
        mbox('Program generation LOG:\n\n' + robot.LOG)

    input("Press Enter to close...")

if __name__ == "__main__":
    """Function to call when the module is executed by itself: test"""
    test_post()
