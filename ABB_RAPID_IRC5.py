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
    MAX_LINES_X_PROG = 5000  # maximum number of lines per program. It will then generate multiple "pages (files)"
    INCLUDE_SUB_PROGRAMS = True
    PROG_EXT = 'mod'        # set the program extension
    
    # other variables
    ROBOT_POST = 'ABB IRC5 including arc welding and 3D printing options'
    ROBOT_NAME = 'unknown'
    PROG_NAMES = []
    PROG_FILES = []    
    PROG_LIST = []
    PROG_CALLS = []
    PROG_CALLS_LIST = []   
    nLines = 0
    nProgs = 0
    
    PROG = []
    TAB = ''
    LOG = ''
    SPEEDDATA = 'rdkSpeed'
    ZONEDATA = 'z1'
    TOOLDATA = 'rdkTool'
    WOBJDATA = 'rdkWObj'
    nAxes = 6
    
    CLAD_ON = False
    CLAD_DATA = 'clad1'
    
    ARC_ON = False
    ARC_WELDDATA = 'weld1'
    ARC_WEAVEDATA = 'weave1'
    ARC_SEAMDATA = 'seam1'

    NEW_E_LENGTH = None
    
    
    def __init__(self, robotpost=None, robotname=None, robot_axes = 6, **kwargs):
        self.ROBOT_POST = robotpost
        self.ROBOT_NAME = robotname
        self.PROG = []
        self.LOG = ''
        self.nAxes = robot_axes
        for k,v in kwargs.items():
            if k == 'lines_x_prog':
                self.MAX_LINES_X_PROG = v      
        
    def ProgStart(self, progname, new_page = False):
        progname_i = progname
        nPages = len(self.PROG_LIST)
        if new_page:            
            if nPages == 0:
                progname_i = progname
            else:
                progname_i = "%s%i" % (self.PROG_NAME, nPages)
                
        else:
            self.nProgs = self.nProgs + 1
            if self.nProgs == 1:# and not self.INCLUDE_SUB_PROGRAMS:
                self.PROG_NAME = progname
                #self.nProgs = self.nProgs + 1
            
        if new_page or not self.INCLUDE_SUB_PROGRAMS or self.nProgs == 1:
            # new file!
            self.PROG_NAMES.append(progname_i)
            self.TAB = ''
            self.addline('%%%')
            self.addline('  VERSION:1')
            self.addline('  LANGUAGE:ENGLISH')
            self.addline('%%%')
            self.addline('')
            self.addline('MODULE MOD_%s' % progname_i)
            self.TAB = ONETAB
            
        if self.nProgs == 1 and nPages == 0:
            self.TAB = ONETAB
            self.addline('')
            self.addline('LOCAL PERS tooldata %s := [TRUE,[[0,0,0],[1,0,0,0]],[2,[0,0,15],[1,0,0,0],0,0,0.005]];' % self.TOOLDATA)
            self.addline('LOCAL PERS wobjdata %s := [FALSE, TRUE, "", [[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];' % self.WOBJDATA)
            self.addline('VAR speeddata %s := [250,500,5000,1000]; ! set default speed' % self.SPEEDDATA)            
            self.addcode(CUSTOM_HEADER)
            self.addcode(CUSTOM_FUNCTIONS)
        
        self.TAB = ONETAB
        self.addline('')
        self.addline('PROC %s()' % progname_i)
        self.TAB = ONETAB + ONETAB # instructions need two tabs
        self.addline('ConfJ \On;')
        self.addline('ConfL \Off;')
        
    def ProgFinish(self, progname, new_page = False):
        self.TAB = ONETAB
        self.PROG += [ONETAB + 'ENDPROC\n']
        if new_page or not self.INCLUDE_SUB_PROGRAMS:# or self.nProgs == 1:
            self.PROG += ['ENDMODULE']
            self.PROG_LIST.append(self.PROG)
            self.PROG_CALLS_LIST.append(self.PROG_CALLS)
            self.PROG = []
            self.PROG_CALLS = []
            self.nLines = 0
        #elif self.nProgs <= 1 or self.INCLUDE_SUB_PROGRAMS:
        #    self.PROG += ['ENDMODULE']

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
        for line in self.PROG:
            fid.write(line)
            fid.write('\n') # print new line
            
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
                self.PROG += ['ENDMODULE']
                self.PROG_LIST.append(self.PROG)
                self.PROG_CALLS_LIST.append(self.PROG_CALLS)
                self.PROG = []
                self.PROG_CALLS = []
                self.nLines = 0
                
            npages = len(self.PROG_LIST)
            progname_main = progname + "Main"
            mainprog = []
            mainprog += ["MODULE MOD_%s\n" % progname_main]
            mainprog += [ONETAB+"!PROC Main()"]
            mainprog += [ONETAB+"PROC %s()" % progname_main]            
            mainprog += [ONETAB+ONETAB+"! This main program needs to be executed to run: %s\n" % progname]
            for i in range(npages):
                mainprog += [ONETAB+ONETAB+"%s()" % self.PROG_NAMES[i]]
            mainprog += ["\n"+ONETAB+"ENDPROC\n"]
            mainprog += ["ENDMODULE"]
            
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
                self.PROG = self.PROG_LIST[i]
                self.PROG_CALLS = self.PROG_CALLS_LIST[i]
                self.progsave(folder_user, self.PROG_NAMES[i], False, show_result)
                
        else:
            self.PROG += ['ENDMODULE'] # Very important!
            self.progsave(folder, progname, ask_user, show_result)
        
         
    def ProgSendRobot(self, robot_ip, remote_path, ftp_user, ftp_pass):
        """Send a program to the robot using the provided parameters. This method is executed right after ProgSave if we selected the option "Send Program to Robot".
        The connection parameters must be provided in the robot connection menu of RoboDK"""
        UploadFTP(self.PROG_FILES, robot_ip, remote_path, ftp_user, ftp_pass)
        
    def MoveJ(self, pose, joints, conf_RLF=None):
        """Add a joint movement"""
        self.addline('MoveAbsJ [%s,%s],%s,%s,%s,\WObj:=%s;' % (angles_2_str(joints), extaxes_2_str(joints), self.SPEEDDATA, self.ZONEDATA, self.TOOLDATA, self.WOBJDATA))
        
    def MoveL(self, pose, joints, conf_RLF=None):
        """Add a linear movement"""
        
        # Control turning arc movement off
        if self.ARC_ON and self.NEW_E_LENGTH is None:
            self.ARC_ON = False
            self.addline('ArcLEnd;')
        
        target = ''
        if pose is None:
            target = 'CalcRobT([%s,%s],%s,\WObj:=%s)' % (angles_2_str(joints), extaxes_2_str(joints), self.TOOLDATA, self.WOBJDATA)
        else:
            if conf_RLF is None:
                conf_RLF = [0,0,0]
            cf1 = 0
            cf4 = 0
            cf6 = 0            
            if joints is not None and len(joints) >= 6:
                cf1 = math.floor(joints[0]/90.0)
                cf4 = math.floor(joints[3]/90.0)
                cf6 = math.floor(joints[5]/90.0)
            [REAR, LOWERARM, FLIP] = conf_RLF
            cfx = 4*REAR + 2*LOWERARM + FLIP
            target = '[%s,[%i,%i,%i,%i],%s]' % (pose_2_str(pose), cf1, cf4, cf6,cfx, extaxes_2_str(joints))

        if self.ARC_ON:
            # ArcL p100, v100, seam1, weld5 \Weave:=weave1, z10, gun1;
            self.addline('ArcL %s,%s,%s,%s,\Weave:=%s,%s,%s,\WObj:=%s;' % (target, self.SPEEDDATA, self.ARC_SEAMDATA, self.ARC_WELDDATA, self.ARC_WEAVEDATA, self.ZONEDATA, self.TOOLDATA, self.WOBJDATA))
        elif self.CLAD_ON:
            self.addline('CladL %s,%s,%s,%s,%s,\WObj:=%s;' % (target, self.SPEEDDATA, self.CLAD_DATA, self.ZONEDATA, self.TOOLDATA, self.WOBJDATA))
        else:
            self.addline('MoveL %s,%s,%s,%s,\WObj:=%s;' % (target, self.SPEEDDATA, self.ZONEDATA, self.TOOLDATA, self.WOBJDATA))

        # Modification for Paul
        self.NEW_E_LENGTH = None 
            
    def MoveC(self, pose1, joints1, pose2, joints2, conf_RLF_1=None, conf_RLF_2=None):
        """Add a circular movement"""
        target1 = ''
        target2 = ''
        if pose1 is None:
            target1 = 'CalcRobT([%s,%s], %s \WObj:=%s)' % (angles_2_str(joints1), extaxes_2_str(joints1), self.TOOLDATA, self.WOBJDATA)
        else:
            if conf_RLF_1 is None:
                conf_RLF_1 = [0,0,0]                
            cf1_1 = 0
            cf4_1 = 0
            cf6_1 = 0   
            if joints1 is not None and len(joints1) >= 6:
                cf1_1 = math.floor(joints1[0]/90.0)
                cf4_1 = math.floor(joints1[3]/90.0)
                cf6_1 = math.floor(joints1[5]/90.0)
            [REAR, LOWERARM, FLIP] = conf_RLF_1
            cfx_1 = 4*REAR + 2*LOWERARM + FLIP
            target1 = '[%s,[%i,%i,%i,%i],%s]' % (pose_2_str(pose1), cf1_1, cf4_1, cf6_1,cfx_1, extaxes_2_str(joints1))
            
        if pose2 is None:
            target2 = 'CalcRobT([%s,%s],%s,\WObj:=%s)' % (angles_2_str(joints2), extaxes_2_str(joints2), self.TOOLDATA, self.WOBJDATA)
        else:
            if conf_RLF_2 is None:
                conf_RLF_2 = [0,0,0]
            cf1_2 = 0
            cf4_2 = 0
            cf6_2 = 0  
            if joints2 is not None and len(joints2) >= 6:
                cf1_2 = math.floor(joints2[0]/90.0)
                cf4_2 = math.floor(joints2[3]/90.0)
                cf6_2 = math.floor(joints2[5]/90.0)
            [REAR, LOWERARM, FLIP] = conf_RLF_2
            cfx_2 = 4*REAR + 2*LOWERARM + FLIP
            target2 = '[%s,[%i,%i,%i,%i],%s]' % (pose_2_str(pose2), cf1_2, cf4_2, cf6_2,cfx_2, extaxes_2_str(joints2))
           
        if self.ARC_ON:
            # ArcL p100, v100, seam1, weld5 \Weave:=weave1, z10, gun1;
            self.addline('ArcC %s,%s,%s,%s,%s,\Weave:=%s,%s,%s,\WObj:=%s;' % (target1, target2, self.SPEEDDATA, self.ARC_SEAMDATA, self.ARC_WELDDATA, self.ARC_WEAVEDATA, self.ZONEDATA, self.TOOLDATA, self.WOBJDATA))
        elif self.CLAD_ON:
            self.addline('CladC %s,%s,%s,%s,%s,%s,\WObj:=%s;' % (target1, target2, self.SPEEDDATA, self.CLAD_DATA, self.ZONEDATA, self.TOOLDATA, self.WOBJDATA))
        else:
            self.addline('MoveC %s,%s,%s,%s,%s,\WObj:=%s;' % (target1, target2, self.SPEEDDATA, self.ZONEDATA, self.TOOLDATA, self.WOBJDATA))
                
    def setFrame(self, pose, frame_id=None, frame_name=None):
        """Change the robot reference frame"""
        #self.addline('%s := [FALSE, TRUE, "", [%s],[[0,0,0],[1,0,0,0]]];' % (self.WOBJDATA, pose_2_str(pose)))
        self.addline('%s.uframe := [%s];' % (self.WOBJDATA, pose_2_str(pose)))
        
    def setTool(self, pose, tool_id=None, tool_name=None):
        """Change the robot TCP"""
        #self.addline('%s := [TRUE,[%s],[2,[0,0,15],[1,0,0,0],0,0,0.005]];' % (self.TOOLDATA, pose_2_str(pose)))
        self.addline('%s.tframe := [%s];' % (self.TOOLDATA, pose_2_str(pose)))
        
    def Pause(self, time_ms):
        """Pause the robot program"""
        if time_ms <= 0:
            self.addline('STOP;')
        else:
            self.addline('WaitTime %.3f;' % (time_ms*0.001))
        
    def setSpeed(self, speed_mms):
        """Changes the robot speed (in mm/s)"""
        #self.SPEEDDATA = 'v%i' % speed_mms
        self.addline('%s := [%.2f,500,5000,1000];' % (self.SPEEDDATA, speed_mms))
    
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
            if code.startswith('ArcLStart'):
                self.ARC_ON = True
            elif code.startswith('ArcLEnd'):
                self.ARC_ON = False
            elif code.startswith('CladLStart'):
                self.CLAD_ON = True
            elif code.startswith('CladLEnd'):
                self.CLAD_ON = False
            elif code.startswith("Extruder("):
                self.addline(code + ';')
                return
            
                # if the program call is Extruder(123.56), we extract the number as a string and convert it to a number
                self.NEW_E_LENGTH = float(code[9:-1]) # it needs to retrieve the extruder length from the program call
                # Parse the Extruder into ArcLStart
                if not self.ARC_ON:
                    # Generate ArcLStart if we are not welding yet
                    self.ARC_ON = True
                    self.addline('ArcLStart;')
                # Do not generate the program call
                return                
                
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
        if self.nProgs > 1 and not self.INCLUDE_SUB_PROGRAMS:
            return
        
        if self.nLines > self.MAX_LINES_X_PROG:
            self.nLines = 0
            self.ProgFinish(self.PROG_NAME, True)
            self.ProgStart(self.PROG_NAME, True)
            
        self.PROG += [self.TAB + newline]
        self.nLines = self.nLines + 1
        
    def addlog(self, newline):
        """Add a log message"""
        if self.nProgs > 1 and not self.INCLUDE_SUB_PROGRAMS:
            return
        self.LOG = self.LOG + newline + '\n'

    def addcode(self, code):
        """Adds custom code, such as a custom header"""
        self.PROG += [code]
        

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

    robot = RobotPost(r'ABB_RAPID_IRC5', r'ABB IRB 6700-155/2.85', 6, axes_type=['R','R','R','R','R','R'])

    robot.ProgStart(r'Prog1')
    robot.RunMessage(r'Program generated by RoboDK 3.1.5 for ABB IRB 6700-155/2.85 on 18/05/2017 11:02:41', True)
    robot.RunMessage(r'Using nominal kinematics.', True)
    robot.setFrame(Pose([0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000]),-1,r'ABB IRB 6700-155/2.85 Base')
    robot.setTool(Pose([380.000000, 0.000000, 200.000000, 0.000000, 90.000000, 0.000000]),1,r'Tool 1')
    robot.setSpeed(2000.000)
    robot.MoveJ(Pose([2103.102861, 0.000000, 1955.294643, -180.000000, -3.591795, -180.000000]), [0.00000, 3.93969, -14.73451, 0.00000, 14.38662, -0.00000], [0.0, 0.0, 0.0])
    robot.MoveJ(Pose([2065.661612, 700.455189, 1358.819971, 180.000000, -3.591795, -180.000000]), [22.50953, 5.58534, 8.15717, 67.51143, -24.42689, -64.06258], [0.0, 0.0, 1.0])
    robot.Pause(500.0)
    robot.setSpeed(100.000)
    robot.RunCode(r'ArcLStart', True)
    robot.MoveL(Pose([2065.661612, 1074.197508, 1358.819971, 149.453057, -3.094347, -178.175378]), [36.19352, 22.86988, -12.37860, 88.83085, -66.57439, -81.72795], [0.0, 0.0, 1.0])
    robot.MoveC(Pose([2468.239418, 1130.614560, 1333.549802, -180.000000, -3.591795, -180.000000]), [28.37934, 35.45210, -28.96667, 85.54799, -28.41204, -83.00289], Pose([2457.128674, 797.241647, 1156.545094, 180.000000, -37.427062, -180.000000]), [18.58928, 43.77805, -40.05410, 155.58093, -37.76022, -148.70252], [0.0, 0.0, 1.0], [0.0, 0.0, 1.0])
    robot.MoveL(Pose([2457.128674, 797.241647, 1156.545094, 180.000000, -37.427062, -180.000000]), [18.58928, 43.77805, -40.05410, 155.58093, -37.76022, -148.70252], [0.0, 0.0, 1.0])
    robot.MoveL(Pose([2469.684137, 397.051453, 1356.565545, -180.000000, -3.591795, -180.000000]), [10.73523, 21.17902, -10.22963, 56.13802, -12.93695, -54.77268], [0.0, 0.0, 1.0])
    robot.MoveL(Pose([2494.452316, 404.343933, 1751.146172, -180.000000, -3.591795, -180.000000]), [10.80299, 25.05092, -31.54821, 132.79244, -14.76878, -133.06820], [0.0, 0.0, 1.0])
    robot.MoveL(Pose([2494.452316, 834.649436, 1751.146172, -180.000000, -3.591795, -180.000000]), [21.49850, 33.45974, -43.37980, 121.21995, -25.32130, -122.42907], [0.0, 0.0, 1.0])
    robot.setZoneData(5.000)
    robot.MoveL(Pose([2147.781731, 834.649436, 1772.906995, -180.000000, -3.591795, -180.000000]), [25.21677, 13.65153, -17.95808, 107.03387, -26.40518, -107.19412], [0.0, 0.0, 1.0])
    robot.MoveL(Pose([2147.781731, 375.769504, 1772.906995, -180.000000, -3.591795, -180.000000]), [11.97030, 5.74930, -8.96838, 119.55454, -13.76610, -119.51539], [0.0, 0.0, 1.0])
    robot.MoveL(Pose([2147.781731, 61.363728, 1772.906995, -180.000000, -3.591795, -180.000000]), [1.98292, 3.75693, -6.84136, -16.54793, 6.96416, 16.55673], [0.0, 0.0, 0.0])
    robot.RunCode(r'ArcLEnd', True)
    robot.MoveL(Pose([2147.781731, 275.581430, 1772.906995, -180.000000, -3.591795, -180.000000]), [8.83799, 4.80606, -7.95436, 127.27676, -11.11070, -127.24243], [0.0, 0.0, 1.0])
    robot.ProgFinish(r'Prog1')
    for line in robot.PROG:
        print(line)
    #print(robot.PROG)
    if len(robot.LOG) > 0:
        mbox('Program generation LOG:\n\n' + robot.LOG)
    input("Press Enter to close...")

if __name__ == "__main__":
    """Function to call when the module is executed by itself: test"""
    test_post()
