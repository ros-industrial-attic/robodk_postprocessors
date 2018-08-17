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
# for a CLOOS robot (TXT file and PKT)
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


def get_safe_name(progname, max_chars = 7):
    """Get a safe program name"""
    # Remove special characters
    for c in r'-[]/\;,><&*:%=+@!#^()|?^':
        progname = progname.replace(c,'')
    # Set a program name by default:
    if len(progname) <= 0:
        progname = 'Program'
    # Force the program to start with a letter (not a number)
    if progname[0].isdigit():
        progname = 'P' + progname
    # Set the maximum size of a program (number of characters)
    if len(progname) > max_chars:
        progname = progname[:max_chars]
    return progname


# ----------------------------------------------------
# Import RoboDK tools
from robodk import *
import sys

# ----------------------------------------------------    
# Object class that handles the robot instructions/syntax
class RobotPost(object):
    """Robot post object defined for Motoman robots"""
    # Enter the robot model:
    PATH_MAKE_SP = 'C:/Program Files (x86)/CLOOS/'
    ROBOT_MODEL = '310'
    SERIAL_NR = '3511462' 
    PROG_EXT = 'txt'             # set the program extension
    MAX_LINES_X_PROG = 5000      # maximum number of lines per program. It will then generate multiple "pages (files)". This can be overriden by RoboDK settings.    
    SPEED_MMS = 50               # Default speed in mm/s
    
    INCLUDE_SUB_PROGRAMS = True # Generate sub programs

    PULSES_ZERO = 4194304 # zero position for pulses
    
    # Pulses per degree (provide these in the robot parameters menu: Double click the motoman robot in RoboDK, select "Parameters"
    PULSES_X_DEG = [1,1,1,1,1,1] 

    # PROG specific variables:
    LINE_COUNT = 0      # Count the number of instructions (limited by MAX_LINES_X_PROG)
    P_COUNT = 0         # Count the number of P targets in one file
    C_COUNT = 0         # Count the number of P targets in one file
    nProgs = 0          # Count the number of programs and sub programs
    
    # other variables
    ROBOT_POST = ''
    ROBOT_NAME = ''
    PROG_FILES = [] # List of Program files to be uploaded through FTP
    
    PROG_NAMES = [] # List of PROG NAMES
    PROG_LIST = [] # List of PROG 
    PROG_TARGETS_LIST = [] # List of PROG     
    
    PROG_NAME = 'unknown'  # Original name of the current program (example: ProgA)
    PROG_NAME_CURRENT = 'unknown' # Auto generated name (different from PROG_NAME if we have more than 1 page per program. Example: ProgA2)
    
    nPages = 0           # Count the number of pages
    PROG_NAMES_MAIN = [] # List of programs called by a main program due to splitting
    
    PROG = []     # Save the program lines
    PROG_TARGETS = []  # Save the program lines (targets section)
    LOG = '' # Save a log
    
    nAxes = 6 # Important: This is usually provided by RoboDK automatically. Otherwise, override the __init__ procedure. 
    AXES_TYPE = ['R','R','R','R','R','R']  # Important: This is usually set up by RoboDK automatically. Otherwise, override the __init__ procedure.
    # 'R' for rotative axis, 'L' for linear axis, 'T' for external linear axis (linear track), 'J' for external rotative axis (turntable)
    #AXES_TYPE = ['R','R','R','R','R','R','T','J','J'] #example of a robot with one external linear track axis and a turntable with 2 rotary axes
    AXES_TRACK = []
    AXES_TURNTABLE = []
    HAS_TRACK = False
    HAS_TURNTABLE = False
    
    # Specific to ARC welding applications
    SPEED_BACKUP = None
    LAST_POSE = None
    POSE_FRAME = eye(4)
    POSE_FRAME = eye(4)
    
    def __init__(self, robotpost=None, robotname=None, robot_axes = 6, **kwargs):
        self.ROBOT_POST = robotpost
        self.ROBOT_NAME = robotname
        self.nAxes = robot_axes
        self.PROG = []
        self.LOG = ''
        #for k,v in kwargs.iteritems(): # python2
        for k,v in kwargs.items():
            if k == 'lines_x_prog':
                self.MAX_LINES_X_PROG = v
            if k == 'axes_type':
                self.AXES_TYPE = v                
            if k == 'pulses_x_deg':
                self.PULSES_X_DEG = v    
                
        for i in range(len(self.AXES_TYPE)):
            if self.AXES_TYPE[i] == 'T':
                self.AXES_TRACK.append(i)
                self.HAS_TRACK = True
            elif self.AXES_TYPE[i] == 'J':
                self.AXES_TURNTABLE.append(i)
                self.HAS_TURNTABLE = True        

                
    def ProgStart(self, progname, new_page = False):
        progname = get_safe_name(progname)
        progname_i = progname
        if new_page:
            #nPages = len(self.PROG_LIST)
            if self.nPages == 0:
                if len(self.PROG_NAMES_MAIN) > 0:
                    print("Can't split %s: Two or more programs are split into smaller programs" % progname)
                    print(self.PROG_NAMES_MAIN)
                    raise Exception("Only one program at a time can be split into smaller programs")
                self.PROG_NAMES_MAIN.append(self.PROG_NAME) # add the first program in the list to be genrated as a subprogram call
                self.nPages = self.nPages + 1

            self.nPages = self.nPages + 1
            progname_i = "%s%i" % (self.PROG_NAME, self.nPages)          
            self.PROG_NAMES_MAIN.append(progname_i)
            
        else:
            if self.nProgs > 1 and not self.INCLUDE_SUB_PROGRAMS:
                return
            self.PROG_NAME = progname
            self.nProgs = self.nProgs + 1
            self.MoveJ(None, [0]*self.nAxes)
            #self.PROG_NAMES = []
            
        self.PROG_NAME_CURRENT = progname_i
        self.PROG_NAMES.append(progname_i)
        
    def ProgFinish(self, progname, new_page = False):
        progname = get_safe_name(progname)
        if not new_page:
            # Reset page count
            self.nPages = 0
        
        header = ''
        header += 'RESTART' + '\n'
        header += 'LIST 1=(4211,3,0,49,91,20,530,0,0,40,50,0,0,0,0,0,0,3,0,0,3,0)' + '\n'
        header += '' + '\n'

        header += 'MAIN' + '\n'
        header += 'STCP (10,0,4998)' + '\n'
        header += 'STOV (-1,0,-1)' + '\n'
        header += '$ (1)' + '\n'        
        
        header_pkt = ''
        header_pkt += '( Robot     : %s )' % self.ROBOT_MODEL + '\n'
        header_pkt += '( Serial Nr : %s )' % self.SERIAL_NR + '\n'
        header_pkt += '( Achszahl  : %i )' % self.nAxes + '\n'
        header_pkt += '( Resolution: 2:2:2:2:2:2: Konfigend)'
               
        self.PROG.insert(0, header)
        self.PROG.append('\nEND')
        
        self.PROG_TARGETS.insert(0, header_pkt)
        
        # Save PROG in PROG_LIST
        self.PROG_LIST.append(self.PROG)
        self.PROG_TARGETS_LIST.append(self.PROG_TARGETS)
        
        self.PROG = []
        self.PROG_TARGETS = []
        self.LINE_COUNT = 0
        self.P_COUNT = 0
        self.C_COUNT = 0        
                
#RESTART
#LIST 1=(4211,3,0,49,91,20,530,0,0,40,50,0,0,0,0,0,0,3,0,0,3,0)
#
#MAIN
#$ (1)
        
    def progsave(self, folder, progname, ask_user = False, show_result = False):
        print(folder)
        if not folder.endswith('/'):
            folder = folder + '/'
        progname = progname + '.' + self.PROG_EXT     
        if ask_user or not DirExists(folder):
            filesave = getSaveFile(folder, progname, 'Save program as...')
            if filesave is not None:
                filesave = filesave.name
            else:
                return
        else:
            filesave = folder + progname
        
        # Save TXT file
        fid = open(filesave, "w")
        for line in self.PROG:
            fid.write(line)
            fid.write('\n')
        fid.close()
        
        # Save PKT file
        filesave_pkt = filesave[:-3]+'pkt'
        fid2 = open(filesave_pkt, "w")
        for line in self.PROG_TARGETS:
            fid2.write(line)
            fid2.write('\n')
        fid2.close()        
        
        print('SAVED: %s\n' % filesave) # tell RoboDK the path of the saved file
        self.PROG_FILES.append(filesave)

        
        # open file with default application
        if show_result:
            if type(show_result) is str:
                # Open file with provided application
                import subprocess
                p = subprocess.Popen([show_result, filesave, filesave_pkt])
            elif type(show_result) is list:
                import subprocess
                p = subprocess.Popen(show_result + [filesave])   
            else:
                # open file with default application
                import os
                os.startfile(filesave)
            #if len(self.LOG) > 0:
            #    mbox('Program generation LOG:\n\n' + self.LOG)
            
            
        # -------- build with CONVSP ---------
        if FileExists(self.PATH_MAKE_SP + 'CONVSP.exe'):
            filesave_S = filesave[:-4] + 'S'
            filesave_P = filesave[:-4] + 'P'            
            print("POPUP: Compiling S file with CONVSP.exe: %s..." % progname)
            sys.stdout.flush()
            import subprocess
            command_list = []
            command_list.append([self.PATH_MAKE_SP + 'CONVSP', filesave.replace('/','\\'), filesave_S.replace('/','\\')])
            command_list.append([self.PATH_MAKE_SP + 'CONVSP', filesave_pkt.replace('/','\\'), filesave_P.replace('/','\\')])            
            
            #output = subprocess.check_output(command)
            #self.LOG = output.decode('utf-8')
            self.LOG += 'Program generation for: ' + progname + '\n'
            for command in command_list:
                with subprocess.Popen(command, stdout=subprocess.PIPE, bufsize=1, universal_newlines=True) as p:
                    for line in p.stdout:
                        line_ok = line.strip()
                        self.LOG += line_ok + '\n'
                        print("POPUP: " + line_ok)
                        sys.stdout.flush()
            self.LOG += '\n'
            
            
    def ProgSave(self, folder, progname, ask_user = False, show_result = False):
        progname = get_safe_name(progname)
        nfiles = len(self.PROG_LIST)
        if nfiles >= 1:
            if self.LINE_COUNT > 0:
                # Progfinish was not called!
                print("Warning: ProgFinish was not called properly")
                self.PROG_LIST.append(self.PROG)
                self.PROG_TARGETS_LIST.append(self.PROG_TARGETS)
                self.PROG_NAMES.append("Unknown")
                self.PROG = []
                self.PROG_TARGETS = []                
                self.LINE_COUNT = 0
                self.P_COUNT = 0
                self.C_COUNT = 0      
            
            if len(self.PROG_NAMES_MAIN) > 1:
                # Warning: the program might be cut to a maximum number of chars
                progname_main = "M_" + self.PROG_NAMES_MAIN[0]
                self.INCLUDE_SUB_PROGRAMS = True # Force generation of main program
                self.ProgStart(progname_main)
                for prog_call in self.PROG_NAMES_MAIN:
                    self.RunCode(prog_call, True)
                    
                self.ProgFinish(progname_main)
            
            # Save the last program added to the PROG_LIST
            self.PROG = self.PROG_LIST.pop()
            self.PROG_TARGETS = self.PROG_TARGETS_LIST.pop()
            progname_last = self.PROG_NAMES.pop()
            self.progsave(folder, progname_last, ask_user, show_result)
            #-------------------------
            #self.LOG = ''
            if len(self.PROG_FILES) == 0:
                # cancelled by user
                return
                
            first_file = self.PROG_FILES[0]
            folder_user = getFileDir(first_file)
            # progname_user = getFileName(self.FILE_SAVED)
            
            # Generate each program
            for i in range(len(self.PROG_LIST)):
                self.PROG = self.PROG_LIST[i]
                self.PROG_TARGETS = self.PROG_TARGETS_LIST[i]
                self.progsave(folder_user, self.PROG_NAMES[i], False, show_result)
                
        elif nfiles == 1:
            self.PROG = self.PROG_LIST[0]
            self.PROG_TARGETS = self.PROG_TARGETS_LIST[0]
            self.progsave(folder, progname, ask_user, show_result)
            
        else:
            print("Warning! Program has not been properly finished")
            self.progsave(folder, progname, ask_user, show_result)

        if show_result and len(self.LOG) > 0:
            mbox('Program generation LOG:\n\n' + self.LOG)
        
    def ProgSendRobot(self, robot_ip, remote_path, ftp_user, ftp_pass):
        """Send a program to the robot using the provided parameters. This method is executed right after ProgSave if we selected the option "Send Program to Robot".
        The connection parameters must be provided in the robot connection menu of RoboDK"""
        UploadFTP(self.PROG_FILES, robot_ip, remote_path, ftp_user, ftp_pass)
        
    def MoveJ(self, pose, joints, conf_RLF=None):
        """Add a joint movement"""
        self.page_size_control() # Important to control the maximum lines per program and not save last target on new program
        target_id = self.add_target_joints(joints, 1)
        self.addline("GP (%i)" % (target_id))
        self.LAST_POSE = pose
        
    def MoveL(self, pose, joints, conf_RLF=None):
        """Add a linear movement"""        
        self.page_size_control() # Important to control the maximum lines per program and not save last target on new program                
        target_id = -1
        if pose is None:
            target_id = self.add_target_joints(joints)
        else:
            target_id = self.add_target_cartesian(self.POSE_FRAME*pose, joints, conf_RLF)

        self.addline("GC (%i)" % (target_id))
        self.LAST_POSE = pose
        
    def MoveC(self, pose1, joints1, pose2, joints2, conf_RLF_1=None, conf_RLF_2=None):
        """Add a circular movement"""
        self.page_size_control() # Important to control the maximum lines per program and not save last target on new program
        target_id1 = self.add_target_cartesian(self.POSE_FRAME*pose1, joints1, conf_RLF_1)
        target_id2 = self.add_target_cartesian(self.POSE_FRAME*pose2, joints2, conf_RLF_2)        
        self.addline("ARC (%i,%i,%i)" % (target_id1-1,target_id1,target_id2))        
        
    def setFrame(self, pose, frame_id, frame_name):
        """Change the robot reference frame"""
        xyzwpr = Pose_2_Motoman(pose)
        self.POSE_FRAME = pose
        self.RunMessage('Using %s (targets wrt base):' % (str(frame_name)), True)
        self.RunMessage('%.1f,%.1f,%.1f,%.1f,%.1f,%.1f' % (xyzwpr[0], xyzwpr[1], xyzwpr[2], xyzwpr[3], xyzwpr[4], xyzwpr[5]), True)
        
    def setTool(self, pose, tool_id, tool_name):
        """Change the robot TCP"""
        xyzwpr = Pose_2_Motoman(pose)
        self.RunMessage('Tool %s should be close to:' % (str(tool_name)), True)
        self.RunMessage('%.1f,%.1f,%.1f,%.1f,%.1f,%.1f' % (xyzwpr[0], xyzwpr[1], xyzwpr[2], xyzwpr[3], xyzwpr[4], xyzwpr[5]), True)
        
    def Pause(self, time_ms):
        """Pause the robot program"""
        if time_ms <= 0:
            self.addline('PAUSE')
        else:
            self.addline('WAITM %.0f' % (time_ms))
        
    def setSpeed(self, speed_mms):
        """Changes the robot speed (in mm/s)"""
        self.SPEED_MMS = speed_mms
    
    def setAcceleration(self, accel_mmss):
        """Changes the robot acceleration (in mm/s2)"""
        self.addlog('Set acceleration not defined')
    
    def setSpeedJoints(self, speed_degs):
        """Changes the robot joint speed (in deg/s)"""
        speedj = max(0.01,min(speed,100.0)) # Joint speed must be in %
        if speedj < 100:
            self.STR_VJ = "VJ=%.2f" % speedj
        else:
            self.STR_VJ = "VJ=%.1f" % speedj
    
    def setAccelerationJoints(self, accel_degss):
        """Changes the robot joint acceleration (in deg/s2)"""
        self.addlog('Set acceleration not defined')
        
    def setZoneData(self, zone_mm):
        """Changes the zone data approach (makes the movement more smooth)"""
        if zone_mm < 0:
            self.STR_PL = ''
        else:
            self.STR_PL = ' PL=%i' % round(min(zone_mm/25, 4))        
        
    def setDO(self, io_var, io_value):
        """Sets a variable (output) to a given value"""
        if type(io_var) != str:  # set default variable name if io_var is a number
            io_var = 'OT#(%s)' % str(io_var)        
        if type(io_value) != str: # set default variable value if io_value is a number            
            if io_value > 0:
                io_value = 'ON'
            else:
                io_value = 'OFF'
        
        # at this point, io_var and io_value must be string values
        #DOUT OT#(2) ON
        self.addline('DOUT %s %s' % (io_var, io_value))
        
    def waitDI(self, io_var, io_value, timeout_ms=-1):
        """Waits for an input io_var to attain a given value io_value. Optionally, a timeout can be provided."""
        if type(io_var) != str:  # set default variable name if io_var is a number
            io_var = 'IN#(%s)' % str(io_var)        
        if type(io_value) != str: # set default variable value if io_value is a number            
            if io_value > 0:
                io_value = 'ON'
            else:
                io_value = 'OFF'
        
        # at this point, io_var and io_value must be string values
        if timeout_ms < 0:
            #WAIT IN#(12)=ON
            self.addline('WAIT %s=%s' % (io_var, io_value))
        else:
            #self.LBL_ID_COUNT = self.LBL_ID_COUNT + 1
            self.addline('WAIT %s=%s T=%.2f' % (io_var, io_value, timeout_ms*0.001))
       
            
    def RunCode(self, code, is_function_call = False):
        """Adds code or a function call"""
        if is_function_call:
            code = get_safe_name(code)
            #if code.startswith("ArcStart"):
                #return
                
            # default program call
            if code.startswith("Extrude"):
                return
            
            code.replace(' ','_')
            self.addline('CALL %s' % (code))
        else:
            #if code.endswith(';'):
                #code = code[:-1]
            self.addline(code)
        
    def RunMessage(self, message, iscomment = False):
        """Add a joint movement"""
        if iscomment:
            self.addline("! %s" % message)
            
        else:
            self.addline('! MSG %s' % message)
        
# ------------------ private ----------------------
    def page_size_control(self):
        if self.LINE_COUNT >= self.MAX_LINES_X_PROG:
            self.ProgFinish(self.PROG_NAME, True)
            self.ProgStart(self.PROG_NAME, True)


    def addline(self, newline, movetype = ' '):
        """Add a program line"""
        if self.nProgs > 1 and not self.INCLUDE_SUB_PROGRAMS:
            return
        
        self.page_size_control()        
        self.LINE_COUNT = self.LINE_COUNT + 1
        self.PROG.append(newline)
            
    def addline_targets(self, newline):
        """Add a line at the end of the program (used for targets)"""
        self.PROG_TARGETS.append(newline)
        
    def addlog(self, newline):
        """Add a log message"""
        if self.nProgs > 1 and not self.INCLUDE_SUB_PROGRAMS:
            return
            
        self.LOG = self.LOG + newline + '\n'
        
# ------------------ targets ----------------------        
    def add_target_joints(self, joints, interpolation = 0):    
        if self.nProgs > 1 and not self.INCLUDE_SUB_PROGRAMS:
            return  
    

        self.C_COUNT = self.C_COUNT + 1
        cid = self.C_COUNT
        
        str_pulses=[]        
        for i in range(len(joints)):
            str_pulses.append('%010i' % round(self.PULSES_ZERO + joints[i] * self.PULSES_X_DEG[i]))
        
        speed = self.SPEED_MMS
        outputs = 0
        self.addline_targets('%05i,%05i,%05i,%05i,' % (cid, speed, interpolation, outputs) + ','.join(str_pulses))                
        return cid
        
#cid,speed,0,0,
#00000,00000,00000,00000,004194304,004194304,004194304,004194304,004194304,004194304
#00001,00050,00000,00001,004294285,005054619,004652139,004286872,003928097,004116772
#00002,00050,00000,00000,004321619,005055554,004649218,004312598,003928967,004111329
#00003,00050,00000,00000,004346289,005060069,004635445,004338196,003934233,004105402
    
    def add_target_cartesian(self, pose, joints, conf_RLF):           
        if self.nProgs > 1 and not self.INCLUDE_SUB_PROGRAMS:
            return
            
        return self.add_target_joints(joints)
            
        xyzwpr = Pose_2_Motoman(pose)
        
        if conf_RLF is None:
            conf_RLF = [0,0,0]

        turns = [0,0,0]
        if len(joints) >= 6:
            turnJ4 = (joints[3]+180)//360
            turnJ6 = (joints[5]+180)//360
            turnJ1 = (joints[0]+180)//360
            turns = [turnJ4, turnJ6, turnJ1]

        confdata = '%i,%i,%i,%i,%i,%i,0,0' % tuple(conf_RLF[:3] + turns[:3])      
        self.C_COUNT = self.C_COUNT + 1
        cid = self.C_COUNT
        self.addline_targets('%i,' % cid + '%.3f,%.3f,%.3f,%.2f,%.2f,%.2f' % tuple(xyzwpr))
        return cid
   



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

    robot = RobotPost('CLOOS test', 'CLOOS robot robot', 6)

    robot.ProgStart("Program")
    robot.RunMessage("Program generated by RoboDK", True)
    robot.setFrame(Pose([807.766544, -963.699898, 41.478944, 0, 0, 0]), None, 0)
    robot.setTool(Pose([62.5, -108.253175, 100, -60, 90, 0]), None, 0)
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
    
    robot.PROG = robot.PROG_LIST.pop()
    print("\n\n--------------- TXT file ----------------\n")
    for line in robot.PROG:
        print(line)
        
    print("\n\n--------------- PKT file ----------------\n")
    robot.PROG_TARGETS = robot.PROG_TARGETS_LIST.pop()
    for line in robot.PROG_TARGETS:
        print(line)
    
    if len(robot.LOG) > 0:
        mbox('Program generation LOG:\n\n' + robot.LOG)

    input("Press Enter to close...")

if __name__ == "__main__":
    """Function to call when the module is executed by itself: test"""
    test_post()
