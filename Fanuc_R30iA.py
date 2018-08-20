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
# for a generic Fanuc robot with RoboDK
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


def get_safe_name(progname, max_chars = 10):
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
    """Robot post object defined for Fanuc robots"""
    PROG_EXT = 'LS'             # set the program extension
    MAX_LINES_X_PROG = 9999    # maximum number of lines per program. It will then generate multiple "pages (files)". This can be overriden by RoboDK settings.
    INCLUDE_SUB_PROGRAMS = True # Generate sub programs
    JOINT_SPEED = '20%'     # set default joint speed motion
    SPEED = '500mm/sec'     # set default cartesian speed motion  
    CNT_VALUE = 'FINE'      # set default CNT value (all motion until smooth value is changed)
    ACTIVE_UF = 9           # Active UFrame Id (register)
    ACTIVE_UT = 9           # Active UTool Id (register)
    SPARE_PR = 9            # Spare Position register for calculations

    # PROG specific variables:
    LINE_COUNT = 0 # Count the number of instructions (limited by MAX_LINES_X_PROG)
    P_COUNT = 0   # Count the number of P targets in one file
    nProgs = 0    # Count the number of programs and sub programs
    LBL_ID_COUNT = 0  # Number of labels used
    
    # other variables
    ROBOT_POST = ''
    ROBOT_NAME = ''
    PROG_FILES = [] # List of Program files to be uploaded through FTP
    
    PROG_NAMES = [] # List of PROG NAMES
    PROG_LIST = [] # List of PROG 
    
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
            #self.PROG_NAMES = []
            
        self.PROG_NAME_CURRENT = progname_i
        self.PROG_NAMES.append(progname_i)
        
    def ProgFinish(self, progname, new_page = False):
        progname = get_safe_name(progname)
        if not new_page:
            # Reset page count
            self.nPages = 0
            
        #if self.nPROGS > 1:
        #    # Fanuc does not support defining multiple programs in the same file, so one program per file
        #    return
        header = ''
        header = header + ('/PROG  %s' % self.PROG_NAME_CURRENT) + '\n' # Use the latest name set at ProgStart
        header = header + '/ATTR' + '\n'
        header = header + 'OWNER\t\t= MNEDITOR;' + '\n'
        header = header + 'COMMENT\t\t= "RoboDK sequence";' + '\n'
        header = header + 'PROG_SIZE\t= 0;' + '\n'
        header = header + 'CREATE\t\t= DATE 31-12-14  TIME 12:00:00;' + '\n'
        header = header + 'MODIFIED\t= DATE 31-12-14  TIME 12:00:00;' + '\n'
        header = header + 'FILE_NAME\t= ;' + '\n'
        header = header + 'VERSION\t\t= 0;' + '\n'
        header = header + ('LINE_COUNT\t= %i;' % (self.LINE_COUNT)) + '\n'
        header = header + 'MEMORY_SIZE\t= 0;' + '\n'
        header = header + 'PROTECT\t\t= READ_WRITE;' + '\n'
        header = header + 'TCD:  STACK_SIZE\t= 0,' + '\n'
        header = header + '      TASK_PRIORITY\t= 50,' + '\n'
        header = header + '      TIME_SLICE\t= 0,' + '\n'
        header = header + '      BUSY_LAMP_OFF\t= 0,' + '\n'
        header = header + '      ABORT_REQUEST\t= 0,' + '\n'
        header = header + '      PAUSE_REQUEST\t= 0;' + '\n'
        #header = header + 'DEFAULT_GROUP\t= 1,*,*,*,*;' + '\n'  #old controllers
        header = header + 'DEFAULT_GROUP\t= 1,*,*,*,*,*,*;' + '\n'
        header = header + 'CONTROL_CODE\t= 00000000 00000000;' + '\n'
        if self.HAS_TURNTABLE:
            header = header + '/APPL' + '\n'
            header = header + '' + '\n'
            header = header + 'LINE_TRACK;' + '\n'
            header = header + 'LINE_TRACK_SCHEDULE_NUMBER      : 0;' + '\n'
            header = header + 'LINE_TRACK_BOUNDARY_NUMBER      : 0;' + '\n'
            header = header + 'CONTINUE_TRACK_AT_PROG_END      : FALSE;' + '\n'
            header = header + '' + '\n'
        header = header + '/MN'
        #header = header + '/MN' + '\n'    # Important! Last line should not have \n
        
        self.PROG.insert(0, header)
        self.PROG.append('/POS')
        self.PROG += self.PROG_TARGETS
        self.PROG.append('/END')
        
        # Save PROG in PROG_LIST
        self.PROG_LIST.append(self.PROG)
        self.PROG = []
        self.PROG_TARGETS = []
        #self.nLines = 0
        self.LINE_COUNT = 0
        self.P_COUNT = 0
        self.LBL_ID_COUNT = 0
        
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
        fid = open(filesave, "w")
        #fid.write(self.PROG)
        for line in self.PROG:
            fid.write(line)
            fid.write('\n')
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
            #if len(self.LOG) > 0:
            #    mbox('Program generation LOG:\n\n' + self.LOG)
        # -------- build with MakeTP ---------
        # set robot first with setrobot.exe (delete robot.ini file)
        PATH_MAKE_TP = 'C:/Program Files (x86)/FANUC/WinOLPC/bin/'
        if FileExists(PATH_MAKE_TP + 'MakeTP.exe'):
            filesave_TP = filesave[:-3] + '.TP'
            print("POPUP: Compiling LS file with MakeTP.exe: %s..." % progname)
            sys.stdout.flush()
            import subprocess
            command = [PATH_MAKE_TP + 'MakeTP.exe', filesave.replace('/','\\'), filesave_TP.replace('/','\\'), '/config', PATH_MAKE_TP + 'robot.ini']
            #output = subprocess.check_output(command)
            #self.LOG = output.decode('utf-8')
            self.LOG += 'Program generation for: ' + progname + '\n'
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
                self.PROG_NAMES.append("Unknown")
                self.PROG = []
                self.LINE_COUNT = 0
            
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
                self.progsave(folder_user, self.PROG_NAMES[i], False, show_result)
                
        elif nfiles == 1:
            self.PROG = self.PROG_NAMES[0]
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
        
        target_id = self.add_target_joints(pose, joints)
        move_ins = 'P[%i] %s %s ;' % (target_id, self.JOINT_SPEED, self.CNT_VALUE)
        self.addline(move_ins, 'J')
        self.LAST_POSE = pose
        
    def MoveL(self, pose, joints, conf_RLF=None):
        """Add a linear movement"""
        
        #if self.LAST_POSE is not None and pose is not None:
        #    # Skip adding a new movement if the new position is the same as the last one
        #    if distance(pose.Pos(), self.LAST_POSE.Pos()) < 0.1 and pose_angle_between(pose, self.LAST_POSE) < 0.1:
        #        return

        self.page_size_control() # Important to control the maximum lines per program and not save last target on new program
                
        if pose is None:
            target_id = self.add_target_joints(pose, joints)
            move_ins = 'P[%i] %s %s ;' % (target_id, self.SPEED, self.CNT_VALUE)
        else:
            target_id = self.add_target_cartesian(pose, joints, conf_RLF)
            move_ins = 'P[%i] %s %s ;' % (target_id, self.SPEED, self.CNT_VALUE)
            
        self.addline(move_ins, 'L')
        self.LAST_POSE = pose
        
    def MoveC(self, pose1, joints1, pose2, joints2, conf_RLF_1=None, conf_RLF_2=None):
        """Add a circular movement"""
        self.page_size_control() # Important to control the maximum lines per program and not save last target on new program
        
        target_id1 = self.add_target_cartesian(pose1, joints1, conf_RLF_1)
        target_id2 = self.add_target_cartesian(pose2, joints2, conf_RLF_2)
        move_ins = 'P[%i] \n       P[%i] %s %s ;' % (target_id1, target_id2, self.SPEED, self.CNT_VALUE)
        self.addline(move_ins, 'C')
        self.LAST_POSE = pose2
        
    def setFrame(self, pose, frame_id=None, frame_name=None):
        """Change the robot reference frame"""
        xyzwpr = Pose_2_Fanuc(pose)
        if frame_id is None or frame_id < 0:            
            for i in range(6):
                self.addline('PR[%i,%i]=%.3f ;' % (self.SPARE_PR, i+1, xyzwpr[i]))
            for i in range(6,self.nAxes):
                self.addline('PR[%i,%i]=%.3f ;' % (self.SPARE_PR, i+1, 0))
            self.addline('UFRAME[%i]=PR[%i] ;' % (self.ACTIVE_UF, self.SPARE_PR))
            self.addline('UFRAME_NUM=%i ;' % (self.ACTIVE_UF))
        else:
            self.ACTIVE_UF = frame_id
            self.addline('UFRAME_NUM=%i ;' % (self.ACTIVE_UF))
            self.RunMessage('UF%i:%.1f,%.1f,%.1f,%.1f,%.1f,%.1f' % (frame_id, xyzwpr[0], xyzwpr[1], xyzwpr[2], xyzwpr[3], xyzwpr[4], xyzwpr[5]), True)
        
    def setTool(self, pose, tool_id=None, tool_name=None):
        """Change the robot TCP"""
        xyzwpr = Pose_2_Fanuc(pose)
        if tool_id is None or tool_id < 0:
            for i in range(6):
                self.addline('PR[%i,%i]=%.3f ;' % (self.SPARE_PR, i+1, xyzwpr[i]))
            for i in range(6,self.nAxes):
                self.addline('PR[%i,%i]=%.3f ;' % (self.SPARE_PR, i+1, 0))
            self.addline('UTOOL[%i]=PR[%i] ;' % (self.ACTIVE_UT, self.SPARE_PR))
            self.addline('UTOOL_NUM=%i ;' % (self.ACTIVE_UT))
        else:
            self.ACTIVE_UT = tool_id
            self.addline('UTOOL_NUM=%i ;' % (self.ACTIVE_UT))
            self.RunMessage('UT%i:%.1f,%.1f,%.1f,%.1f,%.1f,%.1f' % (tool_id, xyzwpr[0], xyzwpr[1], xyzwpr[2], xyzwpr[3], xyzwpr[4], xyzwpr[5]), True)
        
    def Pause(self, time_ms):
        """Pause the robot program"""
        if time_ms <= 0:
            self.addline('PAUSE ;')
        else:
            self.addline('WAIT  %.2f(sec) ;' % (time_ms*0.001))
        
    def setSpeed(self, speed_mms):
        """Changes the robot speed (in mm/s)"""
        if self.SPEED_BACKUP is None:
            # Set the normal speed
            self.SPEED = '%.0fmm/sec' % max(speed_mms, 0.01)
            # assume 5000 mm/s as 100%
            self.JOINT_SPEED = '%.0f%%' % max(min(100.0*speed_mms/5000.0, 100.0), 1) # Saturate percentage speed between 1 and 100

        else:
            # Do not alter the speed as we are in ARC movement mode
            # skip speed settings if it has been overriden
            self.SPEED_BACKUP = '%.0fmm/sec' % max(speed_mms, 0.01)
            # assume 5000 mm/s as 100%
            #self.JOINT_SPEED = '%.0f%%' % max(min(100.0*speed_mms/5000.0, 100.0), 1) # Saturate percentage speed between 1 and 100
    
    def setAcceleration(self, accel_mmss):
        """Changes the robot acceleration (in mm/s2)"""
        self.addlog('setAcceleration not defined')
    
    def setSpeedJoints(self, speed_degs):
        """Changes the robot joint speed (in deg/s)"""
        #self.addlog('setSpeedJoints not defined')
        self.JOINT_SPEED = '%.0f%%' % max(min(100.0*speed_degs/200.0, 100.0), 1) # Saturate percentage speed between 1 and 100
    
    def setAccelerationJoints(self, accel_degss):
        """Changes the robot joint acceleration (in deg/s2)"""
        self.addlog('setAccelerationJoints not defined')
        
    def setZoneData(self, zone_mm):
        """Changes the zone data approach (makes the movement more smooth)"""
        if zone_mm < 0:
            self.CNT_VALUE = 'FINE'
        else:
            self.CNT_VALUE = 'CNT%i' % round(min(zone_mm, 100.0))        
        
    def setDO(self, io_var, io_value):
        """Sets a variable (output) to a given value"""
        if type(io_var) != str:  # set default variable name if io_var is a number
            io_var = 'DO[%s]' % str(io_var)        
        if type(io_value) != str: # set default variable value if io_value is a number            
            if io_value > 0:
                io_value = 'ON'
            else:
                io_value = 'OFF'
        
        # at this point, io_var and io_value must be string values
        self.addline('%s=%s ;' % (io_var, io_value))
        
    def waitDI(self, io_var, io_value, timeout_ms=-1):
        """Waits for an input io_var to attain a given value io_value. Optionally, a timeout can be provided."""
        if type(io_var) != str:  # set default variable name if io_var is a number
            io_var = 'DI[%s]' % str(io_var)        
        if type(io_value) != str: # set default variable value if io_value is a number            
            if io_value > 0:
                io_value = 'ON'
            else:
                io_value = 'OFF'
        
        # at this point, io_var and io_value must be string values
        if timeout_ms < 0:
            self.addline('WAIT %s=%s ;' % (io_var, io_value))
        else:
            self.LBL_ID_COUNT = self.LBL_ID_COUNT + 1
            self.addline('$WAITTMOUT=%i ;' % round(timeout_ms))
            self.addline('WAIT %s=%s TIMEOUT, LBL[%i] ;' % (io_var, io_value, self.LBL_ID_COUNT))
            self.addline('MESSAGE[Timed out for LBL %i] ;' % self.LBL_ID_COUNT)
            self.addline('PAUSE ;')
            self.addline('LBL[%i] ;' % self.LBL_ID_COUNT)
       
    def addlastline(self, add_params):
        """Add parameters to the last command"""
        if len(self.PROG) > 0 and self.PROG[-1].endswith(';\n'):
            self.PROG[-1] = self.PROG[-1][:-2] + add_params + ';' # remove last 2 characters
            
    def RunCode(self, code, is_function_call = False):
        """Adds code or a function call"""
        if is_function_call:
            code = get_safe_name(code, 12)
            if code.startswith("ArcStart"):
                if not code.endswith(')'):
                    code = code + '()'
                
                self.ARC_PARAMS = code[9:-1]
                if len(self.ARC_PARAMS) < 1:
                    # Use default sine wave parameters
                    self.ARC_PARAMS = '2.0Hz,8.0mm,0.075s,0.075'
                
                # use desired weld speed:
                self.SPEED_BACKUP = self.SPEED
                self.SPEED = 'WELD_SPEED'
                
                # Provoke ARC START:
                self.addlastline('Arc Start[11]')
                
                # Tune weave with parameters
                self.addline('Weave Sine[%s] ;' % self.ARC_PARAMS)
                return # Do not generate default program call
            elif code.startswith("ArcEnd"):
                # Provoke ARC END:
                self.addlastline('Arc End[11]')
                
                # Revert to desired speed
                self.SPEED = self.SPEED_BACKUP
                self.SPEED_BACKUP = None
                
                self.ARC_PARAMS = None
                return # Do not generate default program call
            
            # default program call
            code.replace(' ','_')
            self.addline('CALL %s ;' % (code))
        else:
            if not code.endswith(';'):
                code = code + ';'
            self.addline(code)
        
    def RunMessage(self, message, iscomment = False):
        """Add a joint movement"""
        if iscomment:
            #pass
            for i in range(0,len(message), 20):
                i2 = min(i + 20, len(message))
                self.addline('! %s ;' % message[i:i2])
                
        else:
            for i in range(0,len(message), 20):
                i2 = min(i + 20, len(message))
                self.addline('MESSAGE[%s] ;' % message[i:i2])
        
# ------------------ private ----------------------
    def page_size_control(self):
        if self.LINE_COUNT >= self.MAX_LINES_X_PROG:
            #self.nLines = 0
            self.ProgFinish(self.PROG_NAME, True)
            self.ProgStart(self.PROG_NAME, True)


    def addline(self, newline, movetype = ' '):
        """Add a program line"""
        if self.nProgs > 1 and not self.INCLUDE_SUB_PROGRAMS:
            return
        
        self.page_size_control()
        
        self.LINE_COUNT = self.LINE_COUNT + 1
        newline_ok = ('%4i:%s ' % (self.LINE_COUNT, movetype)) + newline            
        self.PROG.append(newline_ok)
            
    def addline_targets(self, newline):
        """Add a line at the end of the program (used for targets)"""
        self.PROG_TARGETS.append(newline)
        
    def addlog(self, newline):
        """Add a log message"""
        if self.nProgs > 1 and not self.INCLUDE_SUB_PROGRAMS:
            return
        self.LOG = self.LOG + newline + '\n'
        
# ------------------ targets ----------------------         
    def add_target_joints(self, pose, joints):
        if self.nProgs > 1 and not self.INCLUDE_SUB_PROGRAMS:
            return
        self.P_COUNT = self.P_COUNT + 1
        add_comma = ""
        if self.HAS_TRACK:
            add_comma = ","
        self.addline_targets('P[%i]{' % self.P_COUNT)
        self.addline_targets('   GP1:')
        self.addline_targets('    UF : %i, UT : %i,    ' % (self.ACTIVE_UF, self.ACTIVE_UT))
        self.addline_targets('\tJ1=    %.3f deg,\tJ2=    %.3f deg,\tJ3=    %.3f deg,' % (joints[0], joints[1], joints[2]))
        self.addline_targets('\tJ4=    %.3f deg,\tJ5=    %.3f deg,\tJ6=    %.3f deg%s' % (joints[3], joints[4], joints[5], add_comma))
        if self.HAS_TRACK:
            # adding external axes (linear track):
            track_str = ''
            for i in range(len(self.AXES_TRACK)):
                track_str = track_str + '\tE%i=%10.3f  mm,' % (i+1, joints[self.AXES_TRACK[i]])
            track_str = track_str[:-1]
            self.addline_targets(track_str)
        if self.HAS_TURNTABLE:
            # adding rotative axes (turntable):
            self.addline_targets('   GP2:')
            self.addline_targets('    UF : %i, UT : %i,' % (self.ACTIVE_UF, self.ACTIVE_UT))
            turntable_str = ''
            for i in range(len(self.AXES_TURNTABLE)):
                turntable_str = turntable_str + '\tJ%i=%10.3f deg,' % (i+1, joints[self.AXES_TURNTABLE[i]])
            turntable_str = turntable_str[:-1]
            self.addline_targets(turntable_str)
        self.addline_targets('};')
        return self.P_COUNT
    
    def add_target_cartesian(self, pose, joints, conf_RLF=None):
        if self.nProgs > 1 and not self.INCLUDE_SUB_PROGRAMS:
            return
        def angle_2_turn(angle):
            if angle >= 0.0:
                return +math.floor((+angle+180.0)/360.0)
            else:
                return -math.floor((-angle+180.0)/360.0)
        #return add_target_joints(pose, joints) # using joints as targets is safer to avoid problems setting up the reference frame and configurations
        xyzwpr = Pose_2_Fanuc(pose)
        config = ['N','U','T'] #normal        
        #config= ['F','D','B'] #alternative
        if conf_RLF is not None:
            if conf_RLF[2] > 0:
                config[0] = 'F' # F means axis 5 >= 0, N means axis 5 < 0
            if conf_RLF[1] > 0:
                config[1] = 'D'
            if conf_RLF[0] > 0:
                config[2] = 'B'
                
        turnJ1 = angle_2_turn(joints[0])
        turnJ4 = angle_2_turn(joints[3])
        turnJ6 = angle_2_turn(joints[5])       
            
        self.P_COUNT = self.P_COUNT + 1
        add_comma = ""
        if self.HAS_TRACK:
            add_comma = ","
        self.addline_targets('P[%i]{' % self.P_COUNT)
        self.addline_targets('   GP1:')
        self.addline_targets('    UF : %i, UT : %i,        CONFIG : \'%c %c %c, %i, %i, %i\',' % (self.ACTIVE_UF, self.ACTIVE_UT, config[0], config[1], config[2], turnJ1, turnJ4, turnJ6))        
        self.addline_targets('\tX =%10.3f  mm,\tY =%10.3f  mm,\tZ =%10.3f  mm,' % (xyzwpr[0], xyzwpr[1], xyzwpr[2]))
        self.addline_targets('\tW =%10.3f deg,\tP =%10.3f deg,\tR =%10.3f deg%s' % (xyzwpr[3], xyzwpr[4], xyzwpr[5], add_comma))
        if self.HAS_TRACK:
            # adding external axes (linear track):
            track_str = ''
            for i in range(len(self.AXES_TRACK)):
                track_str = track_str + '\tE%i=%10.3f  mm,' % (i+1, joints[self.AXES_TRACK[i]])
            track_str = track_str[:-1]
            self.addline_targets(track_str)
        if self.HAS_TURNTABLE:
            # adding rotative axes (turntable):
            self.addline_targets('   GP2:')
            self.addline_targets('    UF : %i, UT : %i,' % (self.ACTIVE_UF, self.ACTIVE_UT))
            turntable_str = ''
            for i in range(len(self.AXES_TURNTABLE)):
                turntable_str = turntable_str + '\tJ%i=%10.3f deg,' % (i+1, joints[self.AXES_TURNTABLE[i]])
            turntable_str = turntable_str[:-1]
            self.addline_targets(turntable_str)
        self.addline_targets('};')
        return self.P_COUNT
    
# syntax examples for joint-defined targets:
#P[1]{
#   GP1:
#    UF : 1, UT : 1,    
#    J1=    0.000 deg,    J2=    -40.966 deg,    J3=    43.328 deg,
#    J4=    0.000 deg,    J5=    -84.792 deg,    J6=    180.000 deg
#};
#P[258]{
#   GP1:
#    UF : 6, UT : 4,
#    J1=   -41.180 deg,    J2=   -26.810 deg,    J3=    11.060 deg,
#    J4=  -217.400 deg,    J5=    71.740 deg,    J6=    85.790 deg,
#    E1=     0.000 deg
#};

# syntax examples for cartesian targets:
#P[2]{
#   GP1:
#    UF : 1, UT : 1,        CONFIG : 'F U T, 0, 0, 0',
#    X =   564.871  mm,    Y =     0.000  mm,    Z =   571.832  mm,
#    W =  -180.000 deg,    P =     0.000 deg,    R =  -180.000 deg
#};   
#P[8:]{
#   GP1:
#     UF : 9, UT : 8,      CONFIG : 'N D B, 0, 0, 0',
#     X =     0.000  mm,   Y =   -10.000  mm,   Z =     0.000  mm,
#     W =     0.000 deg,   P =     0.000 deg,   R =     0.000 deg
#   GP2:
#     UF : 9, UT : 2,
#     J1=     0.000 deg,   J2=     0.000 deg
#}; 



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

    robot = RobotPost('Fanuc_custom', 'Fanuc robot', 6)

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
    
    robot.PROG = robot.PROG_LIST.pop()
    for line in robot.PROG:
        print(line)
    
    if len(robot.LOG) > 0:
        mbox('Program generation LOG:\n\n' + robot.LOG)

    input("Press Enter to close...")

if __name__ == "__main__":
    """Function to call when the module is executed by itself: test"""
    test_post()
