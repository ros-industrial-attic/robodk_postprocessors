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
# for a Motoman robot (Inform III programming language)
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


def get_safe_name(progname, max_chars = 6):
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
    PROG_EXT = 'JBI'             # set the program extension
    MAX_LINES_X_PROG = 2000      # maximum number of lines per program. It will then generate multiple "pages (files)". This can be overriden by RoboDK settings.    
    DONT_USE_MFRAME = True       # Set to false to use MFRAME for setting reference frames automatically within the program
    DONT_USE_SETTOOL = True      # Set to false to use SETTOOL for setting the tool within the program
    USE_RELATIVE_JOB = True      # Set to False to always use pulses (Otherwise, it might require a special/paid option
    
    INCLUDE_SUB_PROGRAMS = True # Generate sub programs
    STR_V = 'V=100.0'         # set default cartesian speed
    STR_VJ = 'VJ=50.00'       # set default joints speed
    STR_PL = ''             # Rounding value (from 0 to 4) (in RoboDK, set to 100 mm rounding for PL=4
    ACTIVE_FRAME = 9        # Active UFrame Id (register)
    ACTIVE_TOOL = 9         # Active UTool Id (register)
    SPARE_PR = 95           # Spare Position register for calculations
    
    REGISTER_DIGITS = 5

    # Pulses per degree (provide these in the robot parameters menu: Double click the motoman robot in RoboDK, select "Parameters"
    PULSES_X_DEG = [1,1,1,1,1,1] 

    # PROG specific variables:
    LINE_COUNT = 0      # Count the number of instructions (limited by MAX_LINES_X_PROG)
    P_COUNT = 0         # Count the number of P targets in one file
    C_COUNT = 0         # Count the number of P targets in one file
    nProgs = 0          # Count the number of programs and sub programs
    LBL_ID_COUNT = 0    # Number of labels used
    
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
    POSE_FRAME = eye(4)
    POSE_FRAME = eye(4)
    LAST_CONFDATA = [None, None, None, None] # [pulses(None, Pulses(0), Cartesian) ,  base(or None), tool, config]
    
    def __init__(self, robotpost=None, robotname=None, robot_axes = 6, **kwargs):
        if self.DONT_USE_MFRAME:
            self.ACTIVE_FRAME = None
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
            #self.PROG_NAMES = []
            
        self.PROG_NAME_CURRENT = progname_i
        self.PROG_NAMES.append(progname_i)
        
    def ProgFinish(self, progname, new_page = False):
        progname = get_safe_name(progname)
        if not new_page:
            # Reset page count
            self.nPages = 0
            
        #if self.nPROGS > 1:
        #    # Motoman does not support defining multiple programs in the same file, so one program per file
        #    return
        header = ''
        header += '/JOB' + '\n'
        header += '//NAME %s' % progname + '\n'
        header += '//POS' + '\n'
        header += '///NPOS %i,0,0,%i,0,0' % (self.C_COUNT, self.P_COUNT)
        
        # Targets are added at this point       
        
        import time        
        datestr = time.strftime("%Y/%m/%d %H:%M")
        
        header_ins = ''
        header_ins += '//INST' + '\n'
        header_ins += '///DATE %s' % datestr + '\n'
        #///DATE 2012/04/25 14:11
        header_ins += '///COMM Generated using RoboDK\n' # comment: max 28 chars
        if self.USE_RELATIVE_JOB:
            header_ins += '///ATTR SC,RW,RJ' + '\n'
            if self.ACTIVE_FRAME is not None:
                header_ins += '///FRAME USER %i' % self.ACTIVE_FRAME + '\n'           
        else:
            header_ins += '///ATTR SC,RW' + '\n'

        header_ins += '///GROUP1 RB1' + '\n'
        header_ins += 'NOP'
        #if self.HAS_TURNTABLE:
        #    header = header + '/APPL' + '\n'
        
        self.PROG.insert(0, header_ins)
        self.PROG.append('END')
        
        self.PROG_TARGETS.insert(0, header)
        
        self.PROG = self.PROG_TARGETS + self.PROG
        
        
        
        # Save PROG in PROG_LIST
        self.PROG_LIST.append(self.PROG)
        self.PROG = []
        self.PROG_TARGETS = []
        self.LINE_COUNT = 0
        self.P_COUNT = 0
        self.C_COUNT = 0
        self.LAST_CONFDATA = [None, None, None, None]
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
        import io
        fid = open(filesave, "w", newline='\r\n')
        #fid.write(self.PROG)
        for line in self.PROG:
            fid.write(line.decode('unicode-escape'))
            fid.write(u'\n')
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
        target_id = self.add_target_joints(joints)
        self.addline("MOVJ C%05d %s%s" % (target_id, self.STR_VJ, self.STR_PL))                    
        self.LAST_POSE = pose
        
    def MoveL(self, pose, joints, conf_RLF=None):
        """Add a linear movement"""        
        #if self.LAST_POSE is not None and pose is not None:
        #    # Skip adding a new movement if the new position is the same as the last one
        #    if distance(pose.Pos(), self.LAST_POSE.Pos()) < 0.1 and pose_angle_between(pose, self.LAST_POSE) < 0.1:
        #        return

        self.page_size_control() # Important to control the maximum lines per program and not save last target on new program
                
        if pose is None:
            target_id = self.add_target_joints(joints)
        else:
            target_id = self.add_target_cartesian(self.POSE_FRAME*pose, joints, conf_RLF)

        self.addline("MOVL C%05d %s%s" % (target_id, self.STR_V, self.STR_PL))        
        self.LAST_POSE = pose
        
    def MoveC(self, pose1, joints1, pose2, joints2, conf_RLF_1=None, conf_RLF_2=None):
        """Add a circular movement"""
        self.page_size_control() # Important to control the maximum lines per program and not save last target on new program
        
        if self.LAST_POSE is not None:
            target_id0 = self.add_target_cartesian(self.POSE_FRAME*self.LAST_POSE, joints1, conf_RLF_1)
            self.addline("MOVC C%05d %s%s" % (target_id0, self.STR_V, self.STR_PL))
        
        target_id1 = self.add_target_cartesian(self.POSE_FRAME*pose1, joints1, conf_RLF_1)
        target_id2 = self.add_target_cartesian(self.POSE_FRAME*pose2, joints2, conf_RLF_2)
            
        self.addline("MOVC C%05d %s%s" % (target_id1, self.STR_V, self.STR_PL))
        self.addline("MOVC C%05d %s%s" % (target_id2, self.STR_V, self.STR_PL))
        
        self.LAST_POSE = None
        
    def setFrame(self, pose, frame_id, frame_name):
        """Change the robot reference frame"""
        xyzwpr = Pose_2_Motoman(pose)
        if self.DONT_USE_MFRAME:
            self.ACTIVE_FRAME = None
            self.POSE_FRAME = pose
            self.RunMessage('Using %s (targets wrt base):' % (str(frame_name)), True)
            self.RunMessage('%.1f,%.1f,%.1f,%.1f,%.1f,%.1f' % (xyzwpr[0], xyzwpr[1], xyzwpr[2], xyzwpr[3], xyzwpr[4], xyzwpr[5]), True)
        else:
            self.POSE_FRAME = eye(4)
            if frame_id is None or frame_id < 0:
                self.RunMessage('Setting Frame %i (%s):' % (self.ACTIVE_FRAME, str(frame_name)), True)            
                decimals = [1000,1000,1000,100,100,100]
                frame_calc = [eye(4), transl(200,0,0), transl(0,200,0)]
                for m in range(3):
                    xyzwpr_pm = Pose_2_Motoman(pose*frame_calc[m])
                    for i in range(6):
                        self.addline("SETE P%05d (%i) %i" % (self.SPARE_PR+m, i+1, round(xyzwpr_pm[i]*decimals[i])))
                    for i in range(6,self.nAxes):
                        self.addline("SETE P%05d (%i) %i" % (self.SPARE_PR+m, i+1, 0))
                    
                self.addline("MFRAME UF#(%i) P%05d P%05d P%05d" % (self.ACTIVE_FRAME, self.SPARE_PR, self.SPARE_PR+1, self.SPARE_PR+2))
                    
            else:
                self.ACTIVE_FRAME = frame_id
                self.RunMessage('Frame %i (%s) should be close to:' % (self.ACTIVE_FRAME, str(frame_name)), True)
                self.RunMessage('%.1f,%.1f,%.1f,%.1f,%.1f,%.1f' % (xyzwpr[0], xyzwpr[1], xyzwpr[2], xyzwpr[3], xyzwpr[4], xyzwpr[5]), True)
        
    def setTool(self, pose, tool_id, tool_name):
        """Change the robot TCP"""
        xyzwpr = Pose_2_Motoman(pose)
        if tool_id is None or tool_id < 0:
            if self.DONT_USE_SETTOOL:
                self.RunMessage('Tool %s should be close to:' % (str(tool_name)), True)
                self.RunMessage('%.1f,%.1f,%.1f,%.1f,%.1f,%.1f' % (xyzwpr[0], xyzwpr[1], xyzwpr[2], xyzwpr[3], xyzwpr[4], xyzwpr[5]), True)
            else:
                self.RunMessage('Setting Tool %i (%s):' % (self.ACTIVE_TOOL, str(tool_name)), True)            
                decimals = [1000,1000,1000,100,100,100]
                for i in range(6):
                    self.addline("SETE P%05d (%i) %i" % (self.SPARE_PR, i+1, round(xyzwpr[i]*decimals[i])))
                for i in range(6,self.nAxes):
                    self.addline("SETE P%05d (%i) %i" % (self.SPARE_PR, i+1, 0))
                    
                self.addline("SETTOOL TL#(%i) P%05d" % (self.ACTIVE_TOOL, self.SPARE_PR))
                
        else:
            self.ACTIVE_TOOL = tool_id
            self.RunMessage('Tool %i (%s) should be close to:' % (self.ACTIVE_TOOL, str(tool_name)), True)
            self.RunMessage('%.1f,%.1f,%.1f,%.1f,%.1f,%.1f' % (xyzwpr[0], xyzwpr[1], xyzwpr[2], xyzwpr[3], xyzwpr[4], xyzwpr[5]), True)
        
    def Pause(self, time_ms):
        """Pause the robot program"""
        if time_ms <= 0:
            self.addline('PAUSE')
        else:
            self.addline('TIMER T=%.2f' % (time_ms*0.001))
        
    def setSpeed(self, speed_mms):
        """Changes the robot speed (in mm/s)"""
        speed_cm_min = speed_mms * 60.0 / 10.0
        speedl = max(0.01,min(speed_cm_min,200.0)) # Important! Filter linear speed is in mm/s or cm/min (otherwise the program stops)
        if speedl < 100:
            self.STR_V = "V=%.2f" % speedl
        else:
            self.STR_V = "V=%.1f" % speedl
    
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
            self.STR_PL = ' PL=%i' % round(min(zone_mm, 8))        
        
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
        if timeout_ms <= 0:
            #WAIT IN#(12)=ON
            self.addline('WAIT %s=%s' % (io_var, io_value))
        else:
            #self.LBL_ID_COUNT = self.LBL_ID_COUNT + 1
            self.addline('WAIT %s=%s T=%.2f' % (io_var, io_value, timeout_ms*0.001))
       
            
    def RunCode(self, code, is_function_call = False):
        """Adds code or a function call"""
        if is_function_call:
            code = get_safe_name(code,8)
            #if code.startswith("ArcStart"):
                #return
                
            # default program call
            code.replace(' ','_')
            self.addline('CALL JOB:%s' % (code))
        else:
            #if code.endswith(';'):
                #code = code[:-1]
            self.addline(code)
        
    def RunMessage(self, message, iscomment = False):
        """Add a message/comment"""
        if iscomment:
            for i in range(0,len(message), 29):
                i2 = min(i + 29, len(message))
                self.addline("'%s" % message[i:i2])
                
        else:
            for i in range(0,len(message), 25):
                i2 = min(i + 25, len(message))
                self.addline('MSG "%s"' % message[i:i2])
        
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

    def setCartesian(self, confdata):
        #self.LAST_CONFDATA = [none/pulses(0)/postype(1), base, tool, config]
        if self.ACTIVE_FRAME is not None and self.ACTIVE_FRAME != self.LAST_CONFDATA[1]:
            self.addline_targets("///USER %i" % self.ACTIVE_FRAME)
            self.LAST_CONFDATA[1] = self.ACTIVE_FRAME        

        if self.ACTIVE_TOOL != self.LAST_CONFDATA[2]:
            self.addline_targets("///TOOL %i" % self.ACTIVE_TOOL)
            self.LAST_CONFDATA[2] = self.ACTIVE_TOOL

        if self.LAST_CONFDATA[0] != 2:
            if self.ACTIVE_FRAME is not None:
                self.addline_targets("///POSTYPE USER")
            else:
                self.addline_targets("///POSTYPE BASE")

            self.addline_targets("///RECTAN")
            self.addline_targets("///RCONF %s" % confdata)
            self.LAST_CONFDATA[3] = confdata
            
            
        elif self.LAST_CONFDATA[3] != confdata:
            self.addline_targets("///RCONF %s" % confdata)
            self.LAST_CONFDATA[3] = confdata

        self.LAST_CONFDATA[0] = 2

    def setPulses(self):
        #self.LAST_CONFDATA = [none/pulses(0)/postype(1), base, tool, config]
        if self.LAST_CONFDATA[0] is None:
            self.addline_targets("///TOOL %i" % self.ACTIVE_TOOL)
            self.LAST_CONFDATA[2] = self.ACTIVE_TOOL
       
        if self.LAST_CONFDATA[0] != 1:
            self.addline_targets("///POSTYPE PULSE")
            self.addline_targets("///PULSE")
            self.LAST_CONFDATA[0] = 1
            
        self.LAST_CONFDATA[0] = 1
        self.LAST_CONFDATA[1] = None
        self.LAST_CONFDATA[2] = None
        self.LAST_CONFDATA[3] = None
        
    def add_target_joints(self, joints):    
        if self.nProgs > 1 and not self.INCLUDE_SUB_PROGRAMS:
            return

        self.setPulses()            
        cid = self.C_COUNT
        self.C_COUNT = self.C_COUNT + 1        
                
        str_pulses=[]        
        for i in range(len(joints)):
            str_pulses.append('%i' % round(joints[i] * self.PULSES_X_DEG[i]))
        
        self.addline_targets('C%05i=' % cid + ','.join(str_pulses))         
        return cid
    
    def add_target_cartesian(self, pose, joints, conf_RLF):           
        if self.nProgs > 1 and not self.INCLUDE_SUB_PROGRAMS:
            return
            
        if not self.USE_RELATIVE_JOB:
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
        self.setCartesian(confdata)            
        cid = self.C_COUNT
        self.C_COUNT = self.C_COUNT + 1        
        self.addline_targets('C%05i=' % cid + '%.3f,%.3f,%.3f,%.2f,%.2f,%.2f' % tuple(xyzwpr))
        return cid
    
#/JOB
#//NAME TESTTCPX
#//POS
#///NPOS 3,0,0,19,0,0
#///TOOL 23
#///POSTYPE PULSE
#///PULSE
#C00000=2730,-84461,-101368,0,-77310,67137
#///TOOL 13
#C00001=76697,-73189,-80544,374,-78336,86207
#C00002=81732,-66267,-100360,-2451,-62876,82497
#///USER 8
#///TOOL 23
#///POSTYPE USER
#///RECTAN
#///RCONF 0,0,0,0,1,0,0,0
#P0010=0.000,0.000,10.205,0.00,0.00,0.00
#P0011=0.001,0.006,-9.794,-170.32,90.00,-170.32
#///RCONF 0,0,0,0,0,0,0,0
#P0012=13.500,0.000,0.000,0.00,0.00,0.00
#///RCONF 0,0,0,0,1,0,0,0
#P0015=0.000,0.000,0.000,180.00,90.00,180.00
#///RCONF 0,0,0,0,0,0,0,0
#P0020=0.000,0.000,0.000,-90.00,0.00,0.00
#P0021=0.000,0.000,-5.000,0.00,0.00,0.00
#///RCONF 1,0,0,0,0,0,0,0
#P0022=0.004,0.003,8.297,-90.00,0.00,-90.00
#P0023=-0.003,-0.003,-9.351,-90.00,0.00,-90.00
#///POSTYPE BASE
#///RCONF 0,0,0,0,0,0,0,0
#P0024=0.000,0.000,8.824,0.00,0.00,0.00
#///TOOL 0
#P0026=-276.205,101.089,162.089,0.08,-83.39,-19.88
#P0027=-276.205,101.634,162.089,179.99,-6.61,160.20
#///USER 8
#///TOOL 23
#///POSTYPE USER
#///RCONF 0,0,0,0,1,0,0,0
#P0028=0.410,-0.016,0.147,0.00,-90.00,0.00
#///RCONF 1,0,0,0,0,0,0,0
#P0030=-0.004,0.001,-0.527,-90.00,0.00,-90.00
#P0100=0.000,0.000,50.000,0.00,90.00,0.00
#P0101=0.000,0.000,-25.000,0.00,90.00,0.00
#P0103=0.000,0.000,25.000,-90.00,0.00,-90.00
#P0104=0.000,0.000,-25.000,-90.00,0.00,-90.00
#P0110=-100.000,0.000,0.000,0.00,0.00,0.00
#P0111=0.000,0.000,200.000,0.00,0.00,0.00
#//INST
#///DATE 2012/04/25 14:11
#///COMM 1ER PROGRAM POUR VERIFIER LE TCP
#///ATTR SC,RW
#///GROUP1 RB1
#NOP
#DOUT OT#(5) ON
#DOUT OT#(2) ON
 



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

    robot = RobotPost('Motomantest', 'Motoman robot', 6)

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
    for line in robot.PROG:
        print(line)
    
    if len(robot.LOG) > 0:
        mbox('Program generation LOG:\n\n' + robot.LOG)

    input("Press Enter to close...")

if __name__ == "__main__":
    """Function to call when the module is executed by itself: test"""
    test_post()
