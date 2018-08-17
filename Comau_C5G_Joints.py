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
#     http://www.robodk.com/doc/en/PythonAPI/postprocessor.html
# ----------------------------------------------------


MACRO_MESSAGE_TP = '''-- Display message on the teach pendant:
  WIN_DEL ('menu:')
  -- popup window over window USR1
  WIN_POPUP('POP1:', 'USR1:')
  -- open a lun on window POP1
  OPEN FILE lun ('POP1:', 'rw')
  WRITE lun ('%s', NL)
  CLOSE FILE lun
  -- let user read the message
  DELAY 5000
  -- Remove and delete window POP1 from user screen
  WIN_REMOVE('POP1:')
  WIN_DEL('POP1:')
  ------------------'''

# ----------------------------------------------------
# Import RoboDK tools
from robodk import *

# gearbox ratio for external axes
RATIO_EXTAX = [1,1,1,1,1,1] #[10.6/360.0, 1, 1, 1, 1, 1]



# ----------------------------------------------------    
# Object class that handles the robot instructions/syntax
class RobotPost(object):
    """Robot post processor class"""
    PROG_EXT = 'pdl'        # set the program extension
    MAX_LINES_X_PROG = 5000  # maximum number of lines per program. It will then generate multiple "pages (files)"
    INCLUDE_SUB_PROGRAMS = True
    #INCLUDE_SUB_PROGRAMS = False
    
    # other variables
    ROBOT_POST = ''
    ROBOT_NAME = ''
    
    # Multiple program files variables
    PROG_NAME = 'unknown' # single program name
    PROG_NAMES = []
    PROG_FILES = []    
    PROG_LIST = []
    PROG_VARS = []
    SPEED_MMS = 1000
    ACCEL_MMSS = 100
    FLY_DIST = -1 # set to >0 to use MOVEFLY
    IMPORTS = []
    PROG = ''
    ROUTINES = ''
    nLines = 0
    nProgs = 0
    LOG = ''
    nAxes = 6
    TAB = ''
    LAST_POSE = None
    LAST_E_LENGTH = 0
    NEW_E_LENGTH = 0
    
    # ---------------------------------------------------
            
    def joints_2_str(self, joints):
        """Contverts a joint target to a string"""
        if joints is not None and len(joints) > 6:
            joints[6] = joints[6]*RATIO_EXTAX[0]
        return '{%s}' % (','.join(format(ji, ".5f") for ji in joints))
    
    def pose_2_str(self, pose,joints=None,conf_RLF=None):
        """Converts a pose target to a string"""
        [x,y,z,w,p,r] = Pose_2_Comau(pose)
        config = []
        # WARNING: Config only available for SMART type of contollers
        if conf_RLF is not None: 
            if conf_RLF[0] > 0:
                config.append('S')
            
            if conf_RLF[1] > 0:
                config.append('E')            
            
        if joints is not None:        
            #self.addline('cnfg_str := POS_GET_CNFG(%s)' % self.joints_2_str(joints))
            #config = 'cnfg_str'
            if len(joints) >= 5 and joints[4] < 0:
                config.append('W')
                
            if len(joints) >= 4:
                t1 = round((joints[3]+180) // 360)
                config.append('T1:%i' % t1)
                
            if len(joints) >= 6:
                t2 = round((joints[5]+180) // 360)
                config.append('T2:%i' % t2)
                t3 = round((joints[2]+180) // 360)
                config.append('T3:%i' % t3)
                
        pos_str = "POS(%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, '%s')" % (x,y,z,w,p,r, ' '.join(config))
        if joints is None or len(joints) <= 6:        
            return pos_str
            #return ('XTNDPOS(POS(%.4f,%.4f,%.4f,%.4f,%.4f,%.4f), , , (%.4f))' % (x,y,z,w,p,r,j7))
        else:
            self.addline('pxtn.POS := ' + pos_str)
            for i in range(6,len(joints)):
                self.addline('pxtn.AUX[%i] := %.5f' % (i-6+1, joints[i]*RATIO_EXTAX[i-6]))
                
            return 'pxtn'
            
    def __init__(self, robotpost=None, robotname=None, robot_axes = 6, **kwargs):
        self.ROBOT_POST = robotpost
        self.ROBOT_NAME = robotname
        self.PROG = ''
        self.LOG = ''
        self.nAxes = robot_axes
        for k,v in kwargs.items():
            if k == 'lines_x_prog':
                self.MAX_LINES_X_PROG = v      
        
    def ProgStart(self, progname, new_page = False):
        progname_i = progname
        if new_page:
            if self.INCLUDE_SUB_PROGRAMS:
                raise Exception("Multiple pages per program is not supported when adding routines in the main program")
            nPages = len(self.PROG_LIST)
            if nPages == 0:
                progname_i = progname
            else:
                progname_i = "%s%i" % (self.PROG_NAME, nPages)
            
        else:
            self.PROG_NAME = progname
            self.nProgs = self.nProgs + 1
            self.PROG_NAMES = []
            if self.nProgs > 1:
                if not self.INCLUDE_SUB_PROGRAMS:
                    return
                else:
                    if progname_i in self.IMPORTS:
                        self.IMPORTS.remove(progname_i)
                    self.addline('ROUTINE R_%s' % progname_i)
                    self.addline('VAR')
                    self.addline('BEGIN')
                    self.TAB = '  '
                    return

        self.PROG_NAMES.append(progname_i)
        self.addline('PROGRAM %s' % progname_i)
        self.addline('-- IMPORTS --')
        #self.addline('CONST')
        self.addline('VAR')
        self.addline('ROUTINE R_%s EXPORTED FROM %s GLOBAL' % (progname_i, progname_i))
        self.addline('')
        self.addline('-- ROUTINES --')
        
        #self.addline('BEGIN R_%s' % progname_i)
        self.addline('ROUTINE R_%s' % progname_i)
        self.addline('VAR')
        #self.addline('  cnfg_str: STRING') # PROG_VARS
        if self.nAxes > 6:
            self.addline('  pxtn: XTNDPOS')
            
        self.addline('-- VARIABLES --') # PROG_VARS
        self.addline('BEGIN')
        self.TAB = '  '
        self.addline('$ORNT_TYPE := RS_WORLD')
        self.addline('$MOVE_TYPE := JOINT')
        self.addline('$JNT_MTURN := TRUE')
        self.addline('$CNFG_CARE := TRUE')
        self.addline('$TURN_CARE := TRUE')
        self.addline('$SING_CARE := FALSE')
        self.addline('$TERM_TYPE := NOSETTLE')
        
        # Use MOVEFLY (rounding)
        self.addline('$FLY_TYPE := FLY_CART')
        self.addline('$FLY_TRAJ := FLY_PASS')
        self.setZoneData(self.FLY_DIST)
        #self.addline('$FLY_DIST := 1')
        self.addline('$STRESS_PER:= 65')
        
    def ProgFinish(self, progname, new_page = False):
        variables = ''
        for line in self.PROG_VARS:
            variables = '  %s\n' % line
            
        self.PROG = self.PROG.replace('-- VARIABLES --\n', variables,1)
        self.PROG_VARS = []
        
        if self.nProgs > 1 and not self.INCLUDE_SUB_PROGRAMS:
            return
        self.TAB = ''
        if self.nProgs <= 1:
            self.PROG = self.PROG + "END R_%s\n\n" % progname
            # Create a the main program which call the main routine
            self.PROG = self.PROG + "BEGIN\n  R_%s\nEND %s\n\n" % (progname, progname)
        else:
            self.ROUTINES = self.ROUTINES + "END R_%s\n\n" % progname
            
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
        # save imports
        imports = ''
        for i in range(len(self.IMPORTS)):
            imports = imports + "IMPORT '%s'\n" % self.IMPORTS[i]
        self.PROG = self.PROG.replace("-- IMPORTS --\n", imports, 1)
        # save routines
        self.PROG = self.PROG.replace("-- ROUTINES --\n", self.ROUTINES, 1)
        
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
            mainprog = "PROGRAM %s\n" % progname_main
            for i in range(npages):
                mainprog += "IMPORT '%s'\n" % self.PROG_NAMES[i]            
                
            mainprog += "CONST\nVAR\n"
            mainprog += "BEGIN\n"
            for i in range(npages):
                mainprog += "  R_%s()\n" % self.PROG_NAMES[i]
                
            mainprog += "END %s\n" % progname_main
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
        #self.addline('MOVE JOINT TO ' + joints_2_str(joints))# MOVE JOINT TO: won't use absolute axis position
        self.addline('MOVE TO ' + self.joints_2_str(joints)) # MOVE TO: absolute axis position
        
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
            distance_mm = norm(subs3(pose1.Pos(), pose2.Pos()))
            # calculate movement time in seconds
            time_s = Calculate_Time(distance_mm, self.SPEED_MMS, self.ACCEL_MMSS)
            # add material
            self.addline("$AOUT[5] := %.3f" % (add_material/time_s))
        else:
            # DO not add material
            self.addline("$AOUT[5] := 0")
            
    def new_movec(self, pose1, pose2, pose3):
        return
        
    def MoveL(self, pose, joints, conf_RLF=None):
        """Add a linear movement"""
        pose = None
        # Filter sending the same movement twice
        if self.LAST_POSE is not None and pose is not None:
            # Skip adding a new movement if the new position is the same as the last one
            if distance(pose.Pos(), self.LAST_POSE.Pos()) < 0.001 and pose_angle_between(pose, self.LAST_POSE) < 0.01:
                return
        
        target = ''
        if pose is None:
            target = self.joints_2_str(joints)
        else:
            target = self.pose_2_str(pose,joints)
            
        #self.new_move(self.LAST_POSE, pose) #used for 3D printing
        if self.FLY_DIST > 0:
            self.addline('MOVEFLY LINEAR TO %s ADVANCE' % target)
        else:
            self.addline('MOVE LINEAR TO ' + target)
            
        self.LAST_POSE = pose
        
    def MoveC(self, pose1, joints1, pose2, joints2, conf_RLF_1=None, conf_RLF_2=None):
        """Add a circular movement"""
        #self.new_movec(self.LAST_POSE, pose1, pose2) #used for 3D printing
        if self.FLY_DIST > 0:
            self.addline('MOVEFLY CIRCULAR TO %s VIA %s ADVANCE' % (self.pose_2_str(pose2,joints2), self.pose_2_str(pose1,joints1)))
        else:
            self.addline('MOVE CIRCULAR TO %s VIA %s' % (self.pose_2_str(pose2,joints2), self.pose_2_str(pose1,joints1)))
            
        self.LAST_POSE = pose2
        
    def setFrame(self, pose, frame_id=None, frame_name=None):
        """Change the robot reference frame"""
        self.addline('$UFRAME := ' + self.pose_2_str(pose))
        
    def setTool(self, pose, tool_id=None, tool_name=None):
        """Change the robot TCP"""
        self.addline('$TOOL := ' + self.pose_2_str(pose))
        
    def Pause(self, time_ms):
        """Pause the robot program"""
        if time_ms <= 0:
            self.addline('PAUSE')
        else:
            self.addline('DELAY %.0f' % (time_ms))
    
    def setSpeed(self, speed_mms):
        """Changes the robot speed (in mm/s)"""
        self.SPEED_MMS = speed_mms        
        self.addline('$SPD_OPT := SPD_LIN')
        self.addline('$LIN_SPD := %.3f' % (speed_mms*0.001))
    
    def setAcceleration(self, accel_mmss):
        """Changes the robot acceleration (in mm/s2)"""
        self.ACCEL_MMSS = accel_mmss
        self.addlog('setAcceleration not defined')
    
    def setSpeedJoints(self, speed_degs):
        """Changes the robot joint speed (in deg/s)"""
        self.addline('$ROT_SPD := %.3f' % (speed_degs*pi/180.0))
    
    def setAccelerationJoints(self, accel_degss):
        """Changes the robot joint acceleration (in deg/s2)"""
        self.addlog('setAccelerationJoints not defined')
        
    def setZoneData(self, zone_mm):
        """Changes the zone data approach (makes the movement more smooth)"""
        #self.addlog('setZoneData not defined (%.1f mm)' % zone_mm)
        self.FLY_DIST = zone_mm
        self.addline('$FLY_DIST := %.3f' % zone_mm)
        
    def setDO(self, io_var, io_value):
        """Sets a variable (output) to a given value"""
        if type(io_var) != str:  # set default variable name if io_var is a number
            io_var = '$DOUT[%s]' % str(io_var)        
        if type(io_value) != str: # set default variable value if io_value is a number            
            if io_value > 0:
                io_value = 'ON'
            else:
                io_value = 'OFF'
        
        # at this point, io_var and io_value must be string values
        self.addline('%s := %s' % (io_var, io_value))
        
    def waitDI(self, io_var, io_value, timeout_ms=-1):
        """Waits for an input io_var to attain a given value io_value. Optionally, a timeout can be provided."""
        if type(io_var) != str:  # set default variable name if io_var is a number
            io_var = '$DIN[%s]' % str(io_var)        
        if type(io_value) != str: # set default variable value if io_value is a number            
            if io_value > 0:
                io_value = 'ON'
            else:
                io_value = 'OFF'
        
        # at this point, io_var and io_value must be string values
        if timeout_ms < 0:
            #self.addline('WAIT FOR %s==%s' % (io_var, io_value))
            self.addline('$TIMER[1] := 0')
            self.addline('REPEAT')
            self.addline('  DELAY 1')
            self.addline('UNTIL (%s = %s)' % (io_var, io_value))           
        else:
            self.addline('$TIMER[1] := 0')
            self.addline('REPEAT')
            self.addline('  DELAY 1')
            self.addline('UNTIL (%s = %s) OR ($TIMER[1] > %.1f)' % (io_var, io_value, timeout_ms))
            self.addline('IF $TIMER[1] > %.1f THEN' % timeout_ms)
            self.addline('-- TIMED OUT! Important: This section must be updated accordingly')
            self.addline('  DELAY 2000')
            #self.addline('  PAUSE')#Important, this must be updated to perform a specific action
            self.addline('ENDIF')            
        
    def RunCode(self, code, is_function_call = False):
        """Adds code or a function call"""
        if self.nProgs > 1 and not self.INCLUDE_SUB_PROGRAMS:
            return
        
        if is_function_call:
            if code.startswith("Extruder("):
                # if the program call is Extruder(123.56), we extract the number as a string and convert it to a number
                self.NEW_E_LENGTH = float(code[9:-1]) # it needs to retrieve the extruder length from the program call
                # Do not generate program call
                return
                
            code.replace(' ','_')
            bracket_id = code.find('(')
            import_prog = None
            if bracket_id < 0:
                # no brackets
                import_prog = code                
                #code = code + '()'
            else:
                # brackets
                import_prog = code[:bracket_id]

            # Add import directive only if we have not added it before
            if not import_prog in self.IMPORTS:
                self.IMPORTS.append(import_prog)
                
            self.addline('R_' + code)
        else:
            self.addline(code)
        
    def RunMessage(self, message, iscomment = False):
        """Add a joint movement"""
        if iscomment:
            self.addline('-- ' + message)
        else:
            #self.addline('TYPE "' + message + '"')
            #-------- Option 1: Show message on the teach pendant: Important! Fails if there is no teach pendant
            #self.PROG_VARS.append('lun: INTEGER')
            #self.addline(MACRO_MESSAGE_TP % message)
            #-------- Option 2: Just comment the message
            self.addline('-- ' + message)

        
# ------------------ private ----------------------                
    def addline(self, newline):
        """Add a program line"""
        if self.nProgs > 1 and not self.INCLUDE_SUB_PROGRAMS:
            return
        
        if not self.INCLUDE_SUB_PROGRAMS and self.nLines > self.MAX_LINES_X_PROG:
            self.nLines = 0
            self.ProgFinish(self.PROG_NAME, True)
            self.ProgStart(self.PROG_NAME, True)

        if self.nProgs > 1:
            self.ROUTINES = self.ROUTINES + self.TAB + newline + '\n'
        else:
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
