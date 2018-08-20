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
# for a Kuka robot with RoboDK
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


HEADER = '''
;FOLD INI
BAS (#INITMOV,0 )
;ENDFOLD (INI)

;FOLD STARTPOS
$BWDSTART = FALSE
PDAT_ACT = PDEFAULT
BAS(#PTP_DAT)
FDAT_ACT = {TOOL_NO 0,BASE_NO 0,IPO_FRAME #BASE}
BAS(#FRAMES)
;ENDFOLD

;FOLD SET DEFAULT SPEED
$VEL.CP=0.2
BAS(#VEL_PTP,100)
BAS(#TOOL,0)
BAS(#BASE,0)
;ENDFOLD

$ADVANCE = 5

PTP $AXIS_ACT ; skip BCO quickly
'''

# ----------------------------------------------------
def pose_2_str(pose):
    """Converts a pose target to a string"""
    #[x,y,z,w,p,r] = Pose_2_KUKA(pose)
    #return ('X %.3f, Y %.3f, Z %.3f, A %.3f, B %.3f, C %.3f' % (x,y,z,r,p,w)) # old version had to be switched
    [x,y,z,r,p,w] = pose_2_xyzrpw(pose)
    return ('X %.3f,Y %.3f,Z %.3f,A %.3f,B %.3f,C %.3f' % (x,y,z,w,p,r))

def pose_2_str_ext(pose,joints):
    njoints = len(joints)
    if njoints <= 6:
        return pose_2_str(pose)
    else:     
        extaxes = ''
        for i in range(njoints-6):
            extaxes = extaxes + (',E%i %.5f' % (i+1, joints[i+6]))
        return pose_2_str(pose) + extaxes
    
def angles_2_str(angles):
    """Prints a joint target"""
    str = ''
    data = ['A1','A2','A3','A4','A5','A6','E1','E2','E3','E4','E5','E6']
    for i in range(len(angles)):
        str = str + ('%s %.5f,' % (data[i], angles[i]))
    str = str[:-1]
    return str
    
def conf_2_str(confRLF):
    if confRLF is None:
        return "'B010'"
    strconf = ""
    if confRLF[2] > 0:
        strconf = strconf + '1'
    else:
        strconf = strconf + '0'
        
    if confRLF[1] == 0:
        strconf = strconf + '1'
    else:
        strconf = strconf + '0'
        
    if confRLF[0] > 0:
        strconf = strconf + '1'
    else:
        strconf = strconf + '0'
    
    return "'B%s'" % strconf    
    
def joints_2_turn_str(joints):
    if joints is None:
        return "'B000000'"
        
    strturn = ""
    for i in range(len(joints)):
        if joints[i] < 0:
            strturn = '1' + strturn
        else:
            strturn = '0' + strturn
    return "'B%s'" % strturn  


# ----------------------------------------------------    
# Object class that handles the robot instructions/syntax
class RobotPost(object):
    """Robot post object"""
    PROG_EXT = 'src'        # set the program extension
    
    # other variables
    ROBOT_POST = ''
    ROBOT_NAME = ''
    PROG_FILES = []
    PROG_NAME = 'unknown'
    PROG_COUNT = 0
    
    BASE_ID = 1
    TOOL_ID = 1
    speed_ms = 1
    speed_degs = 180
    accel_mss = 1
    accel_degss = 200
    VEL_PTP = 100 # speed in percentage
        
    PROG = ''
    PROG_DAT = ''
    nPosDat = 0
    LOG = ''
    nAxes = 6
    APO_VALUE = -1 # set to one for smooth path
    C_DIS = ' C_DIS'
    C_PTP = ' C_PTP'
    ARC_ON = False
    ARC_REQUIRED = False
    nArcId = 0
    ARC_Pgno = 108
    
    PROG_CALLS = []
    
    def __init__(self, robotpost=None, robotname=None, robot_axes = 6, **kwargs):
        self.ROBOT_POST = robotpost
        self.ROBOT_NAME = robotname
        self.PROG = ''
        self.LOG = ''
        self.nAxes = robot_axes
        
    def ProgStart(self, progname):
        self.PROG_COUNT = self.PROG_COUNT + 1
        self.addline('DEF %s ( )' % progname)
        if self.PROG_COUNT == 1:
            self.PROG = self.PROG + HEADER
            self.PROG_NAME = progname
        #if self.nAxes > 6:
            #self.addline('$ACT_EX_AX = %i' % (self.nAxes-6))    
        
    def ProgFinish(self, progname):
        self.addline('END')
        
    def ProgSave(self, folder, progname, ask_user = False, show_result = False):
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
        #fid.write("&ACCESS RVP\n")
        #fid.write("&REL 1\n")
        #fid.write("&COMMENT Generated by RoboDK\n")
        fid.write("&ACCESS RVP\n")
        fid.write("&REL 1\n") #fid.write("&REL 29\n")
        fid.write("&PARAM TEMPLATE = C:\\KRC\\Roboter\\Template\\vorgabe\n")
        fid.write("&PARAM EDITMASK = *\n")
        fid.write(self.PROG)
        fid.close()
        print('SAVED: %s\n' % filesave) # tell RoboDK the path of the saved file
        filesave_dat = filesave[:-3] + 'dat'
        fid2 = open(filesave_dat, "w")
        fid2.write('&ACCESS RVP\n')
        fid2.write('&REL 1\n')
        fid2.write("&COMMENT Generated by RoboDK\n")
        fid2.write('&PARAM TEMPLATE = C:\\KRC\\Roboter\\Template\\vorgabe\n')
        fid2.write('&PARAM EDITMASK = *\n')
        fid2.write('DEFDAT  %s\n\n' % self.PROG_NAME)
        fid2.write('EXT BAS (BAS_COMMAND :IN,REAL :IN )\n')

        for prog_nm in self.PROG_CALLS:
            fid2.write("EXT %s()\n" % prog_nm)
        fid2.write('\n')
        fid2.write(self.PROG_DAT)
        fid2.write('\nENDDAT\n\n')
        fid2.close()
        print('SAVED: %s\n' % filesave_dat) # tell RoboDK the path of the saved file
        self.PROG_FILES = [filesave, filesave_dat]
        # open file with default application
        if show_result:
            if type(show_result) is str:
                # Open file with provided application
                import subprocess
                p = subprocess.Popen([show_result, filesave_dat, filesave])
            elif type(show_result) is list:
                import subprocess
                p = subprocess.Popen(show_result + [filesave])   
            else:
                # open file with default application
                import os
                os.startfile(filesave)
                os.startfile(filesave_dat)
            if len(self.LOG) > 0:
                mbox('Program generation LOG:\n\n' + self.LOG)
        
    def ProgSendRobot(self, robot_ip, remote_path, ftp_user, ftp_pass):
        """Send a program to the robot using the provided parameters. This method is executed right after ProgSave if we selected the option "Send Program to Robot".
        The connection parameters must be provided in the robot connection menu of RoboDK"""
        UploadFTP(self.PROG_FILES, robot_ip, remote_path, ftp_user, ftp_pass)
        
    def MoveJ(self, pose, joints, conf_RLF=None):
        """Add a joint movement"""
        self.nPosDat = self.nPosDat + 1
        vname = '%i' % self.nPosDat
        
        # Write SRC information ---------------------------
        #;FOLD PTP P1 Vel=70 % PDAT1 Tool[1]:graco_negra Base[0];%{PE}%R 8.3.42,%MKUKATPBASIS,%CMOVE,%VPTP,%P 1:PTP, 2:P1, 3:, 5:70, 7:PDAT1
        #;FOLD PTP P1 Vel=70 % PDAT1 Tool[1]:graco_negra Base[0];%{PE}%R 8.3.32,%MKUKATPBASIS,%CMOVE,%VPTP,%P 1:PTP, 2:P1, 3:, 5:70, 7:PDAT1        
        #$BWDSTART=FALSE
        #PDAT_ACT=PPDAT1
        #FDAT_ACT=FP1
        #BAS(#PTP_PARAMS,70)
        #PTP XP1      
        str_cont = ''
        str_cdis = ''
        if self.APO_VALUE >= 0:
            str_cdis = 'C_PTP'
            str_cont = 'CONT '        
        self.addline(';FOLD PTP P%s %sVel=%.0f %% PDAT%s Tool[%i] Base[%i];%%{PE}%%R 5.5.31,%%MKUKATPBASIS,%%CMOVE,%%VPTP,%%P 1:PTP, 2:P%s, 3:%s, 5:%.0f, 7:PDAT%s' % (vname, str_cont, self.VEL_PTP, vname, self.TOOL_ID, self.BASE_ID, vname, str_cdis, self.VEL_PTP, vname))
        self.addline('$BWDSTART=FALSE')
        self.addline('PDAT_ACT=PPDAT%s' % vname)
        self.addline('FDAT_ACT=FP%s' % vname)
        self.addline('BAS(#PTP_PARAMS,%.0f)' % self.VEL_PTP)
        self.addline('PTP XP%s%s' % (vname,self.C_PTP))
        self.addline(';ENDFOLD')	
        
        # Write DAT information ---------------------------
        #DECL E6POS XP1={X 27.0236969,Y 1220.23962,Z 669.846619,A -90.0563354,B 53.4202652,C -178.565933,S 2,T 35,E1 0.0,E2 0.0,E3 0.0,E4 0.0,E5 0.0,E6 0.0}
        #DECL FDAT FP1={TOOL_NO 1,BASE_NO 0,IPO_FRAME #BASE,POINT2[] " ",TQ_STATE FALSE}
        #DECL PDAT PPDAT1={VEL 100.000,ACC 100.000,APO_DIST 100.000,GEAR_JERK 50.0000,EXAX_IGN 0}
        self.addDAT('DECL E6AXIS XP%s={%s}' % (vname, angles_2_str(joints))) 
        self.addDAT('DECL FDAT FP%s={TOOL_NO %i,BASE_NO %i,IPO_FRAME #BASE,POINT2[] " ",TQ_STATE FALSE}' % (vname, self.TOOL_ID, self.BASE_ID)) 
        self.addDAT('DECL PDAT PPDAT%s={VEL %.3f,ACC 100.000,APO_DIST %.3f,APO_MODE #CPTP}' % (vname, self.VEL_PTP, max(self.APO_VALUE,1)))        
        
    def MoveL(self, pose, joints, conf_RLF=None):
        """Add a linear movement"""        
        self.nPosDat = self.nPosDat + 1
        vname = '%i' % self.nPosDat
        
        # Write SRC information ---------------------------
        #;FOLD LIN P56 Vel=2 m/s CPDAT37 Tool[1]:graco_negra Base[1]:vol1;%{PE}%R 8.3.42,%MKUKATPBASIS,%CMOVE,%VLIN,%P 1:LIN, 2:P56, 3:, 5:2, 7:CPDAT37
        #$BWDSTART=FALSE
        #LDAT_ACT=LCPDAT37
        #FDAT_ACT=FP56
        #BAS(#CP_PARAMS,2)
        #LIN XP56

        # If ARC information is provided:
        #;FOLD LIN P5  CPDAT4 ARC  Pgno= 108 W3 Tool[1]:ArcGun Base[0];%{PE}%R 5.5.0,%MKUKATPA20,%CARC_SWI,%VLIN,%P 1:LIN, 2:P5, 3:, 5:0.2, 7:CPDAT4, 10:108, 11:W3
        #$BWDSTART=FALSE
        #LDAT_ACT=LCPDAT4
        #FDAT_ACT=FP5
        #BAS(#CP_PARAMS,LDEFAULT.VEL)
        #A20(ARC_SWIP,ADEFAULT,MW3,108)
        #LIN XP5 
        #;ENDFOLD
        str_cont = ''
        str_cdis = ''
        if self.APO_VALUE >= 0:
            str_cdis = 'C_DIS'
            str_cont = 'CONT '

        wid = ''
        if not self.ARC_ON:
            self.addline(';FOLD LIN P%s %sVel=%.3f m/s CPDAT%s Tool[%i] Base[%i];%%{PE}%%R 5.5.0,%%MKUKATPA20,%%CARC_SWI,%%VLIN,%%P 1:LIN, 2:P%s, 3:%s, 5:%.0f, 7:CPDAT%s' % (vname, str_cont, self.speed_ms, vname, self.TOOL_ID, self.BASE_ID, vname, str_cdis, self.speed_ms, vname))
        else:
            self.nArcId = self.nArcId + 1
            wid = 'W%i' % self.nArcId            
            self.addline(';FOLD LIN P%s  CPDAT%s ARC  Pgno= %i %s Tool[%i] Base[%i];%%{PE}%%R 5.5.0,%%MKUKATPBASIS,%%CMOVE,%%VLIN,%%P 1:LIN, 2:P%s, 3:%s, 5:%.0f, 7:CPDAT%s, 10:%i, 11:%s' % (vname, str_cont, self.ARC_Pgno, wid, self.TOOL_ID, self.BASE_ID, vname, str_cdis, self.speed_ms, vname, self.ARC_Pgno, wid))
        self.addline('$BWDSTART=FALSE')
        self.addline('LDAT_ACT=LCPDAT%s' % vname)
        self.addline('FDAT_ACT=FP%s' % vname)
        if not self.ARC_ON:            
            self.addline('BAS(#CP_PARAMS,%.0f)' % self.speed_ms)           
        else:
            self.addline('BAS(#CP_PARAMS,LDEFAULT.VEL)')
            self.addline('A20(ARC_SWIP,ADEFAULT,M%s,%i)' % (wid, self.ARC_Pgno))
            
        self.addline('LIN XP%s%s' % (vname,self.C_DIS))
        self.addline(';ENDFOLD')
        
        # Write DAT information ---------------------------
        #DECL E6POS XP56={X -293.060028,Y 165.117493,Z 18.0715256,A -64.8358612,B 39.9032936,C -161.694565,S 2,T 3,E1 0.0,E2 0.0,E3 0.0,E4 0.0,E5 0.0,E6 0.0}
        #DECL FDAT FP56={TOOL_NO 1,BASE_NO 1,IPO_FRAME #BASE,POINT2[] " ",TQ_STATE FALSE}
        #DECL LDAT LCPDAT37={VEL 2.00000,ACC 100.000,APO_DIST 100.000,APO_FAC 50.0000,AXIS_VEL 100.000,AXIS_ACC 100.000,ORI_TYP #VAR,CIRC_TYP #BASE,JERK_FAC 50.0000,GEAR_JERK 50.0000,EXAX_IGN 0}

        # with arc information:
        #DECL E6POS XP3={X 663.532104,Y -614.288025,Z 2346.43799,A 171.796906,B -55.8371086,C 6.86900616,S 0,T 14,E1 -2848.03809,E2 0.0,E3 0.0,E4 0.0,E5 0.0,E6 0.0}
        #DECL FDAT FP3={TOOL_NO 1,BASE_NO 0,IPO_FRAME #BASE,POINT2[] " ",TQ_STATE FALSE}
        #DECL LDAT LCPDAT2={VEL 2.0,ACC 100.0,APO_DIST 100.0,APO_FAC 50.0,ORI_TYP #VAR,CIRC_TYP #BASE,JERK_FAC 50.0}
        #DECL WELD_FI MW1={PRG_NO 1,VELOCITY 0.109999999,WEAVFIG_MECH 1,WEAVLEN_MECH 6.0,WEAVAMP_MECH 7.0,WEAVANG_MECH 180.0,END_TIME 0.0149999997}
        self.addDAT('DECL E6POS XP%s={%s}' % (vname, pose_2_str_ext(pose,joints))) 
        self.addDAT('DECL FDAT FP%s={TOOL_NO %i,BASE_NO %i,IPO_FRAME #BASE,POINT2[] " ",TQ_STATE FALSE}' % (vname, self.TOOL_ID, self.BASE_ID)) 
        self.addDAT('DECL LDAT LCPDAT%s={VEL %.5f,ACC 100.000,APO_DIST %.3f,APO_FAC 50.0000,ORI_TYP #VAR,CIRC_TYP #BASE,JERK_FAC 50.0000}' % (vname, self.speed_ms, max(self.APO_VALUE,1))) 
        if not self.ARC_ON:
            pass
        else:
            self.addDAT('DECL WELD_FI M%s={PRG_NO 1,VELOCITY %.3f,WEAVFIG_MECH 1,WEAVLEN_MECH 6.0,WEAVAMP_MECH 7.0,WEAVANG_MECH 180.0,END_TIME 0.015}' % (wid, self.speed_ms))
        
    def MoveC(self, pose1, joints1, pose2, joints2, conf_RLF_1=None, conf_RLF_2=None):
        """Add a circular movement"""
        self.addline('CIRC {' + pose_2_str_ext(pose1,joints1) + '},{' + pose_2_str_ext(pose2,joints2) + '}' + self.C_DIS)
        
    def setFrame(self, pose, frame_id=None, frame_name=None):
        """Change the robot reference frame"""
        if frame_name is not None and frame_name.endswith("Base"): # robot base frame
            frame_id = 0
            self.BASE_ID = frame_id
            self.addline('BASE_DATA[%i] = {FRAME: %s}' % (self.BASE_ID, pose_2_str(pose)))            
        elif frame_id is not None and frame_id >= 0: # specified ID reference frame
            self.BASE_ID = frame_id
            self.addline('BASE_DATA[%i] = {FRAME: %s}' % (self.BASE_ID, pose_2_str(pose)))
        else: # unspecified ID reference frame
            self.BASE_ID = 1
            self.addline('BASE_DATA[%i] = {FRAME: %s}' % (self.BASE_ID, pose_2_str(pose)))
        
    def setTool(self, pose, tool_id=None, tool_name=None):
        """Change the robot TCP"""
        if tool_id is not None and tool_id >= 0:
            self.TOOL_ID = tool_id
            self.addline('TOOL_DATA[%i] = {FRAME: %s}' % (self.TOOL_ID, pose_2_str(pose)))
        else:
            self.TOOL_ID = 1
            self.addline('TOOL_DATA[%i] = {FRAME: %s}' % (self.TOOL_ID, pose_2_str(pose)))
            #self.addline('$TOOL = {FRAME: ' + pose_2_str(pose) + '}')
        
    def Pause(self, time_ms):
        """Pause the robot program"""
        if time_ms <= 0:
            self.addline('HALT')
        else:
            self.addline('WAIT SEC %.3f' % (time_ms*0.001))
        
    def setSpeed(self, speed_mms):
        """Changes the robot speed (in mm/s)"""
        self.speed_ms = speed_mms/1000.0
        self.addline('$VEL.CP = %.5f' % (self.speed_ms))
    
    def setAcceleration(self, accel_mmss):
        """Changes the current robot acceleration"""
        self.accel_mss = accel_mmss/1000.0
        self.addline('$ACC.CP = %.5f' % (self.accel_mss))
                
    def setSpeedJoints(self, speed_degs):
        """Changes the robot joint speed (in deg/s)"""
        self.speed_degs = speed_degs
        self.addline('$VEL.ORI1 = %.5f' % speed_degs)
        self.addline('$VEL.ORI2 = %.5f' % speed_degs)
    
    def setAccelerationJoints(self, accel_degss):
        """Changes the robot joint acceleration (in deg/s2)"""
        self.accel_degss = accel_degss
        self.addline('$ACC.ORI1 = %.5f' % accel_degss)
        self.addline('$ACC.ORI2 = %.5f' % accel_degss)
        
    def setZoneData(self, zone_mm):
        """Changes the zone data approach (makes the movement more smooth)"""
        self.APO_VALUE = zone_mm
        if zone_mm >= 0:
            self.addline('$APO.CPTP = %.3f' % zone_mm)
            self.addline('$APO.CDIS = %.3f' % zone_mm)
            C_DIS = ' C_DIS'
            C_PTP = ' C_PTP'
        else:
            C_DIS = ''
            C_PTP = ''
        
    def setDO(self, io_var, io_value):
        """Sets a variable (output) to a given value"""
        if type(io_var) != str:  # set default variable name if io_var is a number
            io_var = '$OUT[%s]' % str(io_var)        
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
            io_var = '$IN[%s]' % str(io_var)        
        if type(io_value) != str: # set default variable value if io_value is a number            
            if io_value > 0:
                io_value = 'TRUE'
            else:
                io_value = 'FALSE'
        
        # at this point, io_var and io_value must be string values
        if timeout_ms < 0:
            self.addline('WAIT FOR (%s==%s)' % (io_var, io_value))
        else:
            self.addline('START_TIMER:')
            self.addline('$TIMER_STOP[1]=TRUE')
            self.addline('$TIMER_FLAG[1]=FALSE')
            self.addline('$TIMER[1]=%.3f' % (float(timeout_ms)*0.001))
            self.addline('$TIMER_STOP[1]=FALSE')
            self.addline('WAIT FOR (%s==%s OR $TIMER_FLAG[1])' % (io_var, io_value))
            self.addline('$TIMER_STOP[1]=TRUE')
            self.addline('IF $TIMER_FLAG[1]== TRUE THEN')
            self.addline('    HALT ; Timed out!')
            self.addline('    GOTO START_TIMER')
            self.addline('ENDIF')      
        
    def RunCode(self, code, is_function_call = False):
        """Adds code or a function call"""
        if is_function_call:
            code.replace(' ','_')
            if not code.endswith(')'):
                code = code + '()'
            if code.startswith('ARC_ON'):
                self.ARC_ON = True
                self.ARC_REQUIRED = True
            elif code.startswith('ARC_OFF'):
                self.ARC_ON = False
            else:
            
                fcn_def = code
                if '(' in fcn_def:
                    fcn_def = fcn_def.split('(')[0]
                if not (fcn_def in self.PROG_CALLS):
                    self.PROG_CALLS.append(fcn_def)
                
                self.addline(code)
        else:
            self.addline(code)
        
    def RunMessage(self, message, iscomment = False):
        """Add a joint movement"""
        if iscomment:
            self.addline('; ' + message)
        else:
            self.addline('Wait for StrClear($LOOP_MSG[])')
            self.addline('$LOOP_CONT = TRUE')
            self.addline('$LOOP_MSG[] = "%s"' % message)
            
# ------------------ private ----------------------                
    def addline(self, newline):
        """Add a program line"""
        self.PROG = self.PROG + newline + '\n'
        
    def addDAT(self, newline):
        self.PROG_DAT = self.PROG_DAT + newline + '\n'
        
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
