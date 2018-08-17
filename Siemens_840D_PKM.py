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
# This file is a sample POST PROCESSOR script to generate robot programs for a
# Siemens controller (Siemens Sinumerik programming language)
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


# Customize the following parameters:
M_WAIT_DI = 'M66'       # Provide the M code to wait for a digital input
M_SET_DO_HIGH = 'M62'   # Provide the M code to set a digital output HIGH (1 or True)
M_SET_DO_LOW = 'M62'    # Provide the M code to set a digital output LOW (0 or False)

MM_2_UNITS = 1.0 # Use Millimeter units
#MM_2_UNITS = 1.0/25.4 # Use Inch units


# ----------------------------------------------------
# Import RoboDK tools
from robodk import *

def conf_2_STAT(confRLF):
    if confRLF is None:
        return 2 #"'B010'"
    config = 0
    if confRLF[2] > 0:
        config = config + 4
        
    if confRLF[1] == 0:
        config = config + 2
        
    if confRLF[0] > 0:
        config = config + 1
    
    return config
    
def joints_2_TU(joints):
    if joints is None:
        return 0 # "'B000000'"
        
    turn = 0
    njoints = len(joints)
    for i in range(njoints):
        if joints[i] < 0:
            turn = turn + 2**(njoints-1-i)
    return turn


# ----------------------------------------------------    
# Object class that handles the robot instructions/syntax
class RobotPost(object):
    """Robot post object"""
    PROG_EXT = 'mpf'        # set the program extension
    
    # other variables
    ROBOT_POST = ''
    ROBOT_NAME = ''
    PROG_FILES = []
    
    PROG = ''
    PROG_COUNT = 0
    LOG = ''
    nAxes = 6
    nId = 0
    REF_FRAME = eye(4)
    INV_TOOL_FRAME = eye(4) # Force TC_DCP to 0 by post multiplying poses
    
    SPEED_UNITS_MIN = 5000 * MM_2_UNITS
    SPEED_DEG_MIN = 2000
    Nline = 0
    
    LAST_X = None
    LAST_Y = None
    LAST_Z = None
    LAST_POSE = None
    TRAORI = None
    
    
    # ----------------------------------------------------
    def pose_2_str(self, pose, remember_last=False, joints=None):
        """Prints a pose target"""
        x,y,z = pose.Pos()
        i,j,k = pose.VZ()
        x = x * MM_2_UNITS
        y = y * MM_2_UNITS
        z = z * MM_2_UNITS
        strjnts = ''
        if joints is not None and len(joints) > 5:
            strjnts = " Y2=%.3f" % joints[5]
            
        if remember_last:
            G_LINE = ''
            if self.LAST_X != x:
                G_LINE += 'X%.3f ' % x
            if self.LAST_Y != y:
                G_LINE += 'Y%.3f ' % y
            if self.LAST_Z != z or len(G_LINE) == 0:
                G_LINE += 'Z%.3f ' % z
            G_LINE += 'A3=%.3f ' % i
            G_LINE += 'B3=%.3f ' % j
            G_LINE += 'C3=%.3f ' % k
            self.LAST_X = x
            self.LAST_Y = y
            self.LAST_Z = z            
            G_LINE = G_LINE[:-1]        
            return G_LINE + strjnts
        else:
            return ('X%.3f Y%.3f Z%.3f A3%.3f B3%.3f C3%.3f%s' % (x,y,z,i,j,k,strjnts))
        
    def joints_2_str(self, joints):
        """Prints a joint target"""
        str = ''
        data = ['ST1','ST2','ST3','A','C','G','H','I','J','K','L']
        for i in range(len(joints)):
            str = str + ('%s=%.6f ' % (data[i], joints[i]))
        str = str[:-1]
        return str
    
    
    def __init__(self, robotpost=None, robotname=None, robot_axes = 6, **kwargs):
        self.ROBOT_POST = robotpost
        self.ROBOT_NAME = robotname
        self.PROG = ''
        self.LOG = ''
        self.nAxes = robot_axes
        
    def ProgStart(self, progname):
        self.PROG_COUNT = self.PROG_COUNT + 1
        if self.PROG_COUNT <= 1:
            import datetime
            self.addcomment('File: %s' % progname)
            self.addcomment('Created on %s' % datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
            self.addcomment('Program created using RoboDK')
            if MM_2_UNITS == 1.0:
                self.addline('G710 G40 ; metric, no tool radius compensation')
            elif MM_2_UNITS == 1.0/25.4:
                self.addline('G700 G40 ; inch, no tool radius compensation')
            else:
                raise Exception("Unknown units!! Define MM_2_UNITS properly.")
            
            self.addline('G17 G94 G90 G60 G601 FNORM')            
            self.addline('SOFT ; Smooth acceleration')            
            self.addline('FFWON ; Look ahead')
            self.addline('FIFOCTRL')
            self.addline('G645')
            self.addline('DYNNORM ; Specific settings for acceleration')
            self.addline('COMPCAD ; Not working with radius compensation')
            self.addcomment('')
            
        else:
            self.addcomment('')
            self.addcomment('---------- Subprogram: %s ----------' % progname)
            #self.addline('PROC %s' % progname)
            self.addline('%s:' % progname)
            self.addline('TRAORI')
            self.TRAORI = True
        
        
    def ProgFinish(self, progname):
        if self.PROG_COUNT <= 1:    
            self.addcomment('')
            self.addcomment('Stop Spindle')
            self.addline('M5')
            self.addcomment('')
            self.addcomment('End of main program ' + progname)
            self.addline('M30')
            self.addcomment('---------------------------')
            self.addcomment('')
        else:
            #self.addline('RET("%s_done")' % progname) # needs to be in a file as SPF
            #self.addline('M17 ; end of subprogram %s' % progname) # needs to be in a file as SPF
            self.addline('GOTOB ' + progname + "_done")
            self.addcomment('------------------------------------')
            self.addcomment('') 
        
    def ProgSave(self, folder, progname, ask_user=False, show_result=False):
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
        print('SAVED: %s\n' % filesave)
        #---------------------- show result
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
        
    def set_cartesian_space(self):
        if not self.TRAORI:
            self.TRAORI = True
            self.addline('TRAORI')
            self.addline('G54')
        
    def set_joint_space(self):
        if self.TRAORI:
            self.TRAORI = False
            self.addline('TRAFOOF')
        
    def MoveJ(self, pose, joints, conf_RLF=None):
        """Add a joint movement"""
        self.set_joint_space()
        self.addline('G1 ' + self.joints_2_str(joints) + ' F%.1f' % self.SPEED_DEG_MIN)
        #self.addline('G0 ' + self.joints_2_str(joints)) # G0 is the fastest 
        if pose is not None:
            self.addline('; TRAORI')
            self.addline('; PTP G1 ' + self.pose_2_str(self.REF_FRAME * pose * self.INV_TOOL_FRAME, True) + ' STAT=%i TU=%i F%.1f ; same joint coordinate' % (conf_2_STAT(conf_RLF), joints_2_TU(joints), self.SPEED_UNITS_MIN))
            self.addline('; TRAFOOF')

        self.LAST_POSE = None
        
    def MoveL(self, pose, joints, conf_RLF=None):
        """Add a linear movement"""
        if pose is None:
            self.set_joint_space()
            self.addline('G1 ' + self.joints_2_str(joints) + ' F%.1f' % self.SPEED_UNITS_MIN)
        else:
            self.set_cartesian_space()                
            self.addline('G1 ' + self.pose_2_str(self.REF_FRAME * pose * self.INV_TOOL_FRAME, True, joints) + ' F%.1f' % self.SPEED_UNITS_MIN)
            #self.addline('PTP G1 ' + self.pose_2_str(self.REF_FRAME * pose * self.INV_TOOL_FRAME, True) + ' STAT=%i TU=%i F%.1f' % (conf_2_STAT(conf_RLF), joints_2_TU(joints), self.SPEED_UNITS_MIN))
            # Note: it is important to have 
            return            
            
            if self.LAST_POSE is None:
                self.addline('G1 ' + self.pose_2_str(self.REF_FRAME * pose * self.INV_TOOL_FRAME, True, joints) + ' F%.1f' % self.SPEED_UNITS_MIN)
            else:
                pose_shift = invH(self.LAST_POSE)*pose
                angle = pose_angle(pose_shift)*180/pi
                
                x,y,z,w,p,r = Pose_2_UR(pose_shift)
                x = x * MM_2_UNITS
                y = y * MM_2_UNITS
                z = z * MM_2_UNITS       
                
                steps = int(angle/(1))
                steps = float(max(1,steps))
                self.addline('; next move %.1f deg divided in %i steps' % (angle, steps))
                xd = x/steps
                yd = y/steps
                zd = z/steps
                wd = w/steps
                pd = p/steps
                rd = r/steps
                for i in range(int(steps)):
                    factor = i+1
                    hi = UR_2_Pose([xd*factor,yd*factor,zd*factor,wd*factor,pd*factor,rd*factor])
                    self.addline('G1 ' + self.pose_2_str(self.REF_FRAME*self.LAST_POSE*hi*self.INV_TOOL_FRAME, True) + ' F%.1f' % self.SPEED_UNITS_MIN)
            
            self.LAST_POSE = pose
        
    def MoveC(self, pose1, joints1, pose2, joints2, conf_RLF_1=None, conf_RLF_2=None):
        """Add a circular movement"""
        self.nId = self.nId + 1
        xyz1 = (self.REF_FRAME*pose1*self.INV_TOOL_FRAME).Pos()
        xyz2 = (self.REF_FRAME*pose2*self.INV_TOOL_FRAME).Pos()  
        #xyz1 = (pose1).Pos()
        #xyz2 = (pose2).Pos()          
        self.addline('G2 X%.3f Y%.3f Z%.3f I1=%.3f J1=%.3f K1=%.3f F%.1f' % (xyz2[0], xyz2[1], xyz2[2], xyz1[0], xyz1[1], xyz1[2], self.SPEED_UNITS_MIN))
        
    def setFrame(self, pose, frame_id=None, frame_name=None):
        """Change the robot reference frame"""
        self.addcomment('------ Update reference: %s ------' % (frame_name if frame_name is not None else ''))
        #self.addline('TRAORI')
        self.set_cartesian_space()              
        [x,y,z,a,b,c] = Pose_2_Staubli(pose)
        x = x * MM_2_UNITS
        y = y * MM_2_UNITS
        z = z * MM_2_UNITS
        self.addline('; $P_UIFR[1]=CTRANS(X,%.5f,Y,%.5f,Z,%.5f):CROT(X,%.5f,Y,%.5f,Z,%.5f)' % (x,y,z,a,b,c))
        self.addline('G54')
        self.addcomment('---------------------------')
        self.addcomment('')
        
        
    def setTool(self, pose, tool_id=None, tool_name=None):
        """Change the robot TCP"""
        #if tool_id is not None and tool_id > 0:
        #    self.addline('T%i D1' % tool_id)
        #    self.addcomment('Using controller definition for tool %i' % frame_id)
        #    return
        
        self.nId = self.nId + 1
        self.addcomment('------ Update TCP: %s ------' % (tool_name if tool_name is not None else ''))
        self.set_cartesian_space()#self.addline('TRAORI')
        [ x, y, z,_a,_b,_c] = Pose_2_Staubli(roty(pi)*pose)
        [_x,_y,_z, a, b, c] = Pose_2_Staubli(pose)
        x = x * MM_2_UNITS
        y = y * MM_2_UNITS
        z = z * MM_2_UNITS
        self.INV_TOOL_FRAME = invH(pose)
        self.INV_TOOL_FRAME.setPos([0,0,0])
        self.addcomment('$TC_DP5[1,1]=%.5f' % x)
        self.addcomment('$TC_DP4[1,1]=%.5f' % y)
        self.addcomment('$TC_DP3[1,1]=%.5f' % z)
        #self.addline('$TC_DPC3[1,1]=%.5f' % a)        
        #self.addline('$TC_DPC2[1,1]=%.5f' % b)
        #self.addline('$TC_DPC1[1,1]=%.5f' % c)
        self.addcomment('$TC_DPC3[1,1]=0.0')        
        self.addcomment('$TC_DPC2[1,1]=0.0')
        self.addcomment('$TC_DPC1[1,1]=0.0')
        self.addline('T="%s" D1' % tool_name) # Use tool 1 profile 1
        
        self.addcomment('---------------------------- ')
        self.addcomment('')
        
    def Pause(self, time_ms):
        """Pause the robot program"""
        if time_ms < 0:
            #self.addline('G9 ; STOP') # can't continue
            self.addline('M0 ; STOP')
        else:
            self.addline('G4 F%.0f ; pause in seconds' % (time_ms*0.001))
    
    def setSpeed(self, speed_mms):
        """Changes the robot speed (in mm/s)"""
        self.SPEED_UNITS_MIN = speed_mms*60.0*MM_2_UNITS
    
    def setAcceleration(self, accel_mmss):
        """Changes the robot acceleration (in mm/s2)"""
        self.addcomment('Acceleration set to %.3f mm/s2' % accel_mmss)
    
    def setSpeedJoints(self, speed_degs):
        """Changes the robot joint speed (in deg/s)"""
        #self.addcomment('Joint speed set to %.3f deg/s' % speed_degs)
        self.SPEED_DEG_MIN = speed_degs * 60
    
    def setAccelerationJoints(self, accel_degss):
        """Changes the robot joint acceleration (in deg/s2)"""
        self.addcomment('Joint acceleration set to %.3f deg/s2' % accel_degss)
        
    def setZoneData(self, zone_mm):
        """Changes the rounding radius (aka CNT, APO or zone data) to make the movement smoother"""
        #self.addcomment('Look ahead desired tolerance: %.1f mm' % zone_mm)
        if zone_mm < 0:
            self.addline('CYCLE832(0,_OFF,1)')
        else:
            self.addline('CYCLE832(0.1,_FINISH,1)')
        
    def setDO(self, io_var, io_value):
        """Sets a variable (digital output) to a given value"""
        comment = 'Set digital output %s = %s' % (io_var, io_value)
        if type(io_var) != str:  # set default variable name if io_var is a number
            io_var = 'P%s' % str(io_var)
        if type(io_value) != str: # set default variable value if io_value is a number
            if io_value > 0:
                io_value = M_SET_DO_HIGH
            else:
                io_value = M_SET_DO_LOW

        # at this point, io_var and io_value must be string values
        self.addline('%s %s ; %s' % (io_value, io_var, comment))

    def waitDI(self, io_var, io_value, timeout_ms=-1):
        """Waits for a variable (digital input) io_var to attain a given value io_value. Optionally, a timeout can be provided."""
        comment = 'Wait Digital Input %s = %s' % (io_var, io_value)
        if timeout_ms > 0:
            comment = comment + ' (timeout = %.3f)' % (timeout_ms*0.001)
            
        if type(io_var) != str:  # set default variable name if io_var is a number
            io_var = 'P%s' % str(io_var)
        if type(io_value) != str: # set default variable value if io_value is a number
            if io_value > 0:
                io_value = 'L3'
            else:
                io_value = 'L4'

        # at this point, io_var and io_value must be string values
        if timeout_ms < 0:
            self.addline('%s %s %s Q9999; %s' % (M_WAIT_DI, io_var, io_value, comment))
        else:
            self.addline('%s %s %s Q%.3f; %s' % (M_WAIT_DI, io_var, io_value, timeout_ms*0.001, comment))
        
    def RunCode(self, code, is_function_call = False):
        """Adds code or a function call"""
        if is_function_call:
            code.replace(' ','_')
            #self.addline(code)
            
            if code.lower().startswith("setrpm("):
                # if the program call is Extruder(123.56), we extract the number as a string and convert it to a number
                new_rpm = float(code[7:-1]) # it needs to retrieve the extruder length from the program call
                self.addline('S' + code)
                
            else:
                self.addline('GOTOF ' + code)
                self.addline(code + '_done:')
            
        else:
            self.addcomment(code)
        
    def RunMessage(self, message, iscomment = False):
        """Display a message in the robot controller screen (teach pendant)"""
        if iscomment:
            self.addcomment(message)
        else:
            self.addcomment('Display message: %s' % message)
        
# ------------------ private ----------------------                
    def addline(self, newline):
        """Add a program line"""
        self.Nline = self.Nline + 10
        self.PROG = self.PROG + ('N%02i ' % self.Nline) + newline + '\n'        
        
    def addcomment(self, newline):
        """Add a comment line"""
        self.PROG = self.PROG + '; ' + newline + '\n'
        
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

    robot = RobotPost()

    robot.ProgStart("Program")
    robot.RunMessage("Program generated by RoboDK using a custom post processor", True)
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
    robot.MoveJ(None, [-46.18419, -6.77518, -20.54925, 71.38674, 49.58727, -302.54752] )
    robot.ProgFinish("Program")
    # robot.ProgSave(".","Program",True)
    print(robot.PROG)
    if len(robot.LOG) > 0:
        mbox('Program generation LOG:\n\n' + robot.LOG)

    input("Press Enter to close...")

if __name__ == "__main__":
    """Function to call when the module is executed by itself: test"""
    test_post()
