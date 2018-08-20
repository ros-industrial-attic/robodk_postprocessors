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
# for a Staubli (VAL3) robot with RoboDK
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


# Program.pjx file (references data file as %s.dtx)
PROGRAM_PJX = '''<?xml version="1.0" encoding="utf-8" ?>
<Project xmlns="http://www.staubli.com/robotics/VAL3/Project/3">
  <Parameters version="s7.3.1" stackSize="5000" millimeterUnit="true" />
  <Programs>
    <Program file="start.pgx" />
    <Program file="stop.pgx" />
  </Programs>
  <Database>
    <Data file="%s.dtx" />
  </Database>
  <Libraries>
  </Libraries>
</Project>
'''

DATA_DTX = '''<?xml version="1.0" encoding="utf-8" ?>
<Database xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns="http://www.staubli.com/robotics/VAL3/Data/2">
  <Datas>
    <Data name="%s" access="public" xsi:type="array" type="frame" size="%i">
%s    </Data>
    <Data name="jPark" access="public" xsi:type="array" type="jointRx" size="1">
      <Value key="0" j1="0.000" j2="0.000" j3="90.000" j4="0.000" j5="90.000" j6="0.000" />
    </Data>
    <Data name="%s" access="public" xsi:type="array" type="jointRx" size="%i">
%s    </Data>
    <Data name="mNomSpeed" access="public" xsi:type="array" type="mdesc" size="1">
      <Value key="0" accel="100" vel="100" decel="100" tmax="99999" rmax="99999" blend="off" leave="50" reach="50" />
    </Data>
    <Data name="%s" access="public" xsi:type="array" type="mdesc" size="%i">
%s    </Data>
    <Data name="nTraj" access="public" xsi:type="array" type="num" size="1"/>
    <Data name="nTimeStop" access="private" xsi:type="array" type="num" size="1"/>
    <Data name="nTimeStart" access="private" xsi:type="array" type="num" size="1"/>
    <Data name="nMode" access="private" xsi:type="array" type="num" size="1"/>
    <Data name="nEtat" access="private" xsi:type="array" type="num" size="1"/>
    <Data name="%s" access="public" xsi:type="array" type="pointRx" size="%i">
%s    </Data>
    <Data name="%s" access="public" xsi:type="array" type="tool" size="%i">
%s    </Data>
  </Datas>
</Database>
'''

# start.pjx file (references data file as %s.dtx)
START_PGX = '''<?xml version="1.0" encoding="utf-8" ?>
<Programs xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns="http://www.staubli.com/robotics/VAL3/Program/2">
  <Program name="start" access="public">
    <Code><![CDATA[
begin
%s
end
      ]]></Code>
  </Program>
</Programs>
'''

STOP_PGX = '''<?xml version="1.0" encoding="utf-8" ?>
<Programs xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns="http://www.staubli.com/robotics/VAL3/Program/2">
  <Program name="stop" access="private">
    <Code><![CDATA[
        begin
          resetMotion()
          disablePower()
        end
      ]]></Code>
  </Program>
</Programs>
'''


def Pose_2_Staubli(pose):
    """Converts a pose to a Staubli target target"""
    #return Pose_2_Adept(pose)# older versions (V+)
    return Pose_2_TxyzRxyz(pose)# newer versions (VAL3)

# ----------------------------------------------------
def pose_2_str(pose):
    """Prints a pose target"""
    [x,y,z,r,p,w] = Pose_2_Staubli(pose)
    return ('x="%.3f" y="%.3f" z="%.3f" rx="%.3f" ry="%.3f" rz="%.3f"' % (x,y,z,r,p,w))
    
def angles_2_str(angles):
    """Prints a joint target for Staubli VAL3 XML"""
    str = ''    
    for i in range(len(angles)):
        str = str + ('j%i="%.5f" ' % (i, angles[i]))
    str = str[:-1]
    return str
    
def getSaveFolder(strdir='C:\\', strtitle='Save program folder ...'):
    import tkinter
    from tkinter import filedialog
    options = {}
    options['initialdir'] = strdir
    options['title'] = strtitle
    root = tkinter.Tk()
    root.withdraw()
    file_path = tkinter.filedialog.askdirectory(**options)
    return file_path

# ----------------------------------------------------    
# Object class that handles the robot instructions/syntax
class RobotPost(object):
    """Robot post object"""    
    # other variables
    ROBOT_POST = ''
    ROBOT_NAME = ''
    PROG_FILES = []
    PROG_PGX = ''
    PROG_DTX = ''
    LOG = ''
    nAxes = 6
    TAB_PGX = '  '
    DEFAULT_SPEED = 150
    DEFAULT_SMOOTH = 0.1
    SPEED = DEFAULT_SPEED
    SMOOTH = DEFAULT_SMOOTH
    REF_NAME = 'fReference'
    REF_CURRENT = 'world[0]'
    REF_DATA = ''
    REF_COUNT = 0
    TOOL_NAME = 'tTool'
    TOOL_CURRENT = 'flange[0]'
    TOOL_DATA = ''
    TOOL_COUNT = 0
    SPEED_NAME = 'mSpeed'
    SPEED_CURRENT = 'mNomSpeed'
    SPEED_DATA = ''
    SPEED_COUNT = 0
    JOINT_NAME = 'jJoint'
    JOINT_DATA = ''
    JOINT_COUNT = 0
    POINT_NAME = 'pPoint'
    POINT_DATA = ''
    POINT_COUNT = 0
    
    def __init__(self, robotpost=None, robotname=None, robot_axes = 6, **kwargs):
        self.ROBOT_POST = robotpost
        self.ROBOT_NAME = robotname
        self.PROG = ''
        self.LOG = ''
        self.nAxes = robot_axes
        
    def ProgStart(self, progname):
        self.addline('// Program %s start' % progname)
        
    def ProgFinish(self, progname):
        self.addline('')
        self.addline('waitEndMove()')
        self.addline('// Program %s end' % progname)
        
    def ProgSave(self, folder, progname, ask_user = False, show_result = False):
        if ask_user or not DirExists(folder):
            foldersave = getSaveFolder(folder, 'Save program as...')
            if foldersave is not None and len(foldersave) > 0:
                foldersave = foldersave
            else:
                return
        else:
            foldersave = folder
        
        folderprog = foldersave + '/' + progname
        if not DirExists(folderprog):
            import os
            os.makedirs(folderprog)
        #-----------------------------------
        # start.pgx
        start_file = folderprog + '/start.pgx'
        fid = open(start_file, "w")
        fid.write(START_PGX % self.PROG_PGX)
        fid.close()
        #-----------------------------------
        # stop.pgx
        stop_file = folderprog + '/stop.pgx'
        fid = open(stop_file, "w")
        fid.write(STOP_PGX)
        fid.close()
        #-----------------------------------
        # program.pjx
        project_file = folderprog + '/%s.pjx' % progname
        fid = open(project_file, "w")
        fid.write(PROGRAM_PJX % progname)
        fid.close()
        print('SAVED: %s\n' % project_file)
        #-----------------------------------
        # program.dtx
        program_data = folderprog + '/%s.dtx' % progname
        fid = open(program_data, "w")
        fid.write(DATA_DTX % (self.REF_NAME, self.REF_COUNT, self.REF_DATA,  self.JOINT_NAME, self.JOINT_COUNT, self.JOINT_DATA,  self.SPEED_NAME, self.SPEED_COUNT, self.SPEED_DATA,  self.POINT_NAME, self.POINT_COUNT, self.POINT_DATA,  self.TOOL_NAME, self.TOOL_COUNT, self.TOOL_DATA))
        fid.close()
        #-----------------------------------
        
        #self.UploadFTP(folderprog)
        self.PROG_FILES = folderprog
        
        if show_result:            
            if type(show_result) is str:
                # Open file with provided application
                import subprocess
                p = subprocess.Popen([show_result, start_file])
                p = subprocess.Popen([show_result, program_data])                
            elif type(show_result) is list:
                import subprocess
                p = subprocess.Popen(show_result + [filesave])   
            else:
                # open file with default application
                import os
                os.startfile(start_file)
                os.startfile(program_data)
            if len(self.LOG) > 0:
                mbox('Program generation LOG:\n\n' + self.LOG)
        
    def ProgSendRobot(self, robot_ip, remote_path, ftp_user, ftp_pass):
        """Send a program to the robot using the provided parameters. This method is executed right after ProgSave if we selected the option "Send Program to Robot".
        The connection parameters must be provided in the robot connection menu of RoboDK"""
        UploadFTP(self.PROG_FILES, robot_ip, remote_path, ftp_user, ftp_pass)
        
    def MoveJ(self, pose, joints, conf_RLF=None):
        """Add a joint movement"""
        #nTraj=movej(jJoints[0],tTool[0],mSpeed[0])
        #waitEndMove()
        #      <Value key="0" j1="0.000" j2="-10.000" j3="100.000" j4="0.000" j5="0.000" j6="-90.000" />
        variable = '%s[%i]' % (self.JOINT_NAME, self.JOINT_COUNT)	
        self.JOINT_DATA = self.JOINT_DATA + '      <Value key="%i" %s />\n' % (self.JOINT_COUNT, angles_2_str(joints))
        self.JOINT_COUNT = self.JOINT_COUNT + 1        
        self.addline('nTraj=movej(%s,%s,%s)' % (variable, self.TOOL_CURRENT, self.SPEED_CURRENT))
        #self.addline('waitEndMove()')
        
    def MoveL(self, pose, joints, conf_RLF=None):
        """Add a linear movement"""
        #nTraj=movej(jJoints[0],tTool[0],mSpeed[0])
        #waitEndMove()
        #      <Value key="0" x="-36.802" y="-6.159" z="500.000" rx="135.407" ry="80.416" rz="46.453" shoulder="lefty" elbow="epositive" wrist="wpositive" fatherId="fPartReal[0]" />
        if conf_RLF == None:
            str_config = 'shoulder="lefty" elbow="epositive" wrist="wpositive"'
        else:
            [rear, lowerarm, flip] = conf_RLF
            str_config = 'shoulder="%s" elbow="%s" wrist="%s"' % ("righty" if rear>0 else "lefty", "enegative" if lowerarm>0 else "epositive", "wnegative" if flip>0 else "wpositive")
        variable = '%s[%i]' % (self.POINT_NAME, self.POINT_COUNT)
        self.POINT_DATA = self.POINT_DATA + '      <Value key="%i" %s %s fatherId="%s" />\n' % (self.POINT_COUNT, pose_2_str(pose), str_config, self.REF_CURRENT)
        self.POINT_COUNT = self.POINT_COUNT + 1        
        self.addline('nTraj=movel(%s,%s,%s)' % (variable, self.TOOL_CURRENT, self.SPEED_CURRENT))
        
    def MoveC(self, pose1, joints1, pose2, joints2, conf_RLF_1=None, conf_RLF_2=None):
        """Add a circular movement"""
        # Needs to be checked
        if conf_RLF_1 == None:
            str_config = 'shoulder="lefty" elbow="epositive" wrist="wpositive"'
        else:
            [rear, lowerarm, flip] = conf_RLF_1
            str_config = 'shoulder="%s" elbow="%s" wrist="%s"' % ("righty" if rear>0 else "lefty", "enegative" if lowerarm>0 else "epositive", "wnegative" if flip>0 else "wpositive")
        variable1 = '%s[%i]' % (self.POINT_NAME, self.POINT_COUNT)
        variable2 = '%s[%i]' % (self.POINT_NAME, self.POINT_COUNT+1)        
        self.POINT_DATA = self.POINT_DATA + '      <Value key="%i" %s %s fatherId="%s" />\n' % (self.POINT_COUNT, pose_2_str(pose1), str_config, self.REF_CURRENT)
        self.POINT_DATA = self.POINT_DATA + '      <Value key="%i" %s %s fatherId="%s" />\n' % (self.POINT_COUNT+1, pose_2_str(pose2), str_config, self.REF_CURRENT)        
        self.POINT_COUNT = self.POINT_COUNT + 2       
        self.addline('nTraj=movec(%s,%s,%s,%s)' % (variable1, variable2, self.TOOL_CURRENT, self.SPEED_CURRENT))
        
    def setFrame(self, pose, frame_id=None, frame_name=None):
        """Change the robot reference frame"""
        #      <Value key="0" x="600.000" y="0.000" z="-465.000" rx="0.400" ry="0.100" rz="-45.000" fatherId="world[0]" />
        self.REF_CURRENT = '%s[%i]' % (self.REF_NAME, self.REF_COUNT)
        self.REF_DATA = self.REF_DATA + '      <Value key="%i" %s fatherId="world[0]" />\n' % (self.REF_COUNT, pose_2_str(pose))
        self.REF_COUNT = self.REF_COUNT + 1
        
    def setTool(self, pose, tool_id=None, tool_name=None):
        """Change the robot TCP"""
        #      <Value key="0" x="-5.972" y="209.431" z="55.323" rx="-90.190" ry="-0.880" rz="89.997" fatherId="flange[0]" ioLink="valve1" />
        self.TOOL_CURRENT = '%s[%i]' % (self.TOOL_NAME, self.TOOL_COUNT)
        self.TOOL_DATA = self.TOOL_DATA + '      <Value key="%i" %s fatherId="flange[0]" ioLink="valve1" />\n' % (self.TOOL_COUNT, pose_2_str(pose))
        self.TOOL_COUNT = self.TOOL_COUNT + 1
        
    def Pause(self, time_ms):
        """Pause the robot program"""
        if time_ms < 0:
            self.addline('popUpMsg("Paused. Press OK to continue")')
        else:
            self.addline('delay(%.3f)' % (time_ms*0.001))
    
    def setSpeed(self, speed_mms):
        """Changes the robot speed (in mm/s)"""
        #      <Value key="0" accel="100" vel="100" decel="100" tmax="50" rmax="100" blend="joint" leave="0.1" reach="0.1" />
        SPEED = speed_mms
        self.SPEED_CURRENT = '%s[%i]' % (self.SPEED_NAME, self.SPEED_COUNT)
        # blend = "off" / "joint" / "Cartesian"
        #self.SPEED_DATA = self.SPEED_DATA + '      <Value key="%i" accel="100" vel="100" decel="100" tmax="%.1f" rmax="100" blend="cartesian" leave="%.1f" reach="%0.1f" />\n' % (self.SPEED_COUNT, speed_mms, self.SMOOTH, self.SMOOTH)
        self.SPEED_DATA = self.SPEED_DATA + '      <Value key="%i" tmax="%.1f" rmax="100" leave="%.1f" reach="%0.1f" blend="cartesian" />\n' % (self.SPEED_COUNT, speed_mms, self.SMOOTH, self.SMOOTH)
        self.SPEED_COUNT = self.SPEED_COUNT + 1
    
    def setAcceleration(self, accel_mmss):
        """Changes the robot acceleration (in mm/s2)"""
        self.ACCEL_MMSS = accel_mmss
    
    def setSpeedJoints(self, speed_degs):
        """Changes the robot joint speed (in deg/s)"""
        self.SPEED_DEGS = speed_degs
    
    def setAccelerationJoints(self, accel_degss):
        """Changes the robot joint acceleration (in deg/s2)"""
        self.ACCEL_DEGSS = accel_degss
        
    def setZoneData(self, zone_mm):
        """Changes the zone data approach (makes the movement more smooth)"""
        self.SMOOTH = zone_mm
        
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
            code.replace(' ','_')
            if not code.endswith(')'):
                code = code + '()'
            #call prog:start()
            #self.addline('call prog:%s' % code)# add as a call
            self.addline('//call prog:%s' % code)# add as a comment
        else:
            self.addline(code)
        
    def RunMessage(self, message, iscomment = False):
        """Display a message in the robot controller screen (teach pendant)"""
        if iscomment:
            self.addline('// ' + message)
        else:
            self.addline('popUpMsg("%s")' % message)
        
# ------------------ private ----------------------                
    def addline(self, newline):
        """Add a program line"""
        self.PROG_PGX = self.PROG_PGX + self.TAB_PGX + newline + '\n'
        
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
    robot.ProgFinish("Program")
    # robot.ProgSave(".","Program",True)
    print(robot.PROG_PGX)
    if len(robot.LOG) > 0:
        mbox('Program generation LOG:\n\n' + robot.LOG)

    input("Press Enter to close...")

if __name__ == "__main__":
    """Function to call when the module is executed by itself: test"""
    test_post()
