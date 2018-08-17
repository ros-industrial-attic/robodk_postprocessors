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
# for a Dobot Magician robot with RoboDK
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
from robolink import *
import os

# Get 32 bit Python path. IMPORTANT! It must be 32 bit Python (not 64)
PYTHON_PATH = os.path.abspath(os.getenv('APPDATA') + '/../Local/Programs/Python/Python36-32/python.exe')

# Get Dobot DLL path based on Desktop path
DOBOT_PATH_DLL = os.path.abspath(os.getenv('HOMEPATH') + '/Desktop/DobotAPI/')

PROGRAM_HEADER = '''# Dobot program generated from RoboDK post processor
import threading
import DobotDllType as dType
import sys

CON_STR = {
    dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
    dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
    dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"}

#Load Dll
api = dType.load()

#Connect Dobot
state = dType.ConnectDobot(api, "", 115200)[0]
print("Connect status:",CON_STR[state])
sys.stdout.flush()

if (state != dType.DobotConnect.DobotConnect_NoError):
    #Disconnect Dobot
    dType.DisconnectDobot(api)
    raise Exception("Connection problems: " + CON_STR[state])
    quit()

#Clean Command Queued
dType.SetQueuedCmdClear(api)

#Async Motion Params Setting
dType.SetHOMEParams(api, 200, 200, 200, 200, isQueued = 1)
dType.SetPTPJointParams(api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued = 1)
dType.SetPTPCommonParams(api, 100, 100, isQueued = 1)

#Async Home
dType.SetHOMECmd(api, temp = 0, isQueued = 1)

# Main program definition
'''

PROGRAM_RUN = '''
    # Save the index command of the last command (set a short pause)
    lastIndex = dType.SetWAITCmd(api, 0.1, isQueued = 1)[0]
    
    # Start to Execute Queued Commands
    dType.SetQueuedCmdStartExec(api)
    
    # Wait until execution is done by verifying current command
    while lastIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:
        dType.dSleep(100)
'''

ROBOT_DISCONNECT = '''
#Disconnect Dobot
dType.DisconnectDobot(api)
'''

# ----------------------------------------------------
def pose_2_str(pose):
    """Prints a pose target"""
    [x,y,z,r,p,w] = Pose_2_TxyzRxyz(pose)
    #return ('%.3f, %.3f, %.3f, %.3f, %.3f, %.3f' % (x,y,z,r*180.0/pi,p*180.0/pi,w*180.0/pi))
    return ('%.3f, %.3f, %.3f, %.3f' % (x,y,z, w*180.0/pi))
    
def joints_2_str(joints):
    """Prints a joint target"""
    str = ''
    for i in range(len(joints)):
        str = str + ('%.6f, ' % joints[i])
    str = str[:-2]
    return str

# ----------------------------------------------------    
# Object class that handles the robot instructions/syntax
class RobotPost(object):
    """Robot post object"""
    PROG_EXT = 'py'        # set the program extension    
    ROBOT_IP = '192.168.0.100';
    ROBOT_PORT = 10000
    
    # other variables
    ROBOT_POST = ''
    ROBOT_NAME = ''
    PROG_FILES = []    
    
    FRAME = eye(4)
    TOOL = eye(4)
    
    PROG = ''
    LOG = ''
    TAB = ''
    MAIN_PROG_NAME = None
    nAxes = 6
    Prog_count = 0

    
    def __init__(self, robotpost=None, robotname=None, robot_axes = 6, **kwargs):
        self.ROBOT_POST = robotpost
        self.ROBOT_NAME = robotname
        self.PROG = ''
        self.LOG = ''
        self.nAxes = robot_axes
        
    def ProgStart(self, progname):
        self.Prog_count = self.Prog_count + 1
        self.addline('def %s():' % progname)
        self.TAB = '    '
        if self.MAIN_PROG_NAME is None:
            self.MAIN_PROG_NAME = progname
            self.addline("'''Main procedure'''")
        else:
            self.addline("'''Subprogram %s'''" % progname)
        self.addline("print('Sending program %s ...')" % progname)
        
    def ProgFinish(self, progname):
        #if self.Prog_count == 1:
            #self.PROG.append(PROGRAM_DISCONNECT)
            #pass
        self.addline("print('Program %s sent')" % progname)
        self.addline("sys.stdout.flush()")
        self.addline("print('Running program %s on robot...')" % progname)
        self.addline("sys.stdout.flush()")        
        self.PROG = self.PROG + PROGRAM_RUN
        self.addline("")
        self.addline("# Stop executing commands")
        self.addline("dType.SetQueuedCmdStopExec(api)")
        self.addline("# Clear all queued commands")
        self.addline("dType.SetQueuedCmdClear(api)")
        self.addline("print('Program %s Finished')" % progname)
        self.addline("sys.stdout.flush()")
        self.TAB = ''
        self.addline('')        
        
        
    def ProgSave(self, folder, progname, ask_user=False, show_result=False):
        import subprocess
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
        
        fid.write(PROGRAM_HEADER)
        fid.write(self.PROG)
        fid.write('\n')
        fid.write('# Main program call: set as a loop\n')
        fid.write('while True:\n')        
        fid.write('    %s()\n' % self.MAIN_PROG_NAME)
        fid.write('    break\n')        
        fid.write(ROBOT_DISCONNECT)
        fid.write('\n\n')
        fid.close()
        print('SAVED: %s\n' % filesave)
        self.PROG_FILES = filesave
        #---------------------- show result
        
        #selection_view = mbox('Do you want to view/edit the program or run it on the robot?', 'View', 'Run')            
        selection_view = True
        
        if selection_view and show_result:
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
        
        if not selection_view:
            #RDK.ShowMessage('Running program...', False)
            proc = subprocess.Popen(['python', filesave])
            
            # Using the same pipe
            #import io
            #proc = subprocess.Popen(['python', filesave], stdout=subprocess.PIPE)
            #for line in io.TextIOWrapper(proc.stdout, encoding="utf-8"):  # or another encoding
            #    RDK.ShowMessage(line, False)
        
        
    def ProgSendRobot(self, robot_ip, remote_path, ftp_user, ftp_pass):
        """Send a program to the robot using the provided parameters. This method is executed right after ProgSave if we selected the option "Send Program to Robot".
        The connection parameters must be provided in the robot connection menu of RoboDK"""
        # retrieve robot IP
        #RDK = Robolink()
        #robot = RDK.Item(self.ROBOT_NAME, ITEM_TYPE_ROBOT)
        #[server_ip, port, remote_path, username, password] = robot.ConnectionParams()    
        #RDK.ShowMessage('Running program...', False)
        import subprocess
        import os
        import sys
        
        process_env = os.environ.copy()
        process_env["PYTHONPATH"] = DOBOT_PATH_DLL + ";"
        process_env["PATH"] = DOBOT_PATH_DLL + ":" + process_env["PATH"]
        
        print("POPUP: Starting process...")
        PROGRAM_FILE = os.path.abspath(self.PROG_FILES)
        print("Python path: " + PYTHON_PATH)
        print("Program file: " + PROGRAM_FILE)
        sys.stdout.flush()
        
        # Start process
        process = subprocess.Popen([PYTHON_PATH, PROGRAM_FILE], shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=process_env, universal_newlines=True, cwd=DOBOT_PATH_DLL)
        
        # Poll process for new output until finished
        for stdout_line in iter(process.stdout.readline, ""):
            print("POPUP: " + stdout_line.strip())
            sys.stdout.flush()
            
        process.stdout.close()
        return_code = process.wait()
        if return_code:
            raise subprocess.CalledProcessError(return_code, [PYTHON_PATH, PROGRAM_FILE])

        if (return_code == 0):
            import time
            time.sleep(2)
            return
        else:
            raise ProcessException(command, return_code, output)
        
    def MoveJ(self, pose, joints, conf_RLF=None):
        """Add a joint movement"""
        # Prioritize a joint move using joint data
        if joints is not None:
            self.addline("dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJANGLEMode, %s, isQueued=1)" % joints_2_str(joints))
        else:
            robot_pose = self.FRAME*pose*invH(self.TOOL)
            self.addline("dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, %s, isQueued=1)" % pose_2_str(robot_pose))
        
    def MoveL(self, pose, joints, conf_RLF=None):
        """Add a linear movement"""
        # Prioritize a linear move using joint data
        if joints is not None:
            self.addline("dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJANGLEMode, %s, isQueued=1)" % joints_2_str(joints))
        else:
            robot_pose = self.FRAME*pose*invH(self.TOOL)
            self.addline("dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, %s, isQueued=1)" % pose_2_str(robot_pose))
        
    def MoveC(self, pose1, joints1, pose2, joints2, conf_RLF_1, conf_RLF_2):
        """Add a circular movement"""
        robot_pose1 = self.FRAME*pose1*invH(self.TOOL)
        robot_pose2 = self.FRAME*pose2*invH(self.TOOL)        
        self.addline('dType.SetARCCmd(api, (%s), (%s),  isQueued=1)' %(pose_2_str(robot_pose1), pose_2_str(robot_pose2)))
        
    def setFrame(self, pose, frame_id=None, frame_name=None):
        """Change the robot reference frame"""
        self.FRAME = pose
        
    def setTool(self, pose, tool_id=None, tool_name=None):
        """Change the robot TCP"""
        self.TOOL = pose
        
    def Pause(self, time_ms):
        """Pause the robot program"""
        if time_ms < 0:
            self.addline("dType.SetWAITCmd(api, 0.1, 1)")
            mbox("Robot paused, press OK to continue")
        else:
            self.addline("dType.SetWAITCmd(api, %s, 1)" % (time_ms * 0.001))
    
    def setSpeed(self, speed_mms):
        """Changes the robot speed (in mm/s)"""
        #self.addline("robot.Run('SetCartVel', %.3f)" % speed_mms)
        pass
    
    def setAcceleration(self, accel_mmss):
        """Changes the robot acceleration (in mm/s2)"""
        self.addlog('Linear acceleration not supported')
    
    def setSpeedJoints(self, speed_degs):
        """Changes the robot joint speed (in deg/s)"""
        #self.addline("robot.Run('SetJointVel', %.3f)" % speed_degs)
        pass
    
    def setAccelerationJoints(self, accel_degss):
        """Changes the robot joint acceleration (in deg/s2)"""
        #self.addlog('Joint acceleration not supported')
        pass
        
    def setZoneData(self, zone_mm):
        """Changes the smoothing radius (aka CNT, APO or zone data) to make the movement smoother"""
        #if zone_mm > 0:
        #    self.addline("robot.Run('SetCornering', 1)")
        #else:
        #    self.addline("robot.Run('SetCornering', 0)")
        pass

    def setDO(self, io_var, io_value):
        """Sets a variable (digital output) to a given value"""
        if type(io_var) != str:  # set default variable name if io_var is a number
            io_var = 'OUT[%s]' % str(io_var)
        if type(io_value) != str: # set default variable value if io_value is a number
            if io_value > 0:
                io_value = 'TRUE'
            else:
                io_value = 'FALSE'

        # at this point, io_var and io_value must be string values
        # self.addline('%s=%s' % (io_var, io_value))
        self.addlog('Digital IOs not managed by the robot (%s=%s)' % (io_var, io_value))

    def waitDI(self, io_var, io_value, timeout_ms=-1):
        """Waits for a variable (digital input) io_var to attain a given value io_value. Optionally, a timeout can be provided."""
        if type(io_var) != str:  # set default variable name if io_var is a number
            io_var = 'IN[%s]' % str(io_var)
        if type(io_value) != str: # set default variable value if io_value is a number
            if io_value > 0:
                io_value = 'TRUE'
            else:
                io_value = 'FALSE'

        # at this point, io_var and io_value must be string values
        if timeout_ms < 0:
            #self.addline('WAIT FOR %s==%s' % (io_var, io_value))
            self.addlog('Digital IOs not managed by the robot (WAIT FOR %s==%s)' % (io_var, io_value))
        else:
            #self.addline('WAIT FOR %s==%s TIMEOUT=%.1f' % (io_var, io_value, timeout_ms))
            self.addlog('Digital IOs not managed by the robot (WAIT FOR %s==%s)' % (io_var, io_value))
        
    def RunCode(self, code, is_function_call = False):
        """Adds code or a function call"""
        if is_function_call:
            code.replace(' ','_')
            if not code.endswith(')'):
                code = code + '()'
            self.addline(code)
        else:
            self.addline(code)
        
    def RunMessage(self, message, iscomment = False):
        """Display a message in the robot controller screen (teach pendant)"""
        if iscomment:
            self.addline('# ' + message)
        else:
            self.addline('print("%s")' % message)
        
# ------------------ private ----------------------                
    def addline(self, newline):
        """Add a program line"""
        self.PROG = self.PROG + self.TAB + newline + '\n'
        
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
