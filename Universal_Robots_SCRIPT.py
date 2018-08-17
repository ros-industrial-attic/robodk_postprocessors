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
# for a Universal Robot with RoboDK
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


SCRIPT_URP = '''<URProgram createdIn="3.0.0" lastSavedIn="3.0.0" name="%s" directory="/" installation="default">
  <children>
    <MainProgram runOnlyOnce="false" motionType="MoveJ" speed="1.0471975511965976" acceleration="1.3962634015954636" useActiveTCP="false">
      <children>
        <Script type="File">
          <cachedContents>%s
</cachedContents>
          <file resolves-to="file">%s</file>
        </Script>
      </children>
    </MainProgram>
  </children>
</URProgram>'''


def get_safe_name(progname):
    """Get a safe program name"""
    for c in r'-[]/\;,><&*:%=+@!#^()|?^':
        progname = progname.replace(c,'')
    if len(progname) <= 0:
        progname = 'Program'
    if progname[0].isdigit():
        progname = 'P' + progname    
    return progname


# ----------------------------------------------------
# Import RoboDK tools
from robodk import *

# ----------------------------------------------------

def pose_2_ur(pose):
    """Calculate the p[x,y,z,rx,ry,rz] position for a pose target"""
    def saturate_1(value):
        return min(max(value,-1.0),1.0)
        
    angle = acos(  saturate_1((pose[0,0]+pose[1,1]+pose[2,2]-1)/2)   )    
    rxyz = [pose[2,1]-pose[1,2], pose[0,2]-pose[2,0], pose[1,0]-pose[0,1]]

    if angle == 0:
        rxyz = [0,0,0]
    else:
        rxyz = normalize3(rxyz)
        rxyz = mult3(rxyz, angle)
    return [pose[0,3], pose[1,3], pose[2,3], rxyz[0], rxyz[1], rxyz[2]]

def pose_2_str(pose):
    """Prints a pose target"""
    [x,y,z,w,p,r] = pose_2_ur(pose)
    MM_2_M = 0.001
    return ('p[%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]' % (x*MM_2_M,y*MM_2_M,z*MM_2_M,w,p,r))
    
def angles_2_str(angles):
    """Prints a joint target"""
    njoints = len(angles)
    d2r = pi/180.0
    if njoints == 6:
        return ('[%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]' % (angles[0]*d2r, angles[1]*d2r, angles[2]*d2r, angles[3]*d2r, angles[4]*d2r, angles[5]*d2r))
    else:
        return 'this post only supports 6 joints'

# ----------------------------------------------------    
# Object class that handles the robot instructions/syntax
class RobotPost(object):
    """Robot post object"""
    PROG_EXT = 'script'        # set the program extension
    SPEED_MS       = 0.3    # default speed for linear moves in m/s
    SPEED_RADS     = 0.75   # default speed for joint moves in rad/s
    ACCEL_MSS      = 3      # default acceleration for lineaer moves in m/ss
    ACCEL_RADSS    = 1.2    # default acceleration for joint moves in rad/ss
    BLEND_RADIUS_M = 0.001  # default blend radius in meters (corners smoothing)
    REF_FRAME      = eye(4) # default reference frame (the robot reference frame)
    LAST_POS = None # last XYZ position
    
    # other variables
    ROBOT_POST = 'unset'
    ROBOT_NAME = 'generic'
    PROG_FILES = []
    MAIN_PROGNAME = 'unknown'
    
    nPROGS = 0
    PROG = []
    SUBPROG = []
    TAB = ''
    LOG = ''    
    
    def __init__(self, robotpost=None, robotname=None, robot_axes = 6, **kwargs):
        self.ROBOT_POST = robotpost
        self.ROBOT_NAME = robotname
        
    def ProgStart(self, progname):
        progname = get_safe_name(progname)
        self.nPROGS = self.nPROGS + 1
        if self.nPROGS <= 1:
            self.TAB = '  '
            self.MAIN_PROGNAME = progname    
        else:
            self.addline('  # Subprogram %s' % progname)   
            self.addline('  def %s():' % progname)
            self.TAB = '    '         
        
    def ProgFinish(self, progname):
        progname = get_safe_name(progname)
        self.TAB = ''
        if self.nPROGS <= 1:
            self.addline('  # End of main program')
            self.addline('end')            
        else:
            self.addline('  end')
            
        self.addline('')
                
    def ProgSave(self, folder, progname, ask_user = False, show_result = False):
        progname = get_safe_name(progname)
        self.PROG.append('%s()' % self.MAIN_PROGNAME)
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
        fid.write('def %s():\n' % self.MAIN_PROGNAME)
        fid.write('  # Default parameters:\n')
        fid.write('  global speed_ms    = %.3f\n' % self.SPEED_MS)
        fid.write('  global speed_rads  = %.3f\n' % self.SPEED_RADS)    
        fid.write('  global accel_mss   = %.3f\n' % self.ACCEL_MSS)
        fid.write('  global accel_radss = %.3f\n' % self.ACCEL_RADSS)
        fid.write('  global blend_radius_m = %.3f\n' % self.BLEND_RADIUS_M)
        fid.write('  \n')
        fid.write('  # Add any suprograms here\n')
        for line in self.SUBPROG:
            fid.write(line)

        fid.write('  # Main program:\n')
        for line in self.PROG:
            fid.write(line)
            
        fid.close()
        print('SAVED: %s\n' % filesave) # tell RoboDK the path of the saved file
        self.PROG_FILES = filesave
        
        #---------------------------- SAVE URP (GZIP compressed XML file)-------------------------
        #filesave_urp = filesave[:-8]+'.urp'
        #fid = open(filesave, "r")        
        #prog_final = fid.read()
        #fid.close()
        #self.PROG_XML = SCRIPT_URP % (self.MAIN_PROGNAME, prog_final, self.MAIN_PROGNAME+'.script')
        #
        ### Comment next line to force transfer of the SCRIPT file
        #self.PROG_FILES = filesave_urp
        #
        #import gzip
        #with gzip.open(filesave_urp, 'wb') as fid_gz:
        #    fid_gz.write(self.PROG_XML.encode('utf-8'))
        #    
        #print('SAVED: %s\n' % filesave_urp) # tell RoboDK the path of the saved file
        #------------------------------------------------------------------------------------------
        
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
    
    def ProgSendRobot(self, robot_ip, remote_path, ftp_user, ftp_pass):
        """Send a program to the robot using the provided parameters. This method is executed right after ProgSave if we selected the option "Send Program to Robot".
        The connection parameters must be provided in the robot connection menu of RoboDK"""
        #UploadFTP(self.PROG_FILES, robot_ip, remote_path, ftp_user, ftp_pass)
        #return
        
        with open(self.PROG_FILES, 'rb') as progfile:
            import socket
            print("POPUP: Connecting to robot...")
            sys.stdout.flush()
            robot_socket = socket.create_connection((robot_ip, 30002))
            print("POPUP: Sending program")
            sys.stdout.flush()
            robot_socket.send(progfile.read())
            print("POPUP: Sending program...")
            sys.stdout.flush()            
            pause(1)
            received = robot_socket.recv(4096)
            robot_socket.close()
                            
            if received:
                #print("POPUP: Program running")
                print(str(received))
                sys.stdout.flush()
                idx_error = -1
                try:
                    idx_error = received.index(b'error')
                except:
                    pass
                if idx_error >= 0:
                    idx_error_end = min(idx_error + 20, len(received))
                    try:
                        idx_error_end = received.index(b'\0',idx_error)
                    except:
                        pass
                    print("POPUP: Robot response: <strong>" + received[idx_error:idx_error_end].decode("utf-8") + "</strong>")
                    sys.stdout.flush()
                    pause(5)
                else:
                    print("POPUP: Program sent. The program should be running on the robot.")
                    sys.stdout.flush()
            else:
                print("POPUP: Problems running program...")
                sys.stdout.flush()
            
        pause(2)
    
    def blend_radius_check(self, pose):
        # check that the blend radius covers 40% of the move (at most)
        blend_radius = 'blend_radius_m';
        current_pos = pose.Pos()
        if self.LAST_POS is None:
            blend_radius = '0'
        else:            
            distance = norm(subs3(self.LAST_POS, current_pos)) # in mm
            if 0.4*distance < self.BLEND_RADIUS_M*1000:
                blend_radius = '%.3f' % (max(0.4*distance*0.001, 0.001))
        self.LAST_POS = current_pos
        return blend_radius
        
    def MoveJ(self, pose, joints, conf_RLF=None):
        """Add a joint movement"""
        if pose is None:
            blend_radius = "0"
            self.LAST_POS = None
        else:
            blend_radius = self.blend_radius_check(pose)
        self.addline('movej(%s,accel_radss,speed_rads,0,%s)' % (angles_2_str(joints), blend_radius))
        
    def MoveL(self, pose, joints, conf_RLF=None):
        """Add a linear movement"""
        # Movement in joint space or Cartesian space should give the same result:
        # pose_wrt_base = self.REF_FRAME*pose
        # self.addline('movel(%s,accel_mss,speed_ms,0,blend_radius_m)' % (pose_2_str(pose_wrt_base)))
        if pose is None:
            blend_radius = "0"
            self.LAST_POS = None
            target = angles_2_str(joints)
        else:
            blend_radius = self.blend_radius_check(pose)
            target = pose_2_str(self.REF_FRAME*pose)
        #self.addline('movel(%s,accel_mss,speed_ms,0,%s)' % (angles_2_str(joints), blend_radius))
        self.addline('movel(%s,accel_mss,speed_ms,0,%s)' % (target, blend_radius))
        
        
    def MoveC(self, pose1, joints1, pose2, joints2, conf_RLF_1=None, conf_RLF_2=None):
        """Add a circular movement"""
        def circle_radius(p0,p1,p2):
            a = norm(subs3(p0,p1))
            b = norm(subs3(p1,p2))
            c = norm(subs3(p2,p0))
            radius = a*b*c/sqrt(pow(a*a+b*b+c*c,2)-2*(pow(a,4)+pow(b,4)+pow(c,4)))            
            return radius
            
        def distance_p1_p02(p0,p1,p2):
            v01 = subs3(p1, p0)
            v02 = subs3(p2, p0)
            return dot(v02,v01)/dot(v02,v02)
            
        p0 = self.LAST_POS
        p1 = pose1.Pos()
        p2 = pose2.Pos()
        if p0 is None:
            self.MoveL(pose2, joints2, conf_RLF_2)
            return
            
        radius = circle_radius(p0, p1, p2)        
        if radius < 1 or radius > 100000:
            self.MoveL(pose2, joints2, conf_RLF_2)
            return
        
        d_p1_p02 = distance_p1_p02(p0, p1, p2)
        if d_p1_p02 < 1:
            self.MoveL(pose2, joints2, conf_RLF_2)
            return      
            
        blend_radius = self.blend_radius_check(pose1)
        self.LAST_POS = pose2.Pos()
        #self.addline('movec(%s,%s,accel_mss,speed_ms,%s)' % (angles_2_str(joints1),angles_2_str(joints2), blend_radius))
        self.addline('movec(%s,%s,accel_mss,speed_ms,%s)' % (pose_2_str(self.REF_FRAME*pose1),pose_2_str(self.REF_FRAME*pose2), blend_radius))
        
    def setFrame(self, pose, frame_id=None, frame_name=None):
        """Change the robot reference frame"""
        # the reference frame is not needed if we use joint space for joint and linear movements
        # the reference frame is also not needed if we use cartesian moves with respect to the robot base frame
        # the cartesian targets must be pre-multiplied by the active reference frame
        self.REF_FRAME = pose    
        self.addline('# set_reference(%s)' % pose_2_str(pose))
        
    def setTool(self, pose, tool_id=None, tool_name=None):
        """Change the robot TCP"""
        self.addline('set_tcp(%s)' % pose_2_str(pose))
        
    def Pause(self, time_ms):
        """Pause the robot program"""
        if time_ms <= 0:
            self.addline('halt() # reimplement this function to force stop')
        else:
            self.addline('sleep(%.3f)' % (time_ms*0.001))
        
    def setSpeed(self, speed_mms):
        """Changes the robot speed (in mm/s)"""
        self.SPEED_MS = speed_mms/1000.0
        self.addline('speed_ms    = %.3f' % self.SPEED_MS)
        
    def setAcceleration(self, accel_mmss):
        """Changes the robot acceleration (in mm/s2)"""    
        self.ACCEL_MSS = accel_mmss/1000.0
        self.addline('accel_mss   = %.3f' % self.ACCEL_MSS)
        
    def setSpeedJoints(self, speed_degs):
        """Changes the robot joint speed (in deg/s)"""
        self.SPEED_RADS = speed_degs*pi/180
        self.addline('speed_rads  = %.3f' % self.SPEED_RADS)
    
    def setAccelerationJoints(self, accel_degss):
        """Changes the robot joint acceleration (in deg/s2)"""
        self.ACCEL_RADSS = accel_degss*pi/180
        self.addline('accel_radss = %.3f' % self.ACCEL_RADSS)
        
    def setZoneData(self, zone_mm):
        """Changes the zone data approach (makes the movement more smooth)"""
        if zone_mm < 0:
            zone_mm = 0            
        self.BLEND_RADIUS_M = zone_mm / 1000.0
        self.addline('blend_radius_m = %.3f' % self.BLEND_RADIUS_M)
        
    def setDO(self, io_var, io_value):
        """Sets a variable (output) to a given value"""
        if type(io_value) != str: # set default variable value if io_value is a number            
            if io_value > 0:
                io_value = 'True'
            else:
                io_value = 'False'
        
        if type(io_var) != str:  # set default variable name if io_var is a number
            newline = 'set_standard_digital_out(%s, %s)' % (str(io_var), io_value)
        else:
            newline = '%s = %s' % (io_var, io_value)
            
        self.addline(newline)
        
    def waitDI(self, io_var, io_value, timeout_ms=-1):
        """Waits for an input io_var to attain a given value io_value. Optionally, a timeout can be provided."""
        if type(io_var) != str:  # set default variable name if io_var is a number
            io_var = 'get_standard_digital_in(%s)' % str(io_var)        
        if type(io_value) != str: # set default variable value if io_value is a number            
            if io_value > 0:
                io_value = 'True'
            else:
                io_value = 'False'
        
        # at this point, io_var and io_value must be string values
        #if timeout_ms < 0:
        self.addline('while (%s != %s):' % (io_var, io_value))
        self.addline('  sync()')
        self.addline('end')
        
    def RunCode(self, code, is_function_call = False):
        """Adds code or a function call"""
        if is_function_call:
            code.replace(' ','_')
            if not code.endswith(')'):
                code = code + '()'
            self.addline(code)
        else:
            #self.addline(code)
            self.addline('# ' + code) #generate custom code as a comment
        
    def RunMessage(self, message, iscomment = False):
        """Show a message on the controller screen"""
        if iscomment:
            self.addline('# ' + message)
        else:
            self.addline('popup("%s","Message",False,False,blocking=True)' % message)
        
# ------------------ private ----------------------                
    def addline(self, newline):
        """Add a program line"""
        if self.nPROGS <= 1:
            self.PROG.append(self.TAB + newline + '\n')
        else:
            self.SUBPROG.append(self.TAB + newline + '\n')
        
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

    robot = RobotPost('Universal Robotics', 'Generic UR robot')

    robot.ProgStart("Program")
    robot.RunMessage("Program generated by RoboDK", True)
    robot.setFrame(Pose([807.766544, -963.699898, 41.478944, 0, 0, 0]))
    robot.setTool(Pose([62.5, -108.253175, 100, -60, 90, 0]))
    robot.setSpeed(100) # set speed to 100 mm/s
    robot.setAcceleration(3000) # set speed to 3000 mm/ss    
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
    for line in robot.PROG:
        print(line[:-1])
    if len(robot.LOG) > 0:
        mbox('Program generation LOG:\n\n' + robot.LOG)

    input("Press Enter to close...")

if __name__ == "__main__":
    """Function to call when the module is executed by itself: test"""
    test_post()
