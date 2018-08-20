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

DEFAULT_HEADER_SCRIPT = """
  #--------------------------
  # Add any default subprograms here
  # For example, to drive a gripper as a program call:
  # def Gripper_Open():
  #   ...
  # end
  #
  # Example to drive a spray gun:
  def SprayOn(value):
    # use the value as an output:
    DO_SPRAY = 5
    if value == 0:
      set_standard_digital_out(DO_SPRAY, False)
    else:
      set_standard_digital_out(DO_SPRAY, True)
    end
  end

  # Example to synchronize 2
  def Synchronize():
    # Use the following digital output to signal the state of the robot:
    DO_SYNC = 1
    
    # Use the following digital input to get the state of another robot:
    DI_SYNC = 1
    
    if (get_standard_digital_out(DO_SYNC) == get_standard_digital_in(DI_SYNC)):
      set_standard_digital_out(DO_SYNC, not (get_standard_digital_out(DI_SYNC)))
      sleep(0.1)
      thread Thread_wait_1():
        while (True):
          sleep(0.01)
        end
      end
      if (get_standard_digital_out(DO_SYNC) != get_standard_digital_in(DI_SYNC)):
        global thread_handler_1=run Thread_wait_1()
        while (get_standard_digital_out(DO_SYNC) != get_standard_digital_in(DI_SYNC)):
          sync()
        end
        kill thread_handler_1
      end
    else:
      if (get_standard_digital_out(DO_SYNC) != get_standard_digital_in(DI_SYNC)):
        set_standard_digital_out(DO_SYNC, not (get_standard_digital_out(DO_SYNC)))
      end
    end
  end

  #
  # Example to move an external axis
  def MoveAxis(value):
    # use the value as an output:
    DO_AXIS_1 = 1
    DI_AXIS_1 = 1
    if value <= 0:
      set_standard_digital_out(DO_AXIS_1, False)
      
      # Wait for digital input to change state
      #while (get_standard_digital_in(DI_AXIS_1) != False):
      #  sync()
      #end
    else:
      set_standard_digital_out(DO_AXIS_1, True)
      
      # Wait for digital input to change state
      #while (get_standard_digital_in(DI_AXIS_1) != True):
      #  sync()
      #end
    end
  end
  #--------------------------
"""

#SCRIPT_URP = '''<URProgram name="%s">
#  <children>
#    <MainProgram runOnlyOnce="false" motionType="MoveJ" speed="1.0471975511965976" acceleration="1.3962634015954636" useActiveTCP="false">
#      <children>
#        <Script type="File">
#          <cachedContents>%s
#</cachedContents>
#          <file resolves-to="file">%s</file>
#        </Script>
#      </children>
#    </MainProgram>
#  </children>
#</URProgram>'''

#SCRIPT_URP = '''<URProgram createdIn="3.0.0" lastSavedIn="3.0.0" name="%s" directory="/" installation="default">
#  <children>
#    <MainProgram runOnlyOnce="true" motionType="MoveJ" speed="1.0471975511965976" acceleration="1.3962634015954636" useActiveTCP="false">
#      <children>
#        <Script type="File">
#          <cachedContents>%s
#</cachedContents>
#          <file resolves-to="file">%s</file>
#        </Script>
#      </children>
#    </MainProgram>
#  </children>
#</URProgram>'''

#<URProgram createdIn="3.4.3.361" lastSavedIn="3.4.3.361" name="%s" directory="." installation="default">
SCRIPT_URP = '''<URProgram createdIn="3.0.0" lastSavedIn="3.0.0" name="%s" directory="." installation="default">
  <children>
    <MainProgram runOnlyOnce="true" motionType="MoveJ" speed="1.0471975511965976" acceleration="1.3962634015954636" useActiveTCP="false">
      <children>
%s      </children>
    </MainProgram>
%s  </children>
</URProgram>'''

def get_safe_name(progname):
    """Get a safe program name"""
    for c in r'-[]/\;,><&*:%=+@!#^|?^':
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
import socket
import struct
# UR information for real time control and monitoring
# Byte shifts for the real time packet:
UR_GET_RUNTIME_MODE = 132*8-4

RUNTIME_CANCELLED = 0
RUNTIME_READY = 1
RUNTIME_BUSY = 2

RUNTIME_MODE_MSG = []
RUNTIME_MODE_MSG.append("Operation cancelled") #0
RUNTIME_MODE_MSG.append("Ready") #1
RUNTIME_MODE_MSG.append("Running") #2 # Running or Jogging

# Get packet size according to the byte array
def UR_packet_size(buf):
    if len(buf) < 4:
        return 0
    return struct.unpack_from("!i", buf, 0)[0]
   
# Check if a packet is complete
def UR_packet_check(buf):
    msg_sz = UR_packet_size(buf)
    if len(buf) < msg_sz:
        print("Incorrect packet size %i vs %i" % (msg_sz, len(buf)))
        return False
    
    return True

# Get specific information from a packet
def UR_packet_value(buf, offset, nval=6):
    if len(buf) < offset+nval:
        print("Not available offset (maybe older Polyscope version?): %i - %i" % (len(buf), offset))
        return None
    format = '!'
    for i in range(nval):
        format+='d'
    return list(struct.unpack_from(format, buf, offset))    #return list(struct.unpack_from("!dddddd", buf, offset))


ROBOT_PROGRAM_ERROR = -1
ROBOT_NOT_CONNECTED = 0
ROBOT_OK = 1

def UR_SendProgramRobot(robot_ip, data):
    print("POPUP: Connecting to robot...")
    sys.stdout.flush()
    robot_socket = socket.create_connection((robot_ip, 30002))
    print("POPUP: Sending program..")
    sys.stdout.flush()
    robot_socket.send(data)
    print("POPUP: Sending program...")
    sys.stdout.flush()            
    pause(1)
    received = robot_socket.recv(4096)
    robot_socket.close()
                    
    if received:
        #print("POPUP: Program running")
        #print(str(received))
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
            return ROBOT_PROGRAM_ERROR
        else:
            print("POPUP: Program sent. The program should be running on the robot.")
            sys.stdout.flush()
            return ROBOT_OK
    else:
        print("POPUP: Robot connection problems...")
        sys.stdout.flush()
        pause(2)
        return ROBOT_NOT_CONNECTED        

# Monitor thread to retrieve information from the robot
def UR_Wait_Ready(robot_ip, percent_cmpl):
    RUNTIME_MODE_LAST = -1
    while True:
        print("Connecting to robot %s:%i" % (robot_ip, 30003))
        rt_socket = socket.create_connection((robot_ip, 30003))
        print("Connected")
        buf = b''
        while True:
            more = rt_socket.recv(4096)
            if more:
                buf = buf + more
                if UR_packet_check(buf):
                    packet_len = UR_packet_size(buf) 
                    packet, buf = buf[:packet_len], buf[packet_len:]
                    RUNTIME_MODE = round(UR_packet_value(packet, UR_GET_RUNTIME_MODE, 1)[0])
                    if RUNTIME_MODE_LAST != RUNTIME_MODE:
                        RUNTIME_MODE_LAST = RUNTIME_MODE
                        if RUNTIME_MODE < len(RUNTIME_MODE_MSG):
                            print("POPUP: Robot " + RUNTIME_MODE_MSG[RUNTIME_MODE] + " (transfer %.1f%% completed)" % percent_cmpl)
                            sys.stdout.flush()
                        else:
                            print("POPUP: Robot Status Unknown (%.i)" % RUNTIME_MODE + " (transfer %.1f%% completed)" % percent_cmpl)
                            sys.stdout.flush()
                            
                        if RUNTIME_MODE == RUNTIME_READY:
                            rt_socket.close()
                            return True
                        
        rt_socket.close()
        return False


UR_MoveJ = 1
UR_MoveL = 2
UR_MoveP = 3
UR_MoveC = 4

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

def circle_radius(p0,p1,p2):
    a = norm(subs3(p0,p1))
    b = norm(subs3(p1,p2))
    c = norm(subs3(p2,p0))
    radius = a*b*c/sqrt(pow(a*a+b*b+c*c,2)-2*(pow(a,4)+pow(b,4)+pow(c,4)))            
    return radius
    
try:
    from html import escape  # python 3.x
except ImportError:
    from cgi import escape  # python 2.x
    
#def distance_p1_p02(p0,p1,p2):
#    v01 = subs3(p1, p0)
#    v02 = subs3(p2, p0)
#    return dot(v02,v01)/dot(v02,v02)
        
# ----------------------------------------------------    
# Object class that handles the robot instructions/syntax
class RobotPost(object):
    """Robot post object"""
    MAX_LINES_X_PROG = 5000    # Maximum number of lines per program. If the number of lines is exceeded, the program will be executed step by step by RoboDK
    UPLOAD_SFTP = False         # If True, it will attempt to upload using SFTP. It requires PYSFTP (pip install pysftp. Important: It requires Visual Studio Community C++ 10.0)
    PROG_EXT = 'script'        # set the program extension
    SPEED_MS       = 0.250  # default speed for linear moves in m/s
    SPEED_RADS     = 0.75   # default speed for joint moves in rad/s
    ACCEL_MSS      = 1.2    # default acceleration for lineaer moves in m/ss
    ACCEL_RADSS    = 1.2    # default acceleration for joint moves in rad/ss
    BLEND_RADIUS_M = 0.001  # default blend radius in meters (corners smoothing)
    MOVEC_MIN_RADIUS = 1    # minimum circle radius to output (in mm). It does not take into account the Blend radius
    MOVEC_MAX_RADIUS = 10000  # maximum circle radius to output (in mm). It does not take into account the Blend radius
    USE_MOVEP = True    # Set to True to use MoveP, set to False to use MoveL
    #--------------------------------
    REF_FRAME      = eye(4) # default reference frame (the robot reference frame)
    TOOL_FRAME     = eye(4)
    TOOL_FRAME_INV = eye(4)
    LAST_POS_ABS = None # last XYZ position
    
    # other variables
    ROBOT_POST = 'unset'
    ROBOT_NAME = 'generic'
    PROG_FILES = []
    MAIN_PROGNAME = 'unknown'
    
    nPROGS = 0
    PROG = []
    PROG_LIST = []
    PROG_URP = []
    FOOTER_URP = ''
    SubProgCount = 0
    SubProgNames = []
    LAST_MOVETYPE = None
    WaypointCount = 0
    MotionParameters = '<motionParameters/>'
    
    PROG_LIST_URP = []
    VARS = []
    VARS_LIST = []
    SUBPROG = []
    SUBPROG_URP = []
    TAB = ''
    TAB_URP = '' # 2 tabs for URP XML are added by default in the main contents
    LOG = ''    
    
    def __init__(self, robotpost=None, robotname=None, robot_axes = 6, **kwargs):
        self.ROBOT_POST = robotpost
        self.ROBOT_NAME = robotname
        for k,v in kwargs.items():
            if k == 'lines_x_prog':
                self.MAX_LINES_X_PROG = v    
        
    def ProgStart(self, progname):
        progname = get_safe_name(progname)
        self.nPROGS = self.nPROGS + 1
        self.SubProgCount = 0
        self.SubProgNames = []
        if self.nPROGS <= 1:
            self.TAB = ''
            # Create global variables:
            self.vars_update()
            self.MAIN_PROGNAME = progname    
        else:
            self.addline('# Subprogram %s' % progname)   
            self.addline('def %s():' % progname)
            self.TAB = '  '         
        
    def ProgFinish(self, progname):
        self.urp_move_close()
        progname = get_safe_name(progname)
        self.TAB = ''
        if self.nPROGS <= 1:
            self.addline('# End of main program')
        else:
            self.addline('end')
            self.addline('')       
                
    def ProgSave(self, folder, progname, ask_user = False, show_result = False):
        progname = get_safe_name(progname)
        progname = progname + '.script'# + self.PROG_EXT
        if ask_user or not DirExists(folder):
            filesave = getSaveFile(folder, progname, 'Save program as...')
            if filesave is not None:
                filesave = filesave.name
            else:
                return
        else:
            filesave = folder + '/' + progname

        self.prog_2_list()        

        fid = open(filesave, "w")
        # Create main program call:
        fid.write('def %s():\n' % self.MAIN_PROGNAME)

        # Add global parameters:
        fid.write('  # Global parameters:\n')
        for line in self.VARS_LIST[0]:
            fid.write('  ' + line+ '\n')            
        #fid.write('  \n')
        fid.write('  ')

        # Add a custom header if desired:
        fid.write(DEFAULT_HEADER_SCRIPT)        
        fid.write('  \n')

        # Add the suprograms that are being used in RoboDK
        for line in self.SUBPROG:
            fid.write('  ' + line + '\n')            
        fid.write('  \n')

        # Add the main code:
        fid.write('  # Main program:\n')
        for prog in self.PROG_LIST:
            for line in prog:
                fid.write('  ' + line + '\n')

        fid.write('end\n\n')
        fid.write('%s()\n' % self.MAIN_PROGNAME)
            
        fid.close()
    
        print('SAVED: %s\n' % filesave) # tell RoboDK the path of the saved file
        self.PROG_FILES = filesave
        
        #---------------------------- SAVE URP (GZIP compressed XML file)-------------------------
        filesave_urp = filesave[:-7] #+'.urp'
        urpxml = ''
        # Add the suprograms that are being used in RoboDK
        #for line in self.SUBPROG_URP:
        #    urpxml += ('        ' + line + '\n')          

        # Add the main code:
        for line in self.PROG_URP:
            urpxml += ('        ' + line + '\n') 
        self.PROG_XML = SCRIPT_URP % (self.MAIN_PROGNAME, urpxml, self.FOOTER_URP)
        # Comment next line to force transfer of the SCRIPT file
        #self.PROG_FILES = filesave_urp
        
        import gzip
        import os
        with gzip.open(filesave_urp, 'wb') as fid_gz:
            fid_gz.write(self.PROG_XML.encode('utf-8'))
        
        try:
            os.remove(filesave_urp+'.urp')
        except OSError:
            pass
            
        os.rename(filesave_urp, filesave_urp+'.urp')
            
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
                os.startfile(filesave)
            if len(self.LOG) > 0:
                mbox('Program generation LOG:\n\n' + self.LOG)
    
        #if len(self.PROG_LIST) > 1:
        #    mbox("Warning! The program " + progname + " is too long and directly running it on the robot controller might be slow. It is better to run it form RoboDK.")


    def ProgSendRobot(self, robot_ip, remote_path, ftp_user, ftp_pass):
        """Send a program to the robot using the provided parameters. This method is executed right after ProgSave if we selected the option "Send Program to Robot".
        The connection parameters must be provided in the robot connection menu of RoboDK"""
        #UploadFTP(self.PROG_FILES, robot_ip, remote_path, ftp_user, ftp_pass)
        #return
        if self.UPLOAD_SFTP:
            import pysftp
            with pysftp.Connection(robot_ip, username=ftp_user, password=ftp_pass) as sftp:
                with sftp.cd(remote_path):             # temporarily chdir to public
                    if type(self.PROG_FILES) is str:
                        sftp.put(self.PROG_FILES)  # upload file to public/ on remote
                    else:
                        for file in self.PROG_FILES:
                            sftp.put(file)  # upload file to public/ on remote
                    #sftp.get('remote_file')         # get a remote file
        
        else:            
            nprogs = len(self.PROG_LIST)
            for i in range(nprogs):
                # Prepare next program execution:
                send_str = ''
                send_str += ('def %s():\n' % self.MAIN_PROGNAME)

                # Add global parameters:
                send_str += ('  # Global parameters:\n')
                for line in self.VARS_LIST[i]:
                    send_str += ('  ' + line + '\n')            
                send_str += ('  \n')

                # Add a custom header if desired:
                send_str += (DEFAULT_HEADER_SCRIPT)        
                send_str += ('  \n')
         
                for line in self.SUBPROG:
                    send_str += '  ' + line+ '\n'
                send_str += ('  \n')

                # Add the main code:
                send_str += ('  # Main program:\n')

                for line in self.PROG_LIST[i]:
                    send_str += '  ' + line
                    send_str += '\n'
                    
                send_str += 'end\n\n'
                send_str += '%s()\n' % self.MAIN_PROGNAME            

                send_bytes = str.encode(send_str)

                # Wait until the robot is ready:
                while i > 0 and not UR_Wait_Ready(robot_ip, i*100.0/nprogs):
                    print("POPUP: Connect robot to run the program program...")
                    sys.stdout.flush()
                    pause(2)  

                # Send script to the robot:
                #print(send_str)
                #input("POPUP: Enter to continue")
                while ROBOT_NOT_CONNECTED == UR_SendProgramRobot(robot_ip, send_bytes):
                    print("POPUP: Connect robot to transfer program...")
                    sys.stdout.flush()
                    pause(2)            
    
    def blend_radius_check(self, pose_abs, ratio_check=0.5):
        # check that the blend radius covers 40% of the move (at most)
        blend_radius = self.BLEND_RADIUS_M;
        #return blend_radius        
        current_pos = pose_abs.Pos()
        if self.LAST_POS_ABS is None:
            blend_radius = 0
        else:            
            distance = norm(subs3(self.LAST_POS_ABS, current_pos)) # in mm
            if ratio_check*distance < self.BLEND_RADIUS_M*1000:
                blend_radius = round(ratio_check*distance*0.001,3)
        #self.LAST_POS_ABS = current_pos
        return blend_radius
        
    def MoveJ(self, pose, joints, conf_RLF=None):
        """Add a joint movement"""
        if pose is None:
            blend_radius = 0
            self.LAST_POS_ABS = None
        else:
            pose_abs = self.REF_FRAME*pose
            blend_radius = self.blend_radius_check(pose_abs)
            self.LAST_POS_ABS = pose_abs.Pos()
            
        if len(joints) < 6:
            self.RunMessage('Move axes to: ' + angles_2_str(joints))
        #else:
        #    self.addline('movej(%s,accel_radss,speed_rads,0,%s)' % (angles_2_str(joints), blend_radius))
        
        # Optionally, use absolute joint movements (move axes, not pose)
        if pose is None:
            self.RunCode('movej(%s)' % angles_2_str(joints))
        else:
            self.RunCode('#movej(%s)' % angles_2_str(joints))
            self.add_urp_move(UR_MoveJ, pose, joints, conf_RLF, blend_radius)
        
        
    def MoveL(self, pose, joints, conf_RLF=None):
        """Add a linear movement"""
        # Movement in joint space or Cartesian space should give the same result:
        # pose_wrt_base = self.REF_FRAME*pose
        # self.addline('movel(%s,accel_mss,speed_ms,0,blend_radius_m)' % (pose_2_str(pose_wrt_base)))
        if pose is None:
            blend_radius = 0
            self.LAST_POS = None
            target = angles_2_str(joints)
        else:            
            pose_abs = self.REF_FRAME*pose
            blend_radius = self.blend_radius_check(pose_abs)
            target = pose_2_str(pose_abs)
            self.LAST_POS_ABS = pose_abs.Pos()
            
        if self.USE_MOVEP:
            self.addline('movep(%s,accel_mss,speed_ms,%.3f)' % (target, blend_radius))
            self.add_urp_move(UR_MoveP, pose, joints, conf_RLF, blend_radius)
        else:
            self.addline('movel(%s,accel_mss,speed_ms,0,%.3f)' % (target, blend_radius))
            self.add_urp_move(UR_MoveL, pose, joints, conf_RLF, blend_radius)
                
    def MoveC(self, pose1, joints1, pose2, joints2, conf_RLF_1=None, conf_RLF_2=None):
        """Add a circular movement"""            
        pose1_abs = self.REF_FRAME*pose1
        pose2_abs = self.REF_FRAME*pose2        
        p0 = self.LAST_POS_ABS
        p1 = pose1_abs.Pos()
        p2 = pose2_abs.Pos()
        if p0 is None:
            self.MoveL(pose2, joints2, conf_RLF_2)
            return
            
        radius = circle_radius(p0, p1, p2)      
        print("MoveC Radius: " + str(radius) + " mm")		
        if radius < self.MOVEC_MIN_RADIUS or radius > self.MOVEC_MAX_RADIUS:
            self.MoveL(pose2, joints2, conf_RLF_2)
            return
            
        blend_radius = self.blend_radius_check(pose1_abs, 0.2)
        #blend_radius = '%.3f' % (0.001*radius) #'0'
        #blend_radius = '0'
        self.LAST_POS_ABS = pose2_abs.Pos()
        
        self.RunMessage('Circular move (R%.1f mm)' % radius, True)
        self.RunCode('movec(%s,%s)' % (pose_2_str(pose1_abs),pose_2_str(pose2_abs)))
        #self.add_urp_move(UR_MoveL, pose1, joints1, conf_RLF1)
        #self.add_urp_move(UR_MoveL, pose2, joints2, conf_RLF2)
        
        # RunCode will add the movement line
        #self.addline('movec(%s,%s,accel_mss,speed_ms,%s)' % (angles_2_str(joints1),angles_2_str(joints2), blend_radius))
        #self.addline('movec(%s,%s,accel_mss,speed_ms,%s)' % (pose_2_str(pose1_abs),pose_2_str(pose2_abs), blend_radius))
        
    def setFrame(self, pose, frame_id=None, frame_name=None):
        """Change the robot reference frame"""
        # the reference frame is not needed if we use joint space for joint and linear movements
        # the reference frame is also not needed if we use cartesian moves with respect to the robot base frame
        # the cartesian targets must be pre-multiplied by the active reference frame
        self.REF_FRAME = pose  
        ref_pose = pose_2_str(pose) 
        
        self.urp_move_close()
        self.RunMessage('Using Ref. %s: %s' % (frame_name, ref_pose), True)
        self.addline('# set_reference(%s)' % ref_pose)
        
    def setTool(self, pose, tool_id=None, tool_name=None):
        """Change the robot TCP"""
        self.TOOL_FRAME = pose
        self.TOOL_FRAME_INV = invH(pose)
        tcp_pose = pose_2_str(pose)
        
        # Important: RunCode will output SCRIPT and URP
        #self.addline('set_tcp(%s)' % tcp_pose)        
        #self.addline('set_payload(1.4, [-0.1181, -0.1181, 0.03])')
        #self.addline('set_gravity([0.0, 0.0, 9.82]))')   
        
        # Add URP
        # self.urp_move_close()
        
        # Add a comment
        self.RunMessage('Using TCP %s: %s' % (tool_name, tcp_pose), True)
        
        # Add the script to run        
        self.RunCode('set_tcp(%s)' % tcp_pose)
        #self.add_urp('<Set type="NoAction">')
        #self.add_urp('  <tcp referencedName="TCP"/>')
        #self.add_urp('</Set>')    
        
    def Pause(self, time_ms):
        """Pause the robot program"""
        if time_ms <= 0:
            self.addline('halt() # reimplement this function to force stop')
            
            self.urp_move_close()
            self.add_urp('<Halt/>')
        else:
            self.addline('sleep(%.3f)' % (time_ms*0.001))
            
            self.urp_move_close()
            self.add_urp('<Wait type="Sleep">')
            self.add_urp('  <waitTime>%.3f</waitTime>' % (time_ms*0.001))
            self.add_urp('</Wait>')
        
    def setSpeed(self, speed_mms):
        """Changes the robot speed (in mm/s)"""
        #if speed_mms < 999.9:
        #    self.USE_MOVEP = True
        #else:
        #    self.USE_MOVEP = False
        self.SPEED_MS = speed_mms/1000.0
        self.addline('speed_ms    = %.3f' % self.SPEED_MS)
        self.urp_move_close()
        self.update_MotionParameters()
        
    def setAcceleration(self, accel_mmss):
        """Changes the robot acceleration (in mm/s2)"""    
        self.ACCEL_MSS = accel_mmss/1000.0
        self.addline('accel_mss   = %.3f' % self.ACCEL_MSS)
        self.update_MotionParameters()
        
    def setSpeedJoints(self, speed_degs):
        """Changes the robot joint speed (in deg/s)"""
        self.SPEED_RADS = speed_degs*pi/180
        self.addline('speed_rads  = %.3f' % self.SPEED_RADS)
        self.update_MotionParameters()
    
    def setAccelerationJoints(self, accel_degss):
        """Changes the robot joint acceleration (in deg/s2)"""
        self.ACCEL_RADSS = accel_degss*pi/180
        self.addline('accel_radss = %.3f' % self.ACCEL_RADSS)
        self.update_MotionParameters()
        
    def setZoneData(self, zone_mm):
        """Changes the zone data approach (makes the movement more smooth)"""
        if zone_mm < 0:
            zone_mm = 0            
        self.BLEND_RADIUS_M = zone_mm / 1000.0
        self.addline('blend_radius_m = %.3f' % self.BLEND_RADIUS_M)
        
    def setDO(self, io_var, io_value):
        """Sets a variable (output) to a given value"""
        io_var_urp = io_var
        io_value_urp = io_value
        
        if type(io_value) != str: # set default variable value if io_value is a number            
            if io_value > 0:                
                io_value = 'True'
                io_value_urp = '1'
            else:
                io_value = 'False'
                io_value_urp = '0'
        
        if type(io_var) != str:  # set default variable name if io_var is a number
            io_var_urp = 'digital_out[%s]' % str(io_var)
            newline = 'set_standard_digital_out(%s, %s)' % (str(io_var), io_value)
        else:
            newline = '%s = %s' % (io_var, io_value)
            
        self.addline(newline)
        
        self.urp_move_close()
        self.add_urp('<Set type="DigitalOutput">')
        self.add_urp('  <pin referencedName="%s"/>' % io_var_urp)
        self.add_urp('  <digitalValue>%s</digitalValue>' % io_value_urp)
        self.add_urp('</Set>')       
        
    def waitDI(self, io_var, io_value, timeout_ms=-1):
        """Waits for an input io_var to attain a given value io_value. Optionally, a timeout can be provided."""
        io_var_urp = io_var
        io_value_urp = io_value
        
        if type(io_var) != str:  # set default variable name if io_var is a number
            io_var = 'get_standard_digital_in(%s)' % str(io_var)    
            io_var_urp = 'digital_in[%s]' % str(io_var)
            
        if type(io_value) != str: # set default variable value if io_value is a number            
            if io_value > 0:
                io_value = 'True'
                io_value_urp = '1'
            else:
                io_value = 'False'
                io_value_urp = '0'
        
        # at this point, io_var and io_value must be string values
        #if timeout_ms < 0:
        self.addline('while (%s != %s):' % (io_var, io_value))
        self.addline('  sync()')
        self.addline('end')
        
        self.urp_move_close()
        self.add_urp('<Wait type="DigitalInput">')
        self.add_urp('  <pin referencedName="%s"/>' % io_var_urp)
        self.add_urp('  <digitalValue>%s</digitalValue>' % io_value_urp)
        self.add_urp('</Wait>')
        
    def RunCode(self, code, is_function_call = False):
        """Adds code or a function call"""
        if is_function_call:
            code = get_safe_name(code)
            if code.lower() == "usemovel":
                self.setZoneData(0)
                self.USE_MOVEP = False
                return
            elif code.lower() == "usemovep":
                self.USE_MOVEP = True
                return
            
            if not code.endswith(')'):
                code = code + '()'
            
            self.addline(code)
            
            progname = code.split('(')[0]
            self.urp_move_close()
            
            self.SubProgCount += 1
            if not progname in self.SubProgNames:
                self.add_urp('<CallSubProgram>')
                self.add_urp('  <subprogram name="%s" keepHidden="true" keepSynchronizedWithDisk="true" trackProgramExecution="false">' % progname)
                self.add_urp('    <programFile resolves-to="file">../programs/%s.urp</programFile>' % progname)
                self.add_urp('    <children>')
                self.add_urp('    </children>')
                self.add_urp('  </subprogram>')
                self.add_urp('</CallSubProgram>')
                
                self.SubProgNames.append(progname)
                str_count = ''
                if self.SubProgCount > 1:
                    str_count = '[%i]' % self.SubProgCount
                    
                self.FOOTER_URP += '  <SubProgram reference="../MainProgram/children/CallSubProgram%s/subprogram"/>\n' % str_count            
                # <SubProgram reference="../MainProgram/children/CallSubProgram/subprogram"/>
                # <SubProgram reference="../MainProgram/children/CallSubProgram[2]/subprogram"/>
            else:
                id_subprog = self.SubProgNames.index(progname) + 1
                str_subprog = ''
                if id_subprog > 1:
                    str_subprog = '[%i]' % (id_subprog)
                    
                print('ID: ' + str(id_subprog))
                print(self.SubProgNames)
                    
                self.add_urp('<CallSubProgram>')
                self.add_urp('  <subprogram reference="../../CallSubProgram%s/subprogram"/>' % str_subprog)
                self.add_urp('</CallSubProgram>')
            
        else:
            self.addline(code)
            #self.addline('# ' + code) #generate custom code as a comment
            
            self.urp_move_close()
            self.add_urp('<Script type="Line">')
            self.add_urp('  <expression>')
            for c in code:
                self.add_urp('    <ExpressionChar character="%s"/>' % escape(c))
            self.add_urp('  </expression>')
            self.add_urp('</Script>')      
        
    def RunMessage(self, message, iscomment = False):
        """Show a message on the controller screen"""
        if iscomment:
            self.addline('# ' + message)
            
            self.urp_move_close()
            self.add_urp('<Comment comment="%s"/>' % escape(message))            
        else:
            self.addline('popup("%s","Message",False,False,blocking=True)' % message)
            
            self.urp_move_close()
            self.add_urp('<Popup type="Message" haltProgram="false" message="%s"/>' % escape(message))
        
# ------------------ private ----------------------
    def vars_update(self):
        # Generate global variables for this program
        self.VARS = []            
        self.VARS.append('global speed_ms    = %.3f' % self.SPEED_MS)
        self.VARS.append('global speed_rads  = %.3f' % self.SPEED_RADS)
        self.VARS.append('global accel_mss   = %.3f' % self.ACCEL_MSS)
        self.VARS.append('global accel_radss = %.3f' % self.ACCEL_RADSS)
        self.VARS.append('global blend_radius_m = %.3f' % self.BLEND_RADIUS_M)
            
    def prog_2_list(self):
        if len(self.PROG) > 1:
            self.PROG_LIST.append(self.PROG)
            self.PROG = []
            self.VARS_LIST.append(self.VARS)
            self.VARS = []
            self.vars_update()
        
    def addline(self, newline):
        """Add a program line"""
        if self.nPROGS <= 1:
            if len(self.PROG) > self.MAX_LINES_X_PROG:
                self.prog_2_list()
                
            self.PROG.append(self.TAB + newline)
        else:
            self.SUBPROG.append(self.TAB + newline)
            
    def add_urp(self, newline):
        """Add a program line"""
        if self.nPROGS <= 1:                
            self.PROG_URP.append(self.TAB_URP + newline)
        else:
            self.SUBPROG_URP.append(self.TAB_URP + newline)
    
    def update_MotionParameters(self):
        self.MotionParameters = '<motionParameters'
        if self.BLEND_RADIUS_M is not None:
            self.MotionParameters += ' blendRadius="%s"' % self.BLEND_RADIUS_M
        if self.SPEED_RADS is not None:
            self.MotionParameters += ' jointSpeed="%s"' % self.SPEED_RADS
        if self.ACCEL_RADSS is not None:
            self.MotionParameters += ' jointAcceleration="%s"' % self.ACCEL_RADSS
        if self.SPEED_MS is not None:
            self.MotionParameters += ' cartesianSpeed="%s"' % self.SPEED_MS
        if self.ACCEL_MSS is not None:
            self.MotionParameters += ' cartesianAcceleration="%s"' % self.ACCEL_MSS
        self.MotionParameters += '/>'
           
    def add_urp_move(self, movetype, pose, joints, conf_RLF, blend_radius):
        if movetype != self.LAST_MOVETYPE:
            self.urp_move_close()
            
        if self.LAST_MOVETYPE is None:
            if movetype == UR_MoveJ:
                self.add_urp('<Move motionType="MoveJ" speed="%.3f" acceleration="%.3f" useActiveTCP="false">' % (self.SPEED_RADS, self.ACCEL_RADSS))
                self.MotionParameters = None
            elif movetype == UR_MoveL:
                self.add_urp('<Move motionType="MoveL" speed="%.3f" acceleration="%.3f" useActiveTCP="false">' % (self.SPEED_MS, self.ACCEL_MSS))
                self.MotionParameters = None
            elif movetype == UR_MoveP:
                self.add_urp('<Move motionType="MoveP" speed="%.3f" acceleration="%.3f" blendRadius="%.3f" useActiveTCP="false">' % (self.SPEED_MS, self.ACCEL_MSS, self.BLEND_RADIUS_M))
                self.MotionParameters = None
                
            #<Move motionType="MoveL" speed="0.25" acceleration="1.2" useActiveTCP="false">
            #<tcp reference="../../Set/tcp"/>
            #<feature class="GeomFeatureReference" referencedName="Joint_0_name"/>
            
            self.add_urp('  <children>')
            
        self.LAST_MOVETYPE = movetype 
        
        self.WaypointCount += 1
        #self.add_urp('    <Waypoint type="Fixed" name="Waypoint_%i" kinematicsFlags="4">' % self.WaypointCount)
        self.add_urp('    <Waypoint type="Fixed" name="Waypoint_%i">' % self.WaypointCount)
        if self.MotionParameters is not None:
            self.add_urp('      %s' % self.MotionParameters)
        elif movetype == UR_MoveL and self.BLEND_RADIUS_M > 0.0:
            # Important: Blend radius needs to be provided for each linear movement (otherwise, "stop" is taken into account)
            self.add_urp('      <motionParameters blendRadius="%.3f"/>' % self.BLEND_RADIUS_M)
            
        # self.MotionParameters = '<motionParameters/>' # once applied, do not alter
        if pose is None:
            # Warning!! Current URCaps 3.4 does not support absolute joint movement. Use script instead
            self.add_urp('      <position joints="%s"/>' % (angles_2_str(joints)[1:-1]))
        else:        
            poseabs = self.REF_FRAME*pose
            poseabsflange = poseabs*self.TOOL_FRAME_INV
            pose_str = pose_2_str(pose)[2:-1]
            poseabs_str = pose_2_str(poseabs)[2:-1]
            poseabsflange_str = pose_2_str(poseabsflange)[2:-1]            
            self.add_urp('      <position joints="%s" pose="%s"/>' % (angles_2_str(joints)[1:-1], poseabs_str))
            self.add_urp('      <poseInFeatureCoordinates pose="%s"/>' % poseabs_str)
            self.add_urp('      <outputFlangePose pose="%s"/>' % poseabsflange_str)
        self.add_urp('    </Waypoint>')
        
    def urp_move_close(self):
        if self.LAST_MOVETYPE is not None:
            self.add_urp('  </children>')
            self.add_urp('</Move>')
            self.LAST_MOVETYPE = None
        
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
        print(line)
    
    print("")
    print("")
    print("")
    for line in robot.PROG_URP:
        print(line)
    if len(robot.LOG) > 0:
        mbox('Program generation LOG:\n\n' + robot.LOG)

    input("Press Enter to close...")

if __name__ == "__main__":
    """Function to call when the module is executed by itself: test"""
    test_post()
