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
from robolink import *


# Program.pjx file (references data file as %s.dtx)
PROGRAM_PJX = '''<?xml version="1.0" encoding="utf-8"?>
<Project xmlns="http://www.staubli.com/robotics/VAL3/Project/3">
<Parameters version="s7.5.3" stackSize="5000" millimeterUnit="true" />
<Programs>
<Program file="MoveJoint.pgx" />
<Program file="MoveLine.pgx" />
<Program file="start.pgx" />
<Program file="stop.pgx" />
<Program file="main.pgx" />
</Programs>
<Database>
<Data file="%s.dtx" />
</Database>
<Libraries />
</Project>
'''

PROGRAM_DTX = '''<?xml version="1.0" encoding="utf-8"?>
<Database xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns="http://www.staubli.com/robotics/VAL3/Data/2">
  <Datas>
    <Data name="mNomSpeed" access="private" xsi:type="array" type="mdesc" size="1" />
    <Data name="fFrame1" access="private" xsi:type="array" type="frame" size="1">
      <Value key="0" %s fatherId="world[0]" />
    </Data>
    <Data name="Input" access="private" xsi:type="array" type="dio" size="16">
      <Value key="0" link="BasicIO-2\\%%I0" />
      <Value key="1" link="BasicIO-2\\%%I1" />
      <Value key="2" link="BasicIO-2\\%%I2" />
      <Value key="3" link="BasicIO-2\\%%I3" />
      <Value key="4" link="BasicIO-2\\%%I4" />
      <Value key="5" link="BasicIO-2\\%%I5" />
      <Value key="6" link="BasicIO-2\\%%I6" />
      <Value key="7" link="BasicIO-2\\%%I7" />
      <Value key="8" link="BasicIO-2\\%%I8" />
      <Value key="9" link="BasicIO-2\\%%I9" />
      <Value key="10" link="BasicIO-2\\%%I10" />
      <Value key="11" link="BasicIO-2\\%%I11" />
      <Value key="12" link="BasicIO-2\\%%I12" />
      <Value key="13" link="BasicIO-2\\%%I13" />
      <Value key="14" link="BasicIO-2\\%%I14" />
      <Value key="15" link="BasicIO-2\\%%I15" />
    </Data>
    <Data name="Output" access="private" xsi:type="array" type="dio" size="16">
      <Value key="0" link="BasicIO-2\\%%Q0" />
      <Value key="1" link="BasicIO-2\\%%Q1" />
      <Value key="2" link="BasicIO-2\\%%Q2" />
      <Value key="3" link="BasicIO-2\\%%Q3" />
      <Value key="4" link="BasicIO-2\\%%Q4" />
      <Value key="5" link="BasicIO-2\\%%Q5" />
      <Value key="6" link="BasicIO-2\\%%Q6" />
      <Value key="7" link="BasicIO-2\\%%Q7" />
      <Value key="8" link="BasicIO-2\\%%Q8" />
      <Value key="9" link="BasicIO-2\\%%Q9" />
      <Value key="10" link="BasicIO-2\\%%Q10" />
      <Value key="11" link="BasicIO-2\\%%Q11" />
      <Value key="12" link="BasicIO-2\\%%Q12" />
      <Value key="13" link="BasicIO-2\\%%Q13" />
      <Value key="14" link="BasicIO-2\\%%Q14" />
      <Value key="15" link="BasicIO-2\\%%Q15" />
    </Data>
    <Data name="tCurrentTool" access="private" xsi:type="array" type="tool" size="1">
      <Value key="0" %s fatherId="flange[0]" ioLink="valve1" />
    </Data>
    <Data name="mCurrentSpeed" access="private" xsi:type="array" type="mdesc" size="1" />
    <Data name="pPointRx" access="private" xsi:type="array" type="pointRx" size="1">
      <Value key="0" fatherId="fFrame1[0]" />
    </Data>
  </Datas>
</Database>
'''

MOVELINE_PGX = '''<?xml version="1.0" encoding="utf-8"?>
<Programs xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns="http://www.staubli.com/robotics/VAL3/Program/2">
  <Program name="MoveLine">
    <Parameters xmlns="http://www.staubli.com/robotics/VAL3/Param/1">
      <Parameter name="x_nX" type="num" xsi:type="element" />
      <Parameter name="x_nY" type="num" xsi:type="element" />
      <Parameter name="x_nZ" type="num" xsi:type="element" />
      <Parameter name="x_nRx" type="num" xsi:type="element" />
      <Parameter name="x_nRy" type="num" xsi:type="element" />
      <Parameter name="x_nRz" type="num" xsi:type="element" />
    </Parameters>
    <Code><![CDATA[begin
  pPointRx.trsf.x = x_nX
  pPointRx.trsf.y = x_nY
  pPointRx.trsf.z = x_nZ
  pPointRx.trsf.rx = x_nRx
  pPointRx.trsf.ry = x_nRy
  pPointRx.trsf.rz = x_nRz
  movel(pPointRx,tCurrentTool,mCurrentSpeed)
end]]></Code>
  </Program>
</Programs>
'''

MOVEJOINT_PGX = '''<?xml version="1.0" encoding="utf-8"?>
<Programs xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns="http://www.staubli.com/robotics/VAL3/Program/2">
  <Program name="MoveJoint">
    <Parameters xmlns="http://www.staubli.com/robotics/VAL3/Param/1">
      <Parameter name="x_nJ1" type="num" xsi:type="element" />
      <Parameter name="x_nJ2" type="num" xsi:type="element" />
      <Parameter name="x_nJ3" type="num" xsi:type="element" />
      <Parameter name="x_nJ4" type="num" xsi:type="element" />
      <Parameter name="x_nJ5" type="num" xsi:type="element" />
      <Parameter name="x_nJ6" type="num" xsi:type="element" />
    </Parameters>
    <Locals>
      <Local name="l_jJoint" type="joint" xsi:type="array" size="1" />
    </Locals>
    <Code><![CDATA[begin
  l_jJoint.j1 = x_nJ1
  l_jJoint.j2 = x_nJ2
  l_jJoint.j3 = x_nJ3
  l_jJoint.j4 = x_nJ4
  l_jJoint.j5 = x_nJ5
  l_jJoint.j6 = x_nJ6
  movej(l_jJoint,tCurrentTool ,mCurrentSpeed)
end]]></Code>
  </Program>
</Programs>
'''

MAIN_PGX = '''<?xml version="1.0" encoding="utf-8"?>
<Programs xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns="http://www.staubli.com/robotics/VAL3/Program/2">
  <Program name="main">
    <Code><![CDATA[begin
  close(tCurrentTool)
  pPointRx.config.shoulder = lefty
  pPointRx.config.elbow = epositive
  pPointRx.config.wrist = wpositive
  mCurrentSpeed.tvel = 100.00
  mCurrentSpeed.blend = joint
  mCurrentSpeed.reach = 0.010
  mCurrentSpeed.leave = 0.010
%s
end]]></Code>
  </Program>
</Programs>'''


# start.pjx file (references data file as %s.dtx)
START_PGX = '''<?xml version="1.0" encoding="utf-8"?>
<Programs xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns="http://www.staubli.com/robotics/VAL3/Program/2">
  <Program name="start">
    <Code><![CDATA[begin
  call main()
end]]></Code>
  </Program>
</Programs>
'''

STOP_PGX = '''<?xml version="1.0" encoding="utf-8"?>
<Programs xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns="http://www.staubli.com/robotics/VAL3/Program/2">
  <Program name="stop">
    <Code><![CDATA[begin
end]]></Code>
  </Program>
</Programs>
'''

# Required if the program must be split:

LOAD_NEXT_ONE = '''<?xml version="1.0" encoding="utf-8" ?>
<Programs xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns="http://www.staubli.com/robotics/VAL3/Program/2">
  <Program name="loadNextOne" access="private">
    <Parameters xmlns="http://www.staubli.com/robotics/VAL3/Param/1">
      <Parameter name="x_sName" type="string" xsi:type="element" />
    </Parameters>
    <Code><![CDATA[
begin
  prog_swap:libLoad(x_sName)
end
      ]]></Code>
  </Program>
</Programs>'''

START_PGX_MAIN = '''<?xml version="1.0" encoding="utf-8" ?>
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

PROGRAM_PJX_MAIN = '''<?xml version="1.0" encoding="utf-8" ?>
<Project xmlns="http://www.staubli.com/robotics/VAL3/Project/3">
  <Parameters version="s7.3.1" stackSize="5000" millimeterUnit="true" />
  <Programs>
    <Program file="loadNextOne.pgx" />
    <Program file="start.pgx" />
    <Program file="stop.pgx" />
  </Programs>
  <Database>
    <Data file="%s.dtx" />
  </Database>
  <Libraries>
    <Library alias="prog" path="./%s.pjx" />
    <Library alias="prog_swap" path="./%s.pjx" />
    <Library alias="tooldata" path="saveChangeTool" />
  </Libraries>
</Project>'''


def Pose_2_Staubli_v2(H):
    """Converts a pose to a Staubli target target"""
    x = H[0,3]
    y = H[1,3]
    z = H[2,3]
    a = H[0,0]
    b = H[0,1]
    c = H[0,2]
    d = H[1,2]
    e = H[2,2]
    if c > (1.0 - 1e-10):
        ry1 = pi/2
        rx1 = 0
        rz1 = atan2(H[1,0],H[1,1])
    elif c < (-1.0 + 1e-10):
        ry1 = -pi/2
        rx1 = 0
        rz1 = atan2(H[1,0],H[1,1])
    else:
        sy = c
        cy1 = +sqrt(1-sy*sy)
        sx1 = -d/cy1
        cx1 = e/cy1
        sz1 = -b/cy1
        cz1 = a/cy1
        rx1 = atan2(sx1,cx1)
        ry1 = atan2(sy,cy1)
        rz1 = atan2(sz1,cz1)
    return [x, y, z, rx1*180.0/pi, ry1*180.0/pi, rz1*180.0/pi]

# ----------------------------------------------------
def pose_2_str(pose):
    """Prints a pose target"""
    [x,y,z,r,p,w] = Pose_2_Staubli_v2(pose)
    return ('%.3f,%.3f,%.3f,%.3f,%.3f,%.3f' % (x,y,z,r,p,w))
    
def angles_2_str(angles):
    """Prints a joint target for Staubli VAL3 XML"""
    str = ''    
    for i in range(len(angles)):
        str = str + ('%.5f,' % (angles[i]))
    str = str[:-1]
    return str

def pose_2_str_dtx(pose):
    """Prints a pose target"""
    [x,y,z,r,p,w] = Pose_2_Staubli_v2(pose)
    return ('x="%.3f" y="%.3f" z="%.3f" rx="%.3f" ry="%.3f" rz="%.3f"' % (x,y,z,r,p,w)) 
    
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
    PROG_NAME = 'unknown'
    MAIN_FOLDER = 'ProgRoboDK'
    PROG_PGX = ''
    PROG_MOVE_COUNT = 0
    PROG_MOVE_COUNT_MAX = 200
    LOG = ''
    REF = eye(4)
    TOOL = eye(4)
    REF_COUNT = 0
    TOOL_COUNT = 0
    nAxes = 6
    TAB_PGX = '  '
    PROG_PGX_LIST = []
    PROG_NAME_LIST = []
    SPEED = -1
    # Warning: if defaults are changed they must also be changed in main.pgx
    CONF_shoulder = 0   # 0=lefty       ; 1=righty
    CONF_lowerarm = 0   # 0=epositive   ; 1=enegative
    CONF_flip = 0       # 0=wpositive   ; 1=wnegative
    
    def __init__(self, robotpost=None, robotname=None, robot_axes = 6, **kwargs):
        self.ROBOT_POST = robotpost
        self.ROBOT_NAME = robotname
        self.PROG = ''
        self.LOG = ''
        self.nAxes = robot_axes
        
    def ProgStart(self, progname):
        self.PROG_NAME = progname
        self.addline('// Program %s start' % progname)
        
    def ProgFinish(self, progname):
        self.addline('')
        self.addline('waitEndMove()')
        self.addline('// Program %s end' % progname)    
    
    def RemoveDirFTP(self, ftp, path):
        import ftplib
        """Recursively delete a directory tree on a remote server."""
        wd = ftp.pwd()
        try:
            names = ftp.nlst(path)
        except ftplib.all_errors as e:
            # some FTP servers complain when you try and list non-existent paths
            print('RemoveDirFTP: Could not remove {0}: {1}'.format(path, e))
            return

        for name in names:
            if os.path.split(name)[1] in ('.', '..'): continue
            print('RemoveDirFTP: Checking {0}'.format(name))
            try:
                ftp.cwd(name)  # if we can cwd to it, it's a folder
                ftp.cwd(wd)  # don't try a nuke a folder we're in
                self.RemoveDirFTP(ftp, name)
            except ftplib.all_errors:
                ftp.delete(name)

        try:
            ftp.rmd(path)
        except ftplib.all_errors as e:
            print('RemoveDirFTP: Could not remove {0}: {1}'.format(path, e))
    
    def UploadFTP(self, localpath):
        import ftplib
        import os
        robot = None
        try:
            RDK = Robolink()
            robot = RDK.Item(self.ROBOT_NAME, ITEM_TYPE_ROBOT)
            [server_ip, port, remote_path, username, password] = robot.ConnectionParams()
        except:
            server_ip = 'localhost'  # enter URL address of the robot it may look like: '192.168.1.123'
            username = 'username'     # enter FTP user name
            password = 'password'     # enter FTP password
            remote_path = '/usr/usrapp/session/default/saveTraj/CAD' # enter the remote path
        import sys
        while True:
            print("POPUP: Uploading program through FTP. Enter server parameters...")
            sys.stdout.flush()
            
            # check if connection parameters are OK
            values_ok = mbox('Using the following FTP connection parameters to transfer the program:\nRobot IP: %s\nRemote path: %s\nFTP username: %s\nFTP password: ****\n\nContinue?' % (server_ip, remote_path, username))
            if values_ok:
                print("Using default connection parameters")
            else:
                server_ip = mbox('Enter robot IP', entry=server_ip)
                if not server_ip:
                    print('FTP upload cancelled by user')
                    return
                remote_path = mbox('Enter the remote path (program folder) on the Staubli robot controller', entry=remote_path)
                if not remote_path:
                    return
                if remote_path.endswith('/'):
                    remote_path = remote_path[:-1]
                rob_user_pass = mbox('Enter user name and password as\nuser-password', entry=('%s-%s' % (username, password)))
                if not rob_user_pass:
                    return    
                name_value = rob_user_pass.split('-')
                if len(name_value) < 2:
                    password = ''
                else:
                    password = name_value[1]
                username = name_value[0]
            print("POPUP: <p>Connecting to <strong>%s</strong> using user name <strong>%s</strong> and password ****</p><p>Please wait...</p>" % (server_ip, username))
            #print("POPUP: Trying to connect. Please wait...")
            sys.stdout.flush()
            if robot is not None:
                robot.setConnectionParams(server_ip, port, remote_path, username, password)
            pause(2)
            try:
                myFTP = ftplib.FTP(server_ip, username, password)
                break;
            except:
                error_str = sys.exc_info()[1]
                print("POPUP: Connection to %s failed: <p>%s</p>" % (server_ip,error_str))
                sys.stdout.flush()
                contin = mbox("Connection to %s failed. Reason:\n%s\n\nRetry?" % (server_ip,error_str))
                if not contin:
                    return

        remote_path_prog = remote_path + '/' + self.MAIN_FOLDER
        myPath = r'%s' % localpath
        print("POPUP: Connected. Deleting existing files on %s..." % remote_path_prog)
        sys.stdout.flush()
        self.RemoveDirFTP(myFTP, remote_path_prog)
        print("POPUP: Connected. Uploading program to %s..." % server_ip)
        sys.stdout.flush()
        try:
            myFTP.cwd(remote_path)
            myFTP.mkd(self.MAIN_FOLDER)
            myFTP.cwd(remote_path_prog)
            #print('asdf')
        except:
            error_str = sys.exc_info()[1]
            print("POPUP: Remote path not found or can't be created: %s" % (remote_path))
            sys.stdout.flush()
            contin = mbox("Remote path\n%s\nnot found or can't create folder.\n\nChange path and permissions and retry." % remote_path)
            return
            
        def uploadThis(path):
            files = os.listdir(path)
            os.chdir(path)
            for f in files:
                if os.path.isfile(path + r'\{}'.format(f)):
                    print('  Sending file: %s' % f)
                    print("POPUP: Sending file: %s" % f)
                    sys.stdout.flush()
                    fh = open(f, 'rb')
                    myFTP.storbinary('STOR %s' % f, fh)
                    fh.close()
                elif os.path.isdir(path + r'\{}'.format(f)):
                    print('  Sending folder: %s' % f)
                    myFTP.mkd(f)
                    myFTP.cwd(f)
                    uploadThis(path + r'\{}'.format(f))
            myFTP.cwd('..')
            os.chdir('..')
        uploadThis(myPath) # now call the recursive function 

    def ProgSave(self, folder, progname, ask_user = False, show_result = False):
        self.close_module()
            
        if ask_user or not DirExists(folder):
            foldersave = getSaveFolder(folder, 'Save program as...')
            if foldersave is not None and len(foldersave) > 0:
                foldersave = foldersave
            else:
                return
        else:
            foldersave = folder
        
        nprogs = len(self.PROG_NAME_LIST)
        print("Saving %i programs..." % nprogs)
        
        main_progname = 'Main' + progname
        if nprogs > 1: # always create a main program
            folderprog = foldersave + '/' + main_progname
            self.MAIN_FOLDER = main_progname
        else:
            folderprog = foldersave + '/' + progname
            self.MAIN_FOLDER = progname
            
        if not DirExists(folderprog):
            import os
            os.makedirs(folderprog)
        
        show_file_list = []
        if nprogs > 1:
            call_sequence = ''
            for i in range(nprogs):
                call_sequence+=('  if prog:libLoad("./%s")!=0\n' % self.PROG_NAME_LIST[i])
                call_sequence+=('    logMsg("Error Loading RoboDK library")\n')
                call_sequence+=('    popUpMsg("Error Loading RoboDK library")\n')
                call_sequence+=('  endIf\n')
                call_sequence+=('  wait(taskStatus("loading")==-1)\n')
                if i < nprogs-1:
                    call_sequence+=('  taskCreate "loading",10,loadNextOne("./%s")\n' % self.PROG_NAME_LIST[i+1])                    
                #call_sequence+=('  prog:fPartReal.trsf=fPartCad.trsf*fCadToReal.trsf\n')
                #call_sequence+=('  prog:tCad.trsf=prog:tCad.trsf*{0,0,tooldata:nLength,0,0,0}\n')               
                call_sequence+=('  call prog:start()\n')
                call_sequence+=('  \n')

            #-----------------------------------
            # start.pgx
            start_file = folderprog + '/start.pgx'
            show_file_list.append(start_file)
            fid = open(start_file, "w")
            fid.write(START_PGX_MAIN % call_sequence)
            fid.close()
            #-----------------------------------
            # mainprog.pjx
            project_file = folderprog + '/%s.pjx' % main_progname
            #show_file_list.append(project_file)
            fid = open(project_file, "w")
            dummy_folder = self.PROG_NAME_LIST[0] + '/' + self.PROG_NAME_LIST[0]
            fid.write(PROGRAM_PJX_MAIN % (main_progname, dummy_folder, dummy_folder))
            fid.close()
            print('SAVED: %s\n' % project_file)
            #-----------------------------------
            # mainprog.dtx
            program_data = folderprog + '/%s.dtx' % main_progname
            show_file_list.append(project_file)
            fid = open(program_data, "w")
            fid.write(PROGRAM_DTX % (pose_2_str_dtx(self.REF), pose_2_str_dtx(self.TOOL)))
            fid.close()
            #-----------------------------------
            # stop.pgx
            stop_file = folderprog + '/stop.pgx'
            fid = open(stop_file, "w")
            fid.write(STOP_PGX)
            fid.close()
            #-----------------------------------
            # loadNextOne.pgx
            program_data = folderprog + '/loadNextOne.pgx'
            fid = open(program_data, "w")
            fid.write(LOAD_NEXT_ONE)
            fid.close()
            #-----------------------------------
        
        for i in range(nprogs):
            if nprogs > 1: # Always create a main program loading sub programs
                folderprog_final = folderprog + '/' + self.PROG_NAME_LIST[i]
            else:
                folderprog_final = folderprog
                
            if not DirExists(folderprog_final):
                import os 
                os.makedirs(folderprog_final)
            
            #-----------------------------------
            # start.pgx
            start_file = folderprog_final + '/start.pgx'
            #show_file_list.append(start_file)
            fid = open(start_file, "w")
            fid.write(START_PGX)
            fid.close()
            #-----------------------------------
            # stop.pgx
            stop_file = folderprog_final + '/stop.pgx'
            fid = open(stop_file, "w")
            fid.write(STOP_PGX)
            fid.close()
            #-----------------------------------
            # MoveJoint.pgx
            movej_file = folderprog_final + '/MoveJoint.pgx'
            #show_file_list.append(start_file)
            fid = open(movej_file, "w")
            fid.write(MOVEJOINT_PGX)
            fid.close()
            #-----------------------------------
            # MoveLine.pgx
            movel_file = folderprog_final + '/MoveLine.pgx'
            fid = open(movel_file, "w")
            fid.write(MOVELINE_PGX)
            fid.close()
            #-----------------------------------
            # program.PJX
            project_file = folderprog_final + '/%s.pjx' % self.PROG_NAME
            #show_file_list.append(project_file)
            fid = open(project_file, "w")
            fid.write(PROGRAM_PJX % self.PROG_NAME)
            fid.close()
            print('SAVED: %s\n' % project_file)
            #-----------------------------------
            # program.dtx
            program_data = folderprog_final + '/%s.dtx' % self.PROG_NAME
            show_file_list.append(program_data)
            fid = open(program_data, "w")
            fid.write(PROGRAM_DTX % (pose_2_str_dtx(self.REF), pose_2_str_dtx(self.TOOL)))
            fid.close()
            #-----------------------------------
            # main.pgx
            main_data = folderprog_final + '/main.pgx'
            show_file_list.append(main_data)
            fid = open(main_data, "w")
            fid.write(MAIN_PGX % (self.PROG_PGX_LIST[i]))
            fid.close()
            #-----------------------------------
        
        #self.UploadFTP(folderprog)
        self.PROG_FILES = folderprog
        
        if show_result:            
            if type(show_result) is str:
                # Open file with provided application
                import subprocess
                for file_i in show_file_list:
                    p = subprocess.Popen([show_result, file_i])
                #p = subprocess.Popen([show_result, start_file])
                #p = subprocess.Popen([show_result, program_data])                
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
        # attempt FTP upload
        
    def ProgSendRobot(self, robot_ip, remote_path, ftp_user, ftp_pass):
        """Send a program to the robot using the provided parameters. This method is executed right after ProgSave if we selected the option "Send Program to Robot".
        The connection parameters must be provided in the robot connection menu of RoboDK"""
        UploadFTP(self.PROG_FILES, robot_ip, remote_path, ftp_user, ftp_pass)
        
    def MoveJ(self, pose, joints, conf_RLF=None):
        """Add a joint movement"""
        self.control_ProgSize()
        self.addline('call MoveJoint(%s)' % angles_2_str(joints))
        
    def MoveL(self, pose, joints, conf_RLF=None):
        """Add a linear movement"""
        if pose is None:
            raise Exception("Linear move must be a Cartesian target for VAL3")
        self.control_ProgSize()
        
        if conf_RLF == None:
            rear = self.CONF_shoulder
            lowerarm = self.CONF_lowerarm
            flip = self.CONF_flip
        else:
            [rear, lowerarm, flip] = conf_RLF
            
        if rear != self.CONF_shoulder:
            self.addline('pPointRx.config.shoulder = %s' % ("righty" if rear>0 else "lefty"))
            self.CONF_shoulder = rear
        if lowerarm != self.CONF_lowerarm:
            self.addline('pPointRx.config.elbow = %s' % ("enegative" if lowerarm>0 else "epositive"))
            self.CONF_lowerarm = lowerarm
        if flip != self.CONF_flip:
            self.addline('pPointRx.config.wrist = %s' % ("wnegative" if flip>0 else "wpositive"))
            self.CONF_flip = flip         
        
        self.addline('call MoveLine(%s)' % pose_2_str(pose))        

    def MoveC(self, pose1, joints1, pose2, joints2, conf_RLF_1=None, conf_RLF_2=None):
        """Add a circular movement"""
        self.control_ProgSize()
        raise Exception("Circular moves not supported for this post processor")
        
    def setFrame(self, pose, frame_id=None, frame_name=None):
        """Change the robot reference frame"""
        if self.REF_COUNT > 1:
            self.RunMessage('Warning: This post processor is meant to use one reference frame. Errors might follow.', True)
            self.log('This post processor is meant to use one reference frame. Errors might follow.')
        
        self.control_ProgSize()
        self.REF = pose
        self.REF_COUNT = self.REF_COUNT + 1
        
    def setTool(self, pose, tool_id=None, tool_name=None):
        """Change the robot TCP"""
        if self.TOOL_COUNT > 1:
            self.RunMessage('Warning: Only one tool allowed per program. Tool change skipped.', True)
            self.log('Only one tool allowed per program')
            return
            
        self.control_ProgSize()
        self.TOOL = pose
        self.TOOL_COUNT = self.TOOL_COUNT + 1
        
    def Pause(self, time_ms):
        """Pause the robot program"""
        self.control_ProgSize()
        if time_ms < 0:
            self.addline('popUpMsg("Paused. Press OK to continue")')
        else:
            self.addline('delay(%.3f)' % (time_ms*0.001))
    
    def setSpeed(self, speed_mms):
        """Changes the robot speed (in mm/s)"""
        if self.SPEED != speed_mms:
            self.SPEED = speed_mms
            self.addline('mCurrentSpeed.tvel = %.3f' % speed_mms)
    
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
        self.addline('mCurrentSpeed.reach = %.3f' % zone_mm)
        self.addline('mCurrentSpeed.leave = %.3f' % zone_mm)
        
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
        self.control_ProgSize()
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
        self.control_ProgSize()
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
        
    def control_ProgSize(self):
        self.PROG_MOVE_COUNT = self.PROG_MOVE_COUNT + 1
        if self.PROG_MOVE_COUNT > self.PROG_MOVE_COUNT_MAX:
            self.close_module()
            
    def close_module(self):
        if self.PROG_MOVE_COUNT == 0:
            return
            
        progname = self.PROG_NAME
        nprogs = len(self.PROG_NAME_LIST)
        if nprogs > 0:
            progname = progname + ('%i' % (nprogs+1))
            
        self.PROG_PGX_LIST.append(self.PROG_PGX)
        self.PROG_NAME_LIST.append(progname)
        self.PROG_PGX = ''        
        self.PROG_MOVE_COUNT = 0        
        # initialise next program
        self.setSpeed(self.SPEED)
        self.PROG_MOVE_COUNT = 0 # very important to avoid writting two programs
 
        

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
