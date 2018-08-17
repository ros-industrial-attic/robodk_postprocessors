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
# for a Staubli (VAL3) robot with RoboDK. This post is made to properly support Staubli's external axes
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
PROGRAM_PJX = '''<?xml version="1.0" encoding="utf-8" ?>
<Project xmlns="http://www.staubli.com/robotics/VAL3/Project/3">
  <Parameters version="s7.3.1" stackSize="5000" millimeterUnit="true" />
  <Programs>
    <Program file="start.pgx" />
    <Program file="stop.pgx" />
    <Program file="setLink.pgx" />
  </Programs>
  <Database>
    <Data file="%s.dtx" />
  </Database>
  <Libraries>
  </Libraries>
</Project>
'''

PROGRAM_PJX_MAIN = '''<?xml version="1.0" encoding="utf-8" ?>
<Project xmlns="http://www.staubli.com/robotics/VAL3/Project/3">
  <Parameters version="s7.3.1" stackSize="5000" millimeterUnit="true" />
  <Programs>
    <Program file="loadNextOne.pgx" />
    <Program file="main.pgx" />
    <Program file="start.pgx" />
    <Program file="stop.pgx" />
  </Programs>
  <Database>
    <Data file="%s.dtx" />
  </Database>
  <Libraries>
    <Library alias="prog" path="./%s.pjx" />
    <Library alias="prog_swap" path="./%s.pjx" />
  </Libraries>
</Project>
'''

START_PGX_MAIN = '''<?xml version="1.0" encoding="utf-8"?>
<Programs xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns="http://www.staubli.com/robotics/VAL3/Program/2">
  <Program name="start" access="public">
    <Code><![CDATA[
begin
  $XframeLink(frExternal)
  call main(frExternal)
end
      ]]></Code>
  </Program>
</Programs>
'''

DATA_DTX = '''<?xml version="1.0" encoding="utf-8" ?>
<Database xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns="http://www.staubli.com/robotics/VAL3/Data/2">
  <Datas>
    <Data name="%s" access="public" xsi:type="array" type="frame" size="%i">
%s    </Data>
    <Data name="jPark" access="public" xsi:type="array" type="jointRx" size="1">
      <Value key="0" j3="90" j5="90" />
    </Data>
    <Data name="%s" access="public" xsi:type="array" type="jointRx" size="%i">
%s    </Data>
    <Data name="mNomSpeed" access="public" xsi:type="array" type="mdesc" size="1">
      <Value key="0" blend="off" />
    </Data>
    <Data name="%s" access="public" xsi:type="array" type="mdesc" size="%i">
%s    </Data>
    <Data name="nTraj" access="public" xsi:type="array" type="num" size="1"/>
    <Data name="%s" access="public" xsi:type="array" type="pointRx" size="%i">
%s    </Data>
    <Data name="%s" access="public" xsi:type="array" type="tool" size="%i">
%s    </Data>
%s  </Datas>
</Database>
'''

DATA_DTX_MAIN = '''<?xml version="1.0" encoding="utf-8" ?>
<Database xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns="http://www.staubli.com/robotics/VAL3/Data/2">
  <Datas>
    <Data name="dioIN" access="public" xsi:type="array" type="dio" size="1">
      <Value key="0" link="fIn0" />
    </Data>
    <Data name="dioOUT" access="public" xsi:type="array" type="dio" size="1">
      <Value key="0" link="fOut0" />
    </Data>
    <Data name="dioRotationON" access="public" xsi:type="array" type="dio" size="1">
      <Value key="0" link="BasicIO-1\\%%Q0" />
    </Data>
    <Data name="dioRotationOFF" access="public" xsi:type="array" type="dio" size="1">
      <Value key="0" link="BasicIO-1\\%%Q1" />
    </Data>
    <Data name="dioBuse" access="public" xsi:type="array" type="dio" size="1">
      <Value key="0" link="BasicIO-1\\%%Q2" />
    </Data>
    <Data name="fPartCad" access="public" xsi:type="array" type="frame" size="1">
%s    </Data>
    <Data name="fCadToReal" access="public" xsi:type="array" type="frame" size="1">
      <Value key="0" x="0" y="0" z="0" rx="0" ry="0" rz="0" fatherId="fPartCad[0]" />
    </Data>
    <Data name="mNomSpeed" access="public" xsi:type="array" type="mdesc" size="1">
      <Value key="0" blend="off" />
    </Data>
    <Data name="nTraj" access="public" xsi:type="array" type="num" size="1"/>
    <Data name="nTimeStop" access="private" xsi:type="array" type="num" size="1"/>
    <Data name="nTimeStart" access="private" xsi:type="array" type="num" size="1"/>
    <Data name="nMode" access="private" xsi:type="array" type="num" size="1"/>
    <Data name="nEtat" access="private" xsi:type="array" type="num" size="1"/>
    <Data name="tCad" access="public" xsi:type="array" type="tool" size="1">
%s    </Data>
    <Data name="frExternal" access="public" xsi:type="array" type="frame" size="1">
      <Value key="0" %s fatherId="world[0]" />
    </Data>
    <Data name="fWorld0" access="public" xsi:type="array" type="frame" size="1">
      <Value key="0" fatherId="world[0]" />
    </Data>
    <Data name="tAdjust" access="public" xsi:type="array" type="tool" size="1">
      <Value key="0" ioLink="valve1" />
    </Data>
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

# For the main program that calls subprograms
MAIN_PGX = '''<?xml version="1.0" encoding="utf-8"?>
<Programs xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns="http://www.staubli.com/robotics/VAL3/Program/2">
  <Program name="main" access="public">
    <Parameters xmlns="http://www.staubli.com/robotics/VAL3/Param/1">
      <Parameter name="x_fCellule" type="frame" xsi:type="element" use="reference" />
    </Parameters>
    <Locals>
      <Local name="l_nRef0" type="num" xsi:type="array" size="1" />
      <Local name="l_nResult" type="num" xsi:type="array" size="1" />
    </Locals>
    <Code><![CDATA[
begin
 
 
  link(fWorld0,x_fCellule)
  l_nRef0 =0
  fWorld0.trsf = $Xposition(world,x_fCellule,l_nRef0)
 
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
</Programs>
'''

SETLINK_PGX = '''<?xml version="1.0" encoding="utf-8"?>
<Programs xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns="http://www.staubli.com/robotics/VAL3/Program/2">
  <Program name="setLink" access="public">
    <Parameters xmlns="http://www.staubli.com/robotics/VAL3/Param/1">
      <Parameter name="x_fCellule" type="frame" xsi:type="element" use="reference" />
    </Parameters>
    <Code><![CDATA[
begin
  link(fPartReal,x_fCellule)
end
      ]]></Code>
  </Program>
</Programs>
'''


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
    return ('x="%.3f" y="%.3f" z="%.3f" rx="%.3f" ry="%.3f" rz="%.3f"' % (x,y,z,r,p,w))
    
def angles_2_str(angles):
    """Prints a joint target for Staubli VAL3 XML"""
    str = ''    
    for i in range(len(angles)):
        str = str + ('j%i="%.5f" ' % ((i+1), angles[i]))
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
    # Important: Enter the location of the external axis:
    FR_EXTERNAL_POSE = transl(678,0,0)
    
    # other variables
    ROBOT_POST = ''
    ROBOT_NAME = ''
    PROG_FILES = []
    PROG_NAME = 'unknown'
    MAIN_FOLDER = 'ProgRoboDK'
    PROG_PGX = ''
    PROG_MOVE_COUNT = 0
    PROG_MOVE_COUNT_MAX = 200
    PROG_PGX_LIST = []
    PROG_DTX_LIST = []
    PROG_PJX_LIST = []
    PROG_NAME_LIST = []
    LOG = ''
    nAxes = 6
    TAB_PGX = '  '
    DEFAULT_SPEED = 150
    DEFAULT_SMOOTH = 0.1
    SPEED = DEFAULT_SPEED
    REF = eye(4)
    TOOL = eye(4)
    SMOOTH = DEFAULT_SMOOTH
    REF_NAME = 'fPartReal'
    REF_CURRENT = 'world[0]'
    REF_DATA = ''
    REF_COUNT = 0
    TOOL_NAME = 'tCad'
    TOOL_CURRENT = 'flange[0]'
    TOOL_DATA = ''
    TOOL_COUNT = 0
    OTHER_DATA = '' # ntargets
    OTHER_COUNT = 0
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
        for k,v in kwargs.items():
            if k == 'pose_turntable':
                pose_turntable = v
                self.FR_EXTERNAL_POSE = pose_turntable
            elif k == 'pose_rail':
                pose_rail = v
                #self.FR_RAIL_POSE = pose_rail        
        
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
        if True: #nprogs > 1: # always create a main program
            folderprog = foldersave + '/' + main_progname
            self.MAIN_FOLDER = main_progname
        else:
            folderprog = foldersave + '/' + progname
            self.MAIN_FOLDER = progname
            
        if not DirExists(folderprog):
            import os
            os.makedirs(folderprog)
        
        show_file_list = []
        if True: #nprogs > 1: # always create a main program
            call_sequence = ''
            for i in range(nprogs):
                call_sequence+=('  if prog:libLoad("./%s")!=0\n' % self.PROG_NAME_LIST[i])
                call_sequence+=('    logMsg("Error Loading RoboDK library")\n')
                call_sequence+=('    popUpMsg("Error Loading RoboDK library")\n')
                call_sequence+=('  endIf\n')
                call_sequence+=('  wait(taskStatus("loading")==-1)\n')
                if i < nprogs-1:
                    call_sequence+=('  taskCreate "loading",10,loadNextOne("./%s")\n' % self.PROG_NAME_LIST[i+1])
                call_sequence+=('  prog:fPartReal.trsf=fPartCad.trsf*fCadToReal.trsf\n')
                #call_sequence+=('  prog:tCad.trsf=prog:tCad.trsf*tAdjust.trsf\n')
                call_sequence+=('  prog:tCad.trsf=tCad.trsf*tAdjust.trsf\n')
                call_sequence+=('  call prog:setLink(fWorld0)\n')
                call_sequence+=('  call prog:start()\n')
                call_sequence+=('  \n')
            
            #-----------------------------------
            # start.pgx (static file)
            start_file = folderprog + '/start.pgx'
            show_file_list.append(start_file)
            fid = open(start_file, "w")
            fid.write(START_PGX_MAIN)
            fid.close()
            #-----------------------------------
            # main.pgx
            start_file = folderprog + '/main.pgx'
            show_file_list.append(start_file)
            fid = open(start_file, "w")
            fid.write(MAIN_PGX % call_sequence)
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
            fid.write(DATA_DTX_MAIN % (self.REF_DATA, self.TOOL_DATA, pose_2_str(self.FR_EXTERNAL_POSE)))
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
            if True: # nprogs > 1: # Always create a main program loading sub programs
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
            fid.write(self.PROG_PGX_LIST[i])
            fid.close()
            #-----------------------------------
            # stop.pgx
            stop_file = folderprog_final + '/stop.pgx'
            fid = open(stop_file, "w")
            fid.write(STOP_PGX)
            fid.close()            
            #-----------------------------------
            # setLink.pgx
            setlink_file = folderprog_final + '/setLink.pgx'
            fid = open(setlink_file, "w")
            fid.write(SETLINK_PGX)
            fid.close()
            #-----------------------------------
            # program.pjx
            project_file = folderprog_final + '/%s.pjx' % self.PROG_NAME_LIST[i]
            #show_file_list.append(project_file)
            fid = open(project_file, "w")
            fid.write(self.PROG_PJX_LIST[i])
            fid.close()
            print('SAVED: %s\n' % project_file)
            #-----------------------------------
            # program.dtx
            program_data = folderprog_final + '/%s.dtx' % self.PROG_NAME_LIST[i]
            #show_file_list.append(project_file)
            fid = open(program_data, "w")
            fid.write(self.PROG_DTX_LIST[i])
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
        #nTraj=movej(jJoints[0],tTool[0],mSpeed[0])
        #waitEndMove()
        #      <Value key="0" j1="0.000" j2="-10.000" j3="100.000" j4="0.000" j5="0.000" j6="-90.000" />
        nextax = len(joints) - 6
        var_extax = ''
        if nextax > 0:
            var_extax = 'nTargetJoints%i' % self.OTHER_COUNT
            self.OTHER_COUNT = self.OTHER_COUNT + 1
            self.OTHER_DATA += '    <Data name="%s" access="private" xsi:type="array" type="num" size="%i">\n' % (var_extax, nextax)
            for i in range(nextax):
                self.OTHER_DATA += '      <Value key="%i" value="%.3f" />\n' % (i, joints[i+6])
            self.OTHER_DATA += '    </Data>\n'
            
        variable = '%s[%i]' % (self.JOINT_NAME, self.JOINT_COUNT)	
        self.JOINT_DATA = self.JOINT_DATA + '      <Value key="%i" %s />\n' % (self.JOINT_COUNT, angles_2_str(joints))
        self.JOINT_COUNT = self.JOINT_COUNT + 1
        if nextax > 0:
            self.addline('nTraj=$Xmovej(%s,%s[0],%s,%s)' % (variable, var_extax, self.TOOL_CURRENT, self.SPEED_CURRENT))        
        else:
            self.addline('nTraj=movej(%s,%s,%s)' % (variable, self.TOOL_CURRENT, self.SPEED_CURRENT))
        #self.addline('waitEndMove()')
        
    def MoveL(self, pose, joints, conf_RLF=None):
        """Add a linear movement"""
        self.control_ProgSize()
        #nTraj=movej(jJoints[0],tTool[0],mSpeed[0])
        #waitEndMove()
        #      <Value key="0" x="-36.802" y="-6.159" z="500.000" rx="135.407" ry="80.416" rz="46.453" shoulder="lefty" elbow="epositive" wrist="wpositive" fatherId="fPartReal[0]" />
        # Configuration needs to be checked for older RoboDK versions
        nextax = len(joints) - 6
        var_extax = ''
        if nextax > 0:
            var_extax = 'nTarget%i' % self.OTHER_COUNT
            self.OTHER_COUNT = self.OTHER_COUNT + 1
            self.OTHER_DATA += '    <Data name="%s" access="private" xsi:type="array" type="num" size="%i">\n' % (var_extax, nextax)
            for i in range(nextax):
                self.OTHER_DATA += '      <Value key="%i" value="%.3f" />\n' % (i, joints[i+6])
            self.OTHER_DATA += '    </Data>\n'
        
        if conf_RLF == None:
            str_config = 'shoulder="lefty" elbow="epositive" wrist="wpositive"'
        else:
            [rear, lowerarm, flip] = conf_RLF
            str_config = 'shoulder="%s" elbow="%s" wrist="%s"' % ("righty" if rear>0 else "lefty", "enegative" if lowerarm>0 else "epositive", "wnegative" if flip>0 else "wpositive")
        variable = '%s[%i]' % (self.POINT_NAME, self.POINT_COUNT)
        self.POINT_DATA = self.POINT_DATA + '      <Value key="%i" %s %s fatherId="%s" />\n' % (self.POINT_COUNT, pose_2_str(pose), str_config, self.REF_CURRENT)
        self.POINT_COUNT = self.POINT_COUNT + 1
        if nextax > 0:
            self.addline('nTraj=$Xmovel(%s,%s[0],%s,%s)' % (variable, var_extax, self.TOOL_CURRENT, self.SPEED_CURRENT))        
        else:
            self.addline('nTraj=movel(%s,%s,%s)' % (variable, self.TOOL_CURRENT, self.SPEED_CURRENT))        

        
    def MoveC(self, pose1, joints1, pose2, joints2, conf_RLF_1=None, conf_RLF_2=None):
        """Add a circular movement"""
        self.control_ProgSize()
        # Configuration needs to be checked for older RoboDK versions
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
        if self.REF_COUNT > 1:
            self.RunMessage('Warning: This post processor is meant to use one reference frame. Errors might follow.', True)
            self.log('This post processor is meant to use one reference frame. Errors might follow.')
        
        self.control_ProgSize()
        self.REF = pose
        #      <Value key="0" x="600.000" y="0.000" z="-465.000" rx="0.400" ry="0.100" rz="-45.000" fatherId="world[0]" />
        self.REF_CURRENT = '%s[%i]' % (self.REF_NAME, self.REF_COUNT)
        self.REF_DATA = self.REF_DATA + '      <Value key="%i" %s fatherId="world[0]" />\n' % (self.REF_COUNT, pose_2_str(self.FR_EXTERNAL_POSE*pose))
        self.REF_COUNT = self.REF_COUNT + 1
        
    def setTool(self, pose, tool_id=None, tool_name=None):
        """Change the robot TCP"""
        if self.TOOL_COUNT > 1:
            self.RunMessage('Warning: Only one tool allowed per program. Tool change skipped.', True)
            self.log('Only one tool allowed per program')
            return
            
        self.control_ProgSize()
        self.TOOL = pose
        #      <Value key="0" x="-5.972" y="209.431" z="55.323" rx="-90.190" ry="-0.880" rz="89.997" fatherId="flange[0]" ioLink="valve1" />
        self.TOOL_CURRENT = '%s[%i]' % (self.TOOL_NAME, self.TOOL_COUNT)
        self.TOOL_DATA = self.TOOL_DATA + '      <Value key="%i" %s fatherId="flange[0]" ioLink="valve1" />\n' % (self.TOOL_COUNT, pose_2_str(pose))
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
        #      <Value key="0" accel="100" vel="100" decel="100" tmax="50" rmax="100" blend="joint" leave="0.1" reach="0.1" />
        self.SPEED = speed_mms
        self.SPEED_CURRENT = '%s[%i]' % (self.SPEED_NAME, self.SPEED_COUNT)
        # blend = "off" / "joint" / "Cartesian"
        #self.SPEED_DATA = self.SPEED_DATA + '      <Value key="%i" accel="100" vel="100" decel="100" tmax="%.1f" rmax="100" blend="cartesian" leave="%.1f" reach="%0.1f" />\n' % (self.SPEED_COUNT, speed_mms, self.SMOOTH, self.SMOOTH)
        self.SPEED_DATA = self.SPEED_DATA + '      <Value key="%i" tmax="%.1f" rmax="100" leave="%.1f" reach="%0.1f" />\n' % (self.SPEED_COUNT, speed_mms, self.SMOOTH, self.SMOOTH)
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
        self.control_ProgSize()
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
            
        self.PROG_PGX_LIST.append(START_PGX % self.PROG_PGX)
        self.PROG_DTX_LIST.append(DATA_DTX % (self.REF_NAME, self.REF_COUNT, self.REF_DATA,  self.JOINT_NAME, self.JOINT_COUNT, self.JOINT_DATA,  self.SPEED_NAME, self.SPEED_COUNT, self.SPEED_DATA,  self.POINT_NAME, self.POINT_COUNT, self.POINT_DATA,  self.TOOL_NAME, self.TOOL_COUNT, self.TOOL_DATA, self.OTHER_DATA))
        self.PROG_PJX_LIST.append(PROGRAM_PJX % progname)
        self.PROG_NAME_LIST.append(progname)
        self.PROG_PGX = ''
        self.REF_DATA = ''
        self.REF_COUNT = 0
        self.TOOL_DATA = ''
        self.TOOL_COUNT = 0
        self.OTHER_DATA = '' # ntargets
        self.OTHER_COUNT = 0
        self.SPEED_DATA = ''
        self.SPEED_COUNT = 0
        self.JOINT_DATA = ''
        self.JOINT_COUNT = 0
        self.POINT_DATA = ''
        self.POINT_COUNT = 0
        self.PROG_MOVE_COUNT = 0        
        # initialise next program
        self.setFrame(self.REF)
        self.setTool(self.TOOL)
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
