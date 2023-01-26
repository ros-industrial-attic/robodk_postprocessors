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

# --------------------------------------------
# --------------- DESCRIPTION ----------------
#
# This is a robotics toolbox for RoboDK robot post processors and RoboDK API for Python
# This toolbox includes a simple matrix class for pose transofmrations (Mat)
# This toolbox has been inspired from Peter Corke's Robotics Toolbox:
# http://petercorke.com/wordpress/toolboxes/robotics-toolbox
#
# In this document: pose = transformation matrix = homogeneous matrix = 4x4 matrix
# Visit the Matrix and Quaternions FAQ for more information about pose/homogeneous transformations
#     http://www.j3d.org/matrix_faq/matrfaq_latest.html
#
# More information about RoboDK post processors here:
#     http://www.robodk.com/help#PostProcessor
#
# More information about the RoboDK API for Python here:
#     http://www.robodk.com/doc/PythonAPI/index.html
# --------------------------------------------

import math
import operator
import sys
import unittest
import time

#----------------------------------------------------
#--------      Generic file usage     ---------------

import os.path
import time

def searchfiles(pattern='C:\\RoboDK\\Library\\*.rdk'):
    """List the files in a directory with a given extension"""
    import glob
    return glob.glob(pattern)

def CurrentFile(file = __file__):
    """Returns the current Python file being executed"""
    return os.path.realpath(file)

def getFileDir(filepath):
    """Returns the directory of a file path"""
    return os.path.dirname(filepath)
    
def getBaseName(filepath):
    """Returns the file name and extension of a file path"""
    return os.path.basename(filepath)

def getFileName(filepath):
    """Returns the file name (with no extension) of a file path"""
    return os.path.splitext(os.path.basename(filepath))[0]
   
def DateModified(filepath, stringformat=False):
    """Returns the time that a file was modified"""
    time_in_s = os.path.getmtime(filepath)
    if stringformat:
        return time.ctime(time_in_s)
    else:
        return time_in_s
    
def DateCreated(filepath, stringformat=False):
    """Returns the time that a file was modified"""
    time_in_s = os.path.getctime(filepath)
    if stringformat:
        return time.ctime(time_in_s)
    else:
        return time_in_s
        
def DirExists(folder):
    """Returns true if the folder exists"""
    return os.path.isdir(folder)

def FileExists(file):
    """Returns true if the file exists"""
    return os.path.exists(file)

#----------------------------------------------------
#--------      Generic math usage     ---------------

pi = math.pi

def pause(seconds):
    """Pause in seconds
    
    :param pause: time in seconds
    :type pause: float"""
    time.sleep(seconds)

def atan2(y,x):
    """Returns angle of a 2D coordinate in the XY plane"""
    return math.atan2(y,x)

def sqrt(value):
    """Returns the square root of a value"""
    return math.sqrt(value)

def sin(value):
    """Returns the sinus of an angle in radians"""
    return math.sin(value)

def cos(value):
    """Returns the cosinus of an angle in radians"""
    return math.cos(value)
    
def asin(value):
    """Returns the arc sinus in radians"""
    return math.asin(value)

def acos(value):
    """Returns the arc cosinus in radians"""
    return math.acos(value)
    
def name_2_id(str_name_id):
    """Returns the number of a numbered object. For example: "Frame 3" returns 3"""
    words = str_name_id.split()
    number = words.pop()
    if number.isdigit():
        return float(number)
    return -1
    

#----------------------------------------------------
#--------     Generic matrix usage    ---------------

def rotx(rx):
    r"""Returns a rotation matrix around the X axis
    
    .. math::
        
        R_x(\theta) = \begin{bmatrix} 1 & 0 & 0 & 0 \\
        0 & c_\theta & -s_\theta & 0 \\
        0 & s_\theta & c_\theta & 0 \\
        0 & 0 & 0 & 1
        \end{bmatrix}
    
    :param rx: rotation around X axis in radians
    :type rx: float"""
    ct = math.cos(rx)
    st = math.sin(rx)
    return Mat([[1,0,0,0],[0,ct,-st,0],[0,st,ct,0],[0,0,0,1]])

def roty(ry):
    r"""Returns a rotation matrix around the Y axis
    
    .. math::
        
        R_y(\theta) = \begin{bmatrix} c_\theta & 0 & s_\theta & 0 \\
        0 & 1 & 0 & 0 \\
        -s_\theta & 0 & c_\theta & 0 \\
        0 & 0 & 0 & 1
        \end{bmatrix}
    
    :param ry: rotation around Y axis in radians
    :type ry: float"""
    ct = math.cos(ry)
    st = math.sin(ry)
    return Mat([[ct,0,st,0],[0,1,0,0],[-st,0,ct,0],[0,0,0,1]])

def rotz(rz):
    r"""Returns a rotation matrix around the Z axis
    
    .. math::
        
        R_x(\theta) = \begin{bmatrix} c_\theta & -s_\theta & 0 & 0 \\
        s_\theta & c_\theta & 0 & 0 \\
        0 & 0 & 1 & 0 \\
        0 & 0 & 0 & 1
        \end{bmatrix}
    
    :param ry: rotation around Y axis in radians
    :type ry: float"""
    ct = math.cos(rz)
    st = math.sin(rz)
    return Mat([[ct,-st,0,0],[st,ct,0,0],[0,0,1,0],[0,0,0,1]])

def transl(tx,ty=None,tz=None):
    r"""Returns a translation matrix
    
    .. math::
        
        T(t_x, t_y, t_z) = \begin{bmatrix} 0 & 0 & 0 & t_x \\
        0 & 0 & 0 & t_y \\
        0 & 0 & 0 & t_z \\
        0 & 0 & 0 & 1
        \end{bmatrix}
    
    :param tx: translation along the X axis
    :type tx: float
    :param ty: translation along the Y axis
    :type ty: float
    :param tz: translation along the Z axis
    :type tz: float
    """
    if ty is None:
        xx = tx[0]
        yy = tx[1]
        zz = tx[2]
    else:
        xx = tx
        yy = ty
        zz = tz    
    return Mat([[1,0,0,xx],[0,1,0,yy],[0,0,1,zz],[0,0,0,1]])
    
def RelTool(target_pose, x, y, z, rx=0,ry=0,rz=0):
    """Calculates a relative target with respect to the tool coordinates. This procedure has exactly the same behavior as ABB's RelTool instruction.
    X,Y,Z are in mm, W,P,R are in degrees."""
    if type(target_pose) != Mat:
        target_pose = target_pose.Pose()
    new_target = target_pose*transl(x,y,z)*rotx(rx*pi/180)*roty(ry*pi/180)*rotz(rz*pi/180)
    return new_target
    
def Offset(target_pose, x, y, z, rx=0,ry=0,rz=0):
    """Calculates a relative target with respect to the reference frame coordinates.
    X,Y,Z are in mm, W,P,R are in degrees."""
    if type(target_pose) != Mat:
        # item object assumed:
        target_pose = target_pose.Pose()
    if not target_pose.isHomogeneous():
        raise Exception(MatrixError, "Pose matrix is not homogeneous!")
    new_target = transl(x,y,z)*rotx(rx*pi/180.0)*roty(ry*pi/180.0)*rotz(rz*pi/180.0)*target_pose
    return new_target

def point_Zaxis_2_pose(point, zaxis, yaxis_hint1=[0,0,1], yaxis_hint2=[0,1,1]):
    """Returns a pose given the origin as a point, a Z axis and a preferred orientation for the Y axis"""
    pose = eye(4)
    pose.setPos(point)
    pose.setVZ(zaxis)
    yaprox = yaxis_hint1
    if angle3(zaxis, yaprox) < 2*pi/180:
        yaprox = yaxis_hint2
    xaxis = normalize3(cross(yaprox, zaxis))
    yaxis = cross(zaxis, xaxis)
    pose.setVX(xaxis)
    pose.setVY(yaxis)
    return pose
    
def eye(size=4):
    r"""Return the identity matrix
        
    .. math::
        
        T(t_x, t_y, t_z) = \begin{bmatrix} 1 & 0 & 0 & 0 \\
        0 & 1 & 0 & 0 \\
        0 & 0 & 1 & 0 \\
        0 & 0 & 0 & 1
        \end{bmatrix}
        
    :param size: square matrix size (4 by default)
    :type size: int"""
    return Mat([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

def size(matrix,dim=None):
    """Returns the size of a matrix (m,n).
    Dim can be set to 0 to return m (rows) or 1 to return n (columns)
    
    :param matrix: pose
    :type matrix: :class:`.Mat`
    :param dim: dimension
    :type dim: int
    """
    return matrix.size(dim)

def tr(matrix):
    """Returns the transpose of the matrix
    
    :param matrix: pose
    :type matrix: :class:`.Mat`"""
    return matrix.tr()

def invH(matrix):
    """Returns the inverse of a homogeneous matrix
    
    :param matrix: pose
    :type matrix: :class:`.Mat`"""
    return matrix.invH()

def catV(mat1, mat2):
    """Concatenate 2 matrices (vertical concatenation)"""
    return mat1.catV(mat2)

def catH(mat1, mat2):
    """Concatenate 2 matrices (horizontal concatenation)"""
    return mat1.catH(mat2)

def tic():
    """Start a stopwatch timer"""
    import time
    global TICTOC_START_TIME
    TICTOC_START_TIME = time.time()

def toc():
    """Read the stopwatch timer"""
    import time
    if 'TICTOC_START_TIME' in globals():
        elapsed = time.time() - TICTOC_START_TIME
        #print("Elapsed time is " + str(elapsed) + " seconds.")
        return elapsed
    else:
        print("Toc: start time not set")
        return -1

def LoadList(strfile, separator=','):
    """Load data from a CSV file or a TXT file to a list of numbers"""
    import csv
    # Read all CSV data:
    csvdata = []
    with open(strfile) as csvfile:
        csvread = csv.reader(csvfile, delimiter=separator, quotechar='|')
        for row in csvread:
            row_nums = [float(i) for i in row]
            csvdata.append(row_nums)
    return csvdata
    
def LoadMat(strfile, separator=','):
    """Load data from a CSV file or a TXT file to a :class:`.Mat` Matrix"""
    return Mat(LoadList(strfile,separator))
        
#----------------------------------------------------
#------ Pose to xyzrpw and xyzrpw to pose------------
def pose_2_xyzrpw(H):
    """Calculates the equivalent position and euler angles ([x,y,z,r,p,w] array) of the given pose according to the following operation:
    Note: transl(x,y,z)*rotz(w*pi/180)*roty(p*pi/180)*rotx(r*pi/180)
    See also: xyzrpw_2_pose()
    
    :param H: pose
    :type H: :class:`.Mat`"""
    x = H[0,3]
    y = H[1,3]
    z = H[2,3]
    if (H[2,0] > (1.0 - 1e-6)):
        p = -pi/2
        r = 0
        w = math.atan2(-H[1,2],H[1,1])
    elif H[2,0] < -1.0 + 1e-6:
        p = pi/2
        r = 0
        w = math.atan2(H[1,2],H[1,1])
    else:
        p = math.atan2(-H[2,0],sqrt(H[0,0]*H[0,0]+H[1,0]*H[1,0]))
        w = math.atan2(H[1,0],H[0,0])
        r = math.atan2(H[2,1],H[2,2])    
    return [x, y, z, r*180/pi, p*180/pi, w*180/pi]
    
def xyzrpw_2_pose(xyzrpw):
    """Calculates the pose from the position and euler angles ([x,y,z,r,p,w] array)
    The result is the same as calling: H = transl(x,y,z)*rotz(w*pi/180)*roty(p*pi/180)*rotx(r*pi/180)
    See also: pose_2_xyzrpw()"""
    [x,y,z,r,p,w] = xyzrpw
    a = r*pi/180
    b = p*pi/180
    c = w*pi/180
    ca = math.cos(a)
    sa = math.sin(a)
    cb = math.cos(b)
    sb = math.sin(b)
    cc = math.cos(c)
    sc = math.sin(c)    
    H = Mat([[cb*cc, cc*sa*sb - ca*sc, sa*sc + ca*cc*sb, x],[cb*sc, ca*cc + sa*sb*sc, ca*sb*sc - cc*sa, y],[-sb, cb*sa, ca*cb, z],[0,0,0,1]])
    return H

def TxyzRxyz_2_Pose(xyzrpw):
    """Calculates the pose from the position and euler angles ([x,y,z,rx,ry,rz] array)
    The result is the same as calling: H = transl(x,y,z)*rotx(rx)*roty(ry)*rotz(rz)"""
    [x,y,z,rx,ry,rz] = xyzrpw
    srx = math.sin(rx);
    crx = math.cos(rx);
    sry = math.sin(ry);
    cry = math.cos(ry);
    srz = math.sin(rz);
    crz = math.cos(rz);
    H = Mat([[ cry*crz, -cry*srz, sry, x],[crx*srz + crz*srx*sry, crx*crz - srx*sry*srz, -cry*srx, y],[srx*srz - crx*crz*sry, crz*srx + crx*sry*srz, crx*cry, z],[0,0,0,1]])
    return H

def Pose_2_TxyzRxyz(H):
    """Converts a pose to a 6-value target as [x,y,z,rx,ry,rz]:
    H = transl(x,y,z)*rotx(rx)*roty(ry)*rotz(rz).
    
    :param H: pose
    :type H: :class:`.Mat`"""
    x = H[0,3]
    y = H[1,3]
    z = H[2,3]
    a = H[0,0]
    b = H[0,1]
    c = H[0,2]
    d = H[1,2]
    e = H[2,2]
    if c > (1.0 - 1e-6):
        ry1 = pi/2
        rx1 = 0
        rz1 = atan2(H[1,0],H[1,1])
    elif c < (-1.0 + 1e-6):
        ry1 = -pi/2
        rx1 = 0
        rz1 = atan2(H[1,0],H[1,1])
    else:
        sy = c
        cy1 = +sqrt(1-sy*sy)
        sx1 = -d/cy1
        cx1 = e/cy1
        sz1 = -b/cy1
        cz1 =a/cy1
        rx1 = atan2(sx1,cx1)
        ry1 = atan2(sy,cy1)
        rz1 = atan2(sz1,cz1)
    return [x, y, z, rx1, ry1, rz1]
    
def Pose_2_Staubli(H):
    """Converts a pose (4x4 matrix) to a Staubli XYZWPR target
    
    :param H: pose
    :type H: :class:`.Mat`"""
    xyzwpr = Pose_2_TxyzRxyz(H)
    xyzwpr[3] = xyzwpr[3]*180.0/pi
    xyzwpr[4] = xyzwpr[4]*180.0/pi
    xyzwpr[5] = xyzwpr[5]*180.0/pi
    return xyzwpr
    
def Pose_2_Motoman(H):
    """Converts a pose (4x4 matrix) to a Motoman XYZWPR target
    
    :param H: pose
    :type H: :class:`.Mat`"""
    xyzwpr = pose_2_xyzrpw(H)
    return xyzwpr
    
def Pose_2_Fanuc(H):
    """Converts a pose (4x4 matrix) to a Fanuc XYZWPR target
    
    :param H: pose
    :type H: :class:`.Mat`"""
    xyzwpr = pose_2_xyzrpw(H)
    return xyzwpr
    
def Motoman_2_Pose(xyzwpr):
    """Converts a Motoman target to a pose (4x4 matrix)"""
    return xyzrpw_2_pose(xyzwpr)
    
def Pose_2_KUKA(H):
    """Converts a pose (4x4 matrix) to a Kuka target
    
    :param H: pose
    :type H: :class:`.Mat`"""
    x = H[0,3]
    y = H[1,3]
    z = H[2,3]
    if (H[2,0]) > (1.0 - 1e-6):
        p = -pi/2
        r = 0
        w = atan2(-H[1,2],H[1,1])
    elif (H[2,0]) < (-1.0 + 1e-6):
        p = pi/2
        r = 0
        w = atan2(H[1,2],H[1,1])
    else:
        p = atan2(-H[2,0],sqrt(H[0,0]*H[0,0]+H[1,0]*H[1,0]))
        w = atan2(H[1,0],H[0,0])
        r = atan2(H[2,1],H[2,2])
    return [x, y, z, w*180/pi, p*180/pi, r*180/pi]
    
def KUKA_2_Pose(xyzrpw):
    """Converts a KUKA XYZABC target to a pose (4x4 matrix)"""
    [x,y,z,r,p,w] = xyzrpw
    a = r*math.pi/180.0
    b = p*math.pi/180.0
    c = w*math.pi/180.0
    ca = math.cos(a)
    sa = math.sin(a)
    cb = math.cos(b)
    sb = math.sin(b)
    cc = math.cos(c)
    sc = math.sin(c)
    return Mat([[cb*ca, ca*sc*sb - cc*sa, sc*sa + cc*ca*sb, x],[cb*sa, cc*ca + sc*sb*sa, cc*sb*sa - ca*sc, y],[-sb, cb*sc, cc*cb, z],[0.0,0.0,0.0,1.0]])

def Adept_2_Pose(xyzrpw):
    """Converts an Adept XYZRPW target to a pose (4x4 matrix)"""
    [x,y,z,r,p,w] = xyzrpw
    a = r*math.pi/180.0
    b = p*math.pi/180.0
    c = w*math.pi/180.0
    ca = math.cos(a)
    sa = math.sin(a)
    cb = math.cos(b)
    sb = math.sin(b)
    cc = math.cos(c)
    sc = math.sin(c)
    return Mat([[ca*cb*cc - sa*sc, - cc*sa - ca*cb*sc, ca*sb, x],[ca*sc + cb*cc*sa, ca*cc - cb*sa*sc, sa*sb, y],[-cc*sb, sb*sc, cb, z],[0.0,0.0,0.0,1.0]])

def Pose_2_Adept(H):
    """Converts a pose to an Adept target    
    
    :param H: pose
    :type H: :class:`.Mat`"""
    x = H[0,3]
    y = H[1,3]
    z = H[2,3]
    if H[2,2] > (1.0 - 1e-6):
        r = 0
        p = 0
        w = atan2(H[1,0],H[0,0])
    elif H[2,2] < (-1.0 + 1e-6):
        r = 0
        p = pi
        w = atan2(H[1,0],H[1,1])
    else:
        cb=H[2,2]
        sb=+sqrt(1-cb*cb)
        sc=H[2,1]/sb
        cc=-H[2,0]/sb        
        sa=H[1,2]/sb
        ca=H[0,2]/sb        
        r = atan2(sa,ca)
        p = atan2(sb,cb)
        w = atan2(sc,cc)
    return [x, y, z, r*180/pi, p*180/pi, w*180/pi]
    
def Comau_2_Pose(xyzrpw):
    """Converts a Comau XYZRPW target to a pose (4x4 matrix)"""
    return Adept_2_Pose(xyzrpw)
    
def Pose_2_Comau(H):
    """Converts a pose to a Comau target
    
    :param H: pose
    :type H: :class:`.Mat`"""
    return Pose_2_Adept(H)
    
def Pose_2_Nachi(pose):
    """Converts a pose to a Nachi XYZRPW target    
    
    :param pose: pose
    :type pose: :class:`.Mat`"""
    [x,y,z,r,p,w] = pose_2_xyzrpw(pose)
    return [x,y,z,w,p,r]
    
def Nachi_2_Pose(xyzwpr):
    """Converts a Nachi XYZRPW target to a pose (4x4 matrix)"""
    return Fanuc_2_Pose(xyzwpr)
    
def pose_2_quaternion(Ti):
    """Returns the quaternion orientation vector of a pose (4x4 matrix)
    
    :param Ti: pose
    :type Ti: :class:`.Mat`"""
    a=(Ti[0,0])
    b=(Ti[1,1])
    c=(Ti[2,2])
    sign2=1
    sign3=1
    sign4=1
    if (Ti[2,1]-Ti[1,2])<0:
        sign2=-1;
    if (Ti[0,2]-Ti[2,0])<0:
        sign3=-1;
    if (Ti[1,0]-Ti[0,1])<0:
        sign4=-1
    q1=sqrt(max(a+b+c+1,0))/2
    q2=sign2*sqrt(max(a-b-c+1,0))/2
    q3=sign3*sqrt(max(-a+b-c+1,0))/2
    q4=sign4*sqrt(max(-a-b+c+1,0))/2    
    return [q1, q2, q3, q4]

def quaternion_2_pose(qin):
    """Returns the pose orientation matrix (4x4 matrix) from a quaternion orientation vector"""
    qnorm = sqrt(qin[0]*qin[0]+qin[1]*qin[1]+qin[2]*qin[2]+qin[3]*qin[3])
    q = qin
    q[0] = q[0]/qnorm
    q[1] = q[1]/qnorm
    q[2] = q[2]/qnorm
    q[3] = q[3]/qnorm
    pose = Mat([[ 1 - 2*q[2]*q[2] - 2*q[3]*q[3]  ,  2*q[1]*q[2] - 2*q[3]*q[0]  ,  2*q[1]*q[3] + 2*q[2]*q[0]   ,  0],
          [2*q[1]*q[2] + 2*q[3]*q[0]       ,  1 - 2*q[1]*q[1] - 2*q[3]*q[3] , 2*q[2]*q[3] - 2*q[1]*q[0] ,  0],
          [2*q[1]*q[3] - 2*q[2]*q[0]       ,  2*q[2]*q[3] + 2*q[1]*q[0]   ,   1 - 2*q[1]*q[1] - 2*q[2]*q[2], 0],
          [0 , 0 , 0 , 1]])
    return pose


def Pose_2_ABB(H):
    """Converts a pose to an ABB target
    
    :param H: pose
    :type H: :class:`.Mat`"""
    q = pose_2_quaternion(H)
    return [H[0,3],H[1,3],H[2,3],q[0],q[1],q[2],q[3]]

def print_pose_ABB(pose):
    """Displays an ABB RAPID target
    
    :param pose: pose
    :type pose: :class:`.Mat`"""
    q = pose_2_quaternion(pose)
    print('[[%.3f,%.3f,%.3f],[%.6f,%.6f,%.6f,%.6f]]'%(pose[0,3],pose[1,3],pose[2,3],q[0],q[1],q[2],q[3]))

def Pose_2_UR(pose):
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
    
def UR_2_Pose(xyzwpr):
    """Calculate the pose target given a p[x,y,z,rx,ry,rz] cartesian target"""
    x,y,z,w,p,r = xyzwpr
    wpr = [w,p,r]
    angle = norm(wpr)
    cosang = cos(0.5*angle)
    
    if angle == 0.0:
        q234 = [0.0,0.0,0.0]
    else:    
        ratio = sin(0.5*angle)/angle
        q234 = mult3(wpr, ratio)
    
    q1234 = [cosang, q234[0], q234[1], q234[2]]
    pose = quaternion_2_pose(q1234)
    pose.setPos([x,y,z])
    return pose
    
#----------------------------------------------------
#-------- ROBOT MODEL (D-H and D-H M) ---------------

def dh(rz,tx=None,tz=None,rx=None):
    """Returns the Denavit-Hartenberg 4x4 matrix for a robot link.
    calling dh(rz,tx,tz,rx) is the same as using rotz(rz)*transl(tx,0,tx)*rotx(rx)
    calling dh(rz,tx,tz,rx) is the same as calling dh([rz,tx,tz,rx])
    """
    if tx is None: [rz,tx,tz,rx] = rz
        
    crx = math.cos(rx)
    srx = math.sin(rx)
    crz = math.cos(rz)
    srz = math.sin(rz)    
    return Mat( [[crz, -srz*crx,  srz*srx, tx*crz],
                 [srz,  crz*crx, -crz*srx, tx*srz],
                 [  0,      srx,      crx,     tz],
                 [  0,        0,        0,      1]]);

def dhm(rx, tx=None, tz=None, rz=None):
    """Returns the Denavit-Hartenberg Modified 4x4 matrix for a robot link (Craig 1986).
    calling dhm(rx,tx,tz,rz) is the same as using rotx(rx)*transl(tx,0,tx)*rotz(rz)
    calling dhm(rx,tx,tz,rz) is the same as calling dhm([rx,tx,tz,rz])
    """
    if tx is None: [rx,tx,tz,rz] = rx
        
    crx = math.cos(rx)
    srx = math.sin(rx)
    crz = math.cos(rz)
    srz = math.sin(rz)    
    return Mat([[crz,        -srz,    0,      tx],
                [crx*srz, crx*crz, -srx, -tz*srx],
                [srx*srz, crz*srx,  crx,  tz*crx],
                [      0,       0,    0,       1]]);

def joints_2_angles(jin, type):
    """Converts the robot joints into angles between links depending on the type of the robot."""
    jout = jin
    if type == 2:
        jout[2] = -jin[1] - jin[2]
        jout[3] = -jin[3]
        jout[4] = -jin[4]
        jout[5] = -jin[5]
    elif type == 3:
        jout[2] = -jin[2]
        jout[3] = -jin[3]
        jout[4] = -jin[4]
        jout[5] = -jin[5]
    elif type == 4:
        jout[2] = +jin[1] + jin[2]
    return jout

def angles_2_joints(jin, type):
    """Converts the angles between links into the robot joint space depending on the type of the robot."""
    jout = jin
    if type == 2:
        jout[2] = -jin[1] - jin[2]
        jout[3] = -jin[3]
        jout[4] = -jin[4]
        jout[5] = -jin[5]
    elif type == 3:
        jout[2] = -jin[2]
        jout[3] = -jin[3]
        jout[4] = -jin[4]
        jout[5] = -jin[5]
    return jout
                
#----------------------------------------------------
#-------- Useful geometric tools ---------------                   

def norm(p):
    """Returns the norm of a 3D vector"""
    return sqrt(p[0]*p[0] + p[1]*p[1] + p[2]*p[2])
   
def normalize3(a):
    """Returns the unitary vector"""
    norminv = 1.0/norm(a)
    return [a[0]*norminv,a[1]*norminv,a[2]*norminv]
    
def cross(a, b):
    """Returns the cross product of two 3D vectors"""
    c = [a[1]*b[2] - a[2]*b[1],
         a[2]*b[0] - a[0]*b[2],
         a[0]*b[1] - a[1]*b[0]]
    return c

def dot(a,b):
    """Returns the dot product of two 3D vectors"""
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]
    
def angle3(a,b):
    """Returns the angle in radians of two 3D vectors"""
    return acos(dot(normalize3(a),normalize3(b)))

def pose_angle(pose):
    """Returns the angle in radians of a 4x4 matrix pose
    
    :param pose: pose
    :type pose: :class:`.Mat`"""
    cos_ang = (pose[0,0]+pose[1,1]+pose[2,2]-1)/2
    cos_ang = min(max(cos_ang,-1),1)
    return acos(cos_ang)

def pose_angle_between(pose1, pose2):
    """Returns the angle in radians between two poses (4x4 matrix pose)"""
    return pose_angle(invH(pose1)*pose2)
    
def mult3(v,d):
    """Multiplies a 3D vector to a scalar"""
    return [v[0]*d, v[1]*d, v[2]*d]

def subs3(a,b):
    """Subtracts two 3D vectors c=a-b"""
    return [a[0]-b[0],a[1]-b[1],a[2]-b[2]]

def add3(a,b):
    """Adds two 3D vectors c=a+b"""
    return [a[0]+b[0],a[1]+b[1],a[2]+b[2]]
    
def distance(a,b):
    """Calculates the distance between two points"""
    return norm(subs3(a,b))


def intersect_line_2_plane(pline,vline,pplane,vplane):
    """Calculates the intersection betweeen a line and a plane"""
    D = -dot(vplane,pplane)
    k = -(D+dot(vplane,pline))/dot(vplane,vline)
    p = add3(pline,mult3(vline,k))
    return p

def proj_pt_2_plane(point,planepoint,planeABC):
    """Projects a point to a plane"""
    return intersect_line_2_plane(point,planeABC,planepoint,planeABC);

def proj_pt_2_line(point, paxe, vaxe):
    """Projects a point to a line"""
    vpaxe2point = subs3(point,paxe)
    dist = dot(vaxe,vpaxe2point)/dot(vaxe,vaxe)
    return add3(paxe,mult3(vaxe,dist))

def fitPlane(points):
    """Best fits a plane to a cloud of points"""
    import numpy as np
    XYZ = np.array(points)    
    [rows,cols] = XYZ.shape
    # Set up constraint equations of the form  AB = 0,
    # where B is a column vector of the plane coefficients
    # in the form b(1)*X + b(2)*Y +b(3)*Z + b(4) = 0.
    p = (np.ones((rows,1)))
    AB = np.hstack([XYZ,p])
    [u, d, v] = np.linalg.svd(AB,0)        
    B = v[3,:]                  # Solution is last column of v.
    nn = np.linalg.norm(B[0:3])
    B = B / nn
    pplane = [0,0,-(B[3]/B[2])]
    vplane = B[0:3].tolist()
    return pplane, vplane  

                
#----------------------------------------------------
#--------       Mat matrix class      ---------------

class MatrixError(Exception):
    """ An exception class for Matrix """
    pass

class Mat(object):
    """A pose matrix object. A pose is a 4x4 matrix that represents the position and orientation of one reference frame with respect to another one, in the 3D space.
    This type is meant to be used for robotics."""
    
    def __init__(self, rows=None, ncols=None):
        if ncols is None:
            if rows is None:
                m = 4
                n = 4
                self.rows = [[0]*n for x in range(m)]
            else:
                if isinstance(rows,Mat):
                    rows = rows.copy().rows
                m = len(rows)
                transpose = 0
                if not isinstance(rows[0],list):
                    rows = [rows]
                    transpose = 1
                n = len(rows[0])
                if any([len(row) != n for row in rows[1:]]):# Validity check
                    raise Exception(MatrixError, "inconsistent row length")
                self.rows = rows
                if transpose:
                    self.rows = [list(item) for item in zip(*self.rows)]
        else:
            m = rows
            n = ncols
            self.rows = [[0]*n for x in range(m)]
    def copy(self):
        sz = self.size()
        newmat = Mat(sz[0],sz[1])
        for i in range(sz[0]):
            for j in range(sz[1]):
                newmat[i,j] = self[i,j]
        return newmat
        
    def __getitem__(self, idx):
        if isinstance(idx,int):#integer A[1]
            return tr(Mat(self.rows[idx]))
        elif isinstance(idx,slice):#one slice: A[1:3]
            return Mat(self.rows[idx])
        else:#two slices: A[1:3,1:3]
            idx1 = idx[0]
            idx2 = idx[1]
            if isinstance(idx1,int) and isinstance(idx2,int):
                return self.rows[idx1][idx2]
            matsize =self.size();
            if isinstance(idx1,slice):
                indices1 = idx1.indices(matsize[0])
                rg1 = range(*indices1)
            else: #is int
                rg1 = range(idx1,idx1+1)
            if isinstance(idx2,slice):
                indices2 = idx2.indices(matsize[1])
                rg2 = range(*indices2)
            else: #is int
                rg2 = range(idx2,idx2+1)                    
            #newm = int(abs((rg1.stop-rg1.start)/rg1.step))
            #newn = int(abs((rg2.stop-rg2.start)/rg2.step))
            newm = rg1
            newn = rg2
            newmat = Mat(len(newm),len(newn))
            cm = 0
            for i in rg1:
                cn = 0
                for j in rg2:
                    newmat.rows[cm][cn] = self.rows[i][j]
                    cn = cn + 1
                cm = cm + 1
            return newmat
    def __setitem__(self, idx, item):
        if isinstance(item,float) or isinstance(item,int):
            item = Mat([[item]])
        elif isinstance(item, list):
            item = Mat(item)
        
        matsize = self.size();
        if isinstance(idx,int):#integer A[1]
            idx1 = idx
            idx2 = 0
            #raise Exception(MatrixError, "Cannot set item. Use [i,:] instead.")
            #self.rows[idx] = item
        elif isinstance(idx,slice):#one slice: A[1:3]
            # raise Exception(MatrixError, "Cannot set item. Use [a:b,:] instead.")
            idx1 = idx
            idx2 = 0
        else:
            idx1 = idx[0]
            idx2 = idx[1]

        # at this point we have two slices: example A[1:3,1:3]       
        if isinstance(idx1,slice):
            indices1 = idx1.indices(matsize[0])
            rg1 = range(*indices1)
        else: #is int
            rg1 = range(idx1,idx1+1)
        if isinstance(idx2,slice):
            indices2 = idx2.indices(matsize[1])
            rg2 = range(*indices2)
        else: #is int
            rg2 = range(idx2,idx2+1)
        #newm = int(abs((rg1.stop-rg1.start)/rg1.step))
        #newn = int(abs((rg2.stop-rg2.start)/rg2.step))
        newm = rg1
        newn = rg2
        itmsz = item.size();
        if len(newm) != itmsz[0] or len(newn) != itmsz[1]:
            raise Exception(MatrixError, "Submatrix indices does not match the new matrix sizes",itmsz[0],"x",itmsz[1],"<-",newm,"x",newn)
        #newmat = Mat(newm,newn)
        cm = 0
        for i in rg1:
            cn = 0
            for j in rg2:
                self.rows[i][j] = item.rows[cm][cn]
                cn = cn + 1
            cm = cm + 1        
        
    def __str__(self):
        #s='\n [ '.join([(', '.join([str(item) for item in row])+' ],') for row in self.rows])
        s='\n [ '.join([(', '.join(['%.3f'%item for item in row])+' ],') for row in self.rows])
        return '[[ ' + s[:-1] + ']\n'

    def __repr__(self):
        s=str(self)
        rank = str(self.size())
        rep="Matrix: %s\n%s" % (rank,s)
        return rep
                         
    def tr(self):
        """Returns the transpose of the matrix"""
        mat = Mat([list(item) for item in zip(*self.rows)])      
        return mat

    def size(self,dim=None):
        """Returns the size of a matrix (m,n).
        Dim can be set to 0 to return m (rows) or 1 to return n (columns)"""
        m = len(self.rows)
        n = len(self.rows[0])
        if dim is None:
            return (m, n)
        elif dim==0:
            return m
        elif dim==1:
            return n
        else:
            raise Exception(MatrixError, "Invalid dimension!")
        
    def catV(self,mat2):
        """Concatenate with another matrix (vertical concatenation)"""
        if not isinstance(mat2, Mat):
            raise Exception(MatrixError, "Concatenation must be performed with 2 matrices")
        sz1 = self.size()
        sz2 = mat2.size()
        if sz1[1] != sz2[1]:
            raise Exception(MatrixError, "Horizontal size of matrices does not match")
        newmat = Mat(sz1[0]+sz2[0],sz1[1])
        newmat[0:sz1[0],:] = self
        newmat[sz1[0]:,:] = mat2        
        return newmat
    
    def catH(self,mat2):
        """Concatenate with another matrix (horizontal concatenation)"""
        if not isinstance(mat2, Mat):
            raise Exception(MatrixError, "Concatenation must be performed with 2 matrices")
        sz1 = self.size()
        sz2 = mat2.size()
        if sz1[0] != sz2[0]:
            raise Exception(MatrixError, "Horizontal size of matrices does not match")
        newmat = Mat(sz1[0],sz1[1]+sz2[1])
        newmat[:,:sz1[1]] = self
        newmat[:,sz1[1]:] = mat2   
        return newmat
    def __eq__(self, mat):
        """Test equality"""
        return (mat.rows == self.rows)
        
    def __add__(self, mat):
        """Add a matrix to this matrix and
        return the new matrix. It doesn't modify
        the current matrix"""
        if isinstance(mat,int) or isinstance(mat,float):
            m, n = self.size()     
            result = Mat(m, n)        
            for x in range(m):
                for y in range(n):
                    result.rows[x][y] = self.rows[x][y] + mat
            return result
        sz = self.size()
        m = sz[0]
        n = sz[1]
        ret = Mat(m,n)
        if sz != mat.size():
            raise Exception(MatrixError, "Trying to add matrixes of varying size!")   
        for x in range(m):
            row = [sum(item) for item in zip(self.rows[x], mat.rows[x])]
            ret.rows[x] = row
        return ret

    def __sub__(self, mat):
        """Subtract a matrix from this matrix and
        return the new matrix. It doesn't modify
        the current matrix"""
        if isinstance(mat,int) or isinstance(mat,float):
            m, n = self.size()     
            result = Mat(m, n)        
            for x in range(m):
                for y in range(n):
                    result.rows[x][y] = self.rows[x][y] - mat
            return result
        sz = self.size()
        m = sz[0]
        n = sz[1]
        ret = Mat(m,n)
        if sz != mat.size():
            raise Exception(MatrixError, "Trying to add matrixes of varying size!")    
        for x in range(m):
            row = [item[0]-item[1] for item in zip(self.rows[x], mat.rows[x])]
            ret.rows[x] = row
        return ret

    def __mul__(self, mat):
        """Multiply a matrix with this matrix and
        return the new matrix. It doesn't modify
        the current matrix"""
        if isinstance(mat,int) or isinstance(mat,float):
            m, n = self.size()     
            mulmat = Mat(m, n)        
            for x in range(m):
                for y in range(n):
                    mulmat.rows[x][y] = self.rows[x][y]*mat
            return mulmat
        if isinstance(mat,list):#case of a matrix times a vector            
            szvect = len(mat)
            m = self.size(0);
            matvect = Mat(mat)            
            if szvect + 1 == m:
                vectok = catV(matvect,Mat([[1]]))
                result = self*vectok
                return (result[:-1,:]).tr().rows[0]
            elif szvect == m:
                result = self*Mat(matvect)
                return result.tr().rows[0]
            else:
                raise Exception(MatrixError, "Invalid product")       
        else:
            matm, matn = mat.size()
            m, n = self.size()
            if (n != matm):
                raise Exception(MatrixError, "Matrices cannot be multipled!")        
            mat_t = mat.tr()
            mulmat = Mat(m, matn)        
            for x in range(m):
                for y in range(mat_t.size(0)):
                    mulmat.rows[x][y] = sum([item[0]*item[1] for item in zip(self.rows[x], mat_t.rows[y])])
            return mulmat
    
    def eye(self, m=4):
        """Make identity matrix of size (mxm)"""
        rows = [[0]*m for x in range(m)]
        idx = 0        
        for row in rows:
            row[idx] = 1
            idx += 1
        return Mat(rows)

    def isHomogeneous(self):
        """returns 1 if it is a Homogeneous matrix"""
        m,n = self.size()
        if m != 4 or n != 4:
            return False
        #if self[3,:] != Mat([[0.0,0.0,0.0,1.0]]):
        #    return False
        test = self[0:3,0:3];
        test = test*test.tr()
        test[0,0] = test[0,0] - 1.0
        test[1,1] = test[1,1] - 1.0
        test[2,2] = test[2,2] - 1.0
        zero = 0.0
        for x in range(3):
            for y in range(3):
                zero = zero + abs(test[x,y])
        if zero > 1e-4:
            return False
        return True

    def RelTool(self, x, y, z, rx=0,ry=0,rz=0):
        """Calculates a target relative with respect to the tool coordinates.
        X,Y,Z are in mm, W,P,R are in degrees."""
        return RelTool(self, x, y, z, rx, ry, rz)
    
    def Offset(self, x, y, z, rx=0,ry=0,rz=0):
        """Calculates a target relative with respect to the reference frame coordinates.
        X,Y,Z are in mm, W,P,R are in degrees."""
        return Offset(self, x, y, z, rx, ry, rz)        
    
    def invH(self):
        """Calculates the inverse of a homogeneous matrix"""
        if not self.isHomogeneous():
            raise Exception(MatrixError, "Pose matrix is not homogeneous. invH() can only compute the inverse of a homogeneous matrix")
        Hout = self.tr()
        Hout[3,0:3] = Mat([[0,0,0]]);
        Hout[0:3,3] = (Hout[0:3,0:3]*self[0:3,3])*(-1)
        return Hout
        
    def tolist(self):
        """Returns the first column vector of the matrix as a list"""
        return tr(self).rows[0]
        
    def Pos(self):
        """Returns the position of a pose (assumes that a 4x4 homogeneous matrix is being used)"""
        return self[0:3,3].tolist()
        
    def VX(self):
        """Returns the X vector of a pose (assumes that a 4x4 homogeneous matrix is being used)"""
        return self[0:3,0].tolist()
        
    def VY(self):
        """Returns the Y vector of a pose (assumes that a 4x4 homogeneous matrix is being used)"""
        return self[0:3,1].tolist()
        
    def VZ(self):
        """Returns the Z vector of a pose (assumes that a 4x4 homogeneous matrix is being used)"""
        return self[0:3,2].tolist()
        
    def setPos(self, newpos):
        """Sets the XYZ position of a pose (assumes that a 4x4 homogeneous matrix is being used)"""
        self[0,3] = newpos[0]
        self[1,3] = newpos[1]
        self[2,3] = newpos[2]
        
    def setVX(self, v_xyz):
        """Sets the VX vector of a pose, which is the first column of a homogeneous matrix (assumes that a 4x4 homogeneous matrix is being used)"""
        v_xyz = normalize3(v_xyz)
        self[0,0] = v_xyz[0]
        self[1,0] = v_xyz[1]
        self[2,0] = v_xyz[2]
        
    def setVY(self, v_xyz):
        """Sets the VY vector of a pose, which is the first column of a homogeneous matrix (assumes that a 4x4 homogeneous matrix is being used)"""
        v_xyz = normalize3(v_xyz)
        self[0,1] = v_xyz[0]
        self[1,1] = v_xyz[1]
        self[2,1] = v_xyz[2]
        
    def setVZ(self, v_xyz):
        """Sets the VZ vector of a pose, which is the first column of a homogeneous matrix (assumes that a 4x4 homogeneous matrix is being used)"""
        v_xyz = normalize3(v_xyz)
        self[0,2] = v_xyz[0]
        self[1,2] = v_xyz[1]
        self[2,2] = v_xyz[2]    
        
    def SaveMat(self, strfile):
        """Save :class:`.Mat` Matrix to a CSV or TXT file"""
        sz = self.size()
        m = sz[0]
        n = sz[1]
        file = open(strfile, 'w')
        for j in range(n):
            for i in range(m):
                file.write('%.6f ' % self.rows[i][j])          
            file.write('\n')                
        file.close()
    
#-------------------------------------------------------
# FTP TRANSFER Tools
def RemoveFileFTP(ftp, filepath):
    """Delete a file on a remote server."""
    import ftplib    
    try:
        ftp.delete(filepath)
    except ftplib.all_errors as e:
        import sys
        print('POPUP: Could not remove file {0}: {1}'.format(filepath, e))
        sys.stdout.flush()

def RemoveDirFTP(ftp, path):
    """Recursively delete a directory tree on a remote server."""
    import ftplib    
    wd = ftp.pwd()
    try:
        names = ftp.nlst(path)
    except ftplib.all_errors as e:
        # some FTP servers complain when you try and list non-existent paths
        print('RemoveDirFTP: Could not remove folder {0}: {1}'.format(path, e))
        return

    for name in names:
        if os.path.split(name)[1] in ('.', '..'): continue
        print('RemoveDirFTP: Checking {0}'.format(name))
        try:
            ftp.cwd(path+'/'+name)  # if we can cwd to it, it's a folder
            ftp.cwd(wd)  # don't try a nuke a folder we're in
            RemoveDirFTP(ftp, path+'/'+name)
        except ftplib.all_errors:
            ftp.delete(path+'/'+name)
            #RemoveFileFTP(ftp, name)

    try:
        ftp.rmd(path)
    except ftplib.all_errors as e:
        print('RemoveDirFTP: Could not remove {0}: {1}'.format(path, e))

def UploadDirFTP(localpath, server_ip, remote_path, username, password):
    """Upload a folder to a robot through FTP recursively"""
    import ftplib
    import os
    import sys
    main_folder = os.path.basename(os.path.normpath(localpath))    
    print("POPUP: <p>Connecting to <strong>%s</strong> using user name <strong>%s</strong> and password ****</p><p>Please wait...</p>" % (server_ip, username))
    sys.stdout.flush()
    try:
        myFTP = ftplib.FTP(server_ip, username, password)
        print('Connection established')
    except:
        error_str = sys.exc_info()[1]
        print("POPUP: <font color=\"red\">Connection to %s failed: <p>%s</p></font>" % (server_ip,error_str))
        sys.stdout.flush()
        pause(4)
        return False

    remote_path_prog = remote_path + '/' + main_folder
    myPath = r'%s' % localpath
    print("POPUP: Connected. Deleting existing files on %s..." % remote_path_prog)
    sys.stdout.flush()
    RemoveDirFTP(myFTP, remote_path_prog)
    print("POPUP: Connected. Uploading program to %s..." % server_ip)
    sys.stdout.flush()
    try:
        myFTP.cwd(remote_path)
        myFTP.mkd(main_folder)
        myFTP.cwd(remote_path_prog)
    except:
        error_str = sys.exc_info()[1]
        print("POPUP: <font color=\"red\">Remote path not found or can't be created: %s</font>" % (remote_path))
        sys.stdout.flush()
        pause(4)
        #contin = mbox("Remote path\n%s\nnot found or can't create folder.\n\nChange path and permissions and retry." % remote_path)
        return False
        
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
    myFTP.close()
    return True
    
def UploadFileFTP(file_path_name, server_ip, remote_path, username, password):
    """Upload a file to a robot through FTP"""
    filepath = getFileDir(file_path_name)
    filename = getBaseName(file_path_name)
    import ftplib
    import os
    import sys
    print("POPUP: <p>Connecting to <strong>%s</strong> using user name <strong>%s</strong> and password ****</p><p>Please wait...</p>" % (server_ip, username))
    sys.stdout.flush()
    try:
        myFTP = ftplib.FTP(server_ip, username, password)
    except:
        error_str = sys.exc_info()[1]
        print("POPUP: <font color=\"red\">Connection to %s failed: <p>%s</p></font>" % (server_ip,error_str))
        sys.stdout.flush()
        pause(4)
        return False

    remote_path_prog = remote_path + '/' + filename
    print("POPUP: Connected. Deleting remote file %s..." % remote_path_prog)
    sys.stdout.flush()
    RemoveFileFTP(myFTP, remote_path_prog)
    print("POPUP: Connected. Uploading program to %s..." % server_ip)
    sys.stdout.flush()
    try:
        myFTP.cwd(remote_path)
    except:
        error_str = sys.exc_info()[1]
        print("POPUP: <font color=\"red\">Remote path not found or can't be created: %s</font>" % (remote_path))
        sys.stdout.flush()
        pause(4)
        #contin = mbox("Remote path\n%s\nnot found or can't create folder.\n\nChange path and permissions and retry." % remote_path)
        return False
        
    def uploadThis(localfile, filename):
        print('  Sending file: %s' % localfile)
        print("POPUP: Sending file: %s" % filename)
        sys.stdout.flush()
        fh = open(localfile, 'rb')
        myFTP.storbinary('STOR %s' % filename, fh)
        fh.close()

    uploadThis(file_path_name, filename)
    myFTP.close()
    return True

def UploadFTP(program, robot_ip, remote_path, ftp_user, ftp_pass):
    """Upload a program or a list of programs to the robot through FTP provided the connection parameters"""
    # Iterate through program list if it is a list of files
    if isinstance(program, list):
        if len(program) == 0:
            print('POPUP: Nothing to transfer')
            return
        for prog in program:
            UploadFTP(prog, robot_ip, remote_path, ftp_user, ftp_pass)
        return
    
    import os
    if os.path.isfile(program):
        print('Sending program file %s...' % program)
        UploadFileFTP(program, robot_ip, remote_path, ftp_user, ftp_pass)
    else:
        print('Sending program folder %s...' % program)
        UploadDirFTP(program, robot_ip, remote_path, ftp_user, ftp_pass)    



#----------------------------------------------------
#--------       MessageBox class      ---------------
# inspired from:
# http://stackoverflow.com/questions/10057672/correct-way-to-implement-a-custom-popup-tkinter-dialog-box

try:
    ## Python 2.X
    import Tkinter as tkinter
    import tkFileDialog as filedialog
except ImportError:
    ## Python 3.X
    from tkinter import filedialog

def getOpenFile():
    """Pop up a file dialog window to select a file to open."""
    root = tkinter.Tk()
    root.withdraw()
    file_path = filedialog.askopenfilename()
    # same as: file_path = tkinter.filedialog.askopenfilename()
    return file_path
    
def getSaveFile(strdir='C:\\', strfile = 'file.txt', strtitle='Save file as ...'):
    """Pop up a file dialog window to select a file to save."""
    options = {}
    options['initialdir'] = strdir
    options['title'] = strtitle
    #options['defaultextension'] = '.txt'
    #options['filetypes'] = [('all files', '.*'), ('text files', '.txt')]
    options['initialfile'] = strfile
    #options['parent'] = root
    root = tkinter.Tk()
    root.withdraw()
    file_path = filedialog.asksaveasfile(**options)
    #same as: file_path = tkinter.filedialog.asksaveasfile(**options)
    return file_path

class MessageBox(object):

    def __init__(self, msg, b1, b2, frame, t, entry):

        root = self.root = tkinter.Tk()
        root.title('Message')
        self.msg = str(msg)
        # ctrl+c to copy self.msg
        root.bind('<Control-c>', func=self.to_clip)
        # remove the outer frame if frame=False
        if not frame: root.overrideredirect(True)
        # default values for the buttons to return
        self.b1_return = True
        self.b2_return = False
        # if b1 or b2 is a tuple unpack into the button text & return value
        if isinstance(b1, tuple): b1, self.b1_return = b1
        if isinstance(b2, tuple): b2, self.b2_return = b2
        # main frame
        frm_1 = tkinter.Frame(root)
        frm_1.pack(ipadx=2, ipady=2)
        # the message
        message = tkinter.Label(frm_1, text=self.msg)
        message.pack(padx=8, pady=8)
        # if entry=True create and set focus
        if entry is not None:
            if entry == True:
                entry = ''
            self.entry = tkinter.Entry(frm_1)
            self.entry.pack()
            self.entry.insert(0, entry)
            self.entry.focus_set()
        # button frame
        frm_2 = tkinter.Frame(frm_1)
        frm_2.pack(padx=4, pady=4)
        # buttons
        btn_1 = tkinter.Button(frm_2, width=8, text=b1)
        btn_1['command'] = self.b1_action
        btn_1.pack(side='left')
        if not entry: btn_1.focus_set()
        btn_2 = tkinter.Button(frm_2, width=8, text=b2)
        btn_2['command'] = self.b2_action
        btn_2.pack(side='left')
        # the enter button will trigger the focused button's action
        btn_1.bind('<KeyPress-Return>', func=self.b1_action)
        btn_2.bind('<KeyPress-Return>', func=self.b2_action)
        # roughly center the box on screen
        # for accuracy see: http://stackoverflow.com/a/10018670/1217270
        root.update_idletasks()
        xp = (root.winfo_screenwidth() // 2) - (root.winfo_width() // 2)
        yp = (root.winfo_screenheight() // 2) - (root.winfo_height() // 2)
        geom = (root.winfo_width(), root.winfo_height(), xp, yp)
        root.geometry('{0}x{1}+{2}+{3}'.format(*geom))
        # call self.close_mod when the close button is pressed
        root.protocol("WM_DELETE_WINDOW", self.close_mod)
        # a trick to activate the window (on windows 7)
        root.deiconify()
        # if t is specified: call time_out after t seconds
        if t: root.after(int(t*1000), func=self.time_out)

    def b1_action(self, event=None):
        try: x = self.entry.get()
        except AttributeError:
            self.returning = self.b1_return
            self.root.quit()
        else:
            if x:
                self.returning = x
                self.root.quit()

    def b2_action(self, event=None):
        self.returning = self.b2_return
        self.root.quit()

    # remove this function and the call to protocol
    # then the close button will act normally
    def close_mod(self):
        pass

    def time_out(self):
        try: x = self.entry.get()
        except AttributeError: self.returning = None
        else: self.returning = x
        finally: self.root.quit()

    def to_clip(self, event=None):
        self.root.clipboard_clear()
        self.root.clipboard_append(self.msg)       
        
        
def mbox(msg, b1='OK', b2='Cancel', frame=True, t=False, entry=None):
    """Create an instance of MessageBox, and get data back from the user.
    
    :param msg: string to be displayed
    :type msg: str
    :param b1: left button text, or a tuple (<text for button>, <to return on press>)
    :type b1: str, tuple
    :param b2: right button text, or a tuple (<text for button>, <to return on press>)
    :type b2: str, tuple
    :param frame: include a standard outerframe: True or False
    :type frame: bool
    :param t: time in seconds (int or float) until the msgbox automatically closes
    :type t: int, float
    :param entry: include an entry widget that will provide its contents returned. Provide text to fill the box
    :type entry: None, bool, str
    Examples:
      mbox('Enter your name', entry=True)
      mbox('Enter your name', entry='default')
      mbox('Male or female?', ('male', 'm'), ('female', 'f'))
      mbox('Process dones')
    """
    msgbox = MessageBox(msg, b1, b2, frame, t, entry)
    msgbox.root.mainloop()
    # the function pauses here until the mainloop is quit
    msgbox.root.destroy()
    return msgbox.returning
