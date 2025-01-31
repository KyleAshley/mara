#!/usr/bin/env python
import subprocess
import os


objCount = 0
objList = []
selectedObject = 0
tablePos = []
tableSize= []

def runOPE(wait=True):
    print "Starting OPE"
    ope_process = subprocess.Popen("./ope-new", shell=True, cwd="/home/carrt/Dropbox/catkin_ws/src/mara/OPE-Release/", preexec_fn=os.setsid)

    if wait:
        ret = ope_process.wait()
        print "Result", ret

    return ret

def loadOPEResults():
    global objCount
    global objList
    global selectedObject
    global tablePos
    global tableSize

    ope_results = open("/home/carrt/Dropbox/catkin_ws/src/mara/OPE-Release/OPE-Results.txt")

    objCount = int(ope_results.readline())
    selectedObject = int(ope_results.readline())

    if objCount > 0:
        tablePos = [float(x) for x in ope_results.readline().split()]
        tableSize = [float(x) for x in ope_results.readline().split()]
        tableSize[0] += 0.3     # scale the table in the X direction to prevent underhand gripping

        # GLOBAL TABLE ADJUSTMENT

        tablePos[0] = tablePos[0]  # X
        tablePos[1] = tablePos[1]  # Y
        tablePos[2] = tablePos[2]  -0.12# Z

        for k in range(objCount):
            ope_results.readline()
            temp_objPos = [float(x) for x in ope_results.readline().split()]
            temp_objSize = [float(x) for x in ope_results.readline().split()]
            temp_objRot = [float(x) for x in ope_results.readline().split()]

            # GLOBAL OBJECT ADJUSTMENT
            temp_objPos[0] = temp_objPos[0] # X
            temp_objPos[1] = temp_objPos[1] # Y
            temp_objPos[2] = temp_objPos[2] - 0.01# Z

            objList.append({'objNumber':k,
                            'objPos':temp_objPos,
                            'objSize':temp_objSize,
                            'objRot':temp_objRot})

    ope_results.close()

def showOPEResults():
    subprocess.Popen("ristretto output.png", shell=True,
                     cwd="/home/carrt/Dropbox/catkin_ws/src/mara/OPE-Release/")
