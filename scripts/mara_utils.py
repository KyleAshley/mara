#!/usr/bin/env python
import os, sys

def inRange(num, lower, upper):
	return (num >= lower) and (num <= upper)

def isNearVal(num, val, tolerance):
	return (num >= val - tolerance) and (num <= val + tolerance)

def removePCDs(path):
    for root, dirs, files in os.walk(path):
        for currentFile in files:
            exts=('.pcd')
            if any(currentFile.lower().endswith(ext) for ext in exts):
                print "Removing file: " + currentFile
                os.remove(os.path.join(root, currentFile))