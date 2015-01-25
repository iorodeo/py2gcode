"""

Copyright 2013 IO Rodeo Inc. 

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

"""
from __future__ import print_function
import math

FT_PER_IN =  1.0/12.0

TAIG_BELT_POS_TO_RPM = {
        'A': 1100.0,
        'B': 1900.0,
        'C': 2900.0, 
        'D': 4300.0,
        'E': 6500.0,
        'F': 10500.0,
        }


def getBosch1617Setting(rpm):
    """
    Get bosch 1617 router setting for given rmp.
    """
    minSetting = 1.0
    maxSetting = 6.0
    minSpeed = 8000.0
    maxSpeed = 25000.0
    slope = (maxSetting - minSetting)/(maxSpeed - minSpeed)
    offset = minSetting - slope*minSpeed
    return slope*rpm + offset
   

def getSFM(rpm, toolDiam):
    """
    Calculates the surface feet per minute
    """
    sfm = rpm*math.pi*toolDiam*FT_PER_IN
    return sfm


def getRPM(sfm, toolDiam):
    """
    Computes the tool rpm.

    args:

    sfm      = surface feet per minute
    toolDiam = tool diameter (inches)

    """
    rpm = sfm/(math.pi*toolDiam*FT_PER_IN)
    return rpm


def getFeedRate(rpm, chipLoad, numTeeth):
    """
    Calculates the feedrate in inches per minute

    args:

    rpm      = spindle speed
    chipLoad = chip load (inches per tooth) 
    numTeeth = number of teeth on tool
    """
    feedRate = rpm*chipLoad*numTeeth
    return feedRate


def getFeedRateAndRPM(sfm,chipLoad,toolDiam,numTeeth):
    """
    Computes the rpm feedrate in inches per minute.

    args:

    sfm       = surface feet per minute
    chipLoad  = chip load (inches per tooth)
    toolDiam  = tool diameter (inches)
    numTeeth  = number of teeth on tool

    """
    rpm = getRPM(sfm,toolDiam)
    feedRate = rpm*chipLoad*numTeeth
    return feedRate, rpm


# -----------------------------------------------------------------------------
if __name__ == '__main__':


    
    if 0:
        sfm = 500
        toolDiam = 0.25
        rpm = getRPM(sfm, toolDiam)

        print()
        print('test getRPM')
        print('-'*40)
        print('SFM:      ', sfm)
        print('toolDiam: ', toolDiam)
        print()
        print('RPM:      ', rpm)
        print()
        print()

        rpm = TAIG_BELT_POS_TO_RPM['C'] 
        chipLoad = 0.003
        numTeeth = 2
        feedRate = getFeedRate(rpm, chipLoad, numTeeth)
        print('test getFeedRate')
        print('-'*40)
        print('RPM:      ', rpm)
        print('chipLoad: ', chipLoad)
        print('numTeeth: ', numTeeth)
        print()
        print('feedRate: ', feedRate)
        print()
        print()

        rpm = TAIG_BELT_POS_TO_RPM['C'] 
        toolDiam  = 0.25
        sfm = getSFM(rpm, toolDiam)
        print('test getSFM')
        print('-'*40)
        print('RPM:      ', rpm)
        print('toolDiam: ', toolDiam)
        print()
        print('SFM:      ', sfm)
        print()
        print()

        sfm = 500
        chipLoad = 0.003
        toolDiam = 0.25
        numTeeth = 2
        feedRate, rpm = getFeedRateAndRPM(sfm,chipLoad,toolDiam,numTeeth)
        print('test getFeedRateAndRPM')
        print('-'*40)
        print('SFM:      ', sfm)
        print('chipLoad: ', chipLoad)
        print('toolDiam: ', toolDiam)
        print('numTeeth: ', numTeeth)
        print()
        print('feedRate: ', feedRate)
        print('RPM:      ', rpm)
        print()

    if 1:
        rpm = 9100.0
        setting = getBosch1617Setting(rpm)
        print('rpm: {}'.format(rpm))
        print('setting: {}'.format(setting))


