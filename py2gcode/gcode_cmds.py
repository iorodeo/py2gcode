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

# Base classes
# -----------------------------------------------------------------------------

class GCodeCmd(object):
    """ 
    Base class for all gcode commands.
    """

    def __init__(self):
        self.motionDict = {}
        self.code = ';NONE'
        self.comment = False 
        self.commentStr = ''

    def __str__(self):
        cmdList= self.getCmdList()
        if self.comment and self.commentStr:
            cmdList.append('({0})'.format(self.commentStr))
        return ' '.join(cmdList)

    def getCmdList(self):
        return [self.code]


class GCodeSingleArgCmd(GCodeCmd):
    """
    Base class for gcode commands with a single argument.
    """

    def __init__(self,value,valueType=float):
        super(GCodeSingleArgCmd,self).__init__()
        self.valueType = valueType # float, int, str
        self.value = value

    def getCmdList(self):
        cmdList = super(GCodeSingleArgCmd,self).getCmdList()
        cmdList.append('{0}'.format(self.valueType(self.value)))
        return cmdList


class GCodeAxisArgCmd(GCodeCmd):
    """
    Base class for gcode commands with axis arguments, such as RapidMotion, 
    LinearFeed, etc.
    """

    axisNames = ('x','y','z','a','b','c','u','v','w') 

    def __init__(self, *args, **kwargs):
        super(GCodeAxisArgCmd,self).__init__()
        self.motionDict = normalizeToKwargs(self.axisNames,args,kwargs)
        if not [ v for k, v in self.motionDict.iteritems() if v is not None]:
            raise ValueError, 'missing commands'

    def getCmdList(self):
        cmdList = super(GCodeAxisArgCmd,self).getCmdList()
        for axis in self.axisNames:  # Use order in axisNames list
            motion = self.motionDict[axis] 
            if motion is not None:
                cmdList.append('{0}{1}'.format(axis.upper(),float(motion)))
        return cmdList


class GCodeHelicalMotion(GCodeCmd):
    """
    Base class for gcode commands involving helical motion.
    """

    def __init__(self,*args, **kwargs):
        super(GCodeHelicalMotion,self).__init__()
        self.direction = kwargs.pop('d').lower()
        if self.direction == 'cw':
            self.code = 'G2'
        elif self.direction == 'ccw':
            self.code = 'G3'
        else:
            raise ValueError, 'unknown dirction {0}'.format(direction)
        self.motionDict = kwargs

        # Make sure we have at least one of the required arguments
        test = False 
        for k in self.requiredKeys:
            if k in kwargs:
                test = True
        if not test:
            raise ValueError, 'missing required key: {0}'.format(self.requiredKeys)


    def getCmdList(self):
        cmdList = super(GCodeHelicalMotion,self).getCmdList()
        for name in self.motionArgs:
            value = self.motionDict[name]
            if value is not None:
                cmdList.append('{0}{1}'.format(name.upper(),float(value)))
        return cmdList



# Motion commands
# -----------------------------------------------------------------------------
class RapidMotion(GCodeAxisArgCmd):

    def __init__(self, *args, **kwargs):
        super(RapidMotion,self).__init__(*args, **kwargs)
        self.code = 'G0'
        self.commentStr = 'Rapid motion'


class LinearFeed(GCodeAxisArgCmd):

    def __init__(self, *args, **kwargs):
        super(LinearFeed,self).__init__(*args, **kwargs)
        self.code = 'G1'
        self.commentStr = 'Linear feed'


class Dwell(GCodeSingleArgCmd):

    def __init__(self,value):
        super(Dwell,self).__init__(value,valueType=float)
        self.code = 'G4'
        self.commentStr = 'Dwell'


class HelicalMotionXY(GCodeHelicalMotion):

    motionArgs = ('x', 'y', 'z', 'i', 'j', 'p') 
    kwargsKeys = ('d' ,) + motionArgs
    requiredKeys = ('i', 'j') # Must have at least one of these
    
    def __init__(self,*args,**kwargs):
        """

        Arguments (positional or keyword)

        d = 'cw' for 'ccw'
        x = (optional) x end position
        y = (optional) y end positino
        z = (optional) z end position helical motion
        i = x offset
        j = y offset
        p = (optional) number of turns

        Note, if x,y given then distances from arc center to start and end
        positions must be equal.
        """
        kwargs = normalizeToKwargs(self.kwargsKeys, args, kwargs)
        super(HelicalMotionXY,self).__init__(*args,**kwargs)
        self.commentStr = 'Helical motion xy-plane, {0}'.format(self.direction)


class HelicalMotionXZ(GCodeHelicalMotion):

    motionArgs = ('x', 'z', 'y', 'i', 'k', 'p')
    kwargsKeys = ('d',) + motionArgs
    requiredKeys = ('i', 'k')  # Must have at least one of these

    def __init__(self,*args,**kwargs):
        """
        Arguments (positional or keyword)

        d = 'cw' or 'ccw'
        x = (optional) x end position
        z = (optional) z end positino
        y = (optional) y end position for helical motion
        i = x offset
        k = z offset
        p = (optional) number of turns

        Note, if x,z given then distances from arc center to start and end
        positions must be equal.

        """
        kwargs = normalizeToKwargs(self.kwargsKeys,args,kwargs)
        super(HelicalMotionXZ,self).__init__(*args, **kwargs)
        self.commentStr = 'Helical motion xz-plane, {0}'.format(self.direction)


class HelicalMotionYZ(GCodeHelicalMotion):

    motionArgs = ('y', 'z', 'x', 'j', 'k', 'p')
    kwargsKeys = ('d',) + motionArgs
    requiredKeys = ('i','k') # Must have at least one of these


    def __init__(self,*args, **kwargs):
        """ 
        Arguments (positional or keyword)

        d = 'cw' or 'ccw'
        y = (optional) y end position
        z = (optional) z end positino
        x = (optional) x end position for helical motion
        j = j offset
        k = z offset
        p = (optional) number of turns

        Note, if y,z given then distances from arc center to start and end
        positions must be equal.

        """
        kwargs = normalizeToKwargs(self.kwargsKeys,args,kwargs)
        super(HelicalMotionYZ,self).__init__()
        self.commentStr = 'Helical motion yz-plane, {0}'.format(self.direction)


class CancelCannedCycle(GCodeCmd):

    def __init__(self):
        super(CancelCannedCycle,self).__init__()
        self.code = 'G80'
        self.commentStr = 'Cancel canned cycle'


# Canned Cycles
# -----------------------------------------------------------------------------

class DrillCycle(GCodeCmd):

    kwargsKeys =  ('x','y','z','r','l','p')
    requiredKeys = ('x','y','z','r')

    def __init__(self,*args,**kwargs):
        """
        x = drill x position
        y = drill y position
        z = drill z position (final)
        r = feed start/retract z position
        l = (optional) number of repetitions
        p = (optional) dwell time in secs 
        """
        super(DrillCycle,self).__init__()
        kwargs = normalizeToKwargs(self.kwargsKeys,args,kwargs)
        for k in self.requiredKeys:
            if kwargs[k] is None:
                raise ValueError, 'missing value for parameter {0}'.format(k)
        self.params = kwargs
        if self.params['p'] is None:
            self.code = 'G81'
            self.commentStr = 'Drill cycle'
        else:
            self.code = 'G82'
            self.commentStr = 'Drill cycle w/ dwell'

    def getCmdList(self):
        cmdList = super(DrillCycle,self).getCmdList()
        for name in self.kwargsKeys:
            value = self.params[name]
            if value is not None:
                if name != 'l':
                    value = float(value)
                else:
                    value = int(value)
                cmdList.append('{0}{1}'.format(name.upper(),value))
        return cmdList
            

class PeckDrillCycle(GCodeCmd):

    def __init__(self):
        pass

# Distance Mode 
# -----------------------------------------------------------------------------

class AbsoluteMode(GCodeCmd):

    def __init__(self):
        super(AbsoluteMode,self).__init__()
        self.code = 'G90'
        self.commentStr = 'Set absolute distance mode'


class IncrementalMode(GCodeCmd):

    def __init__(self):
        super(IncrementalMode,self).__init__()
        self.code = 'G91'
        self.commentStr = 'Set incremental distance mode'


# Feedrate Mode
# -----------------------------------------------------------------------------

class InverseTimeMode(GCodeCmd):

    def __init__(self):
        super(InverseTimeMode,self).__init__()
        self.code = 'G93'
        self.commentStr = 'Set feedrate mode to inverse time'


class UnitsPerMinuteMode(GCodeCmd):

    def __init__(self):
        super(UnitsPerMinuteMode,self).__init__()
        self.code = 'G94'
        self.commentStr = 'Set feedrate mode to units per minute'


class UnitsPerRevMode(GCodeCmd):

    def __init__(self):
        super(UnitsPerRevMode,self).__init__()
        self.code = 'G95'
        self.commentStr = 'Set feedrate mode to units per revolution'

# Coolant
# -----------------------------------------------------------------------------

class MistCoolantOn(GCodeCmd):

    def __init__(self):
        super(MistCoolantOn,self).__init__()
        self.code = 'M7'
        self.commentStr = 'Turn mist coolant on'


class FloodCoolantOn(GCodeCmd):

    def __init__(self):
        super(FloodCoolantOn,self).__init__()
        self.code = 'M8'
        self.commentStr = 'Turn flood coolant on'


class CoolantOff(GCodeCmd):

    def __init__(self):
        super(CoolantOff,self).__init__()
        self.code = 'M9'
        self.commantStr = 'Turn all coolant off'


# Tool length offset
# -----------------------------------------------------------------------------

class EnableToolLengthOffset(GCodeCmd):

    def __init__(self, tool=None):
        super(EnableToolLengthOffset,self).__init__()
        self.code = 'G43'
        self.tool = tool
        self.commentStr = "Tool length offset enabled"
        if self.tool is not None:
            self.commentStr = "{0} for tool {1}".format(self.commentStr,self.tool)

    def getCmdList(self):
        cmdList = super(EnableToolLengthOffset,self).getCmdList()
        if self.tool is not None:
            cmdList.append('H{0}'.format(int(self.tool)))
        return cmdList


class SetToolLengthOffset(GCodeAxisArgCmd):

    def __init__(self,*arg,**kwarg):
        super(SetToolLengthOffset,self).__init__(*arg,**kwarg)
        self.code = "G43.1"
        self.commentStr = "Set tool length offset"


class CancelToolLengthOffset(GCodeCmd):

    def __init__(self):
        super(CancelToolLengthOffset,self).__init__()
        self.code = 'G49'
        self.commentStr = "Cancel tool length offset"


# Cutter compensation
# -----------------------------------------------------------------------------

class CancelCutterCompensation(GCodeCmd):

    def __init__(self):
        super(CancelCutterCompensation,self).__init__()
        self.code = 'G40'
        self.commentStr = 'Cancel cutter radius compensation'


# Units
# -----------------------------------------------------------------------------

class Units(GCodeCmd):

    def __init__(self,unitStr):
        super(Units,self).__init__()
        unitStr = unitStr.lower()
        if unitStr == 'inch' or unitStr == 'in':
            self.code = 'G20'
            self.commentStr = 'Set units to inches'
        elif unitStr == 'mm':
            self.code = 'G21'
            self.commentStr = 'Set units to millimeter'
        else:
            raise ValueError, 'uknown unit {0}'.format(unitStr)

class Inches(Units):

    def __init__(self):
        super(Inches,self).__init__('inch')

class Millimeter(Units):

    def __init__(self):
        super(Millimeter,self).__init__('mm')


# Plane selection
# -----------------------------------------------------------------------------

class SelectPlane(GCodeCmd):

    planeToCodeDict = {
            'xy': 'G17', 
            'zx': 'G18', 
            'yz': 'G19', 
            'uv': 'G17.1', 
            'wu': 'G18.1', 
            'vw': 'G19.1'
            }

    def __init__(self,plane):
        super(SelectPlane,self).__init__()
        self.code = SelectPlane.planeToCodeDict[plane.lower()]
        self.commentStr = 'Select plane {0}'.format(plane)


class SelectPlaneXY(SelectPlane):

    def __init__(self):
        super(SelectPlaneXY,self).__init__('xy')


class SelectPlaneZX(SelectPlane):

    def __init__(self):
        super(SelectPlaneXY,self).__init__('zx')


class SelectPlaneYZ(SelectPlane):

    def __init__(self):
        super(SelectPlaneXY,self).__init__('yz')
        

class SelectPlaneUV(SelectPlane):

    def __init__(self):
        super(SelectPlaneXY,self).__init__('uv')


class SelectPlaneWU(SelectPlane):

    def __init__(self):
        super(SelectPlaneXY,self).__init__('wu')


class SelectPlaneVW(SelectPlane):

    def __init__(self):
        super(SelectPlaneXY,self).__init__('vw')


# Cutter radius compensation
# -----------------------------------------------------------------------------

# Path control mode
# -----------------------------------------------------------------------------

class ExactPathMode(GCodeCmd):

    def __init__(self):
        super(ExactPathMode,self).__init__()
        self.code = 'G61'
        self.commentStr = 'Exact path mode'


class ExactStopMode(GCodeCmd):

    def __init__(self):
        super(ExactStopMode,self).__init__()
        self.code = 'G61.1'
        self.commentStr = 'Exact stop mode'


class PathBlendMode(GCodeCmd):

    kwargsKeys = ('p', 'q')

    def __init__(self,*args,**kwargs):
        kwargs = normalizeToKwargs(self.kwargsKeys,args,kwargs)
        if (kwargs['q'] is not None) and (kwargs['p'] is None):
            raise ValueError, 'naive cam tolerance q speficed with out tolerance p'
        super(PathBlendMode,self).__init__()
        self.params = kwargs
        self.code = 'G64'
        self.commentStr = 'path blend mode'

    def getCmdList(self):
        cmdList = super(PathBlendMode,self).getCmdList()
        for name in self.kwargsKeys:
            value = self.params[name]
            if value is not None:
                cmdList.append('{0}{1}'.format(name,float(value)))
        return cmdList


# Return mode in canned cycles
# ----------------------------------------------------------------------------

class CannedCycleReturnMode(GCodeCmd):

    modeDict = {'prior': 'G98', 'r-word': 'G99'}
    commentDict = {
            'prior'  : "set canned cycle retract to 'return to prior'",
            'r-word' : "set canned cycle retract to 'return to r-word'", 
            }

    def __init__(self,mode):
        super(CannedCycleReturnMode,self).__init__()
        if mode.lower() not in self.modeDict:
            raise ValueError, 'unknown canned cycle return mode {0}'.format(mode)
        self.mode = mode.lower()
        self.code = self.modeDict[self.mode]
        self.commentStr = self.commentDict[self.mode]


# Other modal codes
# -----------------------------------------------------------------------------

class CoordinateSystem(GCodeCmd):

    Number2Code = {1:'G54', 2:'G55', 3:'G56', 4:'G57', 5:'G58', 6:'G59'}

    def __init__(self, n):
        super(CoordinateSystem,self).__init__()
        self.code = CoordinateSystem.Number2Code[n]
        self.commentStr = 'Select coordinate system {0}'.format(n)



class FeedRate(GCodeSingleArgCmd):

    def __init__(self,value):
        super(FeedRate,self).__init__(value, valueType=float)
        self.code = 'F'
        self.commentStr = 'Set feed rate'


class SpindleSpeed(GCodeSingleArgCmd):

    def __init__(self,value):
        super(SpindleSpeed,self).__init__(value,valueType=float)
        self.code = 'S'
        self.commentStr = 'Set Spindle speed'


class SelectTool(GCodeSingleArgCmd):

    def __init__(self,value, valueType=int):
        super(SelectTool,self).__init__(value,valueType=valueType)
        self.code = 'T'
        self.commentStr = 'Select tool'


class ChangeTool(GCodeCmd):

    def __init__(self):
        super(ChangeTool,self).__init__()
        self.code = 'M6'
        self.commentStr = 'Change tool'


class SelectAndChangeTool(SelectTool):

    def __init__(self,value):
        super(SelectAndChangeTool,self).__init__(value,valueType=int)
        self.commentStr = 'Select and change tool'

    def getCmdList(self):
        cmdList = super(SelectAndChangeTool,self).getCmdList()
        cmdList.append(ChangeTool().code)
        return cmdList


class Pause(GCodeCmd):

    def __init__(self):
        super(Pause,self).__init__()
        self.code = 'M0'
        self.commentStr = 'Program pause'


class OptionalPause(GCodeCmd):

    def __init__(self):
        super(OptionalPause,self).__init__()
        self.code = 'M1'
        self.commentStr = 'Optional program pause'



class End(GCodeCmd):

    def __init__(self):
        super(End,self).__init__()
        self.code = 'M2'
        self.commentStr = 'End program'

class Comment(GCodeSingleArgCmd):

    def __init__(self, value):
        super(Comment,self).__init__(value,valueType=str)
        self.code = ';'

class Space(GCodeCmd):

    def __init__(self):
        super(Space,self).__init__()
        self.code  = ''
        self.commentStr = '' 

# Utility functions
#  ----------------------------------------------------------------------------

def normalizeToKwargs(expectedKeys,argsTuple,kwargsDict):
    """
    Normalize, arguments For functions that can take either position or
    keyword arguments.
    """
    kwargsDictNorm = dict([(k,None) for k in expectedKeys])
    if argsTuple and kwargsDict: 
        errMsg = 'mixed argument types - must be either positional of keyword'
        raise ValueError, errMsg
    if argsTuple:
        kwargsDict = dict(zip(expectedKeys,argsTuple))
    kwargsDictNorm.update(kwargsDict)
    return kwargsDictNorm

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    # Simple tests

    import math

    cmd = RapidMotion(0,1,2)
    print(cmd)

    cmd = LinearFeed(x=1, y=2)
    print(cmd)

    cmd = FeedRate(10)
    print(cmd)

    cmd = SpindleSpeed(1000)
    print(cmd)

    cmd = SelectTool(1)
    print(cmd)

    cmd = ChangeTool()
    print(cmd)

    cmd = SelectAndChangeTool(1)
    print(cmd)

    cmd = Dwell(2.5)
    print(cmd)

    cmd = CancelCannedCycle()
    print(cmd)

    cmd = AbsoluteMode()
    print(cmd)

    cmd = IncrementalMode()
    print(cmd)

    cmd = Pause()
    print(cmd)

    cmd = OptionalPause()
    print(cmd)

    cmd = End()
    print(cmd)

    cmd = Comment('Hello this is a comment')
    print(cmd)

    cmd = Space()
    print(cmd)

    cmd = MistCoolantOn()
    print(cmd)

    cmd = FloodCoolantOn()
    print(cmd)

    cmd = CoolantOff()
    print(cmd)

    for i in range(1,7):
        cmd = CoordinateSystem(i)
        print(cmd)

    for name in SelectPlane.planeToCodeDict:
        cmd = SelectPlane(name)
        print(cmd)

    cmd = SelectPlaneXY()
    print(cmd)

    cmd = InverseTimeMode()
    print(cmd)

    cmd = UnitsPerMinuteMode()
    print(cmd)

    cmd = UnitsPerRevMode()
    print(cmd)

    for unit in ('in','mm'):
        cmd = Units(unit)
        print(cmd)

    cmd = Inches()
    print(cmd)

    cmd = Millimeter()
    print(cmd)

    cmd = EnableToolLengthOffset()
    print(cmd)

    cmd  = EnableToolLengthOffset(tool=1)
    print(cmd)

    cmd = SetToolLengthOffset(z=2.0)
    print(cmd)

    cmd = CancelToolLengthOffset()
    print(cmd)

    cmd = HelicalMotionXY('cw',1.0,1.0,None,1.0)
    print(cmd)

    cmd = HelicalMotionXY(d='ccw', i=1.0, j=1.0)
    print(cmd)

    cmd = CannedCycleReturnMode('prior')
    print(cmd)

    cmd = CannedCycleReturnMode('r-word')
    print(cmd)

    cmd = ExactPathMode()
    print(cmd)

    cmd = ExactStopMode()
    print(cmd)

    cmd = PathBlendMode()
    print(cmd)

    cmd = DrillCycle(0,0,-1.0,0.1)
    print(cmd)

    cmd = DrillCycle(x=0,y=0,z=-1.0,r=0.1,l=2)
    print(cmd)

    cmd = DrillCycle(x=0,y=0,z=-1.0,r=0.1,l=2)
    print(cmd)

    cmd = DrillCycle(x=0,y=0,z=-1.0,r=0.1,p=1.0)
    print(cmd)

    cmd = DrillCycle(math.pi,0,-1.0,0.1,None,1.0)
    print(cmd)





