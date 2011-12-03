import sys
import pydc1394

class Camera1394Inspector(object):
    """
    A tool for finding all 1394 cameras connected to the computer and getting 
    information regarding the camera capabilities e.g., video modes, framerates,
    features, etc. 
    """

    def __init__(self):
        self.dc1394 = pydc1394.DC1394Library()

    def find(self):
        """
        Returns a list of a cameras connected to the system. For each camera
        finds the guid, model, unit, and vendor.

        Guids are converted to hex strings
        """
        cameraList = self.dc1394.enumerate_cameras()
        # Convert guids to hex strings
        cameraListConv = []
        for camera in cameraList:
            guid = camera.pop('guid')
            cameraConv = camera
            cameraConv['guid'] = '%x'%(guid,)
            cameraListConv.append(cameraConv)
        return cameraListConv

    def getGUIDDict(self,info=False):
        """
        Create a dictionary of camera index by guid.
        """
        if not info:
            cameraList = self.find()
        else:
            cameraList = self.getInfoAll()
        guidDict = {}
        for camera in cameraList:
            guid = camera.pop('guid')
            guidDict[guid] = camera
        return guidDict

    def getInfoByGUID(self,guid):
        """
        Returns a dictionary containing relevant information regarding the 
        camera.
        """
        # Temporarily redirect stdout as pydc1394 prints out some crap 
        stdoutOrig = sys.stdout
        sys.stdout = NullDevice()

        cameraObj = pydc1394.Camera(self.dc1394,guid)
        videoModes = []

        # Get video modes and frame rates
        for mode in cameraObj.modes:
            if 'FORMAT7' in str(mode) or 'FORMAT6' in str(mode):
                framerates = None
            else:
                try:
                    framerates = mode.framerates
                except RuntimeError:
                    framerates = None
            modeInfo = {'name': str(mode),'framerates': framerates}
            videoModes.append(modeInfo)

        # Get camera features
        features = []
        for fName in cameraObj.features:
            f = getattr(cameraObj,fName)
            try:
                fRange = f.range
            except RuntimeError:
                fRange = f.pos_modes

            fInfo = {'name': str(fName),'range': fRange}
            features.append(fInfo)
        guidDict = self.getGUIDDict(info=False)
        cameraInfo = guidDict[guid]
        cameraInfo.update({'guid': guid, 'video_modes': videoModes, 'features': features})

        # Reset stdout
        sys.stdout = stdoutOrig

        return cameraInfo

    def getInfoAll(self):
        """
        Returns a list a dictionares containing information for all cameras
        """
        cameraList = self.find() 
        cameraInfoList = []
        for camera in cameraList:
            cameraInfo = self.getInfoByGUID(camera['guid'])
            cameraInfoList.append(cameraInfo)
        return cameraInfoList

class NullDevice(object):
    """
    Simple class for redirecting stdout and stderr
    """
    def write(self,s):
        pass

# Some print functions for testing
# -----------------------------------------------------------------------------

def printGUIDDict(guidDict):
    for guid in guidDict: 
        print 'GUID: %s'%(guid,)
        for k,v in guidDict[guid].iteritems():
            if type(v) in (list,tuple):
                print('  %s'%(k,))
                printListOfDicts(v,indent='    ')
            else:
                print '  %s: %s'%(k,v)
        print 

def printListOfDicts(listOfDicts,indent=''):
    for item in listOfDicts:
        printDict(item,indent)

def printDict(x,indent=''):
    for k,v in x.iteritems():
        print '%s%s: %s'%(indent,k,v)

# -----------------------------------------------------------------------------
if __name__ == '__main__': 

    inspector = Camera1394Inspector()
    guidDict =  inspector.getGUIDDict()
    print 
    printGUIDDict(guidDict)
    guidDict = inspector.getGUIDDict(info=True)
    print
    printGUIDDict(guidDict)



