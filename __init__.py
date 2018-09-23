import os, inspect
from lib import navpy
from util import transformations as tr
from util import SRTM, common, file_tools, mavlink_meta

from shapely.geometry import LineString
from shapely import affinity
# Samuel Dudley
# September 2018

# Mission planning tool for mavlink enabled vehicles


# Setup logging



# generic mission object
class BaseMission(object):
    def __init__(self, missionID, takeoffAlt, takoffLoiterTime, outputDir = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'missions'), logName = 'mission'):
        # setup logger
        self.logger = common.setupLogger(logName)
        # set ouput directory
        self.outputDir = outputDir
        # TODO: Check to make sure the dir exists
        self.latRef = None
        self.lonRef = None
        self.altRef = None
        self.frame = 'ned'
        self.points = []
        # setup altitude types
        self.altitudeTypes = {'relative':3, 'terrain':10}
        self.availableFrames = ['ned', 'lla']
        self.missionID = missionID
        self.takeoffAlt = takeoffAlt
        self.takoffLoiterTime = takoffLoiterTime
        self.filePath = os.path.join(self.outputDir, self.missionID+'.txt')
        self.fid = None
        self.missionLine = 0
        self.autoContinue = 1
        self.mavlinkEnums = mavlink_meta.getMavlinkEnums()
    
    def writeWaypointFile(self, actions):
        file_tools.makePath(self.outputDir)
        with open(self.filePath, 'w+') as self.fid:
            for action in actions:
                if inspect.ismethod(action):
                    action()
                else:
                    self.writeGenericAction(action)
    
    def writeWaypointLine(self, line, newLine = True):
        if newLine:
            line +="\n"
        if not self.fid.closed:
            self.fid.write(line)
            self.missionLine += 1
        else:
            # The waypoint file is closed
            self.logger.error('Failed to write to waypoint file')
        
    def writeHomeLLA(self):
        line = "{0} 0 0 {1} 0.0 0.0 0.0 0.0 {2} {3} {4} {5}".format(self.missionLine, self.mavlinkEnums['MAV_CMD']['NAV_WAYPOINT']['value'],
                                                                    self.latRef, self.lonRef, self.altRef,
                                                                    self.autoContinue)
        self.writeWaypointLine(line)
    
    def writeTakeoffLLA(self):
        line = "{0} 0 {1} {2} 0.0 0.0 0.0 0.0 {3} {4} {5} {6}".format(self.missionLine, self.mavlinkEnums['MAV_FRAME']['GLOBAL_RELATIVE_ALT']['value'],
                                                                    self.mavlinkEnums['MAV_CMD']['NAV_TAKEOFF']['value'],
                                                                    self.latRef, self.lonRef, self.takeoffAlt,
                                                                    self.autoContinue)
        self.writeWaypointLine(line)
        
    def writeLoiterTime(self):
        line = "{0} 0 {1} {2} {3} 0.0 0.0 0.0 {4} {5} {6} {7}".format(self.missionLine, self.mavlinkEnums['MAV_FRAME']['GLOBAL_RELATIVE_ALT']['value'],
                                                                    self.mavlinkEnums['MAV_CMD']['NAV_LOITER_TIME']['value'],
                                                                    self.takoffLoiterTime,
                                                                    self.latRef, self.lonRef, self.takeoffAlt,
                                                                    self.autoContinue)
        self.writeWaypointLine(line)
        
    def writeReturnToLaunch(self):
        line = "{0} 0 {1} {2} 0.0 0.0 0.0 0.0 0.0 0.0 0.0 {3}".format(self.missionLine, self.mavlinkEnums['MAV_FRAME']['GLOBAL_RELATIVE_ALT']['value'],
                                                                    self.mavlinkEnums['MAV_CMD']['NAV_RETURN_TO_LAUNCH']['value'],
                                                                    self.autoContinue)
        self.writeWaypointLine(line)
    
    def writeWaypointLLA(self, lla):
        line = "{0} 0 {1} {2} 0.0 0.0 0.0 0.0 {3} {4} {5} {6}".format(self.missionLine, self.mavlinkEnums['MAV_FRAME']['GLOBAL_RELATIVE_ALT']['value'],
                                                                    self.mavlinkEnums['MAV_CMD']['NAV_WAYPOINT']['value'],
                                                                    lla[0], lla[1], lla[2],
                                                                    self.autoContinue)
        self.writeWaypointLine(line)
        
    def writePreamble(self):
        self.writeWaypointLine("QGC WPL 110")
        self.missionLine = 0
    
    def writeGenericAction(self, action):
        line = "{0} {1}".format(self.missionLine, action)
        self.writeWaypointLine(line)
        
    def checkFrame(self):
        if self.frame.lower() in self.availableFrames:
            return True
        else:
            return False
    
    def setReferenceLLA(self, LLA=[]):
        # TODO: check LLA length
        self.latRef = LLA[0]
        self.lonRef = LLA[1]
        self.altRef = LLA[2]
        sss = SRTM.NEDGround(lla_ref = LLA , width_m = 10000 , height_m = 10000 , step_m = 30, logger = self.logger)
    
    def setReferenceLatitude(self, lat):
        self.latRef = lat
    
    def setReferenceLongitude(self, lon):
        self.lonRef = lon
    
    def setReferenceAltitude(self, alt):
        self.altRef = alt
    
    def getPointsNED(self, lla):
        ned = navpy.lla2ned(lla[0], lla[1], lla[2], lat_ref = self.latRef, lon_ref = self.lonRef, alt_ref = self.altRef)
        return list(ned)
    
    def getPointsLLA(self, ned):
        lla = navpy.ned2lla(ned, lat_ref = self.latRef, lon_ref= self.lonRef , alt_ref = self.altRef)
        return list(lla)
        
class GridMission(BaseMission):
    def __init__(self, missionID, takeoffAlt = 10, takoffLoiterTime = 5, append = False):
        super(GridMission, self).__init__(missionID, takeoffAlt, takoffLoiterTime)
        self.logger.debug(missionID)
        
    def generateGrid(self, out = 100, right = 50, yaw = 45, alt = 25):
        # TODO: dynamically calculate lane width from sensor FoV and alt
        laneWidth = 10
        points = []
        if right < 0:
            laneWidth = -laneWidth
        
        for k in range(0, 50, 2):
            points.append((0, k*laneWidth))
            points.append((out, k*laneWidth))
            points.append((out, (k+1)*laneWidth))
            points.append((0, (k+1)*laneWidth))
            if abs(laneWidth*(k+1)) > abs(right) :
                 break;

        line = LineString(points)
#         line = affinity.rotate(line, angle=yaw, origin=list(line.coords)[0], use_radians=False)
        llas = [self.getPointsLLA([point[0], point[1], 0]) for point in list(line.coords)]
        for lla in llas:
            self.writeWaypointLLA([lla[0], lla[1], alt])

if __name__ == '__main__':
    mission = GridMission('grid_test')
    mission.setReferenceLLA([-35.3615074158, 149.163650513, 500])
    actions = [mission.writePreamble,
            mission.writeHomeLLA,
            mission.writeTakeoffLLA,
            mission.writeLoiterTime,
            mission.generateGrid,
            mission.writeReturnToLaunch]
    mission.writeWaypointFile(actions)
