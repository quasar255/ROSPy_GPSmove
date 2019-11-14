#! /usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix #GPS message in ROS
from geopy.distance import geodesic   #https://pypi.python.org/pypi/geopy
from geometry_msgs.msg import Vector3 #Vector class to performs basic 3D vector math operations

class WayPoint():
    def __init__(self, latitude, longitude, altitude):
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude

    def print_data(self):
        return "["+str(self.latitude)+","+str(self.longitude)+","+str(self.altitude)+"]"


class GpsClass(object):
    def __init__(self):
        self._sub = rospy.Subscriber('/fix', NavSatFix, self.callback)
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0
        self.testwaypoint = WayPoint(49.900090,8.899960,0.000000)

    def remove_noise(self,latitude,longitude,altitude):
        return round(latitude,5),round(longitude,5),round(altitude,1)

    def callback(self,msg):
        self.latitude, self.longitude, self.altitude = self.remove_noise(msg.latitude, msg.longitude, msg.altitude)

    def distance_from_waypoint(self,waypoint):
        origin = (self.latitude, self.longitude)
        waypoint = (waypoint.latitude, waypoint.longitude)
        return geodesic(origin, waypoint).meters

    def get_current_gps_pos(self): #Returns newest GPS current position in a Waypoint Format
        return WayPoint(self.latitude, self.longitude, self.altitude)

    def get_direction_to_waypoint(self, waypoint):
        direction = Vector3()
        direction.x = waypoint.latitude - self.latitude
        direction.y = waypoint.longitude - self.longitude
        direction.z = waypoint.altitude - self.altitude

if __name__ == '__main__':
  rospy.init_node('gps_topic_subscriber')
  GpsClass()
  rospy.spin()