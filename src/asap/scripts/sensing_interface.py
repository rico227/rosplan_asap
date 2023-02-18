#!/usr/bin/env python
import numpy as np
from math import sqrt
import rospy

def robot_at(msg, params):
    
    print("sensing_interface: predicates: robot_at")
    
    assert(msg.header.frame_id == "map")
    assert(len(params) == 2)
    return_value = []
    attributes = get_kb_attribute("robot_at")
    current_point = ''

    # aktuellen robot_at Wert auswerten
    for attribute in attributes:
        if not attribute.is_negative:
            current_point = attribute.values[1].value
            break
    
    # Evtl. auf rosparam server warten um vorhandene Parameter anzeigen zu lassen
    while not rospy.has_param("/rosplan_roadmap_server/rosplan_asap/wp"):
        rospy.sleep(0.2)
    points = rospy.get_param("/rosplan_roadmap_server/rosplan_asap/wp")
    distance = float('inf')
    closest_point = ''
    
    # Punkt definieren, welches die kleinste Distanz zum Roboter hat.
    for point in points:
        pose = rospy.get_param("/rosplan_roadmap_server/rosplan_asap/wp/" + point)
        assert(len(pose) > 0)
        x = pose[0] - msg.pose.pose.position.x
        y = pose[1] - msg.pose.pose.position.y
        d = sqrt(x*x + y*y)
        if d < distance:
            distance = d
            closest_point = point
            
    if current_point != closest_point:
        return_value.append(('robot' + ':' + closest_point, True))
        if current_point != '':
            return_value.append(('robot' + ':' + current_point, False))
            
    # (robot_at ?roboter - robot ?wp - waypoint)
    # robot:wp0 -> (robot_at robot wp1)

    return return_value

def robot_attable(msg, params):
    
    print("sensing_interface: predicates: robot_attable")
    
    assert(msg.header.frame_id == "map")
    assert(len(params) == 2)
    return_value = []
    attributes = get_kb_attribute("robot_attable")
    current_table = ''

   # aktuellen robot_attable Wert auswerten
    for attribute in attributes:
        if not attribute.is_negative:
            current_table = attribute.values[1].value
            break
    
    # Evtl. auf rosparam server warten um vorhandene Parameter anzeigen zu lassen
    while not rospy.has_param("/tables"):
        rospy.sleep(0.2)
    tables = rospy.get_param("/tables")
    distance = float('inf')
    closest_table = ''
    
    # Tisch definieren, welches die kleinste Distanz zum Roboter hat.
    for table in tables:
        pose = rospy.get_param("/tables/" + table)
        assert(len(pose) > 0)
        x = pose[0] - msg.pose.pose.position.x
        y = pose[1] - msg.pose.pose.position.y
        d = sqrt(x*x + y*y)
        if d < distance:
            distance = d
            closest_table = table

    if current_table != closest_table:
        return_value.append(('robot' + ':' + closest_table, True))
        return_value.append(('robot' + ':' + current_table, False))
        
    rospy.set_param('/currentTable', closest_table)
        
    # (robot_attable ?roboter - robot ?table - table)
    # robot:tablea1 -> (robot_attable robot tablea1)

    return return_value
    

def free(msg, params):
    
    print("sensing_interface: predicates: free")

    return_value = []
    
    if msg.data == "not_free":
        return_value.append(('robot', False))
    else:
        return_value.append(('robot', True))
    
    # (robot_attable ?roboter - robot ?table - table)
    # robot:tablea1 -> (robot_attable robot tablea1)
    
    # (free ?roboter - robot)
    # robot
    return return_value

def not_free(msg, params):
    
    print("sensing_interface: predicates: not_free")
    
    return_value = []
    
    if msg.data == "free":
        return_value.append(('robot', False))
    else:
        return_value.append(('robot', True))
    
    return return_value