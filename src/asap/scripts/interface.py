#!/usr/bin/env python
import rospy
import sys
from move_base_msgs.msg import *
from std_msgs.msg import String
from rosplan_dispatch_msgs.msg import ActionDispatch, ActionFeedback
from rosplan_knowledge_msgs.srv import *
from rosplan_knowledge_msgs.msg import *
from actionlib import SimpleActionClient
from diagnostic_msgs.msg import KeyValue

class interface:
    
    def __init__(self):

        # ROSPlan ActionDispatch
        self.rosplan_interface_subscriber = rospy.Subscriber("/rosplan_plan_dispatcher/action_dispatch", ActionDispatch , self.callback_pddl_action, queue_size=10)
        
        # ROSPlan ActionFeedback
        self.action_feedback_pub = rospy.Publisher('/rosplan_plan_dispatcher/action_feedback', ActionFeedback, queue_size=10)
        
        # Status, ist entweder "free" oder "not_free"
        self.state_pub = rospy.Publisher('/state', String, queue_size=10)
        
    def callback_pddl_action(self, pddl_action_msg):
        
        self.publish_feedback(pddl_action_msg.plan_id, pddl_action_msg.action_id, ActionFeedback.ACTION_ENABLED)
        
        if pddl_action_msg.name == 'moveto_table':
            self.moveto_table(pddl_action_msg.parameters)
        if pddl_action_msg.name == 'pickup':
            self.pickup(pddl_action_msg.parameters)
        if pddl_action_msg.name == 'putdown':
            self.putdown(pddl_action_msg.parameters)
            
        self.publish_feedback(pddl_action_msg.plan_id, pddl_action_msg.action_id, ActionFeedback.ACTION_SUCCEEDED_TO_GOAL_STATE)
        
    def publish_feedback(self, plan_id, action_id, status):
        fb = ActionFeedback()
        fb.action_id = action_id
        fb.plan_id = plan_id
        fb.status = status
        self.action_feedback_pub.publish(fb)
        
        
    def moveto_table(self, parameter):

        # Check moveto_table third parameter (:parameters (?roboter - robot ?from - table ?to - table))
        to_param = parameter[2].value # "tableb1"
        
        # wp24 Koordinaten werden ueberschrieben.
        if to_param == "tablea1":
            rospy.set_param('/rosplan_roadmap_server/rosplan_asap/wp/wp24', [3.7, 1, 0])
        elif to_param == "tablea2":
            rospy.set_param('/rosplan_roadmap_server/rosplan_asap/wp/wp24', [3.7, 4.5, 0])
        elif to_param == "tablea3":
            rospy.set_param('/rosplan_roadmap_server/rosplan_asap/wp/wp24', [-3.7, 4.5, 0])
        elif to_param == "tablea4":
            rospy.set_param('/rosplan_roadmap_server/rosplan_asap/wp/wp24', [-3.7, 1, 0])
        elif to_param == "tableb1":
            rospy.set_param('/rosplan_roadmap_server/rosplan_asap/wp/wp24', [3.7, -0.5, 0])
        elif to_param == "tableb2":
            rospy.set_param('/rosplan_roadmap_server/rosplan_asap/wp/wp24', [3.7, -4.5, 0])
        elif to_param == "tableb3":
            rospy.set_param('/rosplan_roadmap_server/rosplan_asap/wp/wp24', [-3.5, -0.5, 0])
        elif to_param == "tableb4":
            rospy.set_param('/rosplan_roadmap_server/rosplan_asap/wp/wp24', [-3.6, -4.5, 0])
            
        
    def pickup(self, parameter):
        
        self.state_pub.publish("not_free")
        
        while not rospy.has_param("/currentTable"):
            rospy.sleep(0.2)
        current_point = rospy.get_param('/currentTable')
        
        knowledge_update_service_request = KnowledgeUpdateServiceRequest()
        knowledge_update_service_request.update_type = 3 
        knowledge_update_service_request.knowledge.knowledge_type = KnowledgeItem.FUNCTION
        knowledge_update_service_request.knowledge.is_negative = False
        knowledge_update_service_request.knowledge.attribute_name = 'n_drinks'
        knowledge_update_service_request.knowledge.AP_DECREASE
        knowledge_update_service_request.knowledge.values.append(KeyValue("table", current_point))
        rospy.ServiceProxy('/rosplan_knowledge_base/update', KnowledgeUpdateService)

    def putdown(self, parameter):
        
        self.state_pub.publish("free")
        
        while not rospy.has_param("/currentTable"):
            rospy.sleep(0.2)
        current_point = rospy.get_param('/currentTable')
        
        knowledge_update_service_request = KnowledgeUpdateServiceRequest()
        knowledge_update_service_request.update_type = 3 
        knowledge_update_service_request.knowledge.knowledge_type = KnowledgeItem.FUNCTION
        knowledge_update_service_request.knowledge.is_negative = False
        knowledge_update_service_request.knowledge.attribute_name = 'n_drinks'
        knowledge_update_service_request.knowledge.AP_INCREASE
        knowledge_update_service_request.knowledge.values.append(KeyValue("table", current_point))
        rospy.ServiceProxy('/rosplan_knowledge_base/update', KnowledgeUpdateService)
        
        
def main(args):

    rospy.init_node('interface', anonymous=False)

    schk = interface()

    try:   
        rospy.spin()
    except KeyboardInterrupt:
        print("schnittstelle abbrechen")

if __name__ == '__main__':

    main(sys.argv)