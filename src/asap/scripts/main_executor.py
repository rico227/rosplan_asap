#!/usr/bin/env python
import rospkg
import rospy
import random
import numpy as np
from rosplan_interface_mapping.srv import CreatePRM
from rosplan_knowledge_msgs.srv import *
from rosplan_knowledge_msgs.msg import *
from rosplan_dispatch_msgs.srv import DispatchService, PlanningService
from diagnostic_msgs.msg import KeyValue
from std_srvs.srv import Empty

# get path of pkg
rospack = rospkg.RosPack()
rospy.init_node("coordinator")

# load parameters
max_prm_size = rospy.get_param('~max_prm_size', 1000)
planner_command = rospy.get_param('~planner_command', "")
domain_path = rospy.get_param('~domain_path', "")
problem_path = rospy.get_param('~problem_path', "")
data_path = rospy.get_param('~data_path', "")

# wait for services
rospy.wait_for_service('/rosplan_roadmap_server/create_prm')
rospy.wait_for_service('/rosplan_problem_interface/problem_generation_server')
rospy.wait_for_service('/rosplan_planner_interface/planning_server_params')
rospy.wait_for_service('/rosplan_plan_dispatcher/dispatch_plan')
rospy.wait_for_service('/rosplan_parsing_interface/parse_plan')

rospy.set_param('/tables/tablea1', [3.7, 1, 0])
rospy.set_param('/tables/tablea2', [3.7, 4.5, 0])
rospy.set_param('/tables/tablea3', [-3.7, 4.5, 0])
rospy.set_param('/tables/tablea4', [-3.7, 1, 0])
rospy.set_param('/tables/tableb1', [3.7, -0.5, 0])
rospy.set_param('/tables/tableb2', [3.7, -4.5, 0])
rospy.set_param('/tables/tableb3', [-3.5, -0.5, 0])
rospy.set_param('/tables/tableb4', [-3.6, -4.5, 0])
        
def make_prm(size):
    # generate dense PRM
    rospy.loginfo("KCL: (%s) Creating PRM of size %i" % (rospy.get_name(), size))
    prm = rospy.ServiceProxy('/rosplan_roadmap_server/create_prm', CreatePRM)        
    if not prm(size,0.8,1.6,2.0,65,200000):
        rospy.logerr("KCL: (%s) No PRM was made" % rospy.get_name())
    
    
def generate_problem_and_plan():

    rospy.loginfo("KCL: (%s) Calling problem generation" % rospy.get_name())
    pg = rospy.ServiceProxy('/rosplan_problem_interface/problem_generation_server', Empty)
    if not pg():
        rospy.logerr("KCL: (%s) No problem was generated!" % rospy.get_name())

    rospy.loginfo("KCL: (%s) Calling planner" % rospy.get_name())
    pi = rospy.ServiceProxy('/rosplan_planner_interface/planning_server_params', PlanningService)
    pi_response = pi(domain_path, problem_path, data_path, planner_command, True)

    if not pi_response:
        rospy.logerr("KCL: (%s) No response from the planning server." % rospy.get_name())
        return False
    if not pi_response.plan_found:
        rospy.loginfo("KCL: (%s) No plan could be found." % rospy.get_name())
        return False
    else:
        rospy.loginfo("KCL: (%s) Plan was found." % rospy.get_name())
        return True

def execute_plan():

    rospy.loginfo("KCL: (%s) Calling plan parser" % rospy.get_name())
    pp = rospy.ServiceProxy('/rosplan_parsing_interface/parse_plan', Empty)
    if not pp():
        rospy.logerr("KCL: (%s) The plan was not parsed!" % rospy.get_name())
        return

    rospy.sleep(3)

    rospy.loginfo("KCL: (%s) Calling plan execution" % rospy.get_name())
    pd = rospy.ServiceProxy('/rosplan_plan_dispatcher/dispatch_plan', DispatchService)
    pd_response = pd()

    if not pd_response:
        rospy.logerr("KCL: (%s) No response from the dispatch server." % rospy.get_name())
        return False
    if not pd_response.goal_achieved:
        rospy.loginfo("KCL: (%s) The execution was not successful." % rospy.get_name())
        return False
    else:
        rospy.loginfo("KCL: (%s) Plan was executed." % rospy.get_name())
        return True
    
def get_n_drinks():
    
    print("___main_executor___: get_n_drinks")
    
    knowledge_base_state_response = rospy.ServiceProxy('/rosplan_knowledge_base/state/functions', GetAttributeService)
    get_attribute_service_request = GetAttributeServiceRequest()  
    get_attribute_service_request.predicate_name = 'n_drinks'
    facts = knowledge_base_state_response(get_attribute_service_request)
    n_drinks = {}
    for k in facts.attributes:
        n_drinks[k.values[0].value] = k.function_value
        
    return n_drinks

        
def add_maintask_goal(str_tableno):  
     
    print("___main_executor___: add_maintask_goal")
    
    knowledge_update_service_request = KnowledgeUpdateServiceRequest()
    knowledge_update_service_request.update_type = 1
    knowledge_update_service_request.knowledge.knowledge_type = 4
    knowledge_update_service_request.knowledge.instance_name = "goal" + str_tableno
    ineq = DomainInequality()
    ineq.comparison_type = 4
    token_LHS = ExprBase()
    token_LHS.expr_type = 1
    token_LHS.function.name = 'n_drinks'
    token_LHS.function.typed_parameters.append(KeyValue("table", "tableb"+ str_tableno))
    ineq.LHS.tokens.append(token_LHS)
    token_RHS = ExprBase()
    token_RHS.expr_type = 0
    token_RHS.constant = 2
    ineq.RHS.tokens.append(token_RHS)
    knowledge_update_service_request.knowledge.ineq = ineq
    knowledge_update_response = rospy.ServiceProxy('/rosplan_knowledge_base/update', KnowledgeUpdateService)
    if not knowledge_update_response(knowledge_update_service_request):
        rospy.logerr("___main_executor___: add_maintask_goal: maintask goal was not added!" % rospy.get_name())
        
def add_maintask_init(str_room, str_tableno):  
    
    print("___main_executor___: add_maintask_init")
    
    knowledge_update_service_request = KnowledgeUpdateServiceRequest()
    knowledge_update_service_request.update_type = 0
    knowledge_update_service_request.knowledge.knowledge_type = 1
    knowledge_update_service_request.knowledge.attribute_name = "robot_attable"
    kv_robot = KeyValue('roboter', 'robot')
    kv_table = KeyValue('table', 'table' + str_room + str_tableno)
    knowledge_update_service_request.knowledge.values.append(kv_robot)
    knowledge_update_service_request.knowledge.values.append(kv_table)
    knowledge_update_response = rospy.ServiceProxy('/rosplan_knowledge_base/update', KnowledgeUpdateService)
    if not knowledge_update_response(knowledge_update_service_request):
        rospy.logerr("___main_executor___: add_maintask_init: maintask init was not added!" % rospy.get_name())
        
def remove_maintask():
    
    print("___main_executor___: remove_maintask")
    
    for room in ["a", "b"]:
        for table_no in range(1, 5):
            knowledge_update_service_request = KnowledgeUpdateServiceRequest()
            knowledge_update_service_request.update_type = 3 
            knowledge_update_service_request.knowledge.knowledge_type = KnowledgeItem.INEQUALITY
            knowledge_update_service_request.knowledge.is_negative = False
            knowledge_update_service_request.knowledge.attribute_name = 'n_drinks'
            knowledge_update_service_request.knowledge.values.append(KeyValue("table", "tableb" + room + str(table_no)))
            knowledge_update_response = rospy.ServiceProxy('/rosplan_knowledge_base/update', KnowledgeUpdateService)
            if not knowledge_update_response(knowledge_update_service_request):
                rospy.logerr("___main_executor___: remove_maintask: maintask goal was not removed!" % rospy.get_name())
            
def add_sidetask():
    
    print("___main_executor___: add_sidetask")
    
    rospy.sleep(3)
    make_prm(max_prm_size)
    anzahl_goal_wp = 1
    
    knowledge_base_state_response = rospy.ServiceProxy('/rosplan_knowledge_base/state/propositions', GetAttributeService)
    count = 0
    while count<1:
        rospy.sleep(1)
        get_attribute_service_request = GetAttributeServiceRequest()    
        get_attribute_service_request.predicate_name = 'robot_at'
        facts = knowledge_base_state_response(get_attribute_service_request)
        if not facts:
            rospy.logwarn("___main_executor___: add_sidetask: Proposition service not available." % rospy.get_name())
        count = 0
        for k in facts.attributes:
            if not k.is_negative:
                count = count + 1
    
    for i in range(anzahl_goal_wp):
        wp_goal = random.randint(0,max_prm_size-1)
        knowledge_update_service_request = KnowledgeUpdateServiceRequest()
        knowledge_update_service_request.update_type = 1
        knowledge_update_service_request.knowledge.knowledge_type = 1
        knowledge_update_service_request.knowledge.attribute_name = 'visited'
        kv = KeyValue()
        kv.key = 'wp'
        kv.value = 'wp'+str(wp_goal)
        knowledge_update_service_request.knowledge.values.append(kv)
        knowledge_update_response = rospy.ServiceProxy('/rosplan_knowledge_base/update', KnowledgeUpdateService)
        if not knowledge_update_response(knowledge_update_service_request):
            rospy.logerr("___main_executor___: add_sidetask: sidetask goal was not added!" % rospy.get_name())
            
def remove_sidetask():
    
    print("___main_executor___: remove_sidetask")
    
    for wp_no in range(0, 100):
        knowledge_update_service_request = KnowledgeUpdateServiceRequest()
        knowledge_update_service_request.update_type = 2
        knowledge_update_service_request.knowledge.knowledge_type = 1
        knowledge_update_service_request.knowledge.attribute_name = 'robot_at'
        kv_robot = KeyValue('roboter', 'robot')
        kv_wp = KeyValue('wp', 'wp'+str(wp_no))
        knowledge_update_service_request.knowledge.values.append(kv_robot)
        knowledge_update_service_request.knowledge.values.append(kv_wp)
        knowledge_update_response = rospy.ServiceProxy('/rosplan_knowledge_base/update', KnowledgeUpdateService)
        if not knowledge_update_response(knowledge_update_service_request):
            rospy.logerr("___main_executor___: remove_sidetask: sidetask goal was not removed!" % rospy.get_name())
    
    for wp_no in range(0, 100):
        knowledge_update_service_request = KnowledgeUpdateServiceRequest()
        knowledge_update_service_request.update_type = 3 
        knowledge_update_service_request.knowledge.knowledge_type = 1
        knowledge_update_service_request.knowledge.attribute_name = 'visited'
        kv = KeyValue()
        kv.key = 'wp'
        kv.value = 'wp'+str(wp_no)
        knowledge_update_service_request.knowledge.values.append(kv)
        knowledge_update_response = rospy.ServiceProxy('/rosplan_knowledge_base/update', KnowledgeUpdateService)
        if not knowledge_update_response(knowledge_update_service_request):
            rospy.logerr("___main_executor___: remove_sidetask: sidetask goal was not removed!" % rospy.get_name())
    
try:
         
    while (1):
        
        add_sidetask()
        n_drinks = get_n_drinks()

        if (n_drinks['tableb1'] == 2 and n_drinks['tableb2'] == 2 and n_drinks['tableb3'] == 2 and n_drinks['tableb4'] == 2): 
            main_task = False
        else: 
            main_task = True
    
        ###################### TEST ######################
        #main_task = False --> Explore
        #main_task = True --> Serve Drinks
        #main_task = 
        ##################################################
        
        remove_sidetask()
        remove_maintask()
        
        # Serve Drinks
        if (main_task):
            
            # remove_sidetask() removes everything with wp from init and goal
            # add_maintask_goal("1") adds goal (= (n_drinks tableb1) 2)
            # add_maintask_init("a", "2") adds init (robot_attable robot tablea2)
            
            print("Serve Drinks")
            
            if (n_drinks['tableb1'] < 2):
                remove_maintask()
                add_maintask_init("a", "1")
                add_maintask_goal("1")
            elif (n_drinks['tableb2'] < 2):
                remove_maintask()
                add_maintask_init("a", "2")
                add_maintask_goal("2")
            elif (n_drinks['tableb3'] < 2):
                remove_maintask()
                add_maintask_init("a", "3")
                add_maintask_goal("3")
            elif (n_drinks['tableb4'] < 2):
                remove_maintask()
                add_maintask_init("a", "4")
                add_maintask_goal("4")
                
        # Explore
        else:
            
            # remove_maintask() removes everything related to maintask from init and goal
            # add_sidetask() adds goal and init for sidetask
            
            print("Explore")
            remove_maintask()
            add_sidetask()
            
    
        # generate problem and plan
        plan_found = generate_problem_and_plan()
        rospy.sleep(1)
    
        # execute plan
        if plan_found:
            execute_plan()

except rospy.ServiceException as e:
    rospy.logerr("KCL: (%s) Service call failed: %s" % (rospy.get_name(), e))
    