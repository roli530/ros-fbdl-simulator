import json
from json.encoder import INFINITY
import random
import time

import pymunk
from fribe.loader import load_engine_from_string

from std_msgs.msg import String,Int16,Bool,Float32
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from custom_msgs.msg import Position, Canvas, Names,Distance,Rays
from custom_msgs.srv import Fbdl, Agents, Error, Stop
import math
import threading
from ros2_fuzzy.sensor import Sensor

class Agent(Node):

    def __init__(self, id,name, x,y,state):
        super().__init__(name)

        #Services
        self.create_service(Fbdl, 'fbdl', self.service_callback)
        self.create_service(Stop, 'stop', self.stop_service)
        #Clients
        self.error_client = self.create_client(Error, 'error')
        self.client = self.create_client(Fbdl, 'agent_fbdl')

        self.id = id
        self.name = name
        self.count = 0
        self.value = 0
        self.velx = 0.0
        self.vely = 0.0
        self.test = 0.0
        self.follow_target = 0.0
        self.tiredness = 0.0
        self.x = x
        self.y = y
        self.radius = 70.0
        self.hz = 0.1
        self.agent_speed = 0.0
  
        self.target_x = 0.0
        self.target_y = 0.0
        self.stop = 0.0
        self.canvas_height = 0.0
        self.canvas_width = 0.0
        self.subscribed_topics = []
        self.orientation = 0.0 
      
        self.orientation_error = 0.0
        self.state = state
        self.state_data = 0
        
        #self.new_agent = self.create_publisher(String,"/new_agent" , 10)   #A harmadik egy buffer
        self.stopped = False
        self.collision = 0
        self.other_agents = []
        self.boundary_collision = False
        self.fuzzy_string = ""
        self.nearest_distance = 0.0
        self.nearest_box_distance = 0.0
        self.nearest_special_distance = 0.0
        self.engine = None
        self.linear_vel= 0.0
        self.speed = 0.0
        self.angle = 0.0
        self.sample_angle = [-1, -0.5, 0, 0.5, 1]
        self.min_distance = 100000
        
        self.object_collison = False
        self.reached_goal = -1
        self.charging = False
        self.is_collided = False
        self.object_collision = False
        

        self.mass = 10
        self.moment = pymunk.moment_for_circle(self.mass, 0, 35, (0, 0))
        self.body = pymunk.Body(self.mass, self.moment)
        self.body.position = self.x,self.y
        self.shape = pymunk.Circle(self.body, 45, (0, 0))
        self.shape.friction = 0.2
        self.shape.collision_type = 1
        
        #self.battery_timer = self.create_timer(self.hz, self.battery_publish)
        self.fribe_timer = self.create_timer(0.5, self.fribe)
        self.statistics_timer = self.create_timer(0.1, self.agent_statistics)
        #self.target_timer = self.create_timer(0.1, self.target_publish)
        self.cmd_vel_timer = self.create_timer(self.hz, self.agent_velocity_pub)
        #self.position_timer = self.create_timer(self.hz, self.position_msg)
        self.state_timer = self.create_timer(0.5, self.state_publish)
        self.cmd_vel_pub = self.create_publisher(Twist,self.name+"/cmd_vel",10) # ágens pozicio
        #self.pos_sub = self.create_subscription(Position,"agent"+str(count)+"/pos",self.pos_callback,10) # ágens pozicio
        #self.cmd_vel_sub = self.create_subscription(Twist,"agent"+str(count)+"/cmd_vel",self.cmd_vel_callback,10)
        #self.position_pub = self.create_publisher(Position,"agent"+str(count)+"/position",10) # ágens pozicio
        #self.canvas_sub = self.create_subscription(Canvas,'/canvas',self.canvas_callback,10)
        #self.count_sub = self.create_subscription(Int16, "/counter" , self.count_callback, 10)
        #self.kocka_sub = self.create_subscription(Position,"agent"+str(count)+"/kocka_position", self.kocka_callback, 10)
        # self.agent_spawn_sub = self.create_subscription(Int16, "/agent_spawn", self.spawn_callback, 10)
       
        #self.recharger_sub = self.create_subscription(Position,"/recharger", self.recharger_callback, 10)
        #self.stop_sub = self.create_subscription(Int16,"/stop", self.stop_callback, 10)
        self.stop_agent_sub = self.create_subscription(Int16,self.name+"/collision",self.stop_agent_callback, 10)
        self.state_pub = self.create_publisher(Int16,self.name+"/state",10)
        self.statistics_pub = self.create_publisher(String,self.name+"/statistics",10)
        #self.battery_pub = self.create_publisher(Float32,self.name+"/battery",10)
        
        # self.distance_sub = self.create_subscription(Distance,self.name+"/distance",self.distance_callback, 10)
        # self.box_distance_sub = self.create_subscription(Distance,self.name+"/box_distance",self.box_distance_callback, 10)
        # self.spec_distance_sub = self.create_subscription(Distance,self.name+"/special_distance",self.special_distance_callback, 10)
        #self.charging_sub = self.create_subscription(Int16,self.name+"/objective",self.objective_callback, 10)
        self.fuzzy_sub = self.create_subscription(String,self.name+"/fuzzy",self.fuzzy_callback, 10)
        self.orientation_sub = self.create_subscription(Float32,self.name+"/orientation",self.orientation_callback, 10)
        self.laserScan_sub  = self.create_subscription(Rays,self.name+"/laserScan",self.laserScan_callback, 10)
        self.target_position_sub = self.create_subscription(Position,self.name+"/target_position",self.target_position_callback, 10)
        self.battery_level = 100.0
        self.nearest_color = 0
        self.ray_start= (self.x,self.y)
        self.sensor = Sensor(self.ray_start, 100, 7, self.shape)

        self.agents = []
        self.boxes = []
        self.special_objects = []

        self.universes = []
     
        self.inputValues = {}
        self.rulebases = []
        self.special_object_orientation_error = []

        self.nearest_agent = None
        self.nearest_box = None
        self.nearest_spec = None
        self.error_raised = False
        
        self.agent_tuple = ()
    def service_callback(self, request, response):
        
        if(request.agent_name == self.name):
            self.get_logger().info(f"Received request: {request.agent_name}")
            self.fuzzy_string = request.fbdl
            response.result =  "ok"
            self.get_logger().info(f"Sending response: {response.result}")
            try:
                self.input_universes()
                self.engine = load_engine_from_string(request.fbdl)
                self.error_raised = True
            except Exception as e:
             
                    self.send_error(str(e))
                    print(f"An error occurred: {e}")
                    
            self.send_request()
        return response


    def stop_service(self, request, response):
        
            self.get_logger().info(f"Received request: {request.request}")
            self.stop = request.request
            response.response =  "got it"

            return response
    
    def send_request(self):
        request = Fbdl.Request()
        request.agent_id =  self.id
        request.fbdl = self.fuzzy_string

        future = self.client.call_async(request)
     
        if future.done():
            if future.result() is not None:
                    response = future.result()
                    self.get_logger().info(f"Received result: {response.result}")
            else:
                    self.get_logger().error('Service call failed')

    def send_error(self,error):
        request = Error.Request()
        request.error = error

        future = self.error_client.call_async(request)
     
        if future.done():
            if future.result() is not None:
                    response = future.result()
                    self.get_logger().info(f"Received result: {response.response}")
            else:
                    self.get_logger().error('Service call failed') 
    # def agents_callback(self, request, response):
        
    #     self.get_logger().info(f"Received request: {request.agents}")
    #     response.response =  "ok"
    #     self.agents = request.agents
    #     for i in request.agents:
    #         self.get_logger().info(f"agent: {i} in the list : number : + {len(request.agents)}")
            
    #     self.get_logger().info(f"Sending response: {response.response}")
    #     self.agent_tuple  = tuple(self.agents)

    #     return response


    def objective_callback(self,msg):
        self.reached_goal = msg.data

    # def stop_callback(self,msg):
    #     self.stop = msg.data
    #     self.get_logger().info(str(self.stop))

    def target_position_callback(self,msg):
        self.target_x = msg.pos_x
        self.target_y = msg.pos_y

    def recharger_callback(self,msg):
        self.recharger_x = msg.pos_x
        self.recharger_y = msg.pos_y

    def fuzzy_callback(self,msg):
        self.fuzzy_string = msg.data
        if len(msg.data) != 0:
        
         self.engine = load_engine_from_string(msg.data)
        

    def stop_agent_callback(self,msg):
        self.collision = msg.data

    # def pos_callback(self,msg):
    #     self.x = msg.pos_x
    #     self.y = msg.pos_y
    #     self.orientation = msg.orientation
        
    def calculate_orientation_error(self, target,agent):

        error_x = target.body.position.x - self.body.position.x
        error_y = target.body.position.y  - self.body.position.y

                # Desired angle to target point
        desired_angle = math.atan2(error_y, error_x)
        orientation_error = (desired_angle - self.orientation) % (2 * math.pi)
        if orientation_error > math.pi:
            orientation_error -= 2 * math.pi
        elif orientation_error < -math.pi:
            orientation_error += 2 * math.pi

        self.inputValues[agent] = orientation_error
        return orientation_error
    
    def orientation_callback(self,msg):
        self.special_object_orientation_error  = []
        self.orientation = msg.data

    def laserScan_callback(self,msg):
        ranges = msg.ranges
        self.min_distance = 100000
        for i in ranges:
            if i < self.min_distance:
                self.min_distance = i

        #self.get_logger().info(str(self.min_distance))        
             
    # def distance_callback(self,msg) :
    #     self.nearest_distance = msg.distance
    #     self.nearest_agent = msg.object_id
    # def box_distance_callback(self,msg) :
    #     self.nearest_box_distance = msg.distance
    #     self.nearest_box = msg.object_id
    # def special_distance_callback(self,msg) :
    #     self.nearest_special_distance = msg.distance
    #     self.nearest_spec = msg.object_id

    def state_publish(self): 
        msg = Int16()
        if( self.state_data <= -0.5):
           
            msg.data= -1
            self.state = -1
            self.state_pub.publish(msg)
       
        else:
            if(self.state_data > 0.0):
                msg.data = 1
                self.state = 1
                self.state_pub.publish(msg)   
        

    def battery_publish(self):
        msg = Float32()
        msg.data = self.battery_level
        self.battery_pub.publish(msg)

    def canvas_callback(self, msg):
        self.canvas_height = msg.canvas_height
        self.canvas_width = msg.canvas_width
        
        
    def calculate_distance(self,agent2):
        #buggy if I use agent1.x and agent2.x and y . body.position proved to be more stable.
        distance = math.sqrt((self.body.position.x - agent2.body.position.x)**2 + (self.body.position.y - agent2.body.position.y)**2)
        return distance   

    def agent_statistics(self):
        json_msg = json.dumps(self.inputValues)

        msg = String()
        msg.data = json_msg

        self.statistics_pub.publish(msg)

    def input_universes(self):
     if(len(self.agents) > 0):
        
        #self.inputValues["target_angle"] = self.orientation
        for agent in self.agents:
            key = agent.name + "_distance"
            orientation_error = agent.name + "_orientation_error"
            agent_state = agent.name +"_state"
            agent_speed = agent.name +"_speed"
            agent_goal = agent.name +"_reached_objective"
            agent_collision = agent.name +"_collision"
            agent_raycast = agent.name +"_ray_collision"
            output_speed = "speed"
            output_angle = "angle"
            output_state = "agent_status"
            keys_to_check = [key, orientation_error, agent_state, agent_speed, agent_goal, agent_collision,agent_raycast]

            for universe in keys_to_check:
                if universe not in self.universes:
                    self.universes.append(universe)
            
            output_keys_to_check = [output_speed,output_angle,output_state]

            for rulebase in output_keys_to_check:
                if rulebase not in self.rulebases:
                    self.rulebases.append(rulebase)
            
            self.calculate_orientation_error(agent,orientation_error)
            
            distance = self.calculate_distance(agent)

            self.inputValues[key] = distance
            self.inputValues[agent_state] = agent.state
            self.inputValues[agent_speed] = agent.agent_speed
            self.inputValues[agent_goal] = agent.reached_goal
            self.inputValues[agent_collision] = agent.collision
            self.inputValues[agent_raycast] = agent.min_distance

        if(len(self.special_objects) > 0):

            for i in self.special_objects:
                key = str(i.id) + "_distance"
                key2 = (str(i.id)+"_angle")
                key3 = (str(i.id)+"_color")
                orientation_error = str(i.id) + "_orientation_error"


                keys_to_check = [key, key2, key3, orientation_error]

                for universe in keys_to_check:
                    if universe not in self.universes:
                        self.universes.append(universe)
               
                distance = self.calculate_distance(i)
                if(distance < 20):
                     
                    self.reached_goal = i.id

                self.inputValues[key] = distance
                self.inputValues[key2] = i.body.angle
                self.inputValues[key3] = float(i.color)
                
                self.calculate_orientation_error(i,orientation_error)
                
      
    def fribe(self):
        self.input_universes()

        if(self.engine is None):

                pass
        else:
                # target_angle_universe = "target_angle"
                # self.universes.append(target_angle_universe)
                # target_angle = self.orientation

                try:

                    
                    #self.get_logger().info(f"{self.inputValues}")
                    # self.inputValues[target_angle_universe] = target_angle
                    # for universe_name in self.engine.universe_names:
                    
                    #      self.get_logger().info('Universes found: '+str(universe_name))
                    
                    self.engine.calc_consequences(self.inputValues)
                    
                    self.speed = self.engine.get_state('speed')
                    self.angle = self.engine.get_state('angle')
                    self.state_data = self.engine.get_state('agent_status')  
                
                  
                except Exception as e: 
                        print(f"An error occurred: {e}")
                        if(self.error_raised):
                            error = str(e)
                            self.send_error(error)
                    
                            self.error_raised = False
            #self.get_logger().info("Speed: "+str(self.speed)+" Angle: "+str(self.angle))
            #self.get_logger().info("Speed: "+str(self.speed)+" Angle: "+str(self.angle))
            #print(self.orientation_error)
            #print( self.tiredness)
            #self.get_logger().info(f" tiredness {self.tiredness}")
            #self.get_logger().info(f"selected target: {self.selected_target}")
            
            

    def agent_velocity_pub(self):
        # if(self.tiredness == 1):
        #     self.target_x = self.recharger_x
        #     self.target_y = self.recharger_y
        # else:
        #    self.target_x = self.kocka_x
        #    self.target_y = self.kocka_y

        
    #     # Calculate angular velocity to align with the target orientation
       
        msg = Twist() 
        # Linear velocity in the x-axis.
        #math.pi /2 for 90-degree collision detection on the front.
       

        
        if(self.stop == 1):
                    msg.linear.x = 0.0
                    msg.angular.z = 0.0
        else: 
         
            msg.linear.x = self.speed
            msg.angular.z =  self.angle

        self.agent_speed = msg.linear.x
        self.cmd_vel_pub.publish(msg)
