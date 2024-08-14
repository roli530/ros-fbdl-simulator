import math
from ros2_fuzzy.agent import Agent
import rclpy
from rclpy.node import Node
import threading
from ros2_fuzzy.box import Box
from ros2_fuzzy.object import  Special_Object
from geometry_msgs.msg import Twist

from std_msgs.msg import Int16, String, Float32
from subprocess import Popen, PIPE
from custom_msgs.msg import Position, Canvas, Names, Files, Obstacles,SpecObject,Distance,Rays, Target
from custom_msgs.srv import Delete, NewAgent, DeleteObstacle, CanvasDimensions,SendState,Fbdl,Universes
from custom_msgs.srv import CreateObstacle, CreateSpecialObject,ChangeDimensions, ChangeColor,Load,Agents,Follow
from custom_msgs.srv import Stop, Weight

from rclpy.executors import MultiThreadedExecutor
import os
import time
import json
import pymunk
import pymunk.util
import sqlite3



count = 0
nodes = []
class Controller(Node):

    def __init__(self, name, executor):
        super().__init__(name)
        #Services
        self.create_service(Fbdl, 'agent_fbdl', self.fbdl_service)
        self.create_service(Delete, 'delete_agent', self.delete_agent_service)
        self.create_service(NewAgent, 'new_agent', self.new_agent_callback)
        self.create_service(DeleteObstacle, 'delete_obstacle', self.delete_obstackle_callback)
        self.create_service(CanvasDimensions, 'canvas_dimensions', self.canvas_dimensions_callback)
        self.create_service(CreateObstacle, 'create_obstacle', self.create_obstacle_callback)
        self.create_service(CreateSpecialObject, 'create_special_object', self.create_special_object_callback)
        self.create_service(DeleteObstacle, 'delete_special_object', self.delete_special_object_callback)
        self.create_service(ChangeDimensions, 'change_dimensions', self.change_obstacle_dimension_callback)
        self.create_service(ChangeColor, 'change_color', self.change_color_callback)
        self.create_service(SendState, 'simulation_state', self.send_simulation_state_callback)
        self.create_service(Weight, 'change_weight', self.weight_change_service)
        self.create_service(Universes, 'universes', self.universes_service)
        self.create_service(Load, 'load', self.load_callback)
        self.create_service(Stop, 'stop', self.stop_service)
        #self.client = self.create_client(Agents, 'agents')

        #Publishers, Subscribers,Timers
        #self.spawn_sub = self.create_subscription(Int16, "/agent_spawn",self.listener_callback,10)
        #self.name_pub = self.create_publisher(Names,"/agent_names", 10) # ágens név
        #self.names_sub = self.create_subscription(String,"/names",self.name_callback, 10) # nevek
        #self.count = 0.0
        #self.name_timer = self.create_timer(0.5, self.robot_name_msg)
        #self.count_pub = self.create_publisher(Int16,"/count",10) # ágens pozicio
  
        #self.timer =  self.create_timer(0.1, self.count_publish)
        #self.agent_pub = self.create_publisher(Int16,"/agents", 10) # ágens név
        #self.fuzzy_sub = self.create_subscription(String, "/fuzzy", self.fuzzy_callback,10)
        self.state_sub = self.create_subscription(Obstacles, "/obstacle_state", self.state_callback,10)
        #self.load = self.create_subscription(String, "/load", self.load_callback,10)
        self.obstacle_pos_pub = self.create_publisher(String, "/obstacle_pos", 10)
        self.spec_object_pos_pub = self.create_publisher(SpecObject, "/spec_obj_position", 10)
        self.canvas_sub = self.create_subscription(Canvas,'/canvas',self.canvas_callback,10)
        self.kocka_sub = self.create_subscription(Position,"/agent_spawner_position", self.kocka_callback, 10)
        #self.recharger_sub =  self.create_subscription(Position,"/recharger", self.recharger_callback, 10)
        self.stop_sub = self.create_subscription(Int16, "/stop",self.stop_callback, 10)
        self.spec_obj_sub = self.create_subscription(SpecObject, "/spec_obj_state",self.spec_obj_callback, 10)
        #self.agent_delete_sub  = self.create_subscription(String, "/delete_agent", self.delete_agent_callback,10)
        #self.create_timer(0.1,self.state_publish)
        self.create_timer(0.1,self.special_object_position_publish)
        #self.create_timer(5,self.list_space)
        self.timer = self.create_timer(0.1,self.simulate)
        self.position_timer = self.create_timer(0.1,self.position_pub)
        self.orienation_timer = self.create_timer(0.1, self.orientation_publish)   
        self.timer_paused = False
        #Variables
        self.executor = executor
        self.space = pymunk.Space()
        self.space.damping = 0.5
        self.new_count = 0.0
        self.name = name
        self.fuzzy_data = ""
        self.msg = Names()      
        self.agents = []
        self.boxes = []
        self.spec_objects = []
        self.checkpoints = []
        
        self.all_objects = []
        self.fbdl_publisher = {}
        self.fbdl_subscriber = {}
        self.closest_agent_distance = {}
        self.closest_box_distace = {}
        self.closest_special_object_distance = {}
        self.closest_agent = {}
        self.closest_spec = {}
        self.closest_box = {}
        self.body_to_agent =  {}
    
        self.agent_position_publishers= {}
        self.stop_publisher = {}
        self.charging_agent = {}
        self.pos_publishers = {}
        self.agent_follow_id = {}
        self.cmd_vel_subscribers = {}
        self.agent_orientation_publisher = {}
        self.agent_target_publisher = {}
        self.agent_target_id_sub = {}
        self.agent_rays = {}
        self.agent_publishers = {}
        self.canvas_height = 0.0
        self.canvas_width = 0.0
        self.pause_simulation = 0
        self.load_json = ""
        self.json_state = ""
        self.json_string = ""
        self.load_state = False
        self.space_cleared = False
        self.changed_weight = 0
       
        #self.agent_id = []
       
        self.create_boundary()

        self.db_connection = sqlite3.connect('simulation.db',check_same_thread=False)
        self.db_cursor = self.db_connection.cursor()
        self.create_tables()
        self.drop_tables()
        self.create_tables()
        
        current_directory = os.getcwd()
        self.get_logger().info(str(current_directory))

    def create_tables(self):
        # Define the SQL command to create the table
        create_obstacle_table_sql = '''
        CREATE TABLE IF NOT EXISTS obstacles (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            x FLOAT,
            y FLOAT,
            width FLOAT,
            height FLOAT,
            color STRING
        
        );
        '''

        create_special_object_table_sql = '''
        CREATE TABLE IF NOT EXISTS special_objects (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            x FLOAT,
            y FLOAT,
            radius FLOAT,
            color STRING
        
        );
        '''

        create_agent_table = '''
        CREATE TABLE IF NOT EXISTS agents (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            name STRING,
            x FLOAT,
            y FLOAT,
            orientation FLOAT,
            fbdl STRING,
            state INT

        );
        '''
        # Execute the SQL command to create the table
        self.db_cursor.execute(create_obstacle_table_sql)
        self.get_logger().info("obstacles table created")
        self.db_cursor.execute(create_special_object_table_sql)
        self.get_logger().info("spec table created")
        self.db_cursor.execute(create_agent_table)
        self.get_logger().info("agents table created")
        
    def clear_tables(self):
        self.db_cursor.execute("DELETE FROM obstacles")
        self.db_cursor.execute("DELETE FROM special_objects")
        self.db_cursor.execute("DELETE FROM agents")
        self.db_connection.commit()

    def drop_tables(self):
        self.db_cursor.execute("DROP TABLE obstacles")
        self.db_cursor.execute("DROP TABLE special_objects")
        self.db_cursor.execute("DROP TABLE agents")
        self.db_connection.commit()  
    def close_connection(self):
        
        self.db_cursor.close()
        self.db_connection.close()

    def new_agent_callback(self, request, response):
        # global count
       
        # count += 1

        # msg = Int16()
        # msg.data = 1
        # self.agent_pub.publish(msg)
        #cmd_str = "ros2 run my_robot_controller agent_node --ros-args -r __node:=agent"+str(self.count)
        #Popen(['xterm', '-e', cmd_str], stdin=PIPE)
        
        #nodes.append(Agent("agent"+str(count),count))
        try:
            self.db_cursor.execute("INSERT INTO agents (name,x,y,orientation,fbdl,state) VALUES (?,?,?,?,?,?)", (request.agent_name,self.target.x,self.target.y,0.0,"",0))
            
            
            self.db_connection.commit()

            self.db_cursor.execute("SELECT * FROM agents ORDER BY id DESC LIMIT 1")

            # Fetch the last inserted record
            row = self.db_cursor.fetchone()
            if row:
                id = row[0]
                name = row[1]
                x = row[2]
                y = row[3]
                orientation = row[4]
                fbdl = row[5]
                state = row[6]
                


                agent = Agent(id,name,x,y,state)
            
                topic_name = agent.name+"/cmd_vel"
                
                #self.agent_id.append(agent.id)
                self.executor.add_node(agent)
        
                self.body_to_agent[agent.body] = agent
                self.agent_position_publishers[agent.name] = self.create_position_publishers(agent.name)
                
                self.fbdl_publisher[agent.name] = self.create_publisher_for_agent(agent.name)
                self.closest_agent_distance[agent.name] = self.create_distance_publisher(agent.name)
                self.closest_special_object_distance[agent.name] = self.create_special_distance_publisher(agent.name)
                self.closest_box_distace[agent.name] = self.create_box_distance_publisher(agent.name)
                self.stop_publisher[agent.name] = self.create_stop_publisher(agent.name)
                self.charging_agent[agent.name] = self.create_charging_agent_publisher(agent.name)
                self.agent_orientation_publisher[agent.name] = self.create_agent_orientation_publisher(agent.name)
                self.agent_rays[agent.name] = self.create_agent_rays_publisher(agent.name)
                self.agent_target_publisher[agent.name] = self.create_agent_target_publisher(agent.name)
                
            
                self.cmd_vel_subscribers[agent.name] = self.create_subscription(Twist,topic_name, self.generate_callback( agent),10)
            
                self.space.add(agent.body,agent.shape)
                self.agents.append(agent)
                self.all_objects.append(agent)
                self.universes = []

            row_dict = {
                "id": row[0],
                "name": row[1],
                "x": row[2],
                "y": row[3],
                "orientation" : row[4],
                "fbdl" : row[5],
                "state" : row[6]
            }

            # Convert the dictionary to a JSON string
            row_json = json.dumps(row_dict)
            response.result = row_json
        except Exception as e: 
                print(f"An error occurred: {e}")

        #self.send_request()
        return response

    # def send_request(self):
    #     request = Agents.Request()
    #     request.agents =  self.agent_id
    #     future = self.client.call_async(request)
     
           
    #     if future.done():
    #         if future.result() is not None:
    #                 response = future.result()
    #                 self.get_logger().info(f"Received result: {response.response}")
    #         else:
    #                 self.get_logger().error('Service call failed')
         
    def setBoundary(self):
        static_body = self.space.static_body

        static_lines = [
            pymunk.Segment(static_body, (0, 0), (0, self.canvas_height), 0),
            pymunk.Segment(static_body, (0, self.canvas_height), (self.canvas_width, self.canvas_height), 0),
            pymunk.Segment(static_body, (self.canvas_width, 0), (self.canvas_width, self.canvas_height), 0),
            pymunk.Segment(static_body, (0, 0), (self.canvas_width, 0), 0),
        ]
            
        for line in static_lines:
                line.elasticity = 1
                line.friction = 0
                line.collision_type = 5
                
        self.space.add(*static_lines)
        
    def canvas_dimensions_callback(self, request, response):
        global count 
  
        self.get_logger().info(f"width: {request.width} height:{request.height} ")
        response.agent_number =  count
        self.get_logger().info(f"Sending response: {response.agent_number}")

        self.remove_everything()
        self.remove_everything_from_space()
       
  
        self.setBoundary()

        return response
    
    def weight_change_service(self, request, response):
        
        self.get_logger().info(f"weight: {request.weight}")
        self.changed_weight = request.weight
        response.response = "changed"
        self.get_logger().info(f"Sending response: {response.response}")
        try:
            for existing in self.spec_objects:
                if(request.id == existing.id and existing.body in self.space.bodies):
                    self.space.remove(existing.body,existing.shape) 
                    new_moment = pymunk.moment_for_circle(request.weight, 0, 30)
                    existing.body = pymunk.Body(request.weight,new_moment)
                    existing.shape = pymunk.Circle(existing.body,30) 
                    
                    existing.shape.collision_type = 0
                    existing.body.position = existing.x,existing.y
                    self.space.add(existing.body, existing.shape)
        except Exception as e:
                self.get_logger().info(f"Sending response: {str(e)}")
        return response
    

    def delete_agent_service(self, request, response):
        
        self.get_logger().info(f"Received request: {request.agent_name}")
        response.result =  "ok"
        self.get_logger().info(f"Sending response: {response.result}")
        try:
            for agent in self.agents:
                if(request.agent_name == agent.name):
                    self.get_logger().info("agent " + agent.name +" deleted")
                    self.db_cursor.execute("DELETE FROM agents WHERE name = ?", (request.agent_name,))
                    self.executor.remove_node(agent)
                    self.agents.remove(agent)
                    self.all_objects.remove(agent)
                    self.space.remove(agent.shape, agent.body)
                    #self.agent_id.remove(agent.id)
                    agent.destroy_node()
        except Exception as e:
                self.get_logger().info(f"Sending response: {str(e)}")
        return response
    
    def stop_service(self, request, response):
        
            self.get_logger().info(f"Received request: {request.request}")
            self.pause_simulation = request.request
            response.response =  "got it"
            if(self.pause_simulation == 1):
                self.timer_paused = True
            else:
                self.timer_paused = False

            return response

    def delete_obstackle_callback(self, request, response):
        
        self.get_logger().info(f"Received request: {request.obstacle_id}")
        response.result =  "ok"
        self.get_logger().info(f"Sending response: {response.result}")

        try:
            for box in self.boxes:
                if request.obstacle_id == box.id and box.body in self.space.bodies :
                    self.space.remove(box.body,box.shape)      
                    self.all_objects.remove(box)
                    self.boxes.remove(box)
                    self.db_cursor.execute("DELETE FROM obstacles WHERE id = ?", (box.id,))
        except Exception as e:
            self.get_logger().info(f"Sending response: {str(e)}")
        self.db_connection.commit()
        select_query = "SELECT * FROM obstacles"

        # Execute the SELECT statement
        self.db_cursor.execute(select_query)

        # Fetch the selected data
        data = self.db_cursor.fetchall() 
        for row in data:
            print(row)

        return response
    

    def delete_special_object_callback(self, request, response):
        
        self.get_logger().info(f"Received request: {request.obstacle_id}")
        response.result =  "ok"
        self.get_logger().info(f"Sending response: {response.result}")
        try:
            for spec_obj in self.spec_objects:
                if request.obstacle_id == spec_obj.id :
                    #self.space.remove(spec_obj.body,spec_obj.shape)      
                    self.all_objects.remove(spec_obj)
                    self.spec_objects.remove(spec_obj)
                    self.db_cursor.execute("DELETE FROM special_objects WHERE id = ?", (request.obstacle_id,))
                    if spec_obj in self.checkpoints:
                        self.checkpoints.remove(spec_obj)
        except Exception as e:
            self.get_logger().info(f"Sending response: {str(e)}")
        return response
    
    def create_obstacle_callback(self,request,response):
        id = request.id
        x = request.x
        y = request.y
        width= request.width
        height = request.height
        
        collision_type = request.collision_type

        self.db_cursor.execute("INSERT INTO obstacles ( x, y,width,height) VALUES ( ?, ?,?,?)", ( x, y,width,height))
        select_query = "SELECT * FROM obstacles"

        # Execute the SELECT statement
        self.db_cursor.execute(select_query)
        self.db_connection.commit()
        # Fetch the selected data
        data = self.db_cursor.fetchall() 
        self.db_cursor.execute("SELECT * FROM obstacles ORDER BY id DESC LIMIT 1")

        # Fetch the last inserted record
        row = self.db_cursor.fetchone()
        if row:
            id = row[0]
            x = row[1]
            y = row[2]
            width = row[3]
            height = row[4]

            box = Box(id,x,y,width,height, collision_type)

            self.boxes.append(box)
                    
            self.all_objects.append(box)

            self.space.add(box.body, box.shape)

            self.get_logger().info(f"Obstacle created with id: {box.id},{box.x},{box.y}")

        for row in data:
           
            print(row)

        row_dict = {
            "id": row[0],
            "x": row[1],
            "y": row[2],
            "width": row[3],
            "height" : row[4]
        }

        # Convert the dictionary to a JSON string
        row_json = json.dumps(row_dict)
        response.response = row_json

        
        return response
    
    def change_obstacle_dimension_callback(self,request,response):
            try:
                for i in self.boxes:
                    if(request.id == i.id and i.body in self.space.bodies):
                        
                        self.space.remove(i.body,i.shape)         
                        new_width = request.new_width  # New width
                        new_height = request.new_height  # New height
                        
                        # Recalculate the moment of inertia
                        new_moment = pymunk.moment_for_box(i.body.mass, (new_width, new_height))
                        i.body = pymunk.Body(i.body_mass,new_moment)
                        # Create a new shape with updated dimensions
                        i.shape = pymunk.Poly.create_box(i.body, (new_width, new_height))
                        i.shape.collision_type = 2
                        i.body.position = i.x,i.y
                        condition =  f"id = {request.id}"
                        # Add the new shape to the space
                        self.space.add(i.body, i.shape)
                        self.db_cursor.execute(f"UPDATE obstacles SET width = ?, height = ? WHERE {condition}", (new_width, new_height))

                        # Commit the changes to the database
                        self.db_connection.commit()
                response.result =  "ok"
            except Exception as e:
                self.get_logger().info(f"Sending response: {str(e)}")
            return response
            
            # box.body.position = x+width/2,y+height/2
            # self.boxes.append(box)
            # self.all_objects.append(box)
            # self.space.add(box.body, box.shape)
            # self.get_logger().info("nooooooo")

                   
    def create_special_object_callback(self,request,response):
        id = request.id
        x = request.position_x
        y = request.position_y
        radius = request.radius
        color = request.color
        self.db_cursor.execute("INSERT INTO special_objects ( x, y,radius,color) VALUES ( ?, ?,?,?)", ( x, y,radius,color))
        self.db_connection.commit()
        self.db_cursor.execute("SELECT * FROM special_objects ORDER BY id DESC LIMIT 1")

        # Fetch the last inserted record
        row = self.db_cursor.fetchone()
        if row:
            id = row[0]
            x = row[1]
            y = row[2]
            radius = row[3]
            color = row[4]
        
            special_object = Special_Object(id,x,y, radius, str(color), 0)
        
            
            self.spec_objects.append(special_object)
                    
            self.all_objects.append(special_object)
            
            self.space.add(special_object.body, special_object.shape)

            self.get_logger().info(f"Special object created with id: {special_object.id}")

            if(str(color) == "2"):
                self.checkpoints.append(special_object)
            row_dict = {
            "id": row[0],
            "x": row[1],
            "y": row[2],
            "radius": row[3],
            "color" : row[4]
        }

        self.get_logger().info(f"checkpointok száma : {len(self.checkpoints)}")

        # Convert the dictionary to a JSON string
        row_json = json.dumps(row_dict)
        response.result = row_json

        return response
    

    def change_color_callback(self, request, response):
        
        self.get_logger().info(f"Received request: {request.id}, {request.color}")
        response.result =  "ok"
        self.get_logger().info(f"Sending response: {response.result}")
        try:
            for spec_obj in self.spec_objects:
                if request.id == spec_obj.id :
                    spec_obj.color = request.color
                    
        except Exception as e:
            self.get_logger().info(f"Sending response: {str(e)}")
        return response

    def fbdl_service(self, request, response):
        
            self.get_logger().info(f"Received request: {request.agent_id}{request.fbdl}")
            response.result =  "ok"
            self.get_logger().info(f"Sending response: {response.result}")
            self.db_cursor.execute("UPDATE agents SET fbdl = ? WHERE id = ?", ( request.fbdl,request.agent_id))
            self.get_logger().info("ok")

            return response
    
    def universes_service(self, request, response):
        
            self.get_logger().info(f"Received request: {request.request}")
           
            if(len(self.agents)> 0):
                response.universes = self.agents[0].universes
                response.rulebases = self.agents[0].rulebases
                self.get_logger().info(f"Sending response: {response.universes},{response.rulebases}")
        
            
            return response
    

    def send_simulation_state_callback(self,request,response):
            self.get_logger().info(f"Received request: {request.request}")

            for i in self.spec_objects:
                self.db_cursor.execute("UPDATE special_objects SET x = ?, y = ?, radius = ?, color = ? WHERE id = ?", (i.x, i.y, i.radius, i.color, i.id))

            for i in self.agents:
               self.db_cursor.execute("UPDATE agents SET name = ?, x = ?, y = ?, orientation = ?, fbdl = ?, state = ? WHERE id = ?", (i.name,i.x,i.y,i.orientation,i.fuzzy_string,i.state, i.id)) 
            
            for i in self.boxes:
                self.db_cursor.execute("UPDATE obstacles SET x = ?, y = ?, width = ?, height = ? WHERE id = ?", ( i.x, i.y,i.width,i.height,i.id))
            
            response.response =  "ok"
            self.get_logger().info(f"Sending response: {response.response}")

            data = {
                    "obstacles" : [],
                    "agents" : [],
                    "special_objects" : []

                }
            self.db_cursor.execute(f"SELECT * FROM obstacles")
            rows = self.db_cursor.fetchall()

            # Loop through the rows and print the data
            for row in rows:
                self.get_logger().info(str(row))
                id = row[0]
                
                x = row[1]
                y = row[2]
                width = row[3]
                height = row[4]

                data["obstacles"].append({
                    'id': id,
                    'x': x,
                    'y': y,
                    'width': width,
                    'height': height
                

                })

            self.db_cursor.execute(f"SELECT * FROM agents")
            rows2 = self.db_cursor.fetchall()

            # Loop through the rows and print the data
            for row in rows2:
                self.get_logger().info(str(row))

                id = row[0]
                name = row[1]
                x = row[2]
                y = row[3]
                orientation = row[4]
                fbdl = row[5]
                state = row[6]

                data["agents"].append({
                    'id': id,
                    'name': name,
                    'x': x,
                    'y': y,
                    'orientation' : orientation,
                    'fbdl' : fbdl,
                    'state' : state
                })
                self.get_logger().info(f"{row}")

            self.db_cursor.execute(f"SELECT * FROM special_objects")
            specials = self.db_cursor.fetchall()

            # Loop through the rows and print the data
            for row in specials:
                self.get_logger().info(str(row))
                id = row[0]
                x = row[1]
                y = row[2]
                radius = row[3]
                color = str(row[4])


                data["special_objects"].append({
                    'id': id,
                    'x': x,
                    'y': y,
                    'radius' : radius,
                    'color' : color
                })


            
            json_string = json.dumps(data)

            response.response = json_string

            return response

    def create_publisher_for_agent(self,agent_name):
        topic_name = agent_name+"/fuzzy"
        publisher = self.create_publisher(String,topic_name,10)
        return publisher
    
    def create_position_publishers(self,agent_name):
        
        topic_name = agent_name+"/position"
        publisher = self.create_publisher(Position,topic_name,10)
        return publisher
    
    def create_distance_publisher(self,agent_name):
        topic_name = agent_name+"/distance"
        publisher = self.create_publisher(Distance,topic_name,10)
        return publisher
    
    def create_special_distance_publisher(self,agent_name):
        topic_name = agent_name+"/special_distance"
        publisher = self.create_publisher(Distance,topic_name,10)
        return publisher
    
    def create_box_distance_publisher(self,agent_name):
        topic_name = agent_name+"/box_distance"
        publisher = self.create_publisher(Distance,topic_name,10)
        return publisher
    
    def create_stop_publisher(self,agent_name):
        topic_name = agent_name+"/collision"
        publisher = self.create_publisher(Int16,topic_name,10)
        return publisher
    
    def create_charging_agent_publisher(self,agent_name):
        topic_name = agent_name+"/charging"
        publisher = self.create_publisher(Int16,topic_name,10)
        return publisher
    
    def create_agent_orientation_publisher(self,agent_name):
        topic_name = agent_name+"/orientation"
        publisher = self.create_publisher(Float32,topic_name,10)
        return publisher
    
    def create_agent_rays_publisher(self,agent_name):
        topic_name = agent_name+"/laserScan"
        publisher = self.create_publisher(Rays,topic_name,10)
        return publisher
    
    def create_agent_target_publisher(self, agent_name):
        topic_name = agent_name+"/target_position"
        publisher = self.create_publisher(Position,topic_name,10)
        return publisher
    
    def stop_callback(self,msg):
        self.pause_simulation = msg.data
        if(msg.data == 3):
            self.timer_paused = True
        else:
            self.timer_paused = False

    def recharger_callback(self,msg):
        self.recharger.x = msg.pos_x
        self.recharger.y = msg.pos_y
        self.recharger.body.position =  self.recharger.x,self.recharger.y

    def kocka_callback(self,msg):
        self.target.x  = msg.pos_x
        self.target.y = msg.pos_y     
        self.target.body.position =  self.target.x,self.target.y


    def lerp(self, v1,v2,time):
        return v1 * (1-time) + v2 * time

    
    def canvas_callback(self,msg):
        
        self.canvas_height = msg.canvas_height
        self.canvas_width = msg.canvas_width



    # service needed from this.
    def create_boundary(self):
        
        self.recharger = Special_Object("recharger",3)
        self.all_objects.append(self.recharger)
        #self.space.add(self.recharger.body,self.recharger.shape)
        
        self.target = Special_Object("target",2)
        self.all_objects.append(self.target)
        #self.space.add(self.target.body,self.target.shape)


    def generate_callback(self,agent):
        return lambda msg  : self.cmd_vel_callback(msg, agent)
    
    def cmd_vel_callback(self, msg, agent):
        agent.velx = msg.linear.x * 100
        agent.vely = msg.angular.z *5
        
   
  
        # if(len( self.checkpoints) > 0 and not charging ):
        #     charging = False
        #     #self.get_logger().info(f"{self.checkpoints[0]}")
        #     if msg.data < len(self.checkpoints):
        #         target_checkpoint = self.checkpoints[msg.data]

        #         if self.calculate_distance(agent, target_checkpoint) < 20:
        #             if msg.data + 1 < len(self.checkpoints):  # Ensure there's a next element
        #                 agent.target = msg.data + 1
        #                 msg_p.pos_x = target_checkpoint.x
        #                 msg_p.pos_y = target_checkpoint.y
        #                 self.agent_target_publisher[agent.name].publish(msg_p)
        #                 self.get_logger().info(f"checkpoint reached, next checkpoint: {target_checkpoint.id}")
        #             else:
        #                 # Optionally, handle reaching the end of the checkpoints
        #                 agent.target = 0
        #                 print("Reached the last checkpoint.")
        #         else:
        #             msg_p.pos_x = target_checkpoint.x
        #             msg_p.pos_y = target_checkpoint.y
        #             self.agent_target_publisher[agent.name].publish(msg_p)

        #     else: 
        #         if msg.data >= len(self.checkpoints):
        #             agent.target -= 1  # Adjust the target to match the updated list
                   
        #         else:
        #             agent.target = 0
                    
        #             print("Checkpoint no longer exists. Handling this scenario...")
            

    def orientation_publish(self):


        msg = Float32()
        for agent in self.agents:
            if len(self.agents) > 0:
                agent.agents = self.agents
                agent.special_objects = self.spec_objects
                agent.body.angular_velocity = agent.vely

                lin_x = agent.velx * math.cos(agent.body.angle)
                lin_y = agent.velx * math.sin(agent.body.angle)
                
                agent.body.velocity = (lin_x, lin_y)
                
                agent.x = agent.body.position.x
                agent.y = agent.body.position.y

                msg.data = agent.body.angle

                self.agent_orientation_publisher[agent.name].publish(msg)  


    def fuzzy_callback(self, msg):
        self.fuzzy_data = msg.data
        #print(msg.data)

        fuzzy_msg = String()
        fuzzy_msg.data = msg.data

        # for i in self.agents:
        #     self.agent_publishers[i.name].publish(fuzzy_msg)

    def position_pub(self):
        msg = Position()
        for agent in self.agents:
            agent.sensor.ray_start = (agent.x,agent.y)
            
            msg.pos_x = float(agent.x)
            msg.pos_y = float(agent.y)
            msg.orientation = agent.body.angle                    
            msg.name = agent.name
            
            self.agent_position_publishers[agent.name].publish(msg)

  
    def remove_agents(self):
         for agent in self.agents[:]: 
            print(agent.name + " removed")
            
            self.executor.remove_node(agent)
            self.agents.remove(agent) 
            self.destroy_subscription(self.cmd_vel_subscribers[agent.name])
            self.destroy_publisher(self.closest_agent_distance[agent.name])
            self.destroy_publisher(self.agent_position_publishers[agent.name])
            self.destroy_publisher(self.agent_orientation_publisher[agent.name])
            self.destroy_publisher(self.agent_rays[agent.name])
            self.destroy_publisher(self.fbdl_publisher[agent.name])
            self.destroy_publisher(self.stop_publisher[agent.name])
            time.sleep(0.1)

            try:
                agent.destroy_node()
            except Exception as e:
               
                print(f"An error occurred: {e}")
                

    def remove_everything(self):
        for i in self.all_objects:
            self.all_objects.remove(i)
        
        for i in self.boxes:
            self.boxes.remove(i)
            
        for i in self.spec_objects:
            self.spec_objects.remove(i)
           
    def remove_everything_from_space(self):
        try:   
            self.remove_everything()
            self.remove_agents()
        except Exception as e:
            self.get_logger().info({str(e)})

    def load_callback(self ,request, response):
        
        self.drop_tables()
        self.create_tables()
        self.remove_everything_from_space()
        global count

        self.load_state = True

        if self.load_state == True:
            for body in self.space.bodies:
                self.space.remove(body)
            for shape in self.space.shapes:
                self.space.remove(shape)
            self.space_cleared = True
            self.load_state = False
        
        if(self.space_cleared == True):
            self.setBoundary()
            msg_p= Position()
            fuzzy_msg = String()

            with open("src/web_ros2/src/public/maps/"+request.filename, "r") as json_file:
            # Load and parse the JSON data into a Python dictionary
                self.load_json = json.load(json_file)
        
                print("file opened")

            count = len( self.load_json["agents"])
            self.opened_json_file = json.dumps(self.load_json)
            # Initialize a flag to check if the agent exists
            if(self.canvas_width != 0 and self.canvas_height != 0):

            
                for i in range(len( self.load_json["agents"])):
                        agent_id = self.load_json["agents"][i]["id"]
                        agent_name = self.load_json["agents"][i]["name"]
                        x =  self.load_json["agents"][i]["x"] * self.canvas_width
                        y =  self.load_json["agents"][i]["y"] * self.canvas_height
                        orientation = float( self.load_json["agents"][i]["orientation"])
                        fbdl = self.load_json["agents"][i]["fuzzy"]
                        state = self.load_json["agents"][i]["state"]
                        fuzzy_msg.data =  fbdl
                        
                        agent = Agent(agent_id,agent_name,x,y,state)
                        
                        agent.orientation = orientation
                        self.db_cursor.execute("INSERT INTO agents (id,name,x,y,orientation,fbdl,state) VALUES (?,?,?,?,?,?,?)", (agent_id,agent.name,agent.x,agent.y,agent.orientation,fbdl,agent.state))
                        self.db_connection.commit()
                        self.agent_position_publishers[agent.name] = self.create_position_publishers(agent.name)
                        topic_name = agent.name+"/cmd_vel"
                        
                        self.fbdl_publisher[agent.name] = self.create_publisher_for_agent(agent.name)
                        self.closest_agent_distance[agent.name] = self.create_distance_publisher(agent.name)
                        self.stop_publisher[agent.name] = self.create_stop_publisher(agent.name)
                        self.charging_agent[agent.name] = self.create_charging_agent_publisher(agent.name)
                        self.agent_orientation_publisher[agent.name] = self.create_agent_orientation_publisher(agent.name)
                        self.agent_rays[agent.name] = self.create_agent_rays_publisher(agent.name)
                    
                        self.body_to_agent[agent.body] = agent
                        self.cmd_vel_subscribers[agent.name] = self.create_subscription(Twist,topic_name, self.generate_callback( agent),10)
                        #self.agent_target_id_sub[agent.name] = self.create_subscription(Target,agent_target_id_topic, self.agent_target_id_callback(agent),10)
                        
                        agent.body.position = agent.x,agent.y
                        
                        agent.body.angle = agent.orientation
                        
                                    
                        msg_p.pos_x = agent.body.position.x
                        msg_p.pos_y = agent.body.position.y
                        msg_p.orientation = agent.body.angle

                        self.agent_position_publishers[agent.name].publish(msg_p)

                        self.fbdl_publisher[agent.name].publish(fuzzy_msg)

                        msg_ori = Float32()
                        msg_ori.data = agent.body.angle

                        self.agent_orientation_publisher[agent.name].publish(msg_ori)  
                        self.all_objects.append(agent)
                        self.agents.append(agent)
                        self.executor.add_node(agent)
                        self.space.add(agent.body,agent.shape)
        
                for i in range(len(self.load_json["obstackles"])):
                        
                        obstacle = self.load_json["obstackles"][i]
                        
                        id = obstacle.get("id")
                        x = obstacle.get("x") * self.canvas_width
                        y = obstacle.get("y") * self.canvas_height
                        width = obstacle.get("width")
                        height = obstacle.get("height")
                    
                        box = Box(id,x, y, width, height, 2)
                        self.db_cursor.execute("INSERT INTO obstacles (id, x, y,width,height) VALUES (?,?, ?,?,?)", (id, x, y,width,height))
                        self.db_connection.commit()
                        box.body.position = x+width/2,y+height/2
                        self.boxes.append(box)
                        self.get_logger().info(str(box.id))
                    
                        self.all_objects.append(box)
                        self.space.add(box.body,box.shape)

                for i in range(len(self.load_json["special_objects"])):
                        
                        special_object = self.load_json["special_objects"][i]
                        
                        id = special_object.get("id")
                        x = special_object.get("x") * self.canvas_width
                        y = special_object.get("y") * self.canvas_height
                        radius = special_object.get("radius")
                        color = special_object.get("color")
                        self.db_cursor.execute("INSERT INTO special_objects (id, x, y,radius,color) VALUES (?, ?, ?,?,?)", (id, x, y,radius,color))
                        self.db_connection.commit()
                        spec_obj = Special_Object(id,x, y, radius, color, 0)
                        
                        
                        self.spec_objects.append(spec_obj)
                        
                        self.all_objects.append(spec_obj)
                        
                        self.space.add(spec_obj.body,spec_obj.shape)
            

                data = {
                        "obstacles" : [],
                        "agents" : [],
                        "special_objects" : []

                    }
                self.db_cursor.execute(f"SELECT * FROM obstacles")
                rows = self.db_cursor.fetchall()

                # Loop through the rows and print the data
                for row in rows:
                    self.get_logger().info(str(row))
                    id = row[0]
                    
                    x = row[1]
                    y = row[2]
                    width = row[3]
                    height = row[4]

                    data["obstacles"].append({
                        'id': id,
                        'x': x,
                        'y': y,
                        'width': width,
                        'height': height,
                        'color' : 3
                    

                    })

                self.db_cursor.execute(f"SELECT * FROM agents")
                rows2 = self.db_cursor.fetchall()

                # Loop through the rows and print the data
                for row in rows2:
                    self.get_logger().info(str(row))

                    id = row[0]
                    name = row[1]
                    x = row[2]
                    y = row[3]
                    orientation = row[4]
                    fbdl = row[5]
                    state = row[6]
                    data["agents"].append({
                        'id': id,
                        'name': name,
                        'x': x,
                        'y': y,
                        'orientation' : orientation,
                        'fbdl' : fbdl,
                        'state' : state
                    })

                self.db_cursor.execute(f"SELECT * FROM special_objects")
                specials = self.db_cursor.fetchall()

                # Loop through the rows and print the data
                for row in specials:
                    self.get_logger().info(str(row))
                    id = row[0]
                    x = row[1]
                    y = row[2]
                    radius = row[3]
                    color = str(row[4])


                    data["special_objects"].append({
                        'id': id,
                        'x': x,
                        'y': y,
                        'radius' : radius,
                        'color' : color
                    })


                print("Shapes in the space:")
                for shape in self.space.shapes:
                
                    self.get_logger().info(f" {shape} ")
                json_string = json.dumps(data)

                response.result = json_string
            
        else:
                self.get_logger().info(f" HIBA")
            
        return response
    
    def spec_obj_callback(self,msg):
        id = msg.id
        x = msg.position_x
        y = msg.position_y
        #radius = msg.radius
        #color = msg.color
        
        for existing in self.spec_objects:
            if(existing.id == id and self.timer_paused):
                existing.body.position = x,y
                existing.x = existing.body.position.x
                existing.y = existing.body.position.y
                         
    def state_callback(self,msg):
       
                id = msg.id
                x = msg.position_x
                y = msg.position_y
                width = msg.width
                height = msg.height


                for existing in self.boxes:
                    if(existing.id == id):
                        existing.body.position = x+width/2,y+height/2
                        existing.x = x
                        existing.y = y
                                    
        
                    #self.get_logger().info(str(existing_box.body.angle))
            
    # def state_publish(self):
    #     msg_back = String()
    #     data = {
    #         "obstacles" : [],
    #         "special_objects" : []
    #     }
       
    #     for i in  self.boxes:
    #         data["obstacles"].append({
    #             'id': i.id,
    #             'x': i.body.position.x,
    #             'y': i.body.position.y,
    #             'angle' : i.body.angle
    #         })
    #     box_ids = [box.id for box in self.boxes]
    #     for i in data["obstacles"][:]:  # Iterate over a copy of the list
    #         if i['id'] not in box_ids:
    #             data["obstacles"].remove(i)


    #     for i in  self.spec_objects:
    #         data["special_objects"].append({
    #             'id': i.id,
    #             'x': i.x,
    #             'y': i.y,
    #             'angle' : i.body.angle
    #         })
    #     special_ids = [special.id for special in self.spec_objects]
    #     for i in data["special_objects"][:]:  # Iterate over a copy of the list
    #         if i['id'] not in special_ids:
    #             data["special_objects"].remove(i)

    #     json_string = json.dumps(data)
    #     #self.get_logger().info(str(len(self.boxes)))

    #     msg_back.data = json_string
    #     #if(self.pause_simulation == 0):
            
    #     self.obstacle_pos_pub.publish(msg_back)
            
    def special_object_position_publish(self):
        spec_msg = SpecObject()
        
        for existing in self.spec_objects: 
            if(existing.body in self.space.bodies and not self.timer_paused):
                existing.x = existing.body.position.x
                existing.y = existing.body.position.y
                existing.body.position = existing.x,existing.y
                spec_msg.id = existing.id
            
                spec_msg.position_x = existing.x
                spec_msg.position_y = existing.y

                self.spec_object_pos_pub.publish(spec_msg)
          
    def calculate_distance(self,agent1, agent2):
        #buggy if I use agent1.x and agent2.x and y . body.position proved to be more stable.
        distance = math.sqrt((agent1.body.position.x - agent2.body.position.x)**2 + (agent1.body.position.y - agent2.body.position.y)**2)
        return distance   
  
  
    def on_collision(self,arbiter, space, data):
    # This function is called when a collision occurs
        obj1, obj2 = arbiter.shapes
       
        agent1 = self.body_to_agent[obj1.body]
        agent2 = self.body_to_agent[obj2.body]
        
        msg = Int16()      
        msg.data = 1

        msg2 = Int16()
        msg2.data = 1
        self.stop_publisher[agent1.name].publish(msg)
        self.stop_publisher[agent2.name].publish(msg2)
            
        return True  # Return True to allow collision response, or False to disable it
    
    def on_separation(self,arbiter, space, data):
        obj1, obj2 = arbiter.shapes
       
        agent1 = self.body_to_agent[obj1.body]
        agent2 = self.body_to_agent[obj2.body]

        msg = Int16()      
        msg.data = 0

        msg2 = Int16()
        msg2.data = 0
        self.stop_publisher[agent1.name].publish(msg)
        self.stop_publisher[agent2.name].publish(msg2)
        
       # self.get_logger().info(f"Collision between {agent_shape.name} and boundary")
        return True

    def agent_to_static_collision(self,arbiter, space, data):
        agent_shape, static_shape = arbiter.shapes
        agent_shape = self.body_to_agent[agent_shape.body]

        # Apply the force at the calculated local point
        # force = (-100, -100)  # Adjust the force as needed
        # agent_shape.body.apply_force_at_local_point(force, local_collision_point)
        
        msg = Int16()
        agent_shape.object_collision = True
               
        msg.data = 1
        
        self.stop_publisher[agent_shape.name].publish(msg)
       
        self.get_logger().info(f"Collision between {agent_shape.name} and boundary")
        return True
    

    def agent_to_static_collision_separated(self,arbiter, space, data):
        agent_shape, static_shape = arbiter.shapes
        agent_shape = self.body_to_agent[agent_shape.body]

        # Apply the force at the calculated local point
        # force = (-100, -100)  # Adjust the force as needed
        # agent_shape.body.apply_force_at_local_point(force, local_collision_point)
        
        msg = Int16()
        agent_shape.object_collision = True
               
        msg.data = 0
        self.stop_publisher[agent_shape.name].publish(msg)
        
       # self.get_logger().info(f"Collision between {agent_shape.name} and boundary")
        return True
    
    def agent_to_special_on(self,arbiter, space, data):
        agent_shape, static_shape = arbiter.shapes
        agent_shape = self.body_to_agent[agent_shape.body]

        # Apply the force at the calculated local point
        # force = (-100, -100)  # Adjust the force as needed
        # agent_shape.body.apply_force_at_local_point(force, local_collision_point)
        
        msg = Int16()
        agent_shape.object_collision = True
               
        msg.data = 1
        
        self.stop_publisher[agent_shape.name].publish(msg)
       
        self.get_logger().info(f"Collision between {agent_shape.name} and boundary")
        return True
    

    def agent_to_special_separated(self,arbiter, space, data):
        agent_shape, static_shape = arbiter.shapes
        agent_shape = self.body_to_agent[agent_shape.body]

        msg = Int16()
        agent_shape.object_collision = True
               
        msg.data = 0
        self.stop_publisher[agent_shape.name].publish(msg)
        
       # self.get_logger().info(f"Collision between {agent_shape.name} and boundary")
        return True

    def agent_recharger_collision(self,arbiter, space, data):
        agent_shape, recharger = arbiter.shapes
        agent_shape = self.body_to_agent[agent_shape.body]

        print(f" {agent_shape.id} is charging")

        agent_shape.charging = True
        charging_msg = Int16()
        charging_msg.data = 1

        self.charging_agent[agent_shape.name].publish(charging_msg)

        print(agent_shape.battery_level)
        return True
    
    def collide(self):
          #agent collision handler  . Agent collision type is 1 so checking collision between 1-1
        handler = self.space.add_collision_handler(1, 1)
        handler.begin = self.on_collision  # Set the collision callback function

        sepatare_agent = self.space.add_collision_handler(1, 1)
        sepatare_agent.separate = self.on_separation  # Set the collision callback function

        # # #agent - boundary collision handler Boundary collision type is 2 so checking collision between 1 -2
        agent_to_static_handler = self.space.add_collision_handler(1, 2)
        agent_to_static_handler.pre_solve = self.agent_to_static_collision

        separte = self.space.add_collision_handler(1, 2)
        separte.separate = self.agent_to_static_collision_separated

        special = self.space.add_collision_handler(1, 0)
        special.begin = self.agent_to_special_on  # Set the collision callback function

        special_separate = self.space.add_collision_handler(1, 0)
        special_separate.separate = self.agent_to_special_separated  # Set the collision callback function

        charging_handler = self.space.add_collision_handler(1, 3)
        charging_handler.begin = self.agent_recharger_collision  # Set the collision callback function
        

    def ray_detector(self):
        msg = Rays()
        self.ranges = []
        for agent in self.agents:
            agent.sensor.is_intersect(agent.orientation,self.space)
            self.ranges = agent.sensor.is_intersect(agent.orientation,self.space) 
            msg.ranges = self.ranges
            self.agent_rays[agent.name].publish(msg)
        
    

   
    def count_publish(self):
        msg = Int16()
        msg.data = count
        self.count_pub.publish(msg)

    def name_callback(self, msg):
        
        self.msg.names.append(msg.data)
        #print("aaaaa",msg.data)

    def robot_name_msg(self):
         
        self.name_pub.publish(self.msg)


    # def list_space(self):
    #     for shape in self.space.shapes:
                
    #             self.get_logger().info(f" aaaaaa {shape} ")
    #     self.get_logger().info(f" aaaaaa")          
    def simulate(self):
        
        if not self.timer_paused :
            
            self.ray_detector()
            self.collide()
            #self.detect_collisions()
            self.space.step(1 / 10.0)
            

def main(args=None):
    rclpy.init(args=args)
    
    executor = MultiThreadedExecutor()
    spawner = Controller("controller", executor)
    executor.add_node(spawner)
   # thread = threading.Thread(target=spawner.test)
    #thread.start()
    spawner.get_logger().info("spawner node has started")  
    executor.spin()

   # thread.join()
    executor.shutdown()

    spawner.close_connection()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
    