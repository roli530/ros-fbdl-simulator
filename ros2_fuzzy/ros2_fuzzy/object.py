import pymunk

class Special_Object:
    def __init__(self,id,x=0,y=0,radius=0,color="", collision_type=0 ):
        self.id = id
        self.x = x
        self.y = y
     
        self.radius = radius
        self.color = color


        self.body_mass = 5.0
        self.body_moment = pymunk.moment_for_circle(self.body_mass, 0, 30)
        self.body = pymunk.Body(self.body_mass, self.body_moment)
        self.body.position = self.x, self.y
        
        self.shape = pymunk.Circle(self.body,30) 
        self.shape.friction = 10.0
        
        self.collision_type = collision_type
     
        self.shape.collision_type = self.collision_type
        
      