import pymunk

class Box:
    def __init__(self,id,x,y,width,height, collision_type ):
        self.id = id
        self.x = x
        self.y = y
        
        self.width = width
        self.height = height
        self.color = ""
        
        self.body_mass = 100000000
        self.body_moment = pymunk.moment_for_box(self.body_mass,  (self.width,self.height))
        self.body = pymunk.Body(self.body_mass, self.body_moment)
        self.body.position = self.x + self.width/2 , self.y + self.height/2
        self.shape = pymunk.Poly.create_box(self.body, (self.width, self.height))
        self.shape.friction = 1.0
        self.rotation = self.body.angle
        
        
        self.collision_type = collision_type
     
        self.shape.collision_type = self.collision_type
        