class Agent{
  constructor() {
      this.id = "";
      this.name = "";
      this.x = 0;
      this.y =  0;
      this.radius = 70;
      this.velocity = 0;
      this.topic = "";
      this.velx = 0;
      this.angz = 0;
      this.pos_x = 0;
      this.pos_y = 0;
      this.state = ""
      this.collided = false;
      this.battery_level = 100;
      this.nearest_distance = 0;
      this.fuzzy_rules ="";
      this.color = ""
      this.num_rays = 7;
      this.ray_length = 100
      this.target = null;
      this.checkpoint = 0;
      this.statistics = "";
      this.statistics_data = "";
     
      //this.topic_name = "agent"+count+"/cmd_vel";
     
      this.frontAngle = 0;
      this.angle = 0;
      
      // Create a listener for /my_topic
    }

    setFrontAngle(angle) {
      this.frontAngle = angle;
    }
  
    setName(name){
      return this.name = name;
    }

    spawn(){
        this.x = this.pos_x;
        this.y =  this.pos_y;
    }
  
    move() {

      this.velocity = this.velx; 
      
      this.x = lerp(this.x, this.pos_x, 0.1);
      this.y = lerp(this.y, this.pos_y, 0.1); 
      this.frontAngle = lerp(this.frontAngle, this.angle, 0.1);
      
      }
  

    get_distance_from_target(){

      let d = dist(this.x,this.y, spawn_point.x, spawn_point.y);
      return d;
    }

    toRadians (angle) {
      return angle * (Math.PI / 180);
    }

    show() {
      stroke(255);
      if(this.state == -1){
           this.color = "1";
      }else{
        this.color = "2";
      }

      switch(this.color){
        case "1" :  fill(255, 0, 0); break;
        case "2":  fill(0, 255, 0); break;
        case "3" :  fill(0, 0, 255); break;
       
    }

    let offset_angle = 70
    let angle_step = this.toRadians(offset_angle) / (this.num_rays - 1);
   
    for(let i = 0 ; i< this.num_rays; i++){

      let angle_offset = -this.toRadians(offset_angle/2) + i * angle_step 
      let direction = createVector(cos(this.frontAngle + angle_offset), sin(this.frontAngle + angle_offset));
      let vector = createVector(this.x + direction.x * this.ray_length, this.y + direction.y *  this.ray_length);
      
      line(this.x, this.y, vector.x, vector.y);

    }
       

      push(); // Save the current drawing state
      
      strokeWeight(1);
      translate(this.x, this.y); // Set the origin to the center of the circle
      rotate(this.frontAngle); // Apply the rotation
      
      ellipse(0, 0, this.radius, this.radius);
      textSize(15);
      fill(255, 255, 255);
      text(this.name, 0, 0);

      let frontX = this.radius / 2;
      let frontY = 0;

      fill(255, 0, 0);
      triangle(frontX - 5, frontY + 10, frontX + 15, frontY, frontX - 5, frontY - 10);

      pop(); // Restore the previous drawing state
    }

    new_target(mouse_x,mouse_y){
        this.target = (mouse_x,mouse_y);
    }
  
    collide(){
      if(this.x +this.radius > canvasW|| this.y +this.radius > canvasH || this.x +this.radius< 0 || this.y+this.radius <0){
        //console.log("collision detected");
        this.x = Math.random()* (200 - 50) + 50;
        this.y =  Math.random()* (600 - 50) + 50;
      }
    }
  
    stop(){
      this.velocity = 0;
    }

    unsubscribe_from_topic(topic){
      topic.unsubscribe();
    }
  }