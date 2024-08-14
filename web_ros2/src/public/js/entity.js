class Entity{
    constructor() {
        this.id = 0;
        this.name = "";
        this.x = 0;
        this.y =  0;
        this.radius = 50;
      }
    
      
      show() {
        fill(0,0,255);
        strokeWeight(2);
        ellipse(this.x, this.y, this.radius, this.radius);
        
      }
}