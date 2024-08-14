class Special{

    constructor(id, x, y,radius,color) {
        this.id = id
        this.x = x;
        this.y = y;
        this.pos_x = 0.0
        this.pos_y = 0.0
        this.radius = radius;
        this.weight = 0.0;
        this.color = color;
      
    }
    setWeight(weight){
        this.weight = weight
    }

    show(){
        strokeWeight(2);
        
        this.x = lerp(this.x, this.pos_x, 0.1);
        this.y = lerp(this.y, this.pos_y, 0.1); 
        switch(this.color){
            case "1" :  fill(255, 0, 0); break;
            case "2":  fill(0, 255, 0); break;
            case "3" :  fill(0, 0, 255); break;
           
        }
        push(); // Save the current drawing state
        ellipse(this.x, this.y, this.radius, this.radius);
        pop(); // Restore the drawing state
        
        textSize(30);
        fill(0, 0, 0);
        textAlign(CENTER, CENTER); // Center the text
        text(this.id, this.x, this.y);
      
        
        
    }

}