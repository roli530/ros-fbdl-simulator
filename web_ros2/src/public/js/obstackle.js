class Obstackle{

    constructor(id,x, y,width,height,color) {
        this.id = id
        this.x = x;
        this.y = y;
     
        this.width = width;
        this.height = height;
        this.color = color
        this.angle = 0
    }

    toJSON() {
        return {
          id : this.id,
          x: this.x,
          y: this.y,
      
          height: this.height,
          width: this.width
        };
      }

    show(){

      switch(this.color){
        case "1" :  fill(255, 0, 0); break;
        case "2":  fill(0, 255, 0); break;
        case "3" :  fill(0, 0, 255); break;
       
    }
        push(); 
        
        rect(this.x, this.y, this.width, this.height); 
        rotate(this.angle);
        pop();
    }

}