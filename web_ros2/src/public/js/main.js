
 //html dom
let states = document.getElementById("state");
let messages = document.getElementById("messages");

let agent_velocity = document.getElementById("agent_vel");
let input_name = document.getElementById("nameInput");
let statistics = document.getElementById("statistics");

let agent_state = document.getElementById("agent_state");
let battery = document.getElementById("battery");
let file_list = document.getElementById("file_list");
let behaviour_input = document.getElementById("behaviour_input");
let obstacle_width = document.getElementById("obstacle_width");
let obstacle_height = document.getElementById("obstacle_height");
let display_name = document.getElementById("agent_name");
let color_selector = document.getElementById("colors");
let universes = document.getElementById("universes");
let rulebases = document.getElementById("rulebases");
let weight = document.getElementById("special_object_weight");
//needed variables and constatnts

let agent;
let cmd_vel_topic;
let position_topic;
let topic;
let canvasH ;
let canvasW;
let lin_x  = 0;
let ang_z = 0;
let selected_agent = null;
let selected_obstacle = null;
let selected_special_object = null;
let selected_object_type = ""
let agent_number = 0;
let rob_name;
let lastTime = 0;
let json_agent_number = 0;
let move_cube = false;
let move_spec_obj = false;
let resize = false;
let pause = false;
let add_spec_obj = false;
let current_selected = null
let s = 0
let obstacles = []
let special_objects = []
let universe_list = []
let rulebase_list = []

const entities = [];
const entity_states = ["happy", "angry"];      //kutya állapotai teszt.

const canvas_w = screen.width/1.4
const canvas_h = screen.height/1.7



let simulator;
let codeArea = document.getElementById("codearea");
CodeMirror.defineSimpleMode("fbdl", {
  start: [
      {regex: /(?:universe|rulebase|end|if|rule|when|and|is|this)\b/, token: "keyword"},
      {regex: /\/\/.*/, token: "comment"},
      {regex: /"(?:[^\\]|\\.)*?(?:"|$)/, token: "string"},
      {regex: /0x[a-f\d]+|[-+]?(?:\.\d+|\d+\.?\d*)(?:e[-+]?\d+)?/i, token: "number"},
      {regex: /[-+/*=<>!]+/, token: "operator"},
      {regex: /[a-z$][\w$]*/, token: "variable"},
   
  ],
  meta: {
      lineComment: "//"
  }
});
var editor = CodeMirror.fromTextArea(codeArea, {
  mode: "fbdl", // Set the language mode
  lineNumbers: true, // Display line numbers
  theme: "bespin" // Choose a theme (you can use different themes)
});


editor.setSize(canvas_w, null);
let editor_height = editor.getWrapperElement().offsetHeight


let current_val = codeArea.value;

let state = {
  obstackles: [],
  agents: [],
  special_objects : []
};   


let count = 0;

let topicname;


$("#simulate").click(function(){ 
   
    
    console.log("A new agent has been created!  ");

    let request = new ROSLIB.ServiceRequest({
      agent_name:   input_name.value
    });
    
    // Call the service
    newAgent_client.callService(request, function(result) {
      if (result.result) {
        
        let agent_data = JSON.parse(result.result);
        agent = new Agent();
        agent.id = agent_data.id;
        agent.name = agent_data.name
        agent.color = color_selector.value;
        agent.x = agent_data.x;
        agent.y = agent_data.y;
        agent.pos_x = agent.x;
        agent.pos_y = agent.y;
        agent.state = agent_data.state;
        console.log(spawn_point.x +"  "+ spawn_point.y);
        entities.push(agent); 
        
        
        //cmd_vel_topic = agent.name+"/cmd_vel";
        position_topic = agent.name+"/position";
        state_topic = agent.name+"/state";
        battery_topic = agent.name+"/battery"
        statistics_topic = agent.name+"/statistics"
        error_topic = agent.name+"/error"
    // }
      
        console.log(agent.id)
        //subscribe_to_topic(topic_sub(cmd_vel_topic,"geometry_msgs/Twist"));
        subscribe_to_topic(topic_sub(position_topic,"custom_msgs/Position"));
        subscribe_to_topic(topic_sub(state_topic,"std_msgs/Int16"));
        subscribe_to_topic(topic_sub(battery_topic,"std_msgs/Float32"));
        subscribe_to_topic(topic_sub(statistics_topic,"std_msgs/String"));
       

        console.log('Agent successfully added: ' + agent.name);
      } else {
        console.error('Failed to add agent');
      }
});
 //subscribe_to_topic(topic_sub("/obstacle_pos","std_msgs/String")); 
})

$("#stop").on("click",(function(){  //startra kattintva új agent példány jön létre.
  pause = true;

  let request = new ROSLIB.ServiceRequest({
   request: 1
  });

  // Call the service
  stop_service.callService(request, function(result) {
    if (result.response) {
      console.log("ok");
    } else {
      console.error('Failed to delete agent');
    }
  });

  fetch('http://localhost:3000/files') 
  .then((response) => {
    if (!response.ok) {
      throw new Error('Network response was not ok');
    }
    console.log('Data from the server:', response);
    return response.json();
  })
  .then((data) => {
    $('#file_list').empty();
    //console.log('Data from the server:', data);
    for (let i = 0; i < data.someValue.length; i++) {
      $('#file_list').append('<option value="' + data.someValue[i] + '">' + data.someValue[i] + '</option>');

    }
  })
  .catch((error) => {
    console.error('Fetch error:', error);
  });
        
}));

$("#continue").on("click",(function(){ 
  pause = false
  let request = new ROSLIB.ServiceRequest({
    request: 0
   });
 
   // Call the service
   stop_service.callService(request, function(result) {
     if (result.response) {
       console.log("ok");
     } else {
       console.error('Failed to delete agent');
     }
   });      
}));

$("#change_weight").on("click",(function(){ 
  selected_special_object.setWeight(parseFloat(weight.value));

  let request = new ROSLIB.ServiceRequest({
    weight : selected_special_object.weight,
    id : selected_special_object.id
   });

  weight_service.callService(request, function(result) {
    if (result.response) {
      console.log("ok");
    } else {
      console.error('Failed to change weight');
    }
  });    

  console.log(selected_special_object.weight);
}));

$("#compile").on("click",(function(){ 

  current_val = editor.getValue();
 
  let request = new ROSLIB.ServiceRequest({
    agent_name: selected_agent.name,  
    fbdl : current_val
  });

  selected_agent.fuzzy_rules = current_val
  
  // Call the service
  fuzzy_client.callService(request, function(result) {
    if (selected_agent) {
      console.log(current_val);
    } else {
      console.error('Failed to delete agent');
    }
  });
  // let behavior_fbdl = new ROSLIB.Message({
  //   name: behaviour_input.value,
  //   fbdl: current_val
  // });
  // let topic = "/fbdl";
  // publish_to_topic(topic,"custom_msgs/Fbdl").publish(behavior_fbdl);
  //const simulatorData = JSON.stringify(simulator);
  
}));

$("#special_object").on("click",(function(){ 
  add_spec_obj = true;
  console.log("new special object added")
 
}));

let clicked = false;
$("#new_obstackle").on("click",(function(){ 
  clicked = true;
  console.log("new obstackle added");
  $("#stop_adding").show();
}));

$("#stop_adding").hide();
$("#stop_adding").on("click", function() { 
  clicked = false;
  console.log("Stopped adding obstacles");
  $(this).hide(); 
});

$("#change_dimension").on("click", function() { 
  selected_obstacle.width = parseFloat(obstacle_width.value)
  selected_obstacle.height = parseFloat(obstacle_height.value)
  console.log("new dimension: "+  obstacle_width.value + obstacle_height.value);

  change_dimensions_service();
  
});



$("#display_universes").on("click", function() { 
     display_universes_service();

  
});

$("#change_color").on("click", function() { 
  if(current_selected != null){
    current_selected.color = color_selector.value;
  }
  else{
    console.log("nothing is selected yet")
  }if(current_selected == selected_special_object){
   

    let request = new ROSLIB.ServiceRequest({
      id: selected_special_object.id,
      color : color_selector.value
    });
    
    // Call the service
    change_color_client.callService(request, function(result) {
      if (result.result) {
        console.log('Special object color changed: ' + selected_special_object.id);
      } else {
        console.error('Failed to change color');
      }
    });
  }

});
let new_target = false
$("#load_fbdl").on("click",(function(){ 
  editor.setValue(selected_agent.fuzzy_rules);

}));

$("#fbdl_template").on("click",(function(){ 
 
  let fbdl_str = "";
  const distance_substr = "_distance";
  const orientation_error_substr = "_orientation_error";
  const speed_substr = "_speed";
  const state_substr = "_state";
  const reached_objective ="_reached_objective";
  const collision_substr = "_collision";
  const ray_collision_substr = "_ray_collision";
  const obj_color_subst = "_color";
  //const obj_angle_subst = "_angle"
 
  if(universe_list.length == 0){
    alert("The universe list is empty!. Click on the Display universes button and try again!")
  }else{

    for(let i = 0; i< universe_list.length;i++){
      
      if(universe_list[i].includes(distance_substr)){
        fbdl_str += 'universe "' + universe_list[i] + '"\n' 
                        + '"close" ' +" " + 0 + " " + 0 + "\n"
                        
                        +'"far"' +" " +1000 + " " + 1 +"\n"
                        +"end" +"\n"
                        +"\n"
      }else if(universe_list[i].includes(orientation_error_substr)){
        fbdl_str += 'universe "' + universe_list[i] + '"\n' 
                        + '"max_left" ' +" " + -10 + " " + -1 + "\n"
                       
                        +'"max_right"' +" " +10 + " " + 1 +"\n"
                        +"end" +"\n"
                        +"\n"
      }else if(universe_list[i].includes(speed_substr)){
        fbdl_str += 'universe "' + universe_list[i] + '"\n' 
                        + '"reverse" ' +" " + -1 + " " + -1 + "\n"
                      
                        +'"high"' + " "+ 1 + " " + 1 +"\n"
                       
                        +"end" +"\n"
                        +"\n"

      }
      else if(universe_list[i].includes(state_substr)){
        fbdl_str += 'universe "' + universe_list[i] + '"\n' 
                        + '"angry" ' +" " + -1 + " " + -1 + "\n"
                      
                        +'"happy"' + " "+ 1 + " " + 1 +"\n"
                       
                        +"end" +"\n"
                        +"\n"
      }
     else if(universe_list[i].includes(reached_objective)){
        fbdl_str += 'universe "' + universe_list[i] + '"\n' 
                        + '"no_objective" ' +" " + -1 + " " + -1 + "\n"
                      
                        +'"special_object_number"' + " "+ 1 + " " + 1 +"\n"
                       
                        +"end" +"\n"
                        +"\n"
      

      }else if(universe_list[i].includes(ray_collision_substr)){
        fbdl_str += 'universe "' + universe_list[i] + '"\n' 
                        + '"close" ' +" " + 0 + " " + -1 + "\n"
                      
                        +'"far"' + " "+ 1000 + " " + 1 +"\n"
                       
                        +"end" +"\n"
                        +"\n"
      }
      else if(universe_list[i].includes(collision_substr)){
        fbdl_str += 'universe "' + universe_list[i] + '"\n' 
                        + '"no_collision" ' +" " + 0 + " " + 0 + "\n"
                      
                        +'"collision"' + " "+ 1 + " " + 1 +"\n"
                       
                        +"end" +"\n"
                        +"\n"

      }else if(universe_list[i].includes(obj_color_subst)){
        fbdl_str += 'universe "' + universe_list[i] + '"\n' 
                        + '"red" ' +" " + -1 + " " + -1 + "\n"
                      
                        +'"blue"' + " "+ 1 + " " + 1 +"\n"
                       
                        +"end" +"\n"
                        +"\n"
      }
   
    }
    /*for(let i = 0; i< rulebase_list.length;i++){ 
       if(rulebase_list[i].includes(rulebase_speed_str)){
        fbdl_str += 'rulebase "' + rulebase_list[i] + '"\n' 
                        +"rule"+ "\n"
                        +'"low" ' + "when " + '"agent1_distance" '+ "is" + '"close"'+ "\n"
                        +"end"+ "\n"
                        +"rule"+ "\n"
                        +'"middle" ' + "when " + '"agent1_distance" '+ "is" + '"middle"'+ "\n"
                        +"end"+ "\n"
                        +"rule"+ "\n"
                        +'"high" ' + "when " + '"agent1_distance" '+ "is" + '"far"'+ "\n"
                        +"end"+ "\n"
                        +"end"+ "\n"
                        +"\n"
         
        } else if(rulebase_list[i].includes(rulebase_angle_str)){
          fbdl_str += 'rulebase "' + rulebase_list[i] + '"\n' 
                          +"rule"+ "\n"
                          +'"left" ' + "when " + '"agent1_orientation_error" '+ "is" + '"left"'+ "\n"
                          +"end"+ "\n"
                          +"rule"+ "\n"
                          +'"middle" ' + "when " + '"agent1_orientation_error" '+ "is" + '"middle"'+ "\n"
                          +"end"+ "\n"
                          +"rule"+ "\n"
                          +'"right" ' + "when " + '"agent1_orientation_error" '+ "is" + '"right"'+ "\n"
                          +"end"+ "\n"
                          +"rule"+ "\n"
                          +'"stop" ' + "when " + '"agent1_orientation_error" '+ "is" + '"stop"'+ "\n"
                          +"end"+ "\n"
                          +"end"+ "\n"
                          +"\n"
           }

           if(rulebase_list[i].includes(rulebase_agent_state_str)){
            fbdl_str += 'rulebase "' + rulebase_list[i] + '"\n' 
                            +"rule"+ "\n"
                            +'"angry" ' + "when " + '"agent1_distance" '+ "is" + '"close"'+ "\n"
                            +"end"+ "\n"
                            +"rule"+ "\n"
                            +'"happy" ' + "when " + '"agent1_distance" '+ "is" + '"far"'+ "\n"
                            +"end"+ "\n"

                            +"end"+ "\n"
                            +"\n"
            
            } 

     }
 */
     fbdl_str += "universe " + '"angle"' + "\n" 
     + '"max_left" ' +" " + -10 + " " + -1 + "\n"
                       
     +'"max_right"' +" " +10 + " " + 1 +"\n"
     +"end" +"\n"               
     +"\n"

     fbdl_str += "universe " + '"agent_status"' + "\n" 
     + '"angry" ' +" " + -1 + " " + -1 + "\n"
                       
     +'"happy"' +" " +1 + " " + 1 +"\n"
     +"end" +"\n"               
     +"\n"

     fbdl_str += "universe " + '"speed"' + "\n" 
     + '"reverse" ' +" " + -1 + " " + -1 + "\n"
                       
     +'"high"' +" " +1 + " " + 1 +"\n"
     +"end" +"\n"               
     +"\n"
     
     editor.setValue(fbdl_str);
    
  }


}));
function error_handler(){
  error_service.advertise(function(request, response) {
    console.log('Received service request on ' + request.error);
    alert(request.error)
    response['response'] = 'Set successfully';
    return true;
  });
}


function save_state(){
   // Loop through obstackles and push them into the obstackles array

  for(let i= 0; i< state.obstackles.length; i++){
    console.log("aaaa" + state.obstackles[i].id);
  }

   for (let i = 0; i < obstacles.length; i++) {
      if (i < state.obstackles.length) {
        state.obstackles[i].id = obstacles[i].id;
        state.obstackles[i].x = obstacles[i].x / canvas_w;
        state.obstackles[i].y = obstacles[i].y / canvas_h;
        state.obstackles[i].width = obstacles[i].width;
        state.obstackles[i].height = obstacles[i].height;
        state.obstackles[i].color = obstacles[i].color;
      
    }else{
      state.obstackles.push({
        id: obstacles[i].id,
        x : obstacles[i].x ,
        y : obstacles[i].y ,
        width : obstacles[i].width,
        height: obstacles[i].height,
        color : obstacles[i].color
      })
    }   
   }

  for (let i = 0; i< entities.length; i++){
    if (i < state.agents.length) {
        // If there is an existing object at this index, update its properties
        state.agents[i].x = entities[i].x ;
        state.agents[i].y = entities[i].y ;
        state.agents[i].orientation = entities[i].frontAngle;
        state.agents[i].name = entities[i].name;
        state.agents[i].id = entities[i].id;
        state.agents[i].fuzzy = entities[i].fuzzy_rules;
        state.agents[i].state = entities[i].state;

    } else {
      // If there is no existing object at this index, push a new object
        state.agents.push({
          x : entities[i].x  / canvas_w,
          y : entities[i].y  / canvas_h,
          orientation : entities[i].frontAngle,
          id : entities[i].id,
          name: entities[i].name,
          fuzzy : entities[i].fuzzy_rules,
          state: entities[i].state
          
   })
  } 
  }

  for (let i = 0; i< special_objects.length; i++){
   
        state.special_objects[i].x = special_objects[i].x / canvas_w;
        state.special_objects[i].y = special_objects[i].y / canvas_h;
        state.special_objects[i].color = special_objects[i].color;
       
    } 
  } 


let json_obj = null
let boxes_count = 0
let special_object_count = 0


$("#save_state").on("click", function() { 
  // Use saveJSON() to save the data as a JSON file

  save_state();
  // Save the data object as JSON
  saveJSON(state, 'object_positions.json');
  
});

function remove_everything(){
  state.special_objects.splice(0);
  state.obstackles.splice(0);
  state.agents.splice(0);
  obstacles.splice(0);
  entities.splice(0);
  special_objects.splice(0);
}
$("#load_state").on("click", function() { 
  // Use saveJSON() to save the data as a JSON file
 

  let selected_file = file_list.options[file_list.selectedIndex].text


  // $.ajax({
  //   url: '/load_state',
  //   method: 'GET',
    
  //   contentType: 'application/json',
  //   success: function(result) {
  //     console.log('state loaded:', result);
  //     // Handle the response from the server
  //   },
  //   error: function(error) {
  //     console.error('Error:', error);
  //   },
  // });

  let request = new ROSLIB.ServiceRequest({
    filename: selected_file  
  });

  // Call the service
  load_client.callService(request, function(response) {
    if (response.result) {
      remove_everything();

      console.log('state loaded succesfully' + response.result);
      parsed_result = JSON.parse(response.result)
      
      for(let i= 0; i< parsed_result.obstacles.length;i++){
        let obj = parsed_result.obstacles[i]
        let obstacle = new Obstackle(obj.id,obj.x,obj.y,obj.width,obj.height,obj.color);
        obstacle.angle = obj.angle;
        obstacles.push(obstacle);
        state.obstackles.push(obstacle);

      }
      for (let i = 0; i< parsed_result.agents.length; i++) {
            let agent_data = parsed_result.agents[i]
            let agent = new Agent(agent_data.x, agent_data.y ,agent_data.fuzzy_rules);
            agent.id = agent_data.id;
            agent.name = agent_data.name;
            agent.pos_x = agent_data.x;
            agent.pos_y = agent_data.y;
            agent.frontAngle = agent_data.orientation;
            agent.angle = agent_data.orientation;
            agent.fuzzy_rules = agent_data.fbdl;
            agent.state = agent_data.state;
            entities.push(agent);
            //console.log(data.agents.length);

            //cmd_vel_topic = agent.name+"/cmd_vel";
            position_topic = agent.name+"/position"; 
            name_topic = "agent_names";  
            state_topic = agent.name+"/state"  ;
            battery_topic = agent.name+"/battery";
            statistics_topic = agent.name+"/statistics";
            //subscribe_to_topic(topic_sub(cmd_vel_topic,"geometry_msgs/Twist"));
            subscribe_to_topic(topic_sub(position_topic,"custom_msgs/Position"));
            subscribe_to_topic(topic_sub(name_topic,"custom_msgs/Names"));
            subscribe_to_topic(topic_sub(state_topic,"std_msgs/Int16"));
            subscribe_to_topic(topic_sub(battery_topic,"std_msgs/Float32"));
            subscribe_to_topic(topic_sub(statistics_topic,"std_msgs/String"));
          
            console.log("maaaaaaaaa"+entities.length);
        }
        for(let i= 0; i< parsed_result.special_objects.length;i++){
          let special = parsed_result.special_objects[i]
          let special_object = new Special(special.id,special.x,special.y,special.radius,special.color);
          special_object.pos_x = special.x;
          special_object.pos_y = special.y;
          special_objects.push(special_object);
          state.special_objects.push(special_object);
        }
   
    } else {
      console.error('Failed to delete obstacle');
    }

  });

});

$("#remove").on("click", function() { 
  
  switch(selected_object_type){
    case "agent" : delete_agent_service(); break;
    case "obstacle" : delete_obstacle_service(); break;
    case "special_object" : delete_special_object(); break;
  }
  
})


function delete_special_object(){
  for(let i = 0; i < special_objects.length; i++){
    if(selected_special_object == special_objects[i]){
      special_objects.splice(i,1)
      state.special_objects.splice(i,1)
    }
  }

  let request = new ROSLIB.ServiceRequest({
    obstacle_id: selected_special_object.id  // Replace with the agent name to delete
  });
  
  // Call the service
  delete_special_object_client.callService(request, function(result) {
    if (result.result) {
      console.log('Special object successfully deleted: ' + selected_special_object.id);
    } else {
      console.error('Failed to delete obstacle');
    }

  });
}

function delete_agent_service(){
  let request = new ROSLIB.ServiceRequest({
    agent_name: selected_agent.name  // Replace with the agent name to delete
  });
  
  // Call the service
  serviceClient.callService(request, function(result) {
    if (result.result) {
      console.log('Agent successfully deleted: ' + selected_agent.name);
    } else {
      console.error('Failed to delete agent');
    }
  });
  for(let i = 0; i < entities.length; i++){
    if(selected_agent == entities[i]){
      entities.splice(i,1)
      state.entities.splice(i,1)
      //cmd_vel_topic = selected_agent.name+"/cmd_vel";
      position_topic = selected_agent.name+"/position"; 
      
      state_topic = selected_agent.name+"/state"  ;
      battery_topic = selected_agent.name+"/battery";
      statistics_topic = selected_agent.name+"/statistics";
      //unsubscribe_from_topic(cmd_vel_topic);
      unsubscribe_from_topic(position_topic);
      unsubscribe_from_topic(state_topic);
      unsubscribe_from_topic(battery_topic);
      unsubscribe_from_topic(distance_topic);
     
    }
  }
  
  console.log("agentek száma : "+ count)
}

function delete_obstacle_service(){
  
  for(let i= 0; i< obstacles.length; i++){
    if(obstacles[i] == current_selected)
      obstacles.splice(i, 1);
      state.obstackles.splice(i,1);
     
  }
  
  console.log("new length" + obstacles.length);
  let request = new ROSLIB.ServiceRequest({
    obstacle_id: selected_obstacle.id  // Replace with the agent name to delete
  });
  
  // Call the service
  delete_obstacle_client.callService(request, function(result) {
    if (result.result) {
      console.log('Obstacle successfully deleted: ' + selected_obstacle.id);
    } else {
      console.error('Failed to delete obstacle');
    }

  });
}
function change_dimensions_service(){
  let request = new ROSLIB.ServiceRequest({
    id: selected_obstacle.id,
    new_width : selected_obstacle.width,
    new_height : selected_obstacle.height
  });
  
  // Call the service
  change_dimensions_client.callService(request, function(result) {
    if (result.result) {
      console.log('Special object successfully deleted: ' + selected_obstacle.id);
    } else {
      console.error('Failed to delete obstacle');
    }

  });
}
function contains(list, element) {
  for (var i = 0; i < list.length; i++) {
      if (list[i] === element) {
          return true;
      }
  }
  return false;
}

function display_universes_service(){
  let request = new ROSLIB.ServiceRequest({
    request: "I wish to see the universes and rulebases"
  });
  
  // Call the service
  universes_service.callService(request, function(result) {
      universes.innerHTML = "",
      rulebases.innerHTML = ""
      console.log('universes ' + result.universes);
      console.log('rulebases ' + result.rulebases);
    
      for (let i = 0; i < result.universes.length; i++) {
        universes.innerHTML += result.universes[i] + '<br>'; 
        if(!contains(universe_list,result.universes[i])){
          universe_list.push(result.universes[i])
        }
        
      }
          
      for (let i = 0; i < result.rulebases.length; i++) {
        rulebases.innerHTML += result.rulebases[i] + '<br>'; 
        if(!contains(rulebase_list,result.rulebases[i])){
          rulebase_list.push(result.rulebases[i])
        }
      }
      
  });
}


let spawn_point = new Entity()
let recharge_station = new Entity()

 
 let canvas_data = new ROSLIB.Message({
    canvas_width: canvas_w,
    canvas_height : canvas_h
  });

let target_for_collider;

// function mouseDragged() {
//   if (selectedRectangle && isResizing) {
//     // Change dimensions based on mouse position
//     let newWidth = mouseX - selectedRectangle.x;
//     let newHeight = mouseY - selectedRectangle.y;
//     selectedRectangle.resize(newWidth, newHeight);
//   }
// }
let f = 0

function create_obstacle_service(id,x,y,width,height,collision_type){
  let request = new ROSLIB.ServiceRequest({
    id: id,
    x : x,
    y : y,
    width : width,
    height : height,
    collision_type : collision_type
    
  });
  // Call the service
  create_obstacle_client.callService(request, function(result) {
    if (result.response) {
      console.log(result.response);
      let obstacle_data = JSON.parse(result.response);
      obstackle = new Obstackle(obstacle_data.id,obstacle_data.x,obstacle_data.y, obstacle_data.width, obstacle_data.height, color_selector.value)
      obstacles.push(obstackle);
          
      state.obstackles.push({
            id : obstackle.id,
            x: obstackle.x,
            y: obstackle.y,
            width: obstackle.width,
            height: obstackle.height,
            color : obstackle.color

      });
        for(let i= 0; i< obstacles.length;i++){
          console.log("obstecle id: "+ obstackle.id);
        }
    } else {
      console.error('Failed to create obstacle');
    }
  });
}

function create_special_object_service(id,x,y,radius,color){
  let request = new ROSLIB.ServiceRequest({
    id: id,
    position_x : x,
    position_y : y,
    radius : radius,
    color : color
  
  });
  create_special_object_client.callService(request, function(result) {
    if (result.result) {
      console.log(result.result);
      let spec_data = JSON.parse(result.result);
      spec_obj = new Special(spec_data.id,spec_data.x,spec_data.y, spec_data.radius,color_selector.value);
      special_objects.push(spec_obj);
      state.special_objects.push({
        id: spec_obj.id,
        x : spec_obj.x,
        y : spec_obj.y,
        radius : spec_obj.radius,
        color : spec_obj.color
      })
    } else {
      console.error('Failed to create special_object');
    }
  });

}


function load_simulation_state(){
  let request = new ROSLIB.ServiceRequest({
    request: "give me state"
  
  });

  load_simulation_state_client.callService(request, function(result) {
    if (result.response) {
      console.log(result.result);
  
    
          console.log('state loaded succesfully' + result.response);
          parsed_result = JSON.parse(result.response)
          
          for(let i= 0; i< parsed_result.obstacles.length;i++){
            let obj = parsed_result.obstacles[i]
            let obstacle = new Obstackle(obj.id,obj.x,obj.y,obj.width,obj.height,obj.color);
            obstacle.angle = obj.angle;
            obstacles.push(obstacle);
            state.obstackles.push(obstacle);
    
          }
          for (let i = 0; i< parsed_result.agents.length; i++) {
                let agent_data = parsed_result.agents[i]
                let agent = new Agent(agent_data.x, agent_data.y ,agent_data.fuzzy_rules);
                agent.id = agent_data.id;
                agent.name = agent_data.name;
                agent.pos_x = agent_data.x;
                agent.pos_y = agent_data.y;
                agent.frontAngle = agent_data.orientation;
                agent.angle = agent_data.orientation;
                agent.fuzzy_rules = agent_data.fbdl;
                agent.state = agent_data.state;
                entities.push(agent);
                //console.log(data.agents.length);
    
                //cmd_vel_topic = agent.name+"/cmd_vel";
                position_topic = agent.name+"/position"; 
                name_topic = "agent_names";  
                state_topic = agent.name+"/state"  ;
                battery_topic = agent.name+"/battery";
                statistics_topic = agent.name+"/statistics";
                //subscribe_to_topic(topic_sub(cmd_vel_topic,"geometry_msgs/Twist"));
                subscribe_to_topic(topic_sub(position_topic,"custom_msgs/Position"));
                subscribe_to_topic(topic_sub(name_topic,"custom_msgs/Names"));
                subscribe_to_topic(topic_sub(state_topic,"std_msgs/Int16"));
                subscribe_to_topic(topic_sub(battery_topic,"std_msgs/Float32"));
                subscribe_to_topic(topic_sub(statistics_topic,"std_msgs/String"));
              
                console.log("maaaaaaaaa"+entities.length);
            }
            for(let i= 0; i< parsed_result.special_objects.length;i++){
              let special = parsed_result.special_objects[i]
              let special_object = new Special(special.id,special.x,special.y,special.radius,special.color);
              
              special_objects.push(special_object);
              state.special_objects.push(special_object);
            }
       
      
            for(let i = 0 ; i< special_objects; i++){
              special_objects[i].show()
            }
            for(let i = 0 ; i< entities; i++){
              entities[i].show()
            }
            for(let i = 0 ; i< obstacles; i++){
              obstacles[i].show()
            }
     
    } else {
      console.error('Failed to create special_object');
    }
  });
}

// function sub_to_topics(x){
  
//   for(let i = 0; i< x; i++){
//     entities.push(new Agent());
    
//     entities[i].show();
        
//     cmd_vel_topic = entities[i].name+"/cmd_vel";
//     position_topic = entities[i].name+"/position"; 
//     name_topic = "agent_names";  
//     state_topic = entities[i].name+"/state"  
//     battery_topic = entities[i].name+"/battery"
//     distance_topic = entities[i].name+"/distance"
    
//     subscribe_to_topic(topic_sub(cmd_vel_topic,"geometry_msgs/Twist"));
//     subscribe_to_topic(topic_sub(position_topic,"custom_msgs/Position"));
//     subscribe_to_topic(topic_sub(name_topic,"custom_msgs/Names"));
//     subscribe_to_topic(topic_sub(state_topic,"std_msgs/String"));
//     subscribe_to_topic(topic_sub(battery_topic,"std_msgs/Float32"));
//     subscribe_to_topic(topic_sub(distance_topic,"custom_msgs/Distance")); 
   
//   }
// }

  async function count_sub() {
  

  let request = new ROSLIB.ServiceRequest({
    width: canvas_w,
    height : canvas_h
  });

  
  return new Promise((resolve) => {
    canvas_dimension_service.callService(request, function(result) {

      console.log("agent number : " + result.agent_number);
       agent_number= result.agent_number
       resolve(agent_number)
      });
    
  })
}

function selected(){

  for(let i = 0; i< entities.length; i++){
    let d = dist(mouseX,mouseY, entities[i].x, entities[i].y);
    if(d < entities[i].radius/2){
      selected_agent = entities[i];
      current_selected = selected_agent 
      console.log("agent_selected" + selected_agent.id);
      selected_object_type = "agent";
      
    }
 }
}

function select_spec_obj(){
  for(let i = 0; i< special_objects.length; i++){
    let d = dist(mouseX,mouseY, special_objects[i].x, special_objects[i].y);
    if(d < special_objects[i].radius/2){
      selected_special_object = special_objects[i];
      current_selected = selected_special_object
      console.log("spec_object selected");
      move_spec_obj = true;
      selected_object_type = "special_object";
    }
  }
}
let move_obstackle = false;

function selected_obstackles(){

for(let i = 0; i< obstacles.length; i++){
  let d = dist(mouseX,mouseY, obstacles[i].x, obstacles[i].y);
  if (
    mouseX >= obstacles[i].x &&
    mouseX <= obstacles[i].x + obstacles[i].width &&
    mouseY >= obstacles[i].y &&
    mouseY <= obstacles[i].y + obstacles[i].height
  ){
    selected_obstacle = obstacles[i];
    current_selected = selected_obstacle
    console.log("obstackle selected " +selected_obstacle.id);
    move_obstackle = true;
    selected_object_type = "obstacle";
   
    }

  }
}

function mousePressed(){
  selected();
  selected_obstackles();
  select_spec_obj();

  let d = dist(mouseX,mouseY, spawn_point.x, spawn_point.y);

  //console.log(d);
  if(d < 25){
    move_cube = true;
  }else{
      move_cube = false;
   
  }

    if (clicked && mouseX >= 0 && mouseX <= canvas_w && mouseY >= 0 && mouseY <= canvas_h) {
      
      create_obstacle_service(0,mouseX,mouseY ,20 ,100,2)
  }

  if (add_spec_obj && mouseX >= 0 && mouseX <= canvas_w && mouseY >= 0 && mouseY <= canvas_h) {
   
      create_special_object_service(0,mouseX,mouseY,60,color_selector.value)
      add_spec_obj = false;
  }
 
  
}

function mouseReleased(){
move_cube = false;
move_obstackle = false;
move_spec_obj = false;

}

function mouseDragged(){

if(move_cube){
  spawn_point.x = mouseX;
  spawn_point.y = mouseY;
}
if(move_spec_obj){
  selected_special_object.pos_x = mouseX;
  selected_special_object.pos_y = mouseY;
  

}
if(move_obstackle){
    selected_obstacle.x = mouseX - selected_obstacle.width / 2;
    selected_obstacle.y = mouseY - selected_obstacle.height / 2;
    
}
}

async function setup() {
  frameRate(60);
  let canvas =  createCanvas(canvas_w,canvas_h);
 
  canvas.parent("canvas");   //máskülönben minden más html elem felette lenne pozicionálva.
  spawn_point.x = 100;
  spawn_point.y = 100;
  recharge_station.y = 26
  recharge_station.x = 600
  specobj_position_topic = "/spec_obj_position"; 
  console.log(canvas_w + " +" + canvas_h );
  canvas_pub.publish(canvas_data); 

   
  states.style.height = canvas_h +"px";

  subscribe_to_topic(topic_sub(specobj_position_topic,"custom_msgs/SpecObject"))
  
  error_handler();
  //Meg kell várni az agent nodeok számát ,hogy újra megjelenjenek és ne tünjenek el.
  
  // let x = await count_sub();
  // if(x > 0){
  //    sub_to_topics(x);
  // }
  load_simulation_state()
 }

function draw_boundary(){
  push();
    stroke(255,0,0);
    strokeWeight(10);
    line(0, 0, canvas_w, 0); // Top line
    line(canvas_w, 0, canvas_w, canvas_h); // Right line
    line(canvas_w, canvas_h, 0, canvas_h); // Bottom line
    line(0, canvas_h, 0, 0); // Left line
  pop();
}

function draw() {

  background(0);    //canvas törlés

  draw_boundary();

  if (json_obj != null && ! pause) {
    for (let i = 0; i < json_obj.obstacles.length; i++) {
        let x = json_obj.obstacles[i].x;
        let y = json_obj.obstacles[i].y;
        let angle = json_obj.obstacles[i].angle;
        let id = json_obj.obstacles[i].id;
   
        let matchingBox = obstacles.find(box => box.id === id);
        
        if (matchingBox) {
            matchingBox.x = x
            matchingBox.y = y
            matchingBox.angle = angle
        }
    }
}
// Draw all boxes after updating their positions

  for (let i = 0; i < special_objects.length; i++) {
    special_objects[i].show();
  }
 
  for (let i = 0; i < obstacles.length; i++) {
      obstacles[i].show();
  }
       spawn_point.show();
      
 
  for(let i = 0; i< entities.length; i++){
 
    if (entities[i] === selected_agent) {
        
      // Display the selected agent's data
           
      display_name.innerHTML = entities[i].name;
      statistics.innerHTML = entities[i].statistics;
           
         
      // Add other display logic here if needed
    }
 
        entities[i].show();
        entities[i].move();
      }
 }

 function send_canvas(){
  canvas_pub.publish(canvas_data); 
}

setInterval(send_canvas,500)

function target_position_inteval(){
  target_for_collider = new ROSLIB.Message({pos_x : spawn_point.x, pos_y : spawn_point.y});
  //recharger_station_pos = new ROSLIB.Message({pos_x : recharge_station.x, pos_y : recharge_station.y});
  
  publish_to_topic("/agent_spawner_position","custom_msgs/Position").publish(target_for_collider);
  //publish_to_topic("/recharger","custom_msgs/Position").publish(recharger_station_pos);

}

setInterval(target_position_inteval,100)

setInterval(function(){
  let spec_obj_msg;
  for(let i = 0; i < special_objects.length; i++){
    spec_obj_msg = new ROSLIB.Message({id : special_objects[i].id, position_x : special_objects[i].x, position_y : special_objects[i].y, radius : special_objects[i].radius, color : special_objects[i].color});
    publish_to_topic("/spec_obj_state","custom_msgs/SpecObject").publish(spec_obj_msg);
  }
},500);

setInterval(function(){
  
  //console.log(stateJson)
  for(let i= 0; i< obstacles.length; i++){
   let json_data = new ROSLIB.Message({id : obstacles[i].id, position_x : obstacles[i].x, position_y : obstacles[i].y, width : obstacles[i].width, height: obstacles[i].height});
   publish_to_topic("/obstacle_state","custom_msgs/Obstacles").publish(json_data);
    
  }
 
},100)

setInterval(function(){
  
  for(let i = 0; i< entities.length; i++){
    if(entities[i].target != null){
      let target_pos_msg = new ROSLIB.Message({pos_x : entities[i].target.x, pos_y : entities[i].target.y});
  
    publish_to_topic(entities[i].name+ "/target_position","custom_msgs/Position").publish(target_pos_msg);
    
    }
    
  }
 
},100)