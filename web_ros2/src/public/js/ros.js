//Create ros object to communicate over Rosbridge connection
const ros = new ROSLIB.Ros({ url : 'ws://localhost:9090' });
//192.168.0.112
//192.168.1.72
// When the Rosbridge server connects, fill the span with id “status" with “successful"
ros.on('connection', () => {
   document.getElementById("status").innerHTML = "successful";
 });

 // When the Rosbridge server experiences an error, fill the “status" span with the returned error
 ros.on('error', (error) => {
   document.getElementById("status").innerHTML = `errored out (${error})`;
 });

 // When the Rosbridge server shuts down, fill the “status" span with “closed"
 ros.on('close', () => {
   document.getElementById("status").innerHTML = "closed";
 });

function topic_sub(topic, topic_type){

    let position_listener = new ROSLIB.Topic({
      ros : ros,
      name : topic,
      messageType : topic_type
    
  });
  
  return position_listener
  }
  
  let StateListener = new ROSLIB.Topic({
    ros : ros,
    name : "/state",
    messageType : "std_msgs/String"
  
  });
  
  let position_pub = new ROSLIB.Topic({
    ros : ros,
    name : "/position",
    messageType : "custom_msgs/Position"
  
  });
  
  
  let canvas_pub = new ROSLIB.Topic({
    ros : ros,
    name : "/canvas",
    messageType : "custom_msgs/Canvas"
  
  });
  
  let spawner_pub = new ROSLIB.Topic({
    ros : ros,
    name : "/agent_spawn",
    messageType : "std_msgs/Int16"
  
  });

 
  
  let entity_creator_topic = new ROSLIB.Topic({
    ros : ros,
    name : "/new_agent",
    messageType : "std_msgs/String"
  
  }); 
  
  let distance_topic = new ROSLIB.Topic({
    ros : ros,
    name : "/distance",
    messageType : "std_msgs/Float32"
  
  }); 
  let agent_count = new ROSLIB.Topic({
    ros : ros,
    name : "/count",
    messageType : "std_msgs/Int16"
  
  }); 
  
  let counter = new ROSLIB.Topic({
    ros : ros,
    name : "/counter",
    messageType : "std_msgs/Int16"
  
  }); 

  let error_sub = new ROSLIB.Topic({
    ros : ros,
    name : "/error",
    messageType : "std_msgs/String"
  
  });
  
  let agent_names_topic = new ROSLIB.Topic({
    ros : ros,
    name : "/names",
    messageType : "std_msgs/String"
  
  });    
  
  let kocka_position_topic = new ROSLIB.Topic({
    ros : ros,
    name : "/agent_spawner_position",
    messageType : "custom_msgs/Position"
  
  });
  
  let fuzzy_rule_topic = new ROSLIB.Topic({
    ros : ros,
    name : "/fuzzy",
    messageType : "std_msgs/String"
  
  });
  
  let stop_topic = new ROSLIB.Topic({
    ros : ros,
    name : "/stop",
    messageType : "std_msgs/Int16"
  
  });

  let universe_topic = new ROSLIB.Topic({
    ros : ros,
    name : "/universes",
    messageType : "std_msgs/String"
  
  });

   let ok = false;      //változó a név adás előtti kirajzoláshoz.
   
  
  function subscribe(topic){
    topic.subscribe(function(message) {
      console.log(message.data);
      
    });
  
  }
  function publish_to_topic(topic, type){
    let fuzzy_topic = new ROSLIB.Topic({
      ros : ros,
      name : topic,
      messageType : type
    
    });
    return fuzzy_topic;
  }
  const listener = new ROSLIB.Topic({
    ros: ros,
    name: "/obstacle_pos",
    messageType: "std_msgs/String"
});

const delet_agent_topic = new ROSLIB.Topic({
  ros: ros,
  name: "/delete_agent",
  messageType: "std_msgs/String"
});

function unsubscribe_from_topic(topic){
  topic.unsubscribe(topic)
}
  
  function subscribe_to_topic(topic) {
    topic.subscribe(function(message) {

      for (let i = 0; i < entities.length; i++) {
        // if (topic.name == entities[i].name + "/cmd_vel") {
        //  // console.log("Received message for agent " + (i + 1) + ": ", message);
        //   entities[i].velx = message.linear.x;
        //   entities[i].angz = message.angular.z;
        // }
  
        if (topic.name == entities[i].name  + "/position") {
          // console.log("Received message for agent " + (i + 1) + ": ", message);
           entities[i].pos_x = message.pos_x ;
           entities[i].pos_y = message.pos_y ;
           
           entities[i].angle = message.orientation ;
        }
        if (topic.name == entities[i].name  + "/statistics") {
          // console.log("Received message for agent " + (i + 1) + ": ", message);
          let receivedDict = JSON.parse(message.data);
          entities[i].statistics = ""
          for (const [key, value] of Object.entries(receivedDict)) {
            //console.log(key, value);
            entities[i].statistics += `<span class="key">${key}:</span><br><span class="value">${value.toFixed(2)}</span><br>`;
            
          }
          
            
        }

        if (topic.name == entities[i].name  + "/state") {
            //console.log("Received message for agent " + (i + 1) + ": ", message);
            entities[i].state = message.data;
        } 
        if (topic.name == entities[i].name  + "/battery") {
           //console.log("Received message for agent " + (i + 1) + ": ", message);
             entities[i].battery_level = message.data;
          
         }

      }
      for(let i = 0; i< special_objects.length; i++){
        if (topic.name == "/spec_obj_position") {
          //console.log("Received message for spec_obj " + (i + 1) + ": ", message);
          if(special_objects[i].id == message.id){
            special_objects[i].pos_x = message.position_x
            special_objects[i].pos_y = message.position_y
          }
        }  
      }
     
     });
  }


 let serviceClient = new ROSLIB.Service({
    ros: ros,
    name: '/delete_agent',
    serviceType: 'custom_msgs/Delete'
  });

  let newAgent_client = new ROSLIB.Service({
    ros: ros,
    name: '/new_agent',
    serviceType: 'custom_msgs/NewAgent'
  });

  let delete_obstacle_client = new ROSLIB.Service({
    ros: ros,
    name: '/delete_obstacle',
    serviceType: 'custom_msgs/DeleteObstacle'
  });
  
  let canvas_dimension_service = new ROSLIB.Service({
    ros: ros,
    name: '/canvas_dimensions',
    serviceType: 'custom_msgs/CanvasDimensions'
  });
  
  let fuzzy_client = new ROSLIB.Service({
    ros: ros,
    name: '/fbdl',
    serviceType: 'custom_msgs/Fbdl'
  });
  

  let create_obstacle_client = new ROSLIB.Service({
    ros: ros,
    name: '/create_obstacle',
    serviceType: 'custom_msgs/CreateObstacle'
  });


  let create_special_object_client = new ROSLIB.Service({
    ros: ros,
    name: '/create_special_object',
    serviceType: 'custom_msgs/CreateSpecialObject'
  });

  let delete_special_object_client = new ROSLIB.Service({
    ros: ros,
    name: '/delete_special_object',
    serviceType: 'custom_msgs/DeleteObstacle'
  });


  let change_dimensions_client = new ROSLIB.Service({
    ros: ros,
    name: '/change_dimensions',
    serviceType: 'custom_msgs/ChangeDimensions'
  });

  let change_color_client = new ROSLIB.Service({
    ros: ros,
    name: '/change_color',
    serviceType: 'custom_msgs/ChangeColor'
  });

  let load_client = new ROSLIB.Service({
    ros: ros,
    name: '/load',
    serviceType: 'custom_msgs/Load'
  });


  let load_simulation_state_client = new ROSLIB.Service({
    ros: ros,
    name: '/simulation_state',
    serviceType: 'custom_msgs/SendState'
  });

  let universes_service = new ROSLIB.Service({
    ros: ros,
    name: '/universes',
    serviceType: 'custom_msgs/Universes'
  });


  let error_service = new ROSLIB.Service({
    ros: ros,
    name: '/error',
    serviceType: 'custom_msgs/Error'
  });

  let stop_service = new ROSLIB.Service({
    ros: ros,
    name: '/stop',
    serviceType: 'custom_msgs/Stop'
  });

  let weight_service = new ROSLIB.Service({
    ros: ros,
    name: '/change_weight',
    serviceType: 'custom_msgs/Weight'
  });