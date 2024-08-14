const express = require("express");
const fs = require('fs');
const path = require('path');

const app = express();

// server your css as static
app.use(express.static('public'));
app.use(express.static('public/maps'));
app.use(express.json()); //For communication via json
app.use(express.urlencoded({ extended: true }));   //this is needed for the server to parse url-encoded-data

const directoryPath = path.join(__dirname, 'public', 'maps');

app.get('/files', (req, res) => {
  fs.readdir(directoryPath, (err, files) => {
    if (err) {
      console.error('Error reading directory:', err);
      return res.status(500).json({ error: 'Internal server error' });
    }

    const responseData = {
    
      someValue: files, 
    };

    res.json(responseData);
  });
});

app.get('/', function (req, res) {
  res.sendFile(__dirname + '/public/html/index.html');
 
});

// app.listen(3000, '0.0.0.0', () => {
//   console.log(`Server running at http://localhost/`);    //if other machines want to connect
// });
app.listen(3000, () => {
  console.log("Application started and Listening on port 3000");
});

////////////////////////////////////////////////////////////////////
//DB

// const sqlite3 = require('sqlite3').verbose();
// const db = new sqlite3.Database('/home/berquor/ros2_fuzzy_ws/simulation.db', sqlite3.OPEN_READONLY);


// // Execute the query to create the table
// db.serialize(() => {
 
//   const query = 'SELECT * FROM obstacles';
//   db.all(query, (err, rows) => {
//     if (err) {
//         console.error('Error executing query:', err.message);
//     } else {
//         console.log('Records from the "obstacles" table:');
//         rows.forEach(row => {
//             console.log(row);
//         })
//     }
    
//   });


// });

// app.post("/create_obstacle", (req, res) => {
//   // Query the database
//   db.get("SELECT * FROM obstacles ORDER BY id DESC LIMIT 1", (err, row) => {
//     if (err) {
//       console.error(err);
//       res.status(500).json({ error: "An error occurred" });
//     } else {
//       // Convert the data to JSON
      
//       // Send the JSON data as the response
//       res.json(row);
//     }
//   });

// });
// const query = 'SELECT * FROM obstacles';
// app.get('/load_state', (req, res) => {

  
//     db.all(query, (err, rows) => {
//       if (err) {
//         console.error(err);
//         res.status(500).json({ error: 'An error occurred' });
//       } else {
//         // Send the JSON data as the response
//         console.log("aaaaaaa" , rows)
//         res.json(rows);
//       }
//     });
 

//   // db.get("SELECT * FROM agents", (err, row) => {
//   //   if (err) {
//   //     console.error(err);
//   //     res.status(500).json({ error: "An error occurred" });
//   //   } else {
//   //     // Convert the data to JSON
      
//   //     // Send the JSON data as the response
//   //     res.json(row);
//   //   }
//   // });
//   // db.get("SELECT * FROM special_objects", (err, row) => {
//   //   if (err) {
//   //     console.error(err);
//   //     res.status(500).json({ error: "An error occurred" });
//   //   } else {
//   //     // Convert the data to JSON
//   //     console.log("OKIDOKI")
//   //     // Send the JSON data as the response
//   //     res.json(row);
//   //   }
//   // });

// });


// app.post('/create_agent', (req, res) => {
//   let agent_id = req.body.id;
//   let agent_name = req.body.name;
//   console.log('Received agentName:', agent_id, agent_name); 

//   db.run('INSERT INTO agents (id, name) VALUES (?, ?)', [agent_id, agent_name], function(err) {
//     if (err) {
//       return res.status(500).json({ error: err});
//     }
  
//   });

//   db.all(query, (err, rows) => {
//     if (err) {
//       console.error('Error executing query:', err.message);
//     } else {
//       // Process the retrieved data (rows)
//       console.log('Retrieved data:', rows);
//     }
//   });
// });

// app.post('/delete_agent', (req, res) => {
//   let agent_id = req.body.id;
  
//   console.log('Deleted agentName:', agent_id); 

//   db.run('DELETE FROM agents WHERE id = ?', agent_id, function(err) {
//     if (err) {
//       console.error('Error executing DELETE query:', err.message);
//     } else {
//       console.log(`Record with ID ${agent_id} deleted successfully`);
//     }
//   });

// });
// process.on('SIGINT', () => {
//   db.close();
//   app.close();
// });