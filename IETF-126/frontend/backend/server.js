const express = require("express");
const app = express();
const cors = require("cors");
require("dotenv").config({ path: "./config.env" });
const port = process.env.PORT || 8000;
app.use(cors());
app.use(express.json());
app.use(require("./routes/record"));
// get driver connection
const dbo = require("./db/conn");
 
app.listen(port, () => {
  // perform a database connection when server starts
  dbo.connectToServer(function (err) {
    if (err) console.error(err);
 
  });
  console.log(`Server is running on port: ${port}`);
});

// NATURAL INTENT API
app.post("/api/convert_intent/", (req, res) => {
  console.log("Received intent:", req.body);

  // 여기는 임시 응답
  res.json({
    success: true,
    message: "Intent received successfully",
    data: req.body,
  });
});
