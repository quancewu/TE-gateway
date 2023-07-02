document.getElementById("test").innerHTML = "WebSocket is not connected";

var websocket = new WebSocket('ws://'+location.hostname+':30678');
var slider = document.getElementById("myRange");
var colorselecter = document.getElementById("colorset");
var datetimepicker = document.getElementById("datePicker");
var pwdInput = document.getElementById("ftext1");

// slider.oninput = function () {
//   websocket.send("L" + slider.value);
// }

// colorselecter.oninput = function () {
//   websocket.send("C" + colorselecter.value);
// }

// datetimepicker.oninput = function () {
//   websocket.send("M" + datetimepicker.value);
// }


function sendMsg() {
  websocket.send('L50');
  console.log('Sent message to websocket');
}

function sendText(text) {
  websocket.send("M" + text);
}

function sendTextEng(text) {
  var getSelectedValue = document.querySelector( 'input[name="sendid"]:checked');
  var index = document.getElementsByName('quantity')[0].value
  if(getSelectedValue != null) {   
    console.log("Selected radio button values is: " + getSelectedValue.value);
    var sendtext = ("E:" + getSelectedValue.value + " " + index + ", " + text);
    getSelectedValue.checked = false;
    console.log(sendtext);
    websocket.send(sendtext);
  }
  else{
    console.log("E" + text);
    websocket.send("E" + text);
  }
}

function sendMqttInfo(text) {
  var getSelectedValue = document.querySelector( 'input[name="sendmqtt"]:checked');
  if(getSelectedValue != null) {   
    console.log("Selected radio button values is: " + getSelectedValue.value);
    var sendtext = ("Q: " + getSelectedValue.value + " : " + text);
    getSelectedValue.checked = false;
    console.log(sendtext);
    websocket.send(sendtext);
  }
}


websocket.onopen = function(evt) {
  console.log('WebSocket connection opened');
  websocket.send("It's open!");
  document.getElementById("test").innerHTML = "WebSocket is connected!";
  // document.getElementById("client-id-table").rows[0].cells[0].innerHTML = "test1";
}

websocket.onmessage = function(evt) {
  var msg = evt.data;
  var value;
  switch(msg.charAt(0)) {
    case 'L':
      console.log(msg);
      value = parseInt(msg.replace(/[^0-9\.]/g, ''), 10);
      slider.value = value;
      console.log("Led = " + value);
      break;
    case 'C':
      console.log(msg);
      value = msg.replace(/[C]/g, '');
      colorselecter.value = value;
      console.log("Leds = " + value);
      break;
    case 'E':
      if (!msg.match(/E:/)) {
        console.log(msg);
      }
      else {
        value = msg.replace(/E:/g, '');
        value = value.toString();
        console.log(value);
        // data = data.trim()
        var data = JSON.parse(value);
        var dateTime = new Date(data.update_time*1000);
        console.log(`UpdateTimestamp: ${data.update_time}`);
        document.getElementById("client-id-table").rows[8].cells[1].innerHTML = dateTime.toISOString();
        console.log(`Gatway: ${data.gw}`);
        document.getElementById("client-id-table").rows[1].cells[1].innerHTML = data.tegw;
        console.log(`LR: ${data.lr}`);
        document.getElementById("client-id-table").rows[2].cells[1].innerHTML = data.hostname;
        console.log(`mqttUri: ${data.mqttUri}`);
        document.getElementById("client-id-table").rows[3].cells[1].innerHTML = data.mqttUri;
        console.log(`pub: ${data.pub}`);
        document.getElementById("client-id-table").rows[4].cells[1].innerHTML = data.psk;
        console.log(`sub: ${data.sub}`);
        document.getElementById("client-id-table").rows[5].cells[1].innerHTML = data.pskhint;
        console.log(`pub: ${data.pub}`);
        document.getElementById("client-id-table").rows[6].cells[1].innerHTML = data.pub;
        console.log(`sub: ${data.sub}`);
        document.getElementById("client-id-table").rows[7].cells[1].innerHTML = data.sub;
        console.log(`SU1: ${data.su1}`);
        document.getElementById("client-msu-table").rows[1].cells[1].innerHTML = data.su1;
        console.log(`SU2: ${data.su2}`);
        document.getElementById("client-msu-table").rows[2].cells[1].innerHTML = data.su2;
        console.log(`SU3: ${data.su3}`);
        document.getElementById("client-msu-table").rows[3].cells[1].innerHTML = data.su3;
        console.log(`SU4: ${data.su4}`);
        document.getElementById("client-msu-table").rows[4].cells[1].innerHTML = data.su4;
        console.log(`SU5: ${data.su5}`);
        document.getElementById("client-msu-table").rows[5].cells[1].innerHTML = data.su5;
        console.log(`SU6: ${data.su6}`);
        document.getElementById("client-msu-table").rows[6].cells[1].innerHTML = data.su6;
        console.log(`SU1: ${data.dp1}`);
        document.getElementById("client-msu-table").rows[1].cells[2].innerHTML = data.dp1;
        console.log(`SU2: ${data.dp2}`);
        document.getElementById("client-msu-table").rows[2].cells[2].innerHTML = data.dp2;
        console.log(`SU3: ${data.dp3}`);
        document.getElementById("client-msu-table").rows[3].cells[2].innerHTML = data.dp3;
        console.log(`SU4: ${data.dp4}`);
        document.getElementById("client-msu-table").rows[4].cells[2].innerHTML = data.dp4;
        console.log(`SU5: ${data.dp5}`);
        document.getElementById("client-msu-table").rows[5].cells[2].innerHTML = data.dp5;
        console.log(`SU6: ${data.dp6}`);
        document.getElementById("client-msu-table").rows[6].cells[2].innerHTML = data.dp6;
      }
      
      break;
    case 'M':
      break;
    default:
      document.getElementById("output").innerHTML = evt.data;
      var dateTime = new Date(evt.data*1000);
      document.getElementById("update-time-table").rows[1].cells[1].innerHTML = dateTime.toISOString();
      document.getElementById("update-time-table").rows[2].cells[1].innerHTML = dateTime.toLocaleString();
      // console.log("counts = " + evt.data);
      break;
  }
}

websocket.onclose = function(evt) {
  console.log('Websocket connection closed');
  document.getElementById("test").innerHTML = "WebSocket closed";
}

websocket.onerror = function(evt) {
  console.log('Websocket error: ' + evt);
  document.getElementById("test").innerHTML = "WebSocket error!";
}



pwdInput.addEventListener("keyup", function(event) {
  event.preventDefault();
  if (event.key === "Enter") {
    // event.preventDefault();
    document.getElementById("btn01").click();
    // sendText(document.getElementsByName('ftext1')[0].value);
  }
});