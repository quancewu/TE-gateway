const gel = (e) => document.getElementById(e);

function docReady(fn) {
  // see if DOM is already available
  if (
    document.readyState === "complete" ||
    document.readyState === "interactive"
  ) {
    // call on next available tick
    setTimeout(fn, 1);
  } else {
    document.addEventListener("DOMContentLoaded", fn);
  }
}

var refreshAeroboxInterval = null;

function stopRefreshAeroboxInterval() {
  if (refreshAeroboxInterval != null) {
    clearInterval(refreshAeroboxInterval);
    refreshAeroboxInterval = null;
  }
}


function startRefreshAeroboxInterval() {
  refreshAeroboxInterval = setInterval(refreshdata, 3800);
}

docReady(async function () {
  startRefreshAeroboxInterval();
});
async function refreshdata(url = "data.json") {
    // try {
      var res = await fetch(url);
      var Aerobox_data = await res.json();
      console.info(Aerobox_data);
      // console.info(Aerobox_data.length);
      refreshAeroboxHTML(Aerobox_data);
  }

  function refreshAeroboxHTML(data) {
    var h = "";
    h +=`<table style="width:100%"><tr>
          <th>Parameter</th>
          <th>Value</th>
          <th>Uint</th>
        </tr>`
    h +=`<tr>
        <td>PM2.5</td>
        <td>${data.pm25}</td>
        <td>um/m^3</td>
      </tr>`
    h +=`<tr>
      <td>PM10</td>
      <td>${data.pm10}</td>
      <td>um/m^3</td>
    </tr>`
    h +=`<tr>
        <td>CO2</td>
        <td>${data.co2}</td>
        <td>ppm</td>
      </tr>`
    h +=`<tr>
      <td>temperature</td>
      <td>${data.temp/100}</td>
      <td>degree C</td>
    </tr>`
    h +=`<tr>
      <td>RH</td>
      <td>${data.rh/100}</td>
      <td>%</td>
    </tr>`
    h +=`<tr>
      <td>ID</td>
      <td>${data.id}</td>
      <td>hex</td>
    </tr></table>`
 
    gel("data-list").innerHTML = h;
  }