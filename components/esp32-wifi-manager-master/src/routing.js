const DataPage = () => {
  location.href = '/live';
}

const OtaPage = () => {
  location.href = '/ideaskyota';
}

const ControlPage = () => {
  location.href = '/control';
}

const WifiConnectionManagementPage = () => {
  location.href = '/';
}

const WifiDetailPage = () => {
  wifi_div.style.display = "none";
  document.getElementById("connect-details").style.display = "block";
}

const routingSecGenerator = () => {
  routingSecs = document.querySelectorAll(".routing-sec");
  routingSecs.forEach(routingSec => {
    routingSec.InnerHTML = "";

    let divNode = document.createElement("div");
    divNode.innerHTML = "";
    divNode.innerHTML += '<input type="button" class="da" style="width: 65px; margin:10px;" onclick="DataPage()" aria-label="DataPage" />'
    divNode.innerHTML += '<input type="button" class="wi" style="width: 65px; margin:10px;" onclick="WifiConnectionManagementPage()" aria-label="Wifi" />'
    divNode.innerHTML += '<input type="button" class="ul" style="width: 65px; margin:10px;" onclick="OtaPage()"  aria-label="OTA" />'
    divNode.innerHTML += '<input type="button" class="cn" style="width: 65px; margin:10px;" onclick="ControlPage()"  aria-label="Control" />'
    routingSec.appendChild(divNode);
  });
}

routingSecGenerator()