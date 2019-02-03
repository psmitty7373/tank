function isPrivateAddress(ipaddress) {
    var parts = ipaddress.split('.');
    return parts[0] === '10' ||
            (parts[0] === '172' && (parseInt(parts[1], 10) >= 16 && parseInt(parts[1], 10) <= 31)) ||
            (parts[0] === '192' && parts[1] === '168');
};

pos_log = [];

function connect() {
    if(isPrivateAddress(location.host))
            websocket_url = "ws://" + location.hostname + ':8000/tank_ws';
    else
            websocket_url = "wss://" + location.hostname + '/tank_ws';

    ws = new WebSocket(websocket_url);
    ws.onclose = function() {
        setTimeout(function() {
            connect();
        }, 500);
    }
    ws.onmessage = function(event) {
        document.getElementById("status-bar").innerHTML = event.data;
        msg = JSON.parse(event.data);
        // heading
        if (!msg.a_x)
            document.getElementById("heading").innerHTML = '000';
        else
            document.getElementById("heading").innerHTML = Math.round(msg.a_x).toString(10).padStart(3,'0');
        // position & minimap
        var canvas = document.getElementById("minimap");
        var context = canvas.getContext("2d");
        context.clearRect(0, 0, canvas.width, canvas.height);

        context.strokeStyle = "black";
        context.fillStyle = "white";
        context.beginPath();
        context.lineWidth = 2;
        context.arc(canvas.width/2 + 2, canvas.height/2 + 2, canvas.width / 2 - 4, 0, 2 * Math.PI);
        context.stroke();
        context.fill();

        pos_log.unshift({x: msg.x * 100, y: msg.y * 100});
        pos_log = pos_log.splice(0,50);
        context.fillStyle = "black";
        for (i = 0; i < pos_log.length; i++) {
            context.fillRect(pos_log[i].x, pos_log[i].y, 2, 2);
        }
        context.fillStyle = "red";
        context.fillRect(msg.x * 100, msg.y * 100, 4, 4);

        // reticle
        canvas = document.getElementById("hud_canvas");
        canvas.width = canvas.offsetWidth;
        canvas.height = canvas.offsetHeight;
        context = canvas.getContext("2d");
        context.clearRect(0, 0, canvas.width, canvas.height);
        context.beginPath();
        context.strokeStyle = "#00FF00";
        context.lineWidth = 3;
        context.arc(canvas.width/2, canvas.height/2, 10, 0, 2 * Math.PI);
        context.stroke();
        // right line
        context.beginPath();
        context.moveTo(canvas.width/2 + 10, canvas.height/2);
        context.lineTo(canvas.width/2 + 65, canvas.height/2);
        context.stroke();
        // left line
        context.beginPath();
        context.moveTo(canvas.width/2 - 10, canvas.height/2);
        context.lineTo(canvas.width/2 - 65, canvas.height/2);
        context.stroke();
        // top
        context.beginPath();
        context.moveTo(canvas.width/2, canvas.height/2 + 10);
        context.lineTo(canvas.width/2, canvas.height/2 + 25);
        context.stroke();
        // bottom
        context.beginPath();
        context.moveTo(canvas.width/2, canvas.height/2 - 10);
        context.lineTo(canvas.width/2, canvas.height/2 - 25);
        context.stroke();
    };
}

console.log("touchscreen is", VirtualJoystick.touchScreenAvailable() ? "available" : "not available");

var joystick = new VirtualJoystick({
    container : document.getElementById('interface'),
    mouseSupport : true,
    limitStickTravel : true,
    stickRadius: 100
});

joystick.addEventListener('touchStart', function(){
    console.log('down')
});

joystick.addEventListener('touchEnd', function(){
    console.log('up')
});

function sendJoystick(j) {
    if (ws.readyState == 1) {
        pos = {x:Math.round(j.deltaX()), y: Math.round(-j.deltaY())};
        ws.send(JSON.stringify(pos));
    }
}

setInterval(function() {
    sendJoystick(joystick);
}, 100);

function toggleFullScreen() {
    var doc = window.document;
    var docEl = doc.documentElement;

    var requestFullScreen = docEl.requestFullscreen || docEl.mozRequestFullScreen || docEl.webkitRequestFullScreen || docEl.msRequestFullscreen;
    var cancelFullScreen = doc.exitFullscreen || doc.mozCancelFullScreen || doc.webkitExitFullscreen || doc.msExitFullscreen;

    if (!doc.fullscreenElement && !doc.mozFullScreenElement && !doc.webkitFullscreenElement && !doc.msFullscreenElement) {
        requestFullScreen.call(docEl);
    } else {
        cancelFullScreen.call(doc);
    }
}

document.getElementById("fullscreen-button").addEventListener("click", function() {
    toggleFullScreen();
}, false);

document.getElementById("steering-button").addEventListener("click", function() {
    if (document.getElementById("interface").style.pointerEvents == "all") {
        document.getElementById("interface").style.pointerEvents = "none";
        document.getElementById("steering-button").style.backgroundColor = "red";
    } else {
        document.getElementById("interface").style.pointerEvents = "all";
        document.getElementById("steering-button").style.backgroundColor = "green";
    }
}, false);

var websocketSignalingChannel = new WebSocketSignalingChannel(document.getElementById("remoteVideo"));

(function() {
    connect();
    websocketSignalingChannel.doSignalingConnect()
    let canvas = document.getElementById("minimap");
    canvas.width = 300;
    canvas.height = 300;
})();
