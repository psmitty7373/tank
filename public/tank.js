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
        let canvas = document.getElementById("minimap");
        let context = canvas.getContext("2d");
        context.clearRect(0, 0, canvas.width, canvas.height);
        pos_log.unshift({x: msg.x * 100, y: msg.y * 100});
        pos_log = pos_log.splice(0,50);
        context.fillStyle = "black";
        for (i = 0; i < pos_log.length; i++) {
            context.fillRect(pos_log[i].x, pos_log[i].y, 2, 2);
        }
        context.fillStyle = "red";
        context.fillRect(msg.x * 100, msg.y * 100, 4, 4);
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
