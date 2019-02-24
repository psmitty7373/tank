function isPrivateAddress(ipaddress) {
    let parts = ipaddress.split('.');
    return parts[0] === '10' ||
            (parts[0] === '172' && (parseInt(parts[1], 10) >= 16 && parseInt(parts[1], 10) <= 31)) ||
            (parts[0] === '192' && parts[1] === '168');
};

let anchors = [
        {id: 'a', x: 0, y: 0, z: 0 },
        {id: 'b', x: 0, y: 200, z: 0},
        {id: 'c', x: 200, y: 200, z: 0},
        {id: 'd', x: 200, y: 0, z: 0}
];

let pos_log = [];
let FPS = 1000/60;
let last_time = 0;
let start_time = 0;
let video = document.getElementById('remoteVideo');
let reticle_color = "green";

cv['onRuntimeInitialized']=()=>{
    video.width = video.offsetWidth;
    video.height = video.offsetHeight;
    let cap = new cv.VideoCapture(video);
    let greenLower = new cv.Scalar(29, 86, 6);
    let greenUpper = new cv.Scalar(64, 255, 255);

    function processFrame() {

        let src = new cv.Mat(video.height, video.width, cv.CV_8UC4);
        let dst = new cv.Mat();
        let mask = new cv.Mat();
        let contours = new cv.MatVector();
        let hierarchy = new cv.Mat();
        cap.read(src);

        cv.resize(src, src, new cv.Size(300, 300), 0, 0, cv.INTER_AREA);

        let ksize = new cv.Size(7,7);
        cv.GaussianBlur(src, dst, ksize, 0, 0, cv.BORDER_DEFAULT);
        cv.cvtColor(dst, dst, cv.COLOR_BGR2HSV);

        let low = new cv.Mat(dst.rows, dst.cols, dst.type(), greenLower);
        let high = new cv.Mat(dst.rows, dst.cols, dst.type(), greenUpper);

        cv.inRange(dst, low, high, mask);
        cv.erode(mask, mask, new cv.Mat(), new cv.Point(-1, -1), 3);
        cv.dilate(mask, mask, new cv.Mat(), new cv.Point(-1, -1), 3);

        cv.findContours(mask, contours, hierarchy, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE);

        let color = new cv.Scalar(255, 30, 30, 255);
        cv.drawContours(src, contours, -1, color, 1, cv.LINE_8, hierarchy, 100);
        for (let i = 0; i < contours.size(); ++i) {
            let cnt = contours.get(i);
            let br = cv.boundingRect(cnt);
            cv.rectangle(src, new cv.Point(br.x, br.y), new cv.Point(br.x + br.width, br.y + br.height), color, 1);
            if (br.x < video.width / 2 && (br.x + br.width) > video.width / 2 && br.y < video.height / 2 && (br.y + br.height) > video.height/2)
                reticle_color = "red";
            else
                reticle_color = "green";
        }

        cv.imshow('opencv', src);
        low.delete();
        high.delete();
        src.delete();
        dst.delete();
        mask.delete();
        contours.delete();
        hierarchy.delete();

        setTimeout(processFrame, 100);
    }
    setTimeout(processFrame, 0);
}

function get_center() {
    let c_x = 0;
    let c_y = 0;
    for (let i = 0; i < anchors.length; i++) {
                c_x += anchors[i].x;
                c_y += anchors[i].y;
            }
    c_x = c_x / anchors.length;
    c_y = c_y / anchors.length;
    return { c_x: c_x, c_y: c_y };
}

function draw_reticle() {
    // reticle
    let canvas = document.getElementById("hud_canvas");
    canvas.width = canvas.offsetWidth;
    canvas.height = canvas.offsetHeight;
    context = canvas.getContext("2d");
    context.clearRect(0, 0, canvas.width, canvas.height);
    context.beginPath();
    if (reticle_color == "green")
        context.strokeStyle = "#00FF00";
    else
        context.strokeStyle = "#FF0000";
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
}

function draw_minimap() {
    // position & minimap
    let canvas = document.getElementById("minimap");
    let context = canvas.getContext("2d");
    canvas.width  = canvas.offsetWidth;
    canvas.height = canvas.offsetHeight;

    context.clearRect(0, 0, canvas.width, canvas.height);
    context.transform(1, 0, 0, -1, 0, canvas.height);
    let center = get_center();

    // draw minimap circle
    context.strokeStyle = "black";
    context.fillStyle = "white";
    context.beginPath();
    context.lineWidth = 2;
    context.arc(canvas.width/2 + 1, canvas.height/2 + 1, canvas.width / 2 - 4, 0, 2 * Math.PI);
    context.stroke();
    context.fill();

    // center map
    context.translate(canvas.width/2 - center.c_x, canvas.height/2 - center.c_y);

    // draw points
    let last_pos = null;
    for (let i = 0; i < pos_log.length; i++) {
        if (last_pos && (last_pos.x != pos_log[i].x || last_pos.y != pos_log[i].y)) {
            context.lineWidth = 1;
            context.beginPath();
            context.moveTo(last_pos.x, last_pos.y);
            context.lineTo(pos_log[i].x, pos_log[i].y);
            context.stroke();
        }
        last_pos = pos_log[i];
    }
    if (last_pos) {
        context.fillStyle = "red";
        context.fillRect(last_pos.x, last_pos.y, 4, 4);
    }

    // draw anchors
    for (i = 0; i < anchors.length; i++) {
        if (i == 0)
            context.fillStyle = "green";
        else if (i == 3)
            context.fillStyle = "purple";
        else
            context.fillStyle = "blue";
        context.fillRect(anchors[i].x - 2, anchors[i].y - 2, 4, 4);
    }
}

function draw(ts) {
    if (!start_time)
        start_time = ts;

    let time_diff = last_time ? ts - last_time : FPS;
    let time_elapsed = ts - start_time;
    let time_scale = time_diff / FPS;
    
    last_time = ts;

    draw_reticle();
    draw_minimap();

    requestAnimationFrame(draw);
}

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
        
        // update position log
        pos_log.unshift({x: msg.x, y: msg.y});
        pos_log = pos_log.splice(0,50);

    };
}

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
        pos = { x: j.deltaX(), y: -j.deltaY() };
        pos.x = pos.x / 100;
        pos.y = pos.y / 100;
        x = Math.round(0.5 * Math.sqrt(2 + Math.pow(pos.x, 2) - Math.pow(pos.y, 2) + 2 * pos.x * Math.sqrt(2)) - 0.5 * Math.sqrt(2 + Math.pow(pos.x, 2) - Math.pow(pos.y, 2) - 2 * pos.x * Math.sqrt(2)));
        y = Math.round(0.5 * Math.sqrt(2 - Math.pow(pos.x, 2) + Math.pow(pos.y, 2) + 2 * pos.y * Math.sqrt(2)) - 0.5 * Math.sqrt(2 - Math.pow(pos.x, 2) + Math.pow(pos.y, 2) - 2 * pos.y * Math.sqrt(2)));
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

    let canvas = document.getElementById("opencv");
    canvas.width  = canvas.offsetWidth;
    canvas.height = canvas.offsetHeight;


    requestAnimationFrame(draw);
})();
