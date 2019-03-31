function isPrivateAddress(ipaddress) {
    let parts = ipaddress.split('.');
    return parts[0] === '10' ||
            (parts[0] === '172' && (parseInt(parts[1], 10) >= 16 && parseInt(parts[1], 10) <= 31)) ||
            (parts[0] === '192' && parts[1] === '168');
};

// GLOBALS
let anchors = [
        {id: 'a', x: 0, y: 0, z: 0 },
        {id: 'b', x: 0, y: 200, z: 0},
        {id: 'c', x: 200, y: 200, z: 0},
        {id: 'd', x: 200, y: 0, z: 0}
];

let pos_log = [];
let joystick = { x: 0, y: 0 };
let tank_status = { a_x: 0, a_y: 0, a_z: 0 };
let FPS = 1000/60;
let CPS = 1000/20;
let last_frame = 0;
let last_control = 0;
let time_diff_frame = 0;
let time_diff_control = 0;
let video = document.getElementById('remoteVideo');
let reticle_color = "green";
let weapon_ready = true;

// GAUGES
let lt_gauge = null;
let rt_gauge = null;

// OPENCV
function get_topdown_quad(points, src) {
    let dest = new cv.Mat();
    let corner1 = new cv.Point(points.data32S[0], points.data32S[1]);
    let corner2 = new cv.Point(points.data32S[2], points.data32S[3]);
    let corner3 = new cv.Point(points.data32S[4], points.data32S[5]);
    let corner4 = new cv.Point(points.data32S[6], points.data32S[7]);

    let corners = [{ corner: corner1 }, { corner: corner2 }, { corner: corner3 }, { corner: corner4 }];
    corners.sort((item1, item2) => { return (item1.corner.y < item2.corner.y) ? -1 : (item1.corner.y > item2.corner.y) ? 1 : 0; }).slice(0, 5);

    let tl = corners[0].corner.x < corners[1].corner.x ? corners[0] : corners[1];
    let tr = corners[0].corner.x > corners[1].corner.x ? corners[0] : corners[1];
    let bl = corners[2].corner.x < corners[3].corner.x ? corners[2] : corners[3];
    let br = corners[2].corner.x > corners[3].corner.x ? corners[2] : corners[3];

    let widthBottom = Math.hypot(br.corner.x - bl.corner.x, br.corner.y - bl.corner.y);
    let widthTop = Math.hypot(tr.corner.x - tl.corner.x, tr.corner.y - tl.corner.y);
    let theWidth = (widthBottom > widthTop) ? widthBottom : widthTop;
    let heightRight = Math.hypot(tr.corner.x - br.corner.x, tr.corner.y - br.corner.y);
    let heightLeft = Math.hypot(tl.corner.x - bl.corner.x, tr.corner.y - bl.corner.y);
    let theHeight = (heightRight > heightLeft) ? heightRight : heightLeft;

    let dest_coords = cv.matFromArray(4, 1, cv.CV_32FC2, [0, 0, theWidth - 1, 0, theWidth - 1, theHeight - 1, 0, theHeight - 1]);
    let src_coords = cv.matFromArray(4, 1, cv.CV_32FC2, [tl.corner.x, tl.corner.y, tr.corner.x, tr.corner.y, br.corner.x, br.corner.y, bl.corner.x, bl.corner.y]);
    let dsize = new cv.Size(theWidth, theHeight);
    let M = cv.getPerspectiveTransform(src_coords, dest_coords)
    cv.warpPerspective(src, dest, M, dsize, cv.INTER_LINEAR, cv.BORDER_CONSTANT, new cv.Scalar());
    let r = new cv.Rect();
    let px = dest.ucharPtr(5,5);
    let glyph = new Array(25).fill(0);
    if (px[0] < 128 && px[1] < 128 && px[2] < 128) {
        let rect = new cv.Rect();
        for (let y = 0; y < 5; y++) {
            for (let x = 0; x < 5; x++) {
                let block = new cv.Mat();
                rect.x = x * Math.floor(dest.cols / 5);
                rect.width = Math.floor(dest.cols / 5);
                rect.y = y * Math.floor(dest.rows / 5);
                rect.height = Math.floor(dest.rows / 5);
                block = dest.roi(rect);
                let mean = cv.mean(block);
                let color = new cv.Scalar(255, 30, 30, 255);
                if (mean[0] < 128 && mean[1] < 128 && mean[2] < 128) {
                    glyph[(y * 5) + x] = 1;
                    color[0] = 30;
                    color[1] = 255;
                }
                cv.rectangle(dest, new cv.Point(rect.x, rect.y), new cv.Point(rect.x + rect.width, rect.y + rect.height), color, 1)
                block.delete();
            }
        }
    }
    if (glyph.toString() == "1,1,1,1,1,1,1,0,1,1,1,0,1,1,1,1,1,0,0,1,1,1,1,1,1")
        cv.imshow('opencv', dest);
    dest.delete();
}

cv['onRuntimeInitialized']=()=>{
    video.width = video.offsetWidth;
    video.height = video.offsetHeight;
    let cap = new cv.VideoCapture(video);

    let src = new cv.Mat(video.height, video.width, cv.CV_8UC4);
    let sm = new cv.Mat();
    let gray = new cv.Mat();
    let edges = new cv.Mat();
    let contours = new cv.MatVector();
    let hierarchy = new cv.Mat();
    let color = new cv.Scalar(255, 30, 30, 255);
    let ksize = new cv.Size(5,5);

    function processFrame() {
        cap.read(src);
        cv.resize(src, sm, new cv.Size(1024, 768), 0, 0, cv.INTER_AREA);
        cv.cvtColor(sm, gray, cv.COLOR_BGR2GRAY);
        cv.GaussianBlur(gray, gray, ksize, 0, 0, cv.BORDER_DEFAULT);
        cv.Canny(gray, edges, 100, 200, 3, true);
        cv.findContours(edges, contours, hierarchy, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE);
//        cv.drawContours(sm, contours, -1, color, 1, cv.LINE_8, hierarchy, 100);

        for (let i = 0; i < contours.size(); i++) {
            let cnt = contours.get(i);
            let approx = new cv.Mat();
            let perimeter = cv.arcLength(cnt, false);
            cv.approxPolyDP(cnt, approx, 0.05 * perimeter, true);
            if (approx.rows == 4) {
                get_topdown_quad(approx, sm);
            }

            approx.delete();
//            cv.rectangle(src, new cv.Point(br.x, br.y), new cv.Point(br.x + br.width, br.y + br.height), color, 1);
        }


    //    cv.imshow('opencv', src);
//        setTimeout(processFrame, 100);
    }
//    setTimeout(processFrame, 0);
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

// RETICLE
function draw_reticle() {
    // reticle
    let canvas = document.getElementById("hud_canvas");
    canvas.width = canvas.offsetWidth;
    canvas.height = canvas.offsetHeight;
    ctx = canvas.getContext("2d");
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.beginPath();
    if (reticle_color == "green")
        ctx.strokeStyle = "#00FF00";
    else
        ctx.strokeStyle = "#FF0000";
    ctx.lineWidth = 3;
    ctx.arc(canvas.width/2, canvas.height/2, 10, 0, 2 * Math.PI);
    ctx.stroke();
    // right line
    ctx.beginPath();
    ctx.moveTo(canvas.width/2 + 10, canvas.height/2);
    ctx.lineTo(canvas.width/2 + 65, canvas.height/2);
    ctx.stroke();
    // left line
    ctx.beginPath();
    ctx.moveTo(canvas.width/2 - 10, canvas.height/2);
    ctx.lineTo(canvas.width/2 - 65, canvas.height/2);
    ctx.stroke();
    // top
    ctx.beginPath();
    ctx.moveTo(canvas.width/2, canvas.height/2 + 10);
    ctx.lineTo(canvas.width/2, canvas.height/2 + 25);
    ctx.stroke();
    // bottom
    ctx.beginPath();
    ctx.moveTo(canvas.width/2, canvas.height/2 - 10);
    ctx.lineTo(canvas.width/2, canvas.height/2 - 25);
    ctx.stroke();
}

function draw_attitude() {
    let canvas = document.getElementById("attitude");
    let ctx = canvas.getContext("2d");
    canvas.width  = canvas.offsetWidth;
    canvas.height = canvas.offsetWidth;

    let center = get_center();
    
    // clear
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // normalize pitch and roll
    let roll = 0;
    let pitch = tank_status.a_z
    if (Math.abs(pitch) < 90)
        roll = tank_status.a_y;
    else {
        if (pitch > 0)
            pitch = pitch % 90 - 90;
        else
            pitch = pitch % 90 + 90;
        if (tank_status.a_y > 0)
            roll = 180 - tank_status.a_y;
        else
            roll = -180 - tank_status.a_y;
    }

    // center / pitch / roll
    ctx.translate(0, pitch);
    ctx.translate(canvas.width/2, canvas.height/2);
    ctx.rotate(roll * Math.PI / 180);

    // draw sky / ground
    ctx.fillStyle = "rgba(121, 85, 72, 1)";
    ctx.fillRect(-canvas.width * 2, 0, canvas.width * 4, canvas.height * 4);
    ctx.fillStyle = "rgba(33, 150, 243, 1)";
    ctx.fillRect(-canvas.width * 2, 0, canvas.width * 4, -canvas.height * 4);

    // draw horizon line
    ctx.strokeStyle = "rgba(0, 0, 0, 1)";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(-canvas.width * 2, 0);
    ctx.lineTo(canvas.width * 2, 0);
    ctx.stroke();

    // crop to circle
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.translate(canvas.width/2, canvas.height/2);
    ctx.globalCompositeOperation='destination-in';
    ctx.beginPath()
    ctx.arc(0, 0, canvas.width / 2 - 4, 0, 2 * Math.PI);
    ctx.closePath()
    ctx.fill();

}

// MINIMAP
function draw_minimap() {
    // position & minimap
    let canvas = document.getElementById("minimap");
    let ctx = canvas.getContext("2d");
    canvas.width  = canvas.offsetWidth;
    canvas.height = canvas.offsetHeight;

    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.transform(1, 0, 0, -1, 0, canvas.height);
    let center = get_center();

    // draw minimap circle
    ctx.fillStyle = "rgba(0, 128, 0, 1)";
    ctx.beginPath();
    ctx.arc(canvas.width/2 + 1, canvas.height/2 + 1, canvas.width / 2 - 4, 0, 2 * Math.PI);
    ctx.fill();

    // center map
    ctx.translate(canvas.width/2 - center.c_x, canvas.height/2 - center.c_y);

    // draw points
    let last_pos = null;
    for (let i = 0; i < pos_log.length; i++) {
        if (last_pos && (last_pos.x != pos_log[i].x || last_pos.y != pos_log[i].y)) {
            ctx.lineWidth = 1;
            ctx.beginPath();
            ctx.moveTo(last_pos.x, last_pos.y);
            ctx.lineTo(pos_log[i].x, pos_log[i].y);
            ctx.stroke();
        }
        last_pos = pos_log[i];
    }
    if (last_pos) {
        ctx.fillStyle = "red";
        ctx.fillRect(last_pos.x, last_pos.y, 4, 4);
    }

    // draw anchors
    ctx.strokeStyle = "black";
    for (i = 0; i < anchors.length; i++) {
        ctx.beginPath();
        ctx.moveTo(anchors[i].x, anchors[i].y);
        ctx.lineTo(anchors[(i + 1) % anchors.length].x, anchors[(i + 1) % anchors.length].y);
        ctx.stroke();
    }

    for (i = 0; i < anchors.length; i++) {
        if (i == 0)
            ctx.fillStyle = "green";
        else if (i == 3)
            ctx.fillStyle = "purple";
        else
            ctx.fillStyle = "blue";
        ctx.fillRect(anchors[i].x - 2, anchors[i].y - 2, 4, 4);
    }

}

function update(ts) {
    requestAnimationFrame(update);

    time_diff_frame = ts - last_frame;
    time_diff_control = ts - last_control;
    update_gamepads();

    if (time_diff_frame > FPS) {
        draw_reticle();
        draw_minimap();
        draw_attitude();
        last_frame = ts;
    }

    if (time_diff_control > CPS) {
        last_control = ts;
        send_joystick();
    }

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
        else {
            tank_status.a_x = Math.round(msg.a_x);
            tank_status.a_y = Math.round(msg.a_y);
            tank_status.a_z = Math.round(msg.a_z);
            if (tank_status.a_x == 360)
                tank_status.a_x = 0;
            lt_gauge.set(Math.round(msg.l_t));
            rt_gauge.set(Math.round(msg.r_t));
            document.getElementById("heading").innerHTML = tank_status.a_x.toString(10).padStart(3,'0');
        }
        
        // update position log
        pos_log.unshift({x: msg.x, y: msg.y});
        pos_log = pos_log.splice(0,50);

    };
}

let virt_joystick = new VirtualJoystick({
    container : document.getElementById('interface'),
    mouseSupport : true,
    limitStickTravel : true,
    stickRadius: 100
});

virt_joystick.addEventListener('touchStart', function(){
    console.log('down')
});

virt_joystick.addEventListener('touchEnd', function(){
    console.log('up')
});

function send_joystick() {
    if (ws.readyState == 1) {
        pos = { x: virt_joystick.deltaX(), y: -virt_joystick.deltaY() };
        pos.x = pos.x / 100;
        pos.y = pos.y / 100;

        joystick.x = pos.x;
        joystick.y = pos.y;

        let msg = { t: 't', x: joystick.x, y: joystick.y };
        ws.send(JSON.stringify(msg));
    }
}

function fire_weapon_wrapper() {
    if (weapon_ready) {
        weapon_ready = false;
        fire_weapon(10);
    }
}

function fire_weapon(times) {
    if (ws.readyState == 1) {
        msg = { t: 'f' };
        ws.send(JSON.stringify(msg));
    }
    times--;
    if (times > 0)
        setTimeout(function() { fire_weapon(times); }, 50);
    else
        weapon_ready = true;
}

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

// GAMEPADS
let haveEvents = 'ongamepadconnected' in window;
let controllers = {};

function connect_gamepad(e) {
    add_gamepad(e.gamepad);
}

function add_gamepad(gamepad) {
    controllers[gamepad.index] = gamepad;
}

function disconnect_gamepad(e) {
    remove_gamepad(e.gamepad);
}

function remove_gamepad(gamepad) {
    let d = document.getElementById("controller" + gamepad.index);
    document.body.removeChild(d);
    delete controllers[gamepad.index];
}

function update_gamepads() {
    if (!haveEvents) {
        scan_gamepads();
    }

    let i = 0;
    let j;

    for (j in controllers) {
        let controller = controllers[j];
        for (i = 0; i < controller.buttons.length; i++) {
            let val = controller.buttons[i];
            if (typeof(val) == "object") {
            }
        }
        if (ws.readyState == 1 && controller.axes.length > 2) {
            if (Math.abs(controller.axes[0]) > 0.2 || Math.abs(controller.axes[1]) > 0.2) {
                joystick.x = controller.axes[0];
                joystick.y = -controller.axes[1];
            }
        }
    }
}

function scan_gamepads() {
    let gamepads = navigator.getGamepads ? navigator.getGamepads() : (navigator.webkitGetGamepads ? navigator.webkitGetGamepads() : []);
    for (let i = 0; i < gamepads.length; i++) {
        if (gamepads[i]) {
            if (gamepads[i].index in controllers) {
                controllers[gamepads[i].index] = gamepads[i];
            } else {
                add_gamepad(gamepads[i]);
            }
        }
    }
}


window.addEventListener("gamepadconnected", connect_gamepad);
window.addEventListener("gamepaddisconnected", disconnect_gamepad);

if (!haveEvents) {
  setInterval(scan_gamepads, 500);
}


// GAUGES:
var gauge_opts = {
	angle: 0.15, // The span of the gauge arc
	lineWidth: 0.44, // The line thickness
	radiusScale: 1, // Relative radius
	pointer: {
		length: 0.6, // // Relative to gauge radius
		strokeWidth: 0.035, // The thickness
		color: '#000000'
	},
	limitMax: false,
	limitMin: false,
	generateGradient: true,
	staticZones: [
	   {strokeStyle: "#ff0000", min: -255, max: -200},
	   {strokeStyle: "#008000", min: -200, max: -10},
	   {strokeStyle: "#ffffff", min: -10, max: 10},
	   {strokeStyle: "#008000", min: 10, max: 200},
	   {strokeStyle: "#ff0000", min: 200, max: 255}
	],
	highDpiSupport: true,     // High resolution support
};

// START REMOTE VIDEO
var websocketSignalingChannel = new WebSocketSignalingChannel(document.getElementById("remoteVideo"));

// DOCUMENT READY
(function() {
    connect();
    websocketSignalingChannel.doSignalingConnect()

    let canvas = document.getElementById("opencv");
    canvas.width  = canvas.offsetWidth;
    canvas.height = canvas.offsetHeight;

    let target = document.getElementById('left-throttle');
    lt_gauge = new Gauge(target).setOptions(gauge_opts);
    lt_gauge.maxValue = 255;
    lt_gauge.setMinValue(-255);
    lt_gauge.animationSpeed = 32;
    lt_gauge.set(0);

    target = document.getElementById('right-throttle');
    rt_gauge = new Gauge(target).setOptions(gauge_opts);
    rt_gauge.maxValue = 255;
    rt_gauge.setMinValue(-255);
    rt_gauge.animationSpeed = 32;
    rt_gauge.set(0);

    requestAnimationFrame(update);

    document.addEventListener('keydown', function(event) {
        if(event.keyCode == 32) {
            fire_weapon_wrapper();
        }
    });
})();
