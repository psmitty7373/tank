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
let walls = [];

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
let c_gauge = null;
let v_gauge = null;

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
    //ctx.transform(1, 0, 0, -1, 0, canvas.height);
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
        if (i == 0) {
            ctx.fillStyle = "red";
            ctx.fillRect(last_pos.x - 2.5, last_pos.y - 2.5, 5, 5);
        }
    }

    // draw anchors
    ctx.strokeStyle = "black";
    for (i = 0; i < anchors.length; i++) {
        ctx.beginPath();
        ctx.moveTo(anchors[i].x, anchors[i].y);
        ctx.lineTo(anchors[(i + 1) % anchors.length].x, anchors[(i + 1) % anchors.length].y);
        ctx.stroke();
    }

    // draw walls
    ctx.strokeStyle = "#333333";
    for (i = 0; i < walls.length; i++) {
        ctx.lineWidth = 0.5;
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(walls[i].from.x, walls[i].from.y);
        ctx.lineTo(walls[i].to.x, walls[i].to.y);
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
        // verify msg
        if (msg['t'] == 'status') {
            if (Object.keys(msg).some(r=>['a_x', 'a_y', 'a_z', 'l_t', 'r_t', 'volts', 'current', 'x', 'y'].includes(r))) {
                // heading
                tank_status.a_x = Math.round(msg.a_x);
                tank_status.a_y = Math.round(msg.a_y);
                tank_status.a_z = Math.round(msg.a_z);
                if (tank_status.a_x == 360)
                    tank_status.a_x = 0;

                // throttle
                lt_gauge.set(Math.round(msg.l_t));
                rt_gauge.set(Math.round(msg.r_t));

                // current
                document.getElementById("voltage-value").innerHTML = msg.volts + 'v';
                document.getElementById("current-value").innerHTML = msg.current + 'mah';

                v_gauge.set(msg.volts);
                c_gauge.set(msg.current);

                // heading
                document.getElementById("heading").innerHTML = tank_status.a_x.toString(10).padStart(3,'0');

                // update position log
                pos_log.unshift({x: msg.x, y: msg.y});
                pos_log = pos_log.splice(0,50);
            }
        } else if (msg['t'] == 'arena_config') {
            if (msg['t'] == 'arena_config') {
                if (msg['config'] && msg['config']['walls'])
                    walls = msg['config']['walls'];
            }
        }
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
var throttle_gauge_opts = {
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

var voltage_gauge_opts = {
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
	   {strokeStyle: "#ff0000", min: 5, max: 6.6},
	   {strokeStyle: "#008000", min: 6.6, max: 8},
	   {strokeStyle: "#ff0000", min: 8, max: 10}
	],
	highDpiSupport: true,     // High resolution support
};

var current_gauge_opts = {
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
	   {strokeStyle: "#ff0000", min: 0, max: 100},
	   {strokeStyle: "#008000", min: 100, max: 1200},
	   {strokeStyle: "#ff0000", min: 1200, max: 1600}
	],
	highDpiSupport: true,     // High resolution support
};

// START REMOTE VIDEO
var websocketSignalingChannel = new WebSocketSignalingChannel(document.getElementById("remoteVideo"));

// DOCUMENT READY
(function() {
    connect();
    websocketSignalingChannel.doSignalingConnect()

    let target = document.getElementById('left-throttle');
    lt_gauge = new Gauge(target).setOptions(throttle_gauge_opts);
    lt_gauge.maxValue = 255;
    lt_gauge.setMinValue(-255);
    lt_gauge.animationSpeed = 32;
    lt_gauge.set(0);

    target = document.getElementById('right-throttle');
    rt_gauge = new Gauge(target).setOptions(throttle_gauge_opts);
    rt_gauge.maxValue = 255;
    rt_gauge.setMinValue(-255);
    rt_gauge.animationSpeed = 32;
    rt_gauge.set(0);

    target = document.getElementById('voltage');
    v_gauge = new Gauge(target).setOptions(voltage_gauge_opts);
    v_gauge.maxValue = 10.0;
    v_gauge.setMinValue(5.0);
    v_gauge.animationSpeed = 32;
    v_gauge.set(5);

    target = document.getElementById('current');
    c_gauge = new Gauge(target).setOptions(current_gauge_opts);
    c_gauge.maxValue = 1600.0;
    c_gauge.setMinValue(0.0);
    c_gauge.animationSpeed = 32;
    c_gauge.set(0);

    requestAnimationFrame(update);

    document.addEventListener('keydown', function(event) {
        if(event.keyCode == 32) {
            fire_weapon_wrapper();
        }
    });
})();
