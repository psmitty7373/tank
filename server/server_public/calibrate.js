let anchors = [
    {id: 'a', x: 0, y: 0, z: 0 },
    {id: 'b', x: 0, y: 200, z: 0},
    {id: 'c', x: 200, y: 200, z: 0},
    {id: 'd', x: 200, y: 0, z: 0}
];
let pos_log = [];
let FPS = 1000/60;

let state = 'top-left';
let calibration_data = {tl: [], tr: [], bl: [], br: []};
let base_image = new Image();

let ws = null;

function get_center() {
    c_x = 0;
    c_y = 0;
    for (i = 0; i < anchors.length; i++) {
        c_x += anchors[i].x;
        c_y += anchors[i].y;
    }
    c_x = c_x / anchors.length;
    c_y = c_y / anchors.length;
    return { c_x: c_x, c_y: c_y };
}

function draw(ts) {
    if (!start_time)
        start_time = ts;

    var time_diff = last_time ? ts - last_time : FPS;
    var time_elapsed = ts - start_time;
    var time_scale = time_diff / FPS; // adjust variations in frame rates

    last_time = ts;

    var canvas = document.getElementById("canvas");
    var ctx = canvas.getContext("2d");
    canvas.width  = 2048;
    canvas.height = 768 * 2;
    
    // flip canvas vertically
    ctx.transform(1, 0, 0, -1, 0, canvas.height)

    // clear
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // center canvas
    center = get_center();
    ctx.translate(canvas.width/2 - center.c_x * 2, canvas.height/2 - center.c_y * 2);

    // zoom
    ctx.scale(2, 2);

    // draw anchors
    for (i = 0; i < anchors.length; i++) {
        if (i == 0)
            ctx.fillStyle = "green";
        else if (i == 3)
            ctx.fillStyle = "purple";
        else
            ctx.fillStyle = "blue";
        ctx.fillRect(anchors[i].x - 2, anchors[i].y - 2, 4, 4);
    }

    // draw tanks
    for (var t in pos_log) {
        last_pos = null;
        var i = 0;
        for (; i < pos_log[t].length-1; i++) {
            if (last_pos && (last_pos.x != pos_log[t][i].x || last_pos.y != pos_log[t][i].y)) {
                ctx.lineWidth = 1;
                ctx.beginPath();
                ctx.moveTo(last_pos.x, last_pos.y);
                ctx.lineTo(pos_log[t][i].x, pos_log[t][i].y);
                ctx.stroke();
            }
            last_pos = pos_log[t][i];
        }
        ctx.fillStyle = "red";
        ctx.fillRect(pos_log[t][i].x - 2, pos_log[t][i].y - 2, 4, 4);
    }

    requestAnimationFrame(draw);
}

function translate(value, leftMin, leftMax, rightMin, rightMax) {
    return (value - leftMin) * (rightMax - rightMin) / (leftMax - leftMin) + rightMin;
}

function calibrate(x = null, y = null) {
    var canvas = document.getElementById("canvas");
    var ctx = canvas.getContext("2d");
    canvas.width = 640;
    canvas.height = 480;
    ctx.drawImage(base_image, 0, 0);
    if (state != null) { 
        ctx.font = "10px Arial";
        ctx.fillStyle = "red";
        if (state == 'complete') {
           ctx.fillText("Calibration complete.", 250, 240);
        } else if (x == null && y == null) {
            ctx.fillText("Please click the " + state + " corner of your arena.", 250, 240);
        } else {
            if (state == 'top-left') {
                calibration_data.tl = [x,y];
                state = 'top-right';
            } else if (state == 'top-right') {
                calibration_data.tr = [x,y];
                state = 'bottom-left';
            } else if (state == 'bottom-left') {
                calibration_data.bl = [x,y];
                state = 'bottom-right';
            } else {
                calibration_data.br = [x,y];
                state = 'complete';
                ws.send(JSON.stringify({'t': 'calibrate', 'corners': calibration_data}));
            }
            calibrate();
        }
    }
}

function connect() {
    websocket_url = "ws://" + location.hostname + ':8000/ws';

    ws = new WebSocket(websocket_url);

    ws.onclose = function() {
        setTimeout(function() {
            connect();
        }, 500);
    }

    ws.onopen = function(event) {
        ws.send(JSON.stringify({'t': 'server'}));
        ws.send(JSON.stringify({'t': 'get_cam'}));
    };

    ws.onmessage = function(event) {
        msg = JSON.parse(event.data);
        if (msg['t']) {
            if (msg['t'] == 'cam') {
                base_image.src = "data:image/png;base64," + msg['image'];
                setTimeout(function() {
                    calibrate();
                }, 500);
            }
        }
    };
}

(function() {
    start_time = last_time = null;

    var canvas = document.getElementById("canvas");
    canvas.addEventListener('click', function(event) {
        console.log(event);
        var x = translate(event.x, 0, event.srcElement.clientWidth, 0, 640);
        var y = translate(event.y, 0, event.srcElement.clientHeight, 0, 480);
        calibrate(x, y);
        console.log('blark',x,y);
    }, false);

    connect();
//    requestAnimationFrame(draw);
})();
