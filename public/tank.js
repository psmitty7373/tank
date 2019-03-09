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
let weapon_ready = true;


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
        else {
            let azim = Math.round(msg.a_x);
            if (azim == 360)
                azim = 0;
            document.getElementById("heading").innerHTML = azim.toString(10).padStart(3,'0');
        }
        
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
        pos.t = 't';
        ws.send(JSON.stringify(pos));
    }
}

function fireWeaponWrapper() {
    if (weapon_ready) {
        weapon_ready = false;
        fireWeapon(10);
    }
}

function fireWeapon(times) {
    if (ws.readyState == 1) {
        msg = { t: 'f' };
        ws.send(JSON.stringify(msg));
    }
    times--;
    if (times > 0)
        setTimeout(function() { fireWeapon(times); }, 50);
    else
        weapon_ready = true;
}

setInterval(function() {
    sendJoystick(joystick);
}, 50);

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

    document.addEventListener('keydown', function(event) {
        console.log(event.keyCode);
        if(event.keyCode == 32) {
            fireWeaponWrapper();
        }
    });
})();
