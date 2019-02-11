function isPrivateAddress(ipaddress) {
    var parts = ipaddress.split('.');
    return parts[0] === '10' ||
            (parts[0] === '172' && (parseInt(parts[1], 10) >= 16 && parseInt(parts[1], 10) <= 31)) ||
            (parts[0] === '192' && parts[1] === '168');
};

anchors = [
    {id: 'a', x: 0, y: 0, z: 0 },
    {id: 'b', x: 0, y: 200, z: 0},
    {id: 'c', x: 200, y: 200, z: 0},
    {id: 'd', x: 200, y: 0, z: 0}
];
pos_log = [];
FPS = 1000/60;

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
    var context = canvas.getContext("2d");
    canvas.width  = canvas.offsetWidth;
    canvas.height = canvas.offsetHeight;
    
    // flip canvas vertically
    context.transform(1, 0, 0, -1, 0, canvas.height)

    // clear
    context.clearRect(0, 0, canvas.width, canvas.height);

    // center canvas
    center = get_center();
    context.translate(canvas.width/2 - center.c_x * 2, canvas.height/2 - center.c_y * 2);

    // zoom
    context.scale(2, 2);

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

    // draw tanks
    for (var t in pos_log) {
        last_pos = null;
        var i = 0;
        for (; i < pos_log[t].length-1; i++) {
            if (last_pos && (last_pos.x != pos_log[t][i].x || last_pos.y != pos_log[t][i].y)) {
                context.lineWidth = 1;
                context.beginPath();
                context.moveTo(last_pos.x, last_pos.y);
                context.lineTo(pos_log[t][i].x, pos_log[t][i].y);
                context.stroke();
            }
            last_pos = pos_log[t][i];
        }
        context.fillStyle = "red";
        context.fillRect(pos_log[t][i].x - 2, pos_log[t][i].y - 2, 4, 4);
    }

    requestAnimationFrame(draw);
}

function connect() {
    if(isPrivateAddress(location.host))
            websocket_url = "ws://" + location.hostname + ':8000/ws';
    else
            websocket_url = "wss://" + location.hostname + '/ws';

    ws = new WebSocket(websocket_url);
    ws.onclose = function() {
        setTimeout(function() {
            connect();
        }, 500);
    }
    ws.onmessage = function(event) {
        msg = JSON.parse(event.data);
        if (msg.id && msg.x && msg.y && msg.z) {
            if (pos_log[msg.id] == undefined)
                pos_log[msg.id] = [];
            if (pos_log[msg.id].length >= 50)
                pos_log[msg.id] = pos_log[msg.id].slice(1);
            pos_log[msg.id].push({x: msg.x, y: msg.y});
        }
    };
}

(function() {
    start_time = last_time = null;
    connect();
    requestAnimationFrame(draw);
})();
