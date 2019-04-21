let arena_width = 200;
let arena_height = 200;
let arena_grid_factor = 5;
let state = null;
let first_point = null;
let ws = null;

let anchors = [
    {id: 'a', x: 0, y: 0, z: 0 },
    {id: 'b', x: 0, y: 200, z: 0},
    {id: 'c', x: 200, y: 200, z: 0},
    {id: 'd', x: 200, y: 0, z: 0}
];
let pos_log = [];
let FPS = 1000/60;
let walls = [];

let cursor_pos = { x: -1, y: -1 };

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

function handle_mouse_move(evt) {
    var canvas = evt.target;
    var rect = canvas.getBoundingClientRect();
    cursor_pos = {
        x: (evt.clientX - rect.left) / (rect.right - rect.left) * arena_width,
        y: (evt.clientY - rect.top) / (rect.bottom - rect.top) * arena_height
    }
}

function handle_mouse_click(evt) {
    if (state == null) {
        first_point = { x: Math.round(cursor_pos.x / 10) * 10, y: Math.round(cursor_pos.y / 10) * 10 };
        state = 'first';
    } else if (state == 'first') {
        walls.push({from: first_point, to: {x: Math.round(cursor_pos.x / 10) * 10, y: Math.round(cursor_pos.y / 10) * 10 }});
        state = null;
        first_point = null;
    }
}

function save() {
    if (ws) {
        ws.send(JSON.stringify({'t': 'wall_update', 'wall_data': JSON.stringify(walls) }));
    }
}

function clear_walls() {
    walls = [];
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
    canvas.width  = 400;
    canvas.height = 400;
    
    // flip canvas vertically
    //ctx.transform(1, 0, 0, -1, 0, canvas.height)

    // clear
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // center canvas
    center = get_center();
    ctx.translate(canvas.width/2 - center.c_x * 2, canvas.height/2 - center.c_y * 2);

    // zoom
    ctx.scale(2, 2);

    // draw grid
    ctx.strokeStyle = "#c1c1c1";
    for (var x = 0; x < arena_width; x += arena_width / arena_grid_factor) {
        ctx.lineWidth = 0.5;
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, arena_height);
        ctx.stroke();

        ctx.lineWidth = 0.5;
        ctx.beginPath();
        ctx.moveTo(0, x);
        ctx.lineTo(arena_width, x);
        ctx.stroke();
    }

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

    // draw grid
    for (var x = 0; x <= arena_width; x += 10) {
        for (var y = 0; y <= arena_height; y += 10) {
            if (cursor_pos.x < x + 2 && cursor_pos.x > x - 2 && cursor_pos.y < y + 2 && cursor_pos.y > y - 2) {
                ctx.fillStyle = "red";
            } else if (first_point && x == first_point.x && y == first_point.y) {
                ctx.fillStyle = "green";
            } else
                ctx.fillStyle = "gray";
            ctx.fillRect(x - 0.5, y - 0.5, 1, 1);
        }
    }


    // draw line
    if (state == 'first' && first_point) {
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(first_point.x, first_point.y);
        ctx.lineTo(cursor_pos.x, cursor_pos.y);
        ctx.stroke();
    }

    requestAnimationFrame(draw);
}


function connect() {
    websocket_url = "ws://" + location.hostname + ':8000/ws';
    ws = new WebSocket(websocket_url);

    ws.onclose = function() {
        setTimeout(function() {
            connect();
        }, 1000);
    }

    ws.onopen = function(event) {
        ws.send(JSON.stringify({'t': 'server'}));
    };

    ws.onmessage = function(event) {
        msg = JSON.parse(event.data);
        if (msg['t']) {
            if (msg['t'] == 'arena_config') {
                if (msg['config'] && msg['config']['walls'])
                    walls = msg['config']['walls'];
            }
        }
    };
}


(function() {
    $("#canvas").mousemove(handle_mouse_move);
    $("#canvas").mousedown(handle_mouse_click);
    start_time = last_time = null;
    connect();
    requestAnimationFrame(draw);
})();
