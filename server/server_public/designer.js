let arena_width = 200;
let arena_height = 200;
let arena_grid_factor = 5;
let state = null;
let first_point = null;
let ws = null;
let available_games = { 'Generic': { 'map_features': [] } };
let selected_gid = 0;
let selected_oid = [0, 0];
let selected_type = 'line';

let anchors = [
    {id: 'a', x: 0, y: 0, z: 0 },
    {id: 'b', x: 0, y: 200, z: 0},
    {id: 'c', x: 200, y: 200, z: 0},
    {id: 'd', x: 200, y: 0, z: 0}
];
let pos_log = [];
let FPS = 1000/60;
let map_features = [];

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

function count_oid(gid, oid, max_count) {
    count = 0;
    for (var i = map_features.length - 1; i >= 0; i--) {
        if (map_features[i]['gid'] == gid && map_features[i]['oid'] == oid) {
            if (count + 1 >= max_count) {
                map_features.splice(i, 1);
            }
            else
                count += 1;
        }
    }
    console.log(count);
}

function handle_mouse_click(evt) {
    // walls
    if (selected_oid[0] == 0 && selected_oid[1] == 0) {
        if (state == null) {
            first_point = { x: Math.round(cursor_pos.x / 5) * 5, y: Math.round(cursor_pos.y / 5) * 5 };
            state = 'first';
        } else if (state == 'first') {
            map_features.push({oid: 0, gid:0, type: 'line', from: first_point, to: {x: Math.round(cursor_pos.x / 5) * 5, y: Math.round(cursor_pos.y / 5) * 5 }});
            state = null;
            first_point = null;
        }
    // other objects
    } else {
        // ensure max counts are followed
        if (available_games[selected_gid]['map_features'][selected_oid[1]]['max_count'] > 0) {
            count_oid(selected_oid[0], selected_oid[1], available_games[selected_gid]['map_features'][selected_oid[1]]['max_count']);
        }
        map_features.push({oid: selected_oid[1], gid: selected_oid[0], type: available_games[selected_gid]['map_features'][selected_oid[1]]['type'], pos: {x: Math.round(cursor_pos.x / 5) * 5, y: Math.round(cursor_pos.y / 5) * 5 }});
    }
}

function save_map() {
    if (ws) {
        ws.send(JSON.stringify({'t': 'map_update', 'map_features': JSON.stringify(map_features) }));
    }
}

function clear_map() {
    map_features = [];
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

    // draw map_features
    ctx.strokeStyle = "#333333";
    for (i = 0; i < map_features.length; i++) {
        if (map_features[i]['gid'] == 0 || map_features[i]['gid'] == selected_gid) {
            if (map_features[i]['type'] == 'line') {
                ctx.lineWidth = 0.5;
                ctx.lineWidth = 1;
                ctx.beginPath();
                ctx.moveTo(map_features[i].from.x, map_features[i].from.y);
                ctx.lineTo(map_features[i].to.x, map_features[i].to.y);
                ctx.stroke();
            } else if (map_features[i]['type'] == 'point') {
                ctx.fillStyle = "green";
                ctx.fillRect(map_features[i]['pos']['x'] - 2, map_features[i]['pos']['y'] - 2, 4, 4);
            }
        }
    }

    // draw grid
    for (var x = 0; x <= arena_width; x += 5) {
        for (var y = 0; y <= arena_height; y += 5) {
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


function change_gid(type) {
    selected_gid = type;
    change_oid(0, 0);
    $('#selected_gid').text(available_games[type]['name']);
    $('#oids').empty()
    $('#oids').append('<a class="dropdown-item" href="#" onclick="change_oid(0, 0);return false;">Wall</a>');
    Object.keys(available_games[type]['map_features']).forEach(key => {
        $('#oids').append('<a class="dropdown-item" href="#" onclick="change_oid(' + type + ',' + key + ');return false;">' + available_games[type]['map_features'][key]['name'] + '</a>');
    });
}


function change_oid(gid, oid) {
    selected_oid = [gid, oid];
    if (gid == 0 && oid == 0) {
        $('#selected_oid').text('Wall');
        selected_type = 'line';
    } else {
        $('#selected_oid').text(available_games[selected_gid]['map_features'][oid]['name']);
        selected_type = available_games[selected_gid]['map_features'][oid]['type'];
        state = null;
        first_point = null;
    }
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
            if (msg['t'] == 'game_config') {
                if (msg['config'] && msg['config']['map_features'])
                    map_features = msg['config']['map_features'];
            }
            else if (msg['t'] == 'available_games') {
                available_games = msg['available_games'];
                $('#gids').empty()
                Object.keys(msg['available_games']).forEach(key => {
                    $('#gids').append('<a class="dropdown-item" href="#" onclick="change_gid(' + key + ');return false;">' + msg['available_games'][key]['name'] + '</a>');
                });
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
    $(document).ready(function() {
        $(".dropdown-toggle").dropdown();
    });
})();
