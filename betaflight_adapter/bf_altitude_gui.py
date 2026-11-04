#!/usr/bin/env python3
"""Betaflight MSP altitude monitor. Read-only requests."""

import argparse
import collections
import json
import struct
import threading
import time

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


HTML = r'''
<!doctype html>
<html lang="ru">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>JT-Zero — высота Betaflight</title>
<style>
body {
    background: #101820;
    color: #e9f1f7;
    font: 15px system-ui, sans-serif;
    margin: 0;
    padding: 24px;
}
main { max-width: 1100px; margin: auto; }
h1 { font-size: 25px; }
.stats {
    display: flex;
    gap: 12px;
    flex-wrap: wrap;
    margin: 20px 0;
}
.stat {
    background: #1b2b37;
    border: 1px solid #304352;
    border-radius: 12px;
    padding: 15px;
    min-width: 160px;
    flex: 1;
}
.stat b {
    display: block;
    font-size: 28px;
    margin: 7px 0;
}
.muted { color: #a8bac9; }
canvas {
    display: block;
    width: 100%;
    height: 450px;
    background: #14232e;
    border: 1px solid #304352;
    border-radius: 12px;
}
button {
    background: #2e7096;
    border: 0;
    border-radius: 8px;
    color: white;
    padding: 11px 18px;
    font-weight: 600;
    cursor: pointer;
}
button:hover { background: #3988b4; }
.warn { color: #f3cb77; }
.bar {
    display: flex;
    align-items: center;
    gap: 14px;
    flex-wrap: wrap;
    margin: 16px 0;
}
</style>
</head>
<body>
<main>
<h1>JT-Zero · высота Betaflight</h1>
<p class="muted">
MSP_ALTITUDE (109) · оценённая высота FC ·
опрос до 50 Гц · окно 60 секунд
</p>

<div class="stats">
    <div class="stat">
        Текущая высота
        <b id="height">—</b>
        <span class="muted">м от точки обнуления</span>
    </div>
    <div class="stat">
        Значение FC
        <b id="absolute">—</b>
        <span class="muted">м</span>
    </div>
    <div class="stat">
        Ответы MSP
        <b id="hz">—</b>
        <span class="muted">Гц за последние 2 с</span>
    </div>
    <div class="stat">
        Статус
        <b id="status">Ожидание</b>
        <span id="detail" class="muted">—</span>
    </div>
</div>

<div class="bar">
    <button id="zero">Обнулить высоту</button>
    <button id="clear">Очистить график</button>
    <span class="warn">Только чтение MSP</span>
</div>

<canvas id="chart"></canvas>
<p class="muted">
Ось X — последние 60 секунд.
Ось Y — относительная высота, метры.
</p>
</main>

<script>
const canvas = document.getElementById('chart');
const ctx = canvas.getContext('2d');

let points = [];
let zero = null;
let last = null;
let lastReceived = 0;
let stamp = 0;

function resize() {
    const r = canvas.getBoundingClientRect();
    const d = window.devicePixelRatio || 1;
    canvas.width = Math.round(r.width * d);
    canvas.height = Math.round(r.height * d);
    ctx.setTransform(d, 0, 0, d, 0, 0);
    draw();
}

window.addEventListener('resize', resize);

document.getElementById('zero').onclick = () => {
    if (last !== null) {
        zero = last;
        draw();
    }
};

document.getElementById('clear').onclick = () => {
    points = [];
    draw();
};

function draw() {
    const w = canvas.clientWidth;
    const h = canvas.clientHeight;
    const pad = {l: 58, r: 20, t: 22, b: 35};

    ctx.clearRect(0, 0, w, h);

    const t = performance.now() / 1000;
    points = points.filter(p => t - p.t <= 60);

    const vals = points.map(p => p.h - (zero ?? p.h));

    let min = vals.length ? Math.min(...vals) : -0.5;
    let max = vals.length ? Math.max(...vals) : 0.5;

    min = Math.min(min, -0.1);
    max = Math.max(max, 0.1);

    const extra = Math.max(0.08, (max - min) * 0.12);
    min -= extra;
    max += extra;

    const X = v =>
        pad.l + (w - pad.l - pad.r) * (1 - (t - v) / 60);

    const Y = v =>
        pad.t + (h - pad.t - pad.b) * (max - v) / (max - min);

    ctx.font = '12px system-ui';
    ctx.strokeStyle = '#314657';
    ctx.fillStyle = '#a8bac9';
    ctx.lineWidth = 1;

    for (let i = 0; i <= 5; i++) {
        const v = min + (max - min) * i / 5;
        const y = Y(v);

        ctx.beginPath();
        ctx.moveTo(pad.l, y);
        ctx.lineTo(w - pad.r, y);
        ctx.stroke();
        ctx.fillText(v.toFixed(2), 6, y + 4);
    }

    for (let i = 0; i <= 6; i++) {
        const x = pad.l + (w - pad.l - pad.r) * i / 6;

        ctx.beginPath();
        ctx.moveTo(x, pad.t);
        ctx.lineTo(x, h - pad.b);
        ctx.stroke();

        ctx.fillText((-60 + 10 * i) + ' c', x - 12, h - 12);
    }

    if (!points.length) return;

    ctx.beginPath();

    let started = false;

    for (const p of points) {
        const x = X(p.t);
        const y = Y(p.h - (zero ?? p.h));

        if (!started) {
            ctx.moveTo(x, y);
            started = true;
        } else {
            ctx.lineTo(x, y);
        }
    }

    ctx.strokeStyle = '#59d5a4';
    ctx.lineWidth = 2;
    ctx.stroke();
}

async function poll() {
    try {
        const r = await fetch('/api?since=' + stamp, {
            cache: 'no-store'
        });

        const d = await r.json();

        for (const p of d.points) {
            points.push({
                t: performance.now() / 1000 - (d.now - p.t),
                h: p.h
            });

            last = p.h;
            lastReceived = performance.now();
            stamp = Math.max(stamp, p.id);
        }

        if (zero === null && last !== null) {
            zero = last;
        }

        document.getElementById('height').textContent =
            last === null ? '—' : (last - zero).toFixed(3);

        document.getElementById('absolute').textContent =
            last === null ? '—' : last.toFixed(3);

        document.getElementById('hz').textContent =
            d.hz.toFixed(1);

        document.getElementById('status').textContent =
            d.connected && performance.now() - lastReceived < 1500
                ? 'Данные'
                : 'Нет данных';

        document.getElementById('detail').textContent =
            d.error || d.port;

        draw();

    } catch (e) {
        document.getElementById('status').textContent = 'Нет связи';
        document.getElementById('detail').textContent = String(e);
    }
}

setInterval(poll, 100);
setInterval(draw, 100);

resize();
poll();
</script>
</body>
</html>
'''


class Store:
    def __init__(self, port):
        self.lock = threading.Lock()
        self.points = collections.deque(maxlen=8000)
        self.seq = 0
        self.port = port
        self.error = 'Ожидание UART'
        self.connected = False

    def push(self, altitude):
        with self.lock:
            self.seq += 1
            self.points.append((
                self.seq,
                time.monotonic(),
                altitude
            ))
            self.error = ''

    def snapshot(self, since):
        with self.lock:
            now = time.monotonic()

            recent = [
                (i, t, h)
                for i, t, h in self.points
                if i > since
            ]

            hz = sum(
                t >= now - 2
                for _, t, _ in self.points
            ) / 2.0

            return dict(
                now=now,
                points=[
                    dict(id=i, t=t, h=h)
                    for i, t, h in recent
                ],
                hz=hz,
                port=self.port,
                error=self.error,
                connected=self.connected
            )


def reader(store, port, baud):
    try:
        import serial
    except ImportError:
        store.error = 'Не установлен python3-serial'
        return

    while True:
        try:
            with serial.Serial(
                port,
                baud,
                timeout=0.005,
                write_timeout=0.1
            ) as ser:

                store.connected = True
                store.error = ''

                buf = bytearray()
                pending = False
                next_request = time.monotonic()
                request_time = 0

                while True:
                    now = time.monotonic()

                    if pending and now - request_time > 0.15:
                        pending = False

                    if not pending and now >= next_request:
                        # MSPv1 read-only altitude request.
                        ser.write(b'$M<\x00\x6d\x6d')

                        pending = True
                        request_time = now
                        next_request = now + 0.02

                    chunk = ser.read(256)

                    if chunk:
                        buf.extend(chunk)

                    if len(buf) > 4096:
                        del buf[:-64]

                    while len(buf) >= 6:
                        pos = buf.find(b'$M')

                        if pos < 0:
                            buf.clear()
                            break

                        if pos:
                            del buf[:pos]

                        if len(buf) < 6:
                            break

                        if buf[2] not in (
                            ord('>'),
                            ord('!')
                        ):
                            del buf[0]
                            continue

                        n = buf[3]
                        total = 6 + n

                        if len(buf) < total:
                            break

                        frame = bytes(buf[:total])
                        del buf[:total]

                        checksum = 0

                        for b in frame[3:-1]:
                            checksum ^= b

                        if checksum != frame[-1]:
                            continue

                        if (
                            frame[2] == ord('>')
                            and frame[4] == 109
                            and n >= 4
                        ):
                            height_cm = struct.unpack_from(
                                '<i',
                                frame,
                                5
                            )[0]

                            store.push(height_cm / 100.0)
                            pending = False

        except Exception as e:
            store.connected = False
            store.error = f'UART: {e}'
            time.sleep(1)


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path.startswith('/api'):
            from urllib.parse import urlparse, parse_qs

            try:
                since = int(
                    parse_qs(
                        urlparse(self.path).query
                    ).get('since', ['0'])[0]
                )
            except ValueError:
                since = 0

            data = json.dumps(
                self.server.store.snapshot(since)
            ).encode()

            mime = 'application/json'

        elif self.path in ('/', '/index.html'):
            data = HTML.encode()
            mime = 'text/html; charset=utf-8'

        else:
            self.send_error(404)
            return

        self.send_response(200)
        self.send_header('Content-Type', mime)
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, *args):
        pass


if __name__ == '__main__':
    ap = argparse.ArgumentParser()

    ap.add_argument(
        '--serial',
        default='/dev/ttyAMA4'
    )

    ap.add_argument(
        '--baud',
        type=int,
        default=115200
    )

    ap.add_argument(
        '--http-port',
        type=int,
        default=8765
    )

    args = ap.parse_args()

    store = Store(args.serial)

    threading.Thread(
        target=reader,
        args=(store, args.serial, args.baud),
        daemon=True
    ).start()

    server = ThreadingHTTPServer(
        ('0.0.0.0', args.http_port),
        Handler
    )

    server.store = store

    print(
        f'GUI: http://127.0.0.1:{args.http_port} | '
        f'MSP read-only: {args.serial} @ {args.baud}',
        flush=True
    )

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
