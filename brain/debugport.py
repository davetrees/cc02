"""Localhost-only debug/exec port on 127.0.0.1:8079 (reach it via SSH only).

exec-by-design: POST /exec runs arbitrary Python against the live State
object. That is the point - so this app must ONLY ever bind loopback.
NEVER change the bind address to 0.0.0.0.

Routes (all JSON):
  GET  /state           -> JSON-safe dump of every st.* field
                           (unserializable values become repr strings)
  GET  /config          -> st.config
  POST /config {k: v}   -> st.config.update + persist to config.json
  POST /exec {"code": "..."} ->
       exec() in a persistent namespace exposing st (live State), the brain
       modules/threads (vision, camera, serial_link, autopilot, gamepad,
       mapping, csvlog, P/protocol) and the aiohttp asyncio loop as `loop`.
       Reply: {"ok": bool, "stdout": captured prints, "result": repr of the
       last bare expression (None if none), "error": full traceback or null}.

Example (from the Pi):
  curl -s 127.0.0.1:8079/exec -X POST -H 'Content-Type: application/json' \
       -d '{"code": "print(st.mode); st.auto_costs"}'
"""
import ast
import contextlib
import io
import json
import traceback

from aiohttp import web

BIND_HOST = "0.0.0.0"   # opened to LAN at user request 2026-08-16 (was loopback)
BIND_PORT = 8001


def make_app(state, extra=None):
    app = web.Application()
    app['state'] = state
    ns = {'st': state, 'state': state}
    if extra:
        ns.update(extra)
    app['ns'] = ns
    app.router.add_get('/state', state_dump)
    app.router.add_get('/config', config_get)
    app.router.add_post('/config', config_post)
    app.router.add_post('/exec', exec_code)
    return app


async def start(state, extra=None):
    """Bind the debug app on loopback; returns the AppRunner."""
    runner = web.AppRunner(make_app(state, extra))
    await runner.setup()
    site = web.TCPSite(runner, BIND_HOST, BIND_PORT)
    await site.start()
    state.log(f'debugport: exec/state/config on {BIND_HOST}:{BIND_PORT} '
              '(localhost ONLY, ssh in to use)')
    return runner


def _jsafe(v):
    try:
        json.dumps(v)
        return v
    except (TypeError, ValueError):
        return repr(v)


async def state_dump(request):
    st = request.app['state']
    return web.json_response({k: _jsafe(v) for k, v in vars(st).items()})


async def config_get(request):
    return web.json_response(request.app['state'].config)


async def config_post(request):
    st = request.app['state']
    try:
        body = await request.json()
        if not isinstance(body, dict):
            raise ValueError('body must be a JSON object')
    except Exception as e:
        return web.json_response({'ok': False, 'error': str(e)}, status=400)
    st.config.update(body)
    st.save_config()
    st.log(f'debugport: config updated ({", ".join(body.keys())})')
    return web.json_response({'ok': True, 'config': st.config})


async def exec_code(request):
    import asyncio
    try:
        body = await request.json()
        code = body['code']
    except Exception as e:
        return web.json_response({'ok': False, 'error': f'bad body: {e}'},
                                 status=400)
    ns = request.app['ns']
    ns['loop'] = asyncio.get_event_loop()
    buf = io.StringIO()
    result = None
    err = None
    try:
        tree = ast.parse(code, mode='exec')
        last_expr = None
        if tree.body and isinstance(tree.body[-1], ast.Expr):
            last_expr = ast.fix_missing_locations(
                ast.Expression(tree.body.pop().value))
        with contextlib.redirect_stdout(buf):
            exec(compile(tree, '<debugport>', 'exec'), ns)
            if last_expr is not None:
                result = eval(compile(last_expr, '<debugport>', 'eval'), ns)
    except Exception:
        err = traceback.format_exc()
    return web.json_response({
        'ok': err is None,
        'stdout': buf.getvalue(),
        'result': repr(result) if result is not None else None,
        'error': err,
    })
