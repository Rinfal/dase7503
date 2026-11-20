// No browser or real controller: exercise touch event handlers against a fake API.
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import vm from 'node:vm';

export async function testPhoneFrontend(manual = false) {
  const source = await readFile(new URL('../robot_ws/src/logistics_robot/logistics_robot/web/app.js', import.meta.url), 'utf8');
  const elements = new Map();
  const element = id => {
    if (!elements.has(id)) elements.set(id, {
      dataset: {}, handlers: {}, disabled: false, textContent: '', value: '',
      addEventListener(name, handler) { this.handlers[name] = handler; },
      setPointerCapture() {}, removeAttribute(name) { delete this[name]; }, replaceChildren() {}, append() {},
    });
    return elements.get(id);
  };
  const directions = ['forward', 'back', 'left', 'right', 'ccw', 'cw'].map(name => {
    const button = element('drive-' + name); button.dataset.drive = name; return button;
  });
  const lifts = ['up', 'down'].map(name => {
    const button = element('lift-' + name); button.dataset.lift = name; return button;
  });
  let now = 1000, armed = false, cameraAge = null, delayNextDrive = false, resumeDrive;
  const requests = [];
  const state = () => ({armed, direct_control: !manual, manual_lock: manual, lateral_enabled: true, drive_enabled: true, lift_enabled: true, feedback_age: .02,
    bench_only: false, lift_range_verified: false, lift_busy: false, camera_age: cameraAge,
    lift_pulses_us: {down: 400, up: 750}, video_fps: 10,
    battery_raw: null, server_clock: now / 1000, reason: 'armed'});
  const context = vm.createContext({
    document: {
      getElementById: element,
      querySelectorAll: selector => selector === '[data-drive]' ? directions :
        selector === '[data-lift]' ? lifts : [...directions, ...lifts, element('arm'), element('halt')],
      addEventListener() {}, createElement: () => element('generated'), hidden: false,
    },
    PhoneVideo: {VideoStream: class {
      constructor(image){this.image=image;this.running=false;this.hasFrame=false;}
      start(){if(this.running)return;this.running=true;this.hasFrame=true;this.image.src='blob:test-video';}
      stop(){this.running=false;this.hasFrame=false;this.image.removeAttribute('src');}
    }},
    window: {addEventListener() {}}, performance: {now: () => now},
    setInterval() {}, setTimeout() {}, clearTimeout() {},
    AbortSignal: {timeout() {}}, console,
    fetch: async (path, options = {}) => {
      const body = options.body ? JSON.parse(options.body) : null;
      requests.push({path, body});
      if (path === '/api/arm') armed = true;
      if (path === '/api/lock' || (path === '/api/stop'&&!manual)) armed = false;
      if (path === '/api/drive' && delayNextDrive) {
        delayNextDrive = false;
        await new Promise(resolve => { resumeDrive = resolve; });
      }
      return {ok: true, status: 200,
        json: async () => path === '/api/arm' ? {lease: 'test-lease', server_clock: now / 1000} :
          path === '/api/evidence' ? [] : state()};
    },
  });
  vm.runInContext(source, context);
  const flush = async () => { for (let i = 0; i < 20; i++) await Promise.resolve(); };
  await flush();
  assert.equal(element('arm').hidden, !manual);
  assert.equal(requests.filter(r => r.path==='/api/arm').length, 0);
  if(manual){
    assert.equal(armed,false);
    assert.equal(element('drive-forward').disabled,true);
    await element('arm').handlers.click();await flush();
    assert.equal(armed,true);assert.equal(element('arm').textContent,'上锁驾驶');
  }
  const event = {preventDefault() {}, pointerId: 1};
  const lastDrive = () => requests.filter(r => r.path === '/api/drive').at(-1).body;
  for (const [button, expected] of directions.map((b, i) => [b, [[1,0,0],[-1,0,0],[0,1,0],[0,-1,0],[0,0,1],[0,0,-1]][i]])) {
    button.handlers.pointerdown(event); await flush();
    assert.deepEqual([lastDrive().forward, lastDrive().lateral, lastDrive().angular], expected);
    button.handlers.pointerup(); await flush();
    assert.equal(lastDrive().forward, 0); assert.equal(lastDrive().angular, 0);
  }
  // A lift click waits for in-flight motion, sends neutral, then sends the lift.
  delayNextDrive = true;
  directions[0].handlers.pointerdown(event); await flush();
  const lifting = lifts[0].handlers.click(); await flush();
  assert.equal(requests.filter(r => r.path === '/api/lift').length, 0);
  resumeDrive(); await lifting; await flush();
  const controls = requests.filter(r => ['/api/drive', '/api/lift'].includes(r.path));
  assert.equal(controls.at(-1).path, '/api/lift');
  assert.equal(controls.at(-2).body.forward, 0);
  assert.ok(controls.every(r => r.body.sequence===undefined&&r.body.lease===undefined&&r.body.server_clock===undefined));
  // Held movement continues beyond the former one-second cap until release.
  directions[0].handlers.pointerup(); await flush();
  directions[0].handlers.pointerdown(event); await flush();
  now += 1050;
  await vm.runInContext('update()', context);
  await vm.runInContext('drive()', context);
  assert.equal(lastDrive().forward, 1);
  directions[0].handlers.pointerup(); await flush();
  assert.equal(lastDrive().forward, 0);
  // One persistent MJPEG source, with no per-frame HTTP polling.
  cameraAge = .01;
  await vm.runInContext('update()', context);
  const streamURL = element('frame').src;
  assert.ok(streamURL.startsWith('blob:'));
  await vm.runInContext('update()', context);
  assert.equal(element('frame').src, streamURL);
  assert.equal(requests.filter(r => r.path.startsWith('/api/frame')).length, 0);
  cameraAge = null;
  await vm.runInContext('update()', context);
  assert.equal(element('frame').src, streamURL, 'status delays must not tear down video');
  vm.runInContext('unloadStop()',context);await flush();
  assert.equal(element('frame').src, undefined);
  await element('stop').handlers.click(); await flush();
  assert.equal(armed, manual);
  if(manual){await element('arm').handlers.click();await flush();assert.equal(armed,false);assert.equal(element('drive-forward').disabled,true);}
  console.log(`Phone frontend (${manual?'manual lock':'direct'}): six directions, release, ordered lift, continuous hold, video, stop and lock PASS`);
}
