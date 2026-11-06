/*
Live telemetry from DJI Fly's flight log on the phone.

Waits for DJI Fly to start a new FlightRecord_*.txt (it does so when the drone
connects), reads the bytes added to it over adb, decrypts them with keys
fetched from DJI once per log, and prints every new row as one JSON line on
stdout. Status messages go to stderr.

stdout lines:
  {"event":"clock", "phone_minus_pc": s}           phone clock - PC clock, at start
  {"event":"new_log", "file": name}
  {"event":"keys", "fetch_ms": ms, "groups": n}
  {"t_pc": unix s, "dateTime": ..., "lat": ..., ...}   one per log row (10 Hz)

Usage:
  node live_log.mjs                    wait for a new log (start before connecting the drone)
  node live_log.mjs --existing         follow the newest log already on the phone
  node live_log.mjs --replay <dir>     test without a phone: replay snapshots of a growing log (<size>.txt files)
Needs DJI_API_KEY in the environment; adb is NEO_PILOT_ADB (set by neo_pilot/telemetry.py) or "adb" on the PATH.
Run "npm install" in this folder once.
*/
import { DJILog } from "dji-log-parser-js";
import { execFile } from "child_process";
import { readdirSync, readFileSync } from "fs";
import { join } from "path";
import { promisify } from "util";

const run = promisify(execFile);
const ADB = process.env.NEO_PILOT_ADB || "adb";
const LOG_DIR = "/sdcard/Android/data/dji.go.v5/files/FlightRecord";
const PERIOD_MS = 100;      // pause between reads; one adb read itself takes ~100 ms
const LIST_EVERY_MS = 1000; // how often to look for a newer log file

const args = process.argv.slice(2);
const existing = args.includes("--existing");
const replayDir = args.includes("--replay") ? args[args.indexOf("--replay") + 1] : null;
const apiKey = process.env.DJI_API_KEY;
if (!apiKey) { console.error("DJI_API_KEY is not set"); process.exit(1); }

const emit = (obj) => process.stdout.write(JSON.stringify(obj) + "\n");
const status = (msg) => console.error(`${new Date().toISOString()}  ${msg}`);
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

const adb = async (cmd) => (await run(ADB, ["exec-out", cmd], { encoding: "buffer", maxBuffer: 64 << 20 })).stdout;

// ---- data source: the phone, or a folder of snapshots (replay) ----
async function newestLog() {
  const out = (await adb(`ls -t ${LOG_DIR}/FlightRecord_*.txt | head -1`)).toString().trim();
  return out ? out.split("/").pop() : null;
}
const readFrom = (file, offset) => adb(`tail -c +${offset + 1} '${LOG_DIR}/${file}'`);

function replaySource(dir) {
  const sizes = readdirSync(dir).map((f) => +f.replace(".txt", "")).sort((a, b) => a - b);
  let i = 0;
  return async (offset) => {
    if (i >= sizes.length) return Buffer.alloc(0);
    await sleep(500);   // the real log grew about every 0.5 s
    return readFileSync(join(dir, `${sizes[i++]}.txt`)).subarray(offset);
  };
}

// ---- records: type u8, length u16 LE, data, 0xFF. Only complete records are decoded. ----
function findChainStart(buf) {
  for (let start = 11; start < Math.min(3000, buf.length); start++) {
    let p = start, n = 0;
    while (p + 3 <= buf.length) {
      const e = p + 3 + buf.readUInt16LE(p + 1);
      if (e >= buf.length || buf[e] !== 0xff) break;
      p = e + 1;
      if (++n > 20) return start;
    }
  }
  return -1;
}
function walk(buf, p) {   // advance over complete records, return the end of the last one
  while (p + 3 <= buf.length) {
    const e = p + 3 + buf.readUInt16LE(p + 1);
    if (e >= buf.length || buf[e] !== 0xff) break;
    p = e + 1;
  }
  return p;
}

function row(f) {
  const o = f.osd, g = f.gimbal;
  return {
    t_pc: Date.now() / 1000,
    dateTime: f.custom.dateTime,
    flyTime: o.flyTime,
    lat: o.latitude, lon: o.longitude,
    height: o.height, vpsHeight: o.vpsHeight,
    vN: o.xSpeed, vE: o.ySpeed, vD: o.zSpeed,   // log velocities are North/East/Down
    roll: o.roll, pitch: o.pitch, yaw: o.yaw,
    gimbalPitch: g.pitch, gimbalYaw: g.yaw,
    gpsNum: o.gpsNum, isGPSUsed: o.isGpdUsed, flycState: o.flycState,   // "isGpdUsed" is the library's spelling
    isMotorOn: o.isMotorOn, battery: f.battery.chargeLevel,
  };
}

async function main() {
  let read, file = null, startLog = null;
  if (replayDir) {
    read = replaySource(replayDir);
    file = "replay";
    emit({ event: "new_log", file: replayDir });
  } else {
    const t0 = Date.now();
    const phone = Number((await adb("date +%s%N")).toString()) / 1e9;
    const t1 = Date.now();
    emit({ event: "clock", phone_minus_pc: phone - (t0 + t1) / 2000 });
    startLog = await newestLog();
    if (existing) {
      file = startLog;
      emit({ event: "new_log", file });
    } else {
      status(`waiting for a new flight log (newest now: ${startLog})`);
    }
    read = (offset) => readFrom(file, offset);
  }

  let buf = Buffer.alloc(0), chain = -1, end = 0, keys = null, keyCount = 0, sent = 0, lastList = Date.now();
  let nextKeyTry = 0, decodes = 0;

  for (;;) {
    // Look for a newer log (the drone reconnected or a new flight started).
    if (!replayDir && Date.now() - lastList > LIST_EVERY_MS) {
      lastList = Date.now();
      const newest = await newestLog();
      // The log that was newest at start is an old flight; ignore it unless --existing.
      if (newest && newest !== file && newest !== startLog) {
        file = newest;
        buf = Buffer.alloc(0); chain = -1; end = 0; keys = null; sent = 0;
        emit({ event: "new_log", file });
        status(`new log: ${file}`);
      }
    }
    if (file === null) { await sleep(PERIOD_MS); continue; }

    const added = await read(buf.length);
    if (added.length) buf = Buffer.concat([buf, added]);
    else if (replayDir && keys) { status("replay finished"); return; }

    if (chain < 0) {
      chain = findChainStart(buf);
      if (chain < 0) { await sleep(PERIOD_MS); continue; }
      end = chain;
    }
    const newEnd = walk(buf, end);
    if (newEnd === end && keys) { await sleep(PERIOD_MS); continue; }
    end = newEnd;
    const log = new DJILog(buf.subarray(0, end));

    if (!keys) {
      if (Date.now() < nextKeyTry) continue;
      const req = log.keychainsRequest().keychainsArray;
      if (!req.flat().length) { await sleep(PERIOD_MS); continue; }   // key records not written yet
      const t = Date.now();
      try {
        keys = await log.fetchKeychains(apiKey);
        keyCount = req.flat().length;
        emit({ event: "keys", fetch_ms: Date.now() - t, groups: keys.length });
        status(`keys fetched in ${Date.now() - t} ms`);
      } catch (e) {
        status(`key fetch failed (${e}), retrying in 2 s`);
        nextKeyTry = Date.now() + 2000;
        continue;
      }
    } else if (++decodes % 50 === 0 && log.keychainsRequest().keychainsArray.flat().length !== keyCount) {
      status("log has new key records, fetching keys again");
      keys = null;
      continue;
    }

    const frames = log.frames(keys);
    for (; sent < frames.length; sent++) emit(row(frames[sent]));
    await sleep(PERIOD_MS);
  }
}

main().catch((e) => { status(`fatal: ${e.stack || e}`); process.exit(1); });
