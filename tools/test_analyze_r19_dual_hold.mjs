// Offline synthetic-data checks only; no device or physical model.
import test from 'node:test';
import assert from 'node:assert/strict';
import {analyze} from './analyze_r19_dual_hold.mjs';
function fixture(sign=1) {
  const raw={episode:sign>0?'both-forward-5s':'both-reverse-5s',
    telemetry:[],phases:[],nominal_seconds:5,targets:[sign*200,sign*200],
    motion_to_stop_s:4.97,reason:null,stop_confirmed:true,restore_confirmed:true,
    serial_closed:true,run_phase_seen:[true,true],encoder_delta:[sign*990,sign*990]};
  for(let t=50;t<=4950;t+=50) {
    raw.telemetry.push({phase:'velocity',values:[t,sign*t/5,sign*t/5,sign*100,sign*100,sign*200,sign*200,sign*200,sign*200]});
    raw.phases.push({phase:'velocity',values:[t,2,2,0]});
  }
  return raw;
}
test('forward/reverse exact tracking and blocks',()=>{
  for(const sign of [1,-1]) {
    const d=analyze(fixture(sign));
    assert.equal(d.tracking_pass,true);
    assert.deepEqual(d.tail.sides.map(x=>x.raw_mean),[200,200]);
    assert.equal(d.tail.width_ms,2000);
    assert.equal(d.blocks.length,2);
  }
});
test('unequal speeds fail paired mean criterion',()=>{
  const r=fixture(); for(const x of r.telemetry) {x.values[1]=x.values[0]*.22; x.values[2]=x.values[0]*.18;}
  const d=analyze(r); assert.equal(d.criteria.raw_means,true);
  assert.equal(d.criteria.pair,false); assert.equal(d.tracking_pass,false);
});
test('late drift is not hidden by a passing two-second average',()=>{
  const r=fixture();
  for(const x of r.telemetry) {
    const t=x.values[0],c=t<=2950?t/5:(t<=3950?590+(t-2950)*.16:750+(t-3950)*.24);
    x.values[1]=x.values[2]=c;
  }
  const d=analyze(r);
  assert.equal(d.criteria.raw_means,true); assert.equal(d.criteria.each_1s_block,false);
  assert.equal(d.tracking_pass,false);
});
test('guard failure and incomplete run cannot pass',()=>{
  const r=fixture(); r.reason='Stale telemetry'; assert.equal(analyze(r).tracking_pass,false);
  const q=fixture(); q.motion_to_stop_s=1.2; assert.equal(analyze(q).criteria.full_episode,false);
  q.telemetry=q.telemetry.slice(0,20); q.phases=q.phases.slice(0,20);
  assert.equal(analyze(q).criteria.window_complete,false);
});
test('missing RUN/cleanup cannot pass',()=>{
  const r=fixture(); for(const x of r.phases) x.values[2]=1;
  assert.equal(analyze(r).criteria.both_RUN,false);
  r.stop_confirmed=false; assert.equal(analyze(r).guard_pass,false);
});
test('failed power-monitor cleanup cannot pass, legacy records remain readable',()=>{
  const r=fixture(); r.power_monitor_closed=false;
  assert.equal(analyze(r).guard_pass,false);
  assert.equal(analyze(r).tracking_pass,false);
  r.power_monitor_closed=true; assert.equal(analyze(r).tracking_pass,true);
  delete r.power_monitor_closed; assert.equal(analyze(r).tracking_pass,true);
});
test('coast samples are excluded from speed metrics',()=>{
  const r=fixture(); r.telemetry.push({phase:'after_stop',values:[5000,5000,5000,0,0,0,0,0,0]});
  r.encoder_delta=[5000,5000]; assert.equal(analyze(r).tracking_pass,true);
});
test('controller clock and encoder rollover are measured modulo32',()=>{
  const r=fixture(); for(const x of r.telemetry) {
    x.values[0]=(x.values[0]+4294965000)%4294967296;
    for(const s of [1,2]) x.values[s]=(x.values[s]+2147483000+2147483648)%4294967296-2147483648;
  }
  for(const x of r.phases)x.values[0]=(x.values[0]+4294965000)%4294967296;
  assert.equal(analyze(r).tracking_pass,true);
});
