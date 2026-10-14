// Offline raw-telemetry metrics; no hardware calls, motion or battery inference.
// Criteria are fixed in R19_DUAL_HOLD_PREPARATION_20261013.md.
// Accepts a helper raw object; CLI also accepts a saved {raw,derived,...} record.
import {readFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
export function analyze(raw){const active=raw.telemetry.filter(x=>x.phase==="velocity"),sgn=Math.sign(raw.targets[0]),last=active.at(-1);
// Older records predate the worker; a present failed close must never pass.
const good=raw.reason===null&&raw.stop_confirmed&&raw.restore_confirmed&&raw.serial_closed&&raw.power_monitor_closed!==false&&raw.run_phase_seen.every(Boolean);
if(!last)return {guard_pass:good,no_active_data:true};
const cd=(a,b)=>((a-b+2147483648)%4294967296+4294967296)%4294967296-2147483648;
function window(ms,ageEnd=0){const end=active.reduce((best,x)=>Math.abs(cd(last.values[0],x.values[0])-ageEnd)<Math.abs(cd(last.values[0],best.values[0])-ageEnd)?x:best,active[0]);
const before=active.filter(x=>cd(end.values[0],x.values[0])>=0),start=before.reduce((best,x)=>Math.abs(cd(end.values[0],x.values[0])-ms)<Math.abs(cd(end.values[0],best.values[0])-ms)?x:best,before[0]);
const rows=before.filter(x=>cd(x.values[0],start.values[0])>=0),width=cd(end.values[0],start.values[0]);
if(width<=0)return {width_ms:width,complete:false}; const sides=[0,1].map(side=>{const rates=rows.slice(1).map((x,i)=>sgn*cd(x.values[1+side],rows[i].values[1+side])*1000/cd(x.values[0],rows[i].values[0]));const mean=sgn*cd(end.values[1+side],start.values[1+side])*1000/width;
return{raw_mean:mean,filtered_mean:rows.reduce((s,x)=>s+sgn*x.values[7+side],0)/rows.length,raw_sd:Math.sqrt(rates.reduce((s,x)=>s+(x-mean)**2,0)/rates.length),end_filtered:sgn*end.values[7+side],count_delta:sgn*cd(end.values[1+side],start.values[1+side])};});
const phaseRows=raw.phases.filter(x=>x.phase==="velocity"&&cd(x.values[0],start.values[0])>=0&&cd(end.values[0],x.values[0])>=0),run=phaseRows.length>=rows.length-1&&phaseRows.every(x=>x.values[1]===2&&x.values[2]===2&&x.values[3]===0);
return{width_ms:width,complete:Math.abs(width-ms)<=60&&rows.length>=ms/50-2,sides,pair_mean_difference:Math.abs(sides[0].raw_mean-sides[1].raw_mean),both_RUN:run};}
const W=raw.nominal_seconds===5?2000:1000,tail=window(W),durationok=raw.motion_to_stop_s>=raw.nominal_seconds-.05&&raw.motion_to_stop_s<=raw.nominal_seconds+.03;
const criteria={cleanup_and_guard:good,full_episode:durationok,window_complete:tail.complete,both_RUN:tail.both_RUN,raw_means:tail.sides?.every(x=>Math.abs(x.raw_mean-200)<=20)??false,filtered_means:tail.sides?.every(x=>Math.abs(x.filtered_mean-200)<=30)??false,raw_sds:tail.sides?.every(x=>x.raw_sd<=60)??false,pair:tail.pair_mean_difference<=20};
let blocks=null;if(raw.nominal_seconds===5){blocks=[window(1000,1000),window(1000)];criteria.each_1s_block=blocks.every(b=>b.complete&&b.sides.every(x=>Math.abs(x.raw_mean-200)<=30));}
const out={episode:raw.episode,guard_pass:good,full_episode:durationok,active_ticks:last.values.slice(1,3),final_ticks:raw.encoder_delta,motion_seconds:raw.motion_to_stop_s,max_pair_separation:Math.max(...active.map(x=>Math.abs(x.values[1]-x.values[2]))),peak_filtered:[0,1].map(s=>Math.max(...active.map(x=>sgn*x.values[7+s]))),max_pwm:Math.max(...active.flatMap(x=>x.values.slice(3,5).map(Math.abs)))};
if(raw.nominal_seconds>=3)return{...out,tail,blocks,criteria,tracking_pass:Object.values(criteria).every(Boolean)};
return {...out,smoke_pass:good&&durationok};
}
if (process.argv[1] && resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  if (process.argv.length !== 3) throw new Error('Usage: node analyze_r19_dual_hold.mjs saved_record.json');
  const record=JSON.parse(readFileSync(process.argv[2],'utf8'));
  console.log(JSON.stringify(analyze(record.raw ?? record),null,2));
}
