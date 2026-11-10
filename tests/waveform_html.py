"""Offline Plotly viewer with on-demand, multiresolution waveform chunks.

NPZ remains the full-precision archive. Viewer voltage chunks use float32;
overview levels preserve extrema instead of throwing away narrow spikes.
"""
import base64
import gzip
import html
import json
from pathlib import Path

import numpy as np
from plotly.offline import get_plotlyjs

RAW_CHUNK = 131072
ENVELOPE_CHUNK = 8192
METHODS = ("PWM50", "PWM333", "DSHOT4", "DSHOT8", "DSHOT16")


def envelope(values, factor, start, count):
    """Min/max for each bin, including a partial final bin."""
    segment = values[start * factor:min(len(values), (start + count) * factor)]
    full = len(segment) // factor
    result = np.empty((count, 2), dtype=np.float32)
    if full:
        bins = segment[:full * factor].reshape(full, factor)
        result[:full, 0] = bins.min(axis=1)
        result[:full, 1] = bins.max(axis=1)
    if full < count:
        tail = segment[full * factor:]
        result[full] = (tail.min(), tail.max())
    return result


def write_chunk(path, key, data, count):
    encoded = base64.b64encode(gzip.compress(data, compresslevel=3, mtime=0)).decode("ascii")
    path.write_text(f"window.noiseChunk({json.dumps(key)},'{encoded}',{count});", encoding="utf-8")


def export_viewer(path, summary, arrays):
    path = Path(path)
    assets = path.parent / "waveform_data"
    assets.mkdir(exist_ok=True)
    (assets / "plotly.min.js").write_text(get_plotlyjs(), encoding="utf-8")
    count = len(arrays["time_ns"])
    step_s = summary["settings"]["step_us"] / 1e6
    factors = [1]
    while count / factors[-1] > 2000:
        factors.append(factors[-1] * 16)
    manifest = dict(count=count, step_s=step_s, duration_s=summary["settings"]["duration_s"],
                    start_s=summary.get("waveform_window_start_s", 0),
                    first_error_s=summary.get("first_signal_error_s"), factors=factors,
                    vih=summary["settings"]["vih_v"], vil=summary["settings"]["vil_v"], methods={})
    minimum, maximum = 0., 3.3
    for method in METHODS:
        for key in ("noisy_v", "filtered_v"):
            values = arrays[f"{method}_{key}"]
            minimum = min(minimum, float(values.min()))
            maximum = max(maximum, float(values.max()))
    padding = max(.1, (maximum - minimum) * .05)
    manifest["voltage_range"] = [minimum - padding, maximum + padding]
    for method in METHODS:
        clean, noisy, filtered, binary = (arrays[f"{method}_{key}"] for key in
                                          ("clean_v", "noisy_v", "filtered_v", "binary"))
        manifest["methods"][method] = {}
        for factor in factors:
            level = assets / method / str(factor)
            level.mkdir(parents=True, exist_ok=True)
            bins = (count + factor - 1) // factor
            chunk_size = RAW_CHUNK if factor == 1 else ENVELOPE_CHUNK
            chunks = []
            for chunk_index, start in enumerate(range(0, bins, chunk_size)):
                length = min(chunk_size, bins - start)
                if factor == 1:
                    stop = start + length
                    voltages = np.stack((noisy[start:stop], filtered[start:stop])).astype("<f4")
                    states = ((clean[start:stop] > 0).astype(np.uint8) |
                              (binary[start:stop].astype(np.uint8) << 1))
                    packed = voltages.tobytes() + states.tobytes()
                else:
                    bounds = np.column_stack([envelope(v, factor, start, length) for v in
                                               (clean, noisy, filtered, binary)]).astype("<f4")
                    # Binary GPIO is displayed on the same voltage scale.
                    bounds[:, 6:] *= 3.3
                    packed = bounds.tobytes()
                key = f"{method}:{factor}:{chunk_index}"
                file = level / f"{chunk_index}.js"
                write_chunk(file, key, packed, length)
                chunks.append(dict(key=key, file=file.relative_to(path.parent).as_posix(), start=start, count=length))
            manifest["methods"][method][str(factor)] = chunks
    title = summary.get("effective_profile", {}).get("name", "Gaussian noise")
    content = TEMPLATE.replace("__TITLE__", html.escape(title)).replace(
        "__MANIFEST__", json.dumps(manifest).replace("<", "\\u003c"))
    path.write_text(content, encoding="utf-8")
    return manifest


TEMPLATE = r'''<!doctype html>
<html lang="en"><head><meta charset="utf-8"><title>Noise simulation waveforms</title>
<script src="waveform_data/plotly.min.js"></script>
<style>
body{margin:0;font:15px system-ui,sans-serif;background:#f6f8fa;color:#17212b}
header{position:sticky;top:0;background:white;z-index:10;padding:12px 20px;border-bottom:1px solid #ccd3da}
h1{font-size:20px;margin:0 0 8px}button,input{font:inherit;padding:5px;margin:3px}input{width:110px}
#plot{width:100%;height:1450px}#status{margin:8px 0 0;color:#465569}.error{color:#b00020!important}
</style></head><body>
<header><h1>Full-duration waveforms · __TITLE__</h1>
<button id="full">Full run</button><button id="first">First error</button>
<button id="left">← Pan</button><button id="right">Pan →</button>
<label>Start (s) <input id="start" type="number" step="any"></label>
<label>End (s) <input id="end" type="number" step="any"></label><button id="apply">Show</button>
<div>Wheel to zoom time; drag to pan time; use the toolbar for box zoom. Voltage axes stay fixed. Scroll the page to see all five methods.
Overview traces show bin minima/maxima; zoom in to load individual samples. Voltage display uses float32;
the NPZ archive retains simulation precision. Keep this HTML beside its waveform_data folder.</div>
<div id="status">Loading…</div></header><div id="plot"></div>
<script>
const meta=__MANIFEST__;
const methods=Object.keys(meta.methods), names=['Clean','Noisy','Filtered','GPIO × 3.3 V'];
const colors=['#1768ac','#aaa','#ed7d15','#1a974e'];
const plot=document.getElementById('plot'), status=document.getElementById('status');
const cache=new Map(), pending=new Map(), payloads=new Map();
window.noiseChunk=(key,data,count)=>payloads.set(key,{data,count});
async function chunk(info,factor){
 if(cache.has(info.key)){let v=cache.get(info.key);cache.delete(info.key);cache.set(info.key,v);return v;}
 if(pending.has(info.key))return pending.get(info.key);
 const promise=(async()=>{
  await new Promise((resolve,reject)=>{const script=document.createElement('script');script.src=info.file;
   script.onload=()=>{script.remove();resolve();};script.onerror=()=>{script.remove();reject(new Error('Cannot load '+info.file));};
   document.head.appendChild(script);});
  const payload=payloads.get(info.key);payloads.delete(info.key);
  if(!payload)throw new Error('Missing waveform chunk '+info.key);
  const bytes=Uint8Array.from(atob(payload.data),c=>c.charCodeAt(0));
  const buffer=await new Response(new Blob([bytes]).stream().pipeThrough(new DecompressionStream('gzip'))).arrayBuffer();
  const result={count:payload.count,values:new Float32Array(buffer,0,factor===1?payload.count*2:payload.count*8),
   states:factor===1?new Uint8Array(buffer,payload.count*8,payload.count):null};
  cache.set(info.key,result);while(cache.size>16)cache.delete(cache.keys().next().value);
  return result;
 })();pending.set(info.key,promise);
 try{return await promise;}finally{pending.delete(info.key);}
}
let range=[0,meta.duration_s], generation=0, updating=false, timer;
const layout={height:1450,margin:{l:85,r:30,t:45,b:100},dragmode:'pan',
 legend:{orientation:'h',y:1.03},uirevision:'waveforms',annotations:[],shapes:[],
 grid:{rows:5,columns:1,pattern:'independent',roworder:'top to bottom'}};
const traces=[];
for(let row=0;row<5;row++){
 const axis=row===0?'':String(row+1);
 layout['xaxis'+axis]={title:row===4?{text:'Time (s)'}:undefined,range,anchor:'y'+axis,
  domain:[0,1],matches:row===0?undefined:'x'};
 layout['yaxis'+axis]={title:{text:methods[row]+' (V)'},domain:[1-(row+1)/5+.035,1-row/5-.025],
  range:meta.voltage_range,autorange:false,fixedrange:true};
 for(let field=0;field<4;field++)traces.push({type:'scattergl',mode:'lines',x:[],y:[],
  name:names[field],legendgroup:names[field],showlegend:row===0,xaxis:'x'+axis,yaxis:'y'+axis,
  line:{color:colors[field],width:field===1?.7:1,shape:(field===0||field===3)?'hv':'linear'},hovertemplate:'%{x:.9f} s<br>%{y:.5f} V<extra>'+methods[row]+' '+names[field]+'</extra>'});
 for(const threshold of [meta.vih,meta.vil])layout.shapes.push({type:'line',xref:'x'+axis,yref:'y'+axis,
  x0:0,x1:meta.duration_s,y0:threshold,y1:threshold,line:{color:'#777',dash:'dot',width:1}});
 if(meta.first_error_s!==null)layout.shapes.push({type:'line',xref:'x'+axis,yref:'y'+axis+' domain',
  x0:meta.first_error_s,x1:meta.first_error_s,y0:0,y1:1,line:{color:'red',dash:'dash',width:1}});
}
layout.xaxis5.rangeslider={visible:true,thickness:.045,range:[0,meta.duration_s]};
async function refresh(next){
 const id=++generation;
 let lo=Math.max(0,Number(next[0])),hi=Math.min(meta.duration_s,Number(next[1]));
 if(!Number.isFinite(lo)||!Number.isFinite(hi)||hi<=lo)return;
 range=[lo,hi];document.getElementById('start').value=lo;document.getElementById('end').value=hi;
 const factor=meta.factors.find(f=>Math.ceil((hi-lo)/meta.step_s/f)<=2000)||meta.factors.at(-1);
 status.className='';status.textContent='Loading visible waveforms…';
 try{
  const rows=await Promise.all(methods.map(async(method)=>{
   const x=[],ys=[[],[],[],[]],first=Math.max(0,Math.floor(lo/meta.step_s/factor)-1),last=Math.min(Math.ceil(meta.count/factor),Math.ceil(hi/meta.step_s/factor)+1);
   for(const info of meta.methods[method][String(factor)]){
    if(info.start+info.count<=first||info.start>=last)continue;
    const data=await chunk(info,factor);if(id!==generation)return null;
    for(let bin=Math.max(first,info.start);bin<Math.min(last,info.start+info.count);bin++){
     const j=bin-info.start;
     if(factor===1){x.push(bin*meta.step_s);const bits=data.states[j];
      ys[0].push((bits&1)?3.3:0);ys[1].push(data.values[j]);ys[2].push(data.values[data.count+j]);ys[3].push((bits&2)?3.3:0);
     }else{const center=Math.min((bin+.5)*factor,meta.count-1)*meta.step_s;x.push(center,center);
      for(let field=0;field<4;field++)ys[field].push(data.values[j*8+field*2],data.values[j*8+field*2+1]);}
    }
   }return {x,ys};
  }));
  if(id!==generation)return;
  updating=true;
  const update={x:[],y:[]},axes={};
  rows.forEach(row=>{for(let field=0;field<4;field++){update.x.push(row.x);update.y.push(row.ys[field]);}});
  for(let row=0;row<5;row++)axes['xaxis'+(row?row+1:'')+'.range']=range;
  await Plotly.update(plot,update,axes);
  updating=false;
  status.textContent=factor===1?'Individual simulation samples · '+(meta.step_s*1e6)+' µs resolution':
   'Min/max envelopes · '+(factor*meta.step_s*1e6)+' µs bins · zoom further for individual samples';
  window.waveformViewer={range:[...range],factor,loadedPoints:rows[0].x.length};
 }catch(error){updating=false;status.className='error';status.textContent=error.message;console.error(error);}
}
document.getElementById('full').onclick=()=>refresh([0,meta.duration_s]);
document.getElementById('first').onclick=()=>refresh([meta.start_s,Math.min(meta.duration_s,meta.start_s+.025)]);
document.getElementById('left').onclick=()=>{const width=range[1]-range[0],lo=Math.max(0,range[0]-width/2);refresh([lo,lo+width]);};
document.getElementById('right').onclick=()=>{const width=range[1]-range[0],hi=Math.min(meta.duration_s,range[1]+width/2);refresh([hi-width,hi]);};
document.getElementById('apply').onclick=()=>refresh([document.getElementById('start').value,document.getElementById('end').value]);
(async()=>{
 try{
  if(!window.DecompressionStream)throw new Error('Use a current Chrome, Edge, Firefox, or Safari browser.');
  await Plotly.newPlot(plot,traces,layout,{responsive:true,scrollZoom:true,displaylogo:false});
  plot.on('plotly_relayout',event=>{
   if(updating)return;
   for(let row=0;row<5;row++){const axis='xaxis'+(row?row+1:'');
    const next=event[axis+'.range']||(event[axis+'.range[0]']!==undefined?[event[axis+'.range[0]'],event[axis+'.range[1]']]:null);
    if(next){clearTimeout(timer);timer=setTimeout(()=>refresh(next),120);return;}
    if(event[axis+'.autorange']){clearTimeout(timer);timer=setTimeout(()=>refresh([0,meta.duration_s]),120);return;}
   }
  });
  await refresh([0,meta.duration_s]);
 }catch(error){status.className='error';status.textContent=error.message;console.error(error);}
})();
</script></body></html>'''
