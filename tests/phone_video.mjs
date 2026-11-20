import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import vm from 'node:vm';
import {Blob} from 'node:buffer';
import {TextDecoder,TextEncoder} from 'node:util';
import {performance} from 'node:perf_hooks';
import {setTimeout,clearTimeout,setInterval,clearInterval} from 'node:timers';

class TestAbortController{
  constructor(){this.listeners=[];this.signal={aborted:false,addEventListener:(_,f)=>this.listeners.push(f)};}
  abort(){if(this.signal.aborted)return;this.signal.aborted=true;for(const listener of this.listeners)listener();}
}

export async function testPhoneVideo(){
  const source=await readFile(new URL('../robot_ws/src/logistics_robot/logistics_robot/web/video.js',import.meta.url),'utf8');
  let attempts=0,nextURL=0;
  const statuses=[],image={src:'',hidden:true,removeAttribute(name){delete this[name];}};
  const jpeg=new Uint8Array([255,216,1,2,255,217]);
  const header=new TextEncoder().encode(`--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ${jpeg.length}\r\n\r\n`);
  const packet=new Uint8Array(header.length+jpeg.length+2);packet.set(header);packet.set(jpeg,header.length);packet.set([13,10],header.length+jpeg.length);
  const context=vm.createContext({Uint8Array,TextDecoder,Blob,AbortController:TestAbortController,performance,console,
    setTimeout,clearTimeout,setInterval,clearInterval,
    URL:{createObjectURL:()=>`blob:test-${++nextURL}`,revokeObjectURL(){}},
    fetch:async(url,{signal})=>{
      assert.ok(url.startsWith('/api/video?'), 'Safari must fetch binary framed JPEGs');
      const attempt=++attempts;let reads=0;
      return {ok:true,status:200,body:{getReader:()=>({
        async read(){
          if(attempt===1)return {done:true}; // Clean EOF, no browser error event.
          if(reads++===0)return {done:false,value:packet};
          return new Promise((resolve,reject)=>{
            if(signal.aborted){reject(new Error('aborted'));return;}
            signal.addEventListener('abort',()=>reject(new Error('aborted')),{once:true});
          });
        },async cancel(){}
      })}};
    }
  });
  vm.runInContext(source,context);
  const {JPEGParser,VideoStream}=context.PhoneVideo;
  const parser=new JPEGParser();let decoded=[];
  for(let index=0;index<packet.length;index+=3)decoded.push(...parser.push(packet.slice(index,index+3)));
  assert.equal(decoded.length,1);assert.deepEqual([...decoded[0]],[...jpeg]);
  const both=new Uint8Array(packet.length*2);both.set(packet);both.set(packet,packet.length);
  assert.equal(parser.push(both).length,2);
  assert.throws(()=>new JPEGParser().push(new Uint8Array(2*1024*1024+1)));
  const player=new VideoStream(image,{retryMs:10,stallMs:50,onState:s=>statuses.push(s)});
  player.start();
  const deadline=performance.now()+1500;
  while(performance.now()<deadline&&attempts<3)await new Promise(r=>setTimeout(r,10));
  try{
    assert.ok(attempts>=3,'must recover from both clean EOF and stalled reads');
    assert.ok(player.hasFrame);assert.ok(image.src.startsWith('blob:'));
    assert.ok(statuses.includes('live')&&statuses.includes('retry'));
  }finally{player.stop();}
  assert.equal(image.src,undefined);assert.equal(player.running,false);
  console.log('Video: chunk parsing, multiple frames, bounded memory, EOF reconnect, stall watchdog, retained last image, stop PASS');
}
