"use strict";
// Read the multipart stream explicitly: EOF and stalled reads always reconnect.
(() => {
  class JPEGParser {
    constructor(){this.buffer=new Uint8Array();}
    push(chunk){
      const next=new Uint8Array(this.buffer.length+chunk.length);
      next.set(this.buffer);next.set(chunk,this.buffer.length);this.buffer=next;
      if(next.length>2*1024*1024)throw new Error("视频数据过大");
      const frames=[];
      while(true){
        let end=-1;
        for(let i=0;i<this.buffer.length-3;i++){
          if(this.buffer[i]===13&&this.buffer[i+1]===10&&this.buffer[i+2]===13&&this.buffer[i+3]===10){end=i;break;}
        }
        if(end<0)break;
        if(end>4096)throw new Error("视频头部异常");
        const header=new TextDecoder().decode(this.buffer.subarray(0,end));
        const match=header.match(/Content-Length:\s*(\d+)/i);
        if(!match)throw new Error("缺少视频帧长度");
        const length=Number(match[1]),start=end+4;
        if(length<2||length>1024*1024)throw new Error("视频帧长度异常");
        if(this.buffer.length<start+length+2)break;
        const jpeg=this.buffer.slice(start,start+length);
        if(jpeg[0]!==255||jpeg[1]!==216)throw new Error("视频帧格式异常");
        frames.push(jpeg);this.buffer=this.buffer.slice(start+length+2);
      }
      return frames;
    }
  }

  class VideoStream {
    constructor(image,{onState=()=>{},onUnauthorized=()=>{},retryMs=700,stallMs=2500}={}){
      this.image=image;this.onState=onState;this.onUnauthorized=onUnauthorized;
      this.retryMs=retryMs;this.stallMs=stallMs;this.running=false;this.epoch=0;
      this.hasFrame=false;this.urls=new Set();this.controller=null;
    }
    start(){if(this.running)return;this.running=true;void this.loop(++this.epoch);}
    stop(){
      this.running=false;this.epoch++;this.controller?.abort();
      for(const url of this.urls)URL.revokeObjectURL(url);
      this.urls.clear();this.hasFrame=false;this.image.removeAttribute("src");
    }
    paint(jpeg){
      const url=URL.createObjectURL(new Blob([jpeg],{type:"image/jpeg"}));
      this.urls.add(url);this.image.src=url;this.image.hidden=false;this.hasFrame=true;
      // Leave enough time for mobile browsers to decode before revocation.
      for(const old of [...this.urls])if(old!==url){
        this.urls.delete(old);setTimeout(()=>URL.revokeObjectURL(old),1000);
      }
    }
    async loop(epoch){
      while(this.running&&epoch===this.epoch){
        const controller=new AbortController();this.controller=controller;
        let reader=null,watchdog=null,lastFrame=performance.now();
        const connecting=setTimeout(()=>controller.abort(),5000);
        try{
          this.onState(this.hasFrame?"retry":"connecting");
          // Safari treats multipart/x-mixed-replace as an image resource rather
          // than exposing its bytes reliably through fetch. Parse the same
          // framed JPEG bytes using the binary streaming endpoint instead.
          const response=await fetch("/api/video?t="+Date.now(),{
            cache:"no-store",credentials:"same-origin",signal:controller.signal});
          clearTimeout(connecting);
          if(response.status===401){this.onUnauthorized();break;}
          if(!response.ok||!response.body)throw new Error("等待视频服务");
          reader=response.body.getReader();const parser=new JPEGParser();
          watchdog=setInterval(()=>{if(performance.now()-lastFrame>this.stallMs)controller.abort();},Math.min(500,this.stallMs/2));
          while(this.running&&epoch===this.epoch){
            const {done,value}=await reader.read();
            if(done)throw new Error("视频流已结束");
            const frames=parser.push(value);
            if(frames.length){
              lastFrame=performance.now();
              if(this.running&&epoch===this.epoch){this.paint(frames.at(-1));this.onState("live");}
            }
          }
        }catch(error){
          if(this.running&&epoch===this.epoch)this.onState("retry");
        }finally{
          clearTimeout(connecting);clearInterval(watchdog);controller.abort();
          if(reader)void reader.cancel().catch(()=>{});
        }
        if(this.running&&epoch===this.epoch)await new Promise(resolve=>setTimeout(resolve,this.retryMs));
      }
      if(epoch===this.epoch)this.running=false;
    }
  }
  globalThis.PhoneVideo={JPEGParser,VideoStream};
})();
