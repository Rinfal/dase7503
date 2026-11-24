"use strict";
const $ = id => document.getElementById(id);
let loggedIn = false, lease = "", sequence = 0, state = null, clockOffset = 0;
let driveFlight = Promise.resolve(), liftSending = false;
let controlVersion = Date.now()*1000;
let lastQR = "", controlUntil = 0, videoStatus = "connecting";
const videoPlayer = new PhoneVideo.VideoStream($("frame"),{
  onUnauthorized:showLogin,
  onState:status=>{
    videoStatus=status;
    $("frameLabel").textContent=status==="live"?"LIVE":"重连中";
    $("cameraEmpty").hidden=videoPlayer.hasFrame;
    if(status!=="live")$("cameraState").textContent="视频重连中…";
  }
});
let held = "", sending = false, timer = null, lastStatus = 0, lastEvidence = 0, statusBusy = false;
const directions = {forward:[1,0,0],back:[-1,0,0],left:[0,1,0],right:[0,-1,0],ccw:[0,0,1],cw:[0,0,-1]};
const simple = () => state?.direct_control || state?.manual_lock;
const reasonNames = {emergency_stop:"已锁定",controller_feedback_stale:"控制板反馈超时",command_stale:"操作超时，已自动停车",disarmed:"等待解锁",running:"控制会话有效",armed:"已解锁"};
function message(text){$("message").textContent=text;clearTimeout(timer);timer=setTimeout(()=>$("message").textContent="",4500);}
async function api(path,body){
  const result=await fetch(path,{method:body===undefined?"GET":"POST",headers:body===undefined?{}:{"Content-Type":"application/json"},body:body===undefined?undefined:JSON.stringify(body),cache:"no-store",signal:AbortSignal.timeout(1500)});
  const data=await result.json();
  if(!result.ok){if(result.status===401)showLogin();throw new Error(data.error||"连接失败");}return data;
}
function showLogin(){loggedIn=false;lease="";held="";$("login").hidden=false;$("dashboard").hidden=true;$("connection").textContent="未登录";stopStream();}
function showDashboard(){loggedIn=true;$("login").hidden=true;$("dashboard").hidden=false;$("password").value="";}
function render(s){
  state=s;const feedback=s.feedback_age!==null&&s.feedback_age<.5;
  $("connection").textContent="已连接";$("feedback").textContent=feedback?"在线":"未连接";
  $("liftTarget").textContent=s.lift_command_us&&s.lift_holding===false?"保持信号已关闭":s.lift_command_us===s.lift_pulses_us?.down?"下降目标":s.lift_command_us===s.lift_pulses_us?.up?"抬升目标":s.lift_command_us?String(s.lift_command_us)+"µs":"未下发";
  $("armState").textContent=s.direct_control?"直接操作":s.armed?"已解锁":"已锁定";
  $("arm").textContent=s.manual_lock&&s.armed?"上锁驾驶":"解锁驾驶";
  const usable=s.drive_enabled&&(s.direct_control||s.armed&&(s.manual_lock||lease));
  $("arm").hidden=!!s.direct_control;
  $("driveHint").textContent="按住行驶，松开停车；离开页面会停车。";
  $("gateReason").textContent=!s.drive_enabled?"底盘轮向和板端失联停车尚未验收，目前可查看画面与保存截图。":s.manual_lock?(s.armed?"已解锁 · 点击“上锁驾驶”才锁定。":"点击解锁后操作，动作结束保持解锁。") :s.direct_control?"直接按住方向键即可行驶，无需解锁。":reasonNames[s.reason]||s.reason;
  $("arm").disabled=!s.drive_enabled||(!s.armed&&!feedback)||(!s.manual_lock&&s.armed);
  document.querySelectorAll("[data-drive]").forEach(b=>b.disabled=!usable||s.lift_busy||liftSending||(["left","right"].includes(b.dataset.drive)&&!s.lateral_enabled));
  $("halt").disabled=!s.drive_enabled;
  document.querySelectorAll("[data-lift]").forEach(b=>b.disabled=!s.lift_enabled||!usable||s.lift_busy||liftSending);
  $("liftReason").textContent=s.lift_enabled?(s.lift_busy?"升降执行中，底盘保持停车":`下降 ${s.lift_pulses_us.down}µs · 升起 ${s.lift_pulses_us.up}µs。请先停车再升降。`):"升降未开放";
  const camera=s.camera_age!==null&&s.camera_age<1;
  $("snapshot").disabled=!camera;$("cameraState").textContent=camera&&videoStatus==="live"?`实时 · ${Math.round(s.video_fps||0)} fps`:"视频重连中…";
  $("scanState").textContent=camera?"自动扫码中":"等待相机";
  if(s.qr?.length)showQR(s.qr.at(-1));
  startStream();
  $("frame").hidden=!videoPlayer.hasFrame;$("cameraEmpty").hidden=videoPlayer.hasFrame;
  if(!s.armed&&!s.direct_control){lease="";held="";}
}
async function update(){
  if(!loggedIn||statusBusy)return;
  statusBusy=true;
  try{const s=await api("/api/status");clockOffset=s.server_clock-performance.now()/1000;lastStatus=performance.now();render(s);
    if(performance.now()-lastEvidence>1500){lastEvidence=performance.now();void evidence();}
  }catch(e){held="";lease="";$("connection").textContent="连接中断";document.querySelectorAll("[data-drive],[data-lift],#arm,#halt").forEach(b=>b.disabled=true);$("snapshot").disabled=true;}
  finally{statusBusy=false;}
}
async function drive(force=false){
  if(liftSending&&!force)return;
  if(sending)return driveFlight;
  if(!loggedIn||(!simple()&&!lease))return;
  if(state?.manual_lock&&!state.armed)return;
  if(simple()&&!force&&!held&&performance.now()>controlUntil)return;
  if(!simple()&&performance.now()-lastStatus>400){await stop();return;}
  sending=true;const activeLease=lease, activeVersion=controlVersion;
  const [forward,lateral,angular]=directions[held]||[0,0,0];
  driveFlight=(async()=>{
    try{await api("/api/drive",simple()?{forward,lateral,angular,control_version:activeVersion}:{lease:activeLease,sequence:++sequence,server_clock:performance.now()/1000+clockOffset,forward,lateral,angular});}
    catch(e){if(activeVersion===controlVersion){lease="";held="";controlUntil=0;$("connection").textContent="连接中断，请重新按键";}}
    finally{sending=false;}
  })();
  return driveFlight;
}
async function stop(){held="";lease="";controlUntil=0;controlVersion++;if(loggedIn){try{await api("/api/stop",{control_version:controlVersion});}catch{}}void update();}
function unloadStop(){held="";lease="";controlUntil=0;controlVersion++;stopStream();if(loggedIn)fetch("/api/stop",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({control_version:controlVersion}),keepalive:true}).catch(()=>{});}
async function evidence(){
  try{const records=await api("/api/evidence");const list=$("evidenceList");list.replaceChildren();
    if(!records.length){const p=document.createElement("p");p.className="muted";p.textContent="还没有二维码记录。将本组二维码放入相机视野。";list.append(p);}
    if(records.length&&!lastQR)showQR(records.at(-1),false);
    for(const record of records.reverse()){
      const card=document.createElement(record.url?"a":"div");card.className="evidenceItem";
      if(record.url){card.href=record.url;card.target="_blank";card.rel="noopener";const img=document.createElement("img");img.src=record.url;img.alt="二维码留证截图";img.loading="lazy";card.append(img);}
      const title=document.createElement("strong");title.textContent=record.payload||"二维码";
      const time=document.createElement("span");time.textContent=record.timestamp_utc?new Date(record.timestamp_utc).toLocaleString():"";card.append(title,time);list.append(card);
    }
  }catch(e){message(e.message);}
}
$("loginForm").addEventListener("submit",async e=>{e.preventDefault();try{await api("/api/login",{password:$("password").value});showDashboard();await update();}catch(e){message(e.message);}});
$("arm").addEventListener("click",async()=>{
  held="";controlUntil=0;controlVersion++;
  try{if(state?.manual_lock&&state.armed){await api("/api/lock",{control_version:controlVersion});lease="";}
    else{const s=await api("/api/arm",{control_version:controlVersion});lease=s.lease;sequence=0;clockOffset=s.server_clock-performance.now()/1000;lastStatus=performance.now();}
    await update();
  }catch(e){message(e.message);}
});
document.querySelectorAll("[data-drive]").forEach(button=>{
  button.addEventListener("pointerdown",e=>{if(!loggedIn||(state?.manual_lock&&!state.armed)||(!simple()&&!lease)||held||liftSending||state?.lift_busy)return;e.preventDefault();button.setPointerCapture(e.pointerId);held=button.dataset.drive;controlVersion++;controlUntil=performance.now()+800;void drive();});
  for(const event of ["pointerup","pointercancel","lostpointercapture"]){button.addEventListener(event,()=>{held="";controlVersion++;controlUntil=performance.now()+800;void drive(true);});}
  button.addEventListener("contextmenu",e=>e.preventDefault());
});
$("halt").addEventListener("click",()=>{held="";controlVersion++;controlUntil=performance.now()+800;void drive(true);});$("stop").addEventListener("click",stop);
document.querySelectorAll("[data-lift]").forEach(b=>b.addEventListener("click",async()=>{
  if(liftSending||!loggedIn||(state?.manual_lock&&!state.armed)||(!simple()&&!lease))return;
  liftSending=true;held="";controlVersion++;controlUntil=performance.now()+3500;
  document.querySelectorAll("[data-lift]").forEach(button=>button.disabled=true);
  try{
    await driveFlight;await drive(true);
    if(!simple()&&!lease)throw new Error("请重新解锁后再升降");
    await api("/api/lift",simple()?{direction:b.dataset.lift,control_version:controlVersion}:{lease,sequence:++sequence,server_clock:performance.now()/1000+clockOffset,direction:b.dataset.lift});
  }catch(e){message(e.message);}finally{liftSending=false;void update();}
}));
$("snapshot").addEventListener("click",async()=>{try{const s=await api("/api/snapshot",{});const a=document.createElement("a");a.href=s.image;a.target="_blank";a.rel="noopener";a.textContent="查看已保存截图";$("snapshotResult").replaceChildren(a);message("截图已保存到树莓派");}catch(e){message(e.message);}});
$("refresh").addEventListener("click",evidence);
$("logout").addEventListener("click",async()=>{try{await api("/api/logout",{control_version:++controlVersion});}finally{showLogin();}});
window.addEventListener("blur",unloadStop);window.addEventListener("pagehide",unloadStop);
document.addEventListener("visibilitychange",()=>{if(document.hidden)unloadStop();});
function startStream(){if(loggedIn&&!document.hidden)videoPlayer.start();}
function stopStream(){videoPlayer.stop();}
function showQR(record,notify=true){
  $("qrPayload").textContent=record.payload||"";
  $("qrMeta").textContent=(record.kind==="RACK"?"货架 "+record.rack:record.kind==="TEXT"?"文本二维码":record.kind||"二维码")+
    ((record.observed_at_utc||record.timestamp_utc)?" · "+new Date(record.observed_at_utc||record.timestamp_utc).toLocaleTimeString():"");
  if(lastQR&&lastQR!==record.payload&&notify)message(record.new_evidence===false?"已识别二维码，已有截图记录":"已识别新二维码并保存截图");
  lastQR=record.payload;
}
setInterval(()=>{void drive();},80);setInterval(()=>{void update();},150);
api("/api/status").then(s=>{showDashboard();render(s);void update();}).catch(()=>{});
