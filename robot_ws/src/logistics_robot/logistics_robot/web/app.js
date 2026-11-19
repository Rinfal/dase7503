"use strict";
const $ = id => document.getElementById(id);
let loggedIn = false, lease = "", sequence = 0, state = null, clockOffset = 0;
let held = "", sending = false, timer = null, lastStatus = 0, lastEvidence = 0, imageBusy = false, statusBusy = false;
const directions = {forward:[1,0],back:[-1,0],left:[0,1],right:[0,-1]};
const reasonNames = {emergency_stop:"已锁定",controller_feedback_stale:"控制板反馈超时",command_stale:"操作超时，已自动停车",disarmed:"等待解锁",running:"控制会话有效",armed:"已解锁"};
function message(text){$("message").textContent=text;clearTimeout(timer);timer=setTimeout(()=>$("message").textContent="",4500);}
async function api(path,body){
  const result=await fetch(path,{method:body===undefined?"GET":"POST",headers:body===undefined?{}:{"Content-Type":"application/json"},body:body===undefined?undefined:JSON.stringify(body),cache:"no-store",signal:AbortSignal.timeout(1500)});
  const data=await result.json();
  if(!result.ok){if(result.status===401)showLogin();throw new Error(data.error||"连接失败");}return data;
}
function showLogin(){loggedIn=false;lease="";held="";$("login").hidden=false;$("dashboard").hidden=true;$("connection").textContent="未登录";$("frame").removeAttribute("src");}
function showDashboard(){loggedIn=true;$("login").hidden=true;$("dashboard").hidden=false;$("password").value="";}
function render(s){
  state=s;const feedback=s.feedback_age!==null&&s.feedback_age<.5;
  $("connection").textContent="已连接";$("feedback").textContent=feedback?"在线":"未连接";
  $("battery").textContent=s.battery_raw===null?"—":String(s.battery_raw)+"（单位待核实）";
  $("armState").textContent=s.armed?"已解锁":"已锁定";
  $("driveHint").textContent=s.bench_only?"悬空调试：按住转动，松开停车。地面速度待验收。":"按住行驶，松开停车。离开页面会锁定。";
  $("gateReason").textContent=!s.drive_enabled?"底盘轮向和板端失联停车尚未验收，目前可查看画面与保存截图。":reasonNames[s.reason]||s.reason;
  $("arm").disabled=!s.drive_enabled||!feedback||s.armed;
  document.querySelectorAll("[data-drive]").forEach(b=>b.disabled=!s.armed||!lease);
  $("halt").disabled=!s.armed||!lease;
  document.querySelectorAll("[data-lift]").forEach(b=>b.disabled=!s.lift_enabled||!s.armed||!lease||s.lift_busy);
  $("liftReason").textContent=s.lift_enabled?(s.lift_busy?"升降执行中，底盘保持停车":"请先停车再升降"):"安全行程尚未标定";
  const camera=s.camera_age!==null&&s.camera_age<1;
  $("snapshot").disabled=!camera;$("cameraState").textContent=camera?"实时画面":"等待相机";
  $("frameLabel").textContent=camera?"LIVE":"离线";$("frame").hidden=!camera;$("cameraEmpty").hidden=camera;
  if(!s.armed){lease="";held="";}
}
async function update(){
  if(!loggedIn||statusBusy)return;
  statusBusy=true;
  try{const s=await api("/api/status");clockOffset=s.server_clock-performance.now()/1000;lastStatus=performance.now();render(s);
    if(performance.now()-lastEvidence>5000){lastEvidence=performance.now();void evidence();}
  }catch(e){held="";lease="";$("connection").textContent="连接中断";document.querySelectorAll("[data-drive],[data-lift],#arm,#halt").forEach(b=>b.disabled=true);$("frame").hidden=true;$("cameraEmpty").hidden=false;$("snapshot").disabled=true;}
  finally{statusBusy=false;}
}
async function drive(){
  if(!lease||sending||!loggedIn)return;
  if(performance.now()-lastStatus>400){await stop();return;}
  sending=true;const activeLease=lease;
  const [forward,angular]=directions[held]||[0,0];
  try{await api("/api/drive",{lease:activeLease,sequence:++sequence,server_clock:performance.now()/1000+clockOffset,forward,angular});}
  catch(e){lease="";held="";message(e.message);}
  finally{sending=false;}
}
async function stop(){held="";lease="";if(loggedIn){try{await api("/api/stop",{});}catch{}}void update();}
function unloadStop(){held="";lease="";if(loggedIn)fetch("/api/stop",{method:"POST",headers:{"Content-Type":"application/json"},body:"{}",keepalive:true}).catch(()=>{});}
async function evidence(){
  try{const records=await api("/api/evidence");const list=$("evidenceList");list.replaceChildren();
    if(!records.length){const p=document.createElement("p");p.className="muted";p.textContent="还没有二维码记录。将本组二维码放入相机视野。";list.append(p);}
    for(const record of records.reverse()){
      const card=document.createElement(record.url?"a":"div");card.className="evidenceItem";
      if(record.url){card.href=record.url;card.target="_blank";card.rel="noopener";const img=document.createElement("img");img.src=record.url;img.alt="二维码留证截图";img.loading="lazy";card.append(img);}
      const title=document.createElement("strong");title.textContent=record.payload||"二维码";
      const time=document.createElement("span");time.textContent=record.timestamp_utc?new Date(record.timestamp_utc).toLocaleString():"";card.append(title,time);list.append(card);
    }
  }catch(e){message(e.message);}
}
$("loginForm").addEventListener("submit",async e=>{e.preventDefault();try{await api("/api/login",{password:$("password").value});showDashboard();await update();}catch(e){message(e.message);}});
$("arm").addEventListener("click",async()=>{try{const s=await api("/api/arm",{});lease=s.lease;sequence=0;clockOffset=s.server_clock-performance.now()/1000;lastStatus=performance.now();await drive();await update();}catch(e){message(e.message);}});
document.querySelectorAll("[data-drive]").forEach(button=>{
  button.addEventListener("pointerdown",e=>{if(!lease||held)return;e.preventDefault();button.setPointerCapture(e.pointerId);held=button.dataset.drive;void drive();});
  for(const event of ["pointerup","pointercancel","lostpointercapture"]){button.addEventListener(event,()=>{held="";void drive();});}
  button.addEventListener("contextmenu",e=>e.preventDefault());
});
$("halt").addEventListener("click",()=>{held="";void drive();});$("stop").addEventListener("click",stop);
document.querySelectorAll("[data-lift]").forEach(b=>b.addEventListener("click",async()=>{held="";await drive();try{await api("/api/lift",{lease,sequence:++sequence,server_clock:performance.now()/1000+clockOffset,direction:b.dataset.lift});}catch(e){message(e.message);}void update();}));
$("snapshot").addEventListener("click",async()=>{try{const s=await api("/api/snapshot",{});const a=document.createElement("a");a.href=s.image;a.target="_blank";a.rel="noopener";a.textContent="查看已保存截图";$("snapshotResult").replaceChildren(a);message("截图已保存到树莓派");}catch(e){message(e.message);}});
$("refresh").addEventListener("click",evidence);
$("logout").addEventListener("click",async()=>{try{await api("/api/logout",{});}finally{showLogin();}});
window.addEventListener("blur",unloadStop);window.addEventListener("pagehide",unloadStop);
document.addEventListener("visibilitychange",()=>{if(document.hidden)unloadStop();});
async function frame(){if(!loggedIn||document.hidden||imageBusy||state?.camera_age===null||state?.camera_age>=1)return;imageBusy=true;$("frame").src="/api/frame?t="+Date.now();}
$("frame").addEventListener("load",()=>imageBusy=false);$("frame").addEventListener("error",()=>{imageBusy=false;$("frame").hidden=true;});
setInterval(()=>{void drive();},80);setInterval(()=>{void update();},150);setInterval(()=>{void frame();},200);
api("/api/status").then(s=>{showDashboard();render(s);void update();}).catch(()=>{});
