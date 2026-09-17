// wsclient.js — 浏览器端 WebSocket 封装。
// 对外暴露 window.ArmWS：on(type, fn) / sendJoy / sendCmd / sendXYZ / sendGrip / sendServo / sendRefresh
//
// 消息类型（服务器 -> 浏览器，JSON）：
//   {t:"serial", line:"...", angles?:{s6,s7,s8,s9,ok}}   链路回显（串口模式）
//   {t:"caps", caps:{mode,arm_device_cmds,xyz,gripper,servo_direct,refresh_state,servo_ids}}
//   {t:"link_status", connected, serial_err, comm_err, comm_err_msg}
//   {t:"state", angles?:{...}, tcp?:[x,y,z], device:"sim|serial|mujoco"}   网络模式状态快照
//   {t:"err", msg:"..."}   {t:"pong"}
(function () {
  var url = (location.protocol === "https:" ? "wss://" : "ws://") + location.host + "/ws";
  var ws = null;
  var handlers = { open: [], close: [], serial: [], err: [] };

  function on(type, fn) {
    (handlers[type] || (handlers[type] = [])).push(fn);
  }
  function emit(type, data) {
    (handlers[type] || []).forEach(function (f) { f(data); });
  }

  function connect() {
    try { ws = new WebSocket(url); }
    catch (e) { setTimeout(connect, 1500); return; }

    ws.onopen = function () {
      emit("open");
      send({ t: "ping" });
    };
    ws.onclose = function () {
      emit("close");
      setTimeout(connect, 1500);
    };
    ws.onerror = function () {};
    ws.onmessage = function (ev) {
      var m;
      try { m = JSON.parse(ev.data); } catch (e) { return; }
      if (!m || !m.t) return;
      if (m.t === "serial") {
        emit("serial", m); // 完整对象：main.js 统一解析 line / angles
      } else if (m.t === "link_status") {
        emit("link_status", m);
      } else if (m.t === "caps") {
        emit("caps", m.caps || {});
      } else if (m.t === "state") {
        emit("state", m);
      } else if (m.t === "err") {
        emit("err", m.msg);
      } else if (m.t === "pong") {
        emit("pong", m);
      }
    };
  }

  function send(obj) {
    if (ws && ws.readyState === 1) ws.send(JSON.stringify(obj));
  }

  window.ArmWS = {
    on: on,
    sendJoy: function (side, x, y) { send({ t: "joy", side: side, x: x, y: y }); },
    sendCmd: function (c) { send({ t: "cmd", c: c }); },
    // ---- 网络链路（MeArm-3D TCP）专属 ----
    sendXYZ: function (axis, direction, step) {
      send({ t: "xyz", axis: axis, direction: direction, step: step });
    },
    sendGrip: function (action) { send({ t: "grip", action: action }); },
    sendServo: function (n, angle) { send({ t: "servo", servo: n, angle: angle }); },
    sendRefresh: function () { send({ t: "refresh" }); },
    isReady: function () { return !!(ws && ws.readyState === 1); },
  };

  connect();
})();
