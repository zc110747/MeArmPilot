// main.js — UI 装配：连接状态、模式切换、快捷指令、回显终端、角度显示。
(function () {
  var dot = document.getElementById("conn-dot");
  var ctext = document.getElementById("conn-text");
  var term = document.getElementById("term");
  var serialText = document.getElementById("serial-text");

  function setConn(ok) {
    dot.className = "dot " + (ok ? "on" : "off");
    ctext.textContent = ok ? "已连接" : "断开，重连中…";
  }
  setConn(false);

  ArmWS.on("open", function () { setConn(true); appendLine("== WebSocket 已连接 =="); });
  ArmWS.on("close", function () { setConn(false); appendLine("== WebSocket 断开 =="); });
  ArmWS.on("err", function (msg) { appendLine("[错误] " + msg, true); });

  // 链路连接状态（服务器在接入时同步一次，之后状态翻转会主动推送）
  var serialDot = document.getElementById("serial-dot");
  var serialText = document.getElementById("serial-text");
  // 链路类型决定这一行怎么称呼自己："serial" 模式下它就是串口，
  // "network" 模式下它是到 MeArm-3D 的 TCP 链路。caps 在接入时先到。
  var linkLabel = "串口";
  ArmWS.on("caps", function (caps) {
    linkLabel = (caps && caps.mode === "network") ? "链路" : "串口";
    var dev = document.getElementById("net-device");
    if (dev && caps && caps.mode === "network") dev.textContent = "MeArm-3D";
  });

  // 三态：未连接(red) / 已连接通讯正常(green) / 已连接但通讯失败(amber)
  function setSerial(connected, errMsg, commErr, commErrMsg) {
    serialText.classList.remove("warn");
    if (!connected) {
      serialDot.className = "dot off";
      var reason = errMsg ? "（" + errMsg + "）" : "（重连中…）";
      serialText.textContent = linkLabel + ": 未连接" + reason;
      serialText.classList.add("warn");
      return;
    }
    if (commErr) {
      serialDot.className = "dot comm";
      serialText.textContent = linkLabel + ": 通讯失败" + (commErrMsg ? "（" + commErrMsg + "）" : "");
      serialText.classList.add("warn");
      return;
    }
    serialDot.className = "dot on";
    serialText.textContent = linkLabel + ": 已连接 · 通讯正常";
  }
  ArmWS.on("link_status", function (data) {
    var connected = !!(data && data.connected);
    var errMsg = data && data.serial_err ? data.serial_err : "";
    var commErr = !!(data && data.comm_err);
    var commErrMsg = data && data.comm_err_msg ? data.comm_err_msg : "";
    setSerial(connected, errMsg, commErr, commErrMsg);
    if (commErr) appendLine("[通讯] 失败: " + commErrMsg, true);
    else if (!connected && errMsg) appendLine("[" + linkLabel + "] 未连接: " + errMsg, true);
  });

  // 串口文本（服务器启动时打印，浏览器端仅展示连接状态；具体由回显体现）
  ArmWS.on("serial", function (data) {
    var line = typeof data === "string" ? data : data.line;
    if (line != null) appendLine(line); // 接入快照消息只有 angles、无 line
    if (data && data.angles) updateAngles(data.angles);
  });

  // 回显终端：30 行滑动窗口，始终显示最新（自动滚到底）。
  var TERM_MAX_LINES = 30;
  var termLines = [];

  function appendLine(text) {
    if (!term) return;
    termLines.push(text);
    if (termLines.length > TERM_MAX_LINES) {
      termLines.splice(0, termLines.length - TERM_MAX_LINES);
    }
    term.textContent = termLines.join("\n");
    term.scrollTop = term.scrollHeight;
  }

  function updateAngles(a) {
    set("a6", a.s6); set("a7", a.s7); set("a8", a.s8); set("a9", a.s9);
  }
  function set(id, v) {
    var el = document.getElementById(id);
    if (el) el.textContent = v;
  }

  // 模式切换
  var tabs = document.querySelectorAll(".tab");
  tabs.forEach(function (t) {
    t.addEventListener("click", function () {
      if (t.disabled) return;
      var mode = t.getAttribute("data-mode");
      tabs.forEach(function (x) { x.classList.remove("active"); });
      t.classList.add("active");
      document.querySelectorAll(".panel").forEach(function (p) { p.classList.remove("active"); });
      document.getElementById("panel-" + mode).classList.add("active");
    });
  });

  // 快捷指令按钮
  document.querySelectorAll(".btns button").forEach(function (b) {
    b.addEventListener("click", function () {
      var c = b.getAttribute("data-cmd");
      if (c) ArmWS.sendCmd(c);
    });
  });
})();
