// netpanel.js — 网络链路（MeArm-3D TCP 控制接口）专属面板。
//
// 设计原则：**只新增，不改造**。
//   - 面板区块是 index.html 里新增的一段，串口模式下整块 `hidden`；
//   - 显示哪几项由服务器推来的 `caps` 决定，而不是靠页面猜；
//   - 原有的摇杆、角度面板、快捷指令、日志区一个都没改（快捷指令在网络链路下
//     只**置灰**，不删除 —— 它们在串口模式下必须照旧可用）。
//
// 为什么快捷指令要置灰：RESET / STATUS / JOYHW 是 arm-device **固件**概念，
// MeArm-3D 的 TCP 协议（move / gripper / servo / state）里没有对应物。
// 留一个点了没反应的按钮比置灰更糟。
(function () {
  var panel = document.getElementById("net-panel");
  var rows = document.getElementById("servo-rows");
  if (!panel || !rows) return;

  var caps = {};
  // sliders: TCP 编号 -> {armId, range, val}
  var sliders = {};
  // 正在拖动的滑条编号（拖动期间不被服务端状态回推覆盖，否则手感会"打架"）
  var dragging = null;

  function byId(id) { return document.getElementById(id); }

  // ---- 能力声明：决定显示哪些控件 ----------------------------------------
  ArmWS.on("caps", function (c) {
    caps = c || {};
    if (caps.mode !== "network") {
      panel.hidden = true;
      return;
    }
    panel.hidden = false;

    byId("xyz-btns").hidden = !caps.xyz;
    byId("grip-btns").hidden = !caps.gripper;
    var refresh = byId("net-refresh");
    if (refresh) refresh.hidden = !caps.refresh_state;
    var stepRow = panel.querySelector(".net-row");
    if (stepRow) stepRow.hidden = !caps.xyz;

    // arm-device 文本指令（固件概念）：网络链路不支持 → 置灰 + 说明原因
    var armBtns = document.querySelectorAll("#arm-btns button");
    Array.prototype.forEach.call(armBtns, function (b) {
      b.disabled = !caps.arm_device_cmds;
      if (!caps.arm_device_cmds) {
        b.title = "网络链路不支持 arm-device 固件指令（它是下位机概念）";
      } else {
        b.removeAttribute("title");
      }
    });
    var hint = byId("cmd-hint");
    if (hint) hint.textContent = caps.arm_device_cmds ? "" : "（仅串口模式）";

    if (caps.servo_direct) buildServoRows(caps.servo_ids || []);
  });

  // ---- 状态快照（服务端真值）--------------------------------------------
  ArmWS.on("state", function (m) {
    if (!m) return;
    if (m.angles) applyAngles(m.angles);
    var tcp = byId("net-tcp");
    if (tcp) {
      tcp.textContent = (m.tcp && m.tcp.length === 3)
        ? m.tcp.map(function (v) { return v.toFixed(1); }).join(", ") + " mm"
        : "--";
    }
    var dev = byId("net-device");
    if (dev && m.device) dev.textContent = "MeArm-3D · " + m.device;
  });

  // ---- 舵机直控滑条（按接线表动态生成）----------------------------------
  function buildServoRows(servoIds) {
    rows.innerHTML = "";
    sliders = {};
    if (!servoIds.length) {
      rows.innerHTML = '<div class="hint">服务端未提供舵机接线表</div>';
      return;
    }
    servoIds.forEach(function (armId, i) {
      var n = i + 1; // TCP 编号（MeArm-3D 的 servo 字段）
      var row = document.createElement("div");
      row.className = "servo-row";

      var name = document.createElement("span");
      name.className = "servo-name";
      name.textContent = "S" + armId;

      var range = document.createElement("input");
      range.type = "range";
      range.min = "0";
      range.max = "180";
      range.step = "1";
      range.value = "90";
      range.title = "TCP servo " + n + "（arm-device S" + armId + "）";

      var val = document.createElement("span");
      val.className = "servo-val";
      val.textContent = "90";

      range.addEventListener("pointerdown", function () { dragging = n; });
      range.addEventListener("input", function () { val.textContent = range.value; });
      // change 只在松手时触发一次 ⇒ 天然节流，不需要额外去抖
      range.addEventListener("change", function () {
        val.textContent = range.value;
        dragging = null;
        ArmWS.sendServo(n, parseFloat(range.value));
      });

      row.appendChild(name);
      row.appendChild(range);
      row.appendChild(val);
      rows.appendChild(row);
      sliders[n] = { armId: armId, range: range, val: val };
    });
  }

  window.addEventListener("pointerup", function () { dragging = null; });

  // 服务端状态回推：滑条跟随（除正在拖的那一根）
  function applyAngles(a) {
    setText("a6", a.s6);
    setText("a7", a.s7);
    setText("a8", a.s8);
    setText("a9", a.s9);
    Object.keys(sliders).forEach(function (key) {
      var n = parseInt(key, 10);
      var s = sliders[key];
      if (dragging === n) return;
      var v = angleOf(a, s.armId);
      if (typeof v !== "number") return;
      s.range.value = String(v);
      s.val.textContent = String(v);
    });
  }

  function angleOf(a, armId) {
    if (armId === 6) return a.s6;
    if (armId === 7) return a.s7;
    if (armId === 8) return a.s8;
    if (armId === 9) return a.s9;
    return undefined;
  }

  function setText(id, v) {
    var el = byId(id);
    if (el && v != null) el.textContent = v;
  }

  // ---- 按钮 --------------------------------------------------------------
  Array.prototype.forEach.call(document.querySelectorAll("#xyz-btns button"), function (b) {
    b.addEventListener("click", function () {
      var v = b.getAttribute("data-xyz") || "";
      if (v.length !== 2) return;
      var stepEl = byId("xyz-step");
      var step = stepEl ? parseFloat(stepEl.value) : NaN;
      if (!(step > 0)) return;
      ArmWS.sendXYZ(v.charAt(0), v.charAt(1), step);
    });
  });

  Array.prototype.forEach.call(document.querySelectorAll("#grip-btns button"), function (b) {
    b.addEventListener("click", function () {
      var action = b.getAttribute("data-grip");
      if (action) ArmWS.sendGrip(action);
    });
  });

  var refreshBtn = byId("net-refresh");
  if (refreshBtn) {
    refreshBtn.addEventListener("click", function () { ArmWS.sendRefresh(); });
  }
})();
