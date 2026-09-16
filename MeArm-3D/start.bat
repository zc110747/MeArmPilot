@echo off
rem ============================================================================
rem  ArmPilot MeArm-3D -- one-click launcher
rem ----------------------------------------------------------------------------
rem  Starts the Go backend and the Vite frontend, each in its own console window.
rem
rem  Usage:
rem      start.bat           SIM mode (default, does NOT touch hardware)
rem      start.bat --mujoco  MUJOCO mode (physics simulation, no hardware)
rem      start.bat --real    REAL mode (config.serial.yaml, moves the physical arm)
rem      start.bat --sim     force SIM mode
rem      start.bat --help    show this help
rem
rem  Design notes
rem  ------------
rem  * This file is intentionally pure ASCII. Chinese comments break the Windows
rem    GBK console parser and have bitten us before.
rem  * Model / calibration / limit truth lives ONLY in the robot package, reached
rem    through config\robots.yaml (the single selector). This script never
rem    duplicates, rewrites or overrides any of it.
rem  * REAL mode drives physical servos. The arm WILL move.
rem  * The serial port is read from backend\config.serial.yaml as-is. The script
rem    deliberately does NOT rewrite YAML: doing so from a .bat mangles UTF-8
rem    comments and CRLF line endings, and a half-parsed config is far more
rem    dangerous than a clear "port not found" from the backend.
rem  * A bare ">" inside an echo line that sits in a parenthesised block is parsed
rem    as redirection for the WHOLE block and silently kills it. Every literal ">"
rem    printed as text is therefore caret-escaped as "^>".
rem  * Same trap for "(" and ")". An UNQUOTED ")" in an echo inside an if/for
rem    block closes the block early; the leftover text then trips a syntax error
rem    such as ". was unexpected at this time". Observed 2026-09-14 on the line
rem    "the model truth (robot-package)." -- it killed the launcher before the
rem    very first preflight message. Escape as "^(x^)", or quote the whole text.
rem    (Parens inside DOUBLE QUOTES are safe; parens on a top-level echo are safe.)
rem  * The backend is rebuilt when its sources are newer than the binary, and the
rem    backend is probed via /healthz after launch. Both exist because "started"
rem    and "is actually serving" are different claims.
rem  * Exit code is 0 on success. A non-zero code means the launcher refused to
rem    start (missing tool, stale binary, failed build), not that a service died.
rem ============================================================================
setlocal EnableExtensions EnableDelayedExpansion

rem ---- Locate this script's directory ----------------------------------------
rem  %~dp0 ends with a backslash; pushd handles the trailing separator fine.
set "ROOT=%~dp0"
pushd "%ROOT%" || (echo [FAIL] Cannot enter script directory: %ROOT% & goto :end)

set "BACKEND_DIR=%ROOT%backend"
set "FRONTEND_DIR=%ROOT%frontend"
set "BACKEND_EXE=%BACKEND_DIR%\bin\armpilot-backend.exe"
set "WEB_PORT=8090"
set "FE_PORT=5273"

set "MODE=sim"
set "CFG=config.yaml"

rem ---- Parse arguments -------------------------------------------------------
:parse
if "%~1"=="" goto :parsed
if /i "%~1"=="--help" goto :usage
if /i "%~1"=="-h"     goto :usage
if /i "%~1"=="/?"     goto :usage
if /i "%~1"=="--real"   (set "MODE=real"   & set "CFG=config.serial.yaml" & shift & goto :parse)
if /i "%~1"=="--mujoco" (set "MODE=mujoco" & set "CFG=config.mujoco.yaml" & shift & goto :parse)
if /i "%~1"=="--sim"    (set "MODE=sim"    & set "CFG=config.yaml"        & shift & goto :parse)
echo [FAIL] Unknown argument: %~1
echo        Run "start.bat --help" for usage.
goto :end

:parsed
set "CFG_PATH=%BACKEND_DIR%\%CFG%"

echo.
echo ===========================================================================
echo  ArmPilot MeArm-3D launcher
echo ===========================================================================
echo  Mode      : %MODE%
echo  Backend   : %CFG%
echo.

rem ---- Preflight: required files --------------------------------------------
if not exist "%BACKEND_EXE%" (
  echo [FAIL] Backend binary missing:
  echo        %BACKEND_EXE%
  echo        Build it first:  cd backend ^&^& go build -o bin\armpilot-backend.exe .
  goto :end
)
if not exist "%CFG_PATH%" (
  echo [FAIL] Config file missing:
  echo        %CFG_PATH%
  goto :end
)
if not exist "%ROOT%config\robots.yaml" (
  echo [FAIL] Robot selector missing: config\robots.yaml
  echo        This file points the backend at the model truth ^(robot-package^).
  echo        Refusing to start: without it the wrong robot could silently load.
  goto :end
)

rem ---- Preflight: backend binary vs. its sources ------------------------------
rem  A stale binary is the nastiest failure this launcher can produce: the process
rem  starts, dies on a config error within milliseconds, and its window is gone
rem  before the message can be read. Observed 2026-09-14 -- the exe was a whole
rem  refactor behind and still looked for the retired config\robot.yaml.
rem  So: if any .go source is newer than the binary, rebuild before launching.
rem  Without Go on PATH we FAIL rather than launch something known to be stale.
rem  The comparison uses the bare "%%~tT" stamp, whose order is yyyy/MM/dd HH:mm,
rem  so plain string ordering is chronological -- no date parsing needed.
set "NEWEST_GO="
for /f "delims=" %%F in ('dir /b /a-d /s /o-d "%BACKEND_DIR%\*.go" 2^>nul') do (
  if not defined NEWEST_GO set "NEWEST_GO=%%F"
)

set "STALE="
if defined NEWEST_GO (
  for %%T in ("!NEWEST_GO!") do set "GO_TS=%%~tT"
  for %%T in ("%BACKEND_EXE%") do set "EXE_TS=%%~tT"
  if "!GO_TS!" GTR "!EXE_TS!" set "STALE=1"
)

if not defined STALE goto :after_stale

echo.
echo [0/3] Backend sources are newer than the binary -- rebuilding ...
where go >nul 2>nul
if errorlevel 1 goto :stale_no_go
pushd "%BACKEND_DIR%"
go build -o "bin\armpilot-backend.exe" .
set "BUILD_RC=!errorlevel!"
popd
if not "!BUILD_RC!"=="0" goto :stale_build_failed
echo       Rebuilt OK.
goto :after_stale

:stale_no_go
echo [FAIL] "go" is not on PATH, so the stale backend cannot be rebuilt.
echo        Refusing to launch a binary that does not match the sources.
echo        Build it manually:  cd backend ^&^& go build -o bin\armpilot-backend.exe .
goto :end

:stale_build_failed
echo [FAIL] go build failed -- backend NOT launched. Fix the errors above.
goto :end

:after_stale

where node >nul 2>nul
if errorlevel 1 (
  echo [FAIL] node not found on PATH. Install Node.js 20 or newer and retry.
  goto :end
)

rem  Prefer the locally pinned vite launcher: it guarantees the locked version and
rem  still works when a managed runtime shadows the global npm shim.
set "VITE_CMD=%FRONTEND_DIR%\node_modules\.bin\vite.cmd"
if exist "%VITE_CMD%" goto :got_vite
where npm >nul 2>nul
if errorlevel 1 (
  echo [FAIL] Neither frontend\node_modules\.bin\vite.cmd nor npm is available.
  echo        Run:  cd frontend ^&^& npm install
  goto :end
)

:got_vite

rem ---- Port preflight --------------------------------------------------------
echo [1/3] Checking ports %WEB_PORT% (backend) and %FE_PORT% (frontend) ...
set "BUSY="
for /f "tokens=5" %%P in ('netstat -ano -p TCP ^| findstr /r /c:":%WEB_PORT% .*LISTENING"') do (
  if not "%%P"=="0" set "BUSY=!BUSY! %WEB_PORT%:%%P"
)
for /f "tokens=5" %%P in ('netstat -ano -p TCP ^| findstr /r /c:":%FE_PORT% .*LISTENING"') do (
  if not "%%P"=="0" set "BUSY=!BUSY! %FE_PORT%:%%P"
)

if not defined BUSY (
  echo       Both ports are free.
  goto :after_ports
)

echo.
echo       [WARN] Stale listener^(s^) detected:
for %%B in (!BUSY!) do (
  for /f "tokens=1,2 delims=:" %%X in ("%%B") do (
    echo         port %%X  P-ID %%Y
    tasklist /fi "PID eq %%Y" /fo table /nh 2>nul
  )
)
echo.
echo       A previous run probably did not exit cleanly. If these are ArmPilot
echo       processes they can be terminated now; terminating the wrong process
echo       is possible, so this asks first.
echo.
set "ANSWER="
set /p "ANSWER=      Terminate these processes? [y/N] "
if /i not "!ANSWER!"=="y" (
  echo       Skipped. The backend or frontend may fail to bind -- check manually.
  goto :after_ports
)
for %%B in (!BUSY!) do (
  for /f "tokens=1,2 delims=:" %%X in ("%%B") do (
    echo         terminating P-ID %%Y ...
    taskkill /PID %%Y /F >nul 2>nul
  )
)
echo       Done.

:after_ports

rem ---- Resolve LAN address ---------------------------------------------------
set "LAN_IP="
for /f "tokens=1,2 delims=:" %%A in ('ipconfig ^| findstr /c:"IPv4"') do (
  if not defined LAN_IP (
    set "CAND=%%B"
    set "CAND=!CAND: =!"
    echo !CAND!| findstr /r /c:"^192\.168\." /c:"^10\." >nul
    if not errorlevel 1 set "LAN_IP=!CAND!"
  )
)

rem ---- Launch ----------------------------------------------------------------
rem  Quote pattern matters here. "cd /d "X" && prog" does NOT work: cmd strips the
rem  outer quotes, treats the first token as a directory, then tries to run a
rem  command literally named "then". Passing /d and the program as separate
rem  arguments to start avoids the whole problem.
echo.
echo [2/3] Launching services ...
if /i "%MODE%"=="mujoco" (
  echo.
  echo  +------------------------------------------------------------------+
  echo  ^|  MUJOCO MODE -- physics simulation, no hardware is touched.      ^|
  echo  ^|  Config: config.mujoco.yaml                                      ^|
  echo  ^|  Needs a python interpreter that has the mujoco package.         ^|
  echo  ^|  Close each window, or press Ctrl+C, to stop it.                 ^|
  echo  +------------------------------------------------------------------+
)
if /i "%MODE%"=="real" (
  echo.
  echo  +------------------------------------------------------------------+
  echo  ^|  REAL MODE -- this WILL move the physical arm.                   ^|
  echo  ^|  Config: config.serial.yaml                                      ^|
  echo  ^|  Close each window, or press Ctrl+C, to stop it.                 ^|
  echo  +------------------------------------------------------------------+
)

start "ArmPilot backend [%MODE%]" /d "%BACKEND_DIR%" "%BACKEND_EXE%" -c "%CFG%"

rem  ---- Frontend: auto-connect over WebSocket --------------------------------
rem  The page otherwise boots in MockTransport (in-browser sim) and sits there
rem  until you click Connect -- which is exactly the "launcher ran, backend is
rem  up, but only sim data moves" trap. These env vars make the page connect
rem  itself to the backend we just started.
rem
rem    VITE_AUTO_CONNECT=ws   -> connect the WebSocket backend (not in-page mock)
rem    VITE_AUTO_REAL=1       -> also switch to Real Robot. Only meaningful in
rem                              REAL mode; in SIM mode the backend link end is
rem                              "sim", so the switch is refused with a visible
rem                              warning (the arm is NOT touched either way).
rem
rem  VITE_WS_URL is deliberately left unset: the frontend derives the host from
rem  window.location, so LAN clients (http://<ip>:5273) reach the right machine.
set "FE_AUTO_CONNECT=ws"
set "FE_AUTO_REAL="
if /i "%MODE%"=="real" set "FE_AUTO_REAL=1"

if exist "%VITE_CMD%" (
  start "ArmPilot frontend" /d "%FRONTEND_DIR%" cmd /k "set VITE_AUTO_CONNECT=%FE_AUTO_CONNECT%&& set VITE_AUTO_REAL=%FE_AUTO_REAL%&& "%VITE_CMD%""
) else (
  start "ArmPilot frontend" /d "%FRONTEND_DIR%" cmd /k "set VITE_AUTO_CONNECT=%FE_AUTO_CONNECT%&& set VITE_AUTO_REAL=%FE_AUTO_REAL%&& npm run dev"
)

rem ---- Post-launch liveness probe ---------------------------------------------
rem  "A window appeared" is not evidence that the service came up. Ask the backend
rem  directly; if it never answers, say so HERE, where the message can be read --
rem  the backend's own console may flash past before anyone sees it.
set "HEALTH_OK="
where curl >nul 2>nul
if errorlevel 1 goto :no_curl

for /l %%I in (1,1,10) do (
  if not defined HEALTH_OK (
    ping -n 2 127.0.0.1 >nul 2>nul
    curl -s -m 2 -o nul "http://127.0.0.1:%WEB_PORT%/healthz" >nul 2>nul
    if not errorlevel 1 set "HEALTH_OK=1"
  )
)
if not defined HEALTH_OK goto :health_fail

echo       Backend answered /healthz: it is up and holds a device link.
goto :after_probe

:health_fail
echo.
echo       [WARN] Backend did NOT answer http://127.0.0.1:%WEB_PORT%/healthz.
echo              Read the backend window for the actual reason: bad config,
echo              port already taken, or the wrong device mode. The page may
echo              still load at http://localhost:%FE_PORT% but cannot drive.
echo.
goto :after_probe

:no_curl
echo       (curl not found on PATH -- skipping the backend liveness probe)

:after_probe

echo [3/3] Done.
echo.
echo  ---------------------------------------------------------------------------
echo   Backend  : http://localhost:%WEB_PORT%/healthz
echo   Frontend : http://localhost:%FE_PORT%
if defined LAN_IP echo   LAN      : http://%LAN_IP%:%FE_PORT%
echo  ---------------------------------------------------------------------------
if /i "%MODE%"=="real" (
  echo.
  echo   REAL mode next steps:
  echo     1. Wait for the backend window to print: [serial] connected ...
  echo     2. In the web UI: Connection -^> WebSocket
  echo        URL: ws://localhost:%WEB_PORT%/ws/joint then click Connect
  echo     3. Click "Real Robot" and confirm the hint reads:
  echo        "* driving real arm (link end = serial)"
  echo.
  echo   If that hint does NOT appear, the link is wrong and the arm will not
  echo   move -- check steps 1 and 2 above.
) else if /i "%MODE%"=="mujoco" (
  echo.
  echo   MUJOCO mode: physics simulation, no hardware is touched.
  echo   Link end is the MuJoCo physics server ^(a python subprocess^).
) else (
  echo.
  echo   SIM mode: no hardware is touched. Commands only move the virtual arm.
  echo   To drive the real arm, close these windows and rerun: start.bat --real
  echo   For real physics instead of the kinematic fake:   start.bat --mujoco
)
echo.
echo   Truth file: config\robots.yaml selector ^(not modified by this script^)
echo.
echo   Press any key to close this launcher window.
echo   The two service windows keep running until you close them.
pause >nul
echo   Bye.
goto :end

:usage
rem Read the default robot id from the selector instead of hardcoding a model
rem name. A hardcoded name keeps printing long after the default is switched,
rem and it is exactly the kind of "second copy of the truth" this project bans.
rem ROOT is already set above (line ~43), so this works on the --help path too.
set "DEF_ROBOT="
for /f "tokens=2" %%R in ('findstr /b /c:"default:" "%ROOT%config\robots.yaml" 2^>nul') do set "DEF_ROBOT=%%R"
if not defined DEF_ROBOT set "DEF_ROBOT=?"
echo.
echo  ArmPilot MeArm-3D launcher
echo.
echo    start.bat           SIM mode ^(default; safe, no hardware^)
echo    start.bat --mujoco  MUJOCO mode ^(physics simulation; no hardware^)
echo    start.bat --real    REAL mode ^(config.serial.yaml; the arm moves^)
echo    start.bat --sim     force SIM mode
echo    start.bat --help    this message
echo.
echo  Ports: backend %WEB_PORT%, frontend %FE_PORT%
echo  Serial port: read from backend\config.serial.yaml, edit it there
echo  Truth file : config\robots.yaml selects robot-package\%DEF_ROBOT%\model\robot.yaml
goto :end

:end
popd
endlocal
exit /b 0
