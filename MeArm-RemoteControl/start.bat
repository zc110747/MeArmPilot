@echo off
rem ============================================================================
rem  ArmPilot MeArm-RemoteControl -- one-click launcher
rem ----------------------------------------------------------------------------
rem  Two transports, selected here and handed to the program as one -mode flag:
rem
rem      start.bat             NETWORK mode (default): TCP Client -> MeArm-3D.
rem                            No serial port is opened by this service. The
rem                            physical arm is only moved if MeArm-3D itself is
rem                            running in REAL mode.
rem      start.bat --real      SERIAL mode: opens the serial port to the
rem                            arm-device firmware. THIS DRIVES THE PHYSICAL ARM.
rem      start.bat --serial    force SERIAL mode (alias of --real)
rem      start.bat --network   force NETWORK mode (same as the default)
rem      start.bat -c FILE     use FILE instead of config.yaml
rem      start.bat --build     rebuild before launching
rem      start.bat --no-build  never rebuild (fast path)
rem      start.bat --help      show this message
rem
rem  Design notes
rem  ------------
rem  * Pure ASCII on purpose: Chinese comments break the Windows GBK console
rem    parser and have bitten us before.
rem  * The "network by default" choice lives HERE, not in the YAML. config.yaml
rem    keeps mode: "serial" so that a bare `arm-web.exe -c config.yaml` behaves
rem    exactly as it did before this launcher existed. --real / --network are
rem    translated into a single `-mode` argument. The YAML is never rewritten.
rem  * This script does NOT edit config.yaml. Rewriting YAML from a .bat mangles
rem    UTF-8 comments and CRLF endings, and a half-written config is far more
rem    dangerous than a clear error message from the program.
rem  * Neither the web port nor the MeArm-3D TCP endpoint is duplicated here.
rem    Both endpoints are printed by the programs themselves, arm-web reports
rem    the link state live in the web UI, and the TCP client retries on its own.
rem    A second copy of those numbers inside a .bat is exactly the duplicated
rem    truth this project bans -- and it would silently rot the day someone
rem    edits config.yaml. So: no port preflight, no liveness probe.
rem  * arm-web runs in the FOREGROUND of this window. Its log (link handshake,
rem    reconnect backoff, serial ACK state) is the primary diagnostic and must
rem    not be lost in a window that flashed past. Ctrl+C stops it, then you are
rem    back here and the log is still on screen.
rem  * A bare ">" or "|" inside an echo that sits in a parenthesised block is
rem    parsed as redirection for the WHOLE block and silently kills it. Every
rem    literal one printed as text is caret-escaped. Same trap for an unquoted
rem    ")" inside a block, which closes it early. Both were observed in
rem    MeArm-3D\start.bat; the same discipline is applied here.
rem  * Every exit path ends in PAUSE (delivery contract): a double-clicked
rem    window must not vanish before the reason can be read. When stdin is not
rem    an interactive console (pipes, automation) the pause returns at once, so
rem    this does not block scripted use.
rem  * Exit code 0 = arm-web ran and exited normally. Non-zero = this launcher
rem    refused to start (bad argument, missing binary, failed build), NOT that a
rem    service died.
rem ============================================================================
setlocal EnableExtensions EnableDelayedExpansion

rem ---- Locate this script's directory ----------------------------------------
rem  %~dp0 ends with a backslash; pushd handles the trailing separator fine.
set "ROOT=%~dp0"
pushd "%ROOT%" || (echo [FAIL] Cannot enter script directory: %ROOT% & set "RC=1" & goto :end)

set "EXE=arm-web.exe"
set "CFG=config.yaml"
set "MODE=network"
set "SKIP_BUILD="
set "FORCE_BUILD="
set "RC=0"

rem ---- Parse arguments -------------------------------------------------------
:parse
if "%~1"=="" goto :parsed
if /i "%~1"=="--help"     goto :usage
if /i "%~1"=="-h"         goto :usage
if /i "%~1"=="/?"         goto :usage
if /i "%~1"=="--real"     (set "MODE=serial"    & shift & goto :parse)
if /i "%~1"=="--serial"   (set "MODE=serial"    & shift & goto :parse)
if /i "%~1"=="--network"  (set "MODE=network"   & shift & goto :parse)
if /i "%~1"=="--build"    (set "FORCE_BUILD=1"  & shift & goto :parse)
if /i "%~1"=="--no-build" (set "SKIP_BUILD=1"   & shift & goto :parse)
if /i "%~1"=="-c"         goto :arg_cfg
if /i "%~1"=="--config"   goto :arg_cfg
echo [FAIL] Unknown argument: %~1
echo        Run "start.bat --help" for usage.
set "RC=2"
goto :end

:arg_cfg
if "%~2"=="" goto :err_cfg
set "CFG=%~2"
shift
shift
goto :parse

:err_cfg
echo [FAIL] %~1 needs a file path, e.g.   start.bat -c config.yaml
set "RC=2"
goto :end

:parsed

rem ---- Banner ----------------------------------------------------------------
echo.
echo ===========================================================================
echo  ArmPilot MeArm-RemoteControl launcher
echo ===========================================================================
if /i "%MODE%"=="network" (
  echo  Mode   : NETWORK -- TCP Client to MeArm-3D, no serial port opened
) else (
  echo  Mode   : SERIAL -- serial port to the arm-device firmware
)
echo  Config : %CFG%
echo  Binary : %EXE%
echo.

if /i "%MODE%"=="network" (
  echo  ---------------------------------------------------------------------------
  echo   NETWORK mode
  echo.
  echo    Web joystick -^> arm-web -^> TCP Client -^> MeArm-3D -^> Controller
  echo.
  echo   * Start MeArm-3D FIRST ^(its own start.bat; SIM mode is enough^).
  echo     Without it arm-web keeps retrying and the UI shows the link as down.
  echo   * This service does no IK/FK, no coordinate maths and no limit checks:
  echo     it only sends servo / move / gripper / state to MeArm-3D.
  echo   * Target host and port come from %CFG% ^(network.host / network.port^).
  echo   * The port this page is served on is printed by arm-web below.
  echo  ---------------------------------------------------------------------------
) else (
  echo  +------------------------------------------------------------------+
  echo  ^|  SERIAL mode -- this WILL move the physical arm.                 ^|
  echo  ^|  No TCP client is created in this mode.                          ^|
  echo  +------------------------------------------------------------------+
  echo   Serial port comes from %CFG% ^(serial.port^, default COM4 / 115200 8N1^).
)

rem ---- Preflight: config file ------------------------------------------------
if exist "%CFG%" goto :cfg_ok
echo [FAIL] Config file missing: %CFG%
echo        Copy config.yaml.example to config.yaml, then adjust it.
set "RC=1"
goto :end
:cfg_ok

rem ---- Build when the binary lags behind its sources --------------------------
rem  A stale binary is the nastiest failure this launcher can produce: the
rem  process starts, dies on a config error within milliseconds, and the reason
rem  scrolls away. Sources = every *.go (recursively) AND every embedded web
rem  asset under web\static (they are baked in via go:embed, so a JS edit
rem  without a rebuild ships an old page).
rem  The comparison uses the bare "%%~tT" stamp, whose order is yyyy/MM/dd HH:mm,
rem  so plain string ordering is chronological -- no date parsing needed.
if defined SKIP_BUILD (
  echo [1/3] --no-build: skipping the up-to-date check.
  goto :check_exe
)

echo [1/3] Checking whether %EXE% is up to date ...

set "NEWEST_GO="
for /f "delims=" %%F in ('dir /b /a-d /s /o-d "*.go" 2^>nul') do if not defined NEWEST_GO set "NEWEST_GO=%%F"
set "NEWEST_WEB="
for /f "delims=" %%F in ('dir /b /a-d /s /o-d "web\static\*" 2^>nul') do if not defined NEWEST_WEB set "NEWEST_WEB=%%F"

set "STALE="
if not exist "%EXE%" set "STALE=1"
if defined NEWEST_GO (
  for %%T in ("!NEWEST_GO!") do set "GO_TS=%%~tT"
  for %%T in ("%EXE%") do set "EXE_TS=%%~tT"
  if "!GO_TS!" GTR "!EXE_TS!" set "STALE=1"
)
if defined NEWEST_WEB (
  for %%T in ("!NEWEST_WEB!") do set "WEB_TS=%%~tT"
  for %%T in ("%EXE%") do set "EXE_TS2=%%~tT"
  if "!WEB_TS!" GTR "!EXE_TS2!" set "STALE=1"
)
if defined FORCE_BUILD set "STALE=1"

if not defined STALE (
  echo       Up to date.
  goto :check_exe
)

echo       Sources are newer than %EXE% -- rebuilding.
where go >nul 2>nul
if errorlevel 1 goto :no_go

rem Same environment as build.bat: dependencies resolve from the local module
rem cache, so the build works on a machine without network access.
set "GOFLAGS=-mod=mod"
set "GOPROXY=off"
set "GOSUMDB=off"
go build -o "%EXE%" .
if errorlevel 1 goto :build_failed
echo       Rebuilt OK.

:check_exe
if not exist "%EXE%" goto :no_exe

rem ---- Launch ----------------------------------------------------------------
rem  Foreground on purpose: see the header. -mode overrides the YAML value.
echo [2/3] Launching arm-web ^(Ctrl+C stops it; you return to this window^) ...
echo.
"%EXE%" -c "%CFG%" -mode "%MODE%"
set "RC=!errorlevel!"
echo.
echo [3/3] arm-web exited with code !RC!.
if not "!RC!"=="0" echo       Read the log above for the actual reason.
goto :end

rem ---- Failure paths ---------------------------------------------------------
:no_go
echo [FAIL] %EXE% is out of date but "go" is not on PATH, so it cannot be rebuilt.
echo        Refusing to launch a binary that does not match its sources.
echo        Build it once on a machine with Go:  build.bat
echo        Or accept the existing binary as-is:  start.bat --no-build
set "RC=3"
goto :end

:build_failed
echo [FAIL] go build failed -- arm-web NOT launched. Fix the errors above.
set "RC=3"
goto :end

:no_exe
echo [FAIL] Binary missing: %ROOT%%EXE%
echo        Build it first:  build.bat    ^(or run start.bat without --no-build^)
set "RC=3"
goto :end

rem ---- Usage -----------------------------------------------------------------
:usage
echo.
echo  ArmPilot MeArm-RemoteControl launcher
echo.
echo    start.bat                NETWORK mode ^(default; TCP Client to MeArm-3D^)
echo    start.bat --real         SERIAL mode ^(opens the serial port; arm moves^)
echo    start.bat --serial       force SERIAL mode ^(alias of --real^)
echo    start.bat --network      force NETWORK mode ^(same as the default^)
echo    start.bat -c FILE        use FILE instead of config.yaml
echo    start.bat --build        rebuild before launching
echo    start.bat --no-build     never rebuild
echo    start.bat --help         this message
echo.
echo  The MeArm-3D endpoint is config.yaml ^(network.host / network.port^); the
echo  serial port is config.yaml ^(serial.port^). This script rewrites neither.
echo.
echo  Note: --real here means "this service uses the serial port". It is NOT the
echo  same switch as MeArm-3D --real, which makes THAT service drive the arm.
goto :end

:end
popd
pause
endlocal & exit /b %RC%
