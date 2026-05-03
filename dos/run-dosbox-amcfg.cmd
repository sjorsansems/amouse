@echo off
setlocal

set "DOSBOX=C:\Program Files (x86)\DOSBox-0.74-3\DOSBox.exe"
set "PROJECT=C:\Users\Sjors\OneDrive\Project\amouse - Final"
set "REALCOM=COM1"

if not "%~1"=="" set "REALCOM=%~1"

if not exist "%DOSBOX%" (
  echo DOSBox not found: "%DOSBOX%"
  exit /b 1
)

"%DOSBOX%" ^
  -c "config -set serial serial1=directserial realport:%REALCOM%" ^
  -c "mount c \"%PROJECT%\"" ^
  -c "c:" ^
  -c "cd dos"

endlocal
