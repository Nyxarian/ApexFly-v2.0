@echo off
cd /d "%~dp0"
set CSC=C:\Windows\Microsoft.NET\Framework64\v4.0.30319\csc.exe
if not exist "%CSC%" (
  echo csc.exe not found
  exit /b 1
)
"%CSC%" /nologo /target:winexe /platform:anycpu /out:ApexFlyBlackboxReader.exe /r:System.dll /r:System.Core.dll /r:System.Drawing.dll /r:System.Windows.Forms.dll ApexFlyBlackboxReader.cs ApexFlyWaveform.cs
if errorlevel 1 exit /b 1
if not exist bin mkdir bin
copy /Y ApexFlyBlackboxReader.exe bin\ApexFlyBlackboxReader.exe >nul
echo OK ApexFlyBlackboxReader.exe
