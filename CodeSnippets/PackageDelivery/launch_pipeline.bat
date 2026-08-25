@echo off
set SCRIPTS_DIR=D:\Projects\Uni\Master\bioinspired\CodeSnippets\PackageDelivery
set VENV_ACTIVATE=D:\Projects\Uni\Master\bioinspired\CodeSnippets\Venv\Scripts\activate.bat

echo Launching Multi-Tab Pipeline in Bottom-Left Quarter...

:: Launch Windows Terminal with tabs
start "" wt new-tab --title "SLAM Brain" cmd /k "call \"%VENV_ACTIVATE%\" && cd /d \"%SCRIPTS_DIR%\" && python py_brain.py" ; ^
new-tab --title "Stereo Vision" cmd /k "call \"%VENV_ACTIVATE%\" && cd /d \"%SCRIPTS_DIR%\" && python py_stereo.py" ; ^
new-tab --title "Digital Twin" cmd /k "call \"%VENV_ACTIVATE%\" && cd /d \"%SCRIPTS_DIR%\" && python py_visualizer.py"

:: Dynamically move & scale Windows Terminal to exact bottom-left 50%
powershell -NoProfile -Command ^
  "Add-Type -AssemblyName System.Windows.Forms;" ^
  "Add-Type -TypeDefinition 'using System; using System.Runtime.InteropServices; public class Win32 { [DllImport(\"user32.dll\")] public static extern bool MoveWindow(IntPtr hWnd, int X, int Y, int nWidth, int nHeight, bool bRepaint); }';" ^
  "Start-Sleep -Milliseconds 600;" ^
  "$proc = Get-Process -Name WindowsTerminal -ErrorAction SilentlyContinue | Select-Object -Last 1;" ^
  "if ($proc) { $wa = [System.Windows.Forms.Screen]::PrimaryScreen.WorkingArea; [Win32]::MoveWindow($proc.MainWindowHandle, $wa.Left, $wa.Top + [int]($wa.Height / 2), [int]($wa.Width / 2), [int]($wa.Height / 2), $true); }"