@echo off
setlocal
set HALSIM_EXTENSIONS=C:\Users\jason\Documents\FRC\2021_UARobotics_Infinite_Recharge\SimCharacterization\characterization-project\build\tmp\expandedArchives\halsim_gui-2021.1.2-windowsx86-64.zip_a65f029882460810c7d9fdb675bf2f07\windows\x86-64\shared\halsim_gui.dll
set PATH=C:\Users\jason\Documents\FRC\2021_UARobotics_Infinite_Recharge\SimCharacterization\characterization-project\build\tmp\jniExtractDir;C:\Users\jason\Documents\FRC\2021_UARobotics_Infinite_Recharge\SimCharacterization\characterization-project\build\tmp\jniExtractDir
"C:\Users\Public\wpilib\2021\jdk\bin\java.exe" -Djava.library.path="C:\Users\jason\Documents\FRC\2021_UARobotics_Infinite_Recharge\SimCharacterization\characterization-project\build\tmp\jniExtractDir" -jar "C:\Users\jason\Documents\FRC\2021_UARobotics_Infinite_Recharge\SimCharacterization\characterization-project\build\libs\characterization-project.jar"
endlocal
