@echo off
echo ========================================
echo   Raspberry Pi Pico - USB Laufwerke
echo ========================================
echo.
echo Suche nach Pico im BOOTSEL-Modus (RPI-RP2)...
echo.
wmic logicaldisk get name, volumename | findstr /i "RPI"
echo.
echo Falls nichts angezeigt wird:
echo   1. Pico abstecken
echo   2. BOOTSEL-Taste gedrueckt halten
echo   3. USB-Kabel einstecken
echo   4. Taste loslassen
echo.
echo Der Pico erscheint dann als USB-Laufwerk "RPI-RP2"
echo.
pause
