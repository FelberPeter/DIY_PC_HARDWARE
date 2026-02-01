"""Test serielle Verbindung zum Pico auf COM12"""
import serial
import time

try:
    ser = serial.Serial('COM12', 115200, timeout=2)
    time.sleep(0.5)
    
    # Sende Test und warte auf Antwort
    ser.write(b'\r\n')
    time.sleep(0.3)
    
    # Lies was zurückkommt
    response = ser.read(200)
    if response:
        print(f"Pico antwortet: {response}")
    else:
        print("Keine Antwort vom Pico (vermutlich keine Firmware geladen)")
    
    ser.close()
    print("Verbindung OK - Port ist erreichbar")
    
except serial.SerialException as e:
    print(f"Fehler: {e}")
except Exception as e:
    print(f"Fehler: {e}")
