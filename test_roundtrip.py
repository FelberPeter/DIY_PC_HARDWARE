"""
Roundtrip Test: PC -> Pico -> ESP Receiver -> ESP Sender -> zurück
Misst die Latenz der gesamten Strecke
"""
import serial
import time

# Pico ist auf COM14
PICO_PORT = 'COM14'
BAUD_RATE = 115200

def test_roundtrip():
    print("=" * 50)
    print("ROUNDTRIP TEST")
    print("PC -> Pico -> ESP Receiver -> ESP Sender -> zurück")
    print("=" * 50)
    
    try:
        ser = serial.Serial(PICO_PORT, BAUD_RATE, timeout=5)
        time.sleep(2)  # Warte auf Pico Boot
        
        # Leere Buffer
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        print(f"\nVerbunden mit Pico auf {PICO_PORT}")
        print("Sende Test-Pakete...\n")
        
        test_messages = [
            "PING",
            "Hello ESP-NOW!",
            "Test123",
            "LatencyTest"
        ]
        
        success = 0
        total_latency = 0
        
        for msg in test_messages:
            # Sende Nachricht
            start_time = time.perf_counter()
            ser.write((msg + "\n").encode())
            
            # Warte auf Antwort
            response = ""
            timeout_start = time.time()
            while time.time() - timeout_start < 3:
                if ser.in_waiting:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        response = line
                        if "ECHO:" in line:
                            break
            
            end_time = time.perf_counter()
            latency = (end_time - start_time) * 1000  # ms
            
            if "ECHO:" in response:
                print(f"✅ Gesendet: '{msg}' -> Empfangen: '{response}' ({latency:.2f}ms)")
                success += 1
                total_latency += latency
            else:
                print(f"❌ Gesendet: '{msg}' -> Keine Antwort (Timeout)")
                # Debug: Zeige was empfangen wurde
                remaining = ser.read(500).decode('utf-8', errors='ignore')
                if remaining:
                    print(f"   Debug: {remaining[:100]}...")
            
            time.sleep(0.5)
        
        print("\n" + "=" * 50)
        print(f"Ergebnis: {success}/{len(test_messages)} erfolgreich")
        if success > 0:
            print(f"Durchschnittliche Latenz: {total_latency/success:.2f}ms")
        print("=" * 50)
        
        ser.close()
        
    except serial.SerialException as e:
        print(f"Fehler: {e}")
    except Exception as e:
        print(f"Fehler: {e}")

if __name__ == "__main__":
    test_roundtrip()
