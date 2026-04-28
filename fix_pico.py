import os

file_to_remove = ".pio/libdeps/pico/IRremoteESP8266/src/IRrecv.cpp"
if os.path.exists(file_to_remove):
    try:
        os.remove(file_to_remove)
        print("====== NAM'S ESP: DA XOA IRRECV.CPP DE CHAY PICO ======")
    except Exception as e:
        print(f"Loi xoa file: {e}")
