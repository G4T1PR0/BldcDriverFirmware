from scipy.io import wavfile
import serial
import time
import struct

ser = serial.Serial("/dev/tty.usbmodem1202", 1843200)


def main():
    print("Start")
    # samplerate, data = wavfile.read("./output25000mono.wav")
    samplerate, data = wavfile.read("./zako.wav")

    print("sample rate:", samplerate)
    # for i in range(5000, 5100):
    #     print(data[i])

    # print("End ######################################")

    cnt = 5000
    rx_data = ""

    send = True

    # for i in range(5000, 5100):
    #     send_data = data[cnt]
    #     a = struct.pack(">BBh", 0xFF, 0xFD, send_data)
    #     ser.write(a)

    while True:
        if ser.in_waiting > 0:
            rx_data = ser.read().decode("ascii")

            if rx_data == "x":
                send = False
            elif rx_data == "q":
                send = True

            print(rx_data, end="")
        else:
            # time.sleep(0.1)

            if send == True:
                send_data = data[cnt]

                a = struct.pack(">BBBh", 0xFF, 0xFD, 00, send_data)
                # print("                  send:                    ", send_data)
                ser.write(a)

                cnt += 1
                if cnt >= len(data):
                    cnt = 0


main()
