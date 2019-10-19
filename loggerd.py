#!/usr/bin/env python3
# This code is written for resting state fMRI project in Center for Animal MRI at UNC at Chapel Hill
# Author: SungHo Lee (shlee@unc.edu)

# Module importing
import serial
import socket, time
from signal import pause
import threading
from gpiozero import Button
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn

# setup MCP3008 - adafruit CircuitPython for Analog signal logging
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)
cs = digitalio.DigitalInOut(board.D8)
mcp = MCP.MCP3008(spi, cs)
ch0 = AnalogIn(mcp, MCP.P0)
ch1 = AnalogIn(mcp, MCP.P1)

triggered = threading.Event()
logged = threading.Event()
collect_temp = threading.Event()
stop_event = threading.Event()


def clear_job():
    global collect_temp
    global triggered
    
    collect_temp.clear()
    triggered.clear()


def data_logger():
    global logged_data
    global logged
    global triggered
    global stop_event
    global tt_data
    global init_ts

    triggered.wait()
    resolution = (1.0/sfreq)

    while not stop_event.is_set():
        triggered.clear()
        trig_ts = time.perf_counter()
        tt_data.append(trig_ts - init_ts)

        for n in range(int(sfreq * tr)-1):
            curr_ts = time.perf_counter()
            if curr_ts - trig_ts > tr - resolution:
                break
            logged_data['ts'].append(curr_ts - init_ts)
            logged_data['vsp'].append(ch0.value)
            logged_data['rsp'].append(ch1.value)
            logged_data['btm'].append(bodytemp) # need to remove this line if serial signal will not be used

            sleep_time = resolution - (time.perf_counter() - curr_ts)
            if sleep_time > 0 and sleep_time < resolution:
                time.sleep(sleep_time)
            else:
                pass
        logged.set()
        if len(tt_data) > 10:   # for debug
            stop_event.set()
            break
        triggered.wait()


def trigger_monitor():
    global dummy
    global trig_obj
    global stop_event
    global triggered
    global logged

    dummy_total = int(dummy)
    timeout = tr * 2

    trig_obj.wait_for_press()

    while dummy > 0:
        trig_obj.wait_for_release()
        print('Dummy trigger count: {}/{}'.format(dummy, dummy_total))
        dummy -= 1
        trig_obj.wait_for_press()

    trig_obj.wait_for_release()
    start_time = time.perf_counter()
    while not stop_event.is_set():
        trig_obj.wait_for_press(timeout=timeout)
        triggered.set()
        collect_temp.set()
        if time.perf_counter() - start_time > tr * 2:
            break
        trig_obj.wait_for_release()

        logged.wait()
        logged.clear()
        start_time = time.perf_counter()
    stop_event.set()


def update_bodytemp():
    # This is our temperature monitor(IAKION TEMP9500) specific function.
    global bodytemp
    global collect_temp

    with serial.Serial() as ser:
        ser.baudrate    = TEMP_BAUDRATE
        ser.port        = TEMP_PORT
        ser.timeout     = TEMP_TIMEOUT
        ser.open()
        while not stop_event.is_set():
            collect_temp.clear()
            try:
                ser.write(b'\x06\xf9GVT\x01\xf0>')
                temp        = ser.readline()
                if temp == b'':
                    pass
                else:
                    bodytemp = float(temp[5:10])
            except:
                pass
            collect_temp.wait()

def reset_data():
    global stop_event
    global logged_data
    global tt_data

    stop_event.clear()
    logged_data     = dict(ts=[],   # timestamps
                           vsp=[],  # heart pulse
                           rsp=[],  # resperatory
                           btm=[])  # body temperature
    tt_data         = []            # timestamp for trigger


if __name__ == '__main__':

    # This example code is only collect three physiology monitoring values as below.
    dummy           = 0
    bodytemp        = 0

    HOST            = '127.0.0.1'          # Localhost
    PORT            = 65432                # Arbiturary port
    
    # Our hardware related setup
    TEMP_PORT       = '/dev/ttyUSB0'
    TEMP_BAUDRATE   = 250000
    TEMP_TIMEOUT    = .2

    BLOCKSIZE       = 1024
    start_time      = 0
    
    trig_obj = Button(18)

    try:
        while True:
            reset_data()
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                s.bind((HOST, PORT))
                s.listen()

                conn, addr = s.accept()
            
                with conn:
                    data = conn.recv(BLOCKSIZE)
                    if not data:
                        break
                    try:
                        tr, sfreq, dummy = data.decode("utf-8").split('_')

                        print('TR: {}, Sampling Freq: {}, Number of Dummy Scan: {}'.format(tr, sfreq, dummy))
                        tr = float(tr)
                        sfreq = float(sfreq)
                        dummy = float(dummy)
                    except:
                        raise Exception  # received wrong data structure.
                   
                    try:
                        init_ts = time.perf_counter()
                        bt_obj = threading.Thread(target=update_bodytemp)
                        bt_obj.daemon = True
                        bt_obj.start()
                    
                        dl_obj = threading.Thread(target=data_logger)
                        dl_obj.daemon = True
                        dl_obj.start()

                        tm_obj = threading.Thread(target=trigger_monitor)
                        tm_obj.daemon = True
                        tm_obj.start()

                        stop_event.wait() # if the process end
                    
                    except (KeyboardInterrupt, SystemExit):
                        stop_event.set()

                    time.sleep(0.1)

                    import pickle

                    logged_data['ts'] = [t - logged_data['ts'][0] for t in logged_data['ts']] # zero point alighment
                    logged_data['tt'] = tt_data

                    # correct size missmatch
                    min_datasize = min([len(dp) for key, dp in logged_data.items() if key in ['ts', 'vsp', 'rsp', 'btm']])
                    for key, dp in logged_data.items():
                        if key in ['ts', 'vsp', 'rsp', 'btm']:
                            logged_data[key] = dp[:min_datasize]

                    d = pickle.dumps(logged_data)

                    print('Number of timestamps : {}'.format(len(logged_data['ts'])))
                    print('Number of triggers   : {}'.format(len(tt_data)))
                    print('Size of data         : {}'.format(len(d)))

                    for n in range(len(d) // BLOCKSIZE + 1):
                        conn.sendto(d[n * BLOCKSIZE:(n+1) * BLOCKSIZE], addr)
                    
            clear_job()

    except KeyboardInterrupt:
        pass

