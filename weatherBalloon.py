"""
A module for collecting data and video for a high altitude weather baloon project
"""


import csv
import time
import string
import subprocess
import io
import os
import pathlib
import configparser
import argparse
from datetime import datetime

try:
    # sense hat
    from sense_hat import SenseHat
    # pi camera
    from picamera import PiCamera
    from picamera import Color
    # GPS sensor
    import serial
    import pynmea2
    # BME280 env. sensor
    import bme280
    import smbus2
except:
    print("IMPORT PI EXCEPTION!  Are you not on the Pi?")

try:
    import matplotlib.pyplot as plt 
    import matplotlib
except:
    print("IMPORT matplotlib EXCEPTION!  Are you on the Pi?")

TIME_FRMT = "%Y:%m:%d_%H:%M:%S"

# Here's all the data we'll be tracking:
KEYS = [
    'nowStr',
    'cpu_temperature', 
    'bme_temperature', 
    'bme_humidity', 
    'bme_pressure', 
    'temperature', 
    'humidity', 
    'pressure', 
    'pitch', 
    'roll', 
    'yaw', 
    'lat', 
    'lng',
    'alt',
    'accel_x',
    'accel_y',
    'accel_z'
]

UNITS = [
    '',
    '(C)',
    '(C)',
    '',
    '',
    '(C)',
    '',
    '',
    '(deg)',
    '(deg)',
    '(deg)',
    '(deg)',
    '(deg)',
    '(m)',
    '(Gs)',
    '(Gs)',
    '(Gs)'
]

def str2flt(s):
    "String to Float; returns None if there's an issue"
    f = None
    try:
        f = float(s)
    except:
        f = None
    return f
             
def frmt(s):
    "'45.434343' -> 45.434343 -> '45.43', covering None as well" 

    v = 'NaN'
    f = str2flt(s)
    if f is not None:
        v = "%5.2f" % f
    return v    

def tryToGetGPSInfo(sio):
    """
    A more efficient way of getting not just
    Lattitude and Longitude but also Altitude.
    """
    lat = lng = alt = None
    numAttempts = 10
    for i in range(numAttempts):
        try:
            newdata = sio.readline()
        except UnicodeDecodeError:
            print("got UnicdoeDecodeError in tryToGetGPSInfo")
            continue
            
        if newdata[0:6] == "$GPGGA":
            newmsg = pynmea2.parse(newdata)
            lat = newmsg.latitude
            lng = newmsg.longitude
            alt = newmsg.altitude
            print("Got GGA info: lat=%s, lng=%s, alt=%s" % (lat, lng, alt))
            break
        
    return lat, lng, alt

def tryToGetLatLong():
    """
    Tries to get Lattitude and Longitude from the GPS module.
    There seems to be connection problems with the serial port,
    so we try a few times.  This is a big TBF.
    """
    lat = lng = None
    numAttempts = 10
    for i in range(numAttempts):
        print("connecting to serial port")
        port="/dev/ttyAMA0"
        ser=serial.Serial(port, baudrate=9600, timeout=0.5)
        sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser))
        dataout = pynmea2.NMEAStreamReader()
        #newdata=ser.readline()
        try:
            newdata=sio.readline()
            print("got data: ", newdata)
        except UnicodeDecodeError:
            print("got UnicodeDecodeError, trying again")
            continue
        #newdata = newdata.decode('utf-8')

        if newdata[0:6] == '$GPRMC':
            newmsg=pynmea2.parse(newdata)
            lat=newmsg.latitude
            lng=newmsg.longitude
            gps = "Latitude=" + str(lat) + "and Longitude=" + str(lng)
            print(gps)
            break

    return (lat, lng)
                

def getSensorEnvStrs(sense):
    "Get env data straight from the SenseHAT"

    humidity = sense.get_humidity()
    print("Humidity: %s %%rH" % humidity)
    temp = sense.get_temperature()
    print("Temperature: %s C" % temp)
    pressure = sense.get_pressure()
    print("Pressure: %s Millibars" % pressure)

    return temp, humidity, pressure
    
def captureImage(fn=None):
    "test function for capturing an image with the Pi camera"

    if fn is None:
        fn = "image.jpg"

    c = PiCamera()
    
    # settings
    c.rotation = 180
    c.resolution = (500, 450) #(1920, 1080)

    # text settings
    c.annotate_text_size = 12

    # pic text
    txt = "%s" % datetime.now()
    c.annotate_text = txt 

    # take a picture and save it
    c.capture(fn)

def getEnv(sense):
    "Returns a dict of a number of timestamped sensor values from the Sense Hat"

    temperature, humidity, pressure = getSensorEnvStrs(sense)

    orientation = sense.get_orientation_degrees()
    print("p: {pitch}, r: {roll}, y: {yaw}".format(**orientation))

    accel_only = sense.get_accelerometer_raw()
    print("x: {x}, y: {y}, z: {z}".format(**accel_only))

    # convert these all to the appropriate strings
    # temperature = frmt(temperature)
    # humidity = frmt(humidity)
    # pressure = frmt(pressure)
    # pitch = frmt(orientation['pitch'])
    # roll = frmt(orientation['roll'])
    # yaw = frmt(orientation['yaw'])

    now = datetime.now()
    nowStr = datetime.strftime(now, TIME_FRMT) # "%Y:%m:%d_%H:%M:%S")

    env = {
        'nowStr': nowStr,
        'temperature': temperature,
        'humidity': humidity,
        'pressure': pressure,
        'pitch': orientation['pitch'],
        'roll': orientation['roll'],
        'yaw': orientation['yaw'],
        'accel_x': accel_only['x'],
        'accel_y': accel_only['y'],
        'accel_z': accel_only['z']
    }
    return env

def getEnvStr(env):
    "Convert parts of the given dict to a string suitable for annotating video"

    # convert these all to the appropriate strings
    cpu_temperature = frmt(env['cpu_temperature'])
    bme_temperature = frmt(env['bme_temperature'])
    bme_humidity = frmt(env['bme_humidity'])
    bme_pressure = frmt(env['bme_pressure'])
    temperature = frmt(env['temperature'])
    humidity = frmt(env['humidity'])
    pressure = frmt(env['pressure'])
    pitch = frmt(env['pitch'])
    roll = frmt(env['roll'])
    yaw = frmt(env['yaw']) 
    txt = "(%s)T:%s;H:%s,P:%s,C:%s,R:%s,Y:%s,LG:%s,LT:%s" % (env['nowStr'],
                                                 temperature,
                                                 humidity,
                                                 pressure,
                                                 pitch,
                                                 roll,
                                                 yaw,
                                                 env['lat'],
                                                 env['lng'])
    return txt

def writeEnv(env, filename):
    "Add the dict of data as a row in our CSV file"
    print('writeEnv to: ', filename)
    # translate dict to ordered list:

    row = [str(env[k]) for k in KEYS]
    with open(filename, 'a') as f:
        w = csv.writer(f)
        w.writerow(row)

def annotatedImage():
    "Test function for basic stuff"

    #print("Running Weather Balloon Software")

    # get environment readings
    sense = SenseHat()
    sense.set_imu_config(False, True, True)  # gyroscope and accel only, no compass

    temperature, humidity, pressure = getSensorEnvStrs(sense)

    orientation = sense.get_orientation_degrees()
    print("p: {pitch}, r: {roll}, y: {yaw}".format(**orientation))

    accel_only = sense.get_accelerometer()
    print("p: {pitch}, r: {roll}, y: {yaw}".format(**accel_only))

    # convert these all to the appropriate strings
    temperature = frmt(temperature)
    humidity = frmt(humidity)
    pressure = frmt(pressure)
    pitch = frmt(orientation['pitch'])
    roll = frmt(orientation['roll'])
    yaw = frmt(orientation['yaw'])

     # Take a Picture
    c = PiCamera()
    
    # settings
    c.rotation = 180
    c.resolution = (500, 450) #(1920, 1080)

    # text settings
    c.annotate_text_size = 12

    # pic text
    now = datetime.now()
    nowStr = datetime.strftime(now, "%Y:%m:%d_%H:%M:%S")
    txt = "(%s)T:%s;H:%s,P:%s,C:%s,R:%s,Y:%s" % (nowStr,
                                                 temperature,
                                                 humidity,
                                                 pressure,
                                                 pitch,
                                                 roll,
                                                 yaw)

    c.annotate_text = txt 

    # take a picture and save it
    fn = "image.jpg"
    c.capture(fn)

def plotEnvCsv(filename):
    "For visulizing data written by getAnnotatedVideo; run after the fact"

    rows = []
    with open(filename, 'r') as csvfile:
        r = csv.reader(csvfile)
        for row in r:
            print(row)
            rows.append(row)
    print(rows)

    plotLatLongMap(rows, KEYS)
            
    dts = [datetime.strptime(r[0], TIME_FRMT) for r in rows]
    mdts = matplotlib.dates.date2num(dts)

    # make individual plots of everything but the timestamps        
    keys = KEYS #['nowStr', 'temperature', 'humidity', 'pressure', 'pitch', 'roll', 'yaw']
    units = UNITS #['', '(C)', '', '', '(deg)', '(deg)', '(deg)']
    ignoreZeroPlots = ['lat', 'lng', 'pressure', 'bme_pressure']
    pryPlots = ['pitch', 'roll', 'yaw']
    offset = 0.
    for i in range(1, len(keys)):
        k = keys[i]
        t = "%s %s" % (k, units[i])
     
        try:
            if k in ignoreZeroPlots:
                print("ignoring zeros for plot", k)    
                dataDts = [(float(rows[j][i]), dts[j]) for j in range(len(rows)) if rows[j][i] != 'None' and rows[j][i] is not None and rows[j][i] != '0' and rows[j][i] != '0.0']
            else:    
                dataDts = [(float(rows[j][i]), dts[j]) for j in range(len(rows)) if rows[j][i] != 'None' and rows[j][i] is not None ]
        except:
            print("Error ploting ", k)
            continue

        data = [d[0] for d in dataDts]
        thisDts = [d[1] for d in dataDts]        
        #plt.plot_date(mdts, data)
        if len(thisDts) != len(data):
            plt.plot( data)
        else:
            plt.plot(thisDts, data)
        # plt.plot(data)
        # beautify the x-labels
        plt.gcf().autofmt_xdate()
        plt.xlabel("Time (EST) starting at %s" % rows[0][0])
        plt.ylabel(t)
        plt.title(t)
        plt.savefig(k + ".png")
        plt.show()

    # now make group plots:

    # plot three temperatures together
    keys = ['cpu_temperature', 'bme_temperature', 'temperature']
    colors = ['-b', '-g', '-r']
    #fig, ax = plt.subplots()
    #red_patch = mpatches.Patch(color='red', label='The red data')
    #ax.legend(handles=[red_patch])
    for j, k in enumerate(keys):
        i = KEYS.index(k)
        data = [float(row[i]) for row in rows]
        plt.plot(dts, data, colors[j], label=k)
    # beautify the x-labels
    plt.gcf().autofmt_xdate()
    plt.title('All temperatures (C)')
    plt.xlabel("Time (EST) starting at %s" % rows[0][0])
    plt.ylabel('Temperature (C)')
    plt.legend(loc="upper left")
    # plt.ylim(-1.5, 2.0)    
    plt.savefig("AllTempsC.png")
    plt.show()

    # do it again for Ferenheit
    #  0 c x 9/5 + 32 = F
    for j, k in enumerate(keys):
        i = KEYS.index(k)
        data = [(float(row[i])* (9./5)) + 32.0 for row in rows]
        plt.plot(dts, data, colors[j], label=k)
    # beautify the x-labels
    plt.gcf().autofmt_xdate()
    plt.xlabel("Time (EST) starting at %s" % rows[0][0])
    plt.ylabel('Temperature (F)')
    plt.title('All temperatures (F)')
    plt.legend(loc="upper left")
    plt.savefig("AllTempsF.png")
    plt.show()

    plotDataTogether(['bme_pressure', 'pressure'], colors, 'Pressures', 'Pressures.png', 'Pressures', rows)
    plotDataTogether(['bme_humidity', 'humidity'], colors, 'Humiditys', 'Humiditys.png', 'Humiditys', rows)


def plotLatLongMap(rows, keys):
    """
    Plot the Longitude vs. Latitude, both directly,
    and overlayed on top of a map.
    """

    #keys = ['lat', 'lng']
    i = keys.index('lat')
    k = keys[i]
    t = "%s %s" % (k, "(deg)")

    # gather the data, ignoring bad data
    lats = [float(rows[j][i]) for j in range(len(rows)) if rows[j][i] != 'None' and rows[j][i] is not None and rows[j][i] != '0' and rows[j][i] != '0.0']
    i = keys.index('lng')
    lngs = [float(rows[j][i]) for j in range(len(rows)) if rows[j][i] != 'None' and rows[j][i] is not None and rows[j][i] != '0' and rows[j][i] != '0.0']
    
    #print("Lats range: ", min(lats), max(lats))
    #print("Lngs range: ", min(lngs), max(lngs))
    #Lats range:  38.245827166666665 38.4273425
    #Lngs range:  -79.8276415 -79.27054683333333
    # BBox = Lngs, Lats
    BBox = (-79.8276415, -79.27054683333333, 38.245827166666665,  38.4273425)
    
    # fig, ax = plt.subplots()
    # # ax.scatter(lats, lngs)
    # ax.set_xlim(BBox[2], BBox[3])
    # ax.set_xlim(BBox[0], BBox[1])

    plt.xlabel("Longitude (deg)")
    plt.ylabel("Lattitude (deg)")
    plt.title('Lattitude vs Longitude')

    # plot this data directly
    plt.plot(lngs, lats)

    plt.savefig("LatVsLng.png")
    plt.show()

    # now do it overlayed ontop of a map
    fig, ax = plt.subplots()

    # ax.scatter(lats, lngs)
    ax.set_ylim(BBox[2], BBox[3])
    ax.set_xlim(BBox[0], BBox[1])

    plt.xlabel("Longitude (deg)")
    plt.ylabel("Lattitude (deg)")
    plt.title('Lattitude vs Longitude')
    
    # TBF: make the name of this file a parameter?
    mapFile = 'map.png'
    if not os.path.isfile(mapFile):
        print("ERROR: cannot find map file: %s" % mapFile)
        return
    
    ruh_m = plt.imread(mapFile)
    
    ax.imshow(ruh_m, zorder=0, extent = BBox, aspect= 'equal')

    plt.plot(lngs, lats)

    plt.savefig("LatVsLngMap.png")
    plt.show()

def plotDataTogether(keys, colors, title, fn, ylabel, rows):
    "Make a plot of a number of data using the same axis"

    dts = [datetime.strptime(r[0], TIME_FRMT) for r in rows]

    for j, k in enumerate(keys):
        i = KEYS.index(k)
        data = [(float(row[i])* (9./5)) + 32.0 for row in rows]
        plt.plot(dts, data, colors[j], label=k)

    # beautify the x-labels
    plt.gcf().autofmt_xdate()
    plt.ylabel(ylabel)
    plt.xlabel("Time (EST) starting at %s" % rows[0][0])
    plt.title(title)
    plt.legend(loc="upper left")
    plt.savefig(fn)
    plt.show()


def getBMEData(bus, address):
    "Use the external BME sensor to collect env data"
    data = bme280.sample(bus, address)
    return (data.humidity, data.pressure, data.temperature)

def getCPUTemperature():
    "Returns CPU temperature as a float in Celcius via system command"
    cmd = '/opt/vc/bin/vcgencmd'
    arg = 'measure_temp'
    r = subprocess.run([cmd, arg], stdout=subprocess.PIPE)
    try:
        tmpStr = r.stdout.decode('utf-8')
        # parse the expected string like temp=45.2'C\n
        i1 = tmpStr.find("=")
        i2 = tmpStr.find("'")
        tmp = float(tmpStr[i1+1:i2])
    except:
        print("Failure getting CPU Temperature")
        tmp = None
    return tmp
            
def getAnnotatedVideo(configFile=None):
    "Run video for a while and annotate it with cool data, and write data to disk too"

    print("Running Weather Balloon Software")

    # get some values from the configfile
    if configFile is None:
        fn = "weatherBalloon.conf"
        d = pathlib.Path(__file__).parent.resolve()
        configFile = os.path.join(d, fn)

    config = configparser.ConfigParser()

    print("Opening configfile: ", configFile)
    config.read(configFile)
    print(config)

    # are we using all the sensors?
    use_bme = config['SENSORS'].getboolean('use_bme')
    use_gps = config['SENSORS'].getboolean('use_gps')

    # here's where our video goes
    # how to convert: ffmpeg -r 30 -i video.h264 video.mp4
    d = pathlib.Path(__file__).parent.resolve()
    frmt = "%Y_%m_%d_%H_%M_%S"
    now = datetime.now()
    nowStr = now.strftime(frmt)
    videoFn = "video.%s.h264" % nowStr
    videoFn = os.path.join(d, videoFn)

    # here's where our data goes
    csvFn = "env.%s.csv" % nowStr
    csvFn = os.path.join(d, csvFn)

    # get files ready
    f = open(csvFn, 'w')
    f.close()

    # setup sense hat
    sense = SenseHat()
    sense.set_imu_config(False, True, True)  # gyroscope and accel only, no compass

    # use the sense hat display at all?
    use_display = config['DISPLAY'].getboolean('use_display')
    if not use_display:
        # TBF: how to turn off sense hat LCD???
        # here we set the color to black?
        sense.clear((0, 0, 0))

    # output on the display for the whole time, or just the beginning?
    use_full_display = config['DISPLAY'].getboolean('use_full_display')

    # letters to use on the display
    letters = [str(i) for i in range(0,10)]
    li = 0

    # setup env sensor
    port = 1
    address = 0x77 # Adafruit BME280 address. Other BME280s may be different
    if use_bme:
        bus = smbus2.SMBus(port)
        bme280.load_calibration_params(bus,address)

    # setup gps
    if use_gps:
        print("connecting to serial port for GPS")
        port="/dev/ttyAMA0"
        ser = serial.Serial(port, baudrate=9600, timeout=0.5)
        sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser))
        
    # set up camera
    c = PiCamera()
    
    # settings
    rotationStr = config['CAMERA']['rotation']
    if rotationStr != 'None':
        c.rotation = int(rotationStr)
    resX = config['CAMERA'].getint('resolutionX')    
    resY = config['CAMERA'].getint('resolutionY')    
    c.resolution = (resX, resY) #config['CAMERA']['resolution'] #(500, 450) #(1920, 1080)

    # text settings
    c.annotate_text_size = 12
    c.annotate_foreground = Color('white')
    c.annotate_background = Color('black')
    
    c.start_recording(videoFn)

    # record for how long?
    videoSecs = config['DEFAULT'].getint('length_seconds')

    # it seems to take almost 2 secs to get all the dat,
    # especially from the GPS
    txtSecs = 2

    start = datetime.now()
    lastTxt = datetime.now()
    now = datetime.now()

    nowSecs = time.time()
    startSecs = time.time()
    lastTxtSecs = time.time()
    
    elapsedSecs = 0 #(now - start).seconds
    elapsedTxtSecs = 0 #(now - lastTxt).seconds

    # env = getEnv(sense)
    # txt = getEnvStr(env)
    # c.annotate_text = txt

    # keep taking video till we've exceed the desired length of time
    while elapsedSecs < videoSecs:

        print("elapsedSecs: ", elapsedSecs)
        print("elapsedTxtSecs: ", elapsedTxtSecs)

        # is it time to get data yet?
        if elapsedTxtSecs >= txtSecs: 
            
            # time to get data!

            # GPS data
            if use_gps:
                lat, lng, alt = tryToGetGPSInfo(sio) #tryToGetLatLong()
            else:
                lat, lng, alt = None, None

            # get sensor data from sense hat in dict form
            env = getEnv(sense)

            # add GPS data to this dict
            env['lat'] = lat
            env['lng'] = lng
            env['alt'] = alt

            # add BME280 sensor data
            if use_bme:
                humidity, pressure, temperature = getBMEData(bus,address)
            else:    
                humidity, pressure, temperature = None, None, None
            env['bme_humidity'] = humidity
            env['bme_pressure'] = pressure
            env['bme_temperature'] = temperature

            # get the CPU temperature
            env['cpu_temperature'] = getCPUTemperature()

            # write all this data to disk
            writeEnv(env, csvFn)

            # now put this data in a string to add to video
            txt = getEnvStr(env)
            print(txt)
            c.annotate_text = txt

            # update the display to show we're working
            if use_display and li < 10:
                sense.show_letter(letters[li])
                li += 1
                # we might want to only run this at the very beginning
                # to save power on long videos
                if li > 9:
                    if use_full_display:
                        li = 0
                    else:
                        # turn things off to save power
                        sense.clear((0,0,0))    

            # update last time we added new txt to video
            lastTxt = now
            lastTxtSecs = nowSecs

        c.wait_recording(0.2)
        now = datetime.now()
        nowSecs = time.time()
        elapsedSecs = nowSecs - startSecs #(now - start).seconds
        elapsedTxtSecs = nowSecs - lastTxtSecs #(now - lastTxt).seconds

    # we're done!
    c.stop_recording()

    # clear the sxreen to save power
    sense.clear((0, 0, 0))

def waitForUserStart():
    "Wait for user to initialize starting of data, using joystick"

    print("Waiting for user to start weather balloon software")

    # wait till the joystick is pressed down and released
    sense = SenseHat()
    sense.show_letter('?')

    notPressed = True
    while notPressed:
        try:
            for event in sense.stick.get_events():
                #print(event.direction, event.action)
                if event.direction == 'middle':
                    notPressed = False
        except KeyboardInterupt:
            notPressed = True
            break

    return not notPressed        

def runWeatherBalloon():
    """
    Entry point for weather balloon project:
       * wait for user to initialize taking data
       * take data
       * exit
    """

    start = waitForUserStart()
    if start:
        getAnnotatedVideo()
    else:
        print("user did not choose to start.  exiting")    

def main():
    parser = argparse.ArgumentParser(description='Module for acquiring or analyzing weather balloon data')
    parser.add_argument('file', nargs='?', type=str, help="path to csv file for data analysis; do not provide if starting data acquisition")
    args = parser.parse_args()

    if args.file:    
        plotEnvCsv(args.file)
    else:
        print ("running weather balloon")
        runWeatherBalloon()


if __name__ == '__main__':
    main()
    # hPa: 925 to 111
    # -56 C
