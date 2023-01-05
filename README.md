# weatherBalloon
Raspberry Pi project for a High Altitude Weather Balloon

## Introduction

Raspberry Pi project for a High Altitude Weather Balloon.  It essentially has two halves: data acquisition and data anlaysis.  The data acquisition code performs the following:
   * records several enviornmental factors including temperature, humidity and pressure
   * records GPS location
   * records orientation (pitch, yaw, roll)
   * records video annotated with some of the above values

## Installation

For running the analysis code on a desktop:

Create a python 3 virtualenv with the required packages in requirements.txt

TBF: include more instructions for installation on the Raspberry Pi for running data acquisition.

## Usage

   * source the virtualenv (may not be needed on Raspberry Pi)
   * adjust values in weatherBalloon.conf (including length of video, etc.)
   * python weatherBalloon.py # for running data aquisition
   * python weatherBalloon.py [path to csv file] # for plotting results

## The Rasperry Pi

Here is our assembled Raspberry Pi, with attached:
   * Camera module
   * SenseHat with LCD, joystick, accelerometer, temperature, pressure, and humidity sensors
   * BME temperature, pressure and humidity sensor
   * GPM module and antenna

<img src="images/raspberryPi.jpg" alt="raspberryPi.jpg" width="500">

Here is the Raspberry Pi in it's 3D printed enclosure for the weather balloon launch.

<img src="images/raspberryPiEnclosure.jpg" alt="raspberryPiEnclosure.jpg" width="500">

Here's an informal wiring diagram:

<img src="images/raspberryPiWiring.jpg" alt="raspberryPiWiring.jpg" width="500">

## Assembly Instructions

### Sense Hat

Installing the Sense Hat and using it with Python is straight forward:

   * https://www.raspberrypi.com/documentation/accessories/sense-hat.html
   * https://pythonhosted.org/sense-hat/
   
## Retrieving the Data

The sensor values are recorded in a CSV file marked 'env.[timestamp].csv'.  These values can then be parsed by the plotEnvCsv function.  To see the types of output, see the Results section.



## Results

### March 6 2020 Test Run

Weather Balloon software was run for 10 minutes while taking a short drive.  All sensors and video seemed to work correctly.

Here is a screen shot from the annotated video:

<img src="images/March6_video.png" alt="March6_video.png" width="500">

Here the pressures from the two sensors make sense, since we first dropped 300 feet in altitude, and then backtracked back up again.

<img src="images/March6_pressures.png" alt="March6_pressures.png" width="500">

Here the temperatures from our three sensors are presented.  The SenseHat temperature sensor is right above the CPU, so this shows how it is heated, unlike the BME sensor.

<img src="images/March6_AllTempsF.png" alt="March6_AllTempsF.png" width="500">

### May 18 2020 Balloon Launch

Our launch and recovery of the balloon and the payload, including the Raspberry Pi, was succesfull!  There are too many detail to include here, but here's the highlights:

Here's how the Raspberry Pi was mounted in the payload:

<img src="images/may18launch/raspberryPiPayload2.jpg" alt="raspberryPiPayload2.jpg" width="500">

And the actual launch of the balloon:

<img src="images/may18launch/balloonLaunch.jpg" alt="balloonLaunch.jpg" width="500">

Once we recovered the payload, we were able to extract the video and plot the results.  Here's a screen shot of part of the annotated video:

<img src="images/may18launch/payloadVideo1.png" alt="payloadVideo1.jpg" width="500">

And finally, some of our very interesting results!  Here we track our balloons 2 hour flight over a map of the area:

<img src="images/may18launch/LatVsLngMap.png" alt="LatVsLngMap.png" width="500">

This is a plot of the pressure inside and outside the payload, in units of hPa.  It makes sense that they are very close, since the holes in the payload container would cause the air pressure to equalize.  The constant air pressure at first makes sense since we started taking data for about 45 minutes before we launched.  Then the pressure decreases, which makes sense, since the balloon was increasing in altitude.  From the pressures, it looks like it ascended for about 35 minutes (till about 11:00 AM), then descended about the same distance in only 10 minutes (about 11:10 AM).  It looks like we stopped taking data when it finished it’s decent, rather than continuing for the full configured 4 hours.  This is probably due to the crash landing where the power supply may have been interrupted.  

Also, these pressures show that we perhaps reached an altitude of 46,000 feet!


<img src="images/may18launch/Pressures.png" alt="Pressures.png" width="500">

The plots of the temperatures tell a similar story.  Here the difference is that the temperatures measured inside the payload are quite different from the external sensor readings.  This makes sense, since the small holes in the payload probably allow enough air flow for the pressure to equalize, but not the temperature.

Some interesting things happening here.  First of all, concerns about keeping the electronics warm were unfounded – indeed it’s good they didn’t overheat.  The extreme cold encountered at the top of the balloon’s flight is remarkable (~ -60 F!).  Also of interest is the cooling of the inside of the payload near the end.  This may have been from the rapid decent, causing cool air to rush through the holes, cooling the box down?


<img src="images/may18launch/AllTempsF.png" alt="AllTempsF.png" width="500">


We also were using external cameras on the payload but are not including those here, since they were not taken using this software.

Finally, the actual data from the May 18 launch is included here at images/may18launch/env.2022_05_17_16_22_55.csv
