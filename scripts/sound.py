#!/usr/bin/env python3
import roslib;
import rospy, os, sys
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

# directory with sound assets - change as needed
soundAssets = '/home/shiloh/devel/audio_assets/'
# duration of yak throttle
throttle = 3 # seconds

def play_sound():
    soundhandle.say(data.param)

if __name__ == '__main__':
    soundhandle = SoundClient()
    rospy.sleep(1)
    play_sound()
