from picamera import PiCamera

from timer import sleep

camera = PiCamera()
camera.start_preview(alpha=192)
sleep(1)
camera.capture