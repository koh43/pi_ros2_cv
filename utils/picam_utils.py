from picamera2 import Picamera2

def picam_init(params):
    cam = Picamera2()
    cam.configure(
        cam.create_video_configuration(
            raw=params['raw_config'],
            main=params['main_config']
        )
    )
    cam.start()
    cam.set_controls(
        params['controls']
    )
    return cam