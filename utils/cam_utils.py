class Resolution():
    def __init__(self, res_type):
        if res_type == "FHD":
            self.width  = 1920
            self.height = 1080
        elif res_type == "HD":
            self.width  = 1280
            self.height = 720
        elif res_type == "VGA":
            self.width  = 640
            self.height = 480
        else:
            print("Setting default Resolution: HD")
            self.width = 1280
            self.height = 720
