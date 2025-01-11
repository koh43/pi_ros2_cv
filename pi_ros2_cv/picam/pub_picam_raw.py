#!/usr/bin/env python3

import rclpy
from libcamera import controls

from nodes.picam import pub_picam_raw
from utils import cam_utils

def main(args=None):
    rclpy.init(args=args)

    params = {
        'node_name': 'pub_picam',
        'queue_size': 5,
        'hz': 30,
        'frame_id': 'picam',
        'res': cam_utils.Resolution('HD'),
        'raw_config': {
            # 30 FPS for HD
            'fps' : 30,
            'bit_depth' : 12,
        },
        'main_config': {
            "format"    : 'RGB888',
        },
        'controls': {
            "AeEnable"   : True,
            "AfMode"     : controls.AfModeEnum.Continuous,
            "AwbEnable"  : True,
            "Brightness" : 0.0,
            "Contrast"   : 1.0,
            "Saturation" : 1.0,
            "Sharpness"  : 1.0,
        }
    }

    params['main_config']['size'] = (
        params['res'].width,
        params['res'].height
    )

    app = pub_picam_raw.PUB_PICAM_RAW(params)

    rclpy.spin(app)
    app.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()