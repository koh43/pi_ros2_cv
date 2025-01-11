import rclpy

from nodes.misc import image_view

def main(args=None):
    rclpy.init(args=args)

    params = {
        "topic_name": "/image_color",
        "img_type"  : "compressed",
        'queue_size': 5,
        # "topic_name": "/video/frame",
        # "img_type"  : "compressed",
    }

    image_view_node = image_view.IMAGE_VIEW(params)

    rclpy.spin(image_view_node)
    image_view_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
