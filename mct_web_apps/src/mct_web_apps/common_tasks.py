import config

def get_image_size(scale): 
    """
    Get size of image from scale
    """
    image_width = int(config.camera_image['width']*float(scale))
    image_height = int(config.camera_image['height']*float(scale))
    return image_width, image_height

def camera_name_cmp(x,y):
    """
    Comparison function for camera names
    """
    value_x = int(x.replace('camera_', ''))
    value_y = int(y.replace('camera_', ''))
    if value_x > value_y:
        return 1
    elif value_y > value_x:
        return -1
    else:
        return 0


def mjpeg_info_cmp(x,y):
    """
    Comparison function for sorting a list of (camera_name, camera_info) pairs.
    """
    name_x = x[0]
    name_y = y[0]
    value_x = int(name_x.replace('camera_', ''))
    value_y = int(name_y.replace('camera_', ''))
    if value_x > value_y:
        return 1
    elif value_y > value_x:
        return -1
    else:
        return 0
