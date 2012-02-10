
def get_scale(config, request):
    """
    Gets the scale argument from the request object.
    """ 
    scale_options = config.camera_view_table['scale_options']
    scale_default = config.camera_view_table['scale_default']
    scale = request.args.get('scale',scale_default)
    try:
        scale = float(scale)
    except ValueError:
       scale = scale_default 
    if not scale in scale_options:
        scale = scale_options[0]
    return scale, scale_options





