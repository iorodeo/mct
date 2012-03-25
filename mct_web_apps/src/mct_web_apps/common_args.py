
def get_scale(config, request, view='table'):
    """
    Gets the scale argument from the request object.
    """ 
    if view == 'table':
        scale_options = config.camera_view_table['scale_options']
        scale_default = config.camera_view_table['scale_default']
    elif view == 'single':
        scale_options = config.camera_view_single['scale_options']
        scale_default = config.camera_view_single['scale_default']
    else:
        raise ValueError, 'unknown view option'

    scale = request.args.get('scale',scale_default)

    try:
        scale = float(scale)
    except ValueError:
       scale = scale_default 
    if not scale in scale_options:
        scale = scale_options[0]

    # Convert scale options and scale to strings 
    scale_options = ['{0:1.2f}'.format(x) for x in scale_options]
    scale = '{0:1.2f}'.format(scale)

    return scale, scale_options





