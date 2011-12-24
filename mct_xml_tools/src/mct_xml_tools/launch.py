import os
import jinja2

def create_inspector_launch(filename,machines):
    """
    Creates launch file for camera inspector nodes.
    """
    file_path, file_name = os.path.split(__file__)
    template_dir = os.path.join(file_path, 'templates')
    template_name ='inspector_nodes_launch.xml'

    # Use jinja2 to create xml string
    jinja2_env = jinja2.Environment(loader=jinja2.FileSystemLoader(template_dir))
    template = jinja2_env.get_template(template_name)
    xml_str = template.render(machine_names = machines)

    # Write lauch file
    with open(filename,'w') as f:
        f.write(xml_str)

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    filename = 'test.xml'
    machines = ['c1', 'c2', 'c3']
    create_inspector_launch(filename,machines)
