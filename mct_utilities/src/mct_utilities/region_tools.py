from __future__ import print_function
import os
import networkx as nx
import file_tools

def check_regions_and_camera_pairs(regions_dict, camera_pairs_dict):
    """
    Check the regions and camera pairs dictionaries.
    """
    if not regions_dict.keys() == camera_pairs_dict.keys():
        raise ValueError, 'regions and camera_pairs dictionaries must have the save region names'
    for region_cameras in regions_dict.values():
        check_region(region_cameras)
    for region_name in regions_dict:
        check_camera_pairs(regions_dict[region_name], camera_pairs_dict[region_name])
    
def check_region(region_cameras):
    """
    Checks region for redundant cameras.
    """
    # Check that camera list doesn't contain redundant cameras
    region_cameras_set = set(region_cameras)
    if len(region_cameras_set) != len(region_cameras):
        raise ValueError, 'camera list contains redundant cameras'

    # Check that a homography matrix exists for every camera in the region
    homography_files = file_tools.get_homography_calibration_files(fullpath=False)
    homography_files = map(os.path.splitext, homography_files)
    cameras_w_homography = [name for name, ext in homography_files if 'camera' in name]
    for camera in region_cameras:
        if not camera in cameras_w_homography:
            raise ValueError, '{0} does not have a homography matrix'.format(camera)

def check_camera_pairs(region_cameras, camera_pairs):
    """
    Checks the cameras for the given region cameras. The the camera pairs must
    define a spanning tree for the region cameras.
    """

    # Check if the set of cameras in the region is equal the set of camers in the
    # pairs list - if not then the pairs don't define a spanning tree for the region
    pairs_set = set()
    for camera_0, camera_1 in camera_pairs:
        pairs_set.add(camera_0)
        pairs_set.add(camera_1)
    region_set = set(region_cameras)
    if not (pairs_set == region_set):
        raise ValueError, 'camera pairs to not define a spanning tree for the region'

    # Create graph based on region cameras and camera pairs and check that the graph
    # is a tree - i.e., no cycles
    pairs_graph = nx.Graph()
    pairs_graph.add_nodes_from(region_cameras)
    pairs_graph.add_edges_from(camera_pairs)
    cycles = nx.cycle_basis(pairs_graph,region_cameras[0])
    if cycles:
        raise ValueError, 'graph defined by camera pairs is not a tree'
    components = nx.connected_components(pairs_graph)
    if len(components) != 1:
        raise ValueError, 'graph defined by camea pairs has multiple components'

# -----------------------------------------------------------------------------
if __name__ ==  '__main__':

    regions_dict = file_tools.read_tracking_2d_regions()
    camera_pairs_dict = file_tools.read_tracking_2d_camera_pairs()
    region_cameras = regions_dict['maze']
    camera_pairs = camera_pairs_dict['maze']

    if 0:
        for region_cameras in regions_dict.values():
            check_region(region_cameras)

    if 0:
        check_camera_pairs(region_cameras,camera_pairs)

    if 1:
        check_regions_and_camera_pairs(regions_dict, camera_pairs_dict)
        






