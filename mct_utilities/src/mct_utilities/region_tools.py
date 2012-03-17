from __future__ import print_function
import networkx as nx
import file_tools
import pylab

def check_region(region_cameras):
    """
    Checks region for redundant cameras.
    """
    region_cameras_set = set(region_cameras)
    if len(region_cameras_set) != len(region_cameras):
        raise ValueError, 'camera list contains redundant cameras'

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
    check_camera_pairs(region_cameras,camera_pairs)
        






