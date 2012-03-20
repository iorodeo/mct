from __future__ import print_function
import roslib
roslib.load_manifest('mct_transform_2d')
import numpy
import networkx

from mct_utilities import file_tools

class Transforms2d(object):

    def __init__(self):
        self.regions_dict = file_tools.read_tracking_2d_regions()
        self.camera_pairs_dict = file_tools.read_tracking_2d_camera_pairs()

        # Create dictionaries describing tranformation setup
        self._create_camera_to_region_dict()
        self._create_tracking_plane_to_region_dict()
        self._create_region_to_anchor_plane_dict()
        self._create_tracking_to_anchor_plane_dict()

        # Create transformation dictionaries
        self._create_tracking_plane_to_transform_dict()
        self._create_camera_to_transform_dict()

    def camera_pts_to_tracking_plane(self, camera, tracking_plane, *args):
        """
        Transform points from camera image coords into coordinates of the given
        tracking plane.
        """
        if len(args) == 1:
            pts_camera = args[0]
        elif len(args) == 2:
            x,y = args[0], args[1]
            pts_camera = numpy.array([[x,y]])
        # Get camera points in homogeneous coords.
        pts_camera_hg = numpy.ones((pts_camera.shape[0],3))
        pts_camera_hg[:,0] = pts_camera[:,0]
        pts_camera_hg[:,1] = pts_camera[:,1]

        # Get transformation matrix
        tf_matrix = self.get_camera_to_tracking_plane_tf(camera,tracking_plane)
        tf_matrix_t = tf_matrix.transpose()

        # Get tracking points in homogeneous coords and convert back to 2d
        pts_tracking_hg = numpy.dot(pts_camera_hg,tf_matrix_t)
        denom = numpy.zeros((pts_camera.shape[0],2))
        denom[:,0] = pts_tracking_hg[:,2]
        denom[:,1] = pts_tracking_hg[:,2]
        pts_tracking = pts_tracking_hg[:,:2]/denom
        if len(args) == 1:
            return pts_tracking
        else:
            x_tracking = pts_tracking[0,0]
            y_tracking = pts_tracking[0,1]
            return x_tracking, y_tracking 

    def camera_pts_to_anchor_plane(self, camera, *args):
        """
        Transform points from camera coords into anchor plane coordinates
        """
        tracking_plane = get_tracking_plane_name(camera)
        anchor_plane = self.tracking_to_anchor_plane[tracking_plane]
        return self.camera_pts_to_tracking_plane(camera, anchor_plane, *args)

    def get_camera_homography_tf(self, camera):
        """
        Returns the transform from the camera to its tracking plane
        """
        region = self.get_region(camera)
        return self.camera_to_transform[region][camera]

    def get_camera_to_tracking_plane_tf(self, camera, tracking_plane):
        """
        Returns the transform from the camera image coordinates to the given  tracking
        plane coordinates.
        """
        camera_tracking_plane = get_tracking_plane_name(camera)
        camera_homography_tf = self.get_camera_homography_tf(camera)
        tracking_to_tracking_tf = self.get_tracking_to_tracking_plane_tf(
                camera_tracking_plane, 
                tracking_plane
                )
        camera_to_tracking_tf = numpy.dot(tracking_to_tracking_tf, camera_homography_tf)
        return camera_to_tracking_tf

    def get_camera_to_anchor_plane_tf(self,camera):
        """
        Returns the transfrom from the camea to the anchor plane for its region
        """
        tracking_plane = get_tracking_plane_name(camera)
        camera_homography_tf = self.get_camera_homography_tf(camera)
        tracking_to_anchor_tf = self.get_tracking_to_anchor_plane_tf(tracking_plane)
        camera_to_anchor_tf = numpy.dot(tracking_to_anchor_tf, camera_homography_tf)
        return camera_to_anchor_tf

    def get_tracking_to_anchor_plane_tf(self,tracking_plane):
        """
        Returns the transform from the given tracking plane to the regions anchor plane. 
        """
        anchor_plane = self.tracking_to_anchor_plane[tracking_plane]
        region = self.get_region(tracking_plane)
        return self.tracking_plane_to_transform[region][(tracking_plane,anchor_plane)]

    def get_tracking_to_tracking_plane_tf(self, tracking_plane_0, tracking_plane_1):
        """
        Returns the transfrom between the two given tracking planes
        """
        region = self.get_region(tracking_plane_0)
        return self.tracking_plane_to_transform[region][(tracking_plane_0, tracking_plane_1)]

    def get_region(self, name):
        """
        Returns the region to which 'name' (either a camera or tracking plane) belongs.
        """
        region = None
        try:
            region = self.camera_to_region[name]
        except KeyError:
            pass
        if region is None: 
            region = self.tracking_plane_to_region[name] 
        return region

    def _create_camera_to_region_dict(self):
        """
        Creates a dictionary maping camera to region
        """
        self.camera_to_region = {}
        for region, camera_list in self.regions_dict.iteritems():
            for camera in camera_list:
                self.camera_to_region[camera] = region


    def _create_tracking_plane_to_region_dict(self):
        """
        Creates a dictionary mapping tracking planes to regions
        """
        self.tracking_plane_to_region = {}
        for region, camera_list in self.regions_dict.iteritems():
            for camera in camera_list:
                tracking_plane = get_tracking_plane_name(camera)
                self.tracking_plane_to_region[tracking_plane] = region

    def _create_region_to_anchor_plane_dict(self):
        """
        Creates a dictionary mapping regions to anchor planes
        """
        self.region_to_anchor_plane = {}
        for region, camera_list in self.regions_dict.iteritems():
            anchor_plane = get_tracking_plane_name(camera_list[0])
            self.region_to_anchor_plane[region] = anchor_plane 

    def _create_tracking_to_anchor_plane_dict(self):
        """
        Creates a dictionary mapping tracking planes to their anchor planes
        """
        self.tracking_to_anchor_plane = {}
        for tracking_plane, region in self.tracking_plane_to_region.iteritems():
            anchor_plane = self.region_to_anchor_plane[region]
            self.tracking_to_anchor_plane[tracking_plane] = anchor_plane

    def _create_tracking_plane_to_transform_dict(self):
        """
        Creates a dictionary of tracking plane --> tracking plane transforms for  each region.
        """
        self.tracking_plane_to_transform = {}
        for region, camera_pairs_list in self.camera_pairs_dict.iteritems():
            tf_dict = get_tracking_plane_transforms(camera_pairs_list)
            self.tracking_plane_to_transform[region] = tf_dict;

    def _create_camera_to_transform_dict(self):
        """
        Creates a dictionary of camera --> tracking plane homography transformations for each region
        """
        self.camera_to_transform = {}
        for region, camera_list in self.regions_dict.iteritems():
            transform_dict = {}
            for camera in camera_list:
                homography = file_tools.read_homography_calibration(camera)
                nrow = homography['rows']
                ncol = homography['cols']
                data = homography['data']
                matrix = numpy.array(data).reshape(nrow,ncol)
                transform_dict[camera] = matrix
            self.camera_to_transform[region] = transform_dict

# ----------------------------------------------------------------------------------------------------
def get_tracking_plane_transforms(camera_pairs_list):
    """
    Gets a dictionary of all transforms between tracking planes in the region.
    """
    tf_digraph = create_transform_digraph(camera_pairs_list)
    transform_dict = {}
    for node_0 in  tf_digraph.nodes():
        for node_1 in tf_digraph.nodes():
            tf_matrix = get_transform_matrix(tf_digraph, node_0, node_1)
            transform_dict[(node_0, node_1)] = tf_matrix
    return transform_dict

def create_transform_digraph(camera_pairs_list):
    """
    Creates the transform digraph given the camera paris list.
    """
    tf_digraph = networkx.DiGraph()

    # Add tracking planes as nodes to digraph
    for camera_0, camera_1 in camera_pairs_list:

        # Extract camera numbers and form name for tracking planes
        num_0 = int(camera_0.split('_')[-1])
        num_1 = int(camera_1.split('_')[-1])
        tracking_plane_0 = 'tracking_plane_{0}'.format(num_0)
        tracking_plane_1 = 'tracking_plane_{0}'.format(num_1)

        # Load 2d transforms and convert to 3x3 matrices for (homogeneous coords.)
        transform_0_to_1 = file_tools.read_transform_2d_calibration(camera_0,camera_1)
        transform_1_to_0 = transform_2d_inv(transform_0_to_1)
        matrix_0_to_1 = transform_2d_to_3x3_matrix(transform_0_to_1)
        matrix_1_to_0 = transform_2d_to_3x3_matrix(transform_1_to_0)

        tf_digraph.add_edge(tracking_plane_0, tracking_plane_1, tf_matrix=matrix_0_to_1)
        tf_digraph.add_edge(tracking_plane_1, tracking_plane_0, tf_matrix=matrix_1_to_0)

    return tf_digraph

def get_transform_matrix(tf_digraph, node0, node1):
    """
    Get the transform matrix from node0 to node1. This is defined as the composition
    of the transforms alone the shortest path from node0 to node1.
    """
    path = networkx.shortest_path(tf_digraph,node0,node1)
    path_edges = zip(path[:-1],path[1:]) 
    tf_matrix = numpy.eye(3,3)
    for edge0, edge1 in path_edges:
        tf_matrix_edge = tf_digraph[edge0][edge1]['tf_matrix']
        tf_matrix = numpy.dot(tf_matrix_edge, tf_matrix)
    return tf_matrix


def get_tracking_plane_name(camera):
    num = int(camera.split('_')[-1])
    return 'tracking_plane_{0}'.format(num)


def transform_2d_to_3x3_matrix(transform):
    """
    Converts a 2d transform to a 3x3 matrix
    """
    ang = transform['rotation']
    tx = transform['translation_x']
    ty = transform['translation_y']
    matrix = numpy.array([ 
        [numpy.cos(ang), -numpy.sin(ang),  tx], 
        [numpy.sin(ang),  numpy.cos(ang),  ty], 
        [             0.0,               0.0, 1.0],
        ])
    return matrix


def transform_2d_inv(transform):
    """
    Computes the inverse of a 2d transform
    """
    ang = transform['rotation']
    tx = transform['translation_x']
    ty = transform['translation_y']
    transform_inv = {}
    transform_inv['rotation'] = -ang
    transform_inv['translation_x'] = -numpy.cos(ang)*tx - numpy.sin(ang)*ty
    transform_inv['translation_y'] =  numpy.sin(ang)*tx - numpy.cos(ang)*ty
    return transform_inv

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    import pylab

    if 0:
        t1 = {
                'rotation': 180.0*numpy.pi/180.0,
                'translation_x': 20.0,
                'translation_y': 0.0,
                }

        t2 = transform_2d_inv(t1)
        m1 = transform_2d_to_3x3_matrix(t1)
        m2 = transform_2d_to_3x3_matrix(t2)
        m22 = numpy.linalg.inv(m1)
        m3 = numpy.dot(m2,m1)
        m33 = numpy.dot(m22,m1)

        print(t1)
        print(t2)
        print()
        print(m1)
        print()
        print(m2)
        print()
        print(m22)
        print()
        print(m3)
        print()
        print(m33)
        print()

    if 0:
        camera_pairs_dict = file_tools.read_tracking_2d_camera_pairs()
        camera_pairs_list = camera_pairs_dict['maze']
        tf_digraph = create_transform_digraph(camera_pairs_list)
        print('nodes:')
        print(tf_digraph.nodes())
        print()
        print('edges:')
        print(tf_digraph.edges())
        print()
        m1 = get_transform_matrix(tf_digraph, 'tracking_plane_1',  'tracking_plane_1')
        m2 = get_transform_matrix(tf_digraph, 'tracking_plane_1', 'tracking_plane_1' )
        m1_m2 = numpy.dot(m1,m2)
        print(m1)
        print(m2)
        print(m1_m2)

    if 0:

        camera_pairs_dict = file_tools.read_tracking_2d_camera_pairs()
        camera_pairs_list = camera_pairs_dict['maze']
        transform_dict = get_tracking_plane_transforms(camera_pairs_list)
        print(len(transform_dict))
        for k,v in transform_dict.iteritems():
            print(k)
            print(v)
            print()

    if 1:
        import pylab

        # Create boundry points
        tf2d = Transforms2d()
        bndry_pts = []
        for x in range(0,640):
            bndry_pts.append((x,0))
        for y in range(0,480):
            bndry_pts.append((640,480))
        for x in range(639,-1,-1):
            bndry_pts.append((x,480))
        for y in range(479,-1,-1):
            bndry_pts.append((0,y))
        bndry_pts = numpy.array(bndry_pts,dtype=numpy.float)

        # Map camera boundaries to the anchor plane
        for camera in tf2d.regions_dict['maze']:

            bndry_pts_anchor = tf2d.camera_pts_to_anchor_plane(camera, bndry_pts)
            pylab.plot(bndry_pts_anchor[:,0], bndry_pts_anchor[:,1])

            x,y = tf2d.camera_pts_to_anchor_plane(camera, 320, 240)
            pylab.plot(x,y,'o')

        pylab.axis('equal')
        pylab.show()
        












