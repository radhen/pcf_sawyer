from scipy import spatial

import aolib.img as ig
import aolib.util as ut
import h5py
import numpy as np
import planefit
import scipy.ndimage
from broadcast_frame_test import BroadcastFrame
import tf

from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2, PointField
import rospy
import std_msgs.msg

import scipy.spatial
# import pcl

dilate = scipy.ndimage.morphology.binary_dilation

# todo: support multiple kinects
fx = 366.53131103515625
fy = 366.53131103515625
cx = 258.8788146972656
cy = 209.2510986328125
camera_up_dir = np.array([0., 1., 0])


def camera_matrix():
    return np.array([[fx, 0., cx],
                     [0., -fy, cy],
                     [0., 0., 1.]])


def ptm_from_depth(depth):
    """ Returns a "point map" ptm that maps pixel coordinates to 3D
    coordinates, and a flag that indicates whether the pixel contains a
    valid depth estimate; specifically, ptm[i, j, :] = [X, Y, Z, ok].
    """
    # adapted from https://github.com/OpenKinect/libfreenect2/issues/41#issuecomment-59919674
    depth = np.float32(depth)
    yi, xi = map(np.float32, np.mgrid[:depth.shape[0], :depth.shape[1]])
    z = depth / 1000.
    x = (xi - cx) * z / fx
    y = (yi - cy) * z / fy
    ok = depth > 0
    return np.concatenate([v[:, :, None] for v in [x, y, z, ok]], axis=2)


def nrm_from_ptm(ptm, step=1):
    p1 = np.roll(ptm, step, axis=0)
    p2 = np.roll(ptm, step, axis=1)
    v1 = ptm - p1
    v2 = ptm - p2
    nrm = ut.normalize_im(np.cross(v1[:, :, :3], v2[:, :, :3]))

    nrm[:step, :] = 0
    nrm[:, :step] = 0
    nrm[ptm[:, :, 3] < 1] = 0
    nrm[p1[:, :, 3] < 1] = 0
    nrm[p2[:, :, 3] < 1] = 0

    return np.concatenate([nrm, np.any(nrm != 0, axis=2)[:, :, np.newaxis]], axis=2)


def pts_from_ptm(ptm, inds=False):
    if inds:
        i, j = np.nonzero(ptm[:, :, 3] == 1)
        return ptm[i, j, :3], (i, j)
    else:
        return ptm[ptm[:, :, 3] == 1, :3]


def color_normals(ptm):
    nrm = nrm_from_ptm(ptm, 1)
    im = np.uint8(np.abs(nrm[:, :, :3]) * 255)
    im[nrm[:, :, 3] == 0] = 0
    return im


def fit_ground_plane(depth, num_inputs=4):
    # fit plane
    ptm = ptm_from_depth(depth)
    ptm_0 = pts_from_ptm(ptm)
    # publish_pointcloud(ptm_0)
    thresh = 0.02
    h, w = ptm.shape[:2]
    rect = (int(0.25 * w), int(0.4 * h), int(0.5 * w), int(0.25 * h))
    ptm_sub = ig.sub_img(ptm, rect)

    plane, _ = planefit.fit_plane_ransac(pts_from_ptm(ptm_sub), thresh, seed=0)
    # choose plane such that the normal points up
    if np.dot(plane[:3], camera_up_dir) < 0:
    #     print 'Flipping plane, so that the normal points up'
        plane = -plane
    # print 'Plane:', plane

    # show points that fit on plane
    nrm_vis = color_normals(ptm)
    dist = planefit.dist_from_plane_ptm(ptm, plane)
    on_plane = (dist <= thresh) & (ptm[:, :, 3] > 0)
    plane_vis = nrm_vis.copy()
    plane_vis[on_plane] = 255
    # ig.show(['Plane inliers:', plane_vis, 'Normals:', nrm_vis, 'Flipped:', np.flipud(nrm_vis)])
    return plane


def in_largest_cc(mask, dilation=0):
    if dilation:
        mask = dilate(mask, iterations=dilation).astype('bool')
    labels, num_labels = scipy.ndimage.label(mask)
    counts = np.bincount(labels.flatten())
    counts[0] = -1
    if len(counts) < 2:
        raise RuntimeError('Could not find cc!')
    in_cc = (labels == counts.argmax())
    return in_cc & mask


def parallel_axes(plane):
    x, y, z = plane[:3]
    a = np.array
    if x == 0:
        u = a([0., z, -y])
    elif y == 0:
        u = a([z, 0., -x])
    else:
        u = a([-y, x, 0.])
    v = ut.normalized(np.cross(plane[:3], u))
    return a([u, v])


def ok_pts(pts):
    return pts[pts[:, 3] > 0, :3]



def remove_outliner(points):
    '''
    Statistical outline remover from PCL
    REFERNCE LINK_1: http://www.pointclouds.org/news/2013/02/07/python-bindings-for-the-point-cloud-library/
    REFERNCE LINK_2: http://strawlab.github.io/python-pcl/#pcl.StatisticalOutlierRemovalFilter
    :param points:
    :return: filtered point array
    '''
    p = pcl.PointCloud()
    p.from_array(points)
    fil = p.make_statistical_outlier_filter()
    fil.set_mean_k(np.mean(points))
    fil.set_std_dev_mul_thresh(np.std(points)*0.3)
    filtered_cloud = fil.filter()
    return p.to_array(filtered_cloud)


def fit_cylinder(depth0, depth, wrt_frame, plane_thresh=0.01, new_thresh=0.005, inlier_frac=0.9925, show=False):
    """

    :param depth0:kinectA_ir_optical_link
    :param depth:
    :param plane_thresh:
    :param new_thresh:
    :param inlier_frac:
    :param show: Generate website to debug images
    :return:
    """
    #ut.tic('fit_cylinder')
    #ut.tic('cylinder: initial frame')
    # compute occupancy map for the first frame (should precompute when using many frames...)
    plane = fit_ground_plane(depth0)
    ptm0 = ptm_from_depth(depth0)
    pts0 = pts_from_ptm(ptm0)

    on_plane0 = planefit.dist_from_plane_ptm(ptm0, plane) < plane_thresh
    on_plane0 = in_largest_cc(on_plane0)
    proj_plane0 = planefit.project_onto_plane(plane, ok_pts(ptm0[on_plane0]))

    print ("\n PUBLISHING GROUND PLANE")
    publish_pointcloud_and_center(proj_plane0, [1, 1, 1], "kinectB_link")

    ptm = ptm_from_depth(depth)
    has_pt = ptm[:, :, 3] != 0
    # find points that are very close to the table's surface
    pts, inds = pts_from_ptm(ptm, inds=True)
    # publish_pointcloud(pts)
    proj_plane = planefit.project_onto_plane(plane, pts)
    ok = ut.knnsearch(proj_plane0, proj_plane, method='kd')[0].flatten() <= 0.005
    near_surface = np.zeros(ptm.shape[:2], 'bool')
    near_surface[inds[0][ok], inds[1][ok]] = True

    # find points that are not in the original point cloud
    dist = ut.knnsearch(pts0, pts, method='kd')[0].flatten()
    new_pt = np.zeros_like(near_surface)
    ok = dist > new_thresh
    new_pt[inds[0][ok], inds[1][ok]] = True

    # todo: probably want to filter out the robot's gripper, e.g. with a height check
    occ_before_cc = new_pt & near_surface & has_pt
    occ = in_largest_cc(occ_before_cc)

    # segment object and find height
    pts = ptm[occ]
    pts = pts[pts[:, 3] > 0, :3]
    publish_object_pointcloud = pts

    height = np.percentile(np.abs(planefit.dist_to_plane(plane, pts)), 99.7)
    # find an ellipse that covers the occupied points
    # todo: want to deal with self-occlusion here (kinect won't see the backside of the object)
    pts = ok_pts(ptm[occ])
    proj = planefit.project_onto_plane(plane, pts)
    # add a slight bias toward choosing farther z points, since these will often be self-occluded
    center = np.array(list(np.median(proj[:, :2], axis=0)) + [np.percentile(proj[:, 2], 75.0)])

    publish_pointcloud_and_center(publish_object_pointcloud, list(center), wrt_frame)

    ## --------------- REMOVE OUTLINERS --------------------##
    #  clean_pointcloud = remove_outliner(publish_object_pointcloud)
    # publish_pointcloud_and_center(clean_pointcloud, list(center))

    d = np.sqrt(np.percentile(np.sum((proj - center) ** 2, 1), 95.))  # + 0.01
    scales = np.array([d, d])
    #ut.toc()
    print 'Center: ', center
    print 'Height: ', height
    print 'Diameter: ', d

    ut.tic('showing ellipse')
    # show ellipse
    if show:
        # show points in/out of cylinder
        nrm_vis = color_normals(ptm)
        if show:
            ptm_vis = ut.clip_rescale_im(ptm[:, :, 2], 0.3, 1.25)
            oval_vis1 = ptm_vis.copy()
            oval_missing_vis = ptm_vis.copy()
            for y in xrange(ptm.shape[0]):
                for x in xrange(ptm.shape[1]):
                    if ptm[y, x, 3]:
                        pt = ptm[y, x, :3]
                        proj = (planefit.project_onto_plane(plane, pt[None]) - center)[0]
                        ok = ((proj[:2] / scales[:2]) ** 2).sum() <= 1.
                        ok = ok and planefit.dist_to_plane(plane, pt[None], signed=True)[0] <= height
                        if ok and occ[y, x]:
                            oval_vis1[y, x] = 255
                        elif occ[y, x]:
                            # not covered
                            oval_missing_vis[y, x] = 255

        ptm_vis = ut.clip_rescale_im(ptm[:, :, 2], 0.05, 1.5)
        # show cylinder
        axes = parallel_axes(plane)
        pts_lo, pts_hi = [], []
        for theta in np.linspace(0., 2 * np.pi, 100):
            x = np.array([np.cos(theta), np.sin(theta)])
            pt = axes.T.dot(x * scales[:2]) + center
            pts_lo.append(pt.flatten())
            pts_hi.append(pt + height * plane[:3])

        # print plane[:3]
        pts_lo, pts_hi = map(np.array, [pts_lo, pts_hi])
        proj_lo = ut.inhomog(camera_matrix().dot(pts_lo.T)).T
        proj_hi = ut.inhomog(camera_matrix().dot(pts_hi.T)).T

        oval_vis = ptm_vis.copy()
        c1 = (128, 0, 0)
        c2 = (0, 0, 128)
        c3 = (0, 128, 0)
        oval_vis = ig.draw_lines(oval_vis, proj_lo[:-1], proj_lo[1:], colors=c1, width=2)
        oval_vis = ig.draw_lines(oval_vis, proj_hi[:-1], proj_hi[1:], colors=c2, width=2)
        oval_vis = ig.draw_lines(oval_vis, [proj_hi[0]], [proj_lo[0]], colors=c3, width=2)
        oval_vis = ig.draw_lines(oval_vis, [proj_hi[len(proj_hi) / 2]],
                                 [proj_lo[len(proj_hi) / 2]], colors=c3, width=2)

        def make_vis(x):
            v = ptm_vis.copy()
            v[x] = 255
            return np.flipud(v)

        if show:
            ig.show(['parametric ellipse:', np.flipud(oval_vis),
                     'ellipse:', np.flipud(oval_vis1),
                     'missing:', np.flipud(oval_missing_vis),
                     'occ:', make_vis(occ),
                     'occ_before_cc:', make_vis(occ_before_cc),
                     'near_surface', make_vis(near_surface),
                     'new_pt', make_vis(new_pt),
                     'has_pt', make_vis(has_pt),
                     'on_plane0', make_vis(on_plane0),
                     'input:', np.flipud(ptm_vis),
                     ])
        obj_vis = np.flipud(oval_vis)
        obj_vis = np.fliplr(obj_vis)
    #ut.toc()
    return [center, height, d, publish_object_pointcloud]



def publish_pointcloud_and_center(pts, center, wrt_frame):
    # centroid publishing stuff
    broadcast = tf.TransformBroadcaster()
    # point cloud publishing stuff
    debug_pointcloud = xyz_array_to_pointcloud2(pts, stamp=None, frame=wrt_frame)
    pub = rospy.Publisher('/debug_pointcloud', PointCloud2, queue_size=10)
    # general stuff
    rate = rospy.Rate(30)
    start = rospy.get_time()
    while not rospy.is_shutdown() and (rospy.get_time() - start < 3.0):
        broadcast.sendTransform(center, [0, 0, 0, 1],
                                rospy.Time.now(),
                                "object_wrt_kinect",
                                wrt_frame)
        pub.publish(debug_pointcloud)
        rate.sleep()
    # rospy.signal_shutdown("Finished publishing DEBUG pointcloud")
    print ("Finished publishing DEBUG pointcloud")


def xyz_array_to_pointcloud2(points, stamp=None, frame="kinectB_link"):
    '''
    Create a sensor_msgs.PointCloud2 from an array
    of points.
    reference LINK: https://www.programcreek.com/python/example/99841/sensor_msgs.msg.PointCloud2
    '''
    msg = PointCloud2()
    if stamp:
        msg.header.stamp = stamp
    if frame:
        msg.header.frame_id = frame
    if len(points.shape) == 3:
        msg.height = points.shape[1]
        msg.width = points.shape[0]
    else:
        msg.height = 1
        msg.width = len(points)
    msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12*points.shape[0]
    msg.is_dense = int(np.isfinite(points).all())
    msg.data = np.asarray(points, np.float32).tostring()

    return msg

# def test(data_file = '/media/ao/hd2/gelsight-data/entire_data_success_test_2_data_0.hdf5'):
def test(data_file='/media/ao/hd2/gelsight-data/kinect_data.hdf5'):
    depths = []
    with h5py.File(data_file, 'r') as f:
        for frame in [0, 100, 500, 750, 2000]:
            dset = f['step_%d' % frame]
            depth = dset['camera_depth'][:, :, 0]
            depths.append(np.array(depth, 'float32'))
    for i in xrange(1, len(depths)):
        fit_cylinder(depths[0], depths[i])
    ut.toplevel_locals()
