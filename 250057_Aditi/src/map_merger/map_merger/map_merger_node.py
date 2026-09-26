import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid

import numpy as np
import cv2
import copy


class MapMerger(Node):

    def __init__(self):
        super().__init__('map_merger')

        self.map1 = None
        self.map2 = None

        self.info1 = None
        self.info2 = None

        # -------------------------------------------------
        # Subscribers
        # -------------------------------------------------

        self.sub1 = self.create_subscription(
            OccupancyGrid,
            '/robot1/map',
            self.map1_cb,
            10
        )

        self.sub2 = self.create_subscription(
            OccupancyGrid,
            '/robot2/map',
            self.map2_cb,
            10
        )

        # -------------------------------------------------
        # Publisher
        # -------------------------------------------------

        self.pub = self.create_publisher(
            OccupancyGrid,
            '/global_map',
            10
        )

        # -------------------------------------------------
        # Merge timer
        # -------------------------------------------------

        self.timer = self.create_timer(
            2.0,
            self.merge_maps
        )

        self.get_logger().info(
            'Map Merger started.'
        )

    # =====================================================
    # MAP CALLBACKS
    # =====================================================

    def map1_cb(self, msg):

        self.map1 = np.array(
            msg.data,
            dtype=np.int8
        ).reshape(
            (msg.info.height, msg.info.width)
        )

        self.info1 = copy.deepcopy(msg.info)

    def map2_cb(self, msg):

        self.map2 = np.array(
            msg.data,
            dtype=np.int8
        ).reshape(
            (msg.info.height, msg.info.width)
        )

        self.info2 = copy.deepcopy(msg.info)

    # =====================================================
    # CONVERT OCCUPANCY GRID TO IMAGE
    # =====================================================

    def to_image(self, grid):
        """
        Convert OccupancyGrid values into an image
        suitable for ORB feature detection.

        OccupancyGrid:
            -1 = unknown
             0 = free
           100 = occupied

        Image:
            127 = unknown
            255 = free
              0 = occupied
        """

        img = np.zeros(
            grid.shape,
            dtype=np.uint8
        )

        img[grid == -1] = 127
        img[grid == 0] = 255
        img[grid == 100] = 0

        return img

    # =====================================================
    # MERGE OCCUPANCY VALUES
    # =====================================================

    def merge_values(self, map1, map2):
        """
        Merge two occupancy grids.

        Rules:
            - Unknown does not overwrite known cells.
            - Known cells are preserved.
            - Occupied cells have priority.
        """

        result = np.full(
            map1.shape,
            -1,
            dtype=np.int8
        )

        # -------------------------------------------------
        # Copy known cells from map 1
        # -------------------------------------------------

        known1 = map1 >= 0

        result[known1] = map1[known1]

        # -------------------------------------------------
        # Copy known cells from map 2
        # where map 1 is unknown
        # -------------------------------------------------

        known2 = map2 >= 0

        new_cells = (
            known2 &
            (result == -1)
        )

        result[new_cells] = map2[new_cells]

        # -------------------------------------------------
        # Occupied has priority
        # -------------------------------------------------

        occupied = (
            known2 &
            (map2 == 100)
        )

        result[occupied] = 100

        return result

    # =====================================================
    # MAIN MERGING FUNCTION
    # =====================================================

    def merge_maps(self):

        # -------------------------------------------------
        # Wait until both maps are available
        # -------------------------------------------------

        if self.map1 is None or self.map2 is None:
            return

        if self.info1 is None or self.info2 is None:
            return

        try:

            # =================================================
            # GET MAP SIZES
            # =================================================

            h1, w1 = self.map1.shape
            h2, w2 = self.map2.shape

            self.get_logger().debug(
                f'Map sizes: '
                f'robot1={w1}x{h1}, '
                f'robot2={w2}x{h2}'
            )

            # =================================================
            # CONVERT MAPS TO IMAGES
            # =================================================

            img1 = self.to_image(self.map1)
            img2 = self.to_image(self.map2)

            # =================================================
            # COMMON IMAGE SIZE
            # =================================================

            H = max(h1, h2)
            W = max(w1, w2)

            padded1 = np.full(
                (H, W),
                127,
                dtype=np.uint8
            )

            padded2 = np.full(
                (H, W),
                127,
                dtype=np.uint8
            )

            padded1[
                :h1,
                :w1
            ] = img1

            padded2[
                :h2,
                :w2
            ] = img2

            # =================================================
            # ORB FEATURE DETECTION
            # =================================================

            orb = cv2.ORB_create(
                nfeatures=1000
            )

            kp1, des1 = orb.detectAndCompute(
                padded1,
                None
            )

            kp2, des2 = orb.detectAndCompute(
                padded2,
                None
            )

            # =================================================
            # INITIAL ALIGNED MAP
            # =================================================

            aligned_map2 = np.full(
                (H, W),
                -1,
                dtype=np.int8
            )

            alignment_success = False

            # =================================================
            # ORB MATCHING
            # =================================================

            if (
                des1 is not None
                and des2 is not None
                and len(des1) >= 4
                and len(des2) >= 4
            ):

                matcher = cv2.BFMatcher(
                    cv2.NORM_HAMMING,
                    crossCheck=True
                )

                matches = matcher.match(
                    des1,
                    des2
                )

                matches = sorted(
                    matches,
                    key=lambda x: x.distance
                )

                if len(matches) >= 4:

                    # -----------------------------------------
                    # Points from robot 2
                    # -----------------------------------------

                    src_pts = np.float32([
                        kp2[m.trainIdx].pt
                        for m in matches
                    ]).reshape(
                        -1,
                        1,
                        2
                    )

                    # -----------------------------------------
                    # Corresponding points from robot 1
                    # -----------------------------------------

                    dst_pts = np.float32([
                        kp1[m.queryIdx].pt
                        for m in matches
                    ]).reshape(
                        -1,
                        1,
                        2
                    )

                    # -----------------------------------------
                    # Estimate transformation
                    # -----------------------------------------

                    M, mask = cv2.findHomography(
                        src_pts,
                        dst_pts,
                        cv2.RANSAC,
                        5.0
                    )

                    # =================================================
                    # WARP ROBOT 2 MAP
                    # =================================================

                    if M is not None:

                        # IMPORTANT:
                        # OpenCV 4.5.4 can fail when warpPerspective
                        # receives np.int8.
                        #
                        # Convert OccupancyGrid to float32 first.

                        map2_float = self.map2.astype(
                            np.float32
                        )

                        warped = cv2.warpPerspective(
                            map2_float,
                            M,
                            (W, H),

                            # Keep occupancy values discrete.
                            flags=cv2.INTER_NEAREST,

                            # Unknown outside the warped image.
                            borderMode=cv2.BORDER_CONSTANT,
                            borderValue=-1.0
                        )

                        # -----------------------------------------
                        # Convert back to OccupancyGrid type
                        # -----------------------------------------

                        aligned_map2 = np.rint(
                            warped
                        ).astype(
                            np.int8
                        )

                        alignment_success = True

                        # -----------------------------------------
                        # Count inliers if available
                        # -----------------------------------------

                        inliers = 0

                        if mask is not None:
                            inliers = int(
                                np.sum(mask)
                            )

                        self.get_logger().info(
                            f'ORB alignment successful. '
                            f'Matches: {len(matches)}, '
                            f'Inliers: {inliers}'
                        )

            # =================================================
            # FALLBACK
            # =================================================

            if not alignment_success:

                self.get_logger().warn(
                    'ORB alignment failed. '
                    'Using simple common-grid merge.'
                )

                aligned_map2[
                    :h2,
                    :w2
                ] = self.map2

            # =================================================
            # PUT MAP 1 INTO COMMON GRID
            # =================================================

            common_map1 = np.full(
                (H, W),
                -1,
                dtype=np.int8
            )

            common_map1[
                :h1,
                :w1
            ] = self.map1

            # =================================================
            # MERGE
            # =================================================

            global_map = self.merge_values(
                common_map1,
                aligned_map2
            )

            # =================================================
            # CREATE OCCUPANCY GRID MESSAGE
            # =================================================

            msg = OccupancyGrid()

            msg.header.stamp = (
                self.get_clock()
                .now()
                .to_msg()
            )

            msg.header.frame_id = 'map'

            # Copy map1 metadata
            msg.info = copy.deepcopy(
                self.info1
            )

            # Update dimensions
            msg.info.width = W
            msg.info.height = H

            # Keep the same resolution as robot 1
            msg.info.resolution = (
                self.info1.resolution
            )

            # -------------------------------------------------
            # Publish merged data
            # -------------------------------------------------

            msg.data = (
                global_map
                .flatten()
                .tolist()
            )

            self.pub.publish(msg)

            self.get_logger().info(
                f'Published /global_map: '
                f'{W} x {H}'
            )

        # =====================================================
        # ERROR HANDLING
        # =====================================================

        except Exception as e:

            self.get_logger().error(
                f'Map merge failed: {e}'
            )


# =========================================================
# MAIN
# =========================================================

def main(args=None):

    rclpy.init(args=args)

    node = MapMerger()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
