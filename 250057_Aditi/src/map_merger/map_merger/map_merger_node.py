import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2


class MapMerger(Node):

    def __init__(self):
        super().__init__('map_merger')

        self.map1 = None
        self.map2 = None
        self.info1 = None
        self.info2 = None

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

        self.pub = self.create_publisher(
            OccupancyGrid,
            '/global_map',
            10
        )

        self.timer = self.create_timer(
            2.0,
            self.merge_maps
        )

        self.get_logger().info('Map Merger started.')

    def map1_cb(self, msg):
        self.map1 = np.array(
            msg.data,
            dtype=np.int8
        ).reshape(
            (msg.info.height, msg.info.width)
        )

        self.info1 = msg.info

    def map2_cb(self, msg):
        self.map2 = np.array(
            msg.data,
            dtype=np.int8
        ).reshape(
            (msg.info.height, msg.info.width)
        )

        self.info2 = msg.info

    def to_image(self, grid):
        """
        Convert OccupancyGrid values into an image suitable
        for feature detection.

        Unknown = 127
        Free    = 255
        Occupied = 0
        """

        img = np.zeros_like(grid, dtype=np.uint8)

        img[grid == -1] = 127
        img[grid == 0] = 255
        img[grid == 100] = 0

        return img

    def merge_values(self, map1, map2):
        """
        Merge occupancy values.

        Unknown does not overwrite known space.
        Occupied has priority over free.
        """

        result = np.full(
            map1.shape,
            -1,
            dtype=np.int8
        )

        # Map 1 known cells
        known1 = map1 >= 0
        result[known1] = map1[known1]

        # Map 2 known cells
        known2 = map2 >= 0

        # Where result is still unknown
        new_cells = known2 & (result == -1)
        result[new_cells] = map2[new_cells]

        # Occupied has priority
        occupied = known2 & (map2 == 100)
        result[occupied] = 100

        return result

    def merge_maps(self):

        if self.map1 is None or self.map2 is None:
            return

        if self.info1 is None or self.info2 is None:
            return

        try:

            h1, w1 = self.map1.shape
            h2, w2 = self.map2.shape

            self.get_logger().debug(
                f'Map sizes: robot1={w1}x{h1}, '
                f'robot2={w2}x{h2}'
            )

            img1 = self.to_image(self.map1)
            img2 = self.to_image(self.map2)

            # -------------------------------------------------
            # Make both images the same size for ORB matching
            # -------------------------------------------------

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

            padded1[:h1, :w1] = img1
            padded2[:h2, :w2] = img2

            # -------------------------------------------------
            # ORB feature detection
            # -------------------------------------------------

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

            aligned_map2 = np.full(
                (H, W),
                -1,
                dtype=np.int8
            )

            alignment_success = False

            # -------------------------------------------------
            # ORB matching
            # -------------------------------------------------

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

                    src_pts = np.float32([
                        kp2[m.trainIdx].pt
                        for m in matches
                    ]).reshape(-1, 1, 2)

                    dst_pts = np.float32([
                        kp1[m.queryIdx].pt
                        for m in matches
                    ]).reshape(-1, 1, 2)

                    M, mask = cv2.findHomography(
                        src_pts,
                        dst_pts,
                        cv2.RANSAC,
                        5.0
                    )

                    if M is not None:

                        warped = cv2.warpPerspective(
                            self.map2,
                            M,
                            (W, H),
                            borderValue=-1
                        )

                        aligned_map2 = warped
                        alignment_success = True

                        self.get_logger().info(
                            f'ORB alignment successful. '
                            f'Matches: {len(matches)}'
                        )

            # -------------------------------------------------
            # Fallback if ORB cannot align
            # -------------------------------------------------

            if not alignment_success:

                self.get_logger().warn(
                    'ORB alignment failed. '
                    'Using simple common-grid merge.'
                )

                aligned_map2[
                    :h2,
                    :w2
                ] = self.map2

            # -------------------------------------------------
            # Put map1 into common grid
            # -------------------------------------------------

            common_map1 = np.full(
                (H, W),
                -1,
                dtype=np.int8
            )

            common_map1[
                :h1,
                :w1
            ] = self.map1

            # -------------------------------------------------
            # Merge
            # -------------------------------------------------

            global_map = self.merge_values(
                common_map1,
                aligned_map2
            )

            # -------------------------------------------------
            # Publish OccupancyGrid
            # -------------------------------------------------

            msg = OccupancyGrid()

            msg.header.stamp = (
                self.get_clock()
                .now()
                .to_msg()
            )

            msg.header.frame_id = 'map'

            msg.info = self.info1

            msg.info.width = W
            msg.info.height = H

            msg.data = global_map.flatten().tolist()

            self.pub.publish(msg)

            self.get_logger().info(
                f'Published /global_map: '
                f'{W} x {H}'
            )

        except Exception as e:

            self.get_logger().error(
                f'Map merge failed: {e}'
            )


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
