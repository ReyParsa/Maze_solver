import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math

# Reuse generation utilities (no ROS deps) by local import. If path differs when installed,
# we fall back to a simple internal generator (depth-first maze) matching generate_maze_sdf.
try:
    from robot_maze_simulation.tools.generate_maze_sdf import generate_maze, walls_to_segments
except Exception:
    # minimal fallback versions
    def generate_maze(rows: int, cols: int, seed: int = None, cell_size: float = 0.4):
        import random
        if seed is not None:
            random.seed(seed)
        walls = [[{'top': True, 'right': True, 'bottom': True, 'left': True} for _ in range(cols)] for _ in range(rows)]
        visited = [[False]*cols for _ in range(rows)]
        stack = [(0,0)]
        visited[0][0] = True
        while stack:
            r,c = stack[-1]
            nbs = []
            if r>0 and not visited[r-1][c]: nbs.append((r-1,c))
            if c<cols-1 and not visited[r][c+1]: nbs.append((r,c+1))
            if r<rows-1 and not visited[r+1][c]: nbs.append((r+1,c))
            if c>0 and not visited[r][c-1]: nbs.append((r,c-1))
            if nbs:
                nr,nc = random.choice(nbs)
                if nr==r-1:
                    walls[r][c]['top']=False; walls[nr][nc]['bottom']=False
                elif nr==r+1:
                    walls[r][c]['bottom']=False; walls[nr][nc]['top']=False
                elif nc==c-1:
                    walls[r][c]['left']=False; walls[nr][nc]['right']=False
                elif nc==c+1:
                    walls[r][c]['right']=False; walls[nr][nc]['left']=False
                visited[nr][nc]=True
                stack.append((nr,nc))
            else:
                stack.pop()
        return walls
    def walls_to_segments(walls, rows, cols, cell_size):
        segs=[]
        w=cols*cell_size; h=rows*cell_size
        x0=-w/2.0; y0=-h/2.0
        for r in range(rows):
            for c in range(cols):
                cx=x0+c*cell_size+cell_size/2.0
                cy=y0+r*cell_size+cell_size/2.0
                half=cell_size/2.0
                wcell=walls[r][c]
                if wcell.get('top',False): segs.append(((cx-half,cy-half),(cx+half,cy-half)))
                if wcell.get('bottom',False): segs.append(((cx-half,cy+half),(cx+half,cy+half)))
                if wcell.get('left',False): segs.append(((cx-half,cy-half),(cx-half,cy+half)))
                if wcell.get('right',False): segs.append(((cx+half,cy-half),(cx+half,cy+half)))
        uniq=[]; seen=set()
        for a,b in segs:
            key=(round(a[0],4),round(a[1],4),round(b[0],4),round(b[1],4))
            keyr=(round(b[0],4),round(b[1],4),round(a[0],4),round(a[1],4))
            if key in seen or keyr in seen: continue
            seen.add(key); uniq.append((a,b))
        return uniq


class MazePublisherNode(Node):
    def __init__(self):
        super().__init__('maze_publisher_node')
        # parameters to mirror launch (rows, cols, cell_size, seed)
        self.declare_parameter('rows', 10)
        self.declare_parameter('cols', 10)
        self.declare_parameter('cell_size', 0.4)
        self.declare_parameter('seed', 0)
        self.declare_parameter('publish_period', 4.0)
        self.rows = self.get_parameter('rows').get_parameter_value().integer_value
        self.cols = self.get_parameter('cols').get_parameter_value().integer_value
        self.cell_size = self.get_parameter('cell_size').get_parameter_value().double_value
        self.seed = self.get_parameter('seed').get_parameter_value().integer_value
        period = self.get_parameter('publish_period').get_parameter_value().double_value

        self.pub = self.create_publisher(Path, 'maze_occupancy', 10)
        self._segments = None
        self._build_maze()
        self.timer = self.create_timer(period, self._publish)
        self.get_logger().info(f'Maze publisher: rows={self.rows} cols={self.cols} cell_size={self.cell_size} segments={len(self._segments)}')

    def _build_maze(self):
        try:
            walls = generate_maze(self.rows, self.cols, seed=self.seed if self.seed!=0 else None, cell_size=self.cell_size)
            self._segments = walls_to_segments(walls, self.rows, self.cols, self.cell_size)
        except Exception as e:
            self.get_logger().error(f'Failed to generate maze segments: {e}')
            self._segments = []

    def _publish(self):
        if self._segments is None:
            return
        # Encode each wall segment as two consecutive poses (start, end) so parser can rebuild occupancy.
        path = Path()
        path.header.frame_id = 'map'
        stamp = self.get_clock().now().to_msg()
        path.header.stamp = stamp
        for (sx,sy),(ex,ey) in self._segments:
            p1 = PoseStamped(); p1.header = path.header; p1.pose.position.x = sx; p1.pose.position.y = sy
            p2 = PoseStamped(); p2.header = path.header; p2.pose.position.x = ex; p2.pose.position.y = ey
            path.poses.append(p1)
            path.poses.append(p2)
        self.pub.publish(path)
        self.get_logger().debug(f'Published maze with {len(self._segments)} segments ({len(path.poses)} poses).')


def main(args=None):
    rclpy.init(args=args)
    node = MazePublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
