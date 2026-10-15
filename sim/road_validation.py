"""Conservative planar body/sidewalk and right-hand-lane checks."""
import numpy as np

# Includes the collision chassis, body, bumpers, and mirrors.
HALF_WIDTH = 1.025
HALF_LENGTH = 2.45

def vehicle_polygon(position, forward):
    forward = np.asarray(forward, dtype=float)
    forward /= np.linalg.norm(forward)
    lateral = np.array([forward[1], -forward[0]])
    return np.array([position + s*HALF_WIDTH*lateral + t*HALF_LENGTH*forward
        for s,t in [(-1,-1), (1,-1), (1,1), (-1,1)]])

def separation(a, b):
    """Positive separating-axis gap, or negative overlap, for convex polygons."""
    edges = np.vstack((np.roll(a,-1,axis=0)-a, np.roll(b,-1,axis=0)-b))
    axes = np.column_stack((-edges[:,1],edges[:,0]))
    axes /= np.linalg.norm(axes,axis=1)[:,None]
    ap, bp = a @ axes.T, b @ axes.T
    return float(np.max(np.maximum(bp.min(0)-ap.max(0),ap.min(0)-bp.max(0))))

class RoadValidator:
    def __init__(self, controls):
        nx,nz = controls['blocks_x'], controls['blocks_z']
        px,pz,w = controls['pitch_x'],controls['pitch_z'],controls['road_width']
        self.left=-nx*px/2+px
        self.right=self.left+px
        self.mid=-nz*pz/2+pz
        self.width=w
        hx,hz = (px-w)/2,(pz-w)/2
        self.sidewalks = [np.array([[x-hx,z-hz],[x+hx,z-hz],[x+hx,z+hz],[x-hx,z+hz]])
            for x in np.linspace(-(nx-1)*px/2,(nx-1)*px/2,nx)
            for z in np.linspace(-(nz-1)*pz/2,(nz-1)*pz/2,nz)]
        self.samples = self.overlaps = self.wrong_lane = self.lane_samples = 0
        self.minimum_gap = float('inf')
        self.minimum_lane_gap = float('inf')

    def check(self, position, forward):
        polygon = vehicle_polygon(np.asarray(position), forward)
        gap = min(separation(polygon, box) for box in self.sidewalks)
        self.samples += 1
        self.overlaps += gap < 0
        self.minimum_gap = min(self.minimum_gap, gap)
        # Require the full body to occupy the right lane on settled straights.
        x,z = position
        lane_gap = None
        if forward[1] > .98 and z < self.mid-1.3*self.width:
            lane_gap = min(polygon[:,0].min()-(self.left-self.width/2), self.left-polygon[:,0].max())
        elif forward[1] > .98 and z > self.mid+1.3*self.width:
            lane_gap = min(polygon[:,0].min()-(self.right-self.width/2), self.right-polygon[:,0].max())
        elif forward[0] > .98 and self.left+1.1*self.width < x < self.right-1.3*self.width:
            lane_gap = min(polygon[:,1].min()-self.mid, self.mid+self.width/2-polygon[:,1].max())
        if lane_gap is not None:
            self.lane_samples += 1
            self.wrong_lane += lane_gap < 0
            self.minimum_lane_gap = min(self.minimum_lane_gap, float(lane_gap))
        return gap

    def report(self):
        return {'vehicle_envelope_m': [2*HALF_WIDTH,2*HALF_LENGTH],
            'physics_samples': self.samples, 'sidewalk_overlap_samples': int(self.overlaps),
            'minimum_sidewalk_separating_gap_m': self.minimum_gap,
            'straight_lane_samples': self.lane_samples,
            'wrong_lane_samples': int(self.wrong_lane),
            'minimum_straight_lane_clearance_m': self.minimum_lane_gap,
            'method': 'Oriented conservative vehicle rectangle vs native sidewalk block rectangles; straight-lane bounds; each physics step'}
