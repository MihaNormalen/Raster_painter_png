import math
import argparse
import random
import numpy as np
from PIL import Image
from scipy import ndimage
import time

class PathOptimizer:
    """Ultra-hitra optimizacija poti (Spatial Sort + NN)."""
    @staticmethod
    def optimize(paths, start_pos):
        num_paths = len(paths)
        if num_paths == 0: return []
        
        if num_paths > 5000:
            return sorted(paths, key=lambda p: (p[0][0] // 20, p[0][1] if (p[0][0] // 20) % 2 == 0 else -p[0][1]))
        
        optimized = []
        remaining = list(paths)
        curr = np.array(start_pos)
        while remaining:
            idx = min(range(len(remaining)), key=lambda i: math.hypot(remaining[i][0][0]-curr[0], remaining[i][0][1]-curr[1]))
            path = remaining.pop(idx)
            optimized.append(path)
            curr = np.array(path[-1])
        return optimized

class UltraPainter:
    def __init__(self, cfg):
        self.cfg = cfg
        self.gcode = []
        self.dist_since_dip = 0
        self.current_pos = (cfg['dip_x'], cfg['dip_y']) 
        self.current_max_dist = random.uniform(cfg['min_dist'], cfg['max_dist'])

    def _set_speed(self, mode='travel'):
        c = self.cfg
        accel = c['accel_travel'] if mode == 'travel' else c['accel_paint']
        feed = c['feed'] if mode == 'travel' else c['feed_paint']
        self.gcode.append("M400")
        self.gcode.append(f"M204 P{accel} T{accel}")
        self.gcode.append(f"G1 F{feed}")

    def _perform_dip_and_travel(self, target_x, target_y):
        c = self.cfg
        self.gcode.append(f"\n; --- CIKEL NAMAKANJA ---")
        self.gcode.append(f"G0 Z{c['z_low']:.3f} F3000")
        self._set_speed('travel')
        ax, ay = c['dip_x'] + random.uniform(-c['dip_jitter'], c['dip_jitter']), c['dip_y'] + random.uniform(-c['dip_jitter'], c['dip_jitter'])
        self.gcode.append(f"G0 X{ax:.3f} Y{ay:.3f} Z{c['z_high']:.3f}")
        self.gcode.append(f"G1 Z{c['dip_z']:.3f} F3000")
        
        num_steps = int(c['dip_spiral_loops'] * 4)
        for i in range(num_steps):
            ang = i * (math.pi / 2)
            r = (i / num_steps) * c['dip_spiral_r']
            self.gcode.append(f"G1 X{ax + r*math.cos(ang):.3f} Y{ay + r*math.sin(ang):.3f} F2500")
        
        dx, dy = target_x - c['dip_x'], target_y - c['dip_y']
        dist = math.hypot(dx, dy)
        wx = c['dip_x'] + (dx/dist * c['wipe_r']) if dist > 0 else c['dip_x'] + c['wipe_r']
        wy = c['dip_y'] + (dy/dist * c['wipe_r']) if dist > 0 else c['dip_y']
        
        self.gcode.append(f"G0 Z{c['z_wipe_exit']:.3f} F3000")
        self.gcode.append(f"G0 X{wx:.3f} Y{wy:.3f}")
        self.gcode.append(f"G0 Z{c['z_high']:.3f} F3000") 
        self.gcode.append(f"G0 X{target_x:.3f} Y{target_y:.3f} Z{c['z_low']:.3f}")
        
        self.dist_since_dip = 0
        self.current_max_dist = random.uniform(c['min_dist'], c['max_dist'])
        self.current_pos = (target_x, target_y)

    def generate(self, img_path):
        c = self.cfg
        img = Image.open(img_path).convert('L').transpose(Image.FLIP_TOP_BOTTOM)
        img = img.point(lambda p: 0 if p < 140 else 255)
        
        res = 4.8
        tw, th = int(c['target_width']*res), int(c['target_width']*(img.height/img.width)*res)
        img = img.resize((tw, th), Image.Resampling.NEAREST)
        arr = np.array(img) < 140

        raw_paths = []
        step_px = int((c['brush_w'] * (1 - c['overlap'])) * res)
        
        if c['infill_type'] == 'concentric':
            temp_arr = arr.copy()
            erosion_steps = max(1, step_px)
            while temp_arr.any():
                edges = temp_arr ^ ndimage.binary_erosion(temp_arr, structure=np.ones((3,3)))
                for y in range(edges.shape[0]):
                    line = []
                    for x in range(edges.shape[1]):
                        if edges[y, x]:
                            line.append((x/res + c['x_off'], y/res + c['y_off']))
                        else:
                            if len(line) > 1: raw_paths.append(line)
                            line = []
                    if len(line) > 1: raw_paths.append(line)
                temp_arr = ndimage.binary_erosion(temp_arr, iterations=erosion_steps)
        
        else:
            # LINES INFILL - generiraj črte s ROTIRANIM PATTERNOM
            angle_rad = math.radians(c.get('infill_angle', 0))
            cos_a = math.cos(angle_rad)
            sin_a = math.sin(angle_rad)
            h, w = arr.shape
            cx, cy = w / 2.0, h / 2.0
            
            # V rotiranem koordinatnem sistemu generiraj črte
            diag = int(math.hypot(w, h)) + 10
            
            for y_rot in range(-diag, diag, max(1, step_px)):
                line = []
                for x_rot in range(-diag, diag):
                    # Transform from rotated to original coordinates
                    orig_x = int(cx + x_rot * cos_a - y_rot * sin_a)
                    orig_y = int(cy + x_rot * sin_a + y_rot * cos_a)
                    
                    # Check if inside bounds and active
                    if 0 <= orig_x < w and 0 <= orig_y < h and arr[orig_y, orig_x]:
                        # Transform back to real world coordinates
                        real_x = orig_x / res + c['x_off']
                        real_y = orig_y / res + c['y_off']
                        line.append((real_x, real_y))
                    else:
                        if len(line) > 1:
                            raw_paths.append(line)
                        line = []
                
                if len(line) > 1:
                    raw_paths.append(line)

        # OPTIMIZACIJA
        paths = PathOptimizer.optimize(raw_paths, (c['dip_x'], c['dip_y']))
        
        self.gcode = ["G90", "G21"]
        if paths:
            self._perform_dip_and_travel(paths[0][0][0], paths[0][0][1])
            for path in paths:
                # BIDIRECTIONAL (v obe smeri)
                dist_to_start = math.hypot(path[0][0] - self.current_pos[0], path[0][1] - self.current_pos[1])
                dist_to_end = math.hypot(path[-1][0] - self.current_pos[0], path[-1][1] - self.current_pos[1])
                if dist_to_end < dist_to_start: path = path[::-1]
                
                self._set_speed('travel')
                self.gcode.append(f"G0 X{path[0][0]:.3f} Y{path[0][1]:.3f} Z{c['z_low']:.3f}")
                self._set_speed('paint')
                self.gcode.append(f"G1 Z{c['z_paint']:.3f} F2500")
                self.current_pos = path[0]
                
                for i in range(1, len(path)):
                    px, py = path[i]
                    dist = math.hypot(px - self.current_pos[0], py - self.current_pos[1])
                    if (self.dist_since_dip + dist) > self.current_max_dist:
                        self.gcode.append(f"G0 Z{c['z_low']:.3f} F3000")
                        self._perform_dip_and_travel(px, py)
                        self._set_speed('paint')
                        self.gcode.append(f"G1 Z{c['z_paint']:.3f} F2500")
                    self.gcode.append(f"G1 X{px:.3f} Y{py:.3f}")
                    self.dist_since_dip += dist
                    self.current_pos = (px, py)
                self.gcode.append(f"G0 Z{c['z_low']:.3f} F3000")

        self.gcode.append("M400")
        self.gcode.append(f"G0 Z{c['z_high']:.3f} F3000\nM2")
        return "\n".join(self.gcode)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("input"); parser.add_argument("output")
    args = parser.parse_args()

    config = {
        'target_width': 1030.0,
        'brush_w':      1.3, 
        'overlap':      0.15,
        'infill_type':  'lines',
        'infill_angle': 90.0,    # Poljuben kot: 0, 45, 90, -45 itd.
        'min_dist':     240.0,
        'max_dist':     280.0,
        'z_paint':      0.0,
        'z_low':        2.2,
        'z_high':       31.0,
        'z_wipe_exit':  16.0,
        'x_off':        252.0,
        'y_off':        231.0,
        'dip_x':        66.0,
        'dip_y':        861.0,
        'dip_z':        0.3,
        'dip_jitter':   20.0,
        'dip_spiral_loops': 1.0,
        'dip_spiral_r': 50.0,
        'wipe_r':       70.0,
        'feed':         12000,
        'feed_paint':   400,
        'accel_travel': 12000, 
        'accel_paint':  200,  
    }

    painter = UltraPainter(config)
    with open(args.output, "w") as f:
        f.write(painter.generate(args.input))
