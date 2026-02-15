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
        
        # BACKLASH COMPENSATION tracking
        self.backlash_x = cfg.get('backlash_x', 0.0)
        self.backlash_y = cfg.get('backlash_y', 0.0)
        self.backlash_threshold = cfg.get('backlash_threshold', 0.05)
        
        # Logical position (what we think we're at)
        self.logical_x = cfg['dip_x']
        self.logical_y = cfg['dip_y']
        
        # Physical offset applied (cumulative compensation)
        self.offset_x = 0.0
        self.offset_y = 0.0
        
        # Direction tracking: 1 (positive), -1 (negative), 0 (uninitialized)
        self.dir_x = 0
        self.dir_y = 0
        
        # Current feedrate for GRBL
        self.current_feed = cfg['feed']

    def _add_move(self, cmd, x=None, y=None, z=None, feed=None, comment=None):
        """Add a move with backlash compensation applied."""
        
        # If X or Y specified, check for direction changes and apply compensation
        if x is not None or y is not None:
            target_x = x if x is not None else self.logical_x
            target_y = y if y is not None else self.logical_y
            
            # Check X-axis backlash
            dx = target_x - self.logical_x
            if abs(dx) > self.backlash_threshold:
                new_dir_x = 1 if dx > 0 else -1
                
                # Direction change detected on X
                if self.dir_x != 0 and new_dir_x != self.dir_x:
                    change = self.backlash_x if new_dir_x == 1 else -self.backlash_x
                    self.offset_x += change
                    
                    # Insert compensation move
                    physical_x_fix = self.logical_x + self.offset_x
                    physical_y_fix = self.logical_y + self.offset_y
                    
                    # Safety: prevent negative coordinates
                    if physical_x_fix < 0: physical_x_fix = 0
                    if physical_y_fix < 0: physical_y_fix = 0
                    
                    self.gcode.append(f"; --- FIX X Backlash: {change:.3f}mm ---")
                    self.gcode.append(f"G0 X{physical_x_fix:.3f} Y{physical_y_fix:.3f} F{self.cfg.get('backlash_feed', 200)}")
                
                self.dir_x = new_dir_x
            
            # Check Y-axis backlash
            dy = target_y - self.logical_y
            if abs(dy) > self.backlash_threshold:
                new_dir_y = 1 if dy > 0 else -1
                
                # Direction change detected on Y
                if self.dir_y != 0 and new_dir_y != self.dir_y:
                    change = self.backlash_y if new_dir_y == 1 else -self.backlash_y
                    self.offset_y += change
                    
                    # Insert compensation move
                    physical_x_fix = self.logical_x + self.offset_x
                    physical_y_fix = self.logical_y + self.offset_y
                    
                    if physical_x_fix < 0: physical_x_fix = 0
                    if physical_y_fix < 0: physical_y_fix = 0
                    
                    self.gcode.append(f"; --- FIX Y Backlash: {change:.3f}mm ---")
                    self.gcode.append(f"G0 X{physical_x_fix:.3f} Y{physical_y_fix:.3f} F{self.cfg.get('backlash_feed', 200)}")
                
                self.dir_y = new_dir_y
            
            # Update logical position
            self.logical_x = target_x
            self.logical_y = target_y
        
        # Build the actual move with offsets
        final_x = (x + self.offset_x) if x is not None else None
        final_y = (y + self.offset_y) if y is not None else None
        
        # Safety: prevent negative coordinates
        if final_x is not None and final_x < 0: final_x = 0
        if final_y is not None and final_y < 0: final_y = 0
        
        # Construct G-code line
        line = cmd
        if final_x is not None: line += f" X{final_x:.3f}"
        if final_y is not None: line += f" Y{final_y:.3f}"
        if z is not None: line += f" Z{z:.3f}"
        
        # Add feedrate
        if feed is not None:
            line += f" F{int(feed)}"
            self.current_feed = feed
        elif cmd == "G1":
            # G1 always needs feedrate in GRBL
            line += f" F{int(self.current_feed)}"
        
        if comment:
            line = f"{comment}\n{line}"
        
        self.gcode.append(line)

    def _set_speed(self, mode='travel'):
        """Set feedrate for GRBL (no M204 acceleration commands)."""
        c = self.cfg
        # GRBL doesn't use M204 - acceleration is set via $$ settings
        # Just update the current feedrate
        self.current_feed = c['feed'] if mode == 'travel' else c['feed_paint']

    def _perform_dip_and_travel(self, target_x, target_y):
        c = self.cfg
        self.gcode.append(f"\n; --- CIKEL NAMAKANJA ---")
        
        # Lift Z to safe height first
        self._add_move("G0", z=c['z_low'], feed=3000)
        
        self._set_speed('travel')
        
        # Move to dip position with jitter
        ax = c['dip_x'] + random.uniform(-c['dip_jitter'], c['dip_jitter'])
        ay = c['dip_y'] + random.uniform(-c['dip_jitter'], c['dip_jitter'])
        
        self._add_move("G0", ax, ay, c['z_high'])
        self._add_move("G1", z=c['dip_z'], feed=3000)
        
        # Spiral in paint
        num_steps = int(c['dip_spiral_loops'] * 4)
        for i in range(num_steps):
            ang = i * (math.pi / 2)
            r = (i / num_steps) * c['dip_spiral_r']
            sx = ax + r * math.cos(ang)
            sy = ay + r * math.sin(ang)
            self._add_move("G1", sx, sy, feed=2500)
        
        # Calculate wipe position
        dx, dy = target_x - c['dip_x'], target_y - c['dip_y']
        dist = math.hypot(dx, dy)
        wx = c['dip_x'] + (dx/dist * c['wipe_r']) if dist > 0 else c['dip_x'] + c['wipe_r']
        wy = c['dip_y'] + (dy/dist * c['wipe_r']) if dist > 0 else c['dip_y']
        
        # Exit dip and wipe
        self._add_move("G0", z=c['z_wipe_exit'], feed=3000)
        self._add_move("G0", wx, wy)
        self._add_move("G0", z=c['z_high'], feed=3000)
        self._add_move("G0", target_x, target_y, c['z_low'])
        
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
        
        # GRBL HEADER - G90 (absolute), G21 (mm)
        self.gcode = ["G90 ; Absolute positioning", "G21 ; Millimeters"]
        
        if paths:
            self._perform_dip_and_travel(paths[0][0][0], paths[0][0][1])
            
            for path in paths:
                # BIDIRECTIONAL (v obe smeri)
                dist_to_start = math.hypot(path[0][0] - self.current_pos[0], path[0][1] - self.current_pos[1])
                dist_to_end = math.hypot(path[-1][0] - self.current_pos[0], path[-1][1] - self.current_pos[1])
                if dist_to_end < dist_to_start: 
                    path = path[::-1]
                
                # Move to start of path
                self._set_speed('travel')
                self._add_move("G0", path[0][0], path[0][1], c['z_low'])
                
                # Lower to paint height
                self._set_speed('paint')
                self._add_move("G1", z=c['z_paint'], feed=2500)
                self.current_pos = path[0]
                
                # Paint the path
                for i in range(1, len(path)):
                    px, py = path[i]
                    dist = math.hypot(px - self.current_pos[0], py - self.current_pos[1])
                    
                    # Check if need to re-dip
                    if (self.dist_since_dip + dist) > self.current_max_dist:
                        self._add_move("G0", z=c['z_low'], feed=3000)
                        self._perform_dip_and_travel(px, py)
                        self._set_speed('paint')
                        self._add_move("G1", z=c['z_paint'], feed=2500)
                    
                    self._add_move("G1", px, py)
                    self.dist_since_dip += dist
                    self.current_pos = (px, py)
                
                # Lift after path
                self._add_move("G0", z=c['z_low'], feed=3000)

        # GRBL END - lift to safe height and end program
        self._add_move("G0", z=c['z_high'], feed=3000)
        self.gcode.append("M2 ; End program")
        
        return "\n".join(self.gcode)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("input", help="Input image file")
    parser.add_argument("output", help="Output G-code file")
    args = parser.parse_args()

    config = {
        'target_width': 129.0,
        'brush_w':      1.0, 
        'overlap':      0.15,
        'infill_type':  'lines',
        'infill_angle': 90.0,    # Poljuben kot: 0, 45, 90, -45 itd.
        'min_dist':     90.0,
        'max_dist':     100.0,
        'z_paint':      0.0,
        'z_low':        1.0,
        'z_high':       8.0,
        'z_wipe_exit':  6.0,
        'x_off':        0.0,
        'y_off':        20.0,
        'dip_x':        60.0,
        'dip_y':        0.0,
        'dip_z':        0.3,
        'dip_jitter':   7.0,
        'dip_spiral_loops': 1.0,
        'dip_spiral_r': 10.0,
        'wipe_r':       15.0,
        'feed':         1500,
        'feed_paint':   400,
        'accel_travel': 60,     # Not used in GRBL (use $$ settings)
        'accel_paint':  10,     # Not used in GRBL (use $$ settings)
        
        # BACKLASH COMPENSATION SETTINGS
        'backlash_x':        0.5,    # X-axis backlash in mm
        'backlash_y':        1.6,    # Y-axis backlash in mm
        'backlash_threshold': 0.05,  # Minimum move to trigger compensation
        'backlash_feed':     200,    # Feedrate for compensation moves
    }

    painter = UltraPainter(config)
    gcode_output = painter.generate(args.input)
    
    with open(args.output, "w") as f:
        f.write(gcode_output)
    
    print(f"GRBL G-code with backlash compensation generated: {args.output}")
