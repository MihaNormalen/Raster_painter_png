import math
import argparse
import random
import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
from skimage import measure

class SafeRasterPainter:
    def __init__(self, cfg):
        self.cfg = cfg
        self.gcode = []
        self.dist_since_dip = 0
        self.dip_count = 0 
        self.current_pos = (cfg['x_off'], cfg['y_off']) 
        self.current_max_dist = random.uniform(cfg.get('min_dist', 120.0), cfg.get('max_dist', 250.0))

    def _update_pos(self, x, y):
        self.current_pos = (x, y)

    def _set_machine_speed(self, speed_type='travel'):
        """Sets Feedrate, Acceleration (M204), and Max Feedrate (M203)"""
        c = self.cfg
        if speed_type == 'travel':
            f = c['feed']
            accel = c.get('accel_travel', 200)
        else: # paint
            f = c['feed_paint']
            accel = c.get('accel_paint', 20)
        
        self.gcode.append("M400") # Wait for moves to finish before changing acceleration
        self.gcode.append(f"M204 P{accel} T{accel}") 
        self.gcode.append(f"M203 X{f} Y{f} Z{f}") 
        self.gcode.append(f"G0 F{f}") 

    def _perform_dip_and_travel(self, target_x, target_y):
        c = self.cfg
        self.gcode.append(f"\n; --- Namakanje (Accels + Wipe-on-Exit) ---")
        
        # Jitter for petri longevity
        j_x = random.uniform(-c['dip_jitter'], c['dip_jitter'])
        j_y = random.uniform(-c['dip_jitter'], c['dip_jitter'])
        active_x, active_y = c['dip_x'] + j_x, c['dip_y'] + j_y
        
        # --- 1. DIAGONAL IN ---
        self._set_machine_speed('travel')
        dx = c['dip_x'] - self.current_pos[0]
        dy = c['dip_y'] - self.current_pos[1]
        dist = math.hypot(dx, dy)
        
        if dist > 0:
            dx, dy = dx / dist, dy / dist
            edge_x_in = c['dip_x'] - dx * c['wipe_r']
            edge_y_in = c['dip_y'] - dy * c['wipe_r']
        else:
            edge_x_in, edge_y_in = c['dip_x'] + c['wipe_r'], c['dip_y']
        
        # Travel to edge at safe high Z, then move to center
        self.gcode.append(f"G0 X{edge_x_in:.3f} Y{edge_y_in:.3f} Z{c['z_high']}")
        self.gcode.append(f"G0 X{active_x:.3f} Y{active_y:.3f}")

        # --- 2. MIXING ---
        self._set_machine_speed('paint') # Use paint acceleration for mixing
        self.gcode.append(f"G1 Z{c['dip_z']:.3f} F800")
        direction = 1 if (self.dip_count % 2 == 0) else -1
        self.dip_count += 1
        theta, max_theta = 0, 2.5 * math.pi
        while theta <= max_theta:
            r = (theta / max_theta) * c['dip_spiral_r']
            self.gcode.append(f"G1 X{active_x + r * math.cos(theta * direction):.3f} Y{active_y + r * math.sin(theta * direction):.3f}")
            theta += 0.1
        self.gcode.append("G4 P300") 

        # --- 3. WIPE-ON-EXIT LOGIC ---
        self._set_machine_speed('travel') # Switch back to travel acceleration
        
        dx_out = target_x - c['dip_x']
        dy_out = target_y - c['dip_y']
        dist_out = math.hypot(dx_out, dy_out)
        
        if dist_out > 0:
            dx_out, dy_out = dx_out / dist_out, dy_out / dist_out
            wipe_edge_x = c['dip_x'] + dx_out * c['wipe_r']
            wipe_edge_y = c['dip_y'] + dy_out * c['wipe_r']
        else:
            wipe_edge_x, wipe_edge_y = c['dip_x'] + c['wipe_r'], c['dip_y']
        
        # Wipe sequence
        self.gcode.append(f"G0 Z{c['z_wipe_exit']:.3f}") # Drop to wipe height
        self.gcode.append(f"G0 X{wipe_edge_x:.3f} Y{wipe_edge_y:.3f}") # Move to rim
        self.gcode.append(f"G0 Z{c['z_high']:.3f}") # Lift high to clear
        self.gcode.append(f"G0 X{target_x:.3f} Y{target_y:.3f} Z{c['z_low']:.3f}") # Move to target start
        
        self.dist_since_dip = 0
        self.current_max_dist = random.uniform(c.get('min_dist', 120.0), c.get('max_dist', 250.0))
        self._update_pos(target_x, target_y)

    def _optimize_path_order(self, paths):
        if len(paths) <= 1: return paths
        optimized = []; remaining = list(paths); current = remaining.pop(0); optimized.append(current)
        while remaining:
            current_end = current[-1]; nearest_idx = 0; min_dist = float('inf'); should_reverse = False
            for idx, path in enumerate(remaining):
                d_start = math.hypot(current_end[1]-path[0][1], current_end[0]-path[0][0])
                d_end = math.hypot(current_end[1]-path[-1][1], current_end[0]-path[-1][0])
                if d_start < min_dist: min_dist = d_start; nearest_idx = idx; should_reverse = False
                if d_end < min_dist: min_dist = d_end; nearest_idx = idx; should_reverse = True
            current = remaining.pop(nearest_idx)
            if should_reverse: current = current[::-1]
            optimized.append(current)
        return optimized

    def _generate_linear_segments(self, img, res):
        c = self.cfg; img_w, img_h = img.size; target_w_mm = c['target_width']
        target_h_mm = target_w_mm * (img_h / img_w); angle_rad = math.radians(c['infill_angle'])
        cos_a, sin_a = math.cos(angle_rad), math.sin(angle_rad); step_mm = c['brush_w'] * (1 - c['overlap'])
        diag = math.hypot(target_w_mm, target_h_mm); segments = []
        for h in [i * step_mm for i in range(int(-diag/step_mm), int(diag/step_mm))]:
            path = []
            for d in [j/res for j in range(int(-diag*res), int(diag*res))]:
                cx, cy = target_w_mm/2, target_h_mm/2
                rx, ry = (d-cx)*cos_a-(h-cy)*sin_a+cx, (d-cx)*sin_a+(h-cy)*cos_a+cy
                px, py = int(rx*res), int(ry*res)
                if 0 <= px < img_w and 0 <= py < img_h and img.getpixel((px, py)) == 0:
                    path.append([ry + c['y_off'], rx + c['x_off']])
                else:
                    if len(path) > 1: segments.append(np.array(path))
                    path = []
            if len(path) > 1: segments.append(np.array(path))
        return segments

    def generate(self, img_path):
        c = self.cfg; img = Image.open(img_path).convert('L')
        if c.get('mirror_x', False): img = img.transpose(Image.FLIP_LEFT_RIGHT)
        if c.get('mirror_y', True): img = img.transpose(Image.FLIP_TOP_BOTTOM)
        img = img.point(lambda p: 0 if p < 140 else 255)
        res = 10.0; target_w_mm = c['target_width']
        img = img.resize((int(target_w_mm * res), int(target_w_mm * (img.height/img.width) * res)), Image.Resampling.NEAREST)
        
        if c['infill_type'] == 'concentric':
            binary = (np.array(img) < 140).astype(float); dist_map = distance_transform_edt(binary)
            step_px = (c['brush_w'] * (1 - c['overlap'])) * res; paths = []
            for d in np.arange(step_px/2, np.max(dist_map), step_px):
                for contour in measure.find_contours(dist_map, d):
                    paths.append(np.array([[pt[0]/res + c['y_off'], pt[1]/res + c['x_off']] for pt in contour]))
        else: paths = self._generate_linear_segments(img, res)

        self.gcode = ["G90", "G21"]
        if not paths: return "M2"
        paths = self._optimize_path_order(paths)
        
        # Initial Safe Move
        self._set_machine_speed('travel')
        self.gcode.append(f"G0 Z{c['z_high']}")
        self._perform_dip_and_travel(paths[0][0][1], paths[0][0][0])

        for path in paths:
            self._set_machine_speed('travel')
            self.gcode.append(f"G0 X{path[0][1]:.3f} Y{path[0][0]:.3f} Z{c['z_low']}")
            self._update_pos(path[0][1], path[0][0])
            for i in range(1, len(path)):
                px, py = path[i][1], path[i][0]
                seg_len = math.hypot(px - self.current_pos[0], py - self.current_pos[1])
                
                # Check for dip
                if (self.dist_since_dip + seg_len) > self.current_max_dist:
                    self._perform_dip_and_travel(px, py)
                
                self._set_machine_speed('paint')
                self.gcode.append(f"G1 Z{c['z_paint']:.3f}")
                self.gcode.append(f"G1 X{px:.3f} Y{py:.3f}")
                self.dist_since_dip += seg_len
                self._update_pos(px, py)
            
            self._set_machine_speed('travel')
            self.gcode.append(f"G0 Z{c['z_low']}")
            
        self.gcode.append("M2")
        return "\n".join(self.gcode)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("input"); parser.add_argument("output")
    args = parser.parse_args()

    config = {
        'infill_type':  'lines',
        'infill_angle': 0.0,
        'target_width': 280.0,
        'brush_w':      2.5, 
        'overlap':      0.3,
        'min_dist':     320.0,
        'max_dist':     550.0,
        'z_paint':      0.0,
        'z_low':        1.6,
        'z_high':       36.0,
        'z_wipe_exit':  13.0,
        'x_off':        16.0,
        'y_off':        78.0,
        'dip_x':        91.0,
        'dip_y':        25.0,
        'dip_z':        3.2,
        'dip_jitter':   5.0,
        'dip_spiral_r': 15.0,
        'wipe_r':       27.0,
        'feed':         1500,
        'feed_paint':   600,
        'accel_travel': 200,    # Pospeševanje za travel (mm/s²)
        'accel_paint':  20,     # Pospeševanje za barvanje (mm/s²)
    }

    painter = SafeRasterPainter(config)
    with open(args.output, 'w') as f:
        f.write(painter.generate(args.input))
