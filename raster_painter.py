import math
import argparse
from PIL import Image

class SafeRasterPainter:
    def __init__(self, cfg):
        self.cfg = cfg
        self.gcode = []
        self.dist_since_dip = 0
        self.current_pos = (cfg['x_off'], cfg['y_off']) 

    def _update_pos(self, x, y):
        self.current_pos = (x, y)

    def _perform_dip_and_travel(self, target_x, target_y):
        """ Vaša originalna logika namakanja (2.25 kroga spirale) """
        c = self.cfg
        cur_x, cur_y = self.current_pos
        dip_x, dip_y = c['dip_x'], c['dip_y']
        
        self.gcode.append(f"; --- SMART DIP SEQUENCE ---")
        dx, dy = dip_x - cur_x, dip_y - cur_y
        dist = math.hypot(dx, dy)
        safe_r = c['wipe_r'] + 5.0
        
        if dist > safe_r:
            ratio = (dist - safe_r) / dist
            self.gcode.append(f"G0 X{cur_x + dx * ratio:.3f} Y{cur_y + dy * ratio:.3f} Z{c['z_high']} F{c['feed']}")
            self.gcode.append(f"G0 X{dip_x} Y{dip_y}")
        else:
            self.gcode.append(f"G0 Z{c['z_high']} F{c['feed']}")
            self.gcode.append(f"G0 X{dip_x} Y{dip_y}")

        self.gcode.append(f"G1 Z{c['dip_z']} F2000")
        d_theta, max_theta, theta = 0.1, 4.5 * math.pi, 0
        while theta <= max_theta:
            r = (theta / max_theta) * c['dip_spiral_r']
            self.gcode.append(f"G1 X{dip_x + r * math.cos(theta):.3f} Y{dip_y + r * math.sin(theta):.3f} F1500")
            theta += d_theta
        self.gcode.append("G4 P300")
        
        self.gcode.append(f"G1 Z{c['dip_z'] + 2.0} F1000")
        angle = math.atan2(target_y - dip_y, target_x - dip_x)
        self.gcode.append(f"G1 X{dip_x + c['wipe_r'] * math.cos(angle):.3f} Y{dip_y + c['wipe_r'] * math.sin(angle):.3f} F1500")
        self.gcode.append(f"G0 X{dip_x + safe_r * math.cos(angle):.3f} Y{dip_y + safe_r * math.sin(angle):.3f} Z{c['z_high']} F{c['feed']}")
        self.gcode.append(f"G0 X{target_x:.3f} Y{target_y:.3f} Z{c['z_low']} F{c['feed']}")
        
        self.dist_since_dip = 0
        self._update_pos(target_x, target_y)

    def generate(self, img_path):
        c = self.cfg
        # Naloži sliko in uporabi zrcaljenje če je potrebno
        img = Image.open(img_path).convert('L')
        if c.get('mirror_x', False): img = img.transpose(Image.FLIP_LEFT_RIGHT)
        if c.get('mirror_y', True):  img = img.transpose(Image.FLIP_TOP_BOTTOM)
        img = img.point(lambda p: 0 if p < 140 else 255)
        
        res = 4 # pikslov na mm
        target_w_mm = c['target_width']
        target_h_mm = target_w_mm * (img.height / img.width)
        img = img.resize((int(target_w_mm * res), int(target_h_mm * res)), Image.Resampling.NEAREST)
        img_w, img_h = img.size

        # Matematika za nagnjeno polnilo
        angle_rad = math.radians(c['infill_angle'])
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)
        
        # Razmik med linijami (projekcija na pravokotno os)
        step_mm = c['brush_w'] * (1 - c['overlap'])
        
        # Določi območje skeniranja (diagonalno, da pokrijemo vse)
        diag = math.hypot(target_w_mm, target_h_mm)
        segments = []

        # Skeniramo po 'trakovih' (h) in znotraj trakov po 'dolžini' (d)
        for h in [i * step_mm for i in range(int(-diag/step_mm), int(diag/step_mm))]:
            start_pt = None
            for d in [j/res for j in range(int(-diag*res), int(diag*res))]:
                # Inverzna rotacija: pretvori nagnjene koordinate (d, h) v ravne (x, y)
                # Središče rotacije je sredina slike
                cx, cy = target_w_mm / 2, target_h_mm / 2
                rx = (d - cx) * cos_a - (h - cy) * sin_a + cx
                ry = (d - cx) * sin_a + (h - cy) * cos_a + cy
                
                # Preveri če je točka znotraj slike
                px, py = int(rx * res), int(ry * res)
                
                is_black = False
                if 0 <= px < img_w and 0 <= py < img_h:
                    if img.getpixel((px, py)) == 0:
                        is_black = True
                
                if is_black:
                    if start_pt is None:
                        start_pt = (rx + c['x_off'], ry + c['y_off'])
                else:
                    if start_pt is not None:
                        segments.append({
                            'x1': start_pt[0], 'y1': start_pt[1],
                            'x2': rx + c['x_off'], 'y2': ry + c['y_off']
                        })
                        start_pt = None
            if start_pt:
                segments.append({'x1': start_pt[0], 'y1': start_pt[1], 'x2': rx + c['x_off'], 'y2': ry + c['y_off']})

        # --- G-KODA IN OPTIMIZACIJA POTI ---
        self.gcode = ["G90", "G21"]
        self.gcode.append(f"G0 Z{c['z_high']}")
        
        if segments:
            self._perform_dip_and_travel(segments[0]['x1'], segments[0]['y1'])

        while segments:
            cur_x, cur_y = self.current_pos
            best_dist, best_idx, rev = float('inf'), -1, False
            
            for i, s in enumerate(segments):
                d1 = math.hypot(s['x1'] - cur_x, s['y1'] - cur_y)
                d2 = math.hypot(s['x2'] - cur_x, s['y2'] - cur_y)
                if d1 < best_dist: best_dist, best_idx, rev = d1, i, False
                if d2 < best_dist: best_dist, best_idx, rev = d2, i, True
            
            s = segments.pop(best_idx)
            x_s, y_s = (s['x2'], s['y2']) if rev else (s['x1'], s['y1'])
            x_e, y_e = (s['x1'], s['y1']) if rev else (s['x2'], s['y2'])

            if self.dist_since_dip > c['max_dist']:
                self._perform_dip_and_travel(x_s, y_s)

            self.gcode.append(f"G0 X{x_s:.3f} Y{y_s:.3f} Z{c['z_low']} F{c['feed']}")
            self.gcode.append(f"G1 Z{c['z_paint']} F600")
            self.gcode.append(f"G1 X{x_e:.3f} Y{y_e:.3f}")
            self.gcode.append(f"G0 Z{c['z_low']}")
            
            self.dist_since_dip += math.hypot(x_e - x_s, y_e - y_s)
            self._update_pos(x_e, y_e)

        self.gcode.append(f"G0 Z{c['z_high']}\nM2")
        return "\n".join(self.gcode)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("input")
    parser.add_argument("output")
    args = parser.parse_args()

    config = {
        'infill_angle': 0.0,   # Kot slikanja (slika ostane ravna)
        'target_width': 180.0,
        'brush_w':      1.5,    # Širina čopiča
        'overlap':      0.4,    # Prekrivanje
        'max_dist':     200.0,  # Max pot z enim namakanjem
        
        'z_paint':      0.0,    # Kontaktna višina
        'z_low':        1.4,    
        'z_high':       25.0,
        'x_off':        00.0,   # Odmik od roba mize
        'y_off':        60.0,
        
        'mirror_x':     False,
        'mirror_y':     True,
        
        'dip_x':        91.0,   # Center petrijevke
        'dip_y':        25.0,
        'dip_z':        1.5,
        'dip_spiral_r': 18.0,   # Radij spirale v petrijevki
        'wipe_r':       28.0,   # Radij brisanja ob rob
        'feed':         300
    }

    painter = SafeRasterPainter(config)
    with open(args.output, 'w') as f:
        f.write(painter.generate(args.input))
    print(f"G-koda generirana. Slika je ravna, čopič slika pod kotom {config['infill_angle']}°.")
