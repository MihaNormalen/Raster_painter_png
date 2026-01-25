# Smart Robotic Painter G-Code Generator

This Python script converts raster images into optimized G-code instructions for 3-axis CNC machines or robotic arms equipped with a paint brush. It features automated brush dipping, path optimization, and angled infill while maintaining the correct orientation of the source image.

## Features

* **Angled Infill:** Generates brush strokes at a specific angle (e.g., 30°, 45°) while the image geometry remains upright and aligned with the machine axes.
* **Smart Dipping Logic:** Automated routine for reloading paint in a Petri dish using a spiral motion and directional wiping.
* **Path Optimization:** Uses a Nearest Neighbor algorithm to reduce travel time and air moves between strokes.
* **Distance Tracking:** Automatically triggers a dipping sequence after the brush has traveled a user-defined distance (`max_dist`).
* **Coordinate Management:** Supports X, Y, and Z offsets and axis mirroring to match specific machine setups.

## Usage

Run the script from the terminal by specifying the input image and the desired output path:

```bash
python painter.py <input_image> <output_gcode_file>

## Example:
```bash
python painter.py logo.png output.gcode

## General Settings Variable Description 

* **target_width The width of the finished painting in millimeters.
* **infill_angleThe angle of the brush strokes in degrees (0 = horizontal).
* **brush_w The effective width of the brush tip in mm. 
* **overlapPercentage of brush stroke overlap (0.0 to 1.0).
* **feedMovement speed of the machine (mm/min).

## Coordinates & Heights
## VariableDescription
* **x_off, y_off Starting offset from the machine's (0,0) home position.
* **z_paintThe Z-coordinate where the brush makes contact with the paper.
* **z_lowTravel height for moving between adjacent segments.
* **z_highSafe height for long travels and entering the dipping area.
* **mirror_yFlips the Y-axis (usually necessary for CNC coordinate systems).

##Dipping Parameters
##VariableDescription
* **max_dist Maximum travel distance (mm) before the brush requires more paint.
* **dip_x, dip_y Center coordinates of the paint container (Petri dish).
* **dip_z Z-height for the brush tip during dipping.
* **dip_spiral_r The radius of the spiral motion inside the dish.
* **wipe_rThe radius used for wiping the brush against the rim of the dish.

License
This project is provided "as-is" for robotic and artistic experimentation.
