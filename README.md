# G-Code Generator for painting machine 

This Python script converts raster images into optimized G-code instructions for 3-axis CNC machines or robotic arms equipped with a paint brush. It features automated brush dipping, path optimization, and angled infill while maintaining the correct orientation of the source image.

## Features

* Dual Infill Modes: Supports both standard linear (lines) and intelligent concentric (contour-following) infill patterns for diverse artistic effects.

* Intelligent Diagonal Lead-ins/Outs: Uses synchronized 3-axis movement (X, Y, and Z) to create smooth diagonal entries and exits, preventing tool collisions with the edges of the petri dish.

* Automatic Paint Management: Tracks distance traveled and automatically triggers a re-dipping cycle in the petri dish when the brush runs out of paint.

* Safe Start Logic: Includes a dedicated vertical lift to a safe clearance height (Z-high) upon initial startup to avoid spills or collisions during the first travel move.

* Spiral Dip Pattern: Executes a randomized spiral motion within the petri dish to ensure even paint pickup and prevent wearing down a single spot.

* Integrated Wipe Sequence: Features a calculated "wipe" move against the dish rim after dipping to remove excess paint and prevent drips on the canvas.

* Dynamic Speed & Acceleration Control: Automatically switches between high-speed travel and precision painting speeds with dedicated acceleration profiles for each state.

* Path Optimization: Analyzes segment proximity to minimize unnecessary travel moves and optimizes the painting order of different image regions.

* Image-to-Gcode Conversion: Built-in processing that converts standard grayscale images into optimized painting paths with adjustable scale and resolution.
## Usage

Run the script from the terminal by specifying the input image and the desired output path:

bash
python painter.py <input_image> <output_gcode_file>

## Example:
bash
python painter.py logo.png output.gcode

## General Settings Variable Description 

* infill_type: Choose between 'lines' for traditional raster strokes or 'concentric' for paths that follow the contours of the image.

* target_width: The width of the finished painting in millimeters.

* infill_angle: The angle of the brush strokes in degrees (0 = horizontal).

* brush_w: The effective width of the brush tip in mm.

* overlap: Percentage of brush stroke overlap (0.0 to 1.0).

* feed: Movement speed of the machine during non-painting travel (mm/min).

* feed_paint: Specialized slower movement speed used while the brush is in contact with the paper or paint.

Coordinates & Heights
* x_off, y_off: Starting offset from the machine's (0,0) home position to the corner of the painting.

* z_paint: The Z-coordinate where the brush makes contact with the paper to apply paint.

* z_low: Lower travel height for moving between adjacent segments without hitting the frame.

* z_high: Safe clearance height for long travels and entering the dipping area to avoid collisions.

* mirror_y: Flips the Y-axis to match standard CNC coordinate systems.

* accel_travel: High acceleration setting for fast positioning moves.

* accel_paint: Lower acceleration for smooth, steady brush strokes.

Dipping Parameters
* min_dist / max_dist: The program randomly selects a travel distance between these two values for each dip to simulate natural paint depletion.

* dip_x, dip_y: Center coordinates of the paint container (Petri dish).

* dip_z: The exact Z-depth the brush reaches inside the paint container.

* dip_jitter: Adds a small random offset to each dip location to prevent wearing out one spot in the paint.

* dip_spiral_r: The radius of the spiral motion performed inside the dish to load the brush evenly.

* wipe_r: The radius used for the exit move to wipe the brush against the rim of the dish to prevent drips.
