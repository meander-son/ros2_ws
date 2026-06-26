from svgpathtools import svg2paths2, wsvg
import matplotlib.pyplot as plt

# 1. Load your messy SVG
paths, attributes, svg_attributes = svg2paths2('src/ilia_example.svg')

# 2. Setup your tracking lists
unvisited_paths = list(paths)
unvisited_attrs = list(attributes)

optimized_paths = []
optimized_attrs = []

# Start the robot pen at the origin (0, 0)
current_pen_position = 0.0 + 0.0j 

print(f"Optimizing {len(unvisited_paths)} paths...")

# 3. Main Optimization Loop
while unvisited_paths:
    best_idx = None
    shortest_distance = float('inf')
    should_flip = False
    
    # Search all remaining paths to find the closest one
    for i, path in enumerate(unvisited_paths):
        # Distance to the normal start of the path
        dist_to_start = abs(current_pen_position - path.start)
        # Distance to the end of the path (if we decide to draw it backwards)
        dist_to_end = abs(current_pen_position - path.end)
        
        if dist_to_start < shortest_distance:
            shortest_distance = dist_to_start
            best_idx = i
            should_flip = False
            
        if dist_to_end < shortest_distance:
            shortest_distance = dist_to_end
            best_idx = i
            should_flip = True

    # 4. Extract the chosen path and its original attributes
    chosen_path = unvisited_paths.pop(best_idx)
    chosen_attr = unvisited_attrs.pop(best_idx)
    
    # If drawing it backwards is closer, flip the direction of the geometric path
    if should_flip:
        chosen_path = chosen_path.reversed()
        
    # Move the pen to the end of this path (where the pen finishes drawing)
    current_pen_position = chosen_path.end
    
    # Save the optimized path
    optimized_paths.append(chosen_path)
    optimized_attrs.append(chosen_attr)

# 5. Export the sorted paths into a brand new SVG file
wsvg(optimized_paths, attributes=optimized_attrs, svg_attributes=svg_attributes, filename='src/optimized_output.svg')
print("Optimization complete! Saved to optimized_output.svg")

print("Generating visualization...")

# Create a large, square plot window
plt.figure(figsize=(10, 10))

# We need to track where the pen lifted off last to draw the travel lines
previous_end = None

for i, path in enumerate(optimized_paths):
    # 1. Plot the actual drawing stroke (Pen DOWN)
    # Iterate through the individual segments (lines/curves) that make up this path
    for segment in path:
        num_samples = 10 
        stroke_x = [segment.point(t/num_samples).real for t in range(num_samples + 1)]
        stroke_y = [segment.point(t/num_samples).imag for t in range(num_samples + 1)]
        
        # Draw the solid line (Blue)
        plt.plot(stroke_x, stroke_y, color='blue', linewidth=1)
        
    # 2. Plot the air-travel transition (Pen UP)
    if previous_end is not None:
        travel_x = [previous_end.real, path.start.real]
        travel_y = [previous_end.imag, path.start.imag]
        
        # Draw the dashed transition line (Red, semi-transparent)
        plt.plot(travel_x, travel_y, color='red', linestyle='--', linewidth=0.5, alpha=0.5)
        
    # Optional: Add a green dot at the very first starting point
    if i == 0:
        plt.plot(path.start.real, path.start.imag, marker='o', color='green', markersize=8, zorder=5)

    # Update our tracking variable for the next loop
    previous_end = path.end

# --- Formatting the Plot ---

# IMPORTANT: SVG coordinates start with (0,0) at the TOP-LEFT. 
# Standard math plots start at the BOTTOM-LEFT. We must invert the Y-axis 
# or your drawing will be upside down!
plt.gca().invert_yaxis()

# Ensure the aspect ratio is 1:1 so your drawing isn't stretched
plt.axis('equal') 

plt.title(f"Optimized Robot Path ({len(optimized_paths)} segments)\nBlue = Drawing | Dashed Red = Air Travel")
plt.legend()

# Show the interactive window
plt.show()