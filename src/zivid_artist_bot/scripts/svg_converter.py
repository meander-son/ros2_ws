import sys
import os
import math
import rclpy
from rclpy.node import Node
import zivid

import termios
import tty

import numpy as np
import skimage
import skimage.io
import skimage.color
import skimage.transform
from skimage import measure, exposure, feature, filters
from scipy import ndimage

# ---------------------------------------------------------
# SOTA SEGMENTATION IMPORT
# ---------------------------------------------------------
try:
    from rembg import remove, new_session
except ImportError:
    print("CRITICAL ERROR: 'rembg' is not installed.")
    print("Please run: pip install rembg[cpu] (or rembg[gpu])")
    sys.exit(1)


class ImageToSvgConverter(Node):
    def __init__(self, run_once=False, use_camera=True):
        super().__init__('capture_and_create_svg')
        
        self.run_once = run_once
        self.use_camera = use_camera
        self.image_processed = False 

        # Load the SOTA human segmentation model into memory once on startup
        self.get_logger().info('Loading u2net_human_seg model into memory...')
        self.rembg_session = new_session("u2net_human_seg")

        # Only initialize Zivid Camera if we aren't loading a local file
        if self.use_camera:
            self.get_logger().info('Connecting to Zivid Camera...')
            self.zivid_app = zivid.Application()
            try:
                self.camera = self.zivid_app.connect_camera()
                self.get_logger().info(f'Connected to Zivid Camera: {self.camera.info.model_name}')
            except Exception as e:
                self.get_logger().error(f'Failed to connect to Zivid camera: {e}')
                sys.exit(1)
                
            self.settings_path = '/home/mark/ros2_ws/src/zivid_artist_bot/config/zivid_settings.yml'
        else:
            self.get_logger().info('Local image mode active. Skipping Zivid camera initialization.')

    def wait_for_spacebar(self):
        """
        Pauses the terminal and waits for the user to press the Spacebar.
        Listens for raw keyboard input to avoid needing to press 'Enter'.
        """
        print("\n" + "="*50)
        self.get_logger().info('📷 CAMERA READY: Press [SPACEBAR] to capture...')
        print("="*50 + "\n")
        
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            # Set terminal to raw mode to capture instant keystrokes
            tty.setraw(sys.stdin.fileno())
            while True:
                char = sys.stdin.read(1)
                if char == ' ':  # Spacebar pressed
                    break
                if char == '\x03':  # Ctrl+C pressed
                    raise KeyboardInterrupt
        finally:
            # Always restore normal terminal settings, even if an error occurs
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            
        print("\n") # Add a newline after the raw input breaks
        self.get_logger().info('Spacebar detected! Triggering capture...')

    def capture_and_process(self):
        """
        Triggers a Zivid frame capture, extracts the raw 2D color data,
        saves a debug snapshot, and hands the frame off to the SVG pipeline.
        """
        if not self.use_camera:
            return

        if self.image_processed and self.run_once:
            return

        # ---> HANG AND WAIT FOR USER INPUT <---
        try:
            self.wait_for_spacebar()
        except KeyboardInterrupt:
            self.get_logger().info('\nCapture canceled by user (Ctrl+C). Shutting down.')
            rclpy.shutdown()
            return

        self.get_logger().info('Loading Zivid settings and capturing frame...')
        
        try:
            # Load settings from the config file if it exists, otherwise fall back to defaults
            if os.path.exists(self.settings_path):
                self.get_logger().info(f'Loading Zivid settings from: {self.settings_path}')
                settings = zivid.Settings.load(self.settings_path)
            else:
                self.get_logger().warn(f'Config file not found at {self.settings_path}. Using default capture settings.')
                settings = zivid.Settings(
                    acquisitions=[zivid.Settings.Acquisition()],
                    color=zivid.Settings2D(acquisitions=[zivid.Settings2D.Acquisition()]),
                )

            # Capture 2D/3D frame
            frame = self.camera.capture_2d_3d(settings)
            
            # Extract color array directly from the point cloud (Height, Width, 4) -> RGBA
            self.get_logger().info('Extracting 2D color data from point cloud...')
            rgba_data = frame.point_cloud().copy_data("rgba")

            # Save the raw image immediately for debugging with zero alterations
            debug_path = '/home/mark/ros2_ws/src/input_image.png'
            skimage.io.imsave(debug_path, rgba_data)
            self.get_logger().info(f'Unaltered debug image saved to {debug_path}')

            # Strip the Alpha channel to create a standard RGB image for processing
            image_rgb = rgba_data[:, :, :3]

            # Mark as processed and generate SVG layout
            self.image_processed = True
            self.generate_svg(image_rgb)

            if self.run_once:
                self.get_logger().info('Processing complete. Shutting down node.')
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f'Error during camera capture or image parsing: {e}')
            if self.run_once:
                rclpy.shutdown()

    def process_local_file(self, file_path):
        self.get_logger().info(f'Loading local image file: {file_path}')
        if not os.path.exists(file_path):
            self.get_logger().error(f'File not found: {file_path}')
            return False

        try:
            image_rgb = skimage.io.imread(file_path)
            
            # Strip Alpha channel if it exists in the original file
            if image_rgb.shape[-1] == 4:
                image_rgb = image_rgb[:, :, :3]
                
            self.get_logger().info('Local image loaded successfully. Generating SVG...')
            self.generate_svg(image_rgb)
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to process local image: {e}')
            return False

    def _generate_hatching_lines(self, image, mask, angle_deg, threshold, spacing_px):
        lines = []
        h, w = image.shape
        theta = np.deg2rad(angle_deg)
        cos_t = np.cos(theta)
        sin_t = np.sin(theta)

        corners = np.array([[0,0], [w,0], [w,h], [0,h]])
        d_values = corners[:, 0]*cos_t + corners[:, 1]*sin_t
        d_min, d_max = np.min(d_values), np.max(d_values)

        for d in np.arange(d_min, d_max, spacing_px):
            pts = []
            if abs(sin_t) > 1e-5:
                y0 = d / sin_t
                if 0 <= y0 <= h: pts.append((0, y0))
                
                yw = (d - w*cos_t) / sin_t
                if 0 <= yw <= h: pts.append((w, yw))
                    
            if abs(cos_t) > 1e-5:
                x0 = d / cos_t
                if 0 <= x0 <= w: pts.append((x0, 0))
                    
                xh = (d - h*sin_t) / cos_t
                if 0 <= xh <= w: pts.append((xh, h))

            if len(pts) >= 2:
                pts = sorted(pts)
                p1, p2 = pts[0], pts[-1]

                dist = math.hypot(p2[0]-p1[0], p2[1]-p1[1])
                steps = int(dist)
                if steps == 0: 
                    continue

                x_coords = np.linspace(p1[0], p2[0], steps)
                y_coords = np.linspace(p1[1], p2[1], steps)

                x_idx = np.clip(x_coords.astype(int), 0, w-1)
                y_idx = np.clip(y_coords.astype(int), 0, h-1)

                valid = (mask[y_idx, x_idx]) & (image[y_idx, x_idx] < threshold)

                padded = np.pad(valid, (1, 1), mode='constant', constant_values=False)
                diff = np.diff(padded.astype(int))
                starts = np.where(diff == 1)[0]
                ends = np.where(diff == -1)[0]

                for s, e in zip(starts, ends):
                    if e - s > 3:
                        lines.append((x_coords[s], y_coords[s], x_coords[e-1], y_coords[e-1]))
        return lines

    def generate_svg(self, image_rgb):
        self.get_logger().info('Running high-resolution background removal...')
        
        output_rgba = remove(image_rgb, session=self.rembg_session)
        alpha_channel = output_rgba[:, :, 3]
        raw_mask = (alpha_channel / 255.0).astype(np.float32)

        image_gray = skimage.color.rgb2gray(image_rgb)

        self.get_logger().info('Cropping to subject bounding box...')
        crop_mask = raw_mask > 0.1 
        rows = np.any(crop_mask, axis=1)
        cols = np.any(crop_mask, axis=0)
        
        if np.any(rows) and np.any(cols):
            ymin, ymax = np.where(rows)[0][[0, -1]]
            xmin, xmax = np.where(cols)[0][[0, -1]]

            pad_y = int(0.05 * (ymax - ymin))
            pad_x = int(0.05 * (xmax - xmin))
            ymin, ymax = max(0, ymin - pad_y), min(image_gray.shape[0], ymax + pad_y)
            xmin, xmax = max(0, xmin - pad_x), min(image_gray.shape[1], xmax + pad_x)

            raw_mask = raw_mask[ymin:ymax, xmin:xmax]
            image_gray = image_gray[ymin:ymax, xmin:xmax]

        p2, p98 = np.percentile(image_gray, (2, 98))
        image_enhanced = exposure.rescale_intensity(image_gray, in_range=(p2, p98))

        image_clahe = exposure.equalize_adapthist(image_enhanced, clip_limit=0.03)
        image_clahe = exposure.adjust_gamma(image_clahe, gamma=0.8)

        a4_width_mm, a4_height_mm = 297, 210
        dpi = 96
        mm_to_px = dpi / 25.4

        aspect_ratio = image_gray.shape[1] / image_gray.shape[0]
        if aspect_ratio > (a4_width_mm / a4_height_mm):
            output_w_mm = a4_width_mm
            output_h_mm = a4_width_mm / aspect_ratio
        else:
            output_h_mm = a4_height_mm
            output_w_mm = a4_height_mm * aspect_ratio

        out_w_px = int(output_w_mm * mm_to_px)
        out_h_px = int(output_h_mm * mm_to_px)

        image_scaled = skimage.transform.resize(image_clahe.astype(np.float32), (out_h_px, out_w_px), order=1)
        
        self.get_logger().info('Finalizing mask scaling...')
        mask_scaled_float = skimage.transform.resize(raw_mask, (out_h_px, out_w_px), order=3)
        mask_binary = mask_scaled_float > 0.5
        
        labels = measure.label(mask_binary)
        if labels.max() > 0:
            component_sizes = np.bincount(labels.flat)
            component_sizes[0] = 0  
            largest_size = np.max(component_sizes)
            min_size_threshold = largest_size * 0.05
            valid_labels = np.where(component_sizes >= min_size_threshold)[0]
            mask_binary = np.isin(labels, valid_labels)
            
        mask_smooth = ndimage.gaussian_filter(mask_binary.astype(float), sigma=1.0)
        mask_scaled = mask_smooth > 0.5

        self.get_logger().info('Rendering layered etching paths...')
        svg_lines = []
        svg_lines.append(f'<svg width="{output_w_mm:.1f}mm" height="{output_h_mm:.1f}mm" viewBox="0 0 {out_w_px} {out_h_px}" xmlns="http://www.w3.org/2000/svg">')
        svg_lines.append(f'<rect width="{out_w_px}" height="{out_h_px}" fill="white"/>')

        stroke_width = 0.5 * mm_to_px 

        layers = [
            {"angle": 45,  "threshold": 0.85, "spacing_mm": 1.5},  
            {"angle": 135, "threshold": 0.60, "spacing_mm": 1.5},  
            {"angle": 15,  "threshold": 0.35, "spacing_mm": 1.2},  
            {"angle": 105, "threshold": 0.15, "spacing_mm": 1.0}   
        ]

        for layer in layers:
            spacing_px = layer["spacing_mm"] * mm_to_px
            lines = self._generate_hatching_lines(image_scaled, mask_scaled, layer["angle"], layer["threshold"], spacing_px)
            for (x1, y1, x2, y2) in lines:
                svg_lines.append(f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" stroke="black" stroke-linecap="round" stroke-width="{stroke_width:.2f}"/>')

        self.get_logger().info('Extracting structural contours...')
        image_smooth_filter = filters.gaussian(image_scaled, sigma=2.0)
        edges = feature.canny(image_smooth_filter, sigma=1.0, low_threshold=0.15, high_threshold=0.3, mask=mask_scaled)
        
        edge_contours = measure.find_contours(edges, 0.5)
        for contour in edge_contours:
            if len(contour) > 8:
                path_data = 'M ' + ' L '.join([f'{x:.1f},{y:.1f}' for y, x in contour])
                svg_lines.append(f'<path d="{path_data}" stroke="black" stroke-linecap="round" stroke-linejoin="round" stroke-width="{stroke_width:.2f}" fill="none"/>')

        contours = measure.find_contours(mask_scaled, 0.5)
        if contours:
            largest_contour = max(contours, key=len)
            path_data = 'M ' + ' L '.join([f'{x:.1f},{y:.1f}' for y, x in largest_contour]) + ' Z'
            svg_lines.append(f'<path d="{path_data}" stroke="black" stroke-linecap="round" stroke-linejoin="round" stroke-width="{(0.8 * mm_to_px):.2f}" fill="none"/>')

        svg_lines.append('</svg>')

        output_file = '/home/mark/ros2_ws/src/output.svg'
        with open(output_file, 'w') as f:
            f.write('\n'.join(svg_lines))

        self.get_logger().info(f'Success! Ultimate Etched SVG saved to {output_file}')


def main(args=None):
    rclpy.init(args=args)
    
    image_path = None
    for arg in sys.argv:
        if arg.startswith('--image='):
            image_path = arg.split('=')[1]
            
    run_once_flag = '--once' in sys.argv or image_path is not None
    use_camera_flag = image_path is None  # Only use camera if no local image is provided
    
    node = ImageToSvgConverter(run_once=run_once_flag, use_camera=use_camera_flag)
    
    if image_path:
        node.process_local_file(image_path)
        node.destroy_node()
        rclpy.shutdown()
    else:
        node.capture_and_process()
        
        if rclpy.ok():
            rclpy.spin(node)
            node.destroy_node()
            rclpy.shutdown()
 
if __name__ == '__main__':
    main()