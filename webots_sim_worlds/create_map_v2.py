"""
Script used to create Webots maps from a YAML file, with adjustable cell size
to reduce the number of solids for faster simulation.
"""

import yaml
from PIL import Image
import numpy as np
import csv

if __name__ == '__main__':
    custom_maps_filepath: str = 'custom_maps/'
    map_name: str = 'iilab'
    cell_size: int = 2  # Number of pixels per Webots solid

    # Parse the YAML file
    yaml_filepath: str = custom_maps_filepath + map_name + '_config.yaml'
    with open(yaml_filepath, 'r') as stream:
        yaml_data = yaml.safe_load(stream)

    # Load map image
    image_filename: str = yaml_data['image']
    resolution: float = yaml_data['resolution']
    origin: [float, float, float] = yaml_data['origin']
    occupied_thresh: float = yaml_data['occupied_thresh']
    max_pixel_value_for_wall: int = int(255 * occupied_thresh)

    img: Image = Image.open(custom_maps_filepath + image_filename).convert('L')
    np_img: np.array = np.array(img)
    height: int = len(np_img)
    width: int = len(np_img[0])

    # Aggregate pixels into larger cells
    wall_cells_coords: [(float, float)] = []

    for row in range(0, height, cell_size):
        for col in range(0, width, cell_size):
            block = np_img[row:row+cell_size, col:col+cell_size]
            if np.any(block <= max_pixel_value_for_wall):
                # Compute center of the cell
                center_x = origin[0] + resolution * (col + min(cell_size, width-col) / 2)
                center_y = origin[1] + resolution * (height - row - min(cell_size, height-row) / 2)
                wall_cells_coords.append((center_x, center_y))

    print('num walls =', len(wall_cells_coords))

    # Create CSV points file
    with open(custom_maps_filepath + map_name + '_points.csv', 'w', newline='') as f:
        writer = csv.writer(f)

        # Add borders
        for x in range(width):
            writer.writerow((origin[0] + resolution * x, origin[1]))
            writer.writerow((origin[0] + resolution * x, origin[1] + resolution * (height - 1)))
        for y in range(1, height - 1):
            writer.writerow((origin[0], origin[1] + resolution * y))
            writer.writerow((origin[0] + resolution * (width - 1), origin[1] + resolution * y))

        # Add walls
        for coord in wall_cells_coords:
            writer.writerow(coord)

    # Read base Webots file
    base_map_webots_filepath: str = custom_maps_filepath + 'base_map.wbt'
    with open(base_map_webots_filepath, 'r') as f:
        webots_str: str = f.read()

    # Write new Webots file
    map_webots_filepath: str = custom_maps_filepath + map_name + '.wbt'
    with open(map_webots_filepath, 'w') as f:
        f.write(webots_str)

        # Add rectangular arena
        f.write('RectangleArena {\n')
        f.write('  translation ' + str(origin[0] + resolution * width / 2) + ' ' +
                str(origin[1] + resolution * height / 2) + ' 0.0\n')
        f.write('  floorSize ' + str(resolution * width) + ' ' + str(resolution * height) + '\n')
        f.write('  floorTileSize 0.25 0.25\n')
        f.write('  floorAppearance Parquetry {\n')
        f.write('    type "light strip"\n')
        f.write('  }\n')
        f.write('  wallHeight 0.05\n')
        f.write('}\n')

        # Add walls as solids
        index: int = 0
        for coord in wall_cells_coords:
            f.write('Solid {\n')
            f.write('    translation ' + str(coord[0]) + ' ' + str(coord[1]) + ' 0.025\n')
            f.write('    children [\n')
            f.write('        Shape {\n')
            f.write('            geometry Box {\n')
            f.write('                size ' + str(resolution * cell_size) + ' ' +
                    str(resolution * cell_size) + ' 0.5\n')
            f.write('            }\n')
            f.write('        }\n')
            f.write('    ]\n')
            f.write('    name "solid' + str(index) + '"\n')
            f.write('}\n')
            index += 1
