import searoute as sr
import numpy as np
import matplotlib.pyplot as plt
import cartopy.crs as ccrs
import cartopy.feature as cfeature
from shapely.geometry import Point, LineString
from cartopy.io import shapereader
import time
from tqdm import tqdm
from functools import lru_cache
import math
import zipfile
from xml.dom import minidom

# Read land geometries
land_shp = shapereader.natural_earth(resolution='110m',
                                     category='physical',
                                     name='land')
land_geom = list(shapereader.Reader(land_shp).geometries())

# Calculate great circle distance in nautical miles
def great_circle_distance(lat1, lon1, lat2, lon2):
    """
    Calculate the great circle distance between two points 
    on the earth (specified in decimal degrees)
    Returns distance in nautical miles
    """
    # Convert decimal degrees to radians
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    
    # Haversine formula
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.asin(math.sqrt(a))
    r = 3440.065  # Radius of earth in nautical miles
    return c * r

def normalize_longitude(lon):
    """Normalize longitude to [-180, 180] range"""
    while lon > 180:
        lon -= 360
    while lon < -180:
        lon += 360
    return lon

@lru_cache(maxsize=10000)
def is_on_land(lon, lat):
    """Check if a point is on land (cached for performance)"""
    point = Point(lon, lat)
    return any(geom.contains(point) for geom in land_geom)

# Extract points from KMZ file
kmz_file = "tectonic-summ2000-2015.kmz"
output_dir = "kmz_extracted"


def extract_kmz(kmz_file, output_dir):
    """
    Extracts a KMZ file to the specified output directory and returns coordinates.
    
    :param kmz_file: Path to the KMZ file.
    :param output_dir: Directory where the extracted files will be saved.
    :return: List of coordinate tuples (lon, lat)
    """
    import os
    
    # Create output directory if it doesn't exist
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    coordinate_points = []
    
    try:
        # Unzip KMZ
        with zipfile.ZipFile(kmz_file, 'r') as z:
            z.extract("doc.kml", path=output_dir)

        # Parse KML
        doc = minidom.parse(os.path.join(output_dir, "doc.kml"))
        coords = doc.getElementsByTagName("coordinates")
        
        for coord in coords:
            coord_text = coord.firstChild.nodeValue.strip()
            if coord_text:
                # Split by whitespace and commas to handle different formats
                coord_parts = coord_text.replace('\n', ' ').replace('\t', ' ').split()
                
                for part in coord_parts:
                    if ',' in part:
                        try:
                            # KML format is usually lon,lat,elevation or lon,lat
                            coords_split = part.split(',')
                            if len(coords_split) >= 2:
                                lon = float(coords_split[0])
                                lat = float(coords_split[1])
                                # Normalize longitude
                                lon = normalize_longitude(lon)
                                coordinate_points.append((lon, lat))
                        except (ValueError, IndexError):
                            continue
        
        print(f"Extracted {len(coordinate_points)} coordinate points from KMZ file")
        return coordinate_points
        
    except Exception as e:
        print(f"Error extracting KMZ file: {e}")
        return []

print("Extracting points from KMZ file...")
kmz_points = extract_kmz(kmz_file, output_dir)
# Define multiple origin ports
origins = [
    [153.18, -27.34, 'Brisbane'],  # Original Australia point
    [139.81, 35.62, 'Tokyo'],    # Tokyo area
    [103.75, 1.27, 'Singapore']   # Singapore area
]
max_distance_nm = 2000  # Maximum distance in nautical miles
margin = 100  # Margin in nautical miles

# Create a lat/lon grid with finer resolution
resolution = 1  # Grid resolution in degrees (smaller = finer mesh)
lats = np.arange(-90, 90, resolution)

# Function to get a more detailed route by breaking it into segments
@lru_cache(maxsize=500)
def get_detailed_route(start_lat, start_lon, end_lat, end_lon, num_segments=5):
    """
    Get a route without intermediate point calculations
    """
    try:
        start = [start_lon, start_lat]
        end = [end_lon, end_lat]
        direct_route = sr.searoute(start, end)
        return direct_route
    except Exception as e:
        return None

# Filter points and calculate routes for all origins
print("Calculating reachable points for all origins...")
all_reachable_points = []

for origin_idx, (origin_lon, origin_lat, origin_name) in enumerate(origins):
    print(f"\nProcessing origin {origin_idx + 1}/{len(origins)}: {origin_name}")
    
    # Create a lat/lon grid with finer resolution around each origin
    resolution = 1  # Grid resolution in degrees
    lons = np.arange(origin_lon - 90, origin_lon + 91, resolution)
    lons = np.array([normalize_longitude(lon) for lon in lons])
    lats = np.arange(-90, 90, resolution)
    
    # Pre-filter points by great circle distance
    pre_filtered_points = []
    
    print(f"Pre-filtering points by great circle distance for {origin_name}...")
    for lon in lons:
        for lat in lats:
            # Skip points that are already obviously too far
            gc_distance = great_circle_distance(origin_lat, origin_lon, lat, lon)
            
            if gc_distance > max_distance_nm:  
                continue
                
            # Skip land points
            if is_on_land(float(lon), float(lat)):
                continue
                
            pre_filtered_points.append((lon, lat, gc_distance))
    
    print(f"Filtered to {len(pre_filtered_points)} potential points for {origin_name}")
    
    # Sort by great circle distance - process closer points first
    pre_filtered_points.sort(key=lambda x: x[2])
    
    # Calculate routes for this origin
    origin_reachable_points = []
    for lon, lat, gc_dist in tqdm(pre_filtered_points, desc=f"Routes from {origin_name}"):
        try:
            # Compute sea route
            route = get_detailed_route(origin_lat, origin_lon, lat, lon)
            
            if route:
                distance = route.properties['length'] * 0.539957  # Convert km to nautical miles
                if distance <= max_distance_nm:
                    origin_reachable_points.append((lat, lon, distance, route, origin_idx, origin_name))
        except Exception as e:
            continue
    
    all_reachable_points.extend(origin_reachable_points)
    print(f"Found {len(origin_reachable_points)} reachable points from {origin_name}")

# Create the Pacific-centered plot with all origins
fig = plt.figure(figsize=(20, 12))
ax = plt.axes(projection=ccrs.PlateCarree(central_longitude=180))

# Set global extent
ax.set_global()

# Add map features
ax.add_feature(cfeature.LAND, color='lightgray', alpha=0.8)
ax.add_feature(cfeature.OCEAN, color='lightblue', alpha=0.3)
ax.add_feature(cfeature.COASTLINE, linewidth=0.5)
ax.add_feature(cfeature.BORDERS, linewidth=0.3)

# Add gridlines
gl = ax.gridlines(draw_labels=True, dms=True, x_inline=False, y_inline=False,
                  linewidth=0.5, color='gray', alpha=0.5)
gl.top_labels = False
gl.right_labels = False

# Define colors and markers for each origin
origin_colors = ['purple', 'blue', 'green']
origin_markers = ['o', 's', '^']
origin_labels = []

# Plot all origins
for i, (origin_lon, origin_lat, origin_name) in enumerate(origins):
    ax.plot(origin_lon, origin_lat, marker=origin_markers[i], color=origin_colors[i], 
            markersize=8, transform=ccrs.PlateCarree(), zorder=5, 
            markeredgecolor='black', markeredgewidth=2)
    origin_labels.append(f'{origin_name} (Origin)')

# Plot KMZ points in black
if kmz_points:
    kmz_lons = [point[0] for point in kmz_points]
    kmz_lats = [point[1] for point in kmz_points]
    
    ax.scatter(kmz_lons, kmz_lats, c='red', s=40, alpha=1, 
               transform=ccrs.PlateCarree(), zorder=4, 
               marker='.', label=f'KMZ Points ({len(kmz_points)})',
               edgecolors='black', linewidths=0.5)

# Plot reachable points for each origin with different colors
if all_reachable_points:
    for origin_idx in range(len(origins)):
        # Filter points for this origin
        origin_points = [point for point in all_reachable_points if point[4] == origin_idx]
        
        if origin_points:
            lats_plot = [point[0] for point in origin_points]
            lons_plot = [point[1] for point in origin_points]
            distances = [point[2] for point in origin_points]
            
            # Create scatter plot with origin-specific colors
            scatter = ax.scatter(lons_plot, lats_plot, c=distances, cmap=f'{["Purples", "Blues", "Greens"][origin_idx]}', 
                               s=10, alpha=0.6, transform=ccrs.PlateCarree(), zorder=3,
                               edgecolors=origin_colors[origin_idx], linewidths=0.5)

# # Create a combined colorbar showing distance ranges
# if all_reachable_points:
#     all_distances = [point[2] for point in all_reachable_points]
#     # Create a dummy scatter for the colorbar
#     dummy_scatter = ax.scatter([], [], c=[], cmap='viridis', s=0)
#     cbar = plt.colorbar(dummy_scatter, ax=ax, shrink=0.8, pad=0.02)
#     cbar.set_label('Distance (Nautical Miles)', rotation=270, labelpad=20)
#     # Set the colorbar limits
#     cbar.mappable.set_clim(vmin=min(all_distances), vmax=max(all_distances))

# Add title
total_points = len(all_reachable_points)
plt.title(f'Reachable Destinations within {max_distance_nm} NM from Multiple Origins\n'
         f'Total: {total_points} reachable points', fontsize=16, pad=20)

# Create custom legend
legend_elements = []
for i, (origin_lon, origin_lat, origin_name) in enumerate(origins):
    origin_count = len([point for point in all_reachable_points if point[4] == i])
    legend_elements.append(plt.Line2D([0], [0], marker=origin_markers[i], color='w', 
                                    markerfacecolor=origin_colors[i], markersize=15,
                                    markeredgecolor='black', markeredgewidth=1,
                                    label=f'{origin_name}: {origin_count} points', linestyle='None'))

# Add KMZ points to legend if they exist
if kmz_points:
    legend_elements.append(plt.Line2D([0], [0], marker='.', color='red', 
                                    markersize=15, label=f'KMZ Points: {len(kmz_points)}', 
                                    linestyle='None'))

ax.legend(handles=legend_elements, loc='upper left', bbox_to_anchor=(0, 1))

# Save the figure
plt.savefig('sea_routes_pacific_multiple_origins.png', dpi=1000, bbox_inches='tight')

# Show the plot
plt.show()

print(f"\nSUMMARY:")
print(f"Total reachable destinations: {len(all_reachable_points)}")
print(f"KMZ points extracted: {len(kmz_points) if kmz_points else 0}")
for i, (origin_lon, origin_lat, origin_name) in enumerate(origins):
    origin_count = len([point for point in all_reachable_points if point[4] == i])
    print(f"{origin_name}: {origin_count} reachable points")
print(f"Map saved as: sea_routes_pacific_multiple_origins.png")