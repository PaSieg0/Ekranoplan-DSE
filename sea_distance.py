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

# Define the origin port
origin = [153.6, -28]  
max_distance_nm = 2000  # Maximum distance in nautical miles
margin = 100  # Margin in nautical miles

# Create a lat/lon grid with finer resolution
resolution = 1  # Grid resolution in degrees (smaller = finer mesh)
start_lon = origin[0]
lons = np.arange(start_lon - 90, start_lon + 91, resolution)
lons = np.array([normalize_longitude(lon) for lon in lons])
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

# Filter points and calculate routes
print("Calculating reachable points...")
reachable_points = []

# Pre-filter points by great circle distance
origin_lat, origin_lon = origin[1], origin[0]  # Note: origin is [lon, lat]
pre_filtered_points = []

print("Pre-filtering points by great circle distance...")
for lon in lons:
    for lat in lats:
        # Skip points that are already obviously too far
        gc_distance = great_circle_distance(origin_lat, origin_lon, lat, lon)
        
        # This avoids unnecessary calculations while not removing potentially reachable points
        if gc_distance > max_distance_nm:  
            continue
            
        # Skip land points
        if is_on_land(float(lon), float(lat)):
            continue
            
        pre_filtered_points.append((lon, lat, gc_distance))

print(f"Filtered from {len(lons) * len(lats)} to {len(pre_filtered_points)} potential points")

# Sort by great circle distance - process closer points first
pre_filtered_points.sort(key=lambda x: x[2])

# Use tqdm for a progress bar
for lon, lat, gc_dist in tqdm(pre_filtered_points):
    try:
        # Compute sea route with enhanced detail
        route = get_detailed_route(origin_lat, origin_lon, lat, lon)
        
        if route:
            distance = route.properties['length'] * 0.539957  # Convert km to nautical miles
            if distance <= (max_distance_nm):
                reachable_points.append((lat, lon, distance, route))
    except Exception as e:
        continue

# Create the plot with Cartopy
fig = plt.figure(figsize=(20, 12))

# Use PlateCarree projection which handles dateline crossing well
ax = plt.axes(projection=ccrs.PlateCarree())

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

# Plot the origin point
ax.plot(origin_lon, origin_lat, 'go', markersize=12, 
        transform=ccrs.PlateCarree(), label='Origin Port', zorder=5)

# Prepare data for scatter plot
if reachable_points:
    lats_plot = [point[0] for point in reachable_points]
    lons_plot = [point[1] for point in reachable_points]
    distances = [point[2] for point in reachable_points]
    
    # Create scatter plot with color-coded distances
    scatter = ax.scatter(lons_plot, lats_plot, c=distances, cmap='coolwarm_r', 
                        s=20, alpha=0.7, transform=ccrs.PlateCarree(), zorder=3)
    
    # Add colorbar
    cbar = plt.colorbar(scatter, ax=ax, shrink=0.8, pad=0.02)
    cbar.set_label('Distance (Nautical Miles)', rotation=270, labelpad=20)

# Add title
plt.title(f'Reachable Destinations within {max_distance_nm} NM from Origin\n'
         f'Found {len(reachable_points)} reachable points', fontsize=16, pad=20)

# Add legend
ax.legend(loc='upper left', bbox_to_anchor=(0, 1))

# For better dateline handling, we can also create a version with shifted longitude
fig2 = plt.figure(figsize=(20, 12))
ax2 = plt.axes(projection=ccrs.PlateCarree(central_longitude=180))

# Set global extent
ax2.set_global()

# Add map features
ax2.add_feature(cfeature.LAND, color='lightgray', alpha=0.8)
ax2.add_feature(cfeature.OCEAN, color='lightblue', alpha=0.3)
ax2.add_feature(cfeature.COASTLINE, linewidth=0.5)
ax2.add_feature(cfeature.BORDERS, linewidth=0.3)

# Add gridlines
gl2 = ax2.gridlines(draw_labels=True, dms=True, x_inline=False, y_inline=False,
                   linewidth=0.5, color='gray', alpha=0.5)
gl2.top_labels = False
gl2.right_labels = False

# Plot the origin point (transformed for centered view)
ax2.plot(origin_lon, origin_lat, 'go', markersize=12, 
         transform=ccrs.PlateCarree(), label='Origin Port', zorder=5)

# Plot reachable points
if reachable_points:
    scatter2 = ax2.scatter(lons_plot, lats_plot, c=distances, cmap='coolwarm_r', 
                          s=20, alpha=0.7, transform=ccrs.PlateCarree(), zorder=3)
    
    # Add colorbar
    cbar2 = plt.colorbar(scatter2, ax=ax2, shrink=0.8, pad=0.02)
    cbar2.set_label('Distance (Nautical Miles)', rotation=270, labelpad=20)

# Add title
plt.title(f'Reachable Destinations (Pacific-Centered View)\n'
         f'Found {len(reachable_points)} reachable points', fontsize=16, pad=20)

# Add legend
ax2.legend(loc='upper left', bbox_to_anchor=(0, 1))

# Save both figures
plt.figure(fig.number)
plt.savefig('sea_routes_global.png', dpi=300, bbox_inches='tight')
plt.figure(fig2.number)
plt.savefig('sea_routes_pacific_centered.png', dpi=300, bbox_inches='tight')

# Show the plots
plt.show()

print(f"Maps saved with {len(reachable_points)} reachable destinations")
print("Saved two versions:")
print("1. sea_routes_global.png - Standard global view")
print("2. sea_routes_pacific_centered.png - Pacific-centered view for better dateline handling")