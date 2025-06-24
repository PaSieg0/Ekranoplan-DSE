import searoute as sr
import folium
import numpy as np
from shapely.geometry import Point, LineString
from cartopy.io import shapereader
import time
from tqdm import tqdm
from functools import lru_cache
import math
import webbrowser
import os

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

@lru_cache(maxsize=10000)
def is_on_land(lon, lat):
    """Check if a point is on land (cached for performance)"""
    point = Point(lon, lat)
    return any(geom.contains(point) for geom in land_geom)

# Define the origin port
origin = [153.6, -28]  
max_distance_nm = 2000  # Maximum distance in nautical miles

# Create a lat/lon grid with finer resolution
resolution = 1  # Grid resolution in degrees (smaller = finer mesh)
start_lon = origin[0]
lons = np.arange(start_lon - 90, start_lon + 91, resolution) % 360
lons = np.where(lons > 180, lons - 360, lons)  # Adjust to [-180, 180] range
lats = np.arange(-80, 80, resolution)

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
            if distance <= max_distance_nm:
                reachable_points.append((lat, lon, distance, route))
    except Exception as e:
        continue
    
    # Add a small delay to avoid potential API rate limits
    time.sleep(0.05)  # Reduced delay since we're processing fewer points

# Plot the result with enhanced visualization
m = folium.Map(location=[origin[1], origin[0]], zoom_start=3, tiles='CartoDB positron')

# Add the origin marker
folium.Marker(
    [origin[1], origin[0]], 
    popup="Start Port", 
    icon=folium.Icon(color='green', icon='play')
).add_to(m)

# Create a custom colorscale for distances
def get_color(distance):
    # Color gradient from blue (closer) to red (farther)
    ratio = min(distance / max_distance_nm, 1.0)
    r = min(255, int(255 * ratio))
    b = min(255, int(255 * (1 - ratio)))
    return f'#{r:02x}00{b:02x}'

# Add destination points with color-coded markers
for lat, lon, dist, route in reachable_points:
    # Add the point
    folium.CircleMarker(
        [lat, lon], 
        radius=3,
        color=get_color(dist),
        fill=True,
        fill_opacity=0.7,
        popup=f"Distance: {dist:.0f} NM"
    ).add_to(m)
    
    # Add route lines for select points (to avoid cluttering the map)
    if route and dist % 400 < 50:  # Only show some routes for clarity
        coords = [(y, x) for x, y in route.geometry.coordinates]  # Swap lat/lon for folium
        folium.PolyLine(
            coords, 
            color=get_color(dist), 
            weight=2, 
            opacity=0.7,
            popup=f"Route: {dist:.0f} NM"
        ).add_to(m)

# Add a legend
legend_html = '''
<div style="position: fixed; bottom: 50px; left: 50px; z-index: 1000; background-color: white; 
padding: 10px; border: 2px solid grey; border-radius: 5px;">
<p><strong>Distance (NM)</strong></p>
'''

# Add color gradient to legend
steps = 5
for i in range(steps):
    dist = i * (max_distance_nm / steps)
    color = get_color(dist)
    legend_html += f'<p><span style="color:{color};">●</span> {dist:.0f}</p>'

legend_html += '</div>'
m.get_root().html.add_child(folium.Element(legend_html))

# Save the map
m.save("refined_sea_routes_map.html")
print(f"Map saved with {len(reachable_points)} reachable destinations")
webbrowser.open('file://' + os.path.realpath("refined_sea_routes_map.html"))