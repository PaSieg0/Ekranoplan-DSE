import searoute as sr
import folium
import webbrowser
import os
import numpy as np
import matplotlib.pyplot as plt

all_points = []

# Define origin and destination [longitude, latitude]
origin = [153, -27]     # Netherlands
square_size = 40  # Size of the square in degrees
for x in np.arange(153-square_size/2, 153+square_size/2, 0.2):
    print(x)
    for y in np.arange(-27-square_size/2, -27+square_size/2, 0.2):
        destination = [x, y]  # Indonesia scaled by x and y

        # Calculate sea route
        route = sr.searoute(origin, destination)

        # Print distance in nautical miles
        if route and "properties" in route and "length" in route["properties"]:
            distance_km = route["properties"]["length"]
            # print(f"Route distance: {distance_km/1.852:.2f} nautical miles")
        
        if 1900*1.852 <= distance_km <= 2100*1.852:
            all_points.append(destination)

plt.plot(origin[0], origin[1], 'go')  # Plot origin point
for coord in all_points:
    plt.plot(coord[0], coord[1], 'ro')  # Plot each point on the map
plt.show()

    # # Create folium map centered at midpoint
    # mid_lat = (origin[1] + destination[1]) / 2
    # mid_lon = (origin[0] + destination[0]) / 2
    # m = folium.Map(location=[mid_lat, mid_lon], zoom_start=3)

    # # Add markers for origin and destination
    # folium.Marker(origin[::-1], popup="Origin: Netherlands", icon=folium.Icon(color='green')).add_to(m)
    # folium.Marker(destination[::-1], popup="Destination: Indonesia", icon=folium.Icon(color='red')).add_to(m)

    # # Draw route from GeoJSON coordinates
    # if route and "geometry" in route and "coordinates" in route["geometry"]:
    #     coords = route["geometry"]["coordinates"]  # List of [lon, lat]
    #     folium.PolyLine([(lat, lon) for lon, lat in coords], color="blue", weight=2.5).add_to(m)



# # Save map
# m.save("sea_route.html")

# # Open the map in browser
# webbrowser.open('file://' + os.path.realpath("sea_route.html"))
