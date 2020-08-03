
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.basemap import Basemap
import geopy.distance # uses ellipsoid model of earth for better accuracy
# https://stackoverflow.com/questions/19412462/getting-distance-between-two-points-based-on-latitude-longitude/43211266#43211266
import parse

# fig, positions, edges = parse.scatter_ubx_file("data/day1/nov_5_location1_noDGNSS.ubx")

fig = plt.gcf()
fig.set_size_inches(8, 8)
zoom = 36

labels = ['TestFixed.ubx']
colors = ['b'] # red: NO RTK, Blue: float or no LOS, green: RTK float or fixed


for i in range(len(labels)):
    positions, edges = parse.process_ubx_file("{}".format(labels[i]))
    positions = parse.positions_to_normalized_inches(positions)
    plt.scatter(positions[:,1],positions[:,0], label=labels[i],color=colors[i])
    plt.title("Location Drift over 2 Minutes [Inches]\n Red -> NO RTK, Blue -> RTK Float, Green -> RTK Fixed")



# positions, edges = parse.process_ubx_file("data/day1/nov_5_location1_noDGNSS.ubx")
# positions = parse.positions_to_normalized_inches(positions)
# plt.scatter(positions[:,1],positions[:,0],
#             label="day1/nov_5_location1_noDGNSS.ubx")
#
# positions, edges = parse.process_ubx_file("data/day1/nov_5_location1.ubx")
# positions = parse.positions_to_normalized_inches(positions)
# plt.scatter(positions[:,1],positions[:,0],color='r',
#             label="day1/nov_5_location1.ubx")

plt.xlim((-zoom, zoom))
plt.ylim((-zoom, zoom))
plt.grid()
plt.legend()
# fig = plt.gca()
plt.savefig("scatter1.png", dpi=220)

plt.show()
