from mpl_toolkits.basemap import Basemap
import matplotlib.pyplot as plt

fig = plt.gcf()
# fig.set_size_inches(10, 15)
# Google map
map = Basemap(  llcrnrlon= -80.08240,
                llcrnrlat=  41.15380,
                urcrnrlon= -80.076810,
                urcrnrlat=  41.157210,
                epsg=2271)
map.arcgisimage(service='World_Imagery', verbose= True)

# Scatter of gps data in lat lon
# plt.title("Latitude vs. Longitude")
plt.show()
fig.savefig("Map.png", bbox_inches='tight', pad_inches = -.1, dpi=220)
