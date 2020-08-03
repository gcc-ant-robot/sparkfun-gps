import numpy as np
import matplotlib.pyplot as plt
import pyproj
from mpl_toolkits.basemap import Basemap
import geopy.distance # uses ellipsoid model of earth for better accuracy
# https://stackoverflow.com/questions/19412462/getting-distance-between-two-points-based-on-latitude-longitude/43211266#43211266

def process_ubx_file(fn):
    ubx_file = open(fn,'rb')     # open file with read permissions

    gngll_data_lines = []
    positions = []

    line_iter = 0 # for logging purposes

    for sentence in ubx_file:
        line_iter += 1
        try:
            decoded = sentence.decode('utf-8')
        except UnicodeDecodeError:
            # print("Line {} contains garbage info".format(line_iter))# garbage line
            continue
        else:
            if "$GNGLL" in decoded:
                # This is a line we want to keep
                gngll_data_lines.append(sentence.decode('utf-8'))

    # At this point, we have a list of strings, each string following this format:
    # $GNGLL,4109.30479,N,08004.73378,W,184042.00,A,D*6A
    # Convert to decimal lat lon
    for sentence in gngll_data_lines:
        words = sentence.split(',') # ['$GNGLL', '4109.30479', 'N', '08004.73378', 'W', '184042.00', 'A', 'D*6A\n']
        # First get degree values
        lat_field = float(words[1])
        lon_field = float(words[3])
        latitude = int(lat_field/100)
        longitude = int(lon_field/100)
        # Then get minute values
        lat_minutes = lat_field - latitude*100
        latitude += lat_minutes/60
        # then divide minutes by 60 to get degrees and add this to our degree value.
        lon_minutes = lon_field - longitude*100
        longitude += lon_minutes/60
        # finally, if the norghing is south, -> flip sign
        if words[2] == 'S':
            latitude = -latitude
        if words[4] == 'W':
            longitude = -longitude

        # emprically determined shifts to line up with map
        # latitude = latitude - 8e-6 # shift south
        # longitude = longitude + 12e-6 # shift east
        positions.append([latitude, longitude])

    # TODO: make sure that you round after math to the numebr of decimal places which is appropriate to our gps reciever.
    positions = np.asarray(positions) # data to numpy array

    # EDGES OF Sample
    edges = {}
    edges["left"] = min(positions[:,1])
    edges["right"] = max(positions[:,1])
    edges["top"] = max(positions[:,0])
    edges["bottom"] = min(positions[:,0])

    return positions, edges

def plot_ubx_file(fn, map_range=0.0007, out_fn='output.png'):
    point_size = 2

    #fn = "3D_DGNSS_NO_FLOAT_OR_FIX.ubx"
    positions, edges = process_ubx_file(fn) # process a single ubx file

    fig = plt.gcf()
    fig.set_size_inches(10, 15)
    # Google map
    map = Basemap(llcrnrlon=edges["left"]-map_range,
                  llcrnrlat=edges["bottom"]-map_range,
                  urcrnrlon=edges["right"]+map_range,
                  urcrnrlat=edges["top"]+map_range, epsg=2271)
    map.arcgisimage(service='World_Imagery', verbose= True)

    # Scatter of gps data in lat lon
    x, y = map(positions[:,1], positions[:,0]) # transform data into map coords.
    plt.scatter(x,y,edgecolors='r',s=point_size, )
    # plt.title("Latitude vs. Longitude")
    plt.show()
    fig.savefig(out_fn, dpi=220)

def scatter_ubx_file(fn="myubx.ubx"):
    positions, edges = process_ubx_file(fn) # process a single ubx file
    print(positions.shape)
    fig = plt.gcf()
    # fig.set_size_inches(10, 15)
    # Google map

    # Scatter of gps data in lat lon
    plt.scatter(positions[:,1], positions[:,0],color='r' )
    # plt.show()
    return fig, positions, edges

def positions_to_normalized_inches(positions):
    # converts
    mean_lon = np.mean(positions[:,1])
    mean_lat = np.mean(positions[:,0])

    distance_from_mean_inches = np.ones(positions.shape, dtype=np.longdouble) # same shape as positions
    distance_from_mean_inches[:,1] = positions[:,1] - mean_lon
    distance_from_mean_inches[:,0] = positions[:,0] - mean_lat
    # convert degrees to inches
    distance_from_mean_inches = distance_from_mean_inches * 2*np.pi * 250829568 / 360

    return distance_from_mean_inches


def positions_LLA_to_ECEF_point(positions):

    ecef = pyproj.Proj(proj='geocent')#, ellps='WGS84', datum='WGS84')
    lla = pyproj.Proj(proj='latlong')#, ellps='WGS84', datum='WGS84')

    # no offset given,
    mean_lon = np.mean(positions[:,1])
    mean_lat = np.mean(positions[:,0])

    # convert degrees to inches
    # distance_from_mean_inches = means * 2*np.pi * 250829568 / 360

    ecef_points = np.asarray(pyproj.transform(lla, ecef, mean_lon, mean_lat, 0, radians=False)) # lat, lon, alt
    # print("Some Points: {}".format(ecef_points[i]))

    return ecef_points

def distance_between_ubx_inches(positions1, positions2):

    # no offset given,
    mean_lon1 = np.mean(positions1[:,1])
    mean_lat1 = np.mean(positions1[:,0])

    mean_lon2 = np.mean(positions2[:, 1])
    mean_lat2 = np.mean(positions2[:, 0])

    # radius in inches: 250747136.64 in
    # 4376363.124316111
    # convert degrees to inches

    n = 5129366
    in_north = (mean_lat2 - mean_lat1) * n
    in_east = (mean_lon2 - mean_lon1) * n

    return np.asarray(in_north, in_east)