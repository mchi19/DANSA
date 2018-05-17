import pandas as pd
import gmplot

with open ('tests.TXT','r') as infile:
    latitudes= [i.strip().split(' ')[0] for i in infile]  
    latitudes = list(map(float, latitudes[1:]))
with open ('tests.TXT','r') as infile:
    longitudes= [i.strip().split(' ')[1] for i in infile]
    longitudes = list(map(float, longitudes[1:]))
with open ('tests.TXT','r') as infile:    
    rssi = [i.strip().split(' ')[2] for i in infile]
    rssi = list(map(float, rssi[1:]))
with open ('tests.TXT','r') as infile:    
    bssid = [i.strip().split(' ')[3] for i in infile]
    bssid = list(map(str, bssid[1:]))
      
alteredRSSI = []
for ss in rssi: 
    if(ss < -85):
        quality = 50
    elif (ss <= -75 and ss >= -85):
        quality = 100
    elif (ss <= -69 and ss >= -75):
        quality = 200
    elif (ss <= -40 and ss >= -69):
        quality = 300
    else:
        quality = 400#2 * (ss + 1000);
    alteredRSSI.append(quality)


dicts = {}
for i in range(len(bssid)):
    if bssid[i] not in dicts.keys():
        dicts[bssid[i]] = []
        dicts[bssid[i]].append([latitudes[i], longitudes[i], alteredRSSI[i]])
    else:
        dicts[bssid[i]].append([latitudes[i], longitudes[i], alteredRSSI[i]])
        
print(dicts)

for k,v in dicts.items():
    print(k,v)
    templat = []
    templon = []
    temprssi = []
    for i in v:
        templat.append(i[0])
        templon.append(i[1])
        temprssi.append(i[2])

    gmap = gmplot.GoogleMapPlotter(40.428723, -86.913888, 18)
    gmap.heatmap_weighted(templat , templon, temprssi, radius=25, dissipating=False) #fix radius given formula [i for i in rad]

    k = k.split(":")
    k = "".join(k)
    
    
    gmap.draw("output/[{}]DANSA_WifiHeatMap.html".format(k))
