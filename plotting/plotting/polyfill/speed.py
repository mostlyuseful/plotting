from plotting.polyfill import _polyfill
paths = [
    [(59.1836, 99.1797), (5.73438, 7.13281), (112.176, 6.87109), (59.1836, 99.1797)],
    [(43.2422, 62.5703), (74.668, 62.5703), (74.668, 56.5703), (43.2422, 56.5703), (43.2422, 62.5703)],
    [(80.8789, 50.2539), (59.2227, 14.1797), (38.8125, 49.5352), (80.8789, 50.2539)],
]
out = _polyfill.raster_merge_polygon_eo(paths, 1,20)
