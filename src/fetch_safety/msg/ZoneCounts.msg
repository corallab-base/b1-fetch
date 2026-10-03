# Points per safety zone in one camera cloud, counted on the camera's own
# board by zone_checker_node (so only this, not the cloud, crosses the
# network). zones[] holds the zone names in config order; safety_filter_node
# ignores a message whose names don't match its own zones.
Header header      # stamp: the cloud's stamp; frame_id: the base frame
string[] zones
int32[] counts
