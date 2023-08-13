Perceptions node;

Object containing all arrays with updated objects(bouys and markers) info on topic "bouys_and_markers"

Arrays contain all bouys we know/have seen the past 3 seconds.

Whenever we append and update elements in the array, update timestamp corresponding to objects in array. Objects in array are deleted after 3 seconds.

If we detect a new bouy in an array we get from point cloud, loop through our array, and update alle the position estimates with the new one. We update a value in the array when a estimate from point cloud array is less than x distance from a object we already have in our array. If there is a new object from point cloud array that is not close (more than x distace away) to the objects we have in our array, apped it to our array.

(Obs, do not want timestamps in the message published on topic)

Message type is now defined in vortex_msgs as DetectedObject.msg and DetectedObjectArray!
