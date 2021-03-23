function distance = getDistance(currLat, currLon, targLat, targLon)
    distance = sqrt((currLat-targLat)^2 + (currLon-targLon)^2);
end 

%%
% distanceThreshold = getGPS_accuracy();
% coordinate distance of 0.000028796 is approximately 1 meter
% GPS accuracy is anywhere from 3 meters to 10 meters.
% It may be wise to make the distance threshold not much less than the 
% minimum steering diameter of the rover.