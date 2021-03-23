function steering = getSteering(currHead, currLat, currLon, targLat, targLon)
    % currHead must be from -360 to 360;
    x = cos(deg2rad(targLat)) * sin(deg2rad(currLon-targLon));
    y = cos(deg2rad(currLat)) * sin(deg2rad(targLat)) - sin(deg2rad(currLat)) ...
        * cos(deg2rad(targLat)) * cos(deg2rad(currLon-targLon));
    mag = currHead;
    steering = -rad2deg(atan2(x,y)) - mag;
    if steering >= 180
        steering = steering - 360;
    end
    if steering <= -180
        steering = steering + 360;
    end
end

%%
% currentHeading = 270;
% currentLat = 41.74314012013176;
% currentLon = -111.80718399409389;
% targetLat = 41.7429685841948;
% targetLon = -111.80719086063387;