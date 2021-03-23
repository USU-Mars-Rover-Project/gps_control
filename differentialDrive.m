function L_R_velocity = differentialDrive(steeringAngle, driveSpeed)
    % steeringAngle must be from -180 to 180.
    % maximum drive speed is 255.
    if steeringAngle < 0 
        %R = driveSpeed + mapfun(abs(steeringAngle), 0,180, 0,100);
        R = driveSpeed + interp1([0,180],[0,100],abs(steeringAngle));
    else
        R = driveSpeed;
    end
    if steeringAngle > 0
        L = driveSpeed + interp1([0,180],[0,100],abs(steeringAngle));
    else
        L = driveSpeed;
    end
    L_R_velocity = [L, R];
end

%%
% driveSpeed = 155;