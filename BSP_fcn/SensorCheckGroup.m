function [] = SensorCheckGroup(hObject, ~)

[fds, name] = kVIS_getCurrentFds(hObject);

% if kVIS_fdsGetChannel(fds, 'SIDPAC_fdata','u') == -1
%     errordlg('Required SIDPAC data group missing. Abort.')
%     return
% else
%     fdata0 = kVIS_fdsGetGroup(fds, 'SIDPAC_fdata');
% end

% assemble new channel group
fdata(:,1) = kVIS_fdsGetChannel(fds, 'Base','Time');

% fdata(:,2:4) = AccelBody(fds);
fdata(:,5:7) = AccelNED(fds);
fdata(:,8:10) = GyroR(fds);
% fdata(:,11:13) = AccelBody2(fds, fdata);

varNames{1} = 'Time';

varNames{2} = 'Ax body';
varNames{3} = 'Ay body';
varNames{4} = 'Az body';
varNames{5} = 'Ax ned';
varNames{6} = 'Ay ned';
varNames{7} = 'Az ned';
varNames{8} = 'p att';
varNames{9} = 'q att';
varNames{10} = 'r att';
varNames{11} = 'Ax body2';
varNames{12} = 'Ay body2';
varNames{13} = 'Az body2';

varUnits{1} = 'sec';

varUnits{2} = 'm/s2';
varUnits{3} = 'm/s2';
varUnits{4} = 'm/s2';
varUnits{5} = 'm/s2';
varUnits{6} = 'm/s2';
varUnits{7} = 'm/s2';
varUnits{8} = 'deg/s';
varUnits{9} = 'deg/s';
varUnits{10} = 'deg/s';
varUnits{11} = 'm/s2';
varUnits{12} = 'm/s2';
varUnits{13} = 'm/s2';

frames = cell(size(varUnits));
varFrames = cellfun(@(x) '', frames, 'UniformOutput', false);

%% update KSID

fds = kVIS_fdsAddDataGroup(fds, 'Post Flight Analysis', 'Sensor Check', varNames, varUnits, varFrames, fdata);

kVIS_updateDataSet(hObject, fds, name);

end


function accE = AccelBody(fds)

grav = 9.80665;
DTR = pi/180;

% t = kVIS_fdsGetChannel(fds, 'SIDPAC_fdata','Time');

vel(:,1) = kVIS_fdsGetChannel(fds, 'Misc_Channels','Vx_INS3');
vel(:,2) = kVIS_fdsGetChannel(fds, 'Misc_Channels','Vy_INS3');
vel(:,3) = kVIS_fdsGetChannel(fds, 'Misc_Channels','Vz_INS3');

att(:,1) = kVIS_fdsGetChannel(fds, 'Misc_Channels','Phi_INS3')/DTR;
att(:,2) = kVIS_fdsGetChannel(fds, 'Misc_Channels','Theta_INS3')/DTR;
att(:,3) = kVIS_fdsGetChannel(fds, 'Misc_Channels','Psi_INS3')/DTR;

angVel(:,1) = kVIS_fdsGetChannel(fds, 'Misc_Channels','P_INS3')/DTR;
angVel(:,2) = kVIS_fdsGetChannel(fds, 'Misc_Channels','Q_INS3')/DTR;
angVel(:,3) = kVIS_fdsGetChannel(fds, 'Misc_Channels','R_INS3')/DTR;

velbdy = LbeTransform(vel, att);

t = kVIS_fdsGetChannel(fds, 'Misc_Channels','Time');
dt = mean(diff(t));

velRate = diff(velbdy)/dt;
velRate = [velRate; velRate(end,:)];
% velRate = fdfilt(velRate/0.01, 30, 0.01);

accE = velocityRates2accelerometers(velRate, velbdy, att, angVel, grav);

end

function accE = AccelNED(fds)

grav = 9.80665;

t = kVIS_fdsGetChannel(fds, 'Base','Time');
dt = mean(diff(t));

fBase = kVIS_fdsGetGroup(fds, 'Base');

% GPS
lon=fBase(:,25); % E/W
lat=fBase(:,26); % N/S

E_X = kVIS_fdsGetChannel(fds, 'VN IMU','ecef X');
E_Y = kVIS_fdsGetChannel(fds, 'VN IMU','ecef Y');
E_Z = kVIS_fdsGetChannel(fds, 'VN IMU','ecef Z');

E_VX = kVIS_fdsGetChannel(fds, 'VN IMU','ecef Vx');
E_VY = kVIS_fdsGetChannel(fds, 'VN IMU','ecef Vy');
E_VZ = kVIS_fdsGetChannel(fds, 'VN IMU','ecef Vz');

% position ref
cc.origin = [mean(E_X(1:10)) mean(E_Y(1:10)) mean(E_Z(1:10))];
cc.originLLA = [mean(lon(1:10)) mean(lat(1:10))];

% convert to ned
vel = ECEF2NED([E_VX E_VY E_VZ], cc.origin', cc.originLLA, 1);

att(:,1) = kVIS_fdsGetChannel(fds, 'Base','Roll');
att(:,2) = kVIS_fdsGetChannel(fds, 'Base','Pitch');
att(:,3) = kVIS_fdsGetChannel(fds, 'Base','Yaw');

% velRate = deriv(vel, 0.01);



velRate = diff(vel)/dt;
velRate = [velRate; velRate(end,:)];
% velRate = fdfilt(velRate, 30, 0.01) - [0 0 grav];
velRate = velRate - [0 0 grav];

accE = LbeTransform(velRate, att)/grav;

end


function gyroE = GyroR(fds)

DTR = pi/180;

att(:,1) = kVIS_fdsGetChannel(fds, 'Base','Roll');
att(:,2) = kVIS_fdsGetChannel(fds, 'Base','Pitch');
att(:,3) = kVIS_fdsGetChannel(fds, 'Base','Yaw');


t = kVIS_fdsGetChannel(fds, 'Base','Time');
dt = mean(diff(t));


% angVel = deriv(att, 0.01);
% angVel = fdfilt(angVel, 10, 0.01);

quat = E2Q(att(:,1), att(:,2), att(:,3));

% angVel = diff(att);
% angVel = [angVel; angVel(end,:)];
% angVel = fdfilt(angVel/0.01, 30, 0.01);
% gyroE   = eulerRates2angularVelocity(att, angVel);

quatd = diff(quat)/dt;
quatd = [quatd; quatd(end,:)];
% quatd = fdfilt(quatd, 30, 0.01);

% quatd = deriv(quat, 0.01);

gyroE = quaterionRates2angularVelocity(quat, quatd)/DTR;
end

function accE = AccelBody2(fds, fdata)

grav = 9.80665;

% t = kVIS_fdsGetChannel(fds, 'SIDPAC_fdata','Time');

vel(:,1) = kVIS_fdsGetChannel(fds, 'SIDPAC_fdata','u');
vel(:,2) = kVIS_fdsGetChannel(fds, 'SIDPAC_fdata','v');
vel(:,3) = kVIS_fdsGetChannel(fds, 'SIDPAC_fdata','w');

att(:,1) = kVIS_fdsGetChannel(fds, 'SIDPAC_fdata','Roll');
att(:,2) = kVIS_fdsGetChannel(fds, 'SIDPAC_fdata','Pitch');
att(:,3) = kVIS_fdsGetChannel(fds, 'SIDPAC_fdata','Yaw');

angVel(:,1) = fdata(:,8);
angVel(:,2) = fdata(:,9);
angVel(:,3) = fdata(:,10);

% velRate = deriv(vel, 0.01);

velRate = diff(vel);
velRate = [velRate; velRate(end,:)];
% velRate = fdfilt(velRate/0.01, 30, 0.01);

accE = velocityRates2accelerometers(velRate, vel, att, angVel, grav);

end