function xyzrxyz = Pose_2_Fanuc(h)

rpy = [0; 0; 0];

if (h(3,1)) > 1 - 1e-6
	rpy(2) = -pi/2;
	rpy(1) = 0;
	rpy(3) = atan2(-h(2,3),h(2,2));
elseif (h(3,1))  < -1 + 1e-6
	rpy(2) = pi/2;
	rpy(1) = 0;
	rpy(3) = atan2(h(2,3),h(2,2));
else
	rpy(2) = atan2(-h(3,1),sqrt(h(1,1)*h(1,1)+h(2,1)*h(2,1)));
	rpy(3) = atan2(h(2,1),h(1,1));
	rpy(1) = atan2(h(3,2),h(3,3));
end

rpy = rpy * 180/pi;

xyzrxyz = [h(1:3,4); rpy(1); rpy(2); rpy(3)];