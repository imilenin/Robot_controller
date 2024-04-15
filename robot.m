stepN = 1000;
X = zeros(1,stepN);
Y = zeros(1,stepN);
tolerance = 10;
fi = 0;
AllTime = 100;
stepT = AllTime/stepN;
L = 40;
teta = 0;
X_T = 100;
Y_T = 300;
X_obst = 800;
Y_obst = 100;
size_obst = 50;
X_center_obst =  X_obst + size_obst/2;
Y_center_obst =  Y_obst + size_obst/2;
obstX = [X_obst X_obst+size_obst X_obst+size_obst X_obst X_obst];
obstY = [Y_obst Y_obst Y_obst+size_obst Y_obst+size_obst Y_obst];
VL = 0;
VR = 0;
teta_T = atan((Y_T-Y(1))/(X_T-X(1)));
S2_X = X(1)+0.5*L*cos(deg2rad(60));
S2_Y = Y(1)+0.5*L*sin(deg2rad(60));
S1_X = X(1)+0.5*L*cos(deg2rad(120));
S1_Y = Y(1)+0.5*L*sin(deg2rad(120));
S4_X = X(1)+0.5*L*cos(deg2rad(0));
S4_Y = Y(1)+0.5*L*sin(deg2rad(0));
S3_X = X(1)+0.5*L*cos(deg2rad(180));
S3_Y = Y(1)+0.5*L*sin(deg2rad(180));
tetaS1 = atan((Y_center_obst - S1_X)/(X_center_obst - S1_Y));
tetaS2 = atan((Y_center_obst - S2_X)/(X_center_obst - S2_Y));
tetaS3 = atan((Y_center_obst - S3_X)/(X_center_obst - S3_Y));
tetaS4 = atan((Y_center_obst - S4_X)/(X_center_obst - S4_Y));

if rad2deg(tetaS1) <= 142.2 &&  rad2deg(tetaS1) >= 97.5
    s1 = sqrt((X_center_obst-S1_X)^2+(Y_center_obst-S1_Y)^2);
else
    s1 = 999;
end
if rad2deg(tetaS2) <= 52.5 &&  rad2deg(tetaS2) >= 7.5
    s2 = sqrt((X_center_obst-S2_X)^2+(Y_center_obst-S2_Y)^2);
else
    s2 = 999;
end
if rad2deg(tetaS3) <= 202.5 &&  rad2deg(tetaS3) >= 157.5
    s3 = sqrt((X_center_obst-S3_X)^2+(Y_center_obst-S3_Y)^2);
else
    s3 = 999;
end
if rad2deg(tetaS4) <= 22.5 &&  rad2deg(tetaS4) >= -22.5
    s4 = sqrt((X_center_obst-S4_X)^2+(Y_center_obst-S4_Y)^2);
else
    s4 = 999;
end
step = 2;
input = zeros(1,6);
fis = readfis('Robot3');
for time = 0:stepT:AllTime
    d = sqrt((X_T-X(step-1))^2+(Y_T-Y(step-1))^2);
    fi = teta_T - teta;    
    input(1) = d;
    input(2) = fi;
    input(3) = s1;
    input(4) = s2;
    input(5) = s3;
    input(6) = s4;
    output  = evalfis(fis, input);
    VL = output(1);
    VR = output(2);     
    X(step) = X(step-1) + stepT*(VL+VR)*cos(teta)/2;
    Y(step) = Y(step-1) + stepT*(VL+VR)*sin(teta)/2;
    teta = teta + stepT*(VL-VR)/L;
    Y_T=Y_T+0.05;
    teta_T = atan((Y_T-Y(step))/(X_T-X(step)));
    S2_X = X(step)+0.5*L*cos(deg2rad(60));
    S2_Y = Y(step)+0.5*L*sin(deg2rad(60));
    S1_X = X(step)+0.5*L*cos(deg2rad(120));
    S1_Y = Y(step)+0.5*L*sin(deg2rad(120));
    S4_X = X(step)+0.5*L*cos(deg2rad(0));
    S4_Y = Y(step)+0.5*L*sin(deg2rad(0));
    S3_X = X(step)+0.5*L*cos(deg2rad(180));
    S3_Y = Y(step)+0.5*L*sin(deg2rad(180));
    tetaS1 = atan((Y_center_obst - S1_X)/(X_center_obst - S1_Y))+teta;
    if tetaS1 > 2*pi
        tetaS1 = tetaS1- 2*pi;
    end
    tetaS2 = atan((Y_center_obst - S2_X)/(X_center_obst - S2_Y))+teta;
    if tetaS2 > 2*pi
        tetaS2 = tetaS2- 2*pi;
    end
    tetaS3 = atan((Y_center_obst - S3_X)/(X_center_obst - S3_Y))+teta;
    if tetaS3 > 2*pi
        tetaS3 = tetaS3- 2*pi;
    end
    tetaS4 = atan((Y_center_obst - S4_X)/(X_center_obst - S4_Y))+teta;
    if tetaS4 > 2*pi
        tetaS4 = tetaS4- 2*pi;
    end

    if rad2deg(tetaS1) <= 142.2 &&  rad2deg(tetaS1) >= 97.5
        s1 = sqrt((X_center_obst-S1_X)^2+(Y_center_obst-S1_Y)^2);
    else
        s1 = 999;
    end
    if rad2deg(tetaS2) <= 52.5 &&  rad2deg(tetaS2) >= 7.5
        s2 = sqrt((X_center_obst-S2_X)^2+(Y_center_obst-S2_Y)^2);
    else
        s2 = 999;
    end
    if rad2deg(tetaS3) <= 202.5 &&  rad2deg(tetaS3) >= 157.5
        s3 = sqrt((X_center_obst-S3_X)^2+(Y_center_obst-S3_Y)^2);
    else
        s3 = 999;
    end
    if rad2deg(tetaS4) <= 22.5 &&  rad2deg(tetaS4) >= -22.5
        s4 = sqrt((X_center_obst-S4_X)^2+(Y_center_obst-S4_Y)^2);
    else
        s4 = 999;
    end
    
    step=step+1;
end
plot(X,Y,X_T,Y_T,'b*',obstX,obstY,'r')
axis([0 500 0 500]);
