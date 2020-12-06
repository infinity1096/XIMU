%% data input
% reshape your data into a n*3 matrix, name it as "data"
% each row vector represents a magnetic measurement in XYZ coordinate

load('raw_mag_20201028.mat');
data = raw_mag; clear raw_mag;

%% outlier excluding

n0 = 20;% number of nearest neighbours used to determine outlier
outlier_filter_rate = 0.005;

outlier_rate = zeros(size(data,1),1);

for i = 1 : size(data,1)

    neighbor_idx = knnsearch(data,data(i,:),'K',n0+1);% first vector is itself
    
    neighbors = data(neighbor_idx,:);
    neighbors_pure = neighbors(2:end,:) - neighbors(1,:);

    %neighbors_pure = normalize(neighbors_pure,2);
    outlier_rate(i) = sum(sum(neighbors_pure*neighbors_pure'));
    %outlier_rate(i) = -sum(diag(cov(neighbors_pure)));
end

sorted = sort(outlier_rate);
outlier_th = sorted(floor((1-outlier_filter_rate) * size(sorted,1)));

outliers = data(outlier_rate>=outlier_th,:);
filtered_data = data(outlier_rate<outlier_th,:);

%% compute scale and offset - ellipsoid_fit

data_x = filtered_data(:,1);
data_y = filtered_data(:,2);
data_z = filtered_data(:,3);


[ center, radii, evecs, v, chi2 ] = ellipsoid_fit(filtered_data,'0'); % '0' means to fix the axis on xyz axis. Remove it to fit a arbitrary ellipsoid

x_offset = -center(1);
y_offset =-center(2);
z_offset = -center(3);

x_scale = radii(1);
y_scale = radii(2);
z_scale = radii(3);

A =  eye(3).*radii;
% if fitting a general oval, line below should be used
%A = A.*radii;

A = sign(diag(A))' .* A; % make sure main diagonal positive

undistort_mat = inv(A);
bias = center;

un_distorted_data = (undistort_mat * (filtered_data' - center))';

%% data plotting

figure();
plot3(filtered_data(:,1),filtered_data(:,2),filtered_data(:,3),'.','color','b');
hold on;
plot3(outliers(:,1),outliers(:,2),outliers(:,3),'.','color','r');
title('raw input data - outlier removal');
legend('normal','outlier');
xlabel('X');
ylabel('Y');
xlabel('Z');
axis equal;

figure();
plot(filtered_data(:,1),filtered_data(:,2),'.');
hold on;
plot(filtered_data(:,1),filtered_data(:,3),'.');
plot(filtered_data(:,2),filtered_data(:,3),'.');
title('magnetic sensor before calibration cross-check');
legend(['xy';'xz';'yz']);
xlabel('1st axis uncalibrated');
ylabel('2nd axis uncalibrated');
axis equal

figure();
plot(un_distorted_data(:,1),un_distorted_data(:,2),'.');
hold on;
plot(un_distorted_data(:,1),un_distorted_data(:,3),'.');
plot(un_distorted_data(:,2),un_distorted_data(:,3),'.');
title('magnetic sensor after calibration cross-check');
legend(['xy';'xz';'yz']);
xlabel('1st axis - max-min=2');
ylabel('2nd axis - max-min=2');
axis equal

clear xoffset yoffset zoffset

format longEng

undistort_mat

bias

