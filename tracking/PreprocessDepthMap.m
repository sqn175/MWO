function pointCloud = PreprocessDepthMap(depthMap, camParams, useBilateralFilter)
% undistort
depthMap = undistortImage(depthMap, camParams);

if useBilateralFilter
    depthMap = imbilatfilt(depthMap);
end

% recover the point cloud
height = camParams.ImageSize(1);
width = camParams.ImageSize(2);
pointCloud = zeros(height,width,3);

K = camParams.IntrinsicMatrix';
fx = K(1,1);
fy = K(2,2);
invfx = 1.0/fx;
invfy = 1.0/fy;
cx = K(1,3);%*ImageDownsamplingRate;
cy = K(2,3);%*ImageDownsamplingRate;

% recover 3d coordinate
[U,V]=meshgrid((1:width)-cx,(1:height)-cy);
pointCloud(:,:,3) = depthMap;
pointCloud(:,:,1) = pointCloud(:,:,3).*U*invfx;
pointCloud(:,:,2) = pointCloud(:,:,3).*V*invfy;

end