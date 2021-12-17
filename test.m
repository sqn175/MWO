figure; hold on; axis equal;
    xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
    R = [1     0     0;
         0     0    -1;
         0     1     0];

t = [0,0,0];
cam_p = plotCamera('Location', t, ...
                    'Orientation', R', 'Size', 0.1);
ori_p = plotTransforms([0,0,0],rotm2quat(R), 'FrameSize',3);
