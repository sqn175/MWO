%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author:   Yi Zhou                                                          *
% Contact:  yi.zhou@anu.com                                                  *
% License:  Copyright (c) 2016 Yi Zhou, ANU. All rights reserved.            *
%                                                                            *
% Redistribution and use the code, with or without                           *
% modification, are permitted provided that the following conditions         *
% are met:                                                                   *
% * Redistributions of source code must retain the above copyright           *
%   notice, this list of conditions and the following disclaimer.            *
% * Neither the name of ANU nor the names of its contributors may be         *
%   used to endorse or promote products derived from this software without   *
%   specific prior written permission.                                       *
%                                                                            *
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"*
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE  *
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE *
% ARE DISCLAIMED. IN NO EVENT SHALL ANU OR THE CONTRIBUTORS BE LIABLE        *
% FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL *
% SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER *
% CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT         *
% LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY  *
% OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF     *
% SUCH DAMAGE.                                                               *
%*****************************************************************************/
clc;
clear;
close all;
% load parameter and dir
addpath('../');
addpath('../evaluation');
load_param_MWO;

% Outer loop (data)
inputDataDir = [inputBaseDir, test_data_set_Name];
ResultDir = [ResultBaseDir, test_data_set_Name];

SpcDir = [ResultDir,'/SPC/'];
if( exist(ResultDir,'dir') == 0 )
    mkdir(ResultDir);
end

% load input
% read the image (timestamp) name list
depthMapFiles = dir([inputDataDir,'/depth']);
depthMapList = depthMapFiles(3:end); % ignore the first two entries which are . and ..
numFrames = size(depthMapList,1);

% output file
if saveResult == 1
    fileID = fopen([ResultDir,'/result.txt'],'w');
    fprintf(fileID, '# timestamp tx ty tz qx qy qz qw\n');
end

if do_plot
    traj_x = 0; traj_y = 0; traj_z = 0;
    pc_x = 0; pc_y = 0; pc_z = 0; pc_c = 0;
    nv_x = 0; nv_y = 0; nv_z = 0;
    
    figure(1); title('Tracking result');
    axis equal; hold on;
    xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
    
    % plot historical trajectory
    traj_p = plot3(traj_x, traj_y, traj_z, 'g.');
    traj_p.XDataSource = 'traj_x';
    traj_p.YDataSource = 'traj_y';
    traj_p.ZDataSource = 'traj_z';
    
    % Plot current point cloud
    pc_p = scatter3(pc_x, pc_y, pc_z, [], pc_c, 'SizeData', 1.0);
    pc_p.XDataSource = 'pc_x';
    pc_p.YDataSource = 'pc_y';
    pc_p.ZDataSource = 'pc_z';
    pc_p.CDataSource = 'pc_c';
    
    % Plot current norm vectors
    nv_p = scatter3(nv_x, nv_y, nv_z, '.');
    nv_p.XDataSource = 'nv_x';
    nv_p.YDataSource = 'nv_y';
    nv_p.ZDataSource = 'nv_z';
    
    % plot camera
    cam_p = plotCamera('Location', [0, 0, 0], ...
                        'Orientation', eye(3), 'Size', 0.1);
                    
end

% main loop
for i = 1:numFrames   
    % Preprocess, compute the normals
    % load the depthMap
    depthMap = imread([inputDataDir,'/depth/',depthMapList(i).name]);
    depthMap = double(depthMap) / depth_scale;

    % Preprocess
    pc = PreprocessDepthMap(depthMap, camParams, UseBilateralFilter);
    
    % surface normal fitting
    [sn,spc] = GetSurfaceNormalCell(pc,cellsize, camParams);
    %quiver3(xyz(:,1), xyz(:,2), xyz(:,3),norm(:,1), norm(:,2),norm(:,3));

    % we only use points whose depth is between max(0.5,d_min)
    % and 2*median(d) - d_min
    cind = find(spc(3,:) > d_min & spc(3,:) < d_max);
    spc = spc(:, cind);
    sn = sn(:, cind);
    
    % Tracking
    % Motion notation: R is from Manhattan Frame (MF) to camera coordinate
    %                  t is the translation (position) defined in MW frame
    if InitMfFound == 1 % see whether the MF has been found
        % Tracking MF
        [R,IsTracked] = TrackingMF(R,sn,ConvergeAngle,ConeAngle_tracking,c,minNumSample);

        % if lost tracking
        if IsTracked == 0
            disp(num2str(i));
            disp('lost tracking!');
            break;
        end
        
        % compensate the rotation by convert the point cloud to MF frame.
        spc_MW2 = R'*spc;
        
        % translation estimation
%         D_MW_old = D_MW1;
%         [t_r, D_MW1] = EstimateTranslation(D_MW1,spc_MW2,config,options);
%         D_MW_new = D_MW1;
%         t = t - t_r;
    else
        % Initialization (Seek the dominant MF)
        [MF_can,MF,FindMF] = SeekMMF(sn,numTrial,ConvergeAngle,ConeAngle,c,minNumSample);
        if(FindMF == 1)
            R = ClusterMMF(MF_can,ratio);% The ouput is a cell of several MF_nonRd
            if isempty(R) == 0
                InitMfFound = 1;
                disp('Initialization done!');
            end
            R = R{1};

            % compensate the rotation by convert the point cloud to MF frame.
            spc_MW1 = R'*spc;
            D_MW1 = GetMWDistribution(spc_MW1,config.SamplingInterval,config.scale,config.normalize);           
            
            % Initial translation is zero
            t = [0 0 0]';% the translation (position) is defined in MW frame
            
            % for result convertion
            if convertVicon == 1
                R_result0 = R;
                t_result0 = t;
            end
        end
    end
    
    % record pose estimation
    if InitMfFound == 1       
        % make the result consistent with groundtruth, which are the
        % rotation from camera to vicon (R_v_c) and the location of camera w.r.t
        % vicon (t_v_c). Users need to manually copy the groundtruth of the
        % first frame to here, otherwise the system will output the result
        % w.r.t the first frame.
        if convertVicon == 1
            % rotation
            R_wrt_vicon = R_gt0*R_result0*R';
            q_wrt_vicon = dcm2quat_Eigen(R_wrt_vicon);
            %translation
            t_result_delta = t - t_result0;
            t_wrt_vicon = t_gt0 + R_gt0*R_result0*t_result_delta;
        else
            % convert R to quartnion q
            q = dcm2quat_Eigen(R);
        end
        
        % record the estimated pose
        if saveResult == 1
            if convertVicon == 0
                fprintf(fileID,'%s %5.4f %5.4f %5.4f %5.4f %5.4f %5.4f %5.4f\n',num2str(depthMapList(i).name(1:end-4)),t(1),t(2),t(3),q(2),q(3),q(4),q(1));
            else
                fprintf(fileID,'%s %5.4f %5.4f %5.4f %5.4f %5.4f %5.4f %5.4f\n',num2str(depthMapList(i).name(1:end-4)),t_wrt_vicon(1),t_wrt_vicon(2),t_wrt_vicon(3),q_wrt_vicon(2),q_wrt_vicon(3),q_wrt_vicon(4),q_wrt_vicon(1));
            end
        end
    end
    disp(['Processed frame ', num2str(i), ': ', depthMapList(i).name]);
    
    if do_plot
        if mod(i, 20) ~= 0
            delete(cam_p)
        end
        
        % Plot trajectory
        cam_position = t;
        traj_x = [traj_x, cam_position(1)]; 
        traj_y = [traj_y, cam_position(2)];
        traj_z = [traj_z, cam_position(3)];
        
        % Plot point cloud
        spc = R'*spc + t;
        pc_x = spc(1,:); 
        pc_y = spc(2,:); 
        pc_z = spc(3,:);
        pc_c = pc_z;

        nv = R'*sn + t;
        nv = nv *3;
        nv_x = nv(1,:); 
        nv_y = nv(2,:); 
        nv_z = nv(3,:);
                
        refreshdata
        drawnow
        
        % plot camera
        cam_p = plotCamera('Location', t, ...
                            'Orientation', R, 'Size', 0.1);
                        
    end
end

if saveResult == 1
    fclose(fileID);
end

%% evaluate
evaluation;