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
format long
% intrinsic parameters (RGBD camera parameters)
% TUM fr3 
%     [fx  0 cx]
% K = [ 0 fy cy]
%     [ 0  0  1]
K = [726.30139160156, 0, 356.69226074219;	
     0, 726.30139160156, 186.45402526855;
     0, 0, 1];
% k1, k2, and k3 = radial distortion coefficients of the lens
distortion_k = [0, 0, 0]; 
% p1 and p2 = Tangential distortion coefficients of the lens
distortion_p = [0, 0];
 
camParams = cameraParameters('IntrinsicMatrix', K',...
            'RadialDistortion', distortion_k, ...
            'TangentialDistortion', distortion_p);

% depth scale factor
depth_scale = 5000;
        
% Preprocessing parameters
UseBilateralFilter = 0;
d_min = 0.2;
d_max = 6;

% MMF seeking parameters
numTrial = 100;
ConvergeAngle = 1/180*pi;
ConeAngle = 45/180*pi;
ConeAngle_tracking = 10/180*pi;
c = 20;
minNumSample = 100;% optimal;
ratio = 0.1;

% Distribution correlation parameters (EstimateTraslation)
config.Lb = -0.1;
config.Ub = 0.1;
config.SamplingInterval = 0.01;% smaller interval can give more accurate result, larger ones could improve the speed
config.scale = 0.01;
config.init_t = 0;
config.dt = 0.001;
config.max_iter = 100;
config.normalize = 0;
config.distMeasure = 2;% 1 is L1, 2 is L2
config.DisplayDistribution = 0;

% Non-linear optimization configuration
options = optimset('display','off','LargeScale','off','Algorithm','active-set','GradObj','on', 'TolFun',1e-010, 'TolX',1e-010, 'TolCon', 1e-10);
options = optimset(options, 'MaxFunEvals', config.max_iter);

% VO state
InitMfFound = 0;

% surface normal fitting (cell size: 10*10)
cellsize = 10;

% VO variable
R = eye(3);
t = [0 0 0]';

% Dataset Name List
test_data_set_Name = '/sofa_1';
inputBaseDir = '/home/qin/Downloads/Datasets/ETH3D/training';%'path_to_dataset';%
ResultBaseDir = '/home/qin/Downloads/MWO_result';%'path_to_resultSavingDir';

% other parameters
saveResult = 1;
convertVicon = 1;

% groundtruth of the first frame (for converting the result to w.r.t vicon)
q_gt0 = [0.034742456815185 0.89968318219643 -0.43504210647755 -0.01007466917233];
R_gt0 = quat2dcm_Eigen(q_gt0);
t_gt0 = [0.51160247495498 0.90559451596743 1.5097977995436]';

% Debug parameters
do_plot = true;
