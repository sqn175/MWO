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

% read GT
GT_data = importdata([inputBaseDir, test_data_set_Name, '/groundtruth.txt']);
GT = GT_data.data;
result_data = importdata([ResultBaseDir,test_data_set_Name,'/result.txt']);
result = result_data.data;

numFrame = min(size(GT,1),size(result,1)); % (GT always starts after than result)
error = zeros(numFrame,1);

% rotation
roll_gt = zeros(numFrame,1);
pitch_gt = zeros(numFrame,1);
yaw_gt = zeros(numFrame,1);
time_gt = zeros(numFrame,1);

roll_result = zeros(numFrame,1);
pitch_result = zeros(numFrame,1);
yaw_result = zeros(numFrame,1);
time_result = zeros(numFrame,1);

base_time = min(GT(1,1), result(1,1));
for i = 1:size(GT,1)
    q_gt = [GT(i,8),GT(i,5:7)];
    time_gt(i) = GT(i,1) - base_time;

    R_gt = quat2dcm_Eigen(q_gt);

    [yaw_gt(i),pitch_gt(i),roll_gt(i)] = dcm2angle(R_gt);

    yaw_gt(i) = normAngle(yaw_gt(i));
end
for i = 1:size(result,1)
    q_result = [result(i,8),result(i,5:7)];
    time_result(i) = result(i,1) - base_time;
    
    R_result = quat2dcm_Eigen(q_result);
    
    [yaw_result(i),pitch_result(i),roll_result(i)] = dcm2angle(R_result);

    yaw_result(i) = normAngle(yaw_result(i));

end

%% absolute error analysis
% curve_x = 1:1:numFrame-3;
% p = polyfit(curve_x',error(1:end-3),6);
% curve_y = polyval(p,curve_x);
% figure;
% plot(1:1:numFrame-3,error(1:end-3),'r*');
% hold on;
% % plot(1:1:numFrame,error_dvo,'-b')
% curve_y(curve_y < 0) = 0;
% plot(curve_x,curve_y,'g-','LineWidth',5);
% grid on;
% title('Rotation Matrix Difference (result)');
% legend('RMD','Fitting Curve','Location','northwest');
% xlabel('time (sec)','FontSize',12,'FontWeight','bold');
% ylabel('deg','FontSize',12,'FontWeight','bold');
% set(gca,'FontSize',12,'FontWeight','bold');

%% Rotation
% roll
figure;
plot(time_gt,roll_gt*57.3,'-g','LineWidth',3);
hold on;
plot(time_result,roll_result*57.3,'-r','LineWidth',3);
grid on;
title('Roll');
legend('GT','result');
xlabel('time (sec)','FontSize',12,'FontWeight','bold');
ylabel('deg','FontSize',12,'FontWeight','bold');
set(gca,'FontSize',12,'FontWeight','bold');

% pitch
figure;
plot(time_gt,pitch_gt*57.3,'-g','LineWidth',3);
hold on;
plot(time_result,pitch_result*57.3,'-r','LineWidth',3);
grid on;
title('Pitch');
legend('GT','result');
xlabel('time (sec)','FontSize',12,'FontWeight','bold');
ylabel('deg','FontSize',12,'FontWeight','bold');
set(gca,'FontSize',12,'FontWeight','bold');

% yaw
figure;
plot(time_gt,yaw_gt*57.3,'-g','LineWidth',3);
hold on;
plot(time_result,yaw_result*57.3,'-r','LineWidth',3);
grid on;
title('Yaw');
legend('GT','result');
xlabel('time (sec)','FontSize',12,'FontWeight','bold');
ylabel('deg','FontSize',12,'FontWeight','bold');
set(gca,'FontSize',12,'FontWeight','bold');
