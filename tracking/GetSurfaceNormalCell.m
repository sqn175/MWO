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

function [SurfaceNormal, SuperPC] = GetSurfaceNormalCell(PointCloud,cellsize,camParams)
% This function aims to estimate the surface normal of each cell of the

% creat the cell
height = camParams.ImageSize(1);
width = camParams.ImageSize(2);

cellNumX = floor(width/cellsize);
cellNumY = floor(height/cellsize);

SurfaceNormal = zeros(3,cellNumX*cellNumY);

SuperPC = zeros(3,cellNumX*cellNumY);% the spc has the same data structure 
                                     % as the sn does, which is good for
                                     % corresponing each other.
                                     
% fit the plane for each cell
numCell = 1;
for y = 1:cellNumY
    for x = 1:cellNumX
        X = PointCloud((y-1)*cellsize+1:(y-1)*cellsize + cellsize,(x-1)*cellsize+1:(x-1)*cellsize + cellsize,1);
        Y = PointCloud((y-1)*cellsize+1:(y-1)*cellsize + cellsize,(x-1)*cellsize+1:(x-1)*cellsize + cellsize,2);
        Z = PointCloud((y-1)*cellsize+1:(y-1)*cellsize + cellsize,(x-1)*cellsize+1:(x-1)*cellsize + cellsize,3);
        X = reshape(X,[cellsize*cellsize 1]);
        Y = reshape(Y,[cellsize*cellsize 1]);
        Z = reshape(Z,[cellsize*cellsize 1]);
        
        % find the good 3-D points for fitting
        Z_median = median(Z);
        index = find(abs((Z - Z_median)/Z_median) < 0.03 );
        Z = Z(index);
        X = X(index);
        Y = Y(index);
        
        % set the median location of each cell, i.e. super-point-cloud
        SuperPC(:,numCell) = [median(X);median(Y);median(Z)];
        
        % if not enough good points, do not do fitting
        if numel(index) < 0.5*cellsize^2
            SurfaceNormal(:,numCell) = [NaN;NaN;NaN];
            % if not enough point in this cell, the 
            % super point cloud will be assigned as NaN
            SuperPC(:,numCell) = [NaN;NaN;NaN];
            numCell = numCell + 1;
            continue;
        end
        
        % do the fitting 
        SurfaceNormal(:,numCell) = fitPlane(X,Y,Z);
        numCell = numCell + 1;
    end
end

% !!Attention, the surfacenormal and the spc here are indexed
% corresponding, which is for the further clean of spc for translation estimation.
% clean the NaN in the SurfaceNormal
cind = find(~isnan(SurfaceNormal(1,:)));
SurfaceNormal = SurfaceNormal(:,cind);
% just keep the corresponding spc
SuperPC = SuperPC(:,cind);
end

% plane fitting
function nv = fitPlane(X,Y,Z)
    % fit the plane
    A = [X,Y,Z];
    b = -ones(numel(X),1);
    nv = A\b;
    nv = nv / norm(nv); 
end
