function [ normal ] = flipNormalTowardsViewdirection( normal, view )
% check the angles between view directon and normal.
% If it's larger than 90 then flip it.

tmp = bsxfun(@dot, normal, repmat(reshape(view,3,1),1, size(normal,2)));
pos = find(tmp<0);
normal(:,pos) = bsxfun(@minus, [0,0,0]', normal(:,pos)); 

end

