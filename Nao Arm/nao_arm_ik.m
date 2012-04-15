function theta = nao_arm_ik(R,p,a2,d4)

theta = [0,0,0,0]';

% arg1 = (-R(1,3))/(sqrt(1 - (R(3,3)^2)));
% if (arg1 > 1)||(arg1 < -1)||(isnan(arg1))
%     disp('Arg 1 is invalid')
%     return
% end
% arg2 = (p(3)-R(3,3)*d4)/(a2);
% if (arg2 > 1)||(arg2 < -1)||(isnan(arg2))
%     disp('Arg 2 is invalid')
%     return
% end
arg23 = R(3,3);
if (arg23 > 1)||(arg23 < -1)||(isnan(arg23))
    disp('Arg 23 is invalid')
    return
end
% arg4 = (R(3,1))/(sqrt(1 - (R(3,3))^2));
% if (arg4 > 1)||(arg4 < -1)||(isnan(arg4))
%     disp('Arg 4 is invalid')
%     return
% end



theta(1) = atan2(-R(2,3),-R(1,3));

c2 = (p(2)-R(2,3)*d4)/(a2*sin(theta(1)));
s2 = (p(3)-R(3,3)*d4)/(a2);

theta(2) = atan2(s2,c2);
th2_plus_th3 = acos(arg23);
theta(3) = th2_plus_th3 - theta(2);
theta(4) = atan2(-R(3,2),R(3,1));

end