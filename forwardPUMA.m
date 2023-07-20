function forwardPUMA(theta1,theta2,theta3,theta4,theta5,theta6)
clc
close all
%input: joint angles at each frames
%ouput: end-effector position

format compact
format short
%DH parameters
J = [theta1 theta2 theta3 theta4 theta5 theta6]; %joint angles
A = [-90 0 90 -90 90 0] ; %twist angle
r = [0 431.80 -20.32 0 0 0]; %offset as to xn
d = [671.83 139.70 0 431.80 0 56.50]; %offset as to z(n-1)
%forward kinematics 0T1,1T2,2T3,3T4,4T5,5T6
if J(1,1) >= -160 && J(1,1) <= 160 && J(1,2)>= -225 ...
        && J(1,2) <= 45 && J(1,3) >= -45 && J(1,3) <= 225 ...
        && J(1,4) >= -110 && J(1,4) <= 170 && J(1,5) >= -100 ...
        && J(1,5) <= 100 && J(1,6) >= -266 && J(1,6) <= 266
    T = [];
    %Homogeneus Transformation
    for n = 1:6
        matT = [cosd(J(n)) -sind(J(n))*cosd(A(n)) ...
            sind(J(n))*sind(A(n)) r(n)*cosd(J(n));
            sind(J(n)) cosd(J(n))*cosd(A(n)) ...
            -cosd(J(n))*sind(A(n)) r(n)*sind(J(n));
            0 sind(A(n)) cosd(A(n)) d(n);
            0 0 0 1];
        T = [T; {matT}];
    end
    P = [];
    %Joint Positions
    for i = 1:6
        if i == 1
            P = [P,{T{i}}];
        else 
            matP = P{i-1}*T{i};
            P = [P, {matP}];
        end
    end
    %plotting the joint positions
    x = [0 P{1}(1,4) P{1}(1,4) P{2}(1,4) P{3}(1,4) P{4}(1,4) P{5}(1,4) P{6}(1,4)];
    y = [0 P{1}(2,4) P{2}(2,4) P{2}(2,4) P{3}(2,4) P{4}(2,4) P{5}(2,4) P{6}(2,4)];
    z = [0 P{1}(3,4) P{1}(3,4) P{2}(3,4) P{3}(3,4) P{4}(3,4) P{5}(3,4) P{6}(3,4)];
    hold on
    grid on
    rotate3d on
    plot3([x(1) x(2)],[y(1) y(2)],[z(1) z(2)],'Color','b','LineWidth',5)
    plot3([x(2) x(3)],[y(2) y(3)],[z(2) z(3)],'Color','r','LineWidth',5)
    plot3([x(3) x(4)],[y(3) y(4)],[z(3) z(4)],'Color','g','LineWidth',5)
    plot3([x(4) x(5)],[y(4) y(5)],[z(4) z(5)],'Color','c','LineWidth',5)
    plot3([x(5) x(6)],[y(5) y(6)],[z(5) z(6)],'Color','m','LineWidth',5)
    plot3([x(6) x(7)],[y(6) y(7)],[z(6) z(7)],'Color','y','LineWidth',5)
    plot3([x(7) x(8)],[y(7) y(8)],[z(7) z(8)],'Color','k','LineWidth',5)
    plot3(x,y,z,'o','Color','k')
    xlabel('x-axis')
    ylabel('y-axis')
    zlabel('z-axis')
    title('Forward Kinematics of PUMA 560 Manipulator (6DoF)')
    view(0,0)

    Jin=J';
    Ain=A';
    rin=r';
    din=d';
    DH_PARAMETERS=table(Jin,Ain,rin,din)
    
    disp('The end-effector position is ')
    fprintf('at Px = %f, Py = %f, and Pz = %f',P{6}(1,4),P{6}(2,4),P{6}(3,4));
    disp('  ')
    disp('The DH model is  ')
    disp(P{6})
    disp('The HOMOGENEUS MATRIX  ')
    disp("0T1")
    T{1} %0T1
    disp("1T2")
    T{2} %1T2
    disp("2T3")
    T{3} %2T3
    disp("3T4")
    T{4} %3T4
    disp("4T5")
    T{5} %4T5
    disp("5T6")
    T{6} %5T6
else
    disp('Joint angle out of range');
end
end