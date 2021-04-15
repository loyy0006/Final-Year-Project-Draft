%%Calculation of wrench axis

clear;
clf;
clc;
close all;

load('04-14-21_17_32_00.Occlusal.Linear.loadcell.mat')
Wrench = loadcell_record(2000:3000,2:7);

F = Wrench(:,1:3);
F_r = norm(F);%finding max of resultant forces

model = stlread('dental_model.stl');
trimesh(model);
hold on;

L = length(-30:1:30);
Ly = length(Wrench);
Ly = Ly + 1;
r = zeros(3,L); %initialising the wrench axis data size 


k = -50; %k can be any values to visualise wrench axis 
j = 100; %sampling of data 
i = 1; 
p = 1; %to reference to wrench axis x
n = 2; %to reference to wrench axis y
m = 3; %to reference to wrench axis z


%calculate wrench axis with arbitrary k
while j < Ly
    while k < L
        F = Wrench(j,1:3);
        F = F';
        T = Wrench(j,4:6);
        T = T';
        X = cross(F,T);
        Xcross = (X/norm(F)^2); %l_o
        r(p:m,i) = Xcross + (k*F); %l
        i = i + 1;
        k = k + 1;
    end
    %plot wrench axis
    x1 = r(p,:);
    y1 = r(n,:);
    z1 = r(m,:);
    plot3(x1,y1,z1);%drawnow;
    hold on
    
    i = 1;
    p = p+3;
    n = n+3;
    m = m+3;
    j = j + 10;
    k = -50;
end

grid on 
title('Graph of wrench axis on X, Y, Z')
xlabel('X (mm)')
ylabel('Y(mm)')
zlabel('Z(mm)')
axis equal

%import model into figure 
% model = createpde(3);
% importGeometry(model,'dental_model1.stl');
% pdegplot(model)
% axis([-100  100 -100 100 0 20]) %can change to suit our model

