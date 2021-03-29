% https://www.mathworks.com/matlabcentral/answers/157029-getting-coordinates-from-a-picture

clc;    
close all;  % Close all figures (except those of imtool.)
imtool close all;  % Close all imtool figures if you have the Image Processing Toolbox.
clear;  

format long g;
format compact;
fontSize = 12;

I = imread('gum.jpg');
% Get the dimensions of the image.  numberOfColorBands should be = 3.
[rows, columns, numberOfColorBands] = size(I);
% Display the original color image.
subplot(2, 2, 1);
imshow(I);
axis on;
title('Original Color Image', 'FontSize', fontSize);
% Enlarge figure to full screen.
set(gcf, 'Units', 'Normalized', 'Outerposition', [0, 0, 1, 1]);

% Binarize
I=rgb2gray(I); %Convert to gray scale 
BW = imbinarize(I);
BW(:,1) = BW(:,2);
% Display the binary image.
subplot(2, 2, 2);
imshow(BW);
axis on;
title('Binary Image', 'FontSize', fontSize);

% Scan across columns finding where the top of the hump is
for col = 1 : columns
  yy = find(BW(:, col), 1, 'first');
  
  if isempty(yy)
    y(col) = 0;
  else
    y(col) = yy;
  end
end

subplot(2, 2, 3);
hold on;
plot(1 : columns, y, 'b-', 'LineWidth', 1);
set(gca, 'YDir','reverse') %reverse the y-axis
title('Coordinates of gum', 'FontSize', fontSize);

y_new = y +  7.5590551181 %since 2 millimeter = 7.5590551181 pixel 
plot(1 : columns, y_new, 'r-', 'LineWidth', 1);
set(gca, 'YDir','reverse') %reverse the y-axis
hold on;

% average curve
y_avg = (y_new - y)/2 + y;
plot(1 : columns, y_avg, 'c--', 'LineWidth', 1);
set(gca, 'YDir','reverse') %reverse the y-axis
grid on;
ylabel('Y', 'FontSize', fontSize);
xlabel('X', 'FontSize', fontSize);
legend({'original gumline','2mm below gumline','avg gumline'},'Location','southwest')
hold off;

% Save into csv file to export
data = [1:columns; y; y_new; y_avg]; %Concatenating Matrices
writematrix(data,'coordinates.csv')



