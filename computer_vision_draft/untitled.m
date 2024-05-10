I = imread("capture1\Image25.png");
I = imrotate(I,5);
figure(1)
I = imcrop(I,[50 260 640-50 480-290-50]);
idisp(I)


p1 = ginput(4)'
% p1 = [100  130  170  200;
%   50  0  0  50]
plot_poly(p1,"wo","fill", "b","alpha","0.2")

mn = min(p1');
mx = max(p1');
p2  = [mn(1) mx(2); mn(1) mn(2); mx(1) mn(2); mx(1) mx(2)]'

plot_poly(p2,"k","fill", "r","alpha","0.2")

H = homography(p1,p2);

figure(2)
homwarp(H,I,"full","extrapval",0);
% axis("equal")
