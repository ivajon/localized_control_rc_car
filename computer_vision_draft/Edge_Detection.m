image1 = imread("sample_images\20240404_111345.jpg");
image1 = imresize(image1,0.2);


tic
image1 = rgb2hsv(image1);
h = image1(:,:,1)';
s = image1(:,:,2)';
v = image1(:,:,3)';

method = "canny";
edge_h = edge(h,method);
edge_s = edge(s,method);
edge_v = edge(v,method,0.1);



BW = edge_v;
[H,T,R] = hough(BW,'Theta',-89:0.5:89);

t  = toc
1/t

figure(1)
imshow([h s v])


figure(2)
imshow([edge_h edge_s edge_v])

figure(3)
imshow(imadjust(rescale(H)),'XData',T,'YData',R,...
   'InitialMagnification','fit');
xlabel('\theta')
ylabel('\rho');
axis on, axis normal;
colormap(gca,hot)