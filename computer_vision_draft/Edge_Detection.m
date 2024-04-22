

imds = imageDatastore("capture2\"); %Get images

for i = 1:length(imds.Files)
    tic
    
    image1 = readimage(imds,i);
    size(image1)
    image1 = imrotate(image1,5,"crop");
    cropSize = 30;
    image1 = imcrop(image1,[cropSize cropSize 640-2*cropSize 480-2*cropSize]);
    image1 = imresize(image1,1/4);
    image1 = imgaussfilt(image1);
    
% 
    
    image1 = im2gray(image1);

   edgeY = edge(image1,"prewitt",0.1,"vertical");
   edgeX = edge(image1,"prewitt",0.025,"horizontal");
   % edgeX = edge(image1,"canny",0.2,0.5);
   % SE = strel("square",5);
   % edgeX = imdilate(edgeX,SE);

   % edgeX = imerode(edgeX,SE);

   [Hx,T,R] = hough(edgeX,'RhoResolution',1,'Theta',[-90:0.1:-80 80:0.1:89]);
   P  = houghpeaks(Hx,10);
   lines = houghlines(edgeX,T,R,P,'FillGap',10,'MinLength',30);

      figure(5)
    x = [1 size(image1,2)];
    test = zeros(size(image1));
    hold on
    for k = 1:length(lines)
        rho = lines(k).rho;
        theta = lines(k).theta;
        y = (rho-x*cosd(theta))/sind(theta);
        % plot(x,y,"LineWidth",1,"Color","g")
        test = insertShape(test,"line",[x(1) y(1);x(2),y(2)],"Color","white","Opacity",0,"SmoothEdges",false);
        
    end
    hold off
    test = im2bw(test);
    imshow(test)


   t  = toc;
   fprintf("%f Hz\n",1/t)

   

    figure(1)
    lY = labeloverlay(image1,edgeY,"Colormap",[1 0 0],"Transparency",0);
    imshow(lY)

    figure(2)
    lX = labeloverlay(image1,edgeX,"Colormap",[1 0 0],"Transparency",0);
    imshow(lX)

    figure(3)
    imshow(imadjust(rescale(Hx)),'XData',T,'YData',R,...
       'InitialMagnification','fit');
    title('Limited Theta Range Hough Transform of Gantrycrane Image');
    xlabel('\theta')
    ylabel('\rho');
    axis on, axis normal;
    colormap(gca,hot)

    hold on
    x = T(P(:,2)); y = R(P(:,1));
plot(x,y,'s','color','white');

    hold off

    figure(4), imshow(image1),
    hold on
    max_len = 0;
    for k = 1:length(lines)
       xy = [lines(k).point1; lines(k).point2];
       plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');

       % Plot beginnings and ends of lines
       plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
       plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');

       % Determine the endpoints of the longest line segment
       len = norm(lines(k).point1 - lines(k).point2);
       if ( len > max_len)
          max_len = len;
          xy_long = xy;
       end
    end
    hold off

 
end
