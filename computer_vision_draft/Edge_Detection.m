% Possible solution for estimating drivable area, safe driving direction
% and determination of if obstacles/walls are far away or close

imds = imageDatastore("capture1\"); %Get images
centerPoint = [120,140]; %Initialization values for centerpoint, arbitrary

%Create struct to store video frames
loops = length(imds.Files);
M(loops) = struct('cdata',[],'colormap',[]);

%Loop through a directory containing images
for i = 1:length(imds.Files)
    tic
    %Read image, correct for rotation of camera and scale down
    image1 = readimage(imds,i);
    image1 = imrotate(image1,5,"crop");
    cropSize = 30;
    image1 = imcrop(image1,[cropSize cropSize 640-2*cropSize 480-2*cropSize]);
    image1 = imresize(image1,1/2);
    image1 = imgaussfilt(image1);
    
    %Convert to grayscale and produce binary edge image
    image1 = im2gray(image1);
   edgeX = edge(image1,"prewitt",0.02,"horizontal");

   %Extract lines
   [Hx,T,R] = hough(edgeX,'RhoResolution',1,'Theta',[-88:0.1:-75 75:0.1:88]);
   P  = houghpeaks(Hx,20);
   lines = houghlines(edgeX,T,R,P,'FillGap',10,'MinLength',50);

    %Plot lines across entire image to get binary mask containing stacked
    %lines
    x = [1 size(image1,2)];
    drivingLimit = zeros(size(image1));
    hold on
    for k = 1:length(lines)
        rho = lines(k).rho;
        theta = lines(k).theta;
        y = (rho-x*cosd(theta))/sind(theta);
        drivingLimit = insertShape(drivingLimit,"line",[x(1) y(1);x(2),y(2)],"Color","white","Opacity",0,"SmoothEdges",false);
    end
    hold off
    drivingLimit = im2bw(drivingLimit);
    
    %Step through each column until a line is found, creating a drivability
    %mask.
    drivableArea = zeros(size(drivingLimit));
    minRow = size(drivingLimit,1);
    for col = 1:size(drivingLimit,2)
        notOnLine = true;
        row = size(drivingLimit,1);
        while(notOnLine)
            drivableArea(row,col) = 1;
            row = row -1;
            if(row == 0)
                break
            end
            if(drivingLimit(row,col) == 1)
                notOnLine = false;
            end
        end
        
        if (minRow > row)
            minRow = row;
        end
    end

    %Weighted average filter for center point prediction
    pastWeight = 0.7;
    centerPoint(1) = centerPoint(1)*pastWeight + minRow*(1-pastWeight);
    centerPoint(2) = centerPoint(2)*pastWeight+ round(mean( find(drivableArea(minRow+1,:)==1)))*(1-pastWeight);
    

   t  = toc;
   fprintf("%f Hz\n",1/t)
   
      %Overlay drivability mask and display centerpoint as red circle
     figure(1)
     drivabilityImage = labeloverlay(image1,drivableArea);
     drivabilityImage = insertShape(drivabilityImage,"filled-circle",[centerPoint(2),centerPoint(1),5],"Color","red");
     drivabilityImage = insertText(drivabilityImage,[0 0],sprintf("Rows from bottom: %i",size(drivabilityImage,2)-minRow));
     imshow(drivabilityImage,"Border","tight","InitialMagnification",400)

     %Get the current frame
     M(i) = getframe;
end

% Create video with 20 fps
v = VideoWriter("videos/hough_test",'MPEG-4');
v.FrameRate = 20;
open(v)
writeVideo(v,M)
close(v)
