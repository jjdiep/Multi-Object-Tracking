function flow = ComputeOpticalFlow(l)
i=0;
opticFlow = opticalFlowLK('NoiseThreshold',0.009);
    % Estimate and display the optical flow of objects in the video.
    for i=1:length(l.im_list)
        frameRGB = imread("Test/Images/"+l.im_list(i)+".jpg");
        frameGray = rgb2gray(frameRGB);
        flow(i) = estimateFlow(opticFlow,frameGray);
        i=i+1;
    end
end