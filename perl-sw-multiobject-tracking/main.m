% Main program to run the PHD Filter for 100 timesteps
            %   Author: Sidddharth Ratapani Navin
            %   Date:   10/28/2018
%% Load PHD Filter Parameters
clear;
clc;
tracks = load('tracklets.mat');
tracks = struct2cell(tracks);

U=6;
%%%% Change this to 3.
x_k(1).mu=[-4.0690,1.8031,15.1641]';
x_k(2).mu=[1.0542,1.7318,46.4074]';
x_k(1).P= [1 0 0;0 1 0;0 0 1];
x_k(2).P= [1 0 0;0 1 0;0 0 1];
x_k(1).weight=0.33;
x_k(2).weight=0.33;
x_k(3).weight=0.33;
t_steps=41;
%%%%
%figure('pos',[1500 1500 1500 1500]);
%axis([-500 1200 -1200 500])
%hold on
%ground_truth=zeros(3,2,100);
figure;
hold on
%% Main Loop for a 100 timesteps
for k=1:t_steps
    i=0; 
    %% Step 1.
    % Give Birth to new objects
%     for it1=1:2
%        for it2=1:length(x_k)
%            i=i+1;
%            x_pred(i).weight=BirthWeight(x_k(it2).mu);
%            x_pred(i).mu=mu_gamma(:,it1);
%            x_pred(i).P=diag([100,100,25,25]');
%        end
%        
%     end
    x_pred=CarBirth();
    i=25;
    % Spawning
    for it2=1:J_beta
        for it3=1:J_k
            i=i+1;
            x_pred(i)= SpawnMotionModel(x_k(it3));
            x_pred(i).weight=SpawnWeight(x_pred(i).mu,x_k(it3).mu)*x_k(it3).weight;
        end
    end
    
    %% Step 2.
    %Prediction using existing targets
    
    for it4=1:J_k
        i=i+1; 
        x_pred(i)=ExistingObjectMotionModel(x_k(it4));
    end
    
    J_kpred=i; %Predicted number of targets
    
     %% Step 3.
    % Construct Update for PHD components
    
    for it5=1:J_kpred
        [PHDupdate(it5),x_pred(it5).P]= UpdatePHDComponents(x_pred(it5));
    end
    
    %% Step 4.
    % Update
    % Missed Detections
    
    for it6=1:J_kpred
        x_k(it6)=ObjectMissedDetection(x_pred(it6));
    end
  
    % Detected Objects
    
    l=0; %To iterate all object over all sensor readings
    %z=PositionSensor(k);
    %g=GroundTruth(k);
    %ground_truth(1:length(z),:,k)=g;
    z=RCNNSensor(k);
    for it7=1:length(z)
        l=l+1;
        for it8=1:J_kpred
            x_k((l-1)*J_kpred+it8)= DetectedObjectUpdate(x_pred(it8),z(it7,:),PHDupdate(it8));
        end
    end
    J_k=l*J_kpred; %Update the number of particles
    
    %% Normalize the weights
   
    sum_weight=0;
    for i=1:J_k
        sum_weight=sum_weight+x_k(i).weight;
    end
    
    for i=1:J_k
        x_k(i).weight=x_k(i).weight/sum_weight;
    end
    %% Pruning and Merging
    x_k=PruningAndMerging(x_k,T,U,J_max);
    
    %% Multi Target State Extraction
    x_k=MultiTargetStateExtraction(x_k);
    J_k=length(x_k);
    z_traj{k}=z;
    traj{k}=x_k;
    %% Plot Trajectories
    for i=1:length(x_k)
        %plot(traj(i).mu(1,1:end),traj(i).mu(2,1:end),'r*','LineWidth',0.15);
        %hold on 
        %plotcov2d(x_k(i).mu(1),x_k(i).mu(2),x_k(i).P,'b',0);
        %hold on
    end
%     plot(reshape(ground_truth(1,1,1:k),1,k),reshape(ground_truth(1,2,1:k),1,k),'g-','LineWidth',1.1);
%     hold on
%     plot(reshape(ground_truth(2,1,1:k),1,k),reshape(ground_truth(2,2,1:k),1,k),'g-','LineWidth',1.1);
%     if(k>66)
%     hold on
%     plot(reshape(ground_truth(3,1,67:k),1,k-67+1),reshape(ground_truth(3,2,67:k),1,k-67+1),'g-','LineWidth',1.1);
%     end
%     legend('Estimate','Covariance','Ground Truth');
%     pause(0.05);
    images=load("Test/im_list.mat");
    imshow("Test/Images/"+images.im_list(k)+".jpg");
    hold on
    for iter=1:length(x_k)
%         imshow("Test/Images/"+images.im_list(k)+".jpg");
%         hold on
        plotcov2d(x_k(iter).mu(1),x_k(iter).mu(2),10*x_k(iter).P,'b',0);
        hold on
%         bbox=open("Test/BBox/BBoxData.mat");
%         bbox=struct2cell(bbox);
%         box=bbox{k};
        
%         for iter1=1:size(box,1)
%             centres(1)=(box(iter1,1)+box(iter1,3))/2;
%             centres(2)=(box(iter1,2)+box(iter1,4))/2;
%             rectangle('Position', [centres(1),centres(2),box(iter1,3)-box(iter1,1),box(iter1,4)-box(iter1,2)],'EdgeColor','r','LineWidth',2)
%         end
        pause(0.05);
        saveas(gcf,k+".jpg");
    end
    %EOF
end
