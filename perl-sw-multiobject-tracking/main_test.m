%Initialize known poses in J_k
clear;
clc;
x_k(1).mu=[250,250,0,0]';
x_k(2).mu=[-250,-250,0,0]';
%x_k(1).P=[10 0 0 0;0 10 0 0;0 0 0 0;0 0 0 0];
%x_k(2).P=[10 0 0 0;0 10 0 0;0 0 0 0;0 0 0 0];
x_k(1).P=zeros(4);
x_k(2).P=zeros(4);
x_k(1).weight=0.5;
x_k(2).weight=0.5;
J_k=2;
J_beta=5;
J_gamma=2; %Give birth to 10 particles every time step
T=10^-5;
U=4;
J_max=100;
i=0;
t_steps=100; %Number of time steps
mu_gamma=[[250,250,0,0]',[-250,-250,0,0]'];
figure;
hold on
for k=1:t_steps
    i=0;
    % Give Birth to new objects

    for it1=1:2
       for it2=1:length(x_k)
           i=i+1;
           x_pred(i).weight=BirthWeight(x_k(it2).mu);
           x_pred(i).mu=mu_gamma(:,it1);
           x_pred(i).P=diag([100,100,25,25]');
       end
       
    end
    % Spawning
    for it2=1:J_beta
        for it3=1:J_k
            i=i+1;
            x_pred(i)= SpawnMotionModel(x_k(it3));
            x_pred(i).weight=SpawnWeight(x_pred(i).mu,x_k(it3).mu)*x_k(it3).weight;
        end
    end
    
    %Prediction using existing targets
    
    for it4=1:J_k
        i=i+1;
        x_pred(i)=ExistingObjectMotionModel(x_k(it4));
    end
    
    J_kpred=i; %Predicted number of targets
    
    % Construct Update for PHD components
    
    for it5=1:J_kpred
        [PHDupdate(it5),x_pred(it5).P]= UpdatePHDComponents(x_pred(it5));
    end
    
    % Update
    % Missed Detections
    
    for it6=1:J_kpred
        x_k(it6)=ObjectMissedDetection(x_pred(it6));
    end
    % Detected Objects
    l=0; %To iterate all object over all sensor readings
    z=PositionSensor(k);
    for it7=1:length(z)
        l=l+1;
        for it8=1:J_kpred
            x_k((l*J_kpred+it8))= DetectedObjectUpdate(x_pred(it8),z(it7,:),PHDupdate(it8));
        end
    end
    J_k=l*J_kpred+J_kpred; %Update the number of particles
    
    %Normalize the weights
    
    sum_weight=0;
    for i=1:J_k
        sum_weight=sum_weight+x_k(i).weight;
    end
    
    for i=1:J_k
        x_k(i).weight=x_k(i).weight/sum_weight;
    end
    % Pruning and Merging
    
    x_k=PruningAndMerging(x_k,T,U,J_max);
    %J_k=length(x_k);
    
    % Multi Target State Extraction
    
    x_k=MultiTargetStateExtraction(x_k);
    J_k=length(x_k);
    for i=1:length(x_k)
        traj(i).mu(:,k)=x_k(i).mu;
        plot(traj(i).mu(1,1:end),traj(i).mu(2,1:end),'*');
    end
    
    pause(0.05);
end
