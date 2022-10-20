
clear all
close all
load ([cd '\SimData.mat']);
path=append(cd,'\Parameterestimation\');
OptTendonStrain=[];
Dir1='C:\MyCloud\GitHub\Muscle_Tendon_Passive_Parameter_Estimation\TorqueSimulation\';
Muscname=Data.SimMusclename;
for i=1:1:length(Data.Coordlable)
    u=0;
    stiff_tendon_counter=0;
    OptData=importdata(Data.(Data.Coordlable{i}).ParamSimulPath);
    if contains(Data.Coordlable{i},'Mov')
        RefData=importdata(Data.RefStatepathAnkleMoving);
    else
        RefData=importdata(Data.RefStatepath);
        
    end
        reftime=RefData.data(:,1);
        
%     indx=find(strncmp(OptData.textdata,'num_parameters',14));
%     ParNum=erase(OptData.textdata(indx),'num_parameters=');
%     ParNum=str2num(ParNum{1,1});
%% getting mucsles parameters 
OptPassiveFiber=[];
    for m=1:length(Muscname)
        M_indx=find(contains(OptData.colheaders,Muscname(m))&~contains(OptData.colheaders,'force'));
        for y=1:length(M_indx)
            if contains(OptData.colheaders(M_indx(y)),'tendon_slack')
                OptTSL(i,m)=OptData.data(1,M_indx(y));
            elseif contains(OptData.colheaders(M_indx(y)),'passive_fiber_')
                stiff_tendon_counter=stiff_tendon_counter+1;
                OptPassiveFiber(i,stiff_tendon_counter)=OptData.data(1,M_indx(y));
            elseif contains(OptData.colheaders(M_indx(y)),'tendon_strain')
                u=u+1;
                OptTendonStrain(i,u)=OptData.data(1,M_indx(y));
            end
        end
        
    end
%     Coordlable(i,1)=append("Hip",num2str(Data.Hipangle(i)));
%% Getting refference parameter  
    tendod_slack_ref=Data.(Data.Coordlable{i}).MuscleInfo.TSlack;
    muscle_passive_fiber_at_norm_ref=Data.(Data.Coordlable{i}).MuscleInfo.Passive(~contains(Muscname,Data.ComplianacMusclename));
    tendon_strain_at_norm_ref=Data.(Data.Coordlable{i}).MuscleInfo.Tstrain;
    
%% Calculating parameter error
    tendon_slack_Error(i,:)=(OptTSL(i,:)-tendod_slack_ref)./tendod_slack_ref.*100;
    if OptPassiveFiber
        %     muscle_stiffness=1./(1+OptParam2(i,:));
        muscle_stiffness=2./(OptPassiveFiber(i,:));
        %     muscle_stiffness_ref=1./(1+muscle_passive_fiber_at_norm_ref);
        muscle_stiffness_ref=2./(muscle_passive_fiber_at_norm_ref);
        muscle_stiffness_error(i,:)=(muscle_stiffness-muscle_stiffness_ref)./muscle_stiffness_ref.*100;
    end
    c1=0.2;
    c2=1;
    c3=0.2;
%          tendon_stiffness=1./(1+OptParam3(i,:));
     tendon_stiffness = 1.1*log((1.0 + c3) / c1) ./ (1.0 + OptTendonStrain(i,:) - c2);
%          tendon_stiffness_ref=1./(1+tendon_strain_at_norm_ref);
     tendon_stiffness_ref=1.1*log((1.0 + c3) / c1) ./ (1.0 + tendon_strain_at_norm_ref - c2);
     
    TkErrortendon_stiffness_error(i,:)=(tendon_stiffness-tendon_stiffness_ref)./tendon_stiffness_ref.*100;
    epsilonerror(i,:)=(OptTendonStrain(i,:)-tendon_strain_at_norm_ref)./tendon_strain_at_norm_ref.*100;
 %% Getting ref and FD states       
    Header=OptData.colheaders;
    Ddata=OptData.data;
    time=Ddata(:,1);
    for corindx = 1:length(Data.ActiveCoordinates)
        Angle_idx=find(contains(Header,append(Data.ActiveCoordinates,'/value')));
        Activecoor(:,corindx)=Ddata(:,Angle_idx)*180/3.14; 
        RefAngle_idx=(contains(RefData.colheaders,append(Data.ActiveCoordinates,'/value')));
        RefActivecoor(:,corindx)=RefData.data(:,RefAngle_idx)*180/3.14;
        
    end
%     activation_idx=find(contains(Header,'/forceset/knee_act'));
%     KneeActuator=Ddata(:,activation_idx)*3000;
    % plot(Kneeangle,KneeActuator)
    vq1 = interp1(time,Activecoor,reftime);
    Error=RefActivecoor-vq1;
%     Ankle=
    plot(reftime,Error)
   
    hold on
end

% xlabel ('Knee Angle(Deg)')
% ylabel ('External Torque(N.m)')
% legend(Name,'Location','southeast')
ylabel ('Error(Deg)')
xlabel ('Time (s)')
title('Knee angle')
legend(Data.Coordlable,'Location','southeast')
% end

%% Tendon Slack Length
figure
X = categorical(Muscname);
bar(X,tendon_slack_Error)
ylabel ('Tendon Slack length Estimation Error(%)')
legend(Data.Coordlable,'Location','northeast')

%% Muscle Stiffness
if OptPassiveFiber
figure
X = categorical(Muscname(~contains(Muscname,Data.ComplianacMusclename)));
bar(X,muscle_stiffness_error')
ylabel ('Muscle Stiffness Estimation Error(%)')
legend(Data.Coordlable,'Location','southeast')
% legend(Data.Coordlable,'Location','northeast')
end
%% Tendon stiffness
figure
X2 = categorical(Data.ComplianacMusclename);
bar(X2,TkErrortendon_stiffness_error')
ylabel ('Tendon Stiffness Estimation Error(%)')
% legend(Data.Coordlable,'Location','northeast')
legend(Data.Coordlable,'Location','southeast')


figure
X2 = categorical(Data.ComplianacMusclename);
bar(X2,epsilonerror')
ylabel ('Epsilon Estimation Error(%)')
% legend(Data.Coordlable,'Location','northeast')
legend(Data.Coordlable,'Location','southeast')
