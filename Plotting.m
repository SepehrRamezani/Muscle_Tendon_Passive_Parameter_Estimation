
clear all
close all
load ([cd '\SimData.mat']);
path=append(cd,'\Parameterestimation\');
OptTendonStrain=[];
Dir1='C:\MyCloud\GitHub\Muscle_Tendon_Passive_Parameter_Estimation\TorqueSimulation\';

Muscname=Data.SimMusclename;
for i=1:3:length(Data.Hiplable)
    u=0;
    stiff_tendon_counter=0;
    OptData=importdata(Data.(Data.Hiplable{i}).ParamSimulPath);
    if contains(Data.Hiplable{i},'Moving')
        RefData=importdata(Data.RefStatepathAnkleMoving);
    else
        RefData=importdata(Data.RefStatepath);
        
    end
        reftime=RefData.data(:,1);
        Kneeref=RefData.data(:,(contains(RefData.colheaders,'knee_angle_r/value')))*180/3.14;
        Ankleref=RefData.data(:,(contains(RefData.colheaders,'ankle_angle_r/value')))*180/3.14;
%     indx=find(strncmp(OptData.textdata,'num_parameters',14));
%     ParNum=erase(OptData.textdata(indx),'num_parameters=');
%     ParNum=str2num(ParNum{1,1});
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
%     Hiplable(i,1)=append("Hip",num2str(Data.Hipangle(i)));
    tendod_slack_ref=Data.(Data.Hiplable{i}).MuscleInfo.TSlack;
    muscle_passive_fiber_at_norm_ref=Data.(Data.Hiplable{i}).MuscleInfo.Passive(~contains(Muscname,Data.ComplianacMusclename));
    tendon_strain_at_norm_ref=Data.(Data.Hiplable{i}).MuscleInfo.Tstrain;
    tendon_slack_Error(i,:)=(OptTSL(i,:)-tendod_slack_ref)./tendod_slack_ref.*100;
    %     muscle_stiffness=1./(1+OptParam2(i,:));
    muscle_stiffness=2./(OptPassiveFiber(i,:));
    %     muscle_stiffness_ref=1./(1+muscle_passive_fiber_at_norm_ref);
    muscle_stiffness_ref=2./(muscle_passive_fiber_at_norm_ref);
    muscle_stiffness_error(i,:)=(muscle_stiffness-muscle_stiffness_ref)./muscle_stiffness_ref.*100;
    c1=0.2;
    c2=1;
    c3=0.2;
%          tendon_stiffness=1./(1+OptParam3(i,:));
     tendon_stiffness = 1.1*log((1.0 + c3) / c1) ./ (1.0 + OptTendonStrain(i,:) - c2);
%          tendon_stiffness_ref=1./(1+tendon_strain_at_norm_ref);
     tendon_stiffness_ref=1.1*log((1.0 + c3) / c1) ./ (1.0 + tendon_strain_at_norm_ref - c2);
     
    TkErrortendon_stiffness_error(i,:)=(tendon_stiffness-tendon_stiffness_ref)./tendon_stiffness_ref.*100;
    epsilonerror(i,:)=(OptTendonStrain(i,:)-tendon_strain_at_norm_ref)./tendon_strain_at_norm_ref.*100;
 %%       
    Header=OptData.colheaders;
    Ddata=OptData.data;
    time=Ddata(:,1);
    Angle_idx=find(contains(Header,'knee_angle_r/value'));
    Kneeangle=Ddata(:,Angle_idx)*180/3.14;
    activation_idx=find(contains(Header,'/forceset/knee_act'));
    KneeActuator=Ddata(:,activation_idx)*3000;
    % plot(Kneeangle,KneeActuator)
    vq1 = interp1(time,Kneeangle,reftime);
    Error=Kneeref-vq1;
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
legend(Data.Hiplable,'Location','southeast')
% end

%% Tendon Slack Length
figure
X = categorical(Muscname);
bar(X,tendon_slack_Error)
ylabel ('Tendon Slack length Estimation Error(%)')
legend(Data.Hiplable,'Location','northeast')

%% Muscle Stiffness
figure
X = categorical(Muscname(~contains(Muscname,Data.ComplianacMusclename)));
bar(X,muscle_stiffness_error([1,4],:)')
ylabel ('Muscle Stiffness Estimation Error(%)')
legend([Data.Hiplable{1},Data.Hiplable{4}],'Location','southeast')
% legend(Data.Hiplable,'Location','northeast')

%% Tendon stiffness
figure
X2 = categorical(Data.ComplianacMusclename);
bar(X2,TkErrortendon_stiffness_error([1,4],:)')
ylabel ('Tendon Stiffness Estimation Error(%)')
% legend(Data.Hiplable,'Location','northeast')
legend([Data.Hiplable{1},Data.Hiplable{4}],'Location','southeast')


figure
X2 = categorical(Data.ComplianacMusclename);
bar(X2,epsilonerror([1,4],:)')
ylabel ('Epsilon Estimation Error(%)')
% legend(Data.Hiplable,'Location','northeast')
legend([Data.Hiplable{1},Data.Hiplable{4}],'Location','southeast')
