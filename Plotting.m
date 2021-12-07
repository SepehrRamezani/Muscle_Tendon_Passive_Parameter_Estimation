clear all
close all
load ([cd '\SimData.mat']);
path=append(cd,'\Parameterestimation\');
OptParam3=[];
Dir1='C:\MyCloud\GitHub\Muscle_Tendon_Passive_Parameter_Estimation\TorqueSimulation\';
RefData=importdata(append(Dir1,'referenceCoordinates','.sto'));
ref=RefData.data(:,2)*180/3.14;
reftime=RefData.data(:,1);
Muscname=Data.SimMusclename(2:end);
for i=1:length(Data.Hipangle)
    
    u=0;
    OptData=importdata(append(path,['Parameter_Opt_Hip' num2str(Data.Hipangle(i)) '.sto']));
    indx=find(strncmp(OptData.textdata,'num_parameters',14));
    ParNum=erase(OptData.textdata(indx),'num_parameters=');
    ParNum=str2num(ParNum{1,1});
    for m=1:length(Muscname)
        M_indx=find(contains(OptData.colheaders,Muscname(m))&~contains(OptData.colheaders,'force'));
        for y=1:length(M_indx)
            if contains(OptData.colheaders(M_indx(y)),'tendon_slack')
                OptParam1(i,m)=OptData.data(1,M_indx(y));
            elseif contains(OptData.colheaders(M_indx(y)),'passive_fiber_')
                OptParam2(i,m)=OptData.data(1,M_indx(y));
            elseif contains(OptData.colheaders(M_indx(y)),'tendon_strain')
                u=u+1;
                OptParam3(i,u)=OptData.data(1,M_indx(y));
            end
        end
        
    end
    Hiplable(i,1)=append("Hip",num2str(Data.Hipangle(i)));
    TSlack=Data.(Hiplable(i)).TSlack;
    Passive=Data.(Hiplable(i)).Passive;
    Tstrain=Data.(Hiplable(i)).Tstrain;
    Error1(i,:)=(OptParam1(i,:)-TSlack)./TSlack.*100;
    Error2(i,:)=(OptParam2(i,:)-Passive)./Passive.*100;
    Error3(i,:)=(OptParam3(i,:)-Tstrain)./Tstrain.*100;
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
    Error=ref-vq1;
    plot(reftime,Error)
    hold on
end
% xlabel ('Knee Angle(Deg)')
% ylabel ('External Torque(N.m)')
% legend(Name,'Location','southeast')
ylabel ('Error(Deg)')
xlabel ('Time (s)')
title('Knee angle')
% legend(Name,'Location','southeast')
% end
figure
X = categorical(Muscname);
bar(X,Error1)
ylabel ('TS Estimation Error(%)')
legend(Hiplable,'Location','northeast')
figure
X = categorical(Muscname);
bar(X,Error2)
ylabel ('PS Estimation Error(%)')
legend(Hiplable,'Location','northeast')
figure
X2 = categorical(Data.ComplianacMusclename);
bar(X2,Error3)
ylabel ('PS Estimation Error(%)')
legend(Hiplable,'Location','northeast')
